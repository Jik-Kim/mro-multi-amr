"""core/reid.py 단위 테스트 — 재식별 게이트 (BR-T16 · BR-T17). ROS 없이 합성 색 지문으로 돈다.

todo: "core/reid.py 단위 테스트 (같은 인형 ≥ threshold, 다른 인형 < threshold)".
실제 임계값(Q-13)은 TS-U04 프레임으로 잰다 — 여기서는 판정 논리만 검증한다.
"""
import numpy as np
import pytest

from mro_perception.core.reid import PairingGate, gate, hsv_hist, mean_hist, similarity


def unit(seed):
    """L2 노름 1 인 합성 지문 (hsv_hist 출력과 같은 꼴)."""
    v = np.abs(np.random.default_rng(seed).normal(size=128)).astype(np.float32)
    return v / np.linalg.norm(v)


A, B = unit(1), unit(2)                       # 서로 다른 두 인형


def noisy(base, seed):
    """base 에 잡음을 조금 섞은 지문 — 같은 인형을 다른 프레임에서 본 것."""
    return mean_hist([base, unit(seed) * 0.15 + base])


def test_hsv_hist_shape_and_norm():
    img = np.zeros((40, 30, 3), np.uint8)
    img[..., 0] = 120                         # 파란색 한 덩어리
    h = hsv_hist(img)
    assert h.shape == (128,)
    assert abs(float(np.linalg.norm(h)) - 1.0) < 1e-5


def test_similarity_same_vs_different():
    assert similarity(A, A) > 0.999
    assert similarity(A, noisy(A, 7)) >= 0.8              # 같은 인형 ≥ threshold
    assert similarity(A, B) < 0.8                          # 다른 인형 < threshold


def test_mean_hist_keeps_unit_norm():
    m = mean_hist([A, B, noisy(A, 3)])
    assert abs(float(np.linalg.norm(m)) - 1.0) < 1e-5
    assert mean_hist([A]) is A


def test_gate_stage_order():
    assert gate('worker_a', 'worker_b', 0.99, 0.5) == (False, 'outfit mismatch worker_a!=worker_b')
    ok, reason = gate('worker_a', 'worker_a', 0.4, 0.5)
    assert not ok and reason.startswith('sim')
    assert gate('worker_a', 'worker_a', 0.9, 0.5) == (True, 'ok')


@pytest.fixture
def g():
    g = PairingGate(threshold=0.8, frames=5, on_s=1.0, off_s=2.0)
    g.register('worker_a', A)
    return g


def run(g, t, frames, cands_fn):
    """frames 개 프레임을 0.1 s 간격으로 넣고 (마지막 결과, 다음 t) 를 돌려준다."""
    res = None
    for i in range(frames):
        res = g.update(t, cands_fn(i))
        t += 0.1
    return res, t


def test_not_ready_without_descriptor():
    g = PairingGate(0.8)
    assert not g.ready()
    assert g.update(0.0, [('worker_a', A, 'x')]) == (False, None, 0.0, 'no descriptor')


def test_picks_my_worker_even_if_other_has_higher_conf(g):
    # 후보 목록 앞이 확신도 높은 순 — 남(worker_b)이 앞에 있어도 내 담당을 골라야 한다
    (p, tgt, s, r), t = run(g, 0.0, 1, lambda i: [('worker_b', B, 'other'), ('worker_a', noisy(A, i), 'mine')])
    assert tgt == 'mine' and not p                         # 첫 프레임: 골랐지만 아직 paired 아님 (on_s)
    (p, tgt, s, r), t = run(g, t, 11, lambda i: [('worker_b', B, 'other'), ('worker_a', noisy(A, i), 'mine')])
    assert p and tgt == 'mine' and s >= 0.8


def test_hysteresis_on_off(g):
    (p, *_), t = run(g, 0.0, 12, lambda i: [('worker_a', noisy(A, i), 'mine')])
    assert p
    (p, tgt, *_), t = run(g, t, 5, lambda i: [])           # 0.5 s 안 보임 → 유지
    assert p and tgt is None
    (p, *_), t = run(g, t, 20, lambda i: [])               # 2 s 더 → 해제
    assert not p


def test_single_noisy_frame_is_absorbed(g):
    (p, *_), t = run(g, 0.0, 12, lambda i: [('worker_a', noisy(A, i), 'mine')])
    bad = mean_hist([A, unit(99)])                          # 유사도 ~0.7 짜리 한 장
    assert similarity(bad, A) < 0.8
    p, tgt, s, r = g.update(t, [('worker_a', bad, 'mine')])
    assert p and s >= 0.8                                   # 5 프레임 평균이 흡수


def test_wrong_outfit_never_pairs(g):
    (p, tgt, s, r), _ = run(g, 0.0, 15, lambda i: [('worker_b', noisy(A, i), 'wrong_outfit')])
    assert not p and tgt is None and 'outfit mismatch' in r


def test_same_outfit_wrong_color_does_not_pollute_buffer(g):
    (p, tgt, s, r), t = run(g, 0.0, 15, lambda i: [('worker_a', B, 'stranger')])
    assert not p and r.startswith('sim')
    assert len(g.buf) == 0                                  # 실패 프레임은 버퍼에 남지 않는다
    for i in range(12):                                     # 내 담당 재등장 → 1.0 s 안에 재통과 (BR-T17)
        p, tgt, s, r = g.update(t, [('worker_a', noisy(A, i), 'mine')])
        t += 0.1
        if p:
            break
    assert p and tgt == 'mine' and i <= 11


def test_reregister_new_outfit_resets(g):
    run(g, 0.0, 12, lambda i: [('worker_a', noisy(A, i), 'mine')])
    assert g.paired
    g.register('worker_b', B)                               # 다른 사람이 배정됨
    assert not g.paired and len(g.buf) == 0
    g.register('worker_b', noisy(B, 5))                     # 같은 사람 서술자 갱신 → 상태 유지
    assert g.outfit == 'worker_b'
