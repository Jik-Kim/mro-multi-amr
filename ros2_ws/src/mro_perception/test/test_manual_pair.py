"""FR-13 명시 페어링 폴백 — pair_state 상태기계와 수동 요청 판정 (계약 10절). ROS 없이 돈다.

실행: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_manual_pair.py -v
"""
from mro_perception.core.reid import manual_pair_decision, PairingGate, PairStateTracker
import numpy as np


# ── PairStateTracker: 언제 GATE_FAIL 로 손을 드나 ──────────────────────────────────────────────────────
def _run(tracker, frames):
    """[(t, visible, paired, ready), ...] 를 순서대로 넣고 마지막 상태를 돌려준다."""
    state = None
    for f in frames:
        state = tracker.update(*f)
    return state


def test_no_target_never_counts():
    """후보가 안 보이면 아무리 오래 지나도 NO_TARGET — 사람이 없는 것은 폴백 대상이 아니다 (10.1)."""
    tr = PairStateTracker(fail_s=30.0)
    assert _run(tr, [(t, False, False, True) for t in range(0, 61, 5)]) == 'NO_TARGET'
    assert tr.fail_elapsed(60) == 0.0


def test_visible_unpaired_becomes_gate_fail_at_fail_s():
    """보이는데 못 넘는 상태가 fail_s 를 채우는 순간 GATE_FAIL. 그 전에는 NO_TARGET 을 유지한다."""
    tr = PairStateTracker(fail_s=30.0)
    assert tr.update(0.0, True, False) == 'NO_TARGET'
    assert tr.update(29.9, True, False) == 'NO_TARGET'
    assert tr.update(30.0, True, False) == 'GATE_FAIL'
    assert tr.update(45.0, True, False) == 'GATE_FAIL'


def test_disappearance_resets_timer():
    """후보가 잠깐 사라지면 시계가 0 으로 돌아가, 다시 보인 뒤 fail_s 를 새로 채워야 한다."""
    tr = PairStateTracker(fail_s=30.0)
    for t in range(0, 21):
        tr.update(float(t), True, False)
    assert tr.update(21.0, False, False) == 'NO_TARGET'
    assert tr.fail_elapsed(21.0) == 0.0
    assert tr.update(22.0, True, False) == 'NO_TARGET'
    assert tr.update(51.9, True, False) == 'NO_TARGET'
    assert tr.update(52.0, True, False) == 'GATE_FAIL'


def test_paired_wins_and_clears_timer():
    """짝이 되면 무조건 PAIRED. 세던 시간도 버린다 — 다시 풀리면 30 s 를 처음부터 센다."""
    tr = PairStateTracker(fail_s=30.0)
    tr.update(0.0, True, False)
    assert tr.update(29.0, True, True) == 'PAIRED'
    assert tr.update(31.0, True, False) == 'NO_TARGET'      # 29 → 31 은 2 s 뿐
    assert tr.update(61.0, True, False) == 'GATE_FAIL'


def test_not_ready_does_not_count():
    """게이트가 준비되기 전(미션·서술자 전)에는 보여도 세지 않는다 — 판정이 없으니 '미달' 이 아니다."""
    tr = PairStateTracker(fail_s=30.0)
    assert _run(tr, [(t, True, False, False) for t in range(0, 61, 10)]) == 'NO_TARGET'


def test_reset_returns_to_start():
    """reset() 뒤에는 NO_TARGET 이고 시계도 0 이다."""
    tr = PairStateTracker(fail_s=30.0)
    tr.update(0.0, True, False)
    tr.update(40.0, True, False)
    tr.reset()
    assert tr.state == 'NO_TARGET' and tr.fail_elapsed(40.0) == 0.0


# ── manual_pair_decision: 계약 10.5 표 ──────────────────────────────────────────────────────────
OK = {'req_ns': '/robot3', 'my_ns': '/robot3', 'req_worker': 'W1', 'my_worker': 'W1',
      'paired': False, 'visible': True, 'distance_m': 0.8, 'max_m': 1.0, 'gate_ok': False}


def dec(**kw):
    return manual_pair_decision(**{**OK, **kw})


def test_accepts_when_all_conditions_hold():
    """내 로봇·미확정·보임·1 m 안·게이트 미통과 → 수락, 사유 없음."""
    assert dec() == (True, '')


def test_wrong_robot_by_namespace():
    """다른 로봇에게 온 요청은 WRONG_ROBOT. 이름의 슬래시 유무는 같은 로봇으로 본다."""
    assert dec(req_ns='/robot8') == (False, 'WRONG_ROBOT')
    assert dec(req_ns='robot3') == (True, '')
    assert dec(req_ns='/robot3/') == (True, '')


def test_other_worker_or_no_mission_is_wrong_robot():
    """진행 중 미션의 작업자와 다른 ID 는 WRONG_ROBOT 급 (10.8). 미션이 없는 로봇도 요청 대상이 아니다."""
    assert dec(req_worker='W2') == (False, 'WRONG_ROBOT')
    assert dec(my_worker='') == (False, 'WRONG_ROBOT')


def test_empty_robot_ns_rejected():
    """robot_ns 가 비면 거부 — 네임스페이스 없이 뜬 노드('/')와 같아지는 구멍을 막는다 (10.3)."""
    assert dec(req_ns='') == (False, 'WRONG_ROBOT')
    assert dec(req_ns='', my_ns='/') == (False, 'WRONG_ROBOT')


def test_empty_worker_id_rejected():
    """worker_id 가 비면 성립하지 않는 요청이다 — 두 필드 모두 필수 (10.3)."""
    assert dec(req_worker='') == (False, 'WRONG_ROBOT')


def test_already_paired():
    """이미 짝이면 거부 — 추종 중 대상 교체는 AC-04 를 흔든다. mission/finish 로 먼저 푼다."""
    assert dec(paired=True) == (False, 'ALREADY_PAIRED')


def test_no_target():
    """최근에 본 후보가 없으면 NO_TARGET."""
    assert dec(visible=False) == (False, 'NO_TARGET')


def test_too_far_or_unknown_distance():
    """1 m 밖이거나 거리를 확인할 수 없으면 TOO_FAR (FR-14). 경계값 1.0 m 는 받는다."""
    assert dec(distance_m=1.2) == (False, 'TOO_FAR')
    assert dec(distance_m=None) == (False, 'TOO_FAR')
    assert dec(distance_m=1.0) == (True, '')


def test_gate_ok_rejects_redundant_request():
    """요청 오는 사이 게이트가 스스로 통과했으면 GATE_OK — 수동이 필요 없다."""
    assert dec(gate_ok=True) == (False, 'GATE_OK')


def test_reason_order_first_match_wins():
    """여러 사유가 겹치면 표 순서대로 먼저 걸리는 하나만 돌려준다."""
    assert dec(req_ns='/robot8', paired=True, visible=False) == (False, 'WRONG_ROBOT')
    assert dec(paired=True, visible=False, distance_m=None) == (False, 'ALREADY_PAIRED')
    assert dec(visible=False, distance_m=None, gate_ok=True) == (False, 'NO_TARGET')


# ── PairingGate: 마지막 판정 노출 · reset ─────────────────────────────────────────────────────────────
def _unit(seed):
    rng = np.random.default_rng(seed)
    h = rng.random(128).astype(np.float32)
    return h / np.linalg.norm(h)


def test_gate_last_ok_tracks_latest_frame():
    """last_ok 는 마지막 프레임 판정 — 통과 직후(paired 전) 수동 요청을 GATE_OK 로 거를 수 있다."""
    g = PairingGate(threshold=0.6, frames=3, on_s=1.0, off_s=3.0)
    mine = _unit(1)
    g.register('worker_a', mine)
    g.update(0.0, [('worker_a', mine, 'box')])
    assert g.last_ok is True and g.paired is False           # on_s 전이라 paired 는 아직
    g.update(0.5, [('worker_b', _unit(2), 'box')])
    assert g.last_ok is False


def test_gate_reset_forget_drops_descriptor():
    """reset(forget=True) 는 서술자까지 지워 ready() 가 False — mission/finish 뒤에는 짝을 만들지 않는다."""
    g = PairingGate(threshold=0.6)
    g.register('worker_a', _unit(1))
    g.paired = True
    g.reset(forget=True)
    assert g.ready() is False and g.paired is False and g.last_ok is False
    g.register('worker_a', _unit(1))
    assert g.ready() is True
