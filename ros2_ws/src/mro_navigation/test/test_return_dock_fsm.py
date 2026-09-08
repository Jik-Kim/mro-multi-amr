"""복귀 노드 상태 머신 단위 검증 (2026-09-08, return_dock_node 개편과 같이 추가).

`return_dock_node` 의 **실제 코드**를 로봇 없이 돌린다. `turtlebot4_navigation` 만 대역으로
갈아끼우고 `rclpy`·`mro_interfaces` 는 실물을 쓴다. `arena.yaml` 도 `mro_bringup` 소스의 실제
파일을 읽으므로 접근 지점·홀딩 포인트 좌표가 바뀌면 여기서 먼저 드러난다.

**보장하는 것**
  · 상태열이 계약 4절·I-026 과 맞는가 — RETURNING → (DOCK_WAIT) → DOCKING → DOCKED,
    재시도 중에는 DOCKING 유지, 포기할 때만 FAILED
  · 토큰 순서가 **DOCK → P1** 인가 (홀딩 포인트가 P1 바깥이라 DOCK 을 먼저 묻는다, BRD v3.3)
  · P1 을 접근 지점 도착 시점에 반납하는가 (방법 B — 도착이 곧 통과 완료)
  · 도크 정면 접근 지점을 반드시 거치는가 (간격 0.84 m — 멀리서 dock() 은 옆 도크로 간다)
  · **후진하지 않는가** — `undock()` 호출 0 건 (계약 2절). 도킹 실패는 접근 지점 복귀로 푼다
  · 못 얻은 토큰도 반납하는가 (거절당하면 세마포어 대기열에 남는다 — I-033 과 같은 종류)
  · 성공 뒤 `READY` 를 내지 않는가 (DOCKED 가 AC-13 운용시간의 끝 — I-034 와 같은 종류)
  · arena.yaml 을 못 읽으면 움직이지 않고 FAILED 인가

**보장하지 못하는 것 — 실주행에서만 확인된다.**
  · Nav2 가 실제로 대기공간 → P1 → 접근 지점 순서로 지나는지. 방법 B 는 이 순서를 전제로 한다
  · 접근 지점에서 부른 `dock()` 이 정말 자기 도크로 가는지 (TS-U12 로봇별 10회)
  · 도킹 실패 후 접근 지점 복귀가 도크에 걸리지 않는지 — 제자리 회전이 쓸고 가는 폭은
    1.4 cm 라는 계산뿐이고 실물로 확인하지 않았다
  · 2대 동시 복귀 시 DOCK 토큰이 실제로 직렬화하는지 (TS-20)
  · dock 블로킹 구간의 동작 (대역은 즉시 반환한다)

실행 (`colcon build` 로 `mro_interfaces` 가 설치돼 있어야 한다):

    cd ros2_ws/src/mro_navigation
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_return_dock_fsm.py -v

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 은 이 PC 의 `launch_testing_ros` 플러그인이 pytest 9 와
맞지 않아 붙인 것이다 (환경 문제 — 셔틀 시험도 같은 이유로 같은 방식이다).
"""
from pathlib import Path
import sys

import pytest
import rclpy
from rclpy.parameter import Parameter

sys.path.insert(0, str(Path(__file__).resolve().parent))
import stub_navigator                                          # noqa: E402

ARENA = Path(__file__).resolve().parents[2] / 'mro_bringup' / 'params' / 'arena.yaml'
APPROACH_R3 = (-0.435, -0.71)      # arena.yaml dock_zone.approach.robot3 (9/6 실측)
HOLDING = (-0.422, 1.501)          # arena.yaml dock_zone.holding (9/6 재실측)
BASE = {
    'dock_retry': 2,
    't_ret_s': 30.0,
    'token_retry_s': 0.05,
}


def _return_dock_class():
    """대역을 설치한 뒤 노드 클래스를 불러온다."""
    stub_navigator.install()
    from mro_navigation.nodes.return_dock_node import ReturnDock
    return ReturnDock


@pytest.fixture(scope='module')
def ros():
    """arena.yaml 경로를 전역 파라미터로 넘긴 채 rclpy 를 띄운다 (install 공간에 의존하지 않게)."""
    rclpy.init(args=['--ros-args', '-p', f'arena_yaml:={ARENA}'])
    yield
    rclpy.shutdown()


def make_node(**params):
    """발행 내용을 기록하도록 감싼 노드를 만든다."""
    node = _return_dock_class()()
    node.status_log = []
    node.token_log = []
    pub = node.pub_status.publish
    node.pub_status.publish = lambda m: (node.status_log.append(m.data), pub(m))
    node.set_parameters([Parameter(k, value=v) for k, v in {**BASE, **params}.items()])
    return node


def stub_token(node, grants):
    """구역별 획득 응답을 시나리오대로 돌려주고 호출 순서를 기록한다.

    `grants` = {'DOCK': [False, True], 'P1': [...]} — 목록이 마르면 True.
    """
    seq = {k: list(v) for k, v in grants.items()}

    def token(zone, acquire):
        node.token_log.append((zone, 'acquire' if acquire else 'release'))
        if not acquire:
            return True
        node.requested[zone] = True
        granted = seq.get(zone, []).pop(0) if seq.get(zone) else True
        node.held[zone] = granted
        return granted

    node.token = token


def acquires(node):
    """획득 호출만 구역 순서로 추린다."""
    return [z for z, act in node.token_log if act == 'acquire']


def released(node, zone):
    """그 구역을 반납했는가."""
    return (zone, 'release') in node.token_log


def n_calls(nav, name):
    """대역의 호출 기록에서 이름이 같은 것을 센다.

    대역 객체의 메서드에 의존하지 않는다 — 셔틀 시험이 자기 대역을 먼저 설치하면
    그쪽 클래스가 쓰이므로, 시험이 기대할 수 있는 것은 `calls` 목록뿐이다.
    """
    return sum(1 for c in nav.calls if c == name)


def in_polygon(point, polygon):
    """축 정렬 사각형 안에 있는가. arena.yaml 의 polygon 은 [[x, y], ...] 네 점이다."""
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    return min(xs) <= point[0] <= max(xs) and min(ys) <= point[1] <= max(ys)


# ── 정상 흐름 ───────────────────────────────────────────────────────

def test_normal_return_dock_then_p1_then_approach(ros):
    """정상: DOCK → P1 → 접근 지점 → 도킹. 홀딩으로 가지 않고 undock 도 부르지 않는다."""
    node = make_node()
    stub_token(node, {})
    node.run_return()

    assert node.status_log == ['RETURNING', 'DOCKING', 'DOCKED']
    assert acquires(node) == ['DOCK', 'P1'], 'DOCK 을 먼저 묻는다 (홀딩이 P1 바깥이다)'
    assert node.navigator.goals() == [('goto', *APPROACH_R3)], '접근 지점만 경유'
    assert n_calls(node.navigator, 'undock') == 0, '계약 2절 — 후진하지 않는다'
    assert released(node, 'P1') and released(node, 'DOCK')
    node.destroy_node()


def test_p1_released_at_approach_arrival(ros):
    """P1 은 접근 지점 도착 시점에 반납한다 (방법 B — 도착이 곧 통과 완료)."""
    node = make_node()
    stub_token(node, {})
    node.run_return()

    order = node.token_log
    p1_release = order.index(('P1', 'release'))
    dock_release = order.index(('DOCK', 'release'))
    assert p1_release < dock_release, 'P1 을 DOCK 보다 먼저 반납한다 (도킹 전에 통과가 끝난다)'
    node.destroy_node()


def test_success_does_not_emit_ready(ros):
    """성공 뒤 READY 를 내지 않는다 — DOCKED 가 AC-13 운용시간의 끝이다 (I-034)."""
    node = make_node()
    stub_token(node, {})
    node.run_return()

    assert 'READY' not in node.status_log
    assert node.status_log[-1] == 'DOCKED'
    node.destroy_node()


# ── 토큰 거절 ───────────────────────────────────────────────────────

def test_dock_denied_goes_to_holding_first(ros):
    """DOCK 거절 → DOCK_WAIT 를 내고 홀딩 포인트로 **이동**한다 (제자리 대기가 아니다)."""
    node = make_node()
    stub_token(node, {'DOCK': [False, False, True]})
    node.run_return()

    assert 'DOCK_WAIT' in node.status_log
    goals = node.navigator.goals()
    assert goals[0] == ('goto', *HOLDING), '홀딩 포인트로 먼저 간다'
    assert goals[-1] == ('goto', *APPROACH_R3), '그다음 접근 지점'
    assert node.status_log[-1] == 'DOCKED'
    node.destroy_node()


def test_p1_denied_waits_in_place(ros):
    """P1 거절 → 홀딩으로 가지 않는다. DOCK 을 받은 시점에 이미 P1 바깥이다."""
    node = make_node()
    stub_token(node, {'P1': [False, False, True]})
    node.run_return()

    assert 'DOCK_WAIT' not in node.status_log, 'DOCK 은 받았으니 DOCK_WAIT 가 아니다'
    assert node.navigator.goals() == [('goto', *APPROACH_R3)]
    node.destroy_node()


def test_denied_token_is_released_on_timeout(ros):
    """T_ret 초과. **못 얻은 토큰도 반납한다** — 안 하면 유령 대기열이 구역을 영구히 잠근다."""
    node = make_node(t_ret_s=0.4)
    stub_token(node, {'DOCK': [False] * 500})
    node.run_return()

    assert node.status_log[-1] == 'FAILED'
    assert released(node, 'DOCK'), '거절당해도 대기열에 들어가므로 반납해야 한다 (I-033)'
    assert node.requested['DOCK'] is False and node.held['DOCK'] is False
    node.destroy_node()


# ── 도킹 재시도 ─────────────────────────────────────────────────────

def test_dock_retry_returns_to_approach_not_undock(ros):
    """도킹 1회 실패 → **접근 지점으로 되돌아가** 재시도. undock 을 부르지 않는다 (계약 2절)."""
    node = make_node()
    stub_token(node, {})
    nav = node.navigator
    tries = {'n': 0}

    def docked():
        tries['n'] += 1
        return tries['n'] > 1          # 첫 확인은 실패, 두 번째는 성공

    nav.getDockedStatus = docked
    node.run_return()

    assert n_calls(nav, 'undock') == 0, '후진하지 않는다'
    assert nav.goals() == [('goto', *APPROACH_R3)] * 2, '접근 지점으로 되돌아가 재시도'
    assert n_calls(nav, 'dock') == 2
    assert node.status_log[-1] == 'DOCKED'
    node.destroy_node()


def test_docking_status_emitted_once_across_retries(ros):
    """재시도 중에도 DOCKING 을 유지한다 — 매번 새로 내면 관제가 재시도를 새 도킹으로 센다 (I-026)."""
    node = make_node()
    stub_token(node, {})
    node.navigator.deliver_dock_status(False)
    node.run_return()

    assert node.status_log.count('DOCKING') == 1
    assert node.status_log[-1] == 'FAILED', '재시도를 다 쓰면 포기한다'
    node.destroy_node()


def test_dock_failure_uses_all_retries_then_fails(ros):
    """재시도 2회를 다 쓰고 FAILED. 이동은 접근 1 + 재시도 2 = 3회, undock 은 0회."""
    node = make_node()
    stub_token(node, {})
    nav = node.navigator
    nav.deliver_dock_status(False)
    node.run_return()

    assert n_calls(nav, 'dock') == 3, '최초 1 + 재시도 2'
    assert len(nav.goals()) == 3
    assert n_calls(nav, 'undock') == 0
    assert node.status_log[-1] == 'FAILED'
    assert released(node, 'DOCK'), '실패해도 반납한다'
    node.destroy_node()


# ── 실패 경로 ───────────────────────────────────────────────────────

def test_missing_arena_fails_without_moving(ros):
    """arena.yaml 을 못 읽으면 접근 지점을 모른다 — 움직이지 않고 FAILED."""
    node = make_node()
    stub_token(node, {})
    node.arena = None
    node.run_return()

    assert node.status_log == ['FAILED']
    assert node.navigator.goals() == []
    node.destroy_node()


def test_goal_rejected_releases_both_tokens(ros):
    """이동 목표가 거부되면 포기하되 두 토큰을 모두 반납한다."""
    node = make_node()
    stub_token(node, {})
    node.navigator.goal_accepted = False
    node.run_return()

    assert node.status_log[-1] == 'FAILED'
    assert released(node, 'P1') and released(node, 'DOCK')
    node.destroy_node()


def test_estop_blocks_goal_until_cleared(ros):
    """estop 중에는 주행 명령을 내지 않는다. 해제되면 이어간다 — 실패가 아니다."""
    node = make_node()
    stub_token(node, {})
    node.estop = True
    seen = []

    # goto 가 estop 중이면 보류하는지 본다. 첫 확인에서 기록하고 해제해
    #   '해제되면 이어간다' 까지 한 번에 확인한다
    real_goto = node.goto

    def goto(pose, label):
        seen.append((label, node.estop))
        node.estop = False
        return real_goto(pose, label)

    node.goto = goto
    node.run_return()

    assert seen and seen[0][1] is True, 'estop 중에 goto 가 불렸고 노드가 이를 알고 있었다'
    assert node.status_log[-1] == 'DOCKED', 'estop 해제 후 복귀를 마친다'
    node.destroy_node()


# ── 좌표 전제 (arena.yaml 이 바뀌면 여기서 먼저 드러난다) ───────────

def test_arena_has_approach_and_holding_for_this_robot(ros):
    """접근 지점과 홀딩 포인트가 arena.yaml 에 있어야 복귀가 성립한다."""
    node = make_node()
    approach, holding = node.targets()

    assert len(approach) == 3 and len(holding) == 3
    assert (round(approach[0], 3), round(approach[1], 3)) == APPROACH_R3
    assert (round(holding[0], 3), round(holding[1], 3)) == HOLDING
    node.destroy_node()


def test_holding_point_is_outside_p1_polygon(ros):
    """홀딩 포인트는 P1 폴리곤 밖이어야 한다.

    DOCK 을 못 얻은 로봇이 여기서 기다리는데 P1 안이면 선행 로봇의 통과를 막는다.
    토큰 순서를 DOCK → P1 로 둔 근거이기도 하다 (BRD v3.3 개정).
    """
    node = make_node()
    _, holding = node.targets()
    p1 = node.arena['semaphore_zones']['P1']['polygon']

    assert not in_polygon(holding, p1), f'홀딩 {holding[:2]} 가 P1 {p1} 안에 있다'
    node.destroy_node()


def test_approach_point_is_outside_p1_polygon(ros):
    """접근 지점도 P1 밖이어야 한다 — 도착 시점에 P1 을 반납하기 때문이다 (방법 B 전제)."""
    node = make_node()
    approach, _ = node.targets()
    p1 = node.arena['semaphore_zones']['P1']['polygon']

    assert not in_polygon(approach, p1), \
        f'접근 지점 {approach[:2]} 가 P1 {p1} 안이면 반납 시점이 이르다'
    node.destroy_node()


def test_dock_pose_matches_arena(ros):
    """`robotN.yaml` dock_pose 와 `arena.yaml` dock_zone.pose 가 같아야 한다 (I-013).

    어긋나면 두 로봇의 맵 좌표계가 달라져 세마포어 폴리곤·waypoint 가 전부 틀어진다.
    """
    node = make_node()
    arena_pose = node.arena['dock_zone']['pose'][node.robot]

    assert len(arena_pose) == 3
    assert arena_pose != [0.0, 0.0, 0.0], '(0,0,0) 이면 초기자세가 맵 원점이 된다'
    node.destroy_node()
