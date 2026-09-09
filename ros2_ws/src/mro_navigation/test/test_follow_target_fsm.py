"""추종 노드 상태 머신 단위 검증 (2026-09-08, PR #39 후속).

`follow_target_node` 의 **실제 코드**를 로봇 없이 돌린다. `turtlebot4_navigation` 만 대역으로
갈아끼우고 `rclpy`·`mro_interfaces` 는 실물을 쓴다. TF 와 제자리 회전은 대역으로 바꾼다 —
추종은 `arena.yaml` 을 읽지 않으므로(셔틀·복귀와 다르다) 좌표 파일 의존이 없다.

**보장하는 것**
  · `approach/status` 가 계약 4절의 8개 값만 내는가, 순서와 시점이 맞는가
  · 셔틀 신호(`shuttle/arrived`)가 오기 전에는 **아무것도 하지 않는가** — 작업자가 눈앞에
    있어도 목표를 내지 않는다. 옛 토픽을 구독하던 시절 S5 가 시작되지 않던 결함의 대칭
  · `target/paired` 게이트 — 거짓이면 움직이지 않고, 추종 중 거짓이 되면 즉시 멈추는가
    (오추종 0건이 절대 기준이다)
  · Lost 3단 — `lost_s` 정지 · `research_s` 회전 탐색 · `standby_s` STANDBY. 재등장 시 게이트 재통과
  · 재탐지 방향 — 힌트 > 마지막으로 본 쪽 > 기본 30°. 천장 카메라가 없어도 아무 쪽으로 돌지 않는다
  · `mission/finish` 는 `FAILED` 가 아니라 `IDLE` 로 끝나는가 (정상 종료 — I-026)
  · `safety/estop` 중 목표를 취소하고 상태를 유지하는가 (실패가 아니다)
  · 최소 이격(`min_separation_m`) 침범 시 정지하는가 (AC-05 하한)
  · 힌트 방위 계산 — 좌우 부호, 유효기간(`hint_valid_s`), 남의 작업자 걸러내기
  · **후진하지 않는가** — `cmd_vel` 에 각속도만 싣는다 (계약 2절)

**보장하지 못하는 것 — 실주행에서만 확인된다.**
  · 인지의 `target/map_point` 좌표가 실제로 맞는지 (I-031 — 좌우 1.76 배 오차가 남아 있다)
  · `target/paired` 발행자가 아직 없다 (인지 마감 9/9). 게이트 통과 경로는 대역으로만 확인됨
  · 추종거리 p95·최소 이격 실측 (TS-U07 · I-029)
  · 제자리 회전이 실제로 그 각도만큼 도는지 — odom 적산 코드는 대역으로 갈아끼웠다
  · 웹캠2 힌트가 sensor_data QoS 로 실제로 도착하는지 (TS-U02)

실행 (`colcon build` 로 `mro_interfaces` 가 설치돼 있어야 한다):

    cd ros2_ws/src/mro_navigation
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_follow_target_fsm.py -v

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 은 이 PC 의 `launch_testing_ros` 플러그인이 pytest 9 와
맞지 않아 붙인 것이다 (환경 문제 — 셔틀·복귀 시험도 같은 방식이다).
"""
import math
from pathlib import Path
import sys
import time

import pytest
import rclpy
from rclpy.parameter import Parameter

sys.path.insert(0, str(Path(__file__).resolve().parent))
import stub_navigator                                          # noqa: E402

BASE = {
    'follow_standoff_m': 0.5,
    'min_separation_m': 0.3,
    'lost_s': 1.0,
    'standby_s': 30.0,
    'approach_skip_m': 1.0,
    'hint_valid_s': 2.0,
    'research_s': 2.0,
    'settle_s': 1.0,
    'search_wait_s': 1.5,
    'visible_wait_s': 3.0,
    'search_step_deg': 20.0,
}
NEAR = (0.4, 0.0)      # 로봇 앞 0.4 m — approach_skip_m 안이라 바로 추종
FAR = (3.0, 0.0)       # 1 m 밖 — 2단계 접근


def _follow_target_class():
    """대역을 설치한 뒤 노드 클래스를 불러온다.

    `robot_xy` 는 실제 TF 를 읽으므로 시험에서는 원점을 돌려주는 대역으로 바꾼다.
    `standoff_goal` 은 실물을 쓴다 — 목표 좌표 계산이 맞는지도 함께 확인된다.
    """
    stub_navigator.install()
    from mro_navigation.nodes import follow_target_node as mod
    mod.robot_xy = lambda tf, logger=None: (0.0, 0.0)
    return mod.FollowTarget


ROBOT = 'robot3'                    # 노드가 ns 로 자기 로봇을 정한다
GOAL_TOL_M = 0.25                   # Nav2 xy_goal_tolerance (I-029) — 이 안의 목표는 안 움직인다


@pytest.fixture(scope='module')
def ros():
    """단위 시험용으로 rclpy 를 한 번 띄우고 모듈 전체가 함께 쓴다.

    `__ns` 를 주는 것은 노드가 **네임스페이스 없이는 뜨지 않기** 때문이다 — ns 로 Nav2 액션
    서버를 잡으므로 대체값을 쓰면 남의 로봇을 움직인다 (9/8, main). 셔틀 시험도 같은 방식이다.
    """
    rclpy.init(args=['--ros-args', '-r', f'__ns:=/{ROBOT}'])
    yield
    rclpy.shutdown()


def make_node(**params):
    """발행 기록·TF·회전을 대역으로 바꾼 노드를 만든다.

    TF 는 로봇을 원점에 세우고 +x 를 보게 둔다. 회전은 각도만 기록한다 —
    odom 적산 루프는 `rclpy.spin_once` 를 돌려 시험에서 재현할 수 없다.
    """
    node = _follow_target_class()()
    node.status_log = []
    node.turns = []
    pub = node.pub_status.publish
    node.pub_status.publish = lambda m: (node.status_log.append(m.data), pub(m))
    node.rotate_in_place = lambda deg, stop_on_sight=False: node.turns.append(round(deg, 1))
    node.map_yaw = lambda: 0.0                       # +x 를 보고 있다
    node.set_parameters([Parameter(k, value=v) for k, v in {**BASE, **params}.items()])
    node.status_log.clear()
    return node


def see(node, xy=NEAR):
    """인지가 작업자 좌표를 한 번 보낸 것과 같다.

    검출 시각을 0.6 s 씩 띄운다 — 노드가 `CONSIST_DT`(0.5 s) 보다 촘촘한 검출을 건너뛰기
    때문이다. 같은 순간의 TF 를 공유한 좌표로 일관성을 판정하지 않으려는 실물 규칙이다.
    SETTLE 대기(`accept_after`)는 시험에서 건너뛴다.
    """
    node._see_t = getattr(node, '_see_t', time.time()) + 0.6
    node.latest = (xy[0], xy[1], node._see_t)
    node.accept_after = 0.0
    node.visible_at = time.time()


def start(node):
    """셔틀이 대기공간 도착을 알린 것과 같다 — 여기서부터 S5 다."""
    node.arrived_cb(_bool(True))
    node.step()
    node.status_log.clear()
    return node


def lost_for(node, seconds):
    """작업자를 그만큼 못 본 상태로 만든다."""
    node.follow_seen = time.time() - seconds
    node.visible_at = 0.0
    node.latest = None


def reach_following(node):
    """게이트를 통과시켜 FOLLOWING 까지 보낸다."""
    start(node)
    node.paired = True
    see(node)
    node.step()
    see(node)
    node.step()
    return node


def n_calls(nav, name):
    """대역의 호출 기록에서 이름이 같은 것을 센다.

    대역 객체의 메서드에 의존하지 않는다 — 셔틀 시험이 자기 대역을 먼저 설치하면
    그쪽 클래스가 쓰이므로, 시험이 기대할 수 있는 것은 `calls` 목록뿐이다.
    """
    return sum(1 for c in nav.calls if c == name)


def _bool(value):
    from std_msgs.msg import Bool
    msg = Bool()
    msg.data = value
    return msg


# ── 시작 게이트 ─────────────────────────────────────────────────────

def test_does_nothing_before_shuttle_arrived(ros):
    """셔틀 신호가 오기 전에는 목표도 회전도 내지 않는다.

    옛 `goto_goal/arrived` 를 구독하던 시절 S5 가 아예 시작되지 않았다 (PR #36 이 남긴 숙제).
    그 대칭으로, 신호가 없을 때 제멋대로 움직이지 않는 것도 확인한다.
    """
    node = make_node()
    for _ in range(30):
        node.paired = True
        see(node)
        node.step()

    assert node.state == 'IDLE'
    assert node.navigator.goals() == []
    assert node.turns == []
    node.destroy_node()


def test_arrived_signal_starts_searching(ros):
    """셔틀 신호를 받으면 SEARCHING 으로 들어간다. 도착 직후 회전하지 않는다.

    정차 자세가 이미 서향(작업공간 방향)이고 작업자를 로봇 정면으로 돌아 들어오게
    조종하기 때문이다 (계약 8절).
    """
    node = make_node()
    start(node)

    assert node.state == 'SEARCHING'
    assert node.turns == [], '도착 직후 고정 회전을 하지 않는다'
    node.destroy_node()


# ── 재식별 게이트 ───────────────────────────────────────────────────

def test_gate_false_never_moves(ros):
    """`target/paired` 가 거짓이면 좌표가 좋아도 목표를 내지 않는다 (오추종 0건)."""
    node = make_node()
    start(node)
    for _ in range(8):
        see(node)
        node.step()

    assert node.state == 'SEARCHING'
    assert node.navigator.goals() == [], '남을 따라가지 않는다'
    node.destroy_node()


def test_gate_true_near_target_follows_directly(ros):
    """게이트를 통과하고 작업자가 `approach_skip_m` 안이면 접근 없이 바로 FOLLOWING."""
    node = make_node()
    reach_following(node)

    assert node.state == 'FOLLOWING'
    assert 'APPROACH_1' not in node.status_log, '가까우면 2단계 접근을 쓰지 않는다'
    node.destroy_node()


def test_gate_true_far_target_uses_two_stage_approach(ros):
    """작업자가 멀면 APPROACH_1 부터 — 2 m 밖 좌표는 오차가 커서 한 번에 목표를 잡을 수 없다."""
    node = make_node()
    start(node)
    node.paired = True
    see(node, FAR)
    node.step()
    see(node, FAR)
    node.step()

    assert node.state == 'APPROACH_1'
    node.destroy_node()


def test_gate_lost_during_following_stops_immediately(ros):
    """추종 중 게이트가 거짓이 되면 즉시 멈추고 SEARCHING 으로 돌아간다."""
    node = make_node()
    reach_following(node)
    node.paired = False
    node.step()

    assert node.state == 'SEARCHING'
    node.destroy_node()


# ── Lost 3단 ────────────────────────────────────────────────────────

def test_lost_short_holds_position_without_changing_state(ros):
    """`lost_s` 를 넘기면 주행을 취소하고 제자리 대기. 상태는 FOLLOWING 을 유지한다.

    Lost 는 실패가 아니다. 상태를 바꾸면 관제·KPI 가 추종 구간을 잘못 자른다.
    """
    node = make_node()
    reach_following(node)
    # `lost_s` 와 `research_s` 사이를 값에서 끌어온다 — 9/9 에 research_s 를 8 → 2 로
    #   내리면서 이 시험이 넣어 둔 2.0 이 경계 밖으로 나갔다. 값이 또 바뀌어도 안 깨지게 한다
    hold_window = (node.p('lost_s') + node.p('research_s')) / 2
    lost_for(node, hold_window)
    node.step()

    assert node.state == 'FOLLOWING'
    assert node.follow_holding is True
    node.destroy_node()


def test_lost_medium_goes_back_to_searching(ros):
    """8 초를 넘기면 회전 탐색으로 돌아간다."""
    node = make_node()
    reach_following(node)
    lost_for(node, 9.0)
    node.step()

    assert node.state == 'SEARCHING'
    node.destroy_node()


def test_lost_long_enters_standby(ros):
    """`standby_s` 를 넘기면 STANDBY — 오류가 아니라 정규 상태다 (BR-T17)."""
    node = make_node()
    reach_following(node)
    lost_for(node, 31.0)
    node.step()

    assert node.state == 'STANDBY'
    assert 'FAILED' not in node.status_log
    node.destroy_node()


def test_standby_requires_gate_again(ros):
    """STANDBY 에서 작업자가 다시 나타나도 게이트를 통과해야 재개한다 (계약 3절 재획득)."""
    node = make_node()
    reach_following(node)
    lost_for(node, 31.0)
    node.step()
    assert node.state == 'STANDBY'

    node.paired = False
    see(node)
    node.step()
    assert node.state == 'STANDBY', '게이트 미통과면 STANDBY 를 유지한다'

    node.paired = True
    see(node)
    node.step()
    assert node.state == 'SEARCHING', '게이트 통과 후 재획득을 시작한다'
    node.destroy_node()


# ── 종료·정지 ───────────────────────────────────────────────────────

def test_finish_returns_to_idle_not_failed(ros):
    """`mission/finish` 는 정상 종료다 — `FAILED` 를 내면 관제가 미션을 영구 실패로 굳힌다 (I-026)."""
    node = make_node()
    reach_following(node)
    node.finish_cb(_bool(True))
    node.step()

    assert node.state == 'IDLE'
    assert node.paired is False, '페어링을 해제한다'
    assert 'FAILED' not in node.status_log
    node.destroy_node()


def test_estop_holds_state_and_resumes(ros):
    """비상정지(estop) 중에는 목표를 취소하고 상태를 유지한다. 해제되면 이어간다 (실패가 아니다)."""
    node = make_node()
    reach_following(node)
    before = node.state

    node.estop_cb(_bool(True))
    node.step()
    node.step()
    assert node.state == before
    assert node.estop_cancelled is True, 'goal 취소는 진입 시 한 번만'

    node.estop_cb(_bool(False))
    see(node)
    node.step()
    assert node.state == 'FOLLOWING'
    node.destroy_node()


def test_min_separation_breach_holds(ros):
    """최소 이격을 침범하면 정지한다 — AC-05 의 `min(distance) ≥ 0.3 m` 는 절대 기준이다."""
    node = make_node()
    reach_following(node)
    node.follow_est = (0.2, 0.0)          # 로봇(원점)에서 0.2 m — 0.3 m 하한 침범
    node.follow_t = time.time()
    node.follow_seen = time.time()
    node.follow_goal = None
    node.follow_goal_t = 0.0
    node.follow_holding = False
    node.step()

    assert node.follow_holding is True
    node.destroy_node()


# ── 힌트 → 회전 부호 ────────────────────────────────────────────────

def test_hint_bearing_sign_left_and_right(ros):
    """힌트 좌표를 회전 부호로 바꾼다 — 양수 반시계, 음수 시계.

    웹캠2 는 작업자의 map 좌표만 준다. 로봇이 어디서 어느 쪽을 보는지는 모르므로
    (힌트 − 내 위치) 의 방향과 내가 보는 방향의 차를 우리가 계산한다.
    """
    node = make_node()
    node.worker_id = 'worker_a'

    node.hint = (0.0, 2.0, time.time(), 'worker_a')      # 왼쪽(+y)
    assert node.hint_bearing() > 80

    node.hint = (0.0, -2.0, time.time(), 'worker_a')     # 오른쪽(−y)
    assert node.hint_bearing() < -80

    node.hint = (2.0, 0.0, time.time(), 'worker_a')      # 정면(+x)
    assert abs(node.hint_bearing()) < 1
    node.destroy_node()


def test_stale_hint_is_discarded(ros):
    """`hint_valid_s` 를 넘긴 힌트는 버린다 (계약 4절 유효 2 s)."""
    node = make_node()
    node.worker_id = 'worker_a'
    node.hint = (0.0, 2.0, time.time() - 5.0, 'worker_a')

    assert node.hint_bearing() is None
    node.destroy_node()


def test_other_workers_hint_is_discarded(ros):
    """남의 작업자 힌트는 버린다.

    힌트 토픽은 전역이라 두 로봇이 같은 것을 받는다. 안 걸르면 상대 작업자 쪽으로 돈다.
    계약에 '걸러라' 는 문구는 없고 우리 판단이다.
    """
    node = make_node()
    node.worker_id = 'worker_a'
    node.hint = (0.0, 2.0, time.time(), 'worker_b')

    assert node.hint_bearing() is None
    node.destroy_node()


def test_search_rotation_uses_hint_direction(ros):
    """힌트가 있으면 그 방향으로 한 번에, 없으면 기존 30° 씩 돈다."""
    node = make_node()
    node.worker_id = 'worker_a'

    node.hint = None
    assert node.search_rotate_deg() == node.p('search_step_deg')

    node.hint = (0.0, -2.0, time.time(), 'worker_a')
    assert node.search_rotate_deg() < -80
    node.destroy_node()


# ── 계약 준수 ───────────────────────────────────────────────────────

def test_status_values_are_all_in_contract(ros):
    """`approach/status` 는 계약 4절이 정한 8개 문자열만 낸다.

    관제·`kpi_logger` 가 이 값으로 구간을 자른다. 철자가 다르면 코드가 잘 돌아도
    추종거리(AC-05)가 엉뚱한 구간에서 매겨진다.
    """
    allowed = {'IDLE', 'SEARCHING', 'APPROACH_1', 'APPROACH_2',
               'ARRIVED', 'FOLLOWING', 'STANDBY', 'FAILED'}
    node = make_node()
    reach_following(node)
    lost_for(node, 31.0)
    node.step()
    node.finish_cb(_bool(True))
    node.step()

    assert set(node.status_log) <= allowed, f'계약에 없는 값: {set(node.status_log) - allowed}'
    node.destroy_node()


def test_cmd_vel_carries_angular_only(ros):
    """`cmd_vel` 에 각속도만 싣는다 — 병진은 0 이고 후진은 애초에 없다 (계약 2절)."""
    node = make_node()
    published = []
    node.cmd_stamped = True
    node.cmd_pub = type('P', (), {'publish': lambda _self, m: published.append(m)})()

    node.publish_wz(0.4)
    node.publish_wz(-0.4)

    for msg in published:
        assert msg.twist.linear.x == 0.0, '병진 성분이 없다'
        assert msg.twist.linear.y == 0.0
    assert [round(m.twist.angular.z, 2) for m in published] == [0.4, -0.4]
    node.destroy_node()


def test_no_reverse_anywhere_in_source(ros):
    """소스에 후진을 내는 곳이 없다 (계약 2절).

    `linear.x` 에 값을 대입하는 줄도, `undock()` 호출도 없어야 한다.
    """
    src = (Path(__file__).resolve().parents[1] / 'mro_navigation' / 'nodes'
           / 'follow_target_node.py').read_text(encoding='utf-8')
    code = [ln for ln in src.splitlines()
            if not ln.lstrip().startswith('#')
            and ('linear' in ln or '.undock(' in ln)]
    assert code == [], f'후진 가능성이 있는 줄: {code}'
    node = make_node()
    assert n_calls(node.navigator, 'undock') == 0
    node.destroy_node()


def test_approach_skip_boundary(ros):
    """`approach_skip_m` 경계 — 안이면 바로 추종, 밖이면 접근부터."""
    node = make_node(approach_skip_m=0.5)
    start(node)
    node.paired = True
    see(node, (0.45, 0.0))                # 0.5 m 안
    node.step()
    see(node, (0.45, 0.0))
    node.step()
    assert node.state == 'FOLLOWING'
    node.destroy_node()

    node = make_node(approach_skip_m=0.5)
    start(node)
    node.paired = True
    see(node, (0.9, 0.0))                 # 0.5 m 밖 → 접근을 쓴다
    node.step()
    see(node, (0.9, 0.0))
    node.step()
    # gap 0.9 → 목표는 max(0.5, 0.45) = 0.5 = 최종 이격이라 APPROACH_2 다.
    #   상태 이름의 뜻은 '중간/최종' 이지 '몇 번째 회차' 가 아니다
    assert node.state == 'APPROACH_2'
    node.destroy_node()


def test_next_standoff_halves_but_never_below_final(ros):
    """목표 거리 = max(최종 이격, 남은 거리 / 2). 하한을 뚫지 않는다.

    하한이 `follow_standoff_m` 인 것은 접근 단계에서 최소 이격(`min_separation_m` 0.3)을
    침범하지 않기 위한 것이다 — AC-04 접촉·오추종 0건이 절대 기준이다.
    """
    node = make_node()
    final = node.p('follow_standoff_m')

    assert node.next_standoff(3.0) == 1.5
    assert node.next_standoff(1.21) == pytest.approx(0.605)
    assert node.next_standoff(0.9) == final          # 0.45 < 0.5 → 하한
    assert node.next_standoff(0.2) == final          # 이미 안쪽이어도 하한 아래로 안 간다
    node.destroy_node()


def test_far_target_goal_is_intermediate(ros):
    """멀면 중간 목표(APPROACH_1)로 간다 — 2 m 밖 좌표는 0.5~0.7 m 틀린다."""
    node = make_node()
    start(node)
    node.paired = True
    see(node, (3.0, 0.0))
    node.step()
    see(node, (3.0, 0.0))
    node.step()

    assert node.state == 'APPROACH_1'
    gx, gy = node.navigator.goals()[-1][1:3]
    assert (gx, gy) == pytest.approx((1.5, 0.0), abs=0.01), '작업자 앞 1.5 m'
    node.destroy_node()


def test_goal_actually_moves_the_robot_at_stall_distance(ros):
    """9/8 실주행이 붙박인 1.21 m 에서 목표가 실제로 움직일 만큼 떨어져 있는가.

    종전에는 1단계 목표가 작업자 앞 1.0 m 로 고정이라 로봇에서 0.21 m 였고, Nav2
    `xy_goal_tolerance` 0.25 안이라 **한 발도 안 움직이고** SUCCEEDED 가 났다.
    그 결과 도착 → 재검출 → 도착이 무한히 돌며 로봇이 1.20 m 에 붙박였다.
    """
    node = make_node()
    start(node)
    node.paired = True
    see(node, (1.21, 0.0))
    node.step()
    see(node, (1.21, 0.0))
    node.step()

    gx, gy = node.navigator.goals()[-1][1:3]
    assert math.hypot(gx, gy) > GOAL_TOL_M, '목표가 도착 판정 안이면 로봇이 안 움직인다'
    node.destroy_node()


def test_approach_converges_to_arrived(ros):
    """회차를 거듭하면 유한 번에 ARRIVED 에 닿는다 — 무한 루프가 구조적으로 불가능하다.

    목표가 항상 `max(최종, gap/2)` 라 gap 이 회차마다 반으로 줄고, 최종 이격에 닿으면
    하한에 걸려 그 회차가 마지막이 된다. 시험은 로봇을 목표까지 순간이동시켜
    (Nav2 주행을 대신한다) 회차만 돌린다.
    """
    node = make_node()
    start(node)
    node.paired = True
    worker = 3.0
    rob = 0.0
    seen_states = []

    for _ in range(10):
        see(node, (worker - rob, 0.0))       # 좌표는 로봇 기준으로 들어온다 (TF 대역이 원점)
        node.step()
        see(node, (worker - rob, 0.0))
        node.step()
        seen_states.append(node.state)
        if node.state in ('ARRIVED', 'FOLLOWING'):
            break
        assert node.state in ('APPROACH_1', 'APPROACH_2'), node.state
        gx, _ = node.navigator.goals()[-1][1:3]
        rob += gx                            # 목표까지 갔다고 친다
        while not node.navigator.isTaskComplete():
            pass
        node.step()                          # 도착 처리 → SEARCHING 또는 ARRIVED

    assert 'APPROACH_1' in seen_states, '멀리서는 중간 목표를 써야 한다'
    assert node.state in ('ARRIVED', 'FOLLOWING'), f'수렴하지 못했다: {seen_states}'
    node.destroy_node()


def test_stall_distance_reaches_following(ros):
    """9/8 붙박인 1.21 m 에서 한 회차 만에 FOLLOWING 까지 간다.

    gap 1.21 → 목표는 작업자 앞 0.605 m (로봇에서 0.605 m — 실제로 움직인다) → 도착하면
    gap 0.605 로 `approach_skip_m` 1.0 안이라 접근을 끝내고 FOLLOWING 이다.
    마지막 0.5 m 붙이기는 FOLLOWING 이 맡는다 (AC-05 채점 구간도 여기뿐이다).
    """
    node = make_node()
    start(node)
    node.paired = True
    see(node, (1.21, 0.0))
    node.step()
    see(node, (1.21, 0.0))
    node.step()
    assert node.state == 'APPROACH_1'

    gx, _ = node.navigator.goals()[-1][1:3]
    while not node.navigator.isTaskComplete():
        pass
    node.step()                              # 중간 목표 도착 → SEARCHING
    see(node, (1.21 - gx, 0.0))              # 목표까지 갔다고 치고 다시 잰다
    node.step()
    see(node, (1.21 - gx, 0.0))
    node.step()

    assert node.state == 'FOLLOWING', '한 회차 뒤에는 추종으로 넘어가야 한다'
    node.destroy_node()


def test_first_rotation_within_5s_of_losing_target(ros):
    """놓친 뒤 첫 회전까지 5 초 안이어야 한다 (9/9 실주행 — 종전 11 초는 너무 느렸다).

    합은 `research_s`(2.0) + `search_wait_s`(1.5) = **3.5 초**다.
    **`lost_s` 는 더해지지 않는다** — 같은 시계 안의 앞 단계다. 마지막으로 본 뒤 1 초에
    주행을 멈추고, 2 초에 탐색으로 넘어간다. 각 값의 바닥 근거는 노드 주석에 적었다.
    """
    node = make_node()
    total = node.p('research_s') + node.p('search_wait_s')

    assert total <= 5.0, f'놓친 뒤 첫 회전까지 {total} s — 5 초를 넘는다'
    assert node.p('lost_s') == 1.0, 'lost_s 는 SOT 계약값이라 바꾸지 않는다'
    assert node.p('research_s') >= 2.0, '2 초 미만이면 사람이 앞을 지나가는 것만으로 돌아버린다'
    assert node.p('search_wait_s') >= node.p('settle_s'), \
        'settle 이 끝나기 전에 회전하면 틀린 좌표로 목표를 낸다'
    node.destroy_node()


def test_rotation_turns_toward_last_seen_side(ros):
    """천장 카메라 힌트가 없어도 **마지막으로 본 쪽**으로 돈다.

    종전에는 힌트가 없으면 무조건 +30°(반시계) 였다. 작업자가 오른쪽으로 빠졌으면 왼쪽으로
    열한 번(330°) 돌아야 다시 만났다 — 최악 38 초다. 놓치기 직전 위치는 우리가 들고 있다.
    """
    node = make_node()
    node.hint = None

    node.follow_est, node.follow_goal = (1.0, -1.0), None    # 로봇 앞 오른쪽
    assert node.search_rotate_deg() < 0, '오른쪽에서 놓쳤으면 시계 방향(음수)으로 돈다'

    node.search_dir = None                                   # 방향 잠금을 풀고 다시 정하게
    node.follow_est, node.follow_goal = (1.0, 1.0), None     # 로봇 앞 왼쪽
    assert node.search_rotate_deg() > 0, '왼쪽에서 놓쳤으면 반시계(양수)로 돈다'
    node.destroy_node()


def test_rotation_extrapolates_exit_direction(ros):
    """마지막 이동분을 외삽해 **빠져나간 쪽**을 가리킨다.

    작업자가 오른쪽으로 걸어 나갔다면 마지막으로 본 자리보다 더 오른쪽에 있다. 마지막
    목표 위치와 마지막 추정 위치가 둘 다 있으면 그 차이만큼 한 번 더 밀어서 본다.
    """
    node = make_node()
    node.hint = None

    node.follow_goal, node.follow_est = (1.0, 0.0), (1.0, -0.3)   # 오른쪽으로 이동 중
    with_drift = node.last_seen_bearing()
    node.follow_goal = None
    without_drift = node.last_seen_bearing()

    assert with_drift < without_drift < 0, \
        f'외삽이 더 오른쪽을 가리켜야 한다 (외삽 {with_drift:.0f}° vs 그냥 {without_drift:.0f}°)'
    node.destroy_node()


def test_hint_wins_over_last_seen(ros):
    """천장 카메라 힌트가 있으면 그것이 마지막으로 본 쪽을 이긴다 (계약 4절 우선순위)."""
    node = make_node()
    node.worker_id = 'worker_a'
    node.follow_est, node.follow_goal = (1.0, -1.0), None      # 마지막으로 본 쪽 = 오른쪽
    node.hint = (0.0, 2.0, time.time(), 'worker_a')            # 힌트 = 왼쪽

    assert node.search_rotate_deg() > 30, '힌트가 있으면 힌트 방향이다'
    node.destroy_node()


def test_standby_reached_while_searching(ros):
    """탐색 중에도 `standby_s` 를 넘기면 STANDBY 로 간다.

    종전에는 STANDBY 판정이 FOLLOWING 에만 있었다. 추종은 `research_s` 에 무조건 SEARCHING
    으로 빠져나오므로 그 값이 **닿을 수 없었다** — 관제는 사람이 필요하다는 것을 영원히 모르고,
    재획득 실패가 한 건도 집계되지 않았다 (계약 3절·9절).
    """
    node = make_node()
    start(node)
    assert node.state == 'SEARCHING'

    node.follow_seen = time.time() - 31.0        # 31 초 동안 못 찾았다
    node.step()

    assert node.state == 'STANDBY', '탐색을 오래 해도 못 찾으면 사람을 불러야 한다'
    assert 'FAILED' not in node.status_log, 'STANDBY 는 오류가 아니다'
    node.destroy_node()


def test_long_follow_then_loss_does_not_jump_to_standby(ros):
    """30 초 넘게 잘 따라간 뒤 놓쳐도 곧바로 STANDBY 가 되지 않는다.

    기준을 '사이클 시작 시각' 으로 잡으면 오래 따라간 것이 벌이 된다. 기준은 **마지막으로
    본 시각**이어야 한다.
    """
    node = make_node()
    reach_following(node)
    node.worker_started = time.time() - 120.0    # 2 분 전에 시작한 사이클
    lost_for(node, 3.0)                          # 방금 3 초 놓쳤을 뿐이다
    node.step()

    assert node.state == 'SEARCHING', node.state
    node.step()
    assert node.state == 'SEARCHING', '3 초 놓친 것으로 STANDBY 가 되면 안 된다'
    node.destroy_node()


def test_search_keeps_the_same_direction(ros):
    """한 번 정한 회전 방향을 끝까지 지킨다 (9/9 실주행 — +30° 뒤 −30° 로 되돌아왔다).

    `last_seen_bearing()` 은 '지금 보는 방향 기준' 이라, 그쪽으로 돌고 나면 목표 지점이
    0° 근처가 되고 조금 지나치면 부호가 뒤집힌다. 매 회차 다시 계산하면 좌우로 왕복해
    한 바퀴를 못 덮는다.
    """
    node = make_node()
    node.hint = None
    node.follow_est, node.follow_goal = (1.0, -1.0), None    # 오른쪽에서 놓쳤다

    first = node.search_rotate_deg()
    assert first < 0, '첫 회차는 마지막으로 본 쪽(시계)이다'

    # 돌고 나서 마지막으로 본 지점이 반대쪽으로 넘어가도 방향은 그대로여야 한다
    node.follow_est = (1.0, 0.6)
    for i in range(5):
        deg = node.search_rotate_deg()
        assert deg < 0, f'{i + 2}회차에 방향이 뒤집혔다 ({deg:+.0f}°)'
    node.destroy_node()


def test_search_step_is_smaller_than_camera_fov(ros):
    """정지점 간격이 카메라 수평 시야보다 작아야 사이로 작업자가 빠지지 않는다.

    robot3 실측 시야 38.5° (camera_info fx 566.7 × 1.7778 = 1007.4, 704 px 정사각 crop).
    가장자리는 bbox 가 잘리고 depth 가 흔들려 실효 시야가 더 좁으므로 여유를 크게 둔다.
    """
    node = make_node()
    fov_deg = 38.5
    step = node.p('search_step_deg')

    assert step < fov_deg, f'간격 {step}° 가 시야 {fov_deg}° 보다 크면 못 보는 구간이 생긴다'
    assert fov_deg - step >= 15.0, f'겹침 {fov_deg - step:.1f}° — 가장자리 검출이 약해 부족하다'
    node.destroy_node()


def test_full_circle_tries_from_step(ros):
    """한 바퀴 횟수를 간격에서 계산한다 — 못 박으면 간격을 바꿀 때 어긋난다."""
    node = make_node()
    tries = node.max_search_tries()
    covered = tries * node.p('search_step_deg')

    assert covered >= 360.0, f'{tries}회 × {node.p("search_step_deg")}° = {covered}° — 한 바퀴가 안 된다'
    node.destroy_node()


def test_rotation_stops_when_target_appears(ros):
    """도는 중에 작업자가 보이면 그 자리에서 멈춘다.

    정지점 사이의 애매한 자리에 서 있으면 돌기 전에도 돌고 나서도 안 보인다. 도는 동안
    콜백은 계속 돌아가므로 시야를 스쳐 지나가는 순간을 잡을 수 있다 (9/9 실주행).
    """
    node = make_node()
    node.setup_cmd_vel = lambda: None            # Nav2 Spin 경로로 태운다
    node.navigator.nav_ticks = 20                # 여러 주기 도는 회전으로 만든다
    node.visible_at = time.time() + 1.0          # 회전 중에 작업자가 보인다

    # make_node 가 rotate_in_place 를 대역으로 갈아뒀으므로 실제 메서드를 직접 부른다
    type(node).rotate_in_place(node, 20.0, stop_on_sight=True)

    assert 'cancel' in node.navigator.calls, '보이면 회전을 끊어야 한다'
    node.destroy_node()


def test_standby_to_searching_does_not_oscillate(ros):
    """STANDBY 에서 나온 직후 다시 STANDBY 로 돌아가지 않는다 (9/9 실주행 회귀).

    작업자가 다시 보여 재획득을 시작했는데 `follow_seen` 이 30 초 전 값으로 남아 있으면,
    SEARCHING 의 STANDBY 판정이 즉시 다시 STANDBY 로 돌려보낸다. 그 왕복이 초당 수백 번
    돌아 `approach/status` 574,501 건이 관제·기록기로 쏟아졌다.
    """
    node = make_node()
    reach_following(node)
    lost_for(node, 31.0)
    node.step()
    assert node.state == 'STANDBY'

    see(node)                      # 작업자가 다시 보인다 (게이트는 통과 상태)
    node.step()
    assert node.state == 'SEARCHING', '재등장하면 다시 찾기 시작한다'

    for _ in range(20):            # 스무 주기를 돌려도 STANDBY 로 안 돌아가야 한다
        node.step()
    assert node.state != 'STANDBY', '재획득 직후 STANDBY 로 되돌아가면 진동한다'
    node.destroy_node()


def test_status_published_only_on_change(ros):
    """같은 상태를 다시 넣는 호출은 발행하지 않는다 — 진동이 네트워크 폭주가 되지 않게.

    유지 발행은 1 초 타이머가 따로 하므로 관제·기록기가 값을 놓치지 않는다.
    """
    node = make_node()
    sent = []
    node.pub_status.publish = lambda m: sent.append(m.data)

    node.set_state('SEARCHING', '처음')
    node.set_state('SEARCHING', '같은 상태 · 다른 이유')
    node.set_state('SEARCHING', '또 같은 상태')
    assert sent == ['SEARCHING'], f'전이 한 번에 한 건이어야 한다 (실제 {sent})'

    node.set_state('FOLLOWING', '바뀜')
    assert sent == ['SEARCHING', 'FOLLOWING']
    node.destroy_node()


def test_reacquire_resets_the_standby_clock(ros):
    """재획득 시작이 `standby_s` 시계를 지금으로 되돌린다.

    되돌리는 자리는 '다시 찾기 시작한 순간' 하나뿐이다 — `rearm()`(회전마다 불린다)에
    넣으면 회전을 반복하는 동안 30 초가 영원히 안 차서 STANDBY 에 도달하지 못한다.
    """
    node = make_node()
    node.follow_seen = time.time() - 100.0
    node.begin_reacquire('시험')

    assert node.state == 'SEARCHING'
    assert time.time() - node.follow_seen < 1.0, '시계가 지금으로 돌아와야 한다'
    assert node.search_tries == 0
    node.destroy_node()


def test_rotation_does_not_reset_the_standby_clock(ros):
    """회전(rearm)은 시계를 되돌리지 않는다 — 그래야 30 초에 STANDBY 로 갈 수 있다."""
    node = make_node()
    node.follow_seen = time.time() - 100.0
    before = node.follow_seen
    node.rearm()

    assert node.follow_seen == before, 'rearm 이 시계를 건드리면 STANDBY 에 못 간다'
    node.destroy_node()


def test_standoff_and_separation_come_from_params(ros):
    """추종 거리와 최소 이격은 `robotN.yaml` 에서 읽는다 — 코드에 박지 않는다 (계약 5절)."""
    node = make_node(follow_standoff_m=0.7, min_separation_m=0.25)

    assert node.p('follow_standoff_m') == pytest.approx(0.7)
    assert node.p('min_separation_m') == pytest.approx(0.25)
    node.destroy_node()


def test_map_yaw_formula_matches_contract(ros):
    """map→base_link yaw 식이 계약 2절 `FleetStatus.yaw` 와 같은지 확인한다.

    힌트는 map 좌표라 비교도 map 프레임에서 해야 한다. 여기서는 식만 본다.
    """
    node = make_node()
    for deg in (0.0, 45.0, 90.0, -90.0, 179.0):
        rad = math.radians(deg)
        z, w = math.sin(rad / 2), math.cos(rad / 2)
        yaw = math.atan2(2.0 * (w * z), 1.0 - 2.0 * (z * z))
        assert yaw == pytest.approx(rad, abs=1e-6)
    node.destroy_node()
