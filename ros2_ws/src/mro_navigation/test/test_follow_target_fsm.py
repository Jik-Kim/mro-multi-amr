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
  · Lost 3단 — `lost_s` 정지 · 8 초 회전 탐색 · `standby_s` STANDBY. 재등장 시 게이트 재통과
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
    node.rotate_in_place = lambda deg: node.turns.append(round(deg, 1))
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
    lost_for(node, 2.0)
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
    assert node.search_rotate_deg() == 30.0

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
            if not ln.lstrip().startswith('#') and 'linear' in ln or '.undock(' in ln]
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
