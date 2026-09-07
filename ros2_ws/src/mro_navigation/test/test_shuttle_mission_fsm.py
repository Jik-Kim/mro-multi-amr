"""셔틀 미션 상태 머신 단위 검증 — 초안 (2026-09-07, shuttle_mission_node 개편과 같이 추가).

`shuttle_mission_node` 의 **실제 코드**를 로봇 없이 돌린다. `turtlebot4_navigation` 은 이 저장소
밖 패키지(제어 PC 에만 있다)라 스텁으로 갈아끼우고, `rclpy` 와 `mro_interfaces` 는 실물을 쓴다.
`arena.yaml` 도 `mro_bringup` 소스의 실제 파일을 읽으므로 좌표가 바뀌면 여기서 먼저 드러난다.

**보장하는 것** — 상태 문자열의 순서와 시점이 계약 4절·I-024·I-026 과 맞는가.
  · 정상 미션: SHUTTLE → AT_BAY → SHUTTLE → WAITING (WAITING 은 대기공간 도착 때 한 번)
  · ArUco 불일치·T_lim 초과: FAILED 로 끝나고 WAITING·shuttle/arrived 를 내지 않는다
    (계약 7절 이중 계상 금지 — 포기한 미션은 폐기한다)
  · P1 토큰: 베이를 떠나기 전 획득, 대기공간 도착 후 반납, 거절 중에는 AT_BAY 유지
    (순서 기반 판정 = 방법 B, 9/7 팀 회의. 그 전제인 '베이·대기공간이 P1 폴리곤 밖' 을
     arena.yaml 좌표로 함께 확인한다)
  · dock_pose 가 (0,0,0) 이면 거부한다 (I-013 — 두 로봇의 맵 좌표계가 어긋난다)

**보장하지 못하는 것 — 실주행에서만 확인된다.**
  · Nav2 가 실제로 베이 → P1 → 대기공간 순서로 지나는지. 방법 B 는 이 순서를 전제로 한다
  · 실제 `fleet_manager` 가 이 상태열을 받아 `missions.result` 를 옳게 확정하는지 (TS-07 9/9)
  · 베이 전진 0.3 m 가 선반과 부딪히지 않는지, ArUco 를 실제로 1.5 m 안에서 읽는지
  · undock/dock 블로킹 구간의 동작 (스텁은 즉시 반환한다)

실행 (`colcon build` 로 `mro_interfaces` 가 설치돼 있어야 한다):

    cd ros2_ws/src/mro_navigation
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_shuttle_mission_fsm.py -v

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 은 이 PC 의 `launch_testing_ros` 플러그인이 pytest 9 와
맞지 않아 붙인 것이다 (이 저장소 문제가 아니라 환경 문제 — `test_flake8.py` 도 같은 이유로 못 돈다).
"""
import math
from pathlib import Path
import sys
import types

from mro_interfaces.msg import Mission
import pytest
import rclpy
from rclpy.parameter import Parameter

ARENA = Path(__file__).resolve().parents[2] / 'mro_bringup' / 'params' / 'arena.yaml'
BASE = {
    'dock_pose': [0.019, -0.795, 0.133],
    'bay_dwell_s': 0.2,
    't_lim_s': 30.0,
    'aruco_confirm_s': 0.5,
    'aruco_retry': 1,
    'token_retry_s': 0.1,
}


class StubNavigator:
    """TurtleBot4Navigator 대역. 호출을 기록하고 목표를 nav_ticks 뒤 완료로 만든다."""

    nav_ticks = 3

    def __init__(self, namespace=''):
        """호출 기록과 도킹 상태를 초기화한다."""
        self.ns = namespace
        self.calls = []
        self.docked = True
        self.result_name = 'SUCCEEDED'
        self.goal_accepted = True
        self._ticks = 0

    def getPoseStamped(self, position, rotation):  # noqa: N802 (원본 API 이름)
        """실물과 같은 규약으로 PoseStamped 를 만든다 — rotation 은 도 단위."""
        from geometry_msgs.msg import PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.orientation.z = math.sin(math.radians(rotation) / 2)
        pose.pose.orientation.w = math.cos(math.radians(rotation) / 2)
        return pose

    def getDockedStatus(self):  # noqa: N802
        """현재 도킹 상태."""
        return self.docked

    def dock(self):
        """도킹."""
        self.calls.append('dock')
        self.docked = True

    def undock(self):
        """언독."""
        self.calls.append('undock')
        self.docked = False

    def setInitialPose(self, pose):  # noqa: N802
        """초기자세 전송을 기록한다."""
        self.calls.append(
            ('initpose', round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)))

    def goToPose(self, pose, behavior_tree=''):  # noqa: N802
        """목표 전송을 기록하고 nav_ticks 만큼 미완료 상태를 유지한다."""
        self.calls.append(
            ('goto', round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)))
        self._ticks = self.nav_ticks
        return self.goal_accepted

    def isTaskComplete(self):  # noqa: N802
        """nav_ticks 번 물어보면 완료로 답한다."""
        self._ticks -= 1
        return self._ticks <= 0

    def getResult(self):  # noqa: N802
        """결과 이름만 있는 최소 객체."""
        return types.SimpleNamespace(name=self.result_name)

    def cancelTask(self):  # noqa: N802
        """취소 호출을 기록한다."""
        self.calls.append('cancel')

    def goals(self):
        """전송한 goToPose 목표만 추린다."""
        return [c for c in self.calls if c[0] == 'goto']


def _shuttle_mission_class():
    """turtlebot4_navigation 을 스텁으로 갈아끼운 뒤 노드 클래스를 불러온다."""
    if 'turtlebot4_navigation' not in sys.modules:
        pkg = types.ModuleType('turtlebot4_navigation')
        mod = types.ModuleType('turtlebot4_navigation.turtlebot4_navigator')
        mod.TurtleBot4Navigator = StubNavigator
        pkg.turtlebot4_navigator = mod
        sys.modules['turtlebot4_navigation'] = pkg
        sys.modules['turtlebot4_navigation.turtlebot4_navigator'] = mod
    from mro_navigation.nodes.shuttle_mission_node import ShuttleMission
    return ShuttleMission


@pytest.fixture(scope='module')
def ros():
    """arena.yaml 경로를 전역 파라미터로 넘긴 채 rclpy 를 띄운다 (install 공간에 의존하지 않게)."""
    rclpy.init(args=['--ros-args', '-p', f'arena_yaml:={ARENA}'])
    yield
    rclpy.shutdown()


def make_node(**params):
    """발행 내용을 기록하도록 감싼 노드를 만든다. Nav2 준비 확인은 통과시킨다."""
    node = _shuttle_mission_class()()
    node.status_log = []
    node.arrived_log = []
    node.token_log = []
    node.token_marks = []
    pub_status, pub_arrived = node.pub_status.publish, node.pub_arrived.publish
    node.pub_status.publish = lambda m: (node.status_log.append(m.data), pub_status(m))
    node.pub_arrived.publish = lambda m: (node.arrived_log.append(m.data), pub_arrived(m))
    node.nav2_ready = lambda: True
    node.set_parameters([Parameter(k, value=v) for k, v in {**BASE, **params}.items()])
    return node


def stub_token(node, granted_sequence):
    """토큰 획득 응답을 시나리오대로 돌려주고 호출 순서를 기록한다."""
    answers = iter(granted_sequence)

    def token(acquire):
        action = 'acquire' if acquire else 'release'
        node.token_log.append(action)
        # 몇 번째 목표까지 보낸 뒤에 불렸는지 같이 남긴다 (방법 B 의 시점 검증용)
        node.token_marks.append((action, len(node.navigator.goals())))
        return next(answers, True) if acquire else True

    node.token = token


def assigned(bay='A', mission_id='M1'):
    """관제가 보내는 Mission(ASSIGNED) 을 흉내낸다."""
    message = Mission()
    message.mission_id = mission_id
    message.worker_id = 'W1'
    message.robot_ns = ''
    message.bay = bay
    message.state = Mission.ASSIGNED
    return message


def fresh_marker(node, marker_id=1, distance=0.9):
    """마커 콜백이 방금 값을 넣은 것처럼 만든다 (ArUco id·거리)."""
    now = node.now()
    node.aruco_id = (marker_id, now)
    node.aruco_dist = (distance, now)


def test_normal_mission(ros):
    """정상 미션의 상태열·목표·토큰 순서."""
    node = make_node()
    stub_token(node, [True])
    fresh_marker(node)
    node.run_mission(assigned('A'))
    assert node.status_log == ['SHUTTLE', 'AT_BAY', 'SHUTTLE', 'WAITING', 'READY']
    assert node.arrived_log == [True]
    assert node.token_log == ['acquire', 'release']
    # 베이 → 창고 안쪽 0.3 m 전진 → 대기공간. 전진 목표는 베이 yaw(-3.05 rad) 방향이다
    assert node.navigator.goals() == [
        ('goto', -0.78, 0.138), ('goto', -1.079, 0.111), ('goto', -0.998, 0.937)]
    assert ('initpose', 0.019, -0.795) in node.navigator.calls
    assert (node.mission, node.t0) == (None, None)
    node.destroy_node()


def test_aruco_mismatch_aborts_without_waiting(ros):
    """다른 베이의 마커가 보이면 재정렬 후 포기한다 — 오도착 0건 (FR-08)."""
    node = make_node()
    stub_token(node, [True])
    original_wait = node.wait

    def wait(cond, timeout=None, on_tick=None):
        # 콜백이 없으므로 매 tick 마커를 갱신해 '신선하지만 ID 가 다른' 상황을 만든다
        def tick():
            fresh_marker(node, marker_id=1)
            if on_tick:
                on_tick()
        return original_wait(cond, timeout, tick)

    node.wait = wait
    node.run_mission(assigned('B', 'M2'))
    assert node.status_log == ['SHUTTLE', 'AT_BAY', 'FAILED', 'READY']
    assert node.arrived_log == []
    assert len(node.navigator.goals()) == 2      # 베이 + 재정렬 1회
    assert 'cancel' in node.navigator.calls
    assert node.token_log == []                  # 토큰은 잡지도 않았다
    node.destroy_node()


def test_tlim_discards_mission(ros):
    """T_lim 을 넘기면 포기하고 미션을 폐기한다 (계약 7절 이중 계상 금지)."""
    node = make_node(t_lim_s=0.3, bay_dwell_s=2.0)
    stub_token(node, [True])
    fresh_marker(node)
    node.run_mission(assigned('A', 'M3'))
    assert node.status_log[-2:] == ['FAILED', 'READY']
    assert 'WAITING' not in node.status_log
    assert node.arrived_log == []
    assert (node.mission, node.t0) == (None, None)
    node.destroy_node()


def test_dock_pose_missing_is_refused(ros):
    """dock_pose 가 (0,0,0) 이면 언독하지 않고 포기한다 (I-013)."""
    node = make_node(dock_pose=[0.0, 0.0, 0.0])
    node.run_mission(assigned('A', 'M4'))
    assert node.status_log == ['FAILED', 'READY']
    assert 'undock' not in node.navigator.calls
    node.destroy_node()


def test_token_retry_keeps_at_bay(ros):
    """토큰이 거절되는 동안 상태는 AT_BAY 로 남는다 — WAITING 을 내면 배송 판정이 깨진다."""
    node = make_node()
    stub_token(node, [False, False, True])
    fresh_marker(node)
    node.run_mission(assigned('A', 'M5'))
    assert node.token_log == ['acquire', 'acquire', 'acquire', 'release']
    assert node.status_log == ['SHUTTLE', 'AT_BAY', 'SHUTTLE', 'WAITING', 'READY']
    node.destroy_node()


def test_p1_token_spans_bay_to_waiting(ros):
    """방법 B — 토큰은 베이를 떠나기 전에 잡고 대기공간에 도착한 뒤 푼다 (9/7 팀 회의)."""
    node = make_node()
    stub_token(node, [True])
    fresh_marker(node)
    node.run_mission(assigned('A', 'M6'))
    # 목표 3건: ① 베이 ② 창고 안쪽 전진 ③ 대기공간.
    #   획득은 ②까지 끝난 뒤(=베이를 아직 안 떠남), 반납은 ③까지 끝난 뒤여야 한다
    assert node.token_marks == [('acquire', 2), ('release', 3)]
    assert node.token_log.count('release') == 1     # finally 에서 두 번 풀지 않는다
    node.destroy_node()


def test_aruco_ok(ros):
    """마커 판정 — ID·거리·신선도. 두 토픽은 헤더가 없어 각자 수신 시각으로 본다."""
    node = make_node()
    now = node.now()
    cases = [
        ((1, now), (0.9, now), True, '일치·근거리'),
        ((2, now), (0.9, now), False, 'ID 불일치'),
        ((-1, now), (0.9, now), False, '미검출'),
        ((1, now), (1.8, now), False, '1.5 m 초과'),
        ((1, now - 2.0), (0.9, now), False, 'ID 만 오래됨'),
        ((1, now), (0.9, now - 2.0), False, '거리만 오래됨'),
        (None, (0.9, now), False, 'ID 미수신'),
    ]
    for marker, distance, expected, label in cases:
        node.aruco_id, node.aruco_dist = marker, distance
        assert node.aruco_ok(1) is expected, label
    node.destroy_node()


def _inside(x, y, poly):
    """Ray casting — 방법 B 의 전제를 확인하기 위한 테스트 전용 헬퍼."""
    hit = False
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            hit = not hit
    return hit


def test_method_b_premise_holds_in_arena():
    """방법 B 가 계약 3절을 만족하려면 베이와 대기공간이 둘 다 P1 폴리곤 밖이어야 한다.

    좌표 감시를 없앤 대신, 그 근거가 되는 전제를 arena.yaml 로 여기서 지킨다.
    베이가 폴리곤 안이면 '진입 전 획득' 이 깨지고, 대기공간이 안이면 '이탈 후 반납' 이 깨진다.
    """
    import yaml
    arena = yaml.safe_load(ARENA.read_text(encoding='utf-8'))['arena']
    poly = [(float(a), float(b)) for a, b in arena['semaphore_zones']['P1']['polygon']]

    assert _inside(-0.4, 0.7, poly) is True          # 헬퍼 자체가 맞는지
    for name, bay in arena['bays'].items():
        assert _inside(bay['waypoint'][0], bay['waypoint'][1], poly) is False, f'베이 {name}'
    for robot, spot in arena['waiting_zone'].items():
        assert _inside(spot[0], spot[1], poly) is False, f'대기공간 {robot}'


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
