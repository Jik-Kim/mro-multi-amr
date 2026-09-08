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
  · 초기자세를 **미션이 아니라 기동 시** 보낸다 (I-035). 도크 밖이면 보내지 않고, 기동 경로는
    미션이 없으므로 FAILED 를 내지 않는다. Nav2 가 active 가 아니면 T_lim 을 태우지 않고 곧장 포기
  · 베이 접근·정렬 (9/8, 옛 '창고 안쪽 0.3 m 전진' 대체): 마커에서 bay_marker_range_m 인 자리로
    회전–직진–회전으로 간다. aruco/pose 가 없으면 **목적지를 모르므로 포기**하고, 이동량이
    상한을 넘어도 포기한다. Nav2 목표가 아니라 Spin·DriveOnHeading 이다 — SimpleGoalChecker
    허용오차 xy 0.25 m 가 이 크기의 이동을 '이미 도착' 으로 처리하는 것을 피한다

**보장하지 못하는 것 — 실주행에서만 확인된다.**
  · Nav2 가 실제로 베이 → P1 → 대기공간 순서로 지나는지. 방법 B 는 이 순서를 전제로 한다
  · 실제 `fleet_manager` 가 이 상태열을 받아 `missions.result` 를 옳게 확정하는지 (TS-07 9/9)
  · 베이 전진 0.3 m 가 선반과 부딪히지 않는지, ArUco 를 실제로 1.5 m 안에서 읽는지
  · undock/dock 블로킹 구간의 동작 (스텁은 즉시 반환한다)

실행 (`colcon build` 로 `mro_interfaces` 가 설치돼 있어야 한다):

    bash tools/test_shuttle_offline.sh -v

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 은 이 PC 의 `launch_testing_ros` 플러그인이 pytest 9 와
맞지 않아 붙인 것이다 (이 저장소 문제가 아니라 환경 문제 — `test_flake8.py` 도 같은 이유로 못 돈다).
"""
import math
import os
from pathlib import Path
import sys
import types

from mro_interfaces.msg import Mission
import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

ARENA = Path(__file__).resolve().parents[2] / 'mro_bringup' / 'params' / 'arena.yaml'
# 테스트용 네임스페이스. arena.yaml 의 waiting_zone·dock_zone 에 키가 있는 로봇이어야 한다
TEST_NS = '/robot3'
# 광학 프레임(x 우·y 하·z 앞) → base_link(x 앞·y 좌·z 상) 회전. TB4 의 카메라 TF 와 같은 꼴이다.
#   R = [[0,0,1],[-1,0,0],[0,-1,0]] 이라 광학 (px,py,pz) 가 base 에서 (pz, −px, −py) 가 된다
OPTICAL_TO_BASE_Q = (-0.5, 0.5, -0.5, 0.5)     # (x, y, z, w)
CAMERA_FRAME = 'oakd_rgb_camera_optical_frame'
BASE = {
    'dock_pose': [0.019, -0.795, 0.133],
    'bay_dwell_s': 0.2,
    't_lim_s': 30.0,
    'aruco_confirm_s': 0.5,
    'aruco_retry': 1,
    'token_retry_s': 0.1,
    'bay_align_sample_s': 0.05,   # 마커 표본을 기다리는 시간. 시험을 빠르게 끝내려고 줄인다
    # 접근 단계는 **기본으로 끈다.** 마커 pose 를 주지 않는 시험이 대부분이고, 마커 없이 접근하면
    #   목적지를 모르니 포기가 정상이다 (test_bay_approach_without_marker_pose_aborts 가 그것을 본다).
    #   접근을 보는 시험은 marker_frames() 와 함께 bay_align_enable=True 로 켠다.
    'bay_align_enable': False,
}


class StubNavigator(Node):
    """TurtleBot4Navigator 대역. 호출을 기록하고 목표를 nav_ticks 뒤 완료로 만든다.

    **실물과 같이 rclpy Node 다.** 노드의 `docked_status()` 가 `rclpy.spin_once(navigator)` 를
    부르기 때문에 대역도 Node 여야 한다.
    **`is_docked` 는 실물과 같이 구독 콜백으로만 바뀐다** — `dock()`·`undock()` 은 건드리지 않는다
    (turtlebot4_navigator.py 확인). 대역이 여기서 값을 직접 바꾸면 '묵은 값을 돌려주는' 결함을
    시험이 못 잡는다 (PR #44 PM 지적). 콜백 도착은 `deliver_dock_status()` 로 흉내낸다.
    """

    nav_ticks = 3

    def __init__(self, namespace=''):
        """호출 기록과 도킹 상태를 초기화한다."""
        super().__init__('stub_navigator', namespace=namespace)
        self.ns = namespace
        self.calls = []
        self.is_docked = True     # 실물 TurtleBot4Navigator 와 같은 이름 (노드가 직접 읽는다)
        self.result_name = 'SUCCEEDED'
        self.goal_accepted = True
        self._ticks = 0
        self.pose = None
        # 상대 이동(Spin·DriveOnHeading)은 목표 좌표가 없어 '어디로 갔는지' 를 대역이 직접
        #   굴려야 검증할 수 있다. goToPose 는 목표로 순간이동시킨다 (예전과 같은 동작)
        self.sim = (0.0, 0.0, 0.0)

    def deliver_dock_status(self, value):
        """실물의 _dockCallback 이 도착한 것과 같다 — is_docked 는 이 경로로만 바뀐다."""
        self.is_docked = value

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
        return self.is_docked

    def dock(self):
        """도킹. 실물과 같이 is_docked 를 직접 바꾸지 않는다 (구독 콜백만 바꾼다)."""
        self.calls.append('dock')

    def undock(self):
        """언독. 실물과 같이 is_docked 를 직접 바꾸지 않는다 (구독 콜백만 바꾼다)."""
        self.calls.append('undock')

    def setInitialPose(self, pose):  # noqa: N802
        """초기자세 전송을 기록한다."""
        self.calls.append(
            ('initpose', round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)))

    def goToPose(self, pose, behavior_tree=''):  # noqa: N802
        """목표 전송을 기록하고 nav_ticks 만큼 미완료 상태를 유지한다."""
        self.pose = pose
        q = pose.pose.orientation
        self.sim = (pose.pose.position.x, pose.pose.position.y,
                    math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        self.calls.append(
            ('goto', round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)))
        self._ticks = self.nav_ticks
        return self.goal_accepted

    def spin(self, spin_dist=1.57, time_allowance=10):
        """제자리 회전. 실물 BasicNavigator 와 같은 서명·반환값이다."""
        x, y, yaw = self.sim
        self.sim = (x, y, yaw + float(spin_dist))
        self.calls.append(('spin', round(float(spin_dist), 3)))
        self._ticks = self.nav_ticks
        return self.goal_accepted

    def driveOnHeading(self, dist=0.15, speed=0.025, time_allowance=10):  # noqa: N802
        """정면으로 dist 만큼 직진. 후진은 이 경로로 오지 않는다 (계약 2절)."""
        x, y, yaw = self.sim
        self.sim = (x + dist * math.cos(yaw), y + dist * math.sin(yaw), yaw)
        self.calls.append(('drive', round(float(dist), 3)))
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

    def moves(self):
        """상대 이동(Spin·DriveOnHeading)만 추린다."""
        return [c for c in self.calls if c[0] in ('spin', 'drive')]


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
    """arena.yaml 경로를 전역 파라미터로 넘긴 채 rclpy 를 띄운다 (install 공간에 의존하지 않게).

    네임스페이스도 함께 준다 — 노드가 ns 없이는 뜨지 않는다 (9/8). ns 는 arena.yaml 의
    waiting_zone·dock_zone 조회 키라 대체값을 쓰면 남의 자리로 가므로, 대체값 대신 죽게 했다.
    """
    if os.environ.get('ROS_DOMAIN_ID') != '201' or os.environ.get('ROS_DISCOVERY_SERVER'):
        pytest.fail('실물과 분리해 실행할 것: bash tools/test_shuttle_offline.sh -v')
    rclpy.init(args=['--ros-args', '-r', f'__ns:={TEST_NS}', '-p', f'arena_yaml:={ARENA}',
                     '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'])
    yield
    rclpy.shutdown()


def make_node(simulate_pose=True, **params):
    """FSM 시험은 목표 도달 TF를 공급한다. TF 갱신 시험은 simulate_pose=False로 분리한다."""
    node = _shuttle_mission_class()()
    node.status_log = []
    node.arrived_log = []
    node.token_log = []
    node.token_marks = []
    pub_status, pub_arrived = node.pub_status.publish, node.pub_arrived.publish
    node.pub_status.publish = lambda m: (node.status_log.append(m.data), pub_status(m))
    node.pub_arrived.publish = lambda m: (node.arrived_log.append(m.data), pub_arrived(m))
    node.nav2_state = lambda: (True, 'active')
    node.set_parameters([Parameter(k, value=v) for k, v in {**BASE, **params}.items()])
    if simulate_pose:
        from geometry_msgs.msg import TransformStamped

        def lookup(target, source, *args, **kwargs):
            """map→base_link 는 대역의 현재 자세, base_link→카메라는 고정 광학 회전."""
            transform = TransformStamped()
            transform.header.stamp = node.get_clock().now().to_msg()
            transform.header.frame_id = target
            transform.child_frame_id = source
            if source == CAMERA_FRAME:
                q = transform.transform.rotation
                q.x, q.y, q.z, q.w = OPTICAL_TO_BASE_Q
                return transform
            x, y, yaw = node.navigator.sim
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.rotation.z = math.sin(yaw / 2)
            transform.transform.rotation.w = math.cos(yaw / 2)
            return transform

        node.tf_buffer.lookup_transform = lookup
    return node


class ScriptedPoses(list):
    """aruco/pose 버퍼 대역. marker_sample() 이 clear() 한 뒤 다시 채워 넣는다.

    노드는 '먼저 비우고 새 표본을 기다린다' 라 시험이 미리 넣어 둔 값이 지워진다. 토픽으로
    실제 발행하면 spin 타이밍에 따라 시험이 흔들리므로, 비울 때 같은 대본을 되채워
    **marker_sample() 의 진짜 코드(TF 변환·중앙값·법선 뒤집기)를 그대로 돌린다.**
    """

    def __init__(self, script):
        """대본을 그대로 담고, 되채울 사본을 따로 든다."""
        super().__init__(script)
        self.script = list(script)

    def clear(self):
        """비운 뒤 대본을 되채운다 — 노드에게는 새 프레임이 방금 온 것과 같다."""
        del self[:]
        self.extend(self.script)


def marker_frames(node, fwd, lat, count=5, flip_normal=False):
    """마커가 base_link 에서 (fwd, lat) 에 정면으로 보이는 aruco/pose 를 대본으로 넣는다.

    광학 프레임에서 (px, py, pz) 는 base 에서 (pz, −px, −py) 이므로 px = −lat, pz = fwd 다.
    자세는 x 축 180° 회전(1,0,0,0) — 마커의 +z(법선)가 카메라 쪽 = base −x 를 향한다.
    flip_normal 은 solvePnP 가 법선이 뒤집힌 해를 낸 경우다 (노드가 되돌려야 한다).
    """
    from geometry_msgs.msg import PoseStamped
    script = []
    for _ in range(count):
        m = PoseStamped()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = CAMERA_FRAME
        m.pose.position.x, m.pose.position.y, m.pose.position.z = -lat, 0.0, fwd
        # 네 성분을 다 적는다 — Quaternion 의 기본값이 w=1.0 이라 한 성분만 바꾸면 조용히
        #   정규화되지 않은(= 다른 각도의) 회전이 된다
        q = m.pose.orientation
        if flip_normal:
            q.x, q.y, q.z, q.w = 0.0, 0.0, 0.0, 1.0   # 법선이 +z (카메라 반대쪽) 를 향하는 해
        else:
            q.x, q.y, q.z, q.w = 1.0, 0.0, 0.0, 0.0   # x 축 180° — 법선이 카메라를 향한다
        script.append((m, node.now()))
    node.aruco_poses = ScriptedPoses(script)


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
    # 베이 → 대기공간. 접근은 Nav2 목표가 아니라 상대 이동이고, 이 시험은 마커를 안 주므로
    #   접근 단계를 끈다 (마커가 없으면 목적지를 모른다 → 포기가 정상이다)
    assert node.navigator.goals() == [('goto', -0.78, 0.138), ('goto', -0.998, 0.937)]
    assert node.navigator.moves() == []
    assert ('initpose', 0.019, -0.795) in node.navigator.calls
    assert (node.mission, node.t0) == (None, None)
    node.destroy_node()


def test_bay_approach_without_marker_pose_aborts(ros):
    """aruco/pose 가 없으면 포기한다 — 그 pose 가 유일한 목적지 정보다.

    옛 '0.3 m 전진' 은 마커 없이도 갈 수 있었지만(전방으로 고정량), 접근은 마커에서 몇 m 인
    자리가 목적지라 마커를 못 보면 갈 곳이 없다. 조용히 건너뛰면 적재 위치에 못 간 채
    WAITING 을 내 AC-08 이 틀어진다 (I-024).
    """
    node = make_node(bay_align_enable=True)
    stub_token(node, [True])
    fresh_marker(node)                       # id·거리는 오지만 pose 는 없다 (주입 리그와 같다)
    node.run_mission(assigned('A'))
    assert node.status_log[-2:] == ['FAILED', 'READY']
    assert node.arrived_log == []
    assert node.navigator.moves() == []
    node.destroy_node()


def test_bay_approach_drives_to_target_range(ros):
    """마커가 1.0 m 앞에 있고 목표가 0.5 m 면 0.5 m 전진한다 — 이것이 옛 0.3 m 전진을 대신한다."""
    node = make_node(bay_align_enable=True, bay_align_max_iter=1, bay_marker_range_m=0.5)
    stub_token(node, [True])
    fresh_marker(node)
    marker_frames(node, fwd=1.0, lat=0.0)
    node.run_mission(assigned('A'))
    # 정면이라 첫 회전·마지막 회전이 0 이고 직진만 남는다
    assert node.navigator.moves() == [('drive', 0.5)]
    assert node.status_log[-2:] == ['WAITING', 'READY']
    node.destroy_node()


def test_bay_approach_corrects_lateral_offset(ros):
    """좌우로 치우쳐 보이면 회전–직진–회전으로 접근한다.

    마커가 base_link (1.0, +0.12), 목표가 정면 0.5 m 면 이동은 앞 0.5 · 좌 0.12 라
    첫 회전이 약 +13.5°, 이동량이 0.514 m, 마지막 회전이 그것을 되돌리는 값이어야 한다.
    """
    node = make_node(bay_align_enable=True, bay_align_max_iter=1)   # 대역은 마커가 안 따라온다
    stub_token(node, [True])
    fresh_marker(node)
    marker_frames(node, fwd=1.0, lat=0.12)
    node.run_mission(assigned('A'))
    moves = node.navigator.moves()
    assert [m[0] for m in moves] == ['spin', 'drive', 'spin']
    turn1, step, turn2 = moves[0][1], moves[1][1], moves[2][1]
    assert step == pytest.approx(math.hypot(0.5, 0.12), abs=0.002)
    assert turn1 == pytest.approx(math.atan2(0.12, 0.5), abs=0.002)
    assert turn2 == pytest.approx(-turn1, abs=1e-6)       # 베이 방위로 되돌린다
    assert node.status_log[-2:] == ['WAITING', 'READY']
    node.destroy_node()


def test_bay_approach_stops_when_already_at_target(ros):
    """마커가 이미 목표 자리에 있으면 한 번도 움직이지 않는다."""
    node = make_node(bay_align_enable=True, bay_marker_range_m=1.0, bay_marker_lat_m=0.0)
    stub_token(node, [True])
    fresh_marker(node)
    marker_frames(node, fwd=1.0, lat=0.0)
    node.run_mission(assigned('A'))
    assert node.navigator.moves() == []
    assert node.status_log[-2:] == ['WAITING', 'READY']
    node.destroy_node()


def test_bay_approach_refuses_oversized_move(ros):
    """이동량이 상한을 넘으면 움직이지 않고 포기한다 — 선반 충돌을 막는 마지막 방어다."""
    node = make_node(bay_align_enable=True, bay_align_max_step_m=0.6)
    stub_token(node, [True])
    fresh_marker(node)
    marker_frames(node, fwd=1.9, lat=0.0)                 # 이동 1.4 m — 측정을 의심할 크기
    node.run_mission(assigned('A'))
    assert node.status_log[-2:] == ['FAILED', 'READY']
    assert node.arrived_log == []
    assert node.navigator.moves() == []
    node.destroy_node()


@pytest.mark.parametrize('value', [0.0, -0.5, 0.2])
def test_bay_approach_refuses_unsafe_target_range(ros, value):
    """목표 거리가 없거나 하한(footprint 외접 + 여유)보다 가까우면 움직이지 않는다.

    0.0 을 '거리 보정 안 함' 으로 조용히 넘기면 로봇이 적재 위치에 못 간 채 미션이 성공으로
    끝난다. 0.2 는 footprint 외접 0.189 m 를 빼면 벽까지 1 cm 라 선반에 밀어 넣는 설정이다.
    """
    node = make_node(bay_align_enable=True, bay_marker_range_m=value, bay_align_min_range_m=0.35)
    stub_token(node, [True])
    fresh_marker(node)
    marker_frames(node, fwd=1.0, lat=0.0)
    node.run_mission(assigned('A'))
    assert node.status_log[-2:] == ['FAILED', 'READY']
    assert node.navigator.moves() == []
    node.destroy_node()


def test_marker_sample_restores_flipped_normal(ros):
    """법선이 뒤집힌 solvePnP 해가 와도 방위 오차는 0 이어야 한다 (두 해 문제)."""
    node = make_node()
    marker_frames(node, fwd=1.0, lat=0.0, flip_normal=True)
    sample = node.marker_sample()
    assert sample is not None
    assert sample.fwd == pytest.approx(1.0, abs=1e-6)
    assert sample.lat == pytest.approx(0.0, abs=1e-6)
    assert sample.yaw_error == pytest.approx(0.0, abs=1e-6)
    node.destroy_node()


def test_marker_yaw_is_rejected_when_it_disagrees_with_survey(ros):
    """마커 자세를 켜도 측량값과 게이트 이상 어긋나면 측량값으로 되돌아간다.

    인지가 '정사각 마커 두 해 문제로 흔들린다' 고 명시했다 — 뒤집힌 해를 그대로 믿고 돌면
    선반 쪽으로 크게 틀어진다.
    """
    node = make_node(bay_align_use_marker_yaw=True, bay_align_yaw_gate_deg=25.0)
    marker_frames(node, fwd=1.0, lat=0.0)
    sample = node.marker_sample()
    # 마커 기준 오차는 0 인데 측량 기준으로는 40° 틀어져 있다 → 게이트 밖이라 측량을 쓴다
    phi, source = node.align_yaw(sample, yaw_ref=math.radians(40), yaw_now=0.0)
    assert phi == pytest.approx(math.radians(40))
    assert '기각' in source
    # 8° 차이면 게이트 안 — 마커 값을 쓴다 (AMCL 방위 오차까지 지운다)
    phi, source = node.align_yaw(sample, yaw_ref=math.radians(8), yaw_now=0.0)
    assert phi == pytest.approx(0.0, abs=1e-6)
    assert source == '마커'
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


def test_initial_pose_sent_at_startup(ros):
    """초기자세는 미션이 아니라 기동 시 나간다 (I-035).

    미션을 받은 뒤에 보내면 Nav2 의 global_costmap 이 이미 활성화에 실패한 뒤라 늦다.
    """
    node = make_node()
    node.startup()
    assert ('initpose', 0.019, -0.795) in node.navigator.calls
    # 기동 시에는 로봇을 움직이지 않고, 미션이 없으므로 상태도 바꾸지 않는다
    assert 'dock' not in node.navigator.calls
    assert 'undock' not in node.navigator.calls
    assert node.status_log == []
    node.destroy_node()


def test_startup_skips_initial_pose_when_undocked(ros):
    """도크 밖이면 기동 시 초기자세를 보내지 않는다 — dock_pose 로 초기화하면 측위가 틀어진다."""
    node = make_node()
    node.navigator.is_docked = False
    node.startup()
    assert not [c for c in node.navigator.calls if c[0] == 'initpose']
    assert 'dock' not in node.navigator.calls      # 노드를 띄웠다고 로봇이 움직이면 안 된다
    assert node.status_log == []
    node.destroy_node()


def test_startup_unknown_dock_status_is_tolerated(ros):
    """dock_status 를 못 받아도 기동은 끝난다 — 예전에는 여기서 무한 대기였다 (가이드 D-2)."""
    node = make_node()
    node.docked_status = lambda timeout_s: None
    node.startup()
    assert not [c for c in node.navigator.calls if c[0] == 'initpose']
    assert node.status_log == []
    node.destroy_node()


def test_startup_bad_dock_pose_does_not_publish_failed(ros):
    """기동 경로에서는 FAILED 를 내지 않는다 — 포기할 미션이 없다 (I-024·I-026)."""
    node = make_node(dock_pose=[0.0, 0.0, 0.0])
    node.startup()
    assert node.status_log == []
    assert not [c for c in node.navigator.calls if c[0] == 'initpose']
    # 같은 검사가 미션 경로에서 다시 걸려 그때 FAILED 로 처리된다
    node.run_mission(assigned('A', 'M-startup'))
    assert node.status_log == ['FAILED', 'READY']
    node.destroy_node()


def test_nav2_not_ready_fails_fast(ros):
    """Nav2 가 active 가 아니면 T_lim 을 태우지 않고 곧장 포기한다 (I-035 ⓑ)."""
    node = make_node(t_lim_s=30.0, nav2_poll_max=2, nav2_poll_s=0.05)
    node.nav2_state = lambda: (False, 'bt_navigator 가 inactive 이다')
    started = node.now()
    node.run_mission(assigned('A', 'M-nav2'))
    assert node.status_log == ['FAILED', 'READY']
    assert 'undock' not in node.navigator.calls    # 준비 안 됐으면 언독하지 않는다
    assert node.arrived_log == []
    # 120 초를 태우지 않는다는 것이 이 검사의 요점이다
    assert node.now() - started < 5.0
    node.destroy_node()


def test_wait_nav2_without_mission_does_not_crash(ros):
    """t0 가 없는 경로에서도 경과 시간 로그가 터지지 않는다 (elapsed()).

    초기자세를 기동 시로 옮기면서 wait_nav2() 가 미션 밖에서 불릴 수 있게 됐다.
    self.now() - self.t0 를 그대로 두면 TypeError 다.
    """
    node = make_node(nav2_poll_max=1)
    node.nav2_state = lambda: (False, '서비스가 없다')
    assert node.t0 is None
    from mro_navigation.nodes.shuttle_mission_node import MissionAbort
    with pytest.raises(MissionAbort):
        node.wait_nav2()
    node.destroy_node()


def test_docked_status_does_not_return_stale_cache(ros, monkeypatch):
    """docked_status() 는 캐시가 아니라 새로 받아온 값을 준다 (PR #44 PM 지적).

    `is_docked` 는 구독 콜백으로만 바뀌고 dock()·undock() 은 건드리지 않는다. 조건부로 spin 을
    건너뛰면 묵은 값이 나온다. 특히 startup() 과 첫 미션 사이에는 navigator 를 spin 하는 곳이
    하나도 없어, 그 사이 사람이 로봇을 도크에서 내리면 캐시된 True 를 그대로 믿게 된다.
    """
    node = make_node()
    node.startup()
    assert node.navigator.is_docked is True          # 기동 시 캐시된 값

    real_spin = rclpy.spin_once

    def spin_once(target, timeout_sec=None):
        # navigator 를 spin 할 때 '도크에서 내려졌다' 는 콜백이 도착한 것으로 만든다
        if target is node.navigator:
            target.deliver_dock_status(False)
            return
        return real_spin(target, timeout_sec=timeout_sec)

    monkeypatch.setattr(rclpy, 'spin_once', spin_once)
    assert node.docked_status(1.0) is False          # 캐시(True)를 돌려주면 실패한다
    node.destroy_node()


def test_init_pose_docks_again_when_robot_left_the_dock(ros, monkeypatch):
    """도크 밖이면 init_pose() 가 dock() 을 먼저 부른다 — 캐시를 믿으면 건너뛴다 (I-013).

    건너뛰면 대기공간에 선 채로 dock_pose 를 초기자세로 보내 측위가 통째로 어긋난다.
    """
    node = make_node()
    node.startup()
    node.navigator.calls.clear()

    real_spin = rclpy.spin_once

    def spin_once(target, timeout_sec=None):
        if target is node.navigator:
            target.deliver_dock_status(False)
            return
        return real_spin(target, timeout_sec=timeout_sec)

    monkeypatch.setattr(rclpy, 'spin_once', spin_once)
    node.init_pose()
    assert 'dock' in node.navigator.calls                       # 먼저 도킹하고
    assert ('initpose', 0.019, -0.795) in node.navigator.calls  # 그 다음 초기자세
    assert node.navigator.calls.index('dock') < [
        i for i, c in enumerate(node.navigator.calls) if c[0] == 'initpose'][0]
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
    # 목표 2건: ① 베이 ② 대기공간. 접근은 상대 이동이라 goToPose 를 안 쓴다.
    #   획득은 ①까지 끝난 뒤(= 베이를 아직 안 떠남), 반납은 ②까지 끝난 뒤여야 한다
    assert node.token_marks == [('acquire', 1), ('release', 2)]
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
