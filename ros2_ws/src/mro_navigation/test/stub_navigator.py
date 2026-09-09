"""`TurtleBot4Navigator` 대역 — 추종·복귀 시험이 함께 쓴다.

`turtlebot4_navigation` 은 이 저장소 밖 패키지(제어 PC 에만 있다)라 시험에서는 갈아끼운다.
`rclpy` 와 `mro_interfaces` 는 실물을 쓴다.

**실물과 같이 rclpy Node 다.** 노드가 `rclpy.spin_once(navigator)` 를 부를 수 있어서다.
**`is_docked` 는 실물과 같이 구독 콜백으로만 바뀐다** — `dock()`·`undock()` 은 건드리지 않는다.
대역이 여기서 값을 직접 바꾸면 '묵은 값을 돌려주는' 결함을 시험이 못 잡는다 (PR #44 PM 지적).
콜백 도착은 `deliver_dock_status()` 로 흉내낸다.

**셔틀 시험의 대역을 포함하는 상위집합이다.** `sys.modules['turtlebot4_navigation']` 은 한
프로세스에 하나뿐이라, 세 시험을 한 pytest 세션에서 돌리면 먼저 설치한 쪽 대역이 나머지에도
쓰인다. 셔틀 시험(`test_shuttle_mission_fsm.py`)이 자기 파일 안에 같은 이름의 대역을 두고
있으므로, 이 대역은 그쪽이 기대하는 것(`Node` 상속·`setInitialPose`·`calls` 기록 형식)을
모두 갖춰 어느 순서로 돌아도 셋이 함께 통과하게 한다.
셔틀 시험이 이 파일을 쓰도록 옮기는 것은 그쪽 변경이 필요하니 별건으로 남긴다.
"""
import math
import sys
import types

from rclpy.node import Node


class StubNavigator(Node):
    """호출을 기록하고 목표를 `nav_ticks` 뒤 완료로 만든다.

    **실물과 같이 rclpy Node 다** — 노드가 `rclpy.spin_once(navigator)` 를 부를 수 있어서다.
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
        """도킹. 실물과 같이 is_docked 를 직접 바꾸지 않는다."""
        self.calls.append('dock')

    def dock_send_goal(self):
        self.dock()

    def isDockComplete(self):
        return True

    def undock_send_goal(self):
        self.undock()

    def isUndockComplete(self):
        return True

    def undock(self):
        """언독. **복귀·추종은 부르지 않아야 한다** (계약 2절 후진 금지) — 기록만 남긴다."""
        self.calls.append('undock')

    def setInitialPose(self, pose):  # noqa: N802
        """초기자세 전송을 기록한다 (셔틀이 기동 시 부른다 — I-035)."""
        self.calls.append(
            ('initpose', round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)))

    def spin(self, spin_dist=0.0):
        """제자리 회전. 추종이 `USE_NAV2_SPIN` 일 때만 쓴다.

        **goToPose 와 같이 nav_ticks 뒤 완료로 만든다** — 실물에서도 회전은 액션이라
        여러 주기 걸린다. 세우지 않으면 첫 물음에 완료로 답해서 '도는 동안' 을 보는
        시험이 성립하지 않는다 (9/9, 회전 중 조기 정지 시험).
        """
        self.calls.append(('spin', round(spin_dist, 3)))
        self._ticks = self.nav_ticks

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

    def counts(self, name):
        """이름이 같은 호출 수 — `undock` 이 0 인지 세는 데 쓴다."""
        return sum(1 for c in self.calls if c == name)


def install():
    """`turtlebot4_navigation` 을 대역으로 갈아끼운다. 노드 import 전에 불러야 한다."""
    if 'turtlebot4_navigation' not in sys.modules:
        pkg = types.ModuleType('turtlebot4_navigation')
        mod = types.ModuleType('turtlebot4_navigation.turtlebot4_navigator')
        mod.TurtleBot4Navigator = StubNavigator
        pkg.turtlebot4_navigator = mod
        sys.modules['turtlebot4_navigation'] = pkg
        sys.modules['turtlebot4_navigation.turtlebot4_navigator'] = mod
