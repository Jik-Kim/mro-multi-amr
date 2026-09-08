#!/usr/bin/env python3
"""[이관] Kn1_mini/nav/goto_goal_node.py → mro_navigation/nodes/shuttle_mission_node.py.

역할 변경: '웹캠 차 좌표 앞 1.5 m 로 이동' → 'S4 셔틀 미션'.
  **기동 시** 도크 초기자세 → (미션 대기) → mission/assigned 수신 → Nav2 준비 → undock
  → 베이 waypoint → ArUco 로 도착 확정 → 창고 안쪽 전진 → 적재 체류 → P1 토큰 → 대기공간 → WAITING

기동 순서가 중요하다 (I-035) — **이 노드를 Nav2(T3) 보다 먼저 띄운다.** Nav2 의 global_costmap 은
  기동할 때 map→base_link TF 를 요구하고, AMCL 은 초기자세를 받아야 map→odom 을 낸다. 초기자세가
  늦으면 costmap 활성화가 실패하는데 lifecycle_manager 는 **나중에 TF 가 생겨도 재시도하지 않는다**
  (9/7 실측). 그래서 초기자세를 미션이 아니라 기동 시에 보낸다.

유지한 골격: 도크 초기자세(dock_pose), Nav2 준비 폴링, undock, goToPose 폴링,
  그리고 **콜백 안에서 navigator 를 부르지 않는다** — 콜백은 값만 저장하고 메인 루프가 실행한다.
폐기한 것: webcam/detected 트리거, webcam/car_map_point 목표, goto_goal/arrived
  (계약 4절 '쓰지 않는 옛 토픽'), standoff_goal 기반 목표 선택.

입력  mission/assigned(Mission) · aruco/id(Int32) · aruco/distance(Float32) · safety/estop(Bool)
      TF map→base_link · arena.yaml(베이 waypoint·aruco_id, 대기공간) · robotN.yaml(dock_pose 등)
출력  shuttle/status(String: READY/SHUTTLE/AT_BAY/WAITING/FAILED) · shuttle/arrived(Bool)
협조  /fleet/zone_token 의 P1 토큰 — 베이를 떠나기 전 획득, 대기공간 도착 후 반납 (계약 3절)
      **순서 기반 판정 (방법 B, 9/7 팀 회의).** 경로가 베이 → P1 → 대기공간으로 고정돼 있으므로
      좌표를 매 tick 폴리곤과 대조하지 않는다. 획득 시점(베이)과 반납 시점(대기공간)이 둘 다
      폴리곤 밖이면 '진입 전 획득·이탈 후 반납' 이 성립한다 — 그 전제는 arena.yaml 좌표로
      test_shuttle_mission_fsm.py 가 확인한다. 좌표 감시(방법 A)는 계산이 복잡하고 경로가
      폴리곤 가장자리를 스치면 진입을 놓쳐 버그가 나기 쉬워 폐기했다

판정 관련 규칙 (어기면 KPI 가 조용히 틀어진다)
  · shuttle/status = WAITING 은 **대기공간 도착이 확정된 시점에 한 번만**. fleet_manager 가
    이것으로 mark_waiting() 을 불러 AC-08 배송 성공/실패와 K-05 리드타임을 확정한다 (I-024).
    베이에서 P1 토큰을 기다리는 동안에는 AT_BAY 를 유지한다 — 로봇이 아직 창고에 있기 때문이다.
  · FAILED 는 **포기할 때만** 낸다. 재시도 중에는 직전 상태를 유지한다 (I-026).
  · T_lim 초과로 포기한 미션은 폐기한다 — 그 뒤 늦게 도착해도 WAITING·arrived 를 내지 않는다
    (계약 7절, 이중 계상 금지). 늦은 도착은 관제가 arrived_late 로 따로 남긴다.

TODO([주행]): ① 파라미터 7개와 arena_yaml 경로는 PM 이 robotN.yaml·robot.launch.py 로 옮겼다
  (PR #36 후속, 9/8). **I-035 로 추가한 nav2_poll_max·dock_status_wait_s 두 개는 아직 코드 기본값뿐이라
  robotN.yaml 이관을 PM 에 요청해야 한다** — 값의 단일 출처는 params 다 (계약 5절). 런타임 share 조회는
  런치가 안 넘겨줄 때의 대체값으로만 남아 있다 (의존을 선언하면 순환이라 막을 수 없다)
② follow_target_node.py 가 아직 옛 goto_goal/arrived 를 구독한다. shuttle/arrived 로 바꾸지 않으면
  S5 합류가 시작되지 않는다 (docs/todo.md 에 등록)
③ ArUco '정면 ±30°'(계약 6절)는 여전히 아무도 검사하지 않는다. aruco_detector 가 PR #30 으로
  solvePnP 를 넣어 기운각을 **계산은 하지만 로그로만 남기고 발행하지 않는다** — 게이팅이 필요하면
  인지에 각도 발행을 요청할 것. 거리는 그쪽이 이미 1.5 m 에서 자른다(`max_distance_m`, 넘으면 id=-1)
  므로 아래 aruco_max_dist_m 는 이중 방어다
④ 대기공간 정차 중에도 로봇 몸통이 P1 폴리곤에 살짝 걸친다 — robot3 정차점이 폴리곤에서
  0.20 m 인데 반지름은 0.22 m 다. 반납 시점이 계약상 '완전 이탈' 인지 실주행에서 확인할 것
  (2대 시험 때 다른 로봇이 P1 을 지날 수 있는지가 실제 판단 기준)
⑤ navigator.undock()/dock() 은 블로킹이라 그 동안 이 노드의 콜백·T_lim 검사가 멈춘다
  (수 초 규모라 그대로 둔다)
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.srv import GetState
from mro_interfaces.msg import Mission
from mro_interfaces.srv import ZoneToken
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, Int32, String
from tf2_ros import Buffer, TransformListener
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
import yaml

# 네임스페이스 대체값은 두지 않는다 (9/8) — ns 가 arena.yaml 의 로봇별 좌표 조회 키라
#   틀리면 남의 대기공간·도크로 간다. 아래에서 없으면 죽인다
MISSION_TOPIC = 'mission/assigned'
STATUS_TOPIC = 'shuttle/status'
ARRIVED_TOPIC = 'shuttle/arrived'
ARUCO_ID_TOPIC = 'aruco/id'
ARUCO_DIST_TOPIC = 'aruco/distance'
ESTOP_TOPIC = 'safety/estop'
TOKEN_SRV = '/fleet/zone_token'   # 관제 PC 의 zone_semaphore. 전역 이름
ZONE_P1 = 'P1'                    # arena.yaml semaphore_zones · ZoneToken.srv 와 ID 일치
MAP_FRAME = 'map'
BASE_FRAME = 'base_link'
TICK_S = 0.1                      # spin_once 주기


class MissionAbort(Exception):
    """미션을 포기하는 유일한 경로. 사유를 담아 FAILED 로 빠진다."""


class ShuttleMission(Node):
    def __init__(self):
        super().__init__('shuttle_mission')
        self.ns = self.get_namespace().rstrip('/')
        if not self.ns:
            raise RuntimeError(
                'shuttle_mission: 네임스페이스 없이는 뜰 수 없다 — 로봇 노드는 ns 로 자기 로봇을 정한다.\n'
                '  ros2 launch mro_bringup robot.launch.py namespace:=/robotN\n'
                '  (단독 실행이면 --ros-args -r __ns:=/robotN)\n'
                'ns 가 arena.yaml 의 waiting_zone·dock_zone 조회 키다 — 틀리면 남의 자리로 간다.')
        self.robot = self.ns.strip('/')          # arena.yaml 의 키는 'robot3' (슬래시 없음)
        self.navigator = TurtleBot4Navigator(namespace=self.ns)

        # 파라미터 — 값의 단일 출처는 mro_bringup/params (계약 5절). 여기 기본값은 params 가
        #   아직 안 넘어올 때의 대체값이며, 코드가 값을 소유하지 않는다.
        # arena.yaml 은 mro_bringup 소유지만 **의존을 선언할 수 없다** — mro_bringup 이 이미
        #   mro_navigation 을 의존해서(런치가 노드를 띄운다) 선언하면 colcon 이 순환으로 막는다.
        #   그래서 런타임 조회로만 기본값을 잡고, 못 찾으면 arena_yaml 파라미터로 받는다.
        default_arena = ''
        try:
            default_arena = os.path.join(
                get_package_share_directory('mro_bringup'), 'params', 'arena.yaml')
        except Exception as e:
            self.get_logger().warn(f'mro_bringup 을 찾지 못했다 — arena_yaml 을 직접 넘길 것: {e}')
        # 기본값 (0,0,0) 은 '주입 안 됨' 을 뜻한다. 빈 리스트는 BYTE_ARRAY 로 추론되어
        #   robotN.yaml 의 실제 값을 되레 막으므로 쓸 수 없다 (9/7 확인).
        self.declare_parameter('dock_pose', [0.0, 0.0, 0.0])   # robotN.yaml (필수)
        self.declare_parameter('bay_dwell_s', 5.0)             # robotN.yaml
        self.declare_parameter('t_lim_s', 120.0)               # robotN.yaml
        self.declare_parameter('arena_yaml', default_arena)
        self.declare_parameter('bay_advance_m', 0.3)
        self.declare_parameter('aruco_max_dist_m', 1.5)        # 계약 6절 검출 조건
        self.declare_parameter('aruco_valid_s', 1.0)
        self.declare_parameter('aruco_confirm_s', 3.0)
        self.declare_parameter('aruco_retry', 1)
        self.declare_parameter('token_retry_s', 2.0)           # 계약 3절 '거절되면 2초 후 재요청'
        self.declare_parameter('nav2_poll_s', 5.0)
        # I-035 로 추가 (9/8) — robotN.yaml 이관은 PM 요청 필요 (위 TODO ①)
        self.declare_parameter('nav2_poll_max', 3)          # 이 횟수 안에 active 가 아니면 포기
        self.declare_parameter('dock_status_wait_s', 5.0)   # dock_status 를 기다리는 상한

        self.arena = self._load_arena(self.p('arena_yaml'))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 콜백이 채우는 값들 — 실행은 전부 메인 루프에서 한다
        self.pending = None      # 채택 대기 중인 Mission
        self.mission = None      # 수행 중인 Mission (None 이면 IDLE)
        self.t0 = None           # T_lim 기준 ROS 시각
        self.aruco_id = None     # (id, 수신 시각)
        self.aruco_dist = None   # (m, 수신 시각)
        self.estop = False
        self.last_status = None
        self.token_held = False

        self.create_subscription(Mission, MISSION_TOPIC, self.mission_cb, 10)
        self.create_subscription(Int32, ARUCO_ID_TOPIC, self.aruco_id_cb, 1)
        self.create_subscription(Float32, ARUCO_DIST_TOPIC, self.aruco_dist_cb, 1)
        self.create_subscription(Bool, ESTOP_TOPIC, self.estop_cb, 1)
        self.pub_status = self.create_publisher(String, STATUS_TOPIC, 10)
        self.pub_arrived = self.create_publisher(Bool, ARRIVED_TOPIC, 1)
        self.cli_token = self.create_client(ZoneToken, TOKEN_SRV)
        # 클라이언트는 한 번만 만든다 — 폴링마다 만들면 5 초마다 하나씩 쌓인다
        self.cli_state = self.create_client(GetState, f'{self.ns}/bt_navigator/get_state')

        self.set_status('READY')
        self.get_logger().info(f'대기: {self.ns}/{MISSION_TOPIC} 에 Mission(ASSIGNED) 오면 출발')

    # ── 파라미터·설정 ────────────────────────────────────────────────
    def p(self, name):
        return self.get_parameter(name).value

    def _load_arena(self, path):
        """arena.yaml 은 ROS 파라미터 형식이 아닌 데이터 yaml 이라 파일로 직접 읽는다."""
        if not path:
            self.get_logger().error(
                'arena_yaml 경로가 비었다 — 미션을 받아도 수행할 수 없다. '
                '런치나 --ros-args -p arena_yaml:=<경로> 로 넘길 것')
            return None
        try:
            with open(path, encoding='utf-8') as f:
                return yaml.safe_load(f)['arena']
        except Exception as e:
            self.get_logger().error(
                f'arena.yaml 적재 실패 — 미션을 받아도 수행할 수 없다 ({path}): {e}')
            return None

    # ── 콜백: 값만 저장한다 (navigator 호출 금지) ────────────────────
    def mission_cb(self, m):
        if m.state != Mission.ASSIGNED:
            return                                  # 그 밖의 전이는 관제 내부용이다
        if m.robot_ns not in ('', self.ns):
            return                                  # 단독 시험 시 오수신 방지
        if self.mission is not None or self.pending is not None:
            # 로봇당 미션은 1건이다. 큐를 두면 T_lim 시계를 언제부터 재야 할지가 모호해진다
            self.get_logger().warn(f'이미 미션이 있다 — 새 배정을 버린다: {m.mission_id}')
            return
        self.pending = m                            # 실행은 메인 루프에서

    def aruco_id_cb(self, msg):
        self.aruco_id = (int(msg.data), self.now())

    def aruco_dist_cb(self, msg):
        self.aruco_dist = (float(msg.data), self.now())

    def estop_cb(self, msg):
        self.estop = bool(msg.data)

    # ── 공용 헬퍼 ────────────────────────────────────────────────────
    def now(self):
        """ROS 시각. 로그·CSV·DB 를 합쳐야 하므로 벽시계를 섞지 않는다 (계약 2절)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def set_status(self, s):
        if s == self.last_status:
            return                                  # 같은 값을 연속으로 내지 않는다
        self.last_status = s
        self.pub_status.publish(String(data=s))
        self.get_logger().info(f'[{s}]')

    def over_tlim(self):
        return self.t0 is not None and self.now() - self.t0 > self.p('t_lim_s')

    def elapsed(self):
        """미션 시작 이후 경과 초. 미션 밖(기동 시)에는 t0 가 없으므로 0.0 을 준다."""
        return 0.0 if self.t0 is None else self.now() - self.t0

    def docked_status(self, timeout_s):
        """도킹 여부를 돌려준다. 시간 안에 알 수 없으면 None.

        navigator.getDockedStatus() 는 dock_status 가 올 때까지 **무한히** 기다린다
        (turtlebot4_navigator.py: `while self.is_docked is None`). 로봇이 안 떠 있으면 거기서
        영영 멈추고, 미션 경로에서는 T_lim 조차 돌지 않아 FAILED 도 안 나온다 (가이드 D-2).
        구독은 navigator 의 것을 그대로 쓰되 기다리는 시간에 상한을 둔다.

        **먼저 무조건 한 번 spin 한다** — 원본과 같은 순서다. `is_docked` 는 `_dockCallback`
        으로만 바뀌고 `dock()`·`undock()` 은 건드리지 않으므로(turtlebot4_navigator.py 확인),
        조건부로 건너뛰면 묵은 값을 돌려준다. 특히 `startup()` 과 첫 미션 사이에는 navigator 를
        spin 하는 곳이 하나도 없어, 그 사이에 사람이 로봇을 도크에서 내리면 캐시된 True 를 믿고
        `dock()` 을 건너뛴 채 도크 밖에서 dock_pose 를 초기자세로 보내게 된다 (I-013, PR #44 PM 지적).
        """
        deadline = self.now() + float(timeout_s)
        rclpy.spin_once(self.navigator, timeout_sec=0.1)   # 원본과 같게: 먼저 한 번 받아온다
        while self.navigator.is_docked is None and self.now() < deadline:
            rclpy.spin_once(self.navigator, timeout_sec=0.1)
        return self.navigator.is_docked

    def wait(self, cond, timeout=None, on_tick=None):
        """유일한 대기 수단. 모든 대기가 여기를 지나므로 T_lim 검사가 한 곳에 모인다."""
        t_start = self.now()
        while rclpy.ok():
            if cond():
                return True
            rclpy.spin_once(self, timeout_sec=TICK_S)   # 콜백은 대기 중에도 살아 있다
            if on_tick:
                on_tick()
            if self.over_tlim():
                raise MissionAbort(f"T_lim {self.p('t_lim_s')} s 초과")
            if timeout is not None and self.now() - t_start > timeout:
                return False
        raise MissionAbort('노드 종료')

    def tf_pose(self, warn=True):
        """(x, y, yaw) 또는 None. yaw 식은 계약 2절 FleetStatus.yaw 와 같다.

        warn=False 는 도착 로그용 — 좌표를 못 얻어도 미션에 지장이 없으니 경고를 남기지 않는다.
        """
        try:
            t = self.tf_buffer.lookup_transform(MAP_FRAME, BASE_FRAME, Time())
        except Exception as e:
            if warn:
                self.get_logger().warn(f'TF: {e}')
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def pose_of(self, x, y, yaw):
        """회전 인자는 도 단위다 — getPoseStamped 규약이라 라디안을 넘기면 조용히 틀어진다."""
        return self.navigator.getPoseStamped([float(x), float(y)], math.degrees(yaw))

    def goto(self, pose, label):
        """목표 1건. 콜백 밖에서만 불린다."""
        if self.estop:
            self.get_logger().warn(f'estop 중 — 해제까지 {label} 보류')
            self.wait(lambda: not self.estop)        # 계약 4절: estop 동안 주행 명령 금지
        gx, gy = pose.pose.position.x, pose.pose.position.y
        self.get_logger().info(f'{label} → ({gx:.2f}, {gy:.2f}) 이동 시작')
        if not self.navigator.goToPose(pose):
            raise MissionAbort(f'{label} 목표 거부됨')
        self.wait(self.navigator.isTaskComplete)
        result = self.navigator.getResult().name
        if result != 'SUCCEEDED':
            raise MissionAbort(f'{label} 이동 실패: {result}')
        # 실제 정차 위치를 같이 남긴다 — Nav2 의 xy_goal_tolerance(0.25 m) 때문에
        #   목표와 실제가 그만큼 벌어질 수 있어서, 전진 거리를 로그로 확인해야 한다
        at = self.tf_pose(warn=False)
        if at is None:
            self.get_logger().info(f'{label} 도착')
        else:
            self.get_logger().info(
                f'{label} 도착 — 목표 ({gx:.2f}, {gy:.2f}) / 실제 ({at[0]:.2f}, {at[1]:.2f}), '
                f'오차 {math.hypot(at[0] - gx, at[1] - gy):.2f} m')

    def token(self, acquire):
        """P1 토큰 획득/반납. 서비스가 없으면 (관제 미기동) 통과시켜 단독 시험을 막지 않는다."""
        if not self.cli_token.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('zone_token 서비스 없음 — 단독 모드로 진행')
            return True
        req = ZoneToken.Request()
        req.robot_ns, req.zone_id, req.acquire = self.ns, ZONE_P1, acquire
        fut = self.cli_token.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        res = fut.result()
        if res is None:
            self.get_logger().warn('zone_token 응답 없음')
            return False
        if acquire and not res.granted and res.holder:
            self.get_logger().info(f'P1 점유 중 (holder={res.holder})')
        return bool(res.granted)

    def release_token(self, reason):
        """성공·실패와 무관하게 반드시 반납한다 (계약 3절). 두 번 불려도 안전하다."""
        if not self.token_held:
            return
        self.token(False)
        self.token_held = False
        self.get_logger().info(f'P1 반납 ({reason})')

    def aruco_ok(self, expect_id):
        """마커 ID 와 거리는 헤더가 없어 짝을 맞출 수 없다 → 각자 수신 시각으로 신선도를 본다."""
        if self.aruco_id is None or self.aruco_dist is None:
            return False
        valid = self.p('aruco_valid_s')
        now = self.now()
        if now - self.aruco_id[1] > valid or now - self.aruco_dist[1] > valid:
            return False
        if self.aruco_id[0] == -1:                   # 미검출
            return False
        return (self.aruco_id[0] == expect_id
                and self.aruco_dist[0] <= self.p('aruco_max_dist_m'))

    def nav2_state(self):
        """bt_navigator 의 준비 여부를 (준비됨, 사유) 로 돌려준다.

        사유까지 남기는 이유 — '미준비' 한 줄만으로는 서비스가 아예 없는 것인지, 응답이
        없는 것인지, 라이프사이클이 active 가 아닌 것인지 구분되지 않아 원인을 짚는 데
        시간이 걸린다 (9/7 실주행에서 120 초를 그렇게 썼다).
        """
        if not self.cli_state.wait_for_service(timeout_sec=2.0):
            return False, f'{self.ns}/bt_navigator/get_state 서비스가 없다 — Nav2 미기동'
        fut = self.cli_state.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        res = fut.result()
        if res is None:
            return False, ('get_state 가 응답하지 않는다 — 같은 네임스페이스에 Nav2 가 '
                           '중복으로 떠 있는지 확인할 것 (ros2 node list)')
        label = res.current_state.label
        if label != 'active':
            return False, (f"bt_navigator 가 '{label}' 이다 — 라이프사이클 활성화가 끝나지 "
                           '않았다. lifecycle_manager 로그를 볼 것')
        return True, label

    # ── 미션 시퀀스 (메인 루프에서만 호출) ───────────────────────────
    def run_mission(self, m):
        self.mission = m
        self.t0 = self.now()
        self.token_held = False
        try:
            bay, waiting = self.mission_targets(m)
            self.init_pose()
            self.wait_nav2()
            self.get_logger().info('언독')
            self.navigator.undock()
            self.set_status('SHUTTLE')

            self.goto(self.pose_of(*bay['waypoint']), f'베이 {m.bay}')
            self.set_status('AT_BAY')                # Mission.msg: 'ArUco 확인, 체류 중'
            self.confirm_aruco(bay)
            self.advance_into_bay(bay)
            self.dwell()
            self.acquire_p1()

            self.set_status('SHUTTLE')
            self.goto(self.pose_of(*waiting), '대기공간')
            # 방법 B — 대기공간 도착이 곧 'P1 을 빠져나왔다' 는 뜻이다 (경로가 고정돼 있다)
            self.release_token('대기공간 도착 — P1 통과 완료')

            self.set_status('WAITING')               # AC-08 판정 기준점, 미션당 한 번
            self.pub_arrived.publish(Bool(data=True))
            self.get_logger().info(f'대기공간 도착 — 소요 {self.now() - self.t0:.1f} s')
        except MissionAbort as e:
            self.navigator.cancelTask()
            self.set_status('FAILED')                # 포기할 때만 (I-024·I-026)
            self.get_logger().error(f'미션 {m.mission_id} 포기: {e}')
        finally:
            self.release_token('미션 종료')
            self.mission = None
            self.t0 = None                           # 폐기 — 늦은 도착을 계상하지 않는다 (계약 7절)
            self.aruco_id = self.aruco_dist = None
            self.set_status('READY')

    def mission_targets(self, m):
        """요청 베이와 자기 대기공간 좌표를 arena.yaml 에서 찾는다."""
        if self.arena is None:
            raise MissionAbort('arena.yaml 미적재')
        bay = (self.arena.get('bays') or {}).get(m.bay)
        if not bay or len(bay.get('waypoint', [])) != 3:
            raise MissionAbort(f'arena.yaml 에 베이 {m.bay} waypoint 가 없다')
        waiting = (self.arena.get('waiting_zone') or {}).get(self.robot)
        if not waiting or len(waiting) != 3:
            raise MissionAbort(f'arena.yaml waiting_zone 에 {self.robot} 이 없다')
        self.get_logger().info(
            f"미션 {m.mission_id} · 베이 {m.bay} · 마커 {bay['aruco_id']} · 작업자 {m.worker_id}")
        return bay, waiting

    def dock_pose(self):
        """robotN.yaml 의 dock_pose 를 검증해 [x, y, yaw] 로 돌려준다.

        각 로봇이 (0,0) 으로 초기화하면 두 map 프레임의 원점이 물리적으로 다른 점이 되어
        세마포어 폴리곤·waypoint·FleetStatus 가 전부 어긋난다 — 1대로 시험하면 안 드러난다 (I-013).
        """
        v = [float(a) for a in (self.p('dock_pose') or [])]
        if len(v) != 3 or all(a == 0.0 for a in v):
            # (0,0,0) 은 파라미터가 안 들어왔다는 뜻으로 본다. 실제 도크는 robot3 [0.019,-0.795,0.133],
            #   robot8 [-0.023,0.034,-0.052] 라 원점과 겹치지 않는다. 원점으로 초기화하는 것이
            #   바로 두 map 프레임을 어긋나게 하는 실수다 (I-013) — 조용히 진행하지 않고 포기한다.
            raise MissionAbort(
                f'dock_pose 가 유효하지 않다 ({v}) — robotN.yaml 의 dock_pose 를 확인할 것. '
                '이대로 초기화하면 두 로봇의 맵 좌표계가 어긋난다 (I-013)')
        return v

    def send_initial_pose(self, v):
        """검증된 dock_pose 를 AMCL 에 보낸다. 이 시점부터 map→odom TF 가 나온다."""
        x, y, yaw = float(v[0]), float(v[1]), float(v[2])
        self.navigator.setInitialPose(self.pose_of(x, y, yaw))
        self.get_logger().info(
            f'초기자세 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°) 전송 — 자신의 도크 map 좌표')

    def startup(self):
        """미션을 받기 전에 초기자세를 보낸다 — 메인 루프에서만 부른다 (I-035).

        왜 여기서 보내나 — Nav2 의 global_costmap 은 기동할 때 map→base_link TF 를 요구하고,
        AMCL 은 초기자세를 받아야 map→odom 을 낸다. 초기자세가 늦으면 costmap 활성화가 실패하는데
        lifecycle_manager 는 나중에 TF 가 생겨도 재시도하지 않는다 (9/7 robot3 실측: initialpose
        발행 8 초 뒤에도 global_costmap 은 inactive). 미션을 받은 뒤에 보내면 이미 늦어
        T_lim 120 초를 통째로 태운다.

        기동 시에는 로봇을 움직이지 않는다 — 도킹돼 있지 않으면 초기자세를 보내지 않고 미션
        경로에 맡긴다. 도크 밖에서 dock_pose 로 초기화하면 측위가 대놓고 틀어지기 때문이다.
        여기서는 상태를 바꾸지 않는다 — 미션이 없으므로 FAILED 를 낼 수 없다 (I-024·I-026).
        """
        try:
            v = self.dock_pose()
        except MissionAbort as e:
            # 같은 검사가 미션 경로에서 다시 걸려 그때 FAILED 로 처리된다
            self.get_logger().error(f'기동 시 초기자세 보류: {e}')
            return
        wait_s = self.p('dock_status_wait_s')
        docked = self.docked_status(wait_s)
        if docked is None:
            self.get_logger().warn(
                f'기동 시 초기자세 보류 — dock_status 를 {wait_s} s 안에 받지 못했다. '
                '로봇 전원·네임스페이스·디스커버리를 확인할 것 (가이드 D-2)')
            return
        if not docked:
            self.get_logger().warn(
                '기동 시 초기자세 보류 — 도크에 물려 있지 않다. 도크 밖에서 dock_pose 로 '
                '초기화하면 측위가 틀어진다. 로봇을 도크에 올리고 다시 띄울 것')
            return
        self.send_initial_pose(v)
        self.get_logger().info(
            'Nav2 를 아직 띄우지 않았다면 지금 띄울 것 — 이 TF 로 global_costmap 이 활성화된다 (I-035)')

    def init_pose(self):
        """미션 경로의 초기자세. 기동 시 이후 로봇이 옮겨졌을 수 있어 다시 잡는다."""
        v = self.dock_pose()
        wait_s = self.p('dock_status_wait_s')
        docked = self.docked_status(wait_s)
        if docked is None:
            # 예전에는 여기서 무한 대기라 T_lim 도 안 돌고 FAILED 도 안 나왔다 (가이드 D-2)
            raise MissionAbort(
                f'dock_status 를 {wait_s} s 안에 받지 못했다 — '
                '로봇 전원·네임스페이스·디스커버리를 확인할 것')
        if not docked:
            self.get_logger().info('초기자세 설정을 위해 도킹')
            self.navigator.dock()
        self.send_initial_pose(v)

    def wait_nav2(self):
        """Nav2 가 active 가 될 때까지 **정해진 횟수만** 확인한다 (I-035).

        무한히 기다리면 T_lim 120 초를 통째로 태운다. 그런데 초기자세 없이 뜬 Nav2 는 스스로
        활성화를 재시도하지 않으므로 더 기다린다고 낫지 않는다 — 사유와 함께 빨리 포기해야
        '120 초 뒤 실패' 와 '10 초 만의 Nav2 미기동' 이 회차 기록에서 구분된다.
        경과 시간은 elapsed() 로 찍는다 — t0 가 없는 경로에서도 안전해야 한다.
        """
        tries = max(1, int(self.p('nav2_poll_max')))
        reason = ''
        for attempt in range(1, tries + 1):
            ready, reason = self.nav2_state()
            if ready:
                self.get_logger().info('Nav2 준비됨 (bt_navigator active)')
                return
            self.get_logger().warn(
                f'Nav2 미준비 {attempt}/{tries} ({self.elapsed():.0f} s 경과 / '
                f"T_lim {self.p('t_lim_s'):.0f} s): {reason}")
            if attempt < tries:
                self.wait(lambda: False, timeout=self.p('nav2_poll_s'))
        raise MissionAbort(f'Nav2 미준비 — {tries}회 확인 동안 active 가 아니다: {reason}')

    def confirm_aruco(self, bay):
        """베이 도착 확정. 마커를 확인하지 못하면 전진·체류하지 않고 포기한다 (FR-08 오도착 0건)."""
        retry = int(self.p('aruco_retry'))
        for attempt in range(retry + 1):
            if self.wait(lambda: self.aruco_ok(bay['aruco_id']),
                         timeout=self.p('aruco_confirm_s')):
                self.get_logger().info(
                    f"마커 {bay['aruco_id']} 확인 (거리 {self.aruco_dist[0]:.2f} m)")
                return
            seen = self.aruco_id[0] if self.aruco_id else None
            if attempt == retry:
                raise MissionAbort(f"ArUco 불일치 — 오도착 (기대 {bay['aruco_id']}, 관측 {seen})")
            self.get_logger().warn(
                f'마커 미확인 {attempt + 1}/{retry + 1} (관측 {seen}) — 정차 자세 재정렬')
            self.goto(self.pose_of(*bay['waypoint']), '베이 재정렬')  # 아직 포기 아님 → AT_BAY 유지

    def advance_into_bay(self, bay):
        """확인된 자리에서 창고 안쪽(로봇 전방)으로 bay_advance_m 전진."""
        # 주의 — Nav2 xy_goal_tolerance 가 0.25 m 로 고정이라(I-029, 낮추면 목표 근처에서 진동한다)
        #   0.3 m 목표를 줘도 실제 이동은 0.05~0.3 m 사이다. 도착 로그의 실제 좌표로 확인할 것
        d = self.p('bay_advance_m')
        pose = self.tf_pose()
        if pose is None:
            self.get_logger().warn('TF 없음 — 베이 waypoint 기준으로 전진 목표를 계산한다')
            pose = tuple(bay['waypoint'])
        x, y, yaw = pose
        self.goto(self.pose_of(x + d * math.cos(yaw), y + d * math.sin(yaw), yaw),
                  f'창고 안쪽 {d} m 전진')

    def dwell(self):
        dwell_s = self.p('bay_dwell_s')
        self.get_logger().info(f'적재 체류 {dwell_s} s')
        self.wait(lambda: False, timeout=dwell_s)

    def acquire_p1(self):
        """베이를 떠나기 전에 획득한다 — 여기서부터 대기공간까지가 P1 을 지나는 구간이다.

        거절은 정상이므로 실패로 세지 않는다 (계약 3절). 무한 대기는 T_lim 이 끊고,
        세마포어에도 60 초 강제 회수가 있다.
        """
        while not self.token(True):
            self.get_logger().info(f"P1 대기 — {self.p('token_retry_s')} s 후 재요청")
            self.wait(lambda: False, timeout=self.p('token_retry_s'))  # 무한 대기는 T_lim 이 끊는다
        self.token_held = True
        self.get_logger().info('P1 획득')


def main():
    rclpy.init()
    node = ShuttleMission()
    try:
        try:
            node.startup()       # 초기자세를 미션보다 먼저 (I-035). 콜백 밖 메인 루프다
        except Exception as e:   # 기동 초기화 실패로 노드를 죽이면 미션조차 못 받는다
            node.get_logger().error(f'기동 초기화 실패 — 미션 경로에서 다시 시도한다: {e}')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=TICK_S)
            if node.pending is not None and node.mission is None:
                m, node.pending = node.pending, None
                node.run_mission(m)      # 콜백 밖에서만 navigator 를 부른다
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C 는 KeyboardInterrupt, launch·systemd 의 SIGTERM 은 ExternalShutdownException 이다.
        #   둘 다 잡아야 종료 경로가 같아지고, 아래 finally 가 P1 토큰을 반납한다
        pass
    finally:
        try:
            node.release_token('노드 종료')   # Ctrl+C 로 죽어도 P1 을 물고 있지 않게
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()          # Ctrl+C 시 이미 내려간 경우 중복 호출 방지


if __name__ == '__main__':
    main()
