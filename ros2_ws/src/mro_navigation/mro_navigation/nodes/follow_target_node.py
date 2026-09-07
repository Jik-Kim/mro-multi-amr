"""S5 합류·재식별 → S6~S8 추종·재탐지·Standby.

**대기공간에서 시작한다.** 셔틀이 로봇을 대기공간에 세우고 `shuttle/arrived` 를 내면 그때부터
이 노드가 잡는다. 로봇이 작업자에게 가는 것이 아니라 작업자가 로봇에게 온다
(계약 8절 — 정차 자세는 서향 유지, RC카를 로봇 정면으로 돌아 들어오게 조종한다).

흐름
  1. `shuttle/arrived` 수신                                        → SEARCHING (phase MARKER)
  2. 마커 쪽을 2 초 보고 있는다 — 관제가 잡을 시간을 준다          → phase TURN
     판정은 하지 않는다. 못 잡혔으면 로그로만 알린다
  3. 작업자 쪽으로 돈다 — 천장 카메라 힌트가 있으면 그 방향으로 한 번에,
     없으면 30° 씩 돌며 찾는다. 각도를 사람이 정해 넣지 않는다      → phase WORKER
  4. 작업자 좌표를 일관성 검사로 확정 + `target/paired` 게이트 통과
  5. `approach_skip_m` 안이면 바로 FOLLOWING · 멀면 APPROACH_1 → APPROACH_2 → ARRIVED
  6. FOLLOWING — `follow_standoff_m` 를 유지하며 따라간다. **추종거리 채점 구간은 여기뿐이다**
  7. `lost_s` 안 보이면 주행 취소·제자리 대기 → 8 초면 회전 탐색 → `standby_s` 면 STANDBY
  8. 재등장하면 게이트를 다시 통과해야 재개 (계약 3절 재획득)
  9. `mission/finish` 오면 즉시 페어링 해제·goal 취소 → IDLE (**FAILED 가 아니다, 정상 종료**)

입력  shuttle/arrived(Bool) · target/map_point(PointStamped) · target/visible(Bool)
      target/paired(Bool) · target/worker_id(String) · /webcam2/target_hint(TargetHint, BEST_EFFORT)
      mission/finish(Bool) · safety/estop(Bool) · aruco/id(Int32) · aruco/distance(Float32)
출력  approach/status(String — 계약 4절의 8개 값) · cmd_vel(제자리 회전 전용, Nav2 우회)
      navigate_to_pose (TurtleBot4Navigator)

판정 관련 규칙 (어기면 KPI 가 조용히 틀어진다)
  · `approach/status` 는 계약이 정한 8개 문자열만 낸다 — 관제·kpi_logger 가 이 값으로 구간을
    자른다. 추종거리(AC-05)는 FOLLOWING 인 동안만 재므로 접근 단계가 섞이면 성적이 나빠진다.
  · STANDBY 로 끝난 재획득은 실패로 집계된다 (계약 9절). 함부로 STANDBY 로 빠지지 않는다.
  · estop·mission/finish 로 멈추는 것은 실패가 아니다. FAILED 를 내면 관제가 미션을 영구
    실패로 굳힌다 (I-026). estop 은 '잠깐 대기', finish 는 IDLE 로 돌린다.

도착 마커에 관하여
  계약 6절의 **측위 검증 마커 ID 4** 하나다. 대기공간 남쪽 벽에 붙이고 robot3·robot4 가 같은
  것을 본다 (9/7 에 웹캠1 박스 북쪽 거치대에서 이 위치로 옮겼다).
  **판정·기록은 관제(fleet_manager)가 한다** — `aruco/id` 를 구독해 DB 에 남긴다 (FR-35).
  이 노드가 판정하지 않는 이유는, 셔틀이 '도착했다' 고 주장한 것을 같은 주행 파트가 검증하면
  검증이 되지 않기 때문이다 (계약 9.1 '자기 성적을 자기가 매기지 않는다').
  여기서 '도착' 을 발행하지도 않는다 — 도착 판정은 셔틀의 `shuttle/status = WAITING` 이
  미션당 한 번 확정하며, 또 내면 계약 7절이 금지하는 이중 계상이다.

TODO([주행]):
① 마커 위치가 바뀌었으니 두 곳을 맞춰야 한다 — `arena.yaml` `loc_check_marker` 의 map 좌표
   실측 기입(주행, 지금 [0, 0] 이다) · 계약 6절 ID 4 의 위치 설명 갱신(PM).
   관제의 FR-35 판정은 그 좌표를 읽으므로 좌표가 없으면 검증이 성립하지 않는다.
② STANDBY 에서 '작업자 동선을 점유하지 않는 위치로 물러난다'(계약 3절)를 좌표로 옮기지 못했다.
   arena.yaml 에 대기 지점이 없고, Nav2 로 후퇴시키면 몸을 돌려 이동하므로 작업자를 시야에서
   잃어 재등장을 놓친다. 지금은 제자리에서 시야만 유지한다.
③ v_max(0.3 m/s)는 이 노드가 병진 속도를 내지 않으므로 여기서 제한할 것이 없다. Nav2 의
   max_vel_x 가 실효값이고 지금 0.26 으로 계약값과 다르다 — 아직 정리되지 않은 충돌.
"""
import math
import time

from geometry_msgs.msg import PointStamped, PoseStamped, Twist, TwistStamped
from mro_interfaces.msg import TargetHint
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, Int32, String
from tf2_ros import Buffer, TransformListener
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

from mro_perception.core.target_utils import robot_xy, standoff_goal

DEFAULT_NS = '/robot3'            # 네임스페이스 없이 단독 실행할 때만 쓰는 대체값

# 계약 4절 토픽 이름. 로봇 노드는 상대 이름을 쓰고 launch 가 namespace 를 붙인다.
ARRIVED_TOPIC = 'shuttle/arrived'     # 셔틀이 대기공간에 세워놓고 내는 시작 신호
POINT_TOPIC = 'target/map_point'
VISIBLE_TOPIC = 'target/visible'
PAIRED_TOPIC = 'target/paired'
WORKER_TOPIC = 'target/worker_id'
FINISH_TOPIC = 'mission/finish'
ESTOP_TOPIC = 'safety/estop'
ARUCO_ID_TOPIC = 'aruco/id'
ARUCO_DIST_TOPIC = 'aruco/distance'
STATUS_TOPIC = 'approach/status'
HINT_TOPIC = '/webcam2/target_hint'   # 전역 이름. BEST_EFFORT 라 sensor_data QoS 로 받아야 온다

# 검출 좌표를 믿기 위한 조건. 실측으로 검증된 값이라 건드리지 않는다
SETTLE_S = 2.0                # 이동/회전 끝난 뒤 이 시간 지나서 받은 좌표만 신뢰 (AMCL yaw 수렴 대기)
CONSIST_N = 2                 # 연속 N 개의 검출이
CONSIST_M = 0.3               # 서로 이 거리(m) 안에 있어야 목표로 인정 (TF 타이밍으로 튄 좌표 제거)
CONSIST_DT = 0.5              # 검사에 쓰는 검출 사이 최소 간격(s) — 같은 순간 TF 를 공유하지 않게
SEARCH_WAIT_S = 3.0           # 회전 후 이만큼 기다려도 못 찾으면 다시 회전
                              #   최소 필요치 = SETTLE_S + CONSIST_DT + 검출 몇 프레임 ≈ 2.7 s
VISIBLE_WAIT_S = 8.0          # 보이는데 좌표가 계속 걸러지면 이만큼만 기다리고 회전 (화각 63.7°)

STANDOFF_FAR = 1.0            # 먼 데서 올 때 1단계 목표 거리
SEARCH_ROTATE_DEG = 30.0      # 재탐색 때 한 번에 도는 각도 (부호 = 방향, 힌트가 있으면 힌트가 정한다)
SEARCH_ROTATE_SPEED = 1.5     # rad/s. Nav2 Spin 은 behavior_server max_rotational_vel(1.0)에 묶이고
                              #   액션 왕복 지연도 있어 cmd_vel 로 직접 돌린다
SEARCH_ROTATE_ACC = 6.0       # rad/s^2 명령 감속 프로파일 (실제 가감속은 Create 3 가 자체 제한)
USE_NAV2_SPIN = False         # True 면 Nav2 Spin 사용 (충돌 예측 포함, 대신 느림)
MAX_SEARCH_TRIES = 12         # 30° × 12 = 한 바퀴
NAV_TIMEOUT_S = 90.0

FOLLOW_REGOAL_M = 0.35        # 표적이 마지막 목표 기준 이만큼 움직였을 때만 새 목표 (재계획 남발 방지)
FOLLOW_REGOAL_DT = 1.0        # 새 목표 최소 간격(s)
FOLLOW_MAX_SPEED = 1.0        # 표적 최대 속도(m/s) 가정 — 이보다 빨리 튄 좌표는 TF 오차로 보고 버림
FOLLOW_EMA = 0.5             # 표적 좌표 지수평활 (1.0 = 평활 없음, 작을수록 부드럽지만 지연)
FOLLOW_TOL_M = 0.15          # 표적이 멈춘 뒤 거리 오차가 이보다 크면 한 번 더 보정
FOLLOW_RESEARCH_S = 8.0      # 이 시간 안 보이면 회전 탐색으로 복귀 (계약에 값이 없어 팀 가정)
TICK_S = 0.05                # 메인 루프 주기


class FollowTarget(Node):
    def __init__(self):
        super().__init__('follow_target')
        ns = self.get_namespace()
        self.ns = DEFAULT_NS if ns == '/' else ns
        self.navigator = TurtleBot4Navigator(namespace=self.ns)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- 파라미터. 단일 출처는 mro_bringup/params/robotN.yaml (계약 5절) ----
        self.declare_parameter('follow_standoff_m', 0.5)    # robotN.yaml — 추종 중 유지할 거리
        self.declare_parameter('min_separation_m', 0.3)     # robotN.yaml — 이보다 가까워지면 정지 (AC-05 하한)
        self.declare_parameter('lost_s', 1.0)               # 미검출 이 시간이면 Lost (SOT '.. Lost 1.0 s')
        self.declare_parameter('standby_s', 30.0)           # Lost 가 이만큼 지속되면 STANDBY (SOT 'Standby 30 s')
        self.declare_parameter('approach_skip_m', 1.0)      # 페어링 시 이 거리 안이면 접근 없이 바로 추종
        self.declare_parameter('hint_valid_s', 2.0)         # 계약 4절 '힌트 유효 2 s'
        self.declare_parameter('arrival_marker_id', 4)      # 계약 6절 측위 검증 마커. 0 을 주면 이 단계를 건너뛴다
        self.declare_parameter('arrival_marker_look_s', 2.0)   # 마커 쪽을 보고 있는 시간.
        #   관제가 10 Hz 로 오는 aruco/id 를 잡기에 충분하다. 판정을 우리가 하지 않으니
        #   길게 기다릴 이유가 없다 — 시연 시간만 잡아먹는다

        # ---- 구독 ----
        self.create_subscription(Bool, ARRIVED_TOPIC, self.arrived_cb, 1)
        self.create_subscription(PointStamped, POINT_TOPIC, self.point_cb, 1)
        self.create_subscription(Bool, VISIBLE_TOPIC, self.visible_cb, 1)
        self.create_subscription(Bool, PAIRED_TOPIC, self.paired_cb, 1)
        self.create_subscription(String, WORKER_TOPIC, self.worker_cb, 1)
        self.create_subscription(Bool, FINISH_TOPIC, self.finish_cb, 1)
        self.create_subscription(Bool, ESTOP_TOPIC, self.estop_cb, 1)
        self.create_subscription(Int32, ARUCO_ID_TOPIC, self.aruco_id_cb, 1)
        self.create_subscription(Float32, ARUCO_DIST_TOPIC, self.aruco_dist_cb, 1)
        # 힌트만 QoS 가 다르다. 발행자가 BEST_EFFORT 라 기본 RELIABLE 로 잡으면 한 건도 안 온다
        self.create_subscription(TargetHint, HINT_TOPIC, self.hint_cb, qos_profile_sensor_data)

        self.pub_status = self.create_publisher(String, STATUS_TOPIC, 1)

        # ---- 상태 ----
        self.state = 'IDLE'
        self.phase = None             # SEARCHING 안의 단계: MARKER → TURN → WORKER
        self.stage = 1                # 접근 단계 (1 = 먼 목표, 2 = 가까운 목표)
        self.start_req = False        # shuttle/arrived 를 받았다
        self.finish_req = False       # mission/finish 를 받았다
        self.estop = False
        self.estop_cancelled = False  # estop 진입 시 goal 취소를 한 번만
        self.paired = False
        self.worker_id = ''
        self.visible_at = 0.0
        self.latest = None            # (x, y, 수신 시각)
        self.recent = []              # accept_after 이후 받은 좌표들 (일관성 검사용)
        self.recent_t = 0.0
        self.accept_after = 0.0       # 이 시각 이후 수신된 검출만 사용
        self.search_started = 0.0
        self.search_tries = 0
        self.nav_started = None
        self.hint = None              # (x, y, 수신 시각, worker_id)
        self.aruco_id = None
        self.aruco_dist = None
        self.marker_started = 0.0     # 마커 쪽을 보기 시작한 시각
        self.marker_ok = None         # 도착 마커를 봤나 (None = 아직, True/False = 결과)
        self.worker_started = 0.0     # 작업자 찾기 시작 시각 — 못 찾은 시간을 로그에 적으려고
        self.circles = 0              # 한 바퀴를 몇 번 돌았나
        self.gate_warned = 0.0        # 게이트 대기 경고를 마지막으로 낸 시각
        self.follow_est = None        # (x, y) 평활된 표적 추정 위치
        self.follow_t = 0.0
        self.follow_seen = 0.0        # 마지막으로 표적을 본 시각
        self.last_target = None       # 마지막 접근 목표로 쓴 표적 좌표
        self.follow_goal = None       # 마지막으로 목표를 만든 표적 위치
        self.follow_goal_t = 0.0
        self.follow_holding = False   # Lost 로 주행을 취소한 상태
        self.cmd_pub = None
        self.cmd_stamped = None

        self.create_timer(1.0, lambda: self.pub_status.publish(String(data=self.state)))
        self.get_logger().info(
            f'[{self.state}] 대기공간 도착 신호 {self.ns}/{ARRIVED_TOPIC} 를 기다린다 · '
            f'추종거리 {self.p("follow_standoff_m")} m · 최소 이격 {self.p("min_separation_m")} m · '
            f'Lost {self.p("lost_s")} s · STANDBY {self.p("standby_s")} s')

    def p(self, name):
        return self.get_parameter(name).value

    def set_state(self, state, msg=''):
        self.state = state
        self.pub_status.publish(String(data=state))
        self.get_logger().info(f'[{state}] {msg}')

    # ================= 콜백: 값만 저장한다 =================
    # navigator 를 콜백에서 부르면 단일 스레드가 막혀 다른 토픽이 끊긴다. 실행은 메인 루프가 한다.

    def arrived_cb(self, msg):
        if msg.data:
            self.start_req = True

    def point_cb(self, msg):
        if msg.header.frame_id == 'map':
            self.latest = (msg.point.x, msg.point.y, time.time())

    def visible_cb(self, msg):
        if msg.data:
            self.visible_at = time.time()

    def paired_cb(self, msg):
        """재식별 게이트 (계약 4절). 거짓이 되면 추종을 멈춘다 — 오추종 0건이 절대 기준이다."""
        if self.paired and not msg.data:
            self.get_logger().warn('페어링 해제됨 — 게이트를 다시 통과해야 재개한다')
        self.paired = bool(msg.data)

    def worker_cb(self, msg):
        self.worker_id = msg.data

    def finish_cb(self, msg):
        if msg.data:
            self.finish_req = True

    def estop_cb(self, msg):
        was = self.estop
        self.estop = bool(msg.data)
        if self.estop and not was:
            self.get_logger().warn('estop — 주행 명령을 멈춘다 (실패가 아니다)')
        elif was and not self.estop:
            self.get_logger().info('estop 해제')
            self.estop_cancelled = False

    def aruco_id_cb(self, msg):
        self.aruco_id = int(msg.data)

    def aruco_dist_cb(self, msg):
        self.aruco_dist = float(msg.data)

    def hint_cb(self, msg):
        """웹캠2 가 주는 작업자 map 좌표. 방위가 아니라 좌표다 — 회전 부호는 우리가 계산한다."""
        if msg.point.header.frame_id != 'map':
            return
        self.hint = (msg.point.point.x, msg.point.point.y, time.time(), msg.worker_id)

    # ================= 공통 =================

    def rearm(self):
        """이동·회전 직후 호출. 큐에 남은 좌표를 버리고 SETTLE_S 뒤부터 받은 것만 쓴다.

        움직이는 동안 들어온 좌표는 TF 타이밍 때문에 틀려 있다. AMCL 이 수렴할 시간을 준다.
        """
        self.latest = None
        self.recent = []
        self.recent_t = 0.0
        self.accept_after = time.time() + SETTLE_S
        self.search_started = time.time()

    def gate_ok(self):
        """계약 3절 재식별 게이트. 통과 못 하면 움직이지 않는다 (BR-T16·R-05)."""
        return self.paired

    def abort_motion(self):
        """제자리 회전 같은 블로킹 동작을 중간에 끊어야 하는가 — finish 는 1 초 안에 멈춰야 한다."""
        return self.finish_req or self.estop

    def hold(self, reason):
        """진행 중 주행을 취소하고 제자리에 선다. 상태는 바꾸지 않는다 (실패가 아니다)."""
        if not self.follow_holding:
            self.navigator.cancelTask()
            self.follow_holding = True
            self.get_logger().info(f'정지하고 대기 — {reason}')

    # ================= 사이클 시작·종료 =================

    def begin_cycle(self):
        """대기공간 도착 신호를 받은 시점. 여기서부터가 S5 다."""
        self.start_req = False
        self.stage = 1
        self.search_tries = 0
        self.follow_est = None
        self.follow_goal = None
        self.last_target = None
        self.follow_holding = False
        self.marker_started = time.time()
        self.marker_ok = None
        self.worker_started = 0.0
        self.circles = 0
        self.gate_warned = 0.0
        self.phase = 'MARKER' if int(self.p('arrival_marker_id')) > 0 else 'TURN'
        self.rearm()
        self.set_state('SEARCHING', '대기공간 도착 — 도착 마커 확인' if self.phase == 'MARKER'
                       else '대기공간 도착 — 마커 확인 끄고 바로 회전')

    def do_finish(self):
        """mission/finish 처리 (계약 4절 '≤ 1 s'). 정상 종료이므로 FAILED 가 아니라 IDLE 이다."""
        self.finish_req = False
        self.navigator.cancelTask()
        self.paired = False
        self.phase = None
        self.follow_est = None
        self.follow_goal = None
        self.follow_holding = False
        self.set_state('IDLE', '작업 종료 — 페어링 해제·목표 취소 (정상 종료)')

    # ================= 메인 루프 =================

    def step(self):
        # 1) 종료가 가장 우선. 진행 중 무엇이든 끊고 IDLE 로
        if self.finish_req:
            return self.do_finish()

        # 2) estop 중에는 새 명령을 내지 않는다. 상태는 유지 — 해제되면 그대로 이어간다
        if self.estop:
            if not self.estop_cancelled:
                self.navigator.cancelTask()
                self.estop_cancelled = True
            return

        # 3) 새 사이클. FOLLOWING·STANDBY 중에 다시 와도 새로 시작한다
        if self.start_req:
            return self.begin_cycle()

        if self.state == 'SEARCHING':
            self.step_searching()
        elif self.state in ('APPROACH_1', 'APPROACH_2'):
            self.step_approach()
        elif self.state == 'FOLLOWING':
            self.step_following()
        elif self.state == 'STANDBY':
            self.step_standby()

    # ---- SEARCHING: 마커 → 뒤돌기 → 작업자 ----

    def step_searching(self):
        if self.phase == 'MARKER':
            return self.step_marker()
        if self.phase == 'TURN':
            return self.step_turn()
        return self.step_worker()

    def step_marker(self):
        """마커가 관제에 잡힐 시간만 준다. 판정하지 않는다.

        도착 판정은 셔틀의 shuttle/status = WAITING 이 이미 확정했고(계약 9절), 그것이 진짜인지
        뒷받침하는 것이 이 마커다. 그 뒷받침을 관제가 한다 — aruco/id 를 구독해 DB 에 남긴다.
        우리가 판정하면 셔틀의 주장을 같은 주행 파트가 검증하는 셈이라 검증이 되지 않는다.
        여기서 또 "도착" 을 발행하지도 않는다 — 계약 7절이 금지하는 이중 계상이다.
        """
        want = int(self.p('arrival_marker_id'))
        now = time.time()
        if self.aruco_id == want:
            self.marker_ok = True          # 봤다는 사실만 남긴다. 판정·기록은 관제
        looked = now - self.marker_started
        if looked < self.p('arrival_marker_look_s'):
            return                         # 아직 시간을 주는 중
        if self.marker_ok:
            d = f'{self.aruco_dist:.2f} m' if self.aruco_dist is not None else '거리 미수신'
            self.get_logger().info(f'★ 도착 마커 ID {want} 보였다 (거리 {d}) — 판정은 관제가 한다')
        else:
            self.marker_ok = False
            seen = self.aruco_id if self.aruco_id is not None else '아무것도'
            self.get_logger().error(
                f'★ 도착 마커를 못 알아봤다 — ID {want} 를 {looked:.0f} s 동안 봤지만 {seen} 만 보였다. '
                f'확인할 것: 마커가 시야에 있는가 · 1.5 m 안인가 · 정면 ±30° 인가 · '
                f'인지 perception.yaml 의 marker_ids 에 {want} 가 있는가. '
                f'관제의 측위 확인이 이번 미션에는 안 남는다. 주행은 계속 진행한다')
        self.phase = 'TURN'

    def step_turn(self):
        """작업자 쪽으로 돈다.

        각도를 사람이 정해 넣지 않는다 — 마커를 보는 동안 로봇은 작업자 반대쪽을 향하고 있어
        자기 카메라로는 작업자가 안 보인다. 그때 방향을 아는 것은 천장 카메라(웹캠2)뿐이다.
          힌트가 있으면 그 방향으로 한 번에 돈다.
          없으면 회전하지 않고 바로 작업자 찾기로 넘어간다 — 거기서 30° 씩 돌며 찾는다.
        """
        self.worker_started = time.time()
        b = self.hint_bearing()
        if b is None:
            self.get_logger().info('천장 카메라 힌트 없음 — 30° 씩 돌며 작업자를 찾는다')
            self.phase = 'WORKER'
            return self.rearm()
        self.get_logger().info(f'힌트 방위 {b:+.0f}° — 작업자 쪽으로 돈다')
        self.rotate_in_place(b)
        self.phase = 'WORKER'
        self.rearm()

    def step_worker(self):
        """작업자 좌표를 일관성 검사로 확정하고 게이트를 통과하면 추종으로 넘어간다."""
        if self.latest is not None and self.latest[2] >= self.accept_after:
            tx, ty, t = self.latest
            self.latest = None
            if t - self.recent_t >= CONSIST_DT:
                self.recent_t = t
                self.recent = (self.recent + [(tx, ty)])[-CONSIST_N:]
                if len(self.recent) == CONSIST_N and all(
                        math.hypot(x - tx, y - ty) <= CONSIST_M for x, y in self.recent):
                    mx = sum(x for x, _ in self.recent) / CONSIST_N
                    my = sum(y for _, y in self.recent) / CONSIST_N
                    if not self.gate_ok():
                        # 사람은 보이는데 내 작업자로 확인되지 않았다 — 움직이지 않고 기다린다.
                        #   남을 따라가는 것보다 서 있는 것이 낫다 (오추종 0건이 절대 기준)
                        now = time.time()
                        if now - self.gate_warned > 5.0:
                            self.gate_warned = now
                            self.get_logger().warn(
                                f'★ 사람은 찾았는데 내 작업자로 확인되지 않는다 — '
                                f'좌표 ({mx:.2f},{my:.2f}), {now - self.worker_started:.0f} s 경과. '
                                f'target/paired 가 참이 되어야 따라간다 (인지의 재식별 게이트). '
                                f'추종 개시 안 함')
                        self.recent = []
                        return
                    self.search_tries = 0
                    return self.on_paired(mx, my)
                self.get_logger().info(f'좌표 ({tx:.2f},{ty:.2f}) 수신 — '
                                       f'일관성 확인 중 ({len(self.recent)}/{CONSIST_N})')

        waited = time.time() - self.search_started
        visible = time.time() - self.visible_at < 1.0
        if waited < SEARCH_WAIT_S or (visible and waited < VISIBLE_WAIT_S):
            return                                  # 보이면 회전하지 말고 좋은 좌표를 기다린다
        self.search_tries += 1
        if self.search_tries > MAX_SEARCH_TRIES:
            self.search_tries = 0                   # 계속 돈다 — FAILED 로 죽지 않는다
            self.circles += 1
            self.get_logger().error(
                f'★ 작업자를 못 알아봤다 — 한 바퀴 {self.circles} 번째, '
                f'{time.time() - self.worker_started:.0f} s 경과. '
                f'확인할 것: 작업자가 시야에 있는가 · 1 m 안인가 · '
                f'인지 노드가 target/map_point 를 내고 있는가. 탐색은 계속한다')
        deg = self.search_rotate_deg()
        self.rotate_in_place(deg)
        self.rearm()

    def on_paired(self, tx, ty):
        """게이트 통과. 작업자가 가까우면 접근 없이 바로 추종한다.

        새 시나리오에서는 작업자가 로봇에게 30~50 cm 까지 걸어온다. 그 거리에서 1 m 뒤로
        물러났다 다시 붙는 것은 의미가 없고 최소 이격만 위태롭게 한다. 멀리 있을 때만
        2단계 접근을 쓴다 — 2 m 밖 좌표는 0.5~0.7 m 틀리므로 한 번에 목표를 잡을 수 없다.
        """
        rob = robot_xy(self.tf_buffer, self.get_logger())
        if rob is None:
            self.get_logger().warn('로봇 위치 TF 없음 — 다음 주기에 다시')
            return
        gap = math.hypot(tx - rob[0], ty - rob[1])
        if gap <= self.p('approach_skip_m'):
            self.last_target = (tx, ty)
            self.get_logger().info(f'작업자 확인 ({tx:.2f},{ty:.2f}) {gap:.2f} m — 접근 없이 바로 추종')
            return self.enter_follow()
        self.get_logger().info(f'작업자 확인 ({tx:.2f},{ty:.2f}) {gap:.2f} m — 멀어서 2단계 접근')
        self.stage = 1
        self.send_standoff(tx, ty)

    # ---- 힌트 → 회전 부호 ----

    def hint_bearing(self):
        """힌트 좌표를 '몇 도 돌아야 하는지' 로 바꾼다. 못 쓰면 None.

        웹캠2 는 작업자 map 좌표만 준다 — 로봇이 어디서 어느 쪽을 보는지 모른다.
        그래서 (힌트 − 내 위치) 의 방향과 내가 보는 방향의 차를 우리가 계산한다.
        worker_id 로 내 작업자의 힌트인지 가린다 — 로봇이 두 대고 힌트 토픽은 전역이라
        상대 로봇의 작업자 힌트가 섞여 들어온다. 계약에 '걸러라' 는 문구는 없고 우리 판단이다.
        """
        if self.hint is None:
            return None
        hx, hy, ht, hw = self.hint
        if time.time() - ht > self.p('hint_valid_s'):
            return None                                    # 계약 4절 유효 2 s
        if self.worker_id and hw and hw != self.worker_id:
            return None                                    # 남의 작업자 힌트
        rob = robot_xy(self.tf_buffer)   # 힌트 계산은 매 주기 도니 TF 실패를 로그로 도배하지 않는다
        if rob is None:
            return None
        yaw = self.map_yaw()
        if yaw is None:
            return None
        want = math.atan2(hy - rob[1], hx - rob[0])
        diff = math.atan2(math.sin(want - yaw), math.cos(want - yaw))
        return math.degrees(diff)

    def search_rotate_deg(self):
        """돌 각도. 힌트가 있으면 그 방향으로 한 번에, 없으면 기존 30° 씩."""
        b = self.hint_bearing()
        if b is None:
            self.get_logger().info(f'작업자 없음 — {SEARCH_ROTATE_DEG:.0f}° 회전 '
                                   f'({self.search_tries}/{MAX_SEARCH_TRIES}) · 힌트 없음')
            return SEARCH_ROTATE_DEG
        # 부호가 방향이다 — 양수 반시계, 음수 시계. 너무 작은 각은 30° 로 올려 실제로 시야가 바뀌게
        deg = b if abs(b) >= SEARCH_ROTATE_DEG else math.copysign(SEARCH_ROTATE_DEG, b or 1.0)
        self.get_logger().info(f'힌트 방위 {b:+.0f}° → {deg:+.0f}° 회전')
        return deg

    def map_yaw(self):
        """map→base_link yaw. 힌트는 map 좌표이므로 비교도 map 프레임에서 해야 한다."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time(),
                                                timeout=Duration(seconds=0.2))
        except Exception:
            return None
        q = t.transform.rotation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # ---- 제자리 회전 ----

    def setup_cmd_vel(self):
        """cmd_vel 토픽 타입(Twist / TwistStamped)을 보고 퍼블리셔를 만든다. 못 찾으면 None."""
        if self.cmd_pub is not None:
            return self.cmd_pub
        name = f'{self.ns}/cmd_vel'
        types = dict(self.get_topic_names_and_types()).get(name, [])
        if 'geometry_msgs/msg/TwistStamped' in types:
            self.cmd_stamped = True
        elif 'geometry_msgs/msg/Twist' in types:
            self.cmd_stamped = False
        else:
            return None                      # 아직 못 봄 → 이번엔 Nav2 Spin 으로
        self.cmd_pub = self.create_publisher(
            TwistStamped if self.cmd_stamped else Twist, 'cmd_vel', 1)
        self.get_logger().info(f'제자리 회전: {name} '
                               f'({"TwistStamped" if self.cmd_stamped else "Twist"}) '
                               f'{SEARCH_ROTATE_SPEED} rad/s')
        return self.cmd_pub

    def publish_wz(self, wz):
        if self.cmd_stamped:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'base_link'
            m.twist.angular.z = wz
        else:
            m = Twist()
            m.angular.z = wz
        self.cmd_pub.publish(m)

    def odom_yaw(self):
        """odom→base_link yaw. AMCL 점프가 없어 회전량 적산에 map 보다 적합."""
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_link', Time(),
                                                timeout=Duration(seconds=0.2))
        except Exception:
            return None
        q = t.transform.rotation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def rotate_in_place(self, deg):
        """제자리 회전. 병진이 없고 Nav2 목표를 수행하지 않는 동안에만 부른다.

        블로킹이라 도는 동안 상태기계가 멈춘다. 그래서 estop·mission/finish 가 들어오면
        즉시 끊고 나온다 — finish 는 1 초 안에 멈춰야 한다 (계약 4절).
        """
        if USE_NAV2_SPIN or self.setup_cmd_vel() is None:
            self.navigator.spin(spin_dist=math.radians(deg))
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.abort_motion():
                    self.navigator.cancelTask()
                    return
            return
        target = math.radians(abs(deg))
        sign = 1.0 if deg >= 0 else -1.0
        turned, prev = 0.0, self.odom_yaw()
        t0 = time.time()
        deadline = t0 + 3.0 * target / max(SEARCH_ROTATE_SPEED, 0.1) + 3.0
        stopped = False
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.abort_motion():
                self.get_logger().info('회전 중단 — 종료 또는 estop')
                stopped = True
                break
            if prev is None:                                  # 시작 때 TF 가 없었음 → 시간으로 추정
                turned = (time.time() - t0) * SEARCH_ROTATE_SPEED
            else:
                y = self.odom_yaw()
                if y is not None:                             # 이번 조회 실패면 다음 주기에
                    turned += abs(math.atan2(math.sin(y - prev), math.cos(y - prev)))
                    prev = y
            left = target - turned
            if left <= 0.02:
                stopped = True
                break
            # 남은 각도에 맞춰 감속 (Nav2 Spin 과 같은 프로파일)
            self.publish_wz(sign * max(0.4, min(SEARCH_ROTATE_SPEED,
                                                math.sqrt(2.0 * SEARCH_ROTATE_ACC * left))))
        if not stopped:
            self.get_logger().warn('회전 타임아웃 — 정지')
        for _ in range(3):
            self.publish_wz(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    # ---- 2단계 접근 (작업자가 멀 때만) ----

    def send_standoff(self, tx, ty):
        rob = robot_xy(self.tf_buffer, self.get_logger())
        if rob is None:
            self.get_logger().warn('로봇 위치 TF 없음 — 접근 보류')
            return
        standoff = STANDOFF_FAR if self.stage == 1 else self.p('follow_standoff_m')
        goal, dist, yaw = standoff_goal(*rob, tx, ty, standoff, self.get_clock().now().to_msg())
        if not self.navigator.goToPose(goal):
            self.get_logger().warn('접근 목표 거부됨 — 탐색으로 돌아간다')
            self.phase = 'WORKER'
            return self.set_state('SEARCHING', '목표 거부 — 재탐색')
        self.nav_started = time.time()
        self.last_target = (tx, ty)
        self.set_state(f'APPROACH_{self.stage}',
                       f'작업자 ({tx:.2f},{ty:.2f}) {dist:.2f} m → 앞 {standoff} m '
                       f'({goal.pose.position.x:.2f},{goal.pose.position.y:.2f},'
                       f'{math.degrees(yaw):.0f}°)')

    def step_approach(self):
        if not self.gate_ok():
            # 접근 중 페어링이 풀렸다 — 남을 따라가지 않는다 (오추종 0건)
            self.navigator.cancelTask()
            self.phase = 'WORKER'
            self.rearm()
            return self.set_state('SEARCHING', '접근 중 페어링 해제 — 게이트 재통과 대기')
        if not self.navigator.isTaskComplete():
            if time.time() - self.nav_started > NAV_TIMEOUT_S:
                self.navigator.cancelTask()
                self.phase = 'WORKER'
                self.rearm()
                self.set_state('SEARCHING', '접근 주행 타임아웃 — 재탐색')
            return
        if self.navigator.getResult().name != 'SUCCEEDED':
            self.phase = 'WORKER'
            self.rearm()
            return self.set_state('SEARCHING', f'{self.stage}단계 주행 실패 — 재탐색')
        if self.stage == 1:
            self.stage = 2
            self.phase = 'WORKER'
            self.rearm()
            return self.set_state('SEARCHING', '1단계 도착 — 정지 후 재검출 대기')
        self.set_state('ARRIVED', f'작업자 앞 {self.p("follow_standoff_m")} m 도착')
        self.enter_follow()

    # ---- 추종 ----

    def enter_follow(self):
        now = time.time()
        self.latest = None
        self.follow_est = None
        self.follow_goal = self.last_target   # 이 위치에서 움직여야 새 목표를 낸다
        self.follow_goal_t = now
        self.follow_seen = now
        self.follow_holding = False
        self.set_state('FOLLOWING',
                       f'표적이 {FOLLOW_REGOAL_M} m 이상 움직이면 '
                       f'{self.p("follow_standoff_m")} m 를 유지하며 따라간다')

    def follow_update(self):
        """새 좌표를 속도 게이트 + 지수평활로 흡수. 이동 중 좌표도 사용."""
        if self.latest is None:
            return
        tx, ty, t = self.latest
        self.latest = None
        if self.follow_est is None:
            self.follow_est, self.follow_t = (tx, ty), t
            return
        ex, ey = self.follow_est
        dt = max(t - self.follow_t, 1e-3)
        if math.hypot(tx - ex, ty - ey) > FOLLOW_MAX_SPEED * dt + CONSIST_M:
            self.get_logger().warn(f'좌표 ({tx:.2f},{ty:.2f}) 튐 — {dt:.1f} s 만에 너무 멀리, 버림')
            return
        self.follow_est = (ex + FOLLOW_EMA * (tx - ex), ey + FOLLOW_EMA * (ty - ey))
        self.follow_t = t

    def step_following(self):
        now = time.time()

        # 페어링이 풀리면 즉시 멈춘다 — 남을 따라가는 것보다 서 있는 것이 낫다 (R-05)
        if not self.gate_ok():
            self.navigator.cancelTask()
            self.phase = 'WORKER'
            self.rearm()
            return self.set_state('SEARCHING', '페어링 해제 — 게이트 재통과 대기')

        self.follow_update()
        if self.follow_t > self.follow_seen:
            self.follow_seen = self.follow_t
        if now - self.visible_at < 1.0:
            self.follow_seen = max(self.follow_seen, self.visible_at)

        lost = now - self.follow_seen
        if lost > self.p('standby_s'):
            return self.enter_standby(lost)
        if lost > FOLLOW_RESEARCH_S:
            self.navigator.cancelTask()
            self.stage = 2
            self.search_tries = 0
            self.phase = 'WORKER'
            self.rearm()
            return self.set_state('SEARCHING', f'표적 {lost:.0f} s 안 보임 — 회전 탐색')
        if lost > self.p('lost_s'):
            return self.hold(f'표적 {lost:.1f} s 안 보임')

        if self.follow_est is None or now - self.follow_goal_t < FOLLOW_REGOAL_DT:
            return
        rob = robot_xy(self.tf_buffer, self.get_logger())
        if rob is None:
            return

        gap = math.hypot(self.follow_est[0] - rob[0], self.follow_est[1] - rob[1])

        # 최소 이격 하한 (AC-05 의 절대 기준). 이보다 붙으면 목표를 지우고 선다.
        #   Nav2 는 후진하지 않으므로 물러나게 만들지 않고, 작업자가 떨어지면 다시 따라간다.
        if gap < self.p('min_separation_m'):
            return self.hold(f'최소 이격 {self.p("min_separation_m")} m 침범 (현재 {gap:.2f} m)')

        parked = self.navigator.isTaskComplete()
        if parked and self.follow_goal is not None and \
                self.navigator.getResult().name != 'SUCCEEDED':
            self.get_logger().warn('추종 주행 실패 — 목표 다시 보냄')
            self.follow_goal = None        # 성공 도착이면 유지 (정지한 표적에 goal 남발 방지)
        moved = (math.inf if self.follow_goal is None
                 else math.hypot(self.follow_est[0] - self.follow_goal[0],
                                 self.follow_est[1] - self.follow_goal[1]))
        # 멈춰 있는데 표적까지 standoff 보다 멀면 잔차 보정 (가까운 쪽은 후진하지 않으므로 그대로)
        residual = gap - self.p('follow_standoff_m')
        if moved < FOLLOW_REGOAL_M and not (parked and residual > FOLLOW_TOL_M):
            return
        goal, dist, yaw = standoff_goal(*rob, *self.follow_est, self.p('follow_standoff_m'),
                                        self.get_clock().now().to_msg())
        if not self.navigator.goToPose(goal):      # 새 goal 이 이전 goal 을 선점
            self.get_logger().warn('추종 목표 거부됨')
            return
        self.follow_goal = self.follow_est
        self.follow_goal_t = now
        self.follow_holding = False
        self.get_logger().info(f'추종: 표적 ({self.follow_est[0]:.2f},{self.follow_est[1]:.2f}) '
                               f'{dist:.2f} m → 앞 {self.p("follow_standoff_m")} m')

    # ---- STANDBY ----

    def enter_standby(self, lost):
        """작업자가 오래 사라졌다. 오류가 아니라 정규 상태다 (BR-T17).

        계약 3절은 '작업자 동선을 점유하지 않는 대기 위치로 물러난다' 고 하는데 그 좌표가
        arena.yaml 에 없다. Nav2 로 후퇴시키면 몸을 돌려 이동하므로 작업자를 시야에서 잃고
        재등장을 놓친다 — 그래서 지금은 제자리에서 시야만 유지한다 (TODO ②).
        """
        self.navigator.cancelTask()
        self.follow_holding = True
        self.follow_est = None
        self.follow_goal = None
        self.set_state('STANDBY', f'표적 {lost:.0f} s 안 보임 — 대기 (오류 아님). '
                                  f'재등장하면 게이트를 다시 통과해야 재개한다')

    def step_standby(self):
        """재등장 감지. 게이트를 다시 통과해야 재개한다 (계약 3절 재획득)."""
        now = time.time()
        seen = (self.latest is not None) or (now - self.visible_at < 1.0)
        if not seen:
            return
        if not self.gate_ok():
            self.get_logger().info('작업자 재등장 — target/paired 대기 중',
                                   throttle_duration_sec=5.0)
            return
        self.stage = 2
        self.search_tries = 0
        self.phase = 'WORKER'
        self.rearm()
        self.set_state('SEARCHING', '작업자 재등장 + 게이트 통과 — 재획득 시작')


def main():
    rclpy.init()
    node = FollowTarget()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=TICK_S)
            node.step()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C 는 KeyboardInterrupt, launch·systemd 의 SIGTERM 은 ExternalShutdownException.
        #   둘 다 잡아야 종료 경로가 같아진다
        pass
    finally:
        try:
            node.navigator.cancelTask()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()      # Ctrl+C 시 이미 내려간 경우 중복 호출 방지


if __name__ == '__main__':
    main()
