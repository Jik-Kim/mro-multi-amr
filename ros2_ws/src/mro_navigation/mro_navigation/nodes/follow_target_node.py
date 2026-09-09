"""S5 합류·재식별 → S6~S8 추종·재탐지·Standby.

**대기공간에서 시작한다.** 셔틀이 로봇을 대기공간에 세우고 `shuttle/arrived` 를 내면 그때부터
이 노드가 잡는다. 로봇이 작업자에게 가는 것이 아니라 작업자가 로봇에게 온다
(계약 8절 — 정차 자세는 서향 유지, RC카를 로봇 정면으로 돌아 들어오게 조종한다).

흐름
  1. `shuttle/arrived` 수신 → SEARCHING. 로봇은 이미 작업자 쪽(서향)을 보고 서 있다
  2. 작업자 좌표를 일관성 검사로 확정 + `target/paired` 게이트 통과
  3. `approach_skip_m` 안이면 바로 FOLLOWING · 멀면 **남은 거리의 절반씩 좁히는 점진 접근**
     (`next_standoff`) 으로 최종 이격까지 붙은 뒤 ARRIVED. 회차 수는 정해두지 않는다 —
     목표가 `max(follow_standoff_m, gap/2)` 라 gap 이 반씩 줄고 하한에서 저절로 끝난다.
     상태는 계약 4절의 두 값으로 낸다: **APPROACH_1 = 중간 목표, APPROACH_2 = 최종 목표**
  4. FOLLOWING — `follow_standoff_m` 를 유지하며 따라간다. **추종거리 채점 구간은 여기뿐이다**
  5. `lost_s` 안 보이면 주행 취소·제자리 대기 → `research_s` 면 회전 탐색 → `standby_s` 면 STANDBY.
     **회전 방향은 힌트 > 마지막으로 본 쪽 > 기본 30° 순으로 정한다** — 천장 카메라가 없어도
     놓치기 직전 위치를 우리가 들고 있으므로 아무 쪽이나 돌지 않는다
  6. 재등장하면 게이트를 다시 통과해야 재개 (계약 3절 재획득)
  7. `mission/finish` 오면 즉시 페어링 해제·goal 취소 → IDLE (**FAILED 가 아니다, 정상 종료**)

회전은 놓쳤을 때 다시 찾는 데만 쓴다 (FR-17). 도착 직후에는 돌지 않는다 — 정차 자세가
이미 서향이고 작업자를 로봇 정면으로 돌아 들어오게 조종하기 때문이다 (계약 8절).

입력  shuttle/arrived(Bool) · target/map_point(PointStamped) · target/visible(Bool)
      target/paired(Bool) · target/worker_id(String) · /webcam2/target_hint(TargetHint, BEST_EFFORT)
      mission/finish(Bool) · safety/estop(Bool)
출력  approach/status(String — 계약 4절의 8개 값) · cmd_vel(제자리 회전 전용, Nav2 우회)
      navigate_to_pose (TurtleBot4Navigator)

판정 관련 규칙 (어기면 KPI 가 조용히 틀어진다)
  · `approach/status` 는 계약이 정한 8개 문자열만 낸다 — 관제·kpi_logger 가 이 값으로 구간을
    자른다. 추종거리(AC-05)는 FOLLOWING 인 동안만 재므로 접근 단계가 섞이면 성적이 나빠진다.
  · STANDBY 로 끝난 재획득은 실패로 집계된다 (계약 9절). 함부로 STANDBY 로 빠지지 않는다.
  · estop·mission/finish 로 멈추는 것은 실패가 아니다. FAILED 를 내면 관제가 미션을 영구
    실패로 굳힌다 (I-026). estop 은 '잠깐 대기', finish 는 IDLE 로 돌린다.

TODO([주행]):
① STANDBY 에서 '작업자 동선을 점유하지 않는 위치로 물러난다'(계약 3절)를 좌표로 옮기지 못했다.
   arena.yaml 에 대기 지점이 없고, Nav2 로 후퇴시키면 몸을 돌려 이동하므로 작업자를 시야에서
   잃어 재등장을 놓친다. 지금은 제자리에서 시야만 유지한다.
② v_max(0.3 m/s)는 이 노드가 병진 속도를 내지 않으므로 여기서 제한할 것이 없다. Nav2 의
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
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

from mro_perception.core.target_utils import robot_xy, standoff_goal

# 네임스페이스 대체값은 두지 않는다 (9/8) — 대체값으로 뜨면 남의 로봇에 goToPose 를 건다

# 계약 4절 토픽 이름. 로봇 노드는 상대 이름을 쓰고 launch 가 namespace 를 붙인다.
ARRIVED_TOPIC = 'shuttle/arrived'     # 셔틀이 대기공간에 세워놓고 내는 시작 신호
POINT_TOPIC = 'target/map_point'
VISIBLE_TOPIC = 'target/visible'
PAIRED_TOPIC = 'target/paired'
WORKER_TOPIC = 'target/worker_id'
FINISH_TOPIC = 'mission/finish'
ESTOP_TOPIC = 'safety/estop'
STATUS_TOPIC = 'approach/status'
HINT_TOPIC = '/webcam2/target_hint'   # 전역 이름. BEST_EFFORT 라 sensor_data QoS 로 받아야 온다

# 검출 좌표를 믿기 위한 조건. 실측으로 검증된 값이라 건드리지 않는다
CONSIST_N = 2                 # 연속 N 개의 검출이
CONSIST_M = 0.3               # 서로 이 거리(m) 안에 있어야 목표로 인정 (TF 타이밍으로 튄 좌표 제거)
CONSIST_DT = 0.5              # 검사에 쓰는 검출 사이 최소 간격(s) — 같은 순간 TF 를 공유하지 않게

# 재탐지 시간 넷은 **파라미터**다 (settle_s · research_s · search_wait_s · visible_wait_s).
#   상수로 두면 실주행 중에 재빌드 없이 못 만진다. 9/9 실주행에서 '놓친 뒤 첫 회전까지 11 s'
#   가 너무 느리다는 것이 드러나 값을 내리면서 같이 파라미터로 뺐다.
#   **lost_s 1.0 과 standby_s 30.0 은 SOT 계약값이라 건드리지 않는다.**

# 재탐색 회전 각도는 **파라미터**다 (search_step_deg). 카메라 수평 시야가 실측 38.5° 라
#   (robot3 9/8: camera_info fx 566.7 × camera_fx_scale 1.7778 = 1007.4, 704 px 정사각 crop
#   → 2·atan(704/(2·1007.4))) 30° 로 돌면 인접 정지점이 8.5° 만 겹친다. 화면 가장자리는
#   bbox 가 잘리고 depth 가 흔들려 검출이 약하므로 **실효 시야는 38.5° 보다 좁다** —
#   그래서 30° 에서는 정지점 사이에 못 보는 구간이 생겼다 (9/9 실주행).
SEARCH_ROTATE_SPEED = 1.5     # rad/s. Nav2 Spin 은 behavior_server max_rotational_vel(1.0)에 묶이고
                              #   액션 왕복 지연도 있어 cmd_vel 로 직접 돌린다
SEARCH_ROTATE_ACC = 6.0       # rad/s^2 명령 감속 프로파일 (실제 가감속은 Create 3 가 자체 제한)
USE_NAV2_SPIN = False         # True 면 Nav2 Spin 사용 (충돌 예측 포함, 대신 느림)
NAV_TIMEOUT_S = 90.0

FOLLOW_REGOAL_M = 0.35        # 표적이 마지막 목표 기준 이만큼 움직였을 때만 새 목표 (재계획 남발 방지)
FOLLOW_REGOAL_DT = 1.0        # 새 목표 최소 간격(s)
FOLLOW_MAX_SPEED = 1.0        # 표적 최대 속도(m/s) 가정 — 이보다 빨리 튄 좌표는 TF 오차로 보고 버림
FOLLOW_EMA = 0.5             # 표적 좌표 지수평활 (1.0 = 평활 없음, 작을수록 부드럽지만 지연)
FOLLOW_TOL_M = 0.15          # 표적이 멈춘 뒤 거리 오차가 이보다 크면 한 번 더 보정
TICK_S = 0.05                # 메인 루프 주기


class FollowTarget(Node):
    def __init__(self):
        super().__init__('follow_target')
        self.ns = self.get_namespace().rstrip('/')
        if not self.ns:
            raise RuntimeError(
                'follow_target: 네임스페이스 없이는 뜰 수 없다 — 로봇 노드는 ns 로 자기 로봇을 정한다.\n'
                '  ros2 launch mro_bringup robot.launch.py namespace:=/robotN\n'
                '  (단독 실행이면 --ros-args -r __ns:=/robotN)\n'
                'ns 로 Nav2 액션 서버를 잡는다 — 틀리면 남의 로봇을 움직인다.')
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

        # ---- 재탐지 시간. 놓친 뒤 첫 회전까지 = research_s + search_wait_s ----
        #   **lost_s 는 더해지지 않는다** — 같은 시계(마지막으로 본 뒤 지난 시간) 안의
        #   앞 단계다. lost_s 에 주행을 멈추고, research_s 에 탐색으로 넘어간다.
        #   9/9 실주행에서 종전 값(8.0 + 3.0 = 11 s)이 너무 느렸다. 아래로 3.5 s 다.
        #   더 줄이려면 각 값의 바닥을 알아야 한다:
        #     research_s     사람이 앞을 지나가는 시간이 1~2 s 다. 이보다 짧으면 **잠깐 가린 것만으로
        #                    로봇이 등을 돌린다** — 작업자가 바로 앞에 있는데 돌아서는 것이 가장 나쁘다
        #     settle_s       30° 회전이 0.5 s 이고 그 뒤 AMCL 방위가 수렴할 시간. 이보다 짧으면
        #                    틀린 좌표로 목표를 낸다
        #     search_wait_s  settle_s + 검출 몇 프레임. 인지가 초당 5~8 장이라 1.5 s 면 8~12 장이다
        #   빨리 도는 것이 안전해진 이유는 **방향을 알게 됐기 때문**이다 (search_rotate_deg).
        self.declare_parameter('research_s', 2.0)           # 이 시간 안 보이면 회전 탐색으로
        self.declare_parameter('settle_s', 1.0)             # 이동·회전 뒤 이 시간 지난 좌표만 신뢰
        self.declare_parameter('search_wait_s', 1.5)        # 회전 후 이만큼 기다려도 못 찾으면 또 회전
        self.declare_parameter('visible_wait_s', 3.0)       # 보이는데 좌표가 걸러지면 이만큼만 기다린다
        self.declare_parameter('search_step_deg', 20.0)     # 재탐색 정지점 간격 (시야 38.5° 보다 작게)

        # ---- 구독 ----
        self.create_subscription(Bool, ARRIVED_TOPIC, self.arrived_cb, 1)
        self.create_subscription(PointStamped, POINT_TOPIC, self.point_cb, 1)
        self.create_subscription(Bool, VISIBLE_TOPIC, self.visible_cb, 1)
        self.create_subscription(Bool, PAIRED_TOPIC, self.paired_cb, 1)
        self.create_subscription(String, WORKER_TOPIC, self.worker_cb, 1)
        self.create_subscription(Bool, FINISH_TOPIC, self.finish_cb, 1)
        self.create_subscription(Bool, ESTOP_TOPIC, self.estop_cb, 1)
        # 힌트만 QoS 가 다르다. 발행자가 BEST_EFFORT 라 기본 RELIABLE 로 잡으면 한 건도 안 온다
        self.create_subscription(TargetHint, HINT_TOPIC, self.hint_cb, qos_profile_sensor_data)

        self.pub_status = self.create_publisher(String, STATUS_TOPIC, 1)

        # ---- 상태 ----
        self.state = 'IDLE'
        # **몇 단계인지 세지 않는다.** 세어 두면 그 값이 실제 거리와 어긋난 채 굳는다
        #   (9/8 실주행: on_paired 가 매번 1 로 되돌려 APPROACH_1 이 무한 반복).
        #   드는 것은 '지금 보낸 목표가 최종 이격인가' 하나뿐이고, 목표를 보낼 때마다
        #   그 시점의 실측 거리로 다시 정해진다 — 굳을 수가 없다
        self.approach_final = False
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
        # **한 번 정한 회전 방향을 끝까지 지킨다.** last_seen_bearing 은 '지금 보는 방향
        #   기준' 이라, 그쪽으로 돌고 나면 그 지점이 0° 근처가 되고 조금 지나치면 음수가
        #   된다 — 매 회차 다시 계산하면 좌우로 왕복한다 (9/9 실주행). 방향을 잠가 두면
        #   같은 쪽으로 계속 훑어 한 바퀴를 덮는다. 힌트가 오면 그것이 다시 정한다.
        self.search_dir = None        # +1 반시계 · -1 시계 · None 아직 안 정함
        self.nav_started = None
        self.hint = None              # (x, y, 수신 시각, worker_id)
        self.worker_started = 0.0     # 작업자 찾기 시작 시각 — 못 찾은 시간을 로그에 적으려고
        self.circles = 0              # 한 바퀴를 몇 번 돌았나
        self.gate_warned = 0.0        # 게이트 대기 경고를 마지막으로 낸 시각
        self.follow_est = None        # (x, y) 평활된 표적 추정 위치
        self.follow_t = 0.0
        self.follow_seen = 0.0        # 마지막으로 표적을 '봤다고 치는' 시각 (Lost 단계 판정 기준)
        # **좌표가 마지막으로 들어온 시각.** follow_seen 과 따로 둔다 — follow_seen 은
        #   '보이면 봤다고 치기' 로 되살려지므로, 그 값으로 '좌표가 얼마나 안 왔나' 를
        #   재면 되살릴 때마다 0 으로 돌아가 조건이 영원히 참이 된다 (I-046 을 고치다 겪음).
        self.coord_seen = 0.0
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
            f'Lost {self.p("lost_s")} s · STANDBY {self.p("standby_s")} s · '
            f'재탐지 {self.p("research_s") + self.p("search_wait_s"):.1f} s '
            f'(대기 {self.p("research_s")} + 확인 {self.p("search_wait_s")})')

    def p(self, name):
        return self.get_parameter(name).value

    def max_search_tries(self):
        """한 바퀴에 필요한 회전 횟수. 정지점 간격에서 계산한다 (못 박지 않는다)."""
        return max(1, round(360.0 / max(self.p('search_step_deg'), 1.0)))

    def set_state(self, state, msg=''):
        """상태를 바꾸고 알린다. 값이 실제로 바뀔 때만 발행한다.

        같은 값을 다시 넣는 호출에서도 발행하면, 상태 기계가 어딘가에서 진동할 때
        그것이 그대로 네트워크 폭주가 된다 (9/9: 772 Hz · 574,501 건). 유지 발행은
        1 초 타이머가 따로 하므로 관제·기록기가 값을 놓치지 않는다.

        같은 상태로 다시 들어오는 것 자체는 정상이다 (재탐색 회차마다 SEARCHING).
        그때 이유가 다르면 알아야 하니 로그는 남기되 5 초에 한 번으로 묶는다.
        """
        changed = state != self.state
        self.state = state
        if changed:
            self.pub_status.publish(String(data=state))
            self.get_logger().info(f'[{state}] {msg}')
        elif msg:
            self.get_logger().info(f'[{state}] {msg}', throttle_duration_sec=5.0)

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

    def hint_cb(self, msg):
        """웹캠2 가 주는 작업자 map 좌표. 방위가 아니라 좌표다 — 회전 부호는 우리가 계산한다."""
        if msg.point.header.frame_id != 'map':
            return
        self.hint = (msg.point.point.x, msg.point.point.y, time.time(), msg.worker_id)

    # ================= 공통 =================

    def rearm(self):
        """이동·회전 직후 호출. 큐에 남은 좌표를 버리고 settle_s 뒤부터 받은 것만 쓴다.

        움직이는 동안 들어온 좌표는 TF 타이밍 때문에 틀려 있다. AMCL 이 수렴할 시간을 준다.
        """
        self.latest = None
        self.recent = []
        self.recent_t = 0.0
        self.accept_after = time.time() + self.p('settle_s')
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
        self.approach_final = False
        self.search_tries = 0
        self.search_dir = None        # 새 사이클이면 방향을 다시 정한다
        self.follow_est = None
        self.follow_goal = None
        self.last_target = None
        self.follow_holding = False
        self.worker_started = 0.0
        self.circles = 0
        self.gate_warned = 0.0
        self.worker_started = time.time()
        # 재탐지 시계의 기준. 이 사이클에서 한 번도 못 찾아도 여기서부터 standby_s 를 잰다
        self.follow_seen = time.time()
        self.rearm()
        self.set_state('SEARCHING', '대기공간 도착 — 작업자를 찾는다 (정차 자세가 이미 작업자 쪽이다)')

    def do_finish(self):
        """mission/finish 처리 (계약 4절 '≤ 1 s'). 정상 종료이므로 FAILED 가 아니라 IDLE 이다."""
        self.finish_req = False
        self.navigator.cancelTask()
        self.paired = False
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
        """작업자 좌표를 일관성 검사로 확정하고 게이트를 통과하면 추종으로 넘어간다.

        보이면 회전하지 않고 좋은 좌표를 기다린다. 안 보인 채 시간이 지나면 30° 씩 돌며 찾는다 —
        천장 카메라 힌트가 있으면 그 방향으로 한 번에 돈다 (FR-17).
        """
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
                # 인지가 초당 8장을 보내므로 그대로 찍으면 판정 로그가 묻힌다
                self.get_logger().info(f'좌표 ({tx:.2f},{ty:.2f}) 수신 — '
                                       f'일관성 확인 중 ({len(self.recent)}/{CONSIST_N})',
                                       throttle_duration_sec=2.0)

        # **여기에 STANDBY 판정이 있어야 한다.** 종전에는 step_following 에만 있었는데,
        #   추종은 research_s 에 무조건 SEARCHING 으로 빠져나오므로 lost 가 standby_s 까지
        #   자랄 수가 없었다 — standby_s 30 s 가 **닿을 수 없는 값**이었다. 그래서 관제는
        #   사람이 필요하다는 것을 영원히 모르고, 재획득 실패가 한 건도 집계되지 않았다
        #   (계약 3절 '재획득 실패면 STANDBY' · 9절 'STANDBY 로 끝난 건은 실패로 센다').
        #   기준은 **마지막으로 본 시각(follow_seen)** 이다. 사이클 시작 시각을 쓰면
        #   30 초 넘게 잘 따라간 뒤 놓친 순간 곧바로 STANDBY 가 되어 버린다 — 오래 따라간
        #   것이 벌이 되면 안 된다. begin_cycle 이 follow_seen 을 지금으로 놓으므로
        #   '한 번도 못 찾은 사이클' 도 30 초를 기준으로 재진다.
        unseen = time.time() - self.follow_seen
        if self.follow_seen and unseen > self.p('standby_s'):
            return self.enter_standby(unseen)

        waited = time.time() - self.search_started
        visible = time.time() - self.visible_at < 1.0
        if waited < self.p('search_wait_s') or (visible and waited < self.p('visible_wait_s')):
            return                                  # 보이면 회전하지 말고 좋은 좌표를 기다린다
        self.search_tries += 1
        if self.search_tries > self.max_search_tries():
            self.search_tries = 0                   # 계속 돈다 — FAILED 로 죽지 않는다
            self.circles += 1
            self.get_logger().error(
                f'★ 작업자를 못 알아봤다 — 한 바퀴 {self.circles} 번째, '
                f'{time.time() - self.worker_started:.0f} s 경과. '
                f'확인할 것: 작업자가 시야에 있는가 · 1 m 안인가 · '
                f'인지 노드가 target/map_point 를 내고 있는가. 탐색은 계속한다')
        deg = self.search_rotate_deg()
        self.rotate_in_place(deg, stop_on_sight=True)
        self.rearm()

    def on_paired(self, tx, ty):
        """게이트 통과. 작업자가 가까우면 접근 없이 바로 추종한다.

        새 시나리오에서는 작업자가 로봇에게 30~50 cm 까지 걸어온다. 그 거리에서 1 m 뒤로
        물러났다 다시 붙는 것은 의미가 없고 최소 이격만 위태롭게 한다. 멀리 있을 때만
        2단계 접근을 쓴다 — 2 m 밖 좌표는 0.5~0.7 m 틀리므로 한 번에 목표를 잡을 수 없다.
        """
        rob = robot_xy(self.tf_buffer, self.get_logger())
        if rob is None:
            self.get_logger().warn('로봇 위치 TF 없음 — 다음 주기에 다시',
                                   throttle_duration_sec=3.0)
            return
        gap = math.hypot(tx - rob[0], ty - rob[1])
        if gap <= self.p('approach_skip_m'):
            self.last_target = (tx, ty)
            self.get_logger().info(f'작업자 확인 ({tx:.2f},{ty:.2f}) {gap:.2f} m — 접근 없이 바로 추종')
            return self.enter_follow()
        self.get_logger().info(f'작업자 확인 ({tx:.2f},{ty:.2f}) {gap:.2f} m — 멀어서 점진 접근')
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

    def last_seen_bearing(self):
        """마지막으로 본 작업자가 로봇 기준 몇 도였나. 모르면 None.

        **천장 카메라가 없어도 방향을 알 수 있다.** 놓치기 직전에 작업자가 어디 있었는지는
        우리가 이미 들고 있다 — 그 방향이 '아무 쪽이나 왼쪽으로' 보다 훨씬 나은 첫 추측이다.
        빠져나간 쪽으로 먼저 돌면 대개 한 번에 다시 잡힌다.

        마지막 목표 위치(follow_goal)와 마지막 추정 위치(follow_est)가 둘 다 있으면
        **그 사이 이동 방향을 더해** 시야 밖으로 나간 쪽을 가리킨다 — 작업자가 오른쪽으로
        걸어 나갔다면 마지막으로 본 자리보다 더 오른쪽에 있다.
        """
        if self.follow_est is None:
            return None
        rob = robot_xy(self.tf_buffer)
        yaw = self.map_yaw()
        if rob is None or yaw is None:
            return None
        ex, ey = self.follow_est
        if self.follow_goal is not None:
            gx, gy = self.follow_goal
            # 이동분을 한 번 더 외삽한다. 크게 잡으면 지나치므로 마지막 이동량만큼만
            ex, ey = ex + (ex - gx), ey + (ey - gy)
        want = math.atan2(ey - rob[1], ex - rob[0])
        diff = math.atan2(math.sin(want - yaw), math.cos(want - yaw))
        return math.degrees(diff)

    def search_rotate_deg(self):
        """돌 각도. 힌트 > 잠근 방향 > 마지막으로 본 쪽 순으로 정한다.

        **한 번 정한 방향은 끝까지 지킨다.** 매 회차 `last_seen_bearing()` 을 다시
        계산하면 좌우로 왕복한다 — 그 값은 '지금 보는 방향 기준' 이라, 그쪽으로 돌고
        나면 목표 지점이 0° 근처가 되고 조금 지나치면 부호가 뒤집힌다. 실제로 9/9
        실주행에서 +30° 를 돈 뒤 −30° 로 되돌아왔다. **한 방향으로 계속 훑으면 한
        바퀴에 반드시 덮는다** (정지점 간격 × 횟수 = 360°).

        힌트는 예외다 — 천장 카메라가 **지금** 보고 있는 정보이므로 방향을 다시 정한다.
        """
        step = self.p('search_step_deg')
        b = self.hint_bearing()
        if b is not None:
            self.search_dir = 1.0 if b >= 0 else -1.0
            deg = b if abs(b) >= step else math.copysign(step, b or 1.0)
            self.get_logger().info(f'힌트 방위 {b:+.0f}° → {deg:+.0f}° 회전 '
                                   f'({self.search_tries}/{self.max_search_tries()})')
            return deg

        if self.search_dir is None:
            seen = self.last_seen_bearing()
            self.search_dir = (1.0 if seen >= 0 else -1.0) if seen is not None else 1.0
            why = (f'마지막으로 본 쪽 {seen:+.0f}°' if seen is not None
                   else '마지막 위치도 없어 반시계부터')
            self.get_logger().info(
                f'작업자 없음 — {why} → {self.search_dir * step:+.0f}° 씩 한 방향으로 훑는다 '
                f'(한 바퀴 {self.max_search_tries()}회) · 천장 카메라 힌트 없음')
        else:
            self.get_logger().info(
                f'작업자 없음 — {self.search_dir * step:+.0f}° 회전 '
                f'({self.search_tries}/{self.max_search_tries()}) · 같은 방향 유지')
        return self.search_dir * step

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

    def rotate_in_place(self, deg, stop_on_sight=False):
        """제자리 회전. 병진이 없고 Nav2 목표를 수행하지 않는 동안에만 부른다.

        블로킹이라 도는 동안 상태기계가 멈춘다. 그래서 estop·mission/finish 가 들어오면
        즉시 끊고 나온다 — finish 는 1 초 안에 멈춰야 한다 (계약 4절).

        `stop_on_sight` 는 **재탐색 회전에만** 쓴다. 도는 중에 작업자가 시야를 스쳐 지나가면
        그 자리에서 멈춘다 — 정지점 사이의 애매한 자리에 서 있으면 돌기 전에도 돌고 나서도
        못 보던 것을 잡는다 (9/9 실주행). 도는 동안 콜백은 계속 돌아가므로(`spin_once`)
        `target/visible` 이 들어온다. 회전 시작 시각 이후에 온 것만 센다 — 이전의 묵은
        참을 보고 곧바로 멈추면 회전이 시작되지 않는다.
        """
        t_start = time.time()
        if USE_NAV2_SPIN or self.setup_cmd_vel() is None:
            self.navigator.spin(spin_dist=math.radians(deg))
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.abort_motion():
                    self.navigator.cancelTask()
                    return
                if stop_on_sight and self.visible_at > t_start:
                    self.get_logger().info('회전 중 작업자 발견 — 그 자리에서 멈춘다')
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
            if stop_on_sight and self.visible_at > t_start:
                self.get_logger().info('회전 중 작업자 발견 — 그 자리에서 멈춘다')
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

    # ---- 점진 접근 (작업자가 멀 때만) ----

    def next_standoff(self, gap):
        """다음 목표를 작업자로부터 얼마 앞에 둘지 — 남은 거리의 절반, 최종 이격이 하한.

        멀리서 잰 좌표는 0.5~0.7 m 틀린다. 그래서 한 번에 최종 목표를 잡지 않고 절반만
        가서 다시 잰다. 가까워질수록 좌표도 정확해지므로 회차마다 목표가 나아진다.

        **단계를 세지 않는 이유가 여기 있다.** 목표는 항상 `max(최종, gap/2)` 이므로
        `gap` 은 회차마다 반으로 줄고, 최종 이격에 닿으면 하한에 걸려 루프가 끝난다.
        회차 수를 못 박지 않으니 3 m 에서 오든 1.2 m 에서 오든 같은 규칙 하나로 수렴한다.
        1단계 목표를 1.0 m 로 고정했던 종전 방식은 gap 1.0~1.25 m 에서 목표가 도착 판정
        (`xy_goal_tolerance` 0.25) 안에 들어가 로봇이 한 발도 안 움직였다 (9/8 실주행).

        하한이 `follow_standoff_m` 인 것은 최소 이격(`min_separation_m` 0.3) 을 접근
        단계에서 침범하지 않기 위한 것이다 — AC-04 오추종·접촉 0건이 절대 기준이다.

        **기본값에서는 APPROACH_2 가 나오지 않는다.** 목표가 최종이 되는 조건은
        `gap <= 2 x follow_standoff_m`(= 1.0) 인데, `gap <= approach_skip_m`(= 1.0) 이면
        접근을 아예 건너뛰고 FOLLOWING 으로 가기 때문에 두 조건이 겹치는 구간이 없다.
        마지막 0.5 m 붙이기를 FOLLOWING 이 맡는 것이므로 기능상 빠지는 것은 없다
        (추종거리 채점 구간도 FOLLOWING 뿐이다 — AC-05). APPROACH_2 는
        `approach_skip_m < 2 x follow_standoff_m` 로 두면 나온다. 값을 그렇게 바꿀지는
        실주행 뒤 판단할 일이라 지금은 계약 4절의 값으로 남겨만 둔다.
        """
        return max(self.p('follow_standoff_m'), gap / 2)

    def send_standoff(self, tx, ty):
        rob = robot_xy(self.tf_buffer, self.get_logger())
        if rob is None:
            self.get_logger().warn('로봇 위치 TF 없음 — 접근 보류')
            return
        gap = math.hypot(tx - rob[0], ty - rob[1])
        standoff = self.next_standoff(gap)
        # 상태 이름은 계약 4절이 정한 8개 그대로 쓰되 뜻을 이렇게 읽는다 —
        #   APPROACH_1 = 중간 목표, APPROACH_2 = 최종 목표. 관제·kpi_logger 는 둘을
        #   구분하지 않고 '접근 중' 집합으로만 쓰므로(fleet_manager APPROACH_ACTIVE_STATES)
        #   회차가 몇 번이든 이 두 값으로 표현된다. 규격을 바꾸지 않는다
        self.approach_final = standoff <= self.p('follow_standoff_m')
        goal, dist, yaw = standoff_goal(*rob, tx, ty, standoff, self.get_clock().now().to_msg())
        if not self.navigator.goToPose(goal):
            self.get_logger().warn('접근 목표 거부됨 — 탐색으로 돌아간다')
            return self.set_state('SEARCHING', '목표 거부 — 재탐색')
        self.nav_started = time.time()
        self.last_target = (tx, ty)
        self.set_state('APPROACH_2' if self.approach_final else 'APPROACH_1',
                       f'작업자 ({tx:.2f},{ty:.2f}) {dist:.2f} m → 앞 {standoff:.2f} m '
                       f'({goal.pose.position.x:.2f},{goal.pose.position.y:.2f},'
                       f'{math.degrees(yaw):.0f}°)'
                       f'{" · 최종" if self.approach_final else " · 중간"}')

    def step_approach(self):
        if not self.gate_ok():
            # 접근 중 페어링이 풀렸다 — 남을 따라가지 않는다 (오추종 0건)
            self.navigator.cancelTask()
            self.rearm()
            return self.set_state('SEARCHING', '접근 중 페어링 해제 — 게이트 재통과 대기')
        if not self.navigator.isTaskComplete():
            if time.time() - self.nav_started > NAV_TIMEOUT_S:
                self.navigator.cancelTask()
                self.rearm()
                self.set_state('SEARCHING', '접근 주행 타임아웃 — 재탐색')
            return
        if self.navigator.getResult().name != 'SUCCEEDED':
            self.rearm()
            return self.set_state('SEARCHING', '접근 주행 실패 — 재탐색')
        if not self.approach_final:
            # 중간 목표에 닿았다 — 여기서 다시 재고 또 좁힌다. 다음 목표 거리는 그때의
            #   실측 gap 이 정한다 (next_standoff). 몇 회차인지는 아무도 세지 않는다
            self.rearm()
            return self.set_state('SEARCHING', '중간 목표 도착 — 정지 후 재검출 대기')
        self.set_state('ARRIVED', f'작업자 앞 {self.p("follow_standoff_m")} m 도착')
        self.enter_follow()

    # ---- 추종 ----

    def enter_follow(self):
        now = time.time()
        self.search_dir = None        # 찾았으니 다음에 놓치면 방향을 새로 정한다
        self.latest = None
        self.follow_est = None
        self.follow_goal = self.last_target   # 이 위치에서 움직여야 새 목표를 낸다
        self.follow_goal_t = now
        self.follow_seen = now
        self.coord_seen = now         # 추종 진입 시점부터 좌표를 기다린다
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
            self.rearm()
            return self.set_state('SEARCHING', '페어링 해제 — 게이트 재통과 대기')

        self.follow_update()
        if self.follow_t > self.follow_seen:
            self.follow_seen = self.follow_t
        if self.follow_t > self.coord_seen:
            self.coord_seen = self.follow_t

        # **보이는데 좌표만 안 오는 구간을 visible_wait_s 까지만 인정한다 (I-046).**
        #   `target/visible`(사람이 화면에 있다)과 `target/map_point`(그 사람이 지도 어디인지
        #   안다)는 다른 신호다. 종전에는 보이기만 하면 무한히 '봤다' 로 쳐서 lost 가 매 주기
        #   0 으로 되돌아갔고, research_s·standby_s 에 영영 닿지 않았다 — 로봇은 좌표가 없어
        #   서 있는데 관제에는 FOLLOWING 으로 뜨고 재획득 실패가 0 건으로 남았다 (계약 9절).
        #   TF 가 끊기면 카메라는 멀쩡하니 정확히 이 조건이 된다 (9/8 실주행에 29 s 기록).
        #   SEARCHING 은 이미 같은 규칙이다 — 보여도 visible_wait_s 를 넘기면 회전한다.
        #   짧은 가림(사람이 앞을 스쳐 지나가 좌표가 걸러지는 것)은 3 초 안에 끝나므로
        #   헛돌지 않고, 그보다 길면 좌표 없이 따라가는 척만 하는 것이라 Lost 로 센다.
        if now - self.visible_at < 1.0 and now - self.coord_seen <= self.p('visible_wait_s'):
            self.follow_seen = max(self.follow_seen, self.visible_at)

        lost = now - self.follow_seen
        if lost > self.p('standby_s'):
            return self.enter_standby(lost)
        if lost > self.p('research_s'):
            self.navigator.cancelTask()
            self.search_tries = 0
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
        # 여기서 '가까우니 최종 목표로 가라' 고 지정하지 않는다 — 재획득 시점의 거리를
        #   next_standoff 가 다시 재서 정한다. 재등장한 작업자가 멀어졌을 수도 있다
        self.begin_reacquire('작업자 재등장 + 게이트 통과 — 재획득 시작')

    def begin_reacquire(self, why):
        """STANDBY 를 벗어나 다시 찾기 시작하며 시계를 지금으로 되돌린다.

        이 한 줄이 없으면 `SEARCHING ↔ STANDBY` 가 초당 수백 번 진동한다 (9/9 실주행):
        STANDBY 에서 작업자가 보여 SEARCHING 으로 나가도 `follow_seen` 이 30 초 전
        값이라, SEARCHING 의 STANDBY 판정이 즉시 다시 STANDBY 로 돌려보낸다.
        574,501 건이 발행돼 관제·기록기로 쏟아졌다.

        `rearm()` 에 넣지 않는 이유 — rearm 은 회전마다 불린다. 거기서 시계를 되돌리면
        회전을 반복하는 동안 30 초가 영원히 안 차서 STANDBY 에 도달하지 못한다.
        되돌릴 자리는 '다시 찾기 시작한 순간' 하나뿐이다.
        """
        self.follow_seen = time.time()
        self.search_tries = 0
        self.search_dir = None        # 재획득이면 방향을 다시 정한다
        self.rearm()
        self.set_state('SEARCHING', why)


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
