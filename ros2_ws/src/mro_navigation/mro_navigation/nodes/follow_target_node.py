#!/usr/bin/env python3
"""[이관] Kn1_mini/nav/approach_target_node.py → ros2_ws/src/mro_navigation/mro_navigation/nodes/follow_target_node.py
역할 변경: '차 앞 0.3 m 접근 후 추종' → 'S5 재식별 페어링 → S6~S8 추종·재탐지·Standby'.
유지: 2단계 standoff, SETTLE·일관성 검사, FOLLOW 상태기계(재목표 0.35 m, Lost 3 s 정지, 8 s 회전 탐색).
TODO([주행]): ① FOLLOW_STANDOFF 0.3 → 0.5, 최소 이격 0.3 (BRD 3.2) ② 회전 탐색 방향을 /webcam2/target_hint 방위로 결정 (SEARCH_ROTATE_DEG 부호), 힌트 없으면 기존 동작
③ Lost 30 s → STANDBY 상태 추가, 재등장 시 target/paired 게이트 통과해야 재개 (BR-T17) ④ 접근 시작 조건을 target/paired == True 로 (BR-T16)
⑤ safety/estop == True 동안 goToPose 취소·cmd_vel 발행 금지 ⑥ 모듈 상수 → ROS2 파라미터 (config/navigation.yaml).
"""
"""
approach_target_node.py — [Approach Target]
  /robot3/goto_goal/arrived (Bool) → SEARCHING
  /robot3/target/map_point (PointStamped) → 물체 앞 STANDOFF_FAR 목표 → Nav2 → 도착
  → 정지 후 SETTLE_S 지나서 받은 새 검출로 STANDOFF_NEAR 2단계 → ARRIVED
  /robot3/target/visible (Bool) 가 true 인 동안은 회전하지 않고 좋은 좌표를 기다림
  안 보인 채 SEARCH_WAIT_S 지나면 제자리 SEARCH_ROTATE_DEG 회전 (최대 MAX_SEARCH_TRIES)
  한 바퀴 돌아도 없으면: 웹캠 차 좌표(/robot3/webcam/car_map_point) 앞으로 이동 → 다시 탐색,
  그것도 없으면 SEARCH_WAYPOINTS 를 차례로 방문하며 탐색. 다 실패하면 FAILED
  접근 단계에서는 이동·회전 중 들어온 좌표를 TF 타이밍 때문에 버림 (정지 후 것만 사용)
  도착 후 FOLLOW_ENABLE 이면 FOLLOWING — 표적이 움직이면 STANDOFF_NEAR 를 유지하며 계속 따라감
  (추종 중에는 정지 대기 없이 이동 중 좌표도 사용. 속도 게이트 + 지수평활로 튄 좌표만 거름)
  /robot3/approach/status (String): IDLE / SEARCHING / APPROACH_1 / APPROACH_2 / ARRIVED / FOLLOWING / FAILED
  ARRIVED / FOLLOWING / FAILED 상태에서 arrived 신호를 다시 받으면 새 사이클 (재시작 불필요)

실행: python3 approach_target_node.py --ros-args -r __ns:=/robot3 -r /tf:=tf -r /tf_static:=tf_static
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, TwistStamped
from tf2_ros import Buffer, TransformListener
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

from mro_perception.core.target_utils import robot_xy, standoff_goal

ROBOT_NS = '/robot3'
START_TOPIC = 'goto_goal/arrived'
POINT_TOPIC = 'target/map_point'
VISIBLE_TOPIC = 'target/visible'
STATUS_TOPIC = 'approach/status'
AUTO_START = False           # True: arrived 없이 바로 SEARCHING (단독 테스트용)
STANDOFF_FAR = 1.0
STANDOFF_NEAR = 0.3
SEARCH_WAIT_S = 3.0          # 회전 후 이만큼 기다려도 못 찾으면 다시 회전.
                             # 최소 필요치 = SETTLE_S(2.0) + CONSIST_DT(0.5) + 검출 몇 프레임 ≈ 2.7 s
VISIBLE_WAIT_S = 8.0         # 차는 보이는데 좌표가 계속 걸러지면 이만큼만 기다리고 회전.
                             # 10 fps 로 80회 다 걸렀으면 원인이 지속적(가장자리 bbox·원거리)이라
                             # 기다리는 것보다 30° 돌려 중앙에 놓는 편이 나음 (화각 63.7°)
SETTLE_S = 2.0               # 이동/회전 끝난 뒤 이 시간 지나서 받은 좌표만 신뢰 (AMCL yaw 수렴 대기)
CONSIST_N = 2                # 연속 N개의 검출이
CONSIST_M = 0.3              # 서로 이 거리(m) 안에 있어야 목표로 인정 (TF 타이밍으로 튄 좌표 제거)
CONSIST_DT = 0.5             # 검사에 쓰는 검출 사이 최소 간격(s) — 같은 순간 TF 를 공유하지 않게
SEARCH_ROTATE_DEG = 30.0     # 한 번에 도는 각도 (음수면 시계방향)
SEARCH_ROTATE_SPEED = 1.5    # rad/s — 탐색 회전 각속도. Nav2 Spin 은 behavior_server 의
                             # max_rotational_vel(1.0)에 묶이고 액션 왕복 지연도 있어 cmd_vel 로 직접 돌림
SEARCH_ROTATE_ACC = 6.0      # rad/s^2 — 명령 감속 프로파일 (실제 가감속은 Create 3 가 자체 제한)
USE_NAV2_SPIN = False        # True 면 예전처럼 Nav2 Spin 사용 (충돌 예측 포함, 대신 느림)
MAX_SEARCH_TRIES = 12
NAV_TIMEOUT_S = 90.0
FOLLOW_ENABLE = True         # 도착 후 표적이 움직이면 계속 따라감 (False 면 ARRIVED 에서 종료)
FOLLOW_STANDOFF = STANDOFF_NEAR  # 추종 중 유지할 거리(m)
FOLLOW_REGOAL_M = 0.35       # 표적이 마지막 목표 기준 이만큼 움직였을 때만 새 목표 (Nav2 재계획 남발 방지)
FOLLOW_REGOAL_DT = 1.0       # 새 목표 최소 간격(s)
FOLLOW_MAX_SPEED = 1.0       # 표적 최대 속도(m/s) 가정 — 이보다 빨리 튄 좌표는 TF 오차로 보고 버림
FOLLOW_EMA = 0.5             # 표적 좌표 지수평활 (1.0 = 평활 없음, 작을수록 부드럽지만 지연)
FOLLOW_TOL_M = 0.15          # 표적이 멈춘 뒤 거리 오차가 이보다 크면 한 번 더 보정 (평활 지연·데드밴드 잔차)
FOLLOW_LOST_S = 3.0          # 이 시간 안 보이면 주행 취소하고 제자리 대기
FOLLOW_RESEARCH_S = 8.0      # 이 시간 안 보이면 SEARCHING (회전 탐색) 으로 복귀
WEBCAM_POINT_TOPIC = 'webcam/car_map_point'
WEBCAM_STANDOFF_M = 1.5
SEARCH_WAYPOINTS = []        # [(x, y), ...] 한 바퀴 실패 시 차례로 가서 다시 탐색. 비우면 폴백 없음


class ApproachTarget(Node):
    def __init__(self):
        super().__init__('approach_target')
        ns = self.get_namespace()
        self.ns = ROBOT_NS if ns == '/' else ns
        self.navigator = TurtleBot4Navigator(namespace=self.ns)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Bool, START_TOPIC, self.start_cb, 1)
        self.create_subscription(PointStamped, POINT_TOPIC, self.point_cb, 1)
        self.create_subscription(Bool, VISIBLE_TOPIC, self.visible_cb, 1)
        self.create_subscription(PointStamped, WEBCAM_POINT_TOPIC, self.webcam_cb, 1)
        self.visible_at = 0.0
        self.webcam_point = None      # (x, y) 마지막 웹캠 차 좌표
        self.fallback_idx = 0         # 0 = 웹캠 좌표, 1.. = SEARCH_WAYPOINTS[idx-1]
        self.pub_status = self.create_publisher(String, STATUS_TOPIC, 1)

        self.state = 'SEARCHING' if AUTO_START else 'IDLE'
        self.stage = 1
        self.latest = None            # (x, y, 수신 wall time)
        self.recent = []              # accept_after 이후 받은 좌표들 (일관성 검사용)
        self.recent_t = 0.0           # 마지막으로 recent 에 넣은 시각
        self.accept_after = 0.0       # 이 시각 이후 수신된 검출만 사용 (도착 후 재검출용)
        self.search_started = time.time()
        self.search_tries = 0
        self.nav_started = None
        self.follow_est = None        # (x, y) 평활된 표적 추정 위치
        self.follow_t = 0.0           # follow_est 를 갱신한 검출의 시각
        self.follow_seen = 0.0        # 마지막으로 표적을 본 시각 (visible 또는 좌표)
        self.last_target = None       # 마지막 접근 목표로 쓴 표적 좌표
        self.follow_goal = None       # 마지막으로 목표를 만든 표적 위치
        self.follow_goal_t = 0.0
        self.follow_holding = False   # lost 로 주행을 취소한 상태
        self.cmd_pub = None           # 탐색 회전용 cmd_vel (타입은 첫 사용 때 토픽에서 확인)
        self.cmd_stamped = None
        self.create_timer(1.0, lambda: self.pub_status.publish(String(data=self.state)))
        self.get_logger().info(f'[{self.state}] 시작 신호: {self.ns}/{START_TOPIC}, 목표: {self.ns}/{POINT_TOPIC}')

    def set_state(self, state, msg=''):
        self.state = state
        self.pub_status.publish(String(data=state))
        self.get_logger().info(f'[{state}] {msg}')

    # ---- 콜백: 저장만 ----
    def start_cb(self, msg):
        """IDLE 이면 시작. ARRIVED/FAILED 면 새 사이클 (차를 옮기고 다시 신호 보내면 됨)."""
        if msg.data and self.state in ('IDLE', 'ARRIVED', 'FOLLOWING', 'FAILED'):
            self.stage = 1
            self.search_tries = 0
            self.fallback_idx = 0
            self.rearm()
            self.set_state('SEARCHING', '검출 대기' if self.state == 'IDLE' else '새 사이클 시작')

    def point_cb(self, msg):
        if msg.header.frame_id != 'map':
            return
        self.latest = (msg.point.x, msg.point.y, time.time())

    def webcam_cb(self, msg):
        if msg.header.frame_id == 'map':
            self.webcam_point = (msg.point.x, msg.point.y)

    def visible_cb(self, msg):
        if msg.data:
            self.visible_at = time.time()

    def rearm(self):
        """이동/회전 직후: 큐에 남은 옛 좌표 버리고, SETTLE_S 뒤부터 받은 것만 인정"""
        self.latest = None
        self.recent = []
        self.recent_t = 0.0
        self.accept_after = time.time() + SETTLE_S
        self.search_started = time.time()

    # ---- 메인 루프 ----
    def step(self):
        if self.state == 'SEARCHING':
            self.step_searching()
        elif self.state in ('APPROACH_1', 'APPROACH_2'):
            self.step_approach()
        elif self.state == 'FOLLOWING':
            self.step_following()

    def step_searching(self):
        if self.latest is not None and self.latest[2] >= self.accept_after:
            tx, ty, t = self.latest
            self.latest = None
            if t - self.recent_t < CONSIST_DT:
                return                              # 너무 촘촘한 검출은 건너뜀
            self.recent_t = t
            self.recent = (self.recent + [(tx, ty)])[-CONSIST_N:]
            if len(self.recent) == CONSIST_N and all(
                    math.hypot(x - tx, y - ty) <= CONSIST_M for x, y in self.recent):
                mx = sum(x for x, _ in self.recent) / CONSIST_N
                my = sum(y for _, y in self.recent) / CONSIST_N
                self.search_tries = 0
                return self.send_standoff(mx, my)
            self.get_logger().info(f'좌표 ({tx:.2f},{ty:.2f}) 수신 — 일관성 확인 중 ({len(self.recent)}/{CONSIST_N})')
        waited = time.time() - self.search_started
        visible = time.time() - self.visible_at < 1.0
        if waited < SEARCH_WAIT_S or (visible and waited < VISIBLE_WAIT_S):
            return                                  # 보이면 회전하지 말고 기다림
        self.search_tries += 1
        if self.search_tries > MAX_SEARCH_TRIES:
            return self.search_fallback()
        self.get_logger().info(f'목표 없음 — {SEARCH_ROTATE_DEG:.0f}° 회전 ({self.search_tries}/{MAX_SEARCH_TRIES})')
        self.rotate_in_place(SEARCH_ROTATE_DEG)
        self.rearm()

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
        self.get_logger().info(f'탐색 회전: {name} ({"TwistStamped" if self.cmd_stamped else "Twist"}) '
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
        """제자리 회전. 병진이 없고 Nav2 목표를 수행하지 않는 동안에만 호출하므로 cmd_vel 을 직접 씀."""
        if USE_NAV2_SPIN or self.setup_cmd_vel() is None:
            self.navigator.spin(spin_dist=math.radians(deg))
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.05)
            return
        target = math.radians(abs(deg))
        sign = 1.0 if deg >= 0 else -1.0
        turned, prev = 0.0, self.odom_yaw()
        t0 = time.time()
        deadline = t0 + 3.0 * target / max(SEARCH_ROTATE_SPEED, 0.1) + 3.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if prev is None:                                  # 시작 때 TF 가 없었음 → 시간으로 추정
                turned = (time.time() - t0) * SEARCH_ROTATE_SPEED
            else:
                y = self.odom_yaw()
                if y is not None:                             # 이번 조회 실패면 다음 주기에
                    turned += abs(math.atan2(math.sin(y - prev), math.cos(y - prev)))
                    prev = y
            left = target - turned
            if left <= 0.02:
                break
            # 남은 각도에 맞춰 감속 (Nav2 Spin 과 같은 프로파일). 30° 면 대부분 구간이 최고속.
            self.publish_wz(sign * max(0.4, min(SEARCH_ROTATE_SPEED,
                                                math.sqrt(2.0 * SEARCH_ROTATE_ACC * left))))
        else:
            self.get_logger().warn('탐색 회전 타임아웃 — 정지')
        for _ in range(3):
            self.publish_wz(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def search_fallback(self):
        """한 바퀴 실패 → 웹캠 좌표 앞으로, 그다음 SEARCH_WAYPOINTS 순회. 이동 후 회전 탐색 재시작"""
        rob = robot_xy(self.tf_buffer, self.get_logger())
        goal, label = None, ''
        while goal is None:
            if self.fallback_idx == 0 and self.webcam_point is not None and rob is not None:
                goal, dist, yaw = standoff_goal(*rob, *self.webcam_point, WEBCAM_STANDOFF_M,
                                                self.get_clock().now().to_msg())
                label = f'웹캠 차 좌표 {self.webcam_point} 앞 {WEBCAM_STANDOFF_M} m'
            elif 1 <= self.fallback_idx <= len(SEARCH_WAYPOINTS):
                x, y = SEARCH_WAYPOINTS[self.fallback_idx - 1]
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x, goal.pose.position.y = float(x), float(y)
                goal.pose.orientation.w = 1.0
                label = f'탐색 지점 {self.fallback_idx}/{len(SEARCH_WAYPOINTS)} ({x}, {y})'
            elif self.fallback_idx > len(SEARCH_WAYPOINTS):
                return self.set_state('FAILED', '한 바퀴 + 폴백 지점 전부 탐색했으나 목표 없음')
            self.fallback_idx += 1
        self.get_logger().info(f'목표 없음 → {label} 로 이동 후 재탐색')
        if not self.navigator.goToPose(goal):
            return self.set_state('FAILED', '폴백 이동 목표 거부됨')
        t0 = time.time()
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > NAV_TIMEOUT_S:
                self.navigator.cancelTask()
                break
        self.search_tries = 0
        self.rearm()

    def send_standoff(self, tx, ty):
        rob = robot_xy(self.tf_buffer, self.get_logger())
        if rob is None:
            return self.set_state('FAILED', '로봇 위치 TF 없음')
        standoff = STANDOFF_FAR if self.stage == 1 else STANDOFF_NEAR
        goal, dist, yaw = standoff_goal(*rob, tx, ty, standoff, self.get_clock().now().to_msg())
        if not self.navigator.goToPose(goal):
            return self.set_state('FAILED', '목표 거부됨')
        self.nav_started = time.time()
        self.last_target = (tx, ty)
        self.set_state(f'APPROACH_{self.stage}',
                       f'물체 ({tx:.2f},{ty:.2f}) {dist:.2f} m → 앞 {standoff} m '
                       f'({goal.pose.position.x:.2f},{goal.pose.position.y:.2f},{math.degrees(yaw):.0f}°)')

    def step_approach(self):
        if not self.navigator.isTaskComplete():
            if time.time() - self.nav_started > NAV_TIMEOUT_S:
                self.navigator.cancelTask()
                self.set_state('FAILED', '주행 타임아웃')
            return
        ok = self.navigator.getResult().name == 'SUCCEEDED'
        if not ok:
            return self.set_state('FAILED', f'{self.stage}단계 주행 실패')
        if self.stage == 1:
            self.stage = 2
            self.rearm()
            self.set_state('SEARCHING', '1단계 도착 — 정지 후 재검출 대기')
        else:
            self.set_state('ARRIVED', f'목표 앞 {STANDOFF_NEAR} m 도착')
            if FOLLOW_ENABLE:
                self.enter_follow()

    # ---- 추종 ----
    def enter_follow(self):
        """도착 시점부터 추종 시작. 큐에 남은 옛 좌표는 버리고 새로 받는 것만 씀."""
        now = time.time()
        self.latest = None
        self.follow_est = None
        self.follow_goal = self.last_target   # 도착 시점의 표적 위치가 기준 — 여기서 움직여야 따라감
        self.follow_goal_t = now
        self.follow_seen = now
        self.follow_holding = False
        self.set_state('FOLLOWING',
                       f'정지 — 표적이 {FOLLOW_REGOAL_M} m 이상 움직이면 {FOLLOW_STANDOFF} m 를 유지하며 따라감')

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
            self.get_logger().warn(f'좌표 ({tx:.2f},{ty:.2f}) 튐 — {dt:.1f}s 만에 너무 멀리 이동, 버림')
            return
        a = FOLLOW_EMA
        self.follow_est = (ex + a * (tx - ex), ey + a * (ty - ey))
        self.follow_t = t

    def step_following(self):
        now = time.time()
        self.follow_update()
        if self.follow_t > self.follow_seen:
            self.follow_seen = self.follow_t
        if now - self.visible_at < 1.0:
            self.follow_seen = max(self.follow_seen, self.visible_at)

        lost = now - self.follow_seen
        if lost > FOLLOW_RESEARCH_S:
            self.navigator.cancelTask()
            self.stage = 2
            self.search_tries = 0
            self.fallback_idx = 0
            self.rearm()
            return self.set_state('SEARCHING', f'표적 {lost:.0f}s 안 보임 — 재탐색')
        if lost > FOLLOW_LOST_S:
            if not self.follow_holding:
                self.navigator.cancelTask()
                self.follow_holding = True
                self.get_logger().info(f'표적 {FOLLOW_LOST_S:.0f}s 안 보임 — 정지하고 대기')
            return

        if self.follow_est is None or now - self.follow_goal_t < FOLLOW_REGOAL_DT:
            return
        rob = robot_xy(self.tf_buffer, self.get_logger())
        if rob is None:
            return
        parked = self.navigator.isTaskComplete()
        if parked and self.follow_goal is not None and self.navigator.getResult().name != 'SUCCEEDED':
            self.get_logger().warn('추종 주행 실패 — 목표 다시 보냄')
            self.follow_goal = None        # 성공 도착이면 유지 (정지한 표적에 goal 남발 방지)
        moved = (math.inf if self.follow_goal is None
                 else math.hypot(self.follow_est[0] - self.follow_goal[0],
                                 self.follow_est[1] - self.follow_goal[1]))
        # 멈춰 있는데 표적까지 STANDOFF 보다 멀면 잔차 보정 (가까운 쪽은 후진하지 않으므로 그대로 둠)
        gap = math.hypot(self.follow_est[0] - rob[0], self.follow_est[1] - rob[1]) - FOLLOW_STANDOFF
        if moved < FOLLOW_REGOAL_M and not (parked and gap > FOLLOW_TOL_M):
            return
        goal, dist, yaw = standoff_goal(*rob, *self.follow_est, FOLLOW_STANDOFF,
                                        self.get_clock().now().to_msg())
        if not self.navigator.goToPose(goal):      # 새 goal 이 이전 goal 을 선점
            self.get_logger().warn('추종 목표 거부됨')
            return
        self.follow_goal = self.follow_est
        self.follow_goal_t = now
        self.follow_holding = False
        self.get_logger().info(f'추종: 표적 ({self.follow_est[0]:.2f},{self.follow_est[1]:.2f}) '
                               f'{dist:.2f} m → 앞 {FOLLOW_STANDOFF} m')


def main():
    rclpy.init()
    node = ApproachTarget()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.step()
    except KeyboardInterrupt:
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
