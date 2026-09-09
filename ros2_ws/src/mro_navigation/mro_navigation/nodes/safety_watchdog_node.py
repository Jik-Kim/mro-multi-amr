"""LiDAR 독립 정지·SAFE_ROTATE와 최종 cmd_vel 출력 선택 (I-049).

Nav2 최종 출력은 cmd_vel_navigation, 추종 회전은 cmd_vel_search로 입력한다.
이 노드만 cmd_vel을 발행해야 한다. safety_nav2.launch.py와 함께 사용한다.
정지 확인을 받기 전에는 회전하지 않으며 실패 후에는 정지 상태를 유지한다.
"""
import math
import time

from geometry_msgs.msg import TwistStamped
from irobot_create_msgs.msg import DockStatus
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Header, String

from mro_navigation.core.safety import SafetyController, scan_sectors


CLEAR_FLUSH_S = 0.1     # CLEAR 전이 직후 이만큼은 명령을 버린다 — 전이 순간에 걸친 한 두 건
PAUSE_FORGET_S = 5.0    # 정지 ACK 가 이만큼 없으면 그 노드는 떠난 것으로 본다


class SafetyWatchdog(Node):
    """최종 속도 발행자는 하나이고, ESTOP에는 외부 명령을 버린다."""

    def __init__(self):
        super().__init__('safety_watchdog')
        if not self.get_namespace().strip('/'):
            raise RuntimeError('safety_watchdog: 로봇 네임스페이스가 필요하다')
        defaults = {'stop_dist': 0.30, 'clear_dist': 0.38,
                    'safe_rotate_rate': 0.25, 'safe_rotate_timeout_s': 5.0,
                    'safe_rotate_max_angle_deg': 75.0, 'odom_valid_s': 0.5,
                    'stopped_linear_mps': 0.01, 'stopped_angular_rps': 0.02,
                    'stopped_hold_s': 0.3, 'scan_valid_s': 0.5, 'pause_timeout_s': 2.0,
                    'rotation_clearance_m': 0.21, 'nav_cmd_valid_s': 0.5,
                    'safety_participants': ['follow_target', 'shuttle_mission', 'return_dock']}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.control = SafetyController(
            self.p('stop_dist'), self.p('clear_dist'), self.p('safe_rotate_rate'),
            self.p('safe_rotate_timeout_s'), self.p('scan_valid_s'),
            self.p('pause_timeout_s'), self.p('rotation_clearance_m'),
            self.p('safe_rotate_max_angle_deg'), self.p('odom_valid_s'),
            self.p('stopped_linear_mps'), self.p('stopped_angular_rps'),
            self.p('stopped_hold_s'))
        self.participants = list(self.p('safety_participants'))
        if not self.participants or len(set(self.participants)) != len(self.participants):
            raise ValueError('safety_participants는 중복 없는 주행 노드 목록이어야 한다')
        self.pause_epoch = self.get_clock().now().nanoseconds
        self.paused = {}
        self.clear_at = float('-inf')      # 마지막 CLEAR 전이 시각 (monotonic)
        self.search_at = float('-inf')
        self.epoch = self.control.epoch
        self.last_state = None
        self.last_command_at = float('-inf')
        self.pub_estop = self.create_publisher(Bool, 'safety/estop', 10)
        self.pub_state = self.create_publisher(String, 'safety/state', 10)
        self.pub_vel = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, qos_profile_sensor_data)
        # 도킹 여부. 도크에 물려 있는 동안 전방 판정을 보류한다 — Create3 는 코를 박고
        #   들어가므로 도크가 곧 전방 장애물이고, 그것을 세면 언독이 불가능해진다.
        #   TRANSIENT_LOCAL 이라 늦게 떠도 마지막 값을 받는다
        self.create_subscription(DockStatus, 'dock_status', self.dock_cb,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, 'odom', self.odom_cb, qos_profile_sensor_data)
        for source in ('navigation', 'search'):
            self.create_subscription(TwistStamped, 'cmd_vel_' + source,
                                     lambda m, s=source: self.command_cb(s, m), 10)
        for name in self.participants:
            self.create_subscription(Header, 'safety/paused/' + name,
                                     lambda m, n=name: self.pause_cb(n, m), 10)
        self.create_timer(0.05, self.tick)

    def p(self, name):
        return self.get_parameter(name).value

    def pause_cb(self, name, message):
        stamp = message.stamp.sec * 1000000000 + message.stamp.nanosec
        self.paused[name] = (stamp, time.monotonic())

    def dock_cb(self, message):
        self.control.dock_state(message.is_docked)

    def scan_cb(self, message):
        now = time.monotonic()
        was = self.control.estop
        # **헤더 시각으로 신선도를 재지 않는다.** 헤더는 로봇(Create3) 시계, 비교 기준은 PC 시계다.
        #   둘이 0.5 s 만 어긋나도 모든 스캔이 '오래됨' 이 되어 front 가 영원히 None → STUCK 영구.
        #   '데이터가 안 온다' 는 수신 시각(monotonic)으로 core 가 scan_valid_s 로 판정한다 —
        #   그것이 실제 고장 모드이고 시계 어긋남에 영향받지 않는다
        sectors = scan_sectors(message)
        self.control.scan(now, *sectors)
        if not was and self.control.estop:
            self.pause_epoch = self.get_clock().now().nanoseconds

    def odom_cb(self, message):
        q = message.pose.pose.orientation
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if not math.isfinite(norm) or abs(norm - 1.0) > 0.01:   # 헤더 시각은 보지 않는다 (scan_cb 참조)
            self.control.invalidate_odometry()
            return
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        twist = message.twist.twist
        linear = math.hypot(twist.linear.x, twist.linear.y, twist.linear.z)
        angular = math.hypot(twist.angular.x, twist.angular.y, twist.angular.z)
        self.control.odometry(time.monotonic(), yaw, linear, angular)


    def all_paused(self, now):
        """정지 확인. **떠 있는 노드만 요구한다.**

        종전에는 `safety_participants` 세 개를 전부 요구했다. 그러면 `return_dock` 을 안
        띄운 구성(셔틀+추종 통합 시험)에서 ACK 가 영원히 모자라 `STOPPING` 에서 못 나오고
        **estop 이 영구히 True** 가 된다 — 스캔이 멀쩡한데 노드 하나가 없어서 못 움직이는
        것은 안전이 아니라 가동 불능이다 (9/10 실주행).

        그래서 **한 번이라도 ACK 를 보낸 노드만** 센다. 액션을 돌리는 노드는 estop 동안
        0.1 s 마다 ACK 를 내므로(`MotionPause.publish`) 떠 있으면 반드시 여기에 등록된다.
        떠 있는데 ACK 가 끊긴 것은 그대로 실패로 잡힌다 — 그 노드가 아직 멈추지 않았다는 뜻이다.
        """
        # 오래 침묵한 노드는 죽은 것으로 보고 뺀다. 죽은 노드는 cmd_vel 을 낼 수 없으니
        #   그 ACK 를 기다리는 것은 안전이 아니라 영구 정지다. 잠깐 멈춘 노드(회전 블로킹 등)는
        #   spin_once 로 타이머가 돌아 ACK 가 이어지므로 여기 안 걸린다
        for name in [n for n, (_, at) in self.paused.items() if now - at > PAUSE_FORGET_S]:
            self.get_logger().warn(f'{name} 의 정지 ACK 가 {PAUSE_FORGET_S:.0f} s 넘게 없다 — 떠난 것으로 본다')
            del self.paused[name]
        known = [name for name in self.participants if name in self.paused]
        if not known:
            return True          # 아직 아무도 액션을 돌리지 않았다 — 멈출 것이 없다
        return all(self.paused[name][0] >= self.pause_epoch
                   and 0 <= now - self.paused[name][1] <= 0.5
                   for name in known)

    def command_cb(self, source, message):
        now = time.monotonic()
        # estop 중 도착한 명령은 여기서 버려진다 — 큐에 쌓이지 않으므로 CLEAR 뒤 밀린 명령이
        #   쏟아지는 일은 없고, 전이 순간의 한 두 건만 CLEAR_FLUSH_S 로 거른다.
        #   종전에는 헤더 시각을 CLEAR 시각과 비교했는데, 시계가 어긋나면 **모든 명령이
        #   영원히 버려져 CLEAR 인데도 로봇이 안 움직인다** — 겉으로는 정상인 완전 정지다
        if self.control.estop or now - self.clear_at < CLEAR_FLUSH_S:
            return
        # 탐색 종료의 0 명령도 잠시 우선해 늦은 Nav2 출력과의 경합을 막는다.
        if source == 'search':
            self.search_at = now
        elif now - self.search_at < 0.15:
            return
        values = (message.twist.linear.x, message.twist.linear.y, message.twist.linear.z,
                  message.twist.angular.x, message.twist.angular.y, message.twist.angular.z)
        if not all(math.isfinite(v) for v in values) or message.twist.linear.x < 0:
            return
        # 힌트 회전의 속도·발행 주기는 바꾸지 않고 즉시 전달한다.
        self.pub_vel.publish(message)
        self.last_command_at = now

    def tick(self):
        now = time.monotonic()
        was = self.control.estop
        yaw = self.control.tick(now, self.all_paused(now))
        if not was and self.control.estop:
            self.pause_epoch = self.get_clock().now().nanoseconds
        if self.epoch != self.control.epoch:
            self.epoch = self.control.epoch
            self.clear_at = now
            self.search_at = self.last_command_at = float('-inf')
        if yaw is not None or now - self.last_command_at > self.p('nav_cmd_valid_s'):
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'base_link'
            message.twist.angular.z = yaw or 0.0
            self.pub_vel.publish(message)
        self.pub_estop.publish(Bool(data=self.control.estop))
        self.pub_state.publish(String(data=self.control.state))
        if self.last_state != self.control.state:
            self.get_logger().warn(f'안전 상태 {self.control.state}: {self.control.reason}')
            self.last_state = self.control.state


def main():
    rclpy.init()
    node = SafetyWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
