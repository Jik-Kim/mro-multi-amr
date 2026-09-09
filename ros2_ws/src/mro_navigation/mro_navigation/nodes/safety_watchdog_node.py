"""온보드 안전 감시 — BR-T20 '인지·웹캠·네트워크와 독립된 정지 경로'의 실물.

/scan 만 본다. 전방 부채꼴(±FRONT_HALF_DEG) 안에 stop_dist 이내 점이 N 프레임 연속이면
  ① safety/estop = True 발행
  ② cmd_vel 에 **병진 0 · 각속도만 상한 통과**를 고주기로 발행 (Nav2 보다 나중 명령이 이기도록)
  ③ Nav2 goal 취소 — TODO([주행]): 액션 클라이언트 미구현 (I-036 ⓓ, docs/todo.md 별도 항목)
장애물이 clear_dist 밖으로 벗어나 M 프레임 연속이면 해제. YOLO·웹캠·Wi-Fi 와 무관해야 한다 (TS-14).

**왜 각속도만 통과시키나 (I-036, 계약 v1.13, PM·안전 결정 9/8)**
  예전에는 estop 중 cmd_vel 에 0 을 통째로 발행해 Nav2 의 복구 행동까지 덮었다. 사람은 비켜주지만
  벽은 비켜주지 않아 **스스로 빠져나올 방법이 없었다** — 9/7 실주행에서 벽 앞에 95 초, 9/8 에도
  P1 통로에서 76 초를 갇혔다.
  · **병진은 전부 0.** 전진은 장애물 쪽이라 막고, **후진은 애초에 쓰지 않는다** (계약 2절 확정).
    Create3 의 범퍼·클리프가 전방에만 있어 후진은 감지 없는 주행이고, 이 노드도 전방만 본다.
  · **각속도만 ESTOP_YAW_RATE_MAX 상한으로 통과.** 제자리 회전이 쓸고 가는 폭은 footprint
    내접 0.175 / 외접 0.189 라 **1.4 cm** 뿐이다. 병진 없는 회전은 안전하고, 후진을 뺀 이상
    이것이 유일한 탈출 수단이다.
  · Nav2 의 의도 속도는 `cmd_vel_smoothed` 에서 읽는다. 배선이
    `controller_server → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed → collision_monitor
    → cmd_vel` 이라, 최종 토픽(cmd_vel)을 구독하면 자기 발행을 되먹는다.

**왜 clear_dist 를 0.45 에서 0.38 로 낮췄나**
  평평한 벽 0.30 m 앞에서 회전만으로 풀려면 약 83° 를 돌아야 하는데(부채꼴 최단거리
  `0.30 / cos(θ − 35°)`), **P1 통로 안에서는 그만큼 돌면 반대쪽 벽이 정면 0.395 m 에 들어온다.**
  0.395 < 0.45 라 어느 방향으로 돌아도 해제 조건이 성립하지 않았다. 0.38 이면 성립하고
  히스테리시스 8 cm 는 RPLIDAR A1 오차 ±2 cm 의 네 배다.

**근본 원인은 안전 계층이 아니라 측위다** — 통로 반폭 0.395 m 에 실측 측위 오차 0.32 m(I-009)를
  더하면 최악 벽까지 0.075 m 다. 여기 조치는 걸렸을 때 빠져나오게 할 뿐이고, 애초에 안 걸리게
  하려면 I-029·I-037 로 측위를 올려야 한다.
"""
import math

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

FRONT_HALF_DEG = 35.0
HIT_FRAMES = 2
CLEAR_FRAMES = 5
USE_TWIST_STAMPED = True   # TB4 Jazzy 는 TwistStamped. Kn1_mini approach 노드와 동일 설정
ZERO_RATE_HZ = 20.0
NAV_CMD_TOPIC = 'cmd_vel_smoothed'   # Nav2 의 '의도한 속도' — collision_monitor 앞단


class SafetyWatchdog(Node):
    """LiDAR만으로 독립 정지를 만드는 온보드 안전 계층.

    ``scan_cb``은 정지 상태를 결정하고, :meth:`tick`은 estop 동안 최종 ``cmd_vel``을
    20 Hz로 덮어쓴다. 인지·웹캠·관제 노드가 죽어도 이 경로는 남아야 한다.
    """

    def __init__(self):
        super().__init__('safety_watchdog')
        # 거리·각속도는 시험 중에 재빌드 없이 만져야 하는 값이라 파라미터로 둔다 (AGENTS.md).
        #   프레임 수(HIT/CLEAR)를 초 단위 파라미터로 바꾸는 것은 docs/todo.md 의 별도 항목이다.
        self.declare_parameter('stop_dist', 0.30)            # BRD 3.2 최소 이격 하한
        self.declare_parameter('clear_dist', 0.38)           # 히스테리시스 (I-036 으로 0.45 → 0.38)
        self.declare_parameter('estop_yaw_rate_max', 0.4)    # estop 중 통과시킬 각속도 상한 [rad/s]
        self.declare_parameter('nav_cmd_valid_s', 0.5)       # Nav2 명령 신선도 — 넘으면 회전도 0
        self.estop = False
        self.hit = 0
        self.clear = 0
        self.nav_yaw = None            # (각속도, 수신 시각)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(TwistStamped if USE_TWIST_STAMPED else Twist,
                                 NAV_CMD_TOPIC, self.nav_cmd_cb, 10)
        self.pub_estop = self.create_publisher(Bool, 'safety/estop', 10)
        self.pub_vel = self.create_publisher(TwistStamped if USE_TWIST_STAMPED else Twist, 'cmd_vel', 10)
        self.create_timer(1.0 / ZERO_RATE_HZ, self.tick)
        self.get_logger().info(
            f"safety_watchdog: stop<{self.p('stop_dist')} m, clear>{self.p('clear_dist')} m, "
            f"±{FRONT_HALF_DEG}° · estop 중 병진 0 · 각속도 ±{self.p('estop_yaw_rate_max')} rad/s 통과")

    def p(self, name):
        """파라미터 값."""
        return self.get_parameter(name).value

    def now_s(self):
        """ROS 시각 [s]. 벽시계를 섞지 않는다 (계약 2절)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def nav_cmd_cb(self, msg):
        """Nav2 가 내려는 속도. estop 중에는 여기서 각속도만 빌려 쓴다."""
        tw = msg.twist if USE_TWIST_STAMPED else msg
        self.nav_yaw = (float(tw.angular.z), self.now_s())

    def front_min(self, scan):
        half = math.radians(FRONT_HALF_DEG)
        best = float('inf')
        ang = scan.angle_min
        for r in scan.ranges:
            if -half <= ang <= half and scan.range_min < r < scan.range_max:
                best = min(best, r)
            ang += scan.angle_increment
        return best

    def scan_cb(self, scan):
        d = self.front_min(scan)
        if d < self.p('stop_dist'):
            self.hit += 1
            self.clear = 0
        elif d > self.p('clear_dist'):
            self.clear += 1
            self.hit = 0
        if not self.estop and self.hit >= HIT_FRAMES:
            self.estop = True
            self.get_logger().warn(f'ESTOP: 전방 {d:.2f} m')
            self.pub_estop.publish(Bool(data=True))
        elif self.estop and self.clear >= CLEAR_FRAMES:
            self.estop = False
            self.get_logger().info('estop 해제')
            self.pub_estop.publish(Bool(data=False))

    def estop_yaw(self):
        """통과시킬 각속도를 돌려준다. 신선한 Nav2 명령이 없으면 0 — 멈춘 채로 둔다."""
        if self.nav_yaw is None:
            return 0.0
        yaw, t = self.nav_yaw
        if self.now_s() - t > self.p('nav_cmd_valid_s'):
            return 0.0                       # Nav2 가 명령을 끊었다 — 회전시킬 근거가 없다
        limit = float(self.p('estop_yaw_rate_max'))
        return max(-limit, min(limit, yaw))

    def tick(self):
        if not self.estop:
            return
        # 병진은 전부 0 으로 덮고 각속도만 상한을 씌워 통과시킨다 (I-036).
        #   Twist 를 새로 만들므로 linear 는 0 으로 남는다 — 후진도 나가지 않는다 (계약 2절).
        yaw = self.estop_yaw()
        if USE_TWIST_STAMPED:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.twist.angular.z = yaw
        else:
            m = Twist()
            m.angular.z = yaw
        self.pub_vel.publish(m)


def main():
    rclpy.init()
    node = SafetyWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
