"""온보드 안전 감시 — BR-T20 '인지·웹캠·네트워크와 독립된 정지 경로'의 실물.

/scan 만 본다. 전방 부채꼴(±FRONT_HALF_DEG) 안에 STOP_DIST 이내 점이 N 프레임 연속이면
  ① safety/estop = True 발행  ② cmd_vel 에 0 을 고주기로 발행 (Nav2 controller 보다 나중 명령이 이기도록)
  ③ Nav2 goal 취소 (navigate_to_pose 액션 cancel — TODO: 액션 클라이언트로 진행 중 goal 취소)
장애물이 CLEAR_DIST 밖으로 벗어나 M 프레임 연속이면 estop 해제. YOLO·웹캠·Wi-Fi 상태와 무관하게 동작해야 한다 (TS-14).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool

STOP_DIST = 0.30        # m, BRD 3.2 최소 이격 하한
CLEAR_DIST = 0.45       # 히스테리시스
FRONT_HALF_DEG = 35.0
HIT_FRAMES = 2
CLEAR_FRAMES = 5
USE_TWIST_STAMPED = True   # TB4 Jazzy 는 TwistStamped. Kn1_mini approach 노드와 동일 설정
ZERO_RATE_HZ = 20.0


class SafetyWatchdog(Node):
    def __init__(self):
        super().__init__('safety_watchdog')
        self.estop = False
        self.hit = 0
        self.clear = 0
        self.create_subscription(LaserScan, 'scan', self.scan_cb, qos_profile_sensor_data)
        self.pub_estop = self.create_publisher(Bool, 'safety/estop', 10)
        self.pub_vel = self.create_publisher(TwistStamped if USE_TWIST_STAMPED else Twist, 'cmd_vel', 10)
        self.create_timer(1.0 / ZERO_RATE_HZ, self.tick)
        self.get_logger().info(f'safety_watchdog: stop<{STOP_DIST} m, clear>{CLEAR_DIST} m, ±{FRONT_HALF_DEG}°')

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
        if d < STOP_DIST:
            self.hit += 1
            self.clear = 0
        elif d > CLEAR_DIST:
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

    def tick(self):
        if not self.estop:
            return
        if USE_TWIST_STAMPED:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
        else:
            m = Twist()
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
