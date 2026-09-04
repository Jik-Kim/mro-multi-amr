"""작업 종료 → 페어링 해제 → 도킹 복귀 (S9 / BR-T29).

입력: mission/finish (Bool)  ← fleet_manager 가 관제 UI '작업 종료' 를 로봇 네임스페이스로 전달
출력: dock 액션 (TurtleBot4Navigator), shuttle/status = 'DOCKING' / 'DOCKED'
규칙: 콜백 안에서 navigator 를 부르지 않는다 (Kn1_mini 교훈) — 플래그만 세우고 메인 루프에서 실행.
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator


class ReturnDock(Node):
    def __init__(self):
        super().__init__('return_dock')
        self.flag = False
        self.create_subscription(Bool, 'mission/finish', self.cb, 10)
        self.pub = self.create_publisher(String, 'shuttle/status', 10)

    def cb(self, msg):
        if msg.data:
            self.flag = True


def main():
    rclpy.init()
    node = ReturnDock()
    nav = TurtleBot4Navigator()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.flag:
                node.flag = False
                node.pub.publish(String(data='DOCKING'))
                # TODO([주행]): follow_target_node 에 페어링 해제 신호 (target/paired False) 후 복귀
                nav.dock()
                node.pub.publish(String(data='DOCKED'))
            time.sleep(0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
