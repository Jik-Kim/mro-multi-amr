#!/usr/bin/env python3
"""작업 종료 → 즉시 추종 중단 → 순차 복귀·도킹 (S9·S10 / BR-T29, BR-T31).

입력: mission/finish (Bool)  ← fleet_manager 가 관제 UI 의 작업자별/전체 종료를 로봇 네임스페이스로 전달
출력: dock 액션 (TurtleBot4Navigator), shuttle/status (RETURNING/DOCK_WAIT/DOCKING/DOCKED/FAILED)
협조: /fleet/zone_token 의 'Z-03'(도크 구역) 토큰 — 한 번에 한 대만 도크에 진입한다.
      토큰을 못 얻으면 홀딩 포인트(도크에서 ≥ 1.5 m)로 가서 대기 → Create3 도크 IR 간섭도 함께 회피.

규칙: 콜백 안에서 navigator 를 부르지 않는다 (Kn1_mini 교훈) — 플래그만 세우고 메인 루프에서 실행.
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from mro_interfaces.srv import ZoneToken

DOCK_ZONE = 'Z-03'
TOKEN_RETRY_S = 2.0          # 토큰 거절 시 재요청 주기
# 도크 중심 간 0.9 m (실측 2026-09-05). 좌우 여유가 각 0.45 m 뿐이라 approach 경유 + IR 차폐를 전제로 한다.


class ReturnDock(Node):
    def __init__(self):
        super().__init__('return_dock')
        self.declare_parameter('dock_retry', 2)
        self.declare_parameter('t_ret_s', 180.0)
        # TODO([주행]): arena.yaml 의 dock_zone.holding / dock_zone.approach[robot_ns] 를 파라미터로 읽어
        #   ① 토큰 대기 중에는 holding 으로, ② 토큰 획득 후 자기 도크 정면 approach 로 goToPose 한 뒤 dock() 을 부른다.
        #   도크 간격이 0.9 m(실측) 뿐이라 멀리서 dock() 을 부르면 옆 도크로 향할 수 있다 — approach 경유는 선택이 아니라 필수.
        self.flag = False
        self.ns = self.get_namespace().rstrip('/') or '/robot3'
        self.create_subscription(Bool, 'mission/finish', self.cb, 10)
        self.pub = self.create_publisher(String, 'shuttle/status', 10)
        self.cli = self.create_client(ZoneToken, '/fleet/zone_token')

    def cb(self, msg):
        if msg.data:
            self.flag = True     # 실제 동작은 메인 루프. 추종 중단은 follow_target 이 같은 토픽으로 처리

    def status(self, s):
        self.pub.publish(String(data=s))
        self.get_logger().info(f'[{s}]')

    def token(self, acquire):
        """도크 구역 토큰 획득/반납. 서비스가 없으면 (관제 미기동) True 로 통과시켜 단독 시험을 막지 않는다."""
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('zone_token 서비스 없음 — 단독 모드로 진행')
            return True
        req = ZoneToken.Request()
        req.robot_ns, req.zone_id, req.acquire = self.ns, DOCK_ZONE, acquire
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        res = fut.result()
        return bool(res and res.granted)


def main():
    rclpy.init()
    node = ReturnDock()
    nav = TurtleBot4Navigator()
    retry_max = node.get_parameter('dock_retry').value
    t_ret = node.get_parameter('t_ret_s').value
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.flag:
                time.sleep(0.02)
                continue
            node.flag = False
            t0 = time.time()

            # S9: 진행 중 주행 취소. 추종 중단(페어링 해제)은 follow_target 이 mission/finish 로 처리
            nav.cancelTask()
            node.status('RETURNING')

            # S10: 도크 구역 토큰 — 없으면 홀딩 포인트에서 대기 (후행 로봇)
            if not node.token(True):
                node.status('DOCK_WAIT')
                # TODO([주행]): 여기서 홀딩 포인트로 goToPose 한 뒤 대기할 것 (지금은 제자리 대기)
                while rclpy.ok() and not node.token(True):
                    if time.time() - t0 > t_ret:
                        node.status('FAILED')
                        break
                    rclpy.spin_once(node, timeout_sec=TOKEN_RETRY_S)
                else:
                    pass

            if time.time() - t0 > t_ret:
                node.status('FAILED')
                node.token(False)
                continue

            # 도킹 — 실패 시 이탈 후 재시도, 결과와 무관하게 토큰은 반드시 반납
            ok = False
            for attempt in range(retry_max + 1):
                node.status('DOCKING' if attempt == 0 else f'DOCKING(재시도 {attempt})')
                nav.dock()
                if nav.getDockedStatus():
                    ok = True
                    break
                node.get_logger().warn(f'도킹 실패 {attempt + 1}/{retry_max + 1} — 이탈 후 재시도')
                nav.undock()
            node.token(False)
            node.status('DOCKED' if ok else 'FAILED')
            node.get_logger().info(f'복귀 소요 {time.time() - t0:.1f} s')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
