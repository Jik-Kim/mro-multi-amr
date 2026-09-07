"""구역 세마포어 — 협소 구역을 한 번에 한 대만 점유 (E-3 교착, E-7 도크 경합 / BR-T09, BR-T31 / R-09).

서비스 /fleet/zone_token: acquire=True 면 비어 있을 때만 granted. 먼저 요청한 로봇 우선(FIFO 대기열).
보유 로봇이 HOLD_TIMEOUT 안에 반납하지 않으면 강제 회수하고 이벤트 기록 (교착 자동 해결률 K-06 산출 근거).
"""
import time

import rclpy
from rclpy.node import Node
from mro_interfaces.srv import ZoneToken

# 9/5 실측 아레나 기준으로 정정. 구역은 둘뿐이다 — 로봇끼리만 경합하는 곳에만 토큰을 둔다.
#   P1   : 로봇 구역(도크·창고) ↔ 중앙 복도 병목(0.79 m). 도크 → 창고는 벽 없이 이어져 지나지 않는다.
#          S4 창고에서 대기공간으로 나올 때 · S10 복귀할 때 두 번 통과 → E-3 교착
#   DOCK : 도크 구역. 복귀 시 한 번에 한 대만 진입 → E-7 순차 도킹
# 추종 경로(중앙 복도 → 작업공간, 0.93 m)에는 토큰을 두지 않는다 —
#   토큰을 기다리며 멈추면 작업자를 놓친다. 두 쌍의 출발을 엇갈리게 해서 회피한다.
ZONES = ['P1', 'DOCK']   # bringup/params/arena.yaml 의 semaphore_zones 와 ID 일치
HOLD_TIMEOUT = 60.0           # s, BRD 5.3 '교착 자동 해결' 판정 시간 [팀 가정]


class ZoneSemaphore(Node):
    def __init__(self):
        super().__init__('zone_semaphore')
        self.holder = {z: None for z in ZONES}
        self.since = {z: 0.0 for z in ZONES}
        self.queue = {z: [] for z in ZONES}
        self.create_service(ZoneToken, '/fleet/zone_token', self.cb)
        self.create_timer(1.0, self.watchdog)

    def cb(self, req, res):
        z = req.zone_id
        if z not in self.holder:
            res.granted, res.holder = False, ''
            return res
        if not req.acquire:
            if self.holder[z] == req.robot_ns:
                self.holder[z] = None
                self.get_logger().info(f'{z} 반납 ← {req.robot_ns}')
            # 토큰을 못 받고 포기한 로봇도 반납을 부른다 (계약 3절: 성공·실패와 무관하게 반드시 반납).
            #   대기열에서 빼지 않으면 그 로봇이 FIFO 선두로 남아 다른 로봇이 영구히 거절된다.
            #   watchdog 은 holder 만 비우므로 자동 복구도 되지 않는다.
            if req.robot_ns in self.queue[z]:
                self.queue[z].remove(req.robot_ns)
                self.get_logger().info(f'{z} 대기열에서 제거 ← {req.robot_ns}')
            res.granted, res.holder = True, ''
            return res
        if self.holder[z] in (None, req.robot_ns):
            # FIFO: 대기열 선두가 아니면 거절
            if self.queue[z] and self.queue[z][0] != req.robot_ns:
                res.granted, res.holder = False, self.holder[z] or ''
                return res
            if req.robot_ns in self.queue[z]:
                self.queue[z].remove(req.robot_ns)
            if self.holder[z] != req.robot_ns:      # 새로 획득할 때만 보유 시계를 시작한다
                self.since[z] = time.time()         #   보유자가 재요청해도 HOLD_TIMEOUT 이 리셋되지 않게
                self.get_logger().info(f'{z} 획득 → {req.robot_ns}')
            self.holder[z] = req.robot_ns
            res.granted, res.holder = True, req.robot_ns
            return res
        if req.robot_ns not in self.queue[z]:
            self.queue[z].append(req.robot_ns)
        res.granted, res.holder = False, self.holder[z]
        return res

    def watchdog(self):
        for z, h in self.holder.items():
            if h and time.time() - self.since[z] > HOLD_TIMEOUT:
                # 회수는 로봇이 구역을 실제로 빠져나왔는지 보지 않는다. 정상 통과는 10~20 s(P1 이동만 3~9 s)라
                #   60 s 를 넘는 건 estop 이 오래 걸린 때 정도다. 그때 토큰만 넘어가고 앞 로봇은 아직 안에 있는데,
                #   뒤 로봇은 costmap·collision_monitor·safety_watchdog 에 막혀 충돌하지 않고 앞에서 선다.
                #   즉 증상은 충돌이 아니라 '토큰은 받았는데 못 들어감' 이다 — TS-08 에서 이게 보이면 이 줄을 볼 것.
                #   갱신(heartbeat) 규칙은 계약에 없어 두지 않는다 (9/7 팀 판단).
                # TODO([주행]): 이벤트로도 남겨야 K-06(교착 자동 해결률)을 셀 수 있다 — 지금은 로그뿐이다.
                #   계약 3절이 '이벤트를 남긴다' 고만 적고 토픽·타입을 정의하지 않았다 (I-030).
                self.get_logger().error(f'{z} {h} 보유 {HOLD_TIMEOUT}s 초과 — 강제 회수 (교착 의심)')
                self.holder[z] = None


def main():
    rclpy.init()
    node = ZoneSemaphore()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
