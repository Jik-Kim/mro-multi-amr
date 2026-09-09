#!/usr/bin/env python3
"""
[노드 개요: ZoneSemaphore]
구역 세마포어 — 좁아서 두 대가 동시에 못 지나가는 곳을 한 번에 한 대만 점유하게 한다.
(E-3 병목 교착, E-7 도크 경합 / BR-T09, BR-T31 / R-09)

1. 서비스 대기: /fleet/zone_token  (요청 필드 = robot_ns, zone_id, acquire)
2. 획득 요청(acquire=True):
   - 1순위: 구역이 비어 있고 대기열 선두이면 -> granted=True (내가 보유)
   - 2순위: 이미 내가 보유 중이면 -> granted=True (보유 시계는 리셋하지 않음)
   - 3순위: 다른 로봇이 보유 중이면 -> granted=False + FIFO 대기열에 등록
3. 반납 요청(acquire=False): 보유자면 비우고, 대기 중이었으면 대기열에서 뺀다.
   로봇은 토큰을 받았든 못 받았든 반드시 반납을 부른다 (계약 3절).
4. 감시(1 Hz watchdog): 보유가 HOLD_TIMEOUT 초를 넘으면 강제 회수 -> 교착 자동 복구
5. 이벤트 발행: /fleet/zone_event 로 GRANT / REJECT / RELEASE / RECLAIM 한 줄씩 -> K-06 산출 원천

[구역 진입 판정 방식 — 방법 B (PM 결정 9/7)]
폴리곤 안에 로봇이 있는지 좌표로 계산하지 '않는다'. 로봇 측 노드가
"구역에 들어가기 직전 획득 -> 나온 직후 반납" 순서를 지키는 것으로 상호배제가 성립한다.
그래서 이 노드는 시나리오도, 로봇 위치도, 아레나 좌표도 모른다. 잠금 서버일 뿐이다.
(arena.yaml 의 폴리곤 좌표는 나중에 로그를 검산할 때만 쓴다)

[이벤트 형식 — 계약 v1.11]
  "<zone_id> <robot_ns> <REJECT|GRANT|RELEASE|RECLAIM>"   (std_msgs/String, RELIABLE)
  fleet_manager 가 받아 DB events(kind='ZONE') 에, kpi_logger 가 CSV 에 남긴다.
  K-06(교착 자동 해결률) = REJECT 1건 -> 같은 로봇·구역의 GRANT 가 뒤따르면 '해결'.
  [중요] 그래서 중복 발행을 막는 규칙이 셋 있다. 하나라도 어기면 K-06 이 틀린 값이 된다.
    - GRANT   : 새로 획득할 때만 (보유자 재요청은 발행 안 함)
    - REJECT  : 대기가 시작될 때 1건만 (로봇의 2초 주기 재요청은 발행 안 함)
    - RELEASE : 실제 보유자의 반납만 (못 받고 포기한 로봇의 반납은 발행 안 함)
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mro_interfaces.srv import ZoneToken

# ==========================================
# 전역 상수 및 설정
# ==========================================
# 9/5 실측 아레나 기준. 구역은 둘뿐이다 — 로봇끼리만 경합하는 곳에만 토큰을 둔다.
#   P1   : 로봇 구역(도크·창고) <-> 중앙 복도 병목(0.79 m). 도크 -> 창고는 벽 없이 이어져 지나지 않는다.
#          S4 창고에서 대기공간으로 나올 때 · S10 복귀할 때, 두 번 통과 -> E-3 교착 지점
#   DOCK : 도크 구역. 복귀 시 한 번에 한 대만 진입 -> E-7 순차 도킹
# 추종 경로(중앙 복도 -> 작업공간, 0.93 m)에는 토큰을 두지 않는다 —
#   토큰을 기다리며 멈추면 작업자를 놓친다. 두 쌍의 출발 시점을 엇갈리게 해서 회피한다.
ZONES = ['P1', 'DOCK']        # bringup/params/arena.yaml 의 semaphore_zones 와 ID 가 일치해야 한다
HOLD_TIMEOUT = 60.0           # s, BRD 5.3 '교착 자동 해결' 판정 시간 [팀 가정]
EVENT_TOPIC = '/fleet/zone_event'   # 절대 이름 — 로봇 네임스페이스 아래로 들어가면 관제가 못 받는다
TOKEN_SERVICE = '/fleet/zone_token'


class ZoneSemaphore(Node):
    """P1·DOCK의 보유자, FIFO 대기열, 강제 회수를 관리하는 잠금 서버.

    서비스 콜백 :meth:`cb`은 상태 전이만 원자적으로 처리하고, :meth:`watchdog`이
    반납하지 못한 보유자를 회수한다. 위치·경로 계산은 로봇 측 노드의 책임이다.
    """

    def __init__(self):
        super().__init__('zone_semaphore')

        # 1. 구역별 상태 — 전부 dict 이고 키는 구역 ID
        self.holder = {z: None for z in ZONES}      # 지금 이 구역을 점유한 robot_ns (없으면 None)
        self.since = {z: 0.0 for z in ZONES}        # 그 로봇이 점유를 시작한 시각 (HOLD_TIMEOUT 기준점)
        self.queue = {z: [] for z in ZONES}         # 기다리는 로봇 목록. 앞에서부터 처리(FIFO)
        self.notified = {z: set() for z in ZONES}   # REJECT 를 이미 발행한 로봇. 획득·반납 때 지운다

        # 2. 퍼블리셔 / 서비스
        self.pub_event = self.create_publisher(String, EVENT_TOPIC, 10)   # RELIABLE (계약 8절)
        self.create_service(ZoneToken, TOKEN_SERVICE, self.cb)

        # 3. 감시 타이머 (1 Hz) — 반납하지 않고 사라진 로봇의 토큰을 회수
        self.create_timer(1.0, self.watchdog)

    def event(self, z, ns, kind):
        """계약 v1.11 형식으로 한 줄 발행. K-06 원천이라 형식을 바꾸면 관제·웹캠 집계가 깨진다."""
        self.pub_event.publish(String(data=f'{z} {ns} {kind}'))

    def reject(self, z, ns):
        """
        거절 이벤트. 대기가 '시작될 때' 1건만 낸다 (계약 3절).
        [주의] 로봇은 거절당하면 2초마다 다시 물어본다. 그때마다 발행하면
        K-06 의 분모가 재요청 횟수만큼 부풀려진다. notified 집합으로 억제한다.
        """
        if ns not in self.notified[z]:
            self.notified[z].add(ns)
            self.event(z, ns, 'REJECT')

    def cb(self, req, res):
        """
        토큰 요청·반납 처리.
        [주의] 콜백 안에서는 계산만 한다. 여기서 블로킹 호출을 하면 단일 스레드가 막혀
        다른 로봇의 요청이 대기하고, 그게 곧 교착이다. (지금은 dict 조작뿐이라 안전)
        """
        z = req.zone_id

        # 0단계: 모르는 구역 이름 — 로봇 측 문자열과 ZONES 가 어긋나면 여기서 걸린다
        if z not in self.holder:
            res.granted, res.holder = False, ''
            return res

        # 1단계: 반납 (acquire=False)
        if not req.acquire:
            # (a) 실제 보유자의 반납 -> 구역을 비우고 RELEASE 발행
            if self.holder[z] == req.robot_ns:
                self.holder[z] = None
                self.get_logger().info(f'{z} 반납 ← {req.robot_ns}')
                self.event(z, req.robot_ns, 'RELEASE')
            # (b) 토큰을 못 받고 포기한 로봇도 반납을 부른다 (계약 3절: 성공·실패와 무관하게 반드시 반납).
            #     RELEASE 는 내지 않는다 — 보유한 적이 없다. 짝 없는 REJECT 로 남아
            #     K-06 의 '해결 못 함' 으로 정확히 세어진다.
            #     다만 대기열에서는 반드시 뺀다. 안 빼면 그 로봇이 FIFO 선두로 남아
            #     다른 로봇이 영구히 거절되고, watchdog 은 holder 만 비우므로 자동 복구도 안 된다.
            if req.robot_ns in self.queue[z]:
                self.queue[z].remove(req.robot_ns)
                self.get_logger().info(f'{z} 대기열에서 제거 ← {req.robot_ns}')
            self.notified[z].discard(req.robot_ns)   # 대기 1건 종료 — 다음 대기 때 REJECT 를 다시 낸다
            res.granted, res.holder = True, ''
            return res

        # 2단계: 획득 — 구역이 비어 있거나, 내가 이미 보유 중인 경우
        if self.holder[z] in (None, req.robot_ns):
            # (a) 먼저 기다린 로봇이 있으면 새치기 금지 (FIFO)
            if self.queue[z] and self.queue[z][0] != req.robot_ns:
                self.reject(z, req.robot_ns)
                res.granted, res.holder = False, self.holder[z] or ''
                return res
            # (b) 내 차례가 왔으니 대기열에서 나온다
            if req.robot_ns in self.queue[z]:
                self.queue[z].remove(req.robot_ns)
            # (c) '새로' 획득할 때만 하는 일 — 보유 시계 시작, 로그, GRANT 발행.
            #     보유자가 재요청해도 HOLD_TIMEOUT 이 리셋되지 않게(=회수를 무한 연기하지 못하게)
            #     하고, GRANT 도 중복 발행하지 않게 하는 조건이 이 한 줄이다.
            if self.holder[z] != req.robot_ns:
                self.since[z] = time.time()
                self.get_logger().info(f'{z} 획득 → {req.robot_ns}')
                self.event(z, req.robot_ns, 'GRANT')
                self.notified[z].discard(req.robot_ns)   # 대기 1건 종료 (REJECT -> GRANT 로 짝이 맞는다)
            self.holder[z] = req.robot_ns
            res.granted, res.holder = True, req.robot_ns
            return res

        # 3단계: 다른 로봇이 보유 중 -> 대기열에 등록하고 거절.
        #        로봇은 2초 뒤 다시 물어본다 (재시도 주기는 로봇 측 노드 소관)
        if req.robot_ns not in self.queue[z]:
            self.queue[z].append(req.robot_ns)
        self.reject(z, req.robot_ns)
        res.granted, res.holder = False, self.holder[z]
        return res

    def watchdog(self):
        """
        1 Hz 감시. 반납하지 않고 사라진(죽은·멈춘) 로봇의 토큰을 강제 회수한다.
        이게 없으면 로봇 하나가 죽는 순간 그 구역이 영구히 잠긴다.
        """
        for z, h in self.holder.items():
            if h and time.time() - self.since[z] > HOLD_TIMEOUT:
                # [알아 둘 것] 회수는 로봇이 구역을 실제로 빠져나왔는지 보지 않는다.
                #   정상 통과는 10~20 s(P1 이동만 3~9 s)라 60 s 초과는 estop 이 오래 걸린 때 정도다.
                #   그때 토큰만 넘어가고 앞 로봇은 아직 안에 있는데, 뒤 로봇은
                #   costmap·collision_monitor·safety_watchdog 에 막혀 충돌하지 않고 구역 앞에서 선다.
                #   즉 증상은 충돌이 아니라 '토큰은 받았는데 못 들어감' 이다 — TS-08 에서 이게 보이면 이 줄을 볼 것.
                #   갱신(heartbeat) 규칙은 계약에 없어 두지 않는다 (9/7 팀 판단).
                self.get_logger().error(f'{z} {h} 보유 {HOLD_TIMEOUT}s 초과 — 강제 회수 (교착 의심)')
                self.event(z, h, 'RECLAIM')
                self.holder[z] = None


def main():
    rclpy.init()
    node = ZoneSemaphore()
    try:
        # 콜백이 전부 가벼우므로 기본 spin 하나로 충분하다 (별도 실행 루프 불필요)
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
