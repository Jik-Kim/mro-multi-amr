#!/usr/bin/env python3
"""작업 종료 → 즉시 추종 중단 → 순차 복귀·도킹 (S9·S10 / BR-T29, BR-T31).

흐름
  1. `mission/finish` 수신 → 진행 중 주행 취소                     → RETURNING
  2. **DOCK 토큰을 먼저** 얻는다. 거절되면 홀딩 포인트로 이동해 대기 → DOCK_WAIT
  3. 그다음 P1 토큰. 거절되면 그 자리(P1 바깥)에서 기다린다
  4. 자기 도크 정면 접근 지점으로 이동 — 도착이 곧 P1 통과 완료 → P1 반납
  5. dock 호출. 실패하면 **접근 지점으로 되돌아가** 최대 `dock_retry` 회 재시도 → DOCKING → DOCKED
  6. 결과와 무관하게 DOCK 토큰을 반납한다

토큰 순서가 DOCK → P1 인 이유
  홀딩 포인트(`arena.yaml` `dock_zone.holding`)가 **P1 바깥**(중앙 복도, 최근접 도크에서 1.52 m)이다.
  DOCK 을 못 얻은 로봇은 애초에 P1 에 들어가지 않아야 하므로 DOCK 을 먼저 묻는다 (BRD v3.3 개정).

접근 지점을 반드시 거치는 이유
  도크 간격이 0.84 m(실측) 뿐이다. 멀리서 `dock()` 을 부르면 **옆 도크로 향할 수 있다.**
  IR 차폐판을 두지 않기로 했으므로(9/6, I-006) 접근 지점 경유가 오도킹을 막는 유일한 방어다.

입력  mission/finish(Bool) · safety/estop(Bool) · TF map→base_link
      arena.yaml(dock_zone.approach·holding) · robotN.yaml(dock_retry·t_ret_s·token_retry_s)
출력  shuttle/status(String: RETURNING/DOCK_WAIT/DOCKING/DOCKED/FAILED) · dock·undock·goToPose

후진을 쓰지 않는다 (계약 2절)
  이 노드는 `linear.x < 0` 을 내지 않고 `undock()` 도 부르지 않는다. 도킹이 실패하면
  **접근 지점으로 되돌아가** 다시 시도한다 — 계약이 정한 '제자리 회전 뒤 전진' 이다.
  Create3 의 범퍼·클리프가 전방에만 있어 후진은 감지 없는 주행이기 때문이다.

판정 관련 규칙
  · `DOCKED` 는 AC-13 운용시간의 끝이다 (`kpi_report` 의 `DOCKED_STATES`). 성공 직후 `READY` 를
    내지 않는다 — 내면 운용시간이 도크에서 한 번 더 끊긴다 (I-034 와 같은 종류의 문제).
  · `FAILED` 는 포기할 때만. 재시도 중에는 `DOCKING` 을 유지한다 (I-026).
  · 토큰은 **요청만 해도 반납한다.** 거절당하면 세마포어 대기열에 들어가므로, 못 얻은 채 끝내면
    유령 대기열이 남아 그 구역이 영구히 잠긴다 — watchdog 은 보유자만 회수한다 (I-033 과 같은 종류).

TODO([주행]):
① `robot.launch.py` 가 이 노드에도 `arena_yaml` 을 넘기게 할 것 (지금은 셔틀만 받는다).
   그때까지는 `mro_bringup` share 를 런타임 조회하는 대체값을 쓴다 — 의존을 선언하면 순환이다. PM 요청 필요
"""
from mro_navigation.nodes.motion_pause import MotionPause

import math
import os

from ament_index_python.packages import get_package_share_directory
from mro_interfaces.srv import ZoneToken
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
import yaml

FINISH_TOPIC = 'mission/finish'
ESTOP_TOPIC = 'safety/estop'
STATUS_TOPIC = 'shuttle/status'
TOKEN_SRV = '/fleet/zone_token'     # 관제 PC 의 zone_semaphore. 전역 이름
ZONE_DOCK = 'DOCK'
ZONE_P1 = 'P1'                      # arena.yaml semaphore_zones · ZoneToken.srv 와 ID 일치
MAP_FRAME = 'map'
BASE_FRAME = 'base_link'
TICK_S = 0.1


class ReturnAbort(Exception):
    """복귀를 포기한다. 토큰 반납은 호출자의 finally 가 맡는다."""


class ReturnDock(Node):
    """S9~S10 복귀 상태기계.

    종료 토픽 콜백은 ``pending``만 세우고, :meth:`run_return`이 메인 루프에서
    DOCK→P1 토큰과 접근 지점·도킹 재시도를 순서대로 처리한다.
    """

    def __init__(self):
        super().__init__('return_dock')
        ns = self.get_namespace().rstrip('/')
        if not ns:
            # 대체값을 쓰면 이 노드가 남의 이름으로 토큰을 요청·반납해
            #   **다른 로봇의 구역 점유를 풀어 버린다.** 조용히 틀리느니 여기서 죽는다 (9/8, main)
            raise RuntimeError(
                'return_dock: 네임스페이스 없이는 뜰 수 없다 — 로봇 노드는 ns 로 자기 로봇을 정한다.\n'
                '  ros2 launch mro_bringup robot.launch.py namespace:=/robotN\n'
                '  (단독 실행이면 --ros-args -r __ns:=/robotN)\n'
                '이 노드는 자기 이름으로 DOCK·P1 토큰을 요청하므로 대체값을 쓸 수 없다.')
        self.ns = ns
        self.robot = ns.lstrip('/')
        # ★ 네임스페이스를 넘긴다. 안 넘기면 /robot3/dock 이 아니라 /dock 을 불러
        #   2대 환경에서 도킹이 안 된다 (1대 시험에서는 우연히 통과한다)
        self.navigator = TurtleBot4Navigator(namespace=self.ns)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 런치가 arena_yaml 을 넘기면 이 조회는 쓰이지 않는다. mro_bringup 이 설치되지 않은
        #   환경(단위 시험·부분 빌드)에서 조회가 예외를 던져 노드가 죽지 않게 감싼다 —
        #   경로를 못 찾으면 빈 값으로 두고 _load_arena 가 사유를 남긴다
        try:
            default_arena = os.path.join(
                get_package_share_directory('mro_bringup'), 'params', 'arena.yaml')
        except Exception:
            default_arena = ''
        self.declare_parameter('dock_retry', 2)            # robotN.yaml
        self.declare_parameter('t_ret_s', 180.0)           # robotN.yaml
        self.declare_parameter('token_retry_s', 2.0)       # robotN.yaml · 계약 3절 '거절되면 2초 후 재요청'
        self.declare_parameter('arena_yaml', default_arena)
        self.arena = self._load_arena(self.p('arena_yaml'))

        self.create_subscription(Bool, FINISH_TOPIC, self.finish_cb, 10)
        self.create_subscription(Bool, ESTOP_TOPIC, self.estop_cb, 1)
        self.pub_status = self.create_publisher(String, STATUS_TOPIC, 10)
        self.cli_token = self.create_client(ZoneToken, TOKEN_SRV)

        self.pending = False        # finish 를 받았다 (실행은 메인 루프)
        self.running = False
        self.estop = False
        self.motion_pause = MotionPause(self, 'return_dock', ReturnAbort)
        self.t0 = None
        self.last_status = None
        # 보유(held)와 요청(requested)을 따로 둔다 — 거절당해도 대기열에 들어가므로 반납해야 한다
        self.held = {ZONE_DOCK: False, ZONE_P1: False}
        self.requested = {ZONE_DOCK: False, ZONE_P1: False}
        self.get_logger().info(
            f'[대기] {self.ns}{FINISH_TOPIC} 수신 시 복귀 · '
            f'T_ret {self.p("t_ret_s")} s · 재시도 {self.p("dock_retry")} 회')

    def p(self, name):
        return self.get_parameter(name).value

    def _load_arena(self, path):
        """arena.yaml 은 ROS 파라미터 형식이 아닌 데이터 yaml 이라 파일로 직접 읽는다."""
        try:
            with open(path, encoding='utf-8') as f:
                return yaml.safe_load(f)['arena']
        except Exception as e:
            self.get_logger().error(
                f'arena.yaml 적재 실패 — 접근 지점·홀딩 포인트를 모르므로 복귀할 수 없다 ({path}): {e}')
            return None

    # ── 콜백: 값만 저장한다. navigator 를 부르면 단일 스레드가 막힌다 ──

    def finish_cb(self, msg):
        if not msg.data:
            return
        if self.running:
            self.get_logger().warn('이미 복귀 중이다 — 중복 종료 신호를 버린다')
            return
        self.pending = True

    def estop_cb(self, msg):
        was = self.estop
        self.estop = bool(msg.data)
        if self.estop and not was:
            self.get_logger().warn('estop — 해제까지 주행 명령을 내지 않는다 (실패가 아니다)')
        elif was and not self.estop:
            self.get_logger().info('estop 해제 — 복귀를 이어간다')

    # ── 공용 헬퍼 (셔틀과 같은 방식) ──

    def now(self):
        """ROS 시각. 로그·CSV·DB 를 합쳐야 하므로 벽시계를 섞지 않는다 (계약 2절)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def set_status(self, s):
        if s == self.last_status:
            return                                  # 같은 값을 연속으로 내지 않는다
        self.last_status = s
        self.pub_status.publish(String(data=s))
        self.get_logger().info(f'[{s}]')

    def over_tret(self):
        return self.t0 is not None and self.now() - self.t0 > self.p('t_ret_s')

    def wait(self, cond, timeout=None):
        """유일한 대기 수단. 모든 대기가 여기를 지나므로 T_ret 검사가 한 곳에 모인다."""
        t_start = self.now()
        while rclpy.ok():
            if self.over_tret():
                raise ReturnAbort(f"T_ret {self.p('t_ret_s')} s 초과")
            done = cond()
            if self.over_tret():
                raise ReturnAbort(f"T_ret {self.p('t_ret_s')} s 초과")
            if done:
                return True
            rclpy.spin_once(self, timeout_sec=TICK_S)   # 콜백은 대기 중에도 살아 있다
            if self.over_tret():
                raise ReturnAbort(f"T_ret {self.p('t_ret_s')} s 초과")
            if timeout is not None and self.now() - t_start > timeout:
                return False
        raise ReturnAbort('노드 종료')

    def tf_pose(self, warn=True):
        """(x, y, yaw) 또는 None. yaw 식은 계약 2절 FleetStatus.yaw 와 같다."""
        try:
            t = self.tf_buffer.lookup_transform(MAP_FRAME, BASE_FRAME, Time())
        except Exception as e:
            if warn:
                self.get_logger().warn(f'TF: {e}')
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def pose_of(self, x, y, yaw):
        """회전 인자는 도 단위다 — getPoseStamped 규약이라 라디안을 넘기면 조용히 틀어진다."""
        return self.navigator.getPoseStamped([float(x), float(y)], math.degrees(yaw))

    def goto(self, pose, label):
        """목표 1건. 콜백 밖에서만 불린다."""
        gx, gy = pose.pose.position.x, pose.pose.position.y
        self.motion_pause.navigate(pose, label)
        self.motion_pause.idle()
        at = self.tf_pose(warn=False)
        if at is None:
            self.get_logger().info(f'{label} 도착')
        else:
            self.get_logger().info(
                f'{label} 도착 — 목표 ({gx:.2f}, {gy:.2f}) / 실제 ({at[0]:.2f}, {at[1]:.2f}), '
                f'오차 {math.hypot(at[0] - gx, at[1] - gy):.2f} m')

    # ── 토큰 ──

    def token(self, zone, acquire):
        """구역 토큰 획득/반납. 서비스가 없으면 (관제 미기동) 통과시켜 단독 시험을 막지 않는다."""
        if not self.cli_token.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'zone_token 서비스 없음 — {zone} 없이 단독 모드로 진행')
            return True
        req = ZoneToken.Request()
        req.robot_ns, req.zone_id, req.acquire = self.ns, zone, acquire
        if acquire:
            # **요청을 보내기 전에** 세운다. 응답이 2 s 를 넘기면 요청은 세마포어에 닿아
            #   대기열에 들어갔는데 이 플래그가 False 로 남아 release() 가 반납을 안 보낸다.
            #   그러면 그 로봇이 FIFO 선두로 남아 구역이 영구히 잠긴다 — release() 로 막은
            #   I-033 이 이 경로로 되살아난다 (PM 리뷰 9/8, PR #46)
            self.requested[zone] = True
        fut = self.cli_token.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        res = fut.result()
        if res is None:
            self.get_logger().warn(f'{zone} 토큰 응답 없음 — 요청은 나갔으니 반납 대상으로 남긴다')
            return False
        if acquire:
            self.held[zone] = bool(res.granted)
        return bool(res.granted)

    def release(self, zone, reason):
        """요청만 했어도 반납한다.

        거절당한 로봇은 세마포어 대기열에 남는다. 못 얻은 채 끝내면 그 로봇이 FIFO 선두로
        남아 다른 로봇이 영구히 거절되고, watchdog 은 보유자만 회수하므로 자동 복구도 안 된다.
        """
        if not (self.held[zone] or self.requested[zone]):
            return
        self.token(zone, False)
        self.held[zone] = self.requested[zone] = False
        self.get_logger().info(f'{zone} 반납 — {reason}')

    def acquire_zone(self, zone, on_denied=None):
        """거절은 정상이다 (계약 3절). 무한 대기는 T_ret 이 끊고 세마포어에도 60 s 회수가 있다."""
        if self.token(zone, True):
            self.get_logger().info(f'{zone} 획득')
            return
        if on_denied:
            on_denied()                              # 예: 홀딩 포인트로 물러나 대기
        while not self.token(zone, True):
            self.get_logger().info(f"{zone} 대기 — {self.p('token_retry_s')} s 후 재요청")
            self.wait(lambda: False, timeout=self.p('token_retry_s'))
        self.get_logger().info(f'{zone} 획득')

    # ── 복귀 ──

    def targets(self):
        """접근 지점과 홀딩 포인트를 arena.yaml 에서 찾는다."""
        if self.arena is None:
            raise ReturnAbort('arena.yaml 미적재')
        dz = self.arena.get('dock_zone') or {}
        ap = (dz.get('approach') or {}).get(self.robot)
        ho = dz.get('holding')
        if not ap or len(ap) != 3:
            raise ReturnAbort(f'arena.yaml dock_zone.approach 에 {self.robot} 이 없다')
        if not ho or len(ho) != 3:
            raise ReturnAbort('arena.yaml dock_zone.holding 이 없다')
        return ap, ho

    def go_holding(self, holding):
        """DOCK 을 못 얻었다 — P1 바깥(중앙 복도)에서 기다린다. 선행 로봇의 도킹 경로를 비운다."""
        self.set_status('DOCK_WAIT')
        try:
            self.goto(self.pose_of(*holding), '홀딩 포인트')
        except ReturnAbort:
            raise
        except Exception as e:                       # 이동 실패가 복귀 자체를 막지는 않는다
            self.get_logger().warn(f'홀딩 포인트 이동 실패 — 제자리에서 대기 ({e})')

    def dock_with_retry(self, approach):
        """접근 지점에서 부른다. 실패하면 접근 지점으로 되돌아가 다시 시도한다.

        `undock()` 을 쓰지 않는다 — 도크에서 물러나는 후진 동작이고 계약 2절이 금지했다.
        대신 Nav2 로 접근 지점까지 되돌아간다. Nav2 도 후진하지 않으므로 제자리에서 돌아
        전진해 돌아온다 — 계약이 정한 '제자리 회전 뒤 전진' 그대로다. 제자리 회전이 쓸고 가는
        폭은 외접 0.189 − 내접 0.175 = 1.4 cm 뿐이라 도크 옆에서도 안전하다.
        """
        retry_max = int(self.p('dock_retry'))
        for attempt in range(retry_max + 1):
            if self.estop:
                self.get_logger().warn('estop 중 — 해제까지 도킹 보류')
                self.wait(lambda: not self.estop)
            self.set_status('DOCKING')               # 재시도 중에도 DOCKING 을 유지한다 (I-026)
            if attempt:
                self.get_logger().warn(f'도킹 재시도 {attempt}/{retry_max}')
            if self.motion_pause.dock_once():
                self.motion_pause.idle()
                return True
            if attempt < retry_max:
                self.get_logger().warn('도킹 실패 — 접근 지점으로 되돌아가 재시도 (후진하지 않는다)')
                self.goto(self.pose_of(*approach), '도크 접근 지점 (재시도)')
        return False

    def run_return(self):
        self.running = True
        self.t0 = self.now()
        try:
            approach, holding = self.targets()

            # S9: 진행 중 주행 취소. 추종 중단(페어링 해제)은 follow_target 이 같은 토픽으로 한다
            self.navigator.cancelTask()
            self.set_status('RETURNING')

            # S10: DOCK 을 먼저 — 홀딩 포인트가 P1 바깥이라 순서가 이렇다
            self.acquire_zone(ZONE_DOCK, on_denied=lambda: self.go_holding(holding))
            self.set_status('RETURNING')
            self.acquire_zone(ZONE_P1)               # 여기서 로봇은 이미 P1 바깥이다

            self.goto(self.pose_of(*approach), '도크 접근 지점')
            self.release(ZONE_P1, '접근 지점 도착 — P1 통과 완료')

            ok = self.dock_with_retry(approach)
            if not ok:
                raise ReturnAbort(f"도킹 {self.p('dock_retry')} 회 재시도 후에도 실패")
            self.set_status('DOCKED')
            self.get_logger().info(f'복귀 완료 — 소요 {self.now() - self.t0:.1f} s')
        except ReturnAbort as e:
            self.navigator.cancelTask()
            self.set_status('FAILED')                # 포기할 때만 (I-026)
            self.get_logger().error(f'복귀 포기: {e}')
        finally:
            for zone in (ZONE_P1, ZONE_DOCK):
                self.release(zone, '복귀 종료')
            self.running = False
            self.t0 = None
            # 성공 뒤 READY 를 내지 않는다 — DOCKED 가 AC-13 운용시간의 끝이다 (I-034 와 같은 종류)


def main():
    rclpy.init()
    node = ReturnDock()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=TICK_S)
            node.motion_pause.idle()
            if node.pending:
                node.pending = False
                node.run_return()                    # 콜백 밖에서만 navigator 를 부른다
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C 는 KeyboardInterrupt, launch·systemd 의 SIGTERM 은 ExternalShutdownException
        pass
    finally:
        try:
            for zone in (ZONE_P1, ZONE_DOCK):
                node.release(zone, '노드 종료')       # 죽어도 구역을 물고 있지 않게
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
