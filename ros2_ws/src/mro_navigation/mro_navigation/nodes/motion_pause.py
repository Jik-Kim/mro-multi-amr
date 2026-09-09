"""메인 루프에서 액션 종료를 확인하고 워치독에 정지 확인을 보낸다."""
import time

import rclpy
from std_msgs.msg import Header


class MotionPause:
    """절대 목표는 재전송한다. 종료를 확인하지 못한 액션은 회전을 허용하지 않는다."""

    def __init__(self, owner, name, abort):
        self.owner, self.abort = owner, abort
        self.active = False
        self.ready = True
        self.kind = 'navigation'
        self.pub = owner.create_publisher(Header, 'safety/paused/' + name, 10)
        owner.create_timer(0.1, self.publish)

    def publish(self):
        if self.owner.estop and self.ready:
            self.pub.publish(Header(stamp=self.owner.get_clock().now().to_msg()))

    def complete(self):
        self.active = False
        self.ready = True
        self.publish()

    def cancel(self, dock=False):
        """미션 시간 초과 후에도 제한 시간 안에서 실제 액션 종료를 확인한다."""
        nav = self.owner.navigator
        kind = 'dock' if dock else self.kind
        if kind == 'navigation':
            handle = getattr(nav, 'goal_handle', None)
            if handle is not None:
                handle.cancel_goal_async()
            else:
                nav.cancelTask()
            poll = nav.isTaskComplete
        else:
            handle = getattr(nav, kind + '_goal_handle', None)
            if handle is None:
                raise self.abort('도킹 액션 핸들 없음 — 정지 확인 불가')
            handle.cancel_goal_async()
            poll = nav.isDockComplete if kind == 'dock' else nav.isUndockComplete
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            if poll():
                self.complete()
                return
            rclpy.spin_once(self.owner, timeout_sec=0.05)
        raise self.abort('액션 취소 후 종료 확인 실패 — 정지 유지')

    def wait_clear(self):
        if self.owner.estop:
            self.publish()
            self.owner.wait(lambda: not self.owner.estop)

    def navigate(self, pose, label):
        """ESTOP으로 취소한 고정 목적지는 해제 후 같은 좌표로 재전송한다."""
        n = self.owner
        while rclpy.ok():
            self.wait_clear()
            self.kind = 'navigation'
            self.ready, self.active = False, True
            try:
                if not n.navigator.goToPose(pose):
                    self.complete()
                    raise self.abort(f'{label} 목표 거부됨')
                while rclpy.ok():
                    rclpy.spin_once(n, timeout_sec=0.05)
                    if n.estop:
                        self.cancel()
                        self.wait_clear()
                        break
                    done = n.wait(n.navigator.isTaskComplete, timeout=0.05)
                    if n.estop:
                        continue
                    if done:
                        self.complete()
                        if n.navigator.getResult().name != 'SUCCEEDED':
                            raise self.abort(f'{label} 이동 실패: {n.navigator.getResult().name}')
                        return
            except Exception:
                if self.active:
                    self.cancel()
                raise
        raise self.abort('노드 종료')

    def dock_once(self, undock=False):
        """도킹 계열 액션도 별도 핸들로 취소한다. 중단된 언독은 자동 반복하지 않는다.

        **언독은 estop 을 기다리지 않는다.** Create3 는 도크에 코를 박고 들어가므로 도킹
        상태에서는 전방 ±35° 안에 도크와 벽이 있어 estop 이 항상 걸린다. 여기서 해제를
        기다리면 **언독이 영원히 시작되지 않는다** (9/10 실주행에서 T_lim 120 s 를 통째로
        날렸다). 언독은 그 장애물에서 **멀어지는** 동작이고, 워치독은 cmd_vel 만 통제하므로
        Create3 내부 액션인 이것을 애초에 막지도 못한다 — 기다리는 것은 안전이 아니라
        가동 불능이다. 도킹(dock)은 반대로 앞으로 들어가는 동작이라 해제를 기다린다.
        """
        n = self.owner
        if not undock:
            self.wait_clear()
        self.kind = 'undock' if undock else 'dock'
        self.ready, self.active = False, True
        try:
            if undock:
                n.navigator.undock_send_goal()
                poll = n.navigator.isUndockComplete
            else:
                n.navigator.dock_send_goal()
                poll = n.navigator.isDockComplete
            handle = getattr(n.navigator, self.kind + '_goal_handle', None)
            if handle is not None and not handle.accepted:
                self.complete()
                return False
            while rclpy.ok():
                rclpy.spin_once(n, timeout_sec=0.05)
                # 언독 중 estop 은 도크를 본 것이므로 중단하지 않는다 (위 docstring)
                if n.estop and not undock:
                    self.cancel()
                    self.wait_clear()
                    return False
                done = n.wait(poll, timeout=0.05)
                if n.estop and not undock:
                    continue
                if done:
                    self.complete()
                    if undock:
                        return getattr(n.navigator, 'undock_status', 4) == 4
                    return getattr(n.navigator, 'dock_status', 4) == 4 and bool(n.navigator.is_docked)
        except Exception:
            if self.active:
                self.cancel()
            raise
        return False

    def idle(self):
        self.publish()
