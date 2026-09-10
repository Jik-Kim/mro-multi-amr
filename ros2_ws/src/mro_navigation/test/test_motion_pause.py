"""액션 종료 확인과 고정 목표 재전송 순서를 검증한다."""
from types import SimpleNamespace

import pytest
from mro_navigation.nodes import motion_pause as module


class Owner:
    estop = False

    def __init__(self):
        self.log = []
        self.sends = 0
        self.done = True
        self.stop_once = True
        self.navigator = SimpleNamespace(goToPose=self.send, isTaskComplete=lambda: self.done,
            cancelTask=self.cancel, getResult=lambda: SimpleNamespace(name='SUCCEEDED'))

    def create_publisher(self, *args):
        return SimpleNamespace(publish=lambda m: self.log.append(('paused', True)))

    def get_clock(self):
        from builtin_interfaces.msg import Time
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=1)))

    def create_timer(self, *args):
        pass

    def send(self, pose):
        self.sends += 1
        self.log.append(('goal', pose))
        self.done = self.sends > 1
        return True

    def cancel(self):
        self.log.append(('cancel',))
        self.done = True

    def wait(self, condition, timeout=None):
        if timeout is not None and self.stop_once:
            self.estop, self.stop_once = True, False
            return False
        if timeout is None:
            assert self.done
            assert self.log[-1] == ('paused', True)
            self.log.append(('clear',))
            self.estop = False
        return condition()


def test_cancel_completion_precedes_clear_and_fixed_goal_resend(monkeypatch):
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.rclpy, 'spin_once', lambda *a, **k: None)
    owner = Owner()
    pause = module.MotionPause(owner, 'test', RuntimeError)
    goal = (3, 4)
    pause.navigate(goal, '고정 목적지')
    events = [entry[0] for entry in owner.log]
    assert events.index('cancel') < events.index('clear')
    assert [entry for entry in owner.log if entry[0] == 'goal'] == [('goal', goal)] * 2
    assert owner.log.index(('clear',)) < max(i for i, e in enumerate(owner.log) if e[0] == 'goal')
    assert pause.ready and not pause.active


def test_unconfirmed_cancel_cannot_publish_ready(monkeypatch):
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.rclpy, 'spin_once', lambda *a, **k: None)
    ticks = iter([0, 3])
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(ticks))
    owner = Owner()
    owner.estop = True
    owner.navigator.cancelTask = lambda: None
    owner.done = False
    pause = module.MotionPause(owner, 'test', RuntimeError)
    pause.active, pause.ready = True, False
    with pytest.raises(RuntimeError):
        pause.cancel()
    pause.idle()
    assert not pause.ready and pause.active
    assert ('paused', True) not in owner.log


def test_undock_does_not_wait_for_estop_release(monkeypatch):
    """언독은 estop 해제를 기다리지 않는다.

    도킹 상태에서는 도크가 전방 장애물이라 estop 이 항상 걸린다. 여기서 해제를 기다리면
    **언독이 영원히 시작되지 않는다** (9/10 실주행). 워치독은 `cmd_vel` 만 통제하므로
    Create3 내부 액션인 언독을 애초에 막지도 못한다.
    """
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.rclpy, 'spin_once', lambda *a, **k: None)
    owner = Owner()
    owner.estop = True                         # 도크에 물려 있어 estop 이 걸린 상태
    owner.stop_once = False
    sent = []
    owner.navigator.undock_send_goal = lambda: sent.append('undock')
    owner.navigator.isUndockComplete = lambda: True
    owner.navigator.undock_status = 4          # STATUS_SUCCEEDED
    owner.wait = lambda condition, timeout=None: condition()

    pause = module.MotionPause(owner, 'test', RuntimeError)
    assert pause.dock_once(undock=True) is True
    assert sent == ['undock'], 'estop 이 걸려 있어도 언독 목표는 나가야 한다'
    assert ('cancel',) not in owner.log, 'estop 을 보고 언독을 취소하면 안 된다'
