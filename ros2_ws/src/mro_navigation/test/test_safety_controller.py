"""로봇 없이 시간·센서·정지 확인을 주입하는 안전 회귀 시험."""
import math

import pytest
from mro_navigation.core.safety import SafetyController


def clear(controller, now):
    for _ in range(5):
        controller.scan(now - 0.4, 1.0, 1.0, 1.0, 1.0)
    assert controller.tick(now - 0.4, True) == 0
    controller.odometry(now - 0.4, 0, 0, 0)
    controller.odometry(now, 0, 0, 0)
    controller.scan(now, 1, 1, 1, 1)
    assert controller.tick(now, True) == 0
    assert not controller.estop


def rotating(**kwargs):
    c = SafetyController(**kwargs)
    clear(c, -0.5)
    for _ in range(2):
        c.scan(-0.3, 0.25, 0.25, 1.0, 0.5)
    c.odometry(-0.3, 0, 0, 0)
    c.odometry(0.1, 0, 0, 0)
    assert c.tick(0.1, True) == 0.25
    return c


def test_clear_fifth_frame_stops_early():
    c = rotating()
    for i in range(4):
        c.scan(0.2, 0.39, 0.39, 1, 1)
        assert c.tick(0.2, True) == 0.25
    c.scan(0.2, 0.39, 0.39, 1, 1)
    assert c.tick(0.2, True) == 0
    assert c.estop
    c.odometry(0.21, 0.03, 0, 0.25)
    assert c.tick(0.21, True) == 0
    c.odometry(0.3, 0.04, 0, 0)
    c.odometry(0.65, 0.04, 0, 0)
    c.scan(0.65, 0.39, 0.39, 1, 1)
    assert c.tick(0.65, True) == 0
    assert c.state == 'CLEAR'


def test_five_seconds_is_limit_not_required_duration():
    c = rotating()
    for i in range(1, 51):
        now = 0.1 + i / 10
        c.scan(now, 0.25, 0.25, 1, 1)
        c.odometry(now, 0.25 * (now - 0.1))
        value = c.tick(now, True)
        assert value == (0.25 if i < 50 else 0)
    assert c.state == 'STUCK'
    assert c.rotated < math.radians(75)


def test_actual_angle_limit_even_when_faster_than_command():
    c = rotating()
    c.scan(0.2, 0.25, 0.25, 1, 1)
    c.odometry(0.2, math.radians(76))
    assert c.tick(0.2, True) == 0
    assert c.state == 'STUCK'


def test_angle_accumulates_absolute_motion_across_wrap():
    c = SafetyController()
    c.state = 'SAFE_ROTATE'
    c.odometry(0, math.radians(179))
    c.odometry(0.1, math.radians(-179))
    c.odometry(0.2, math.radians(179))
    assert c.rotated == pytest.approx(math.radians(4))


@pytest.mark.parametrize('fault', ['scan', 'odom', 'pause', 'clearance'])
def test_recovery_fault_stops_and_never_retries(fault):
    c = rotating()
    c.scan(1, 0.25, 0.25, 1, 1)
    c.odometry(1, 0.1)
    if fault == 'scan':
        c.scan_at = 0
    elif fault == 'odom':
        c.odom_at = 0
    elif fault == 'clearance':
        c.sweep = 0.18
    assert c.tick(1, fault != 'pause') == 0
    assert c.state == 'STUCK'
    c.scan(1.1, 0.25, 0.25, 1, 1)
    c.odometry(1.1, 0.1)
    assert c.tick(1.1, True) == 0
    assert c.state == 'STUCK'


def test_no_rotation_until_all_actions_paused():
    c = SafetyController()
    clear(c, 0)
    for _ in range(2):
        c.scan(0.1, 0.25, 0.25, 1, 1)
    c.odometry(0.1, 0)
    assert c.tick(0.1, False) == 0
    assert c.state == 'STOPPING'
    assert c.tick(0.1, True) == 0
    c.odometry(0.11, 0, 0, 0)
    c.odometry(0.45, 0, 0, 0)
    assert c.tick(0.45, True) == 0.25


def test_initial_obstacle_never_starts_unrequested_rotation():
    c = SafetyController()
    c.scan(0, 0.25, 0.25, 1, 1)
    c.odometry(0, 0)
    assert c.tick(0, True) == 0
    assert c.state == 'STOPPING'


@pytest.mark.parametrize('linear,angular', [(0.1, 0), (0, 0.1), (float('nan'), 0), (None, None)])
def test_action_ack_cannot_replace_physical_stop(linear, angular):
    c = SafetyController()
    clear(c, 0)
    for _ in range(2):
        c.scan(0.1, 0.25, 0.25, 1, 1)
    c.odometry(0.1, 0, linear, angular)
    c.odometry(0.45, 0, linear, angular)
    assert c.tick(0.45, True) == 0
    assert c.state == 'STOPPING'


def test_stale_odometry_cannot_clear_estop():
    c = rotating()
    for _ in range(5):
        c.scan(0.2, 1, 1, 1, 1)
    c.tick(0.2, True)
    c.odometry(0.21, 0, 0, 0)
    c.odometry(0.55, 0, 0, 0)
    c.scan(1.1, 1, 1, 1, 1)
    assert c.tick(1.1, True) == 0
    assert c.estop


def test_invalid_odometry_breaks_stationary_interval():
    c = SafetyController()
    c.odometry(0, 0, 0, 0)
    c.invalidate_odometry()
    c.odometry(0.4, 0, 0, 0)
    assert not c.stopped(0.4)


def test_obstacle_returns_during_braking_never_restarts_rotation():
    c = rotating()
    for _ in range(5):
        c.scan(0.2, 1, 1, 1, 1)
    assert c.tick(0.2, True) == 0
    c.scan(0.3, 0.25, 0.25, 1, 1)
    assert c.tick(0.3, True) == 0
    assert c.state == 'STUCK'


def test_docked_front_obstacle_is_not_an_obstacle():
    """도크에 물려 있으면 전방 0.1 m 도 estop 을 걸지 않는다.

    Create3 는 도크에 코를 박고 들어간다 — 도킹 상태에서 전방 ±35° 안에 있는 것은 도크와
    그 뒤 벽이고 `stop_dist` 0.30 m 안에 반드시 들어온다. 그것을 장애물로 세면 **estop 이
    항상 걸려 언독 자체가 불가능해진다** (9/10 실주행에서 T_lim 을 통째로 날렸다).
    언독은 그 장애물에서 멀어지는 후진 동작이라 전방 판정으로 막을 이유가 없다.
    """
    c = SafetyController()
    c.dock_state(True)
    for t in (-0.8, -0.6, -0.4, -0.2, 0.0):
        c.scan(t, 0.10, 0.10, 0.10, 0.10)      # 도크가 코앞에 있다
        c.odometry(t, 0, 0, 0)
        c.tick(t, True)
    assert not c.estop, '도킹 중 전방 판정으로 estop 이 걸리면 언독을 못 한다'
    assert c.state == 'CLEAR'


def test_leaving_the_dock_restores_front_judgement():
    """도크를 벗어나면 전방 판정이 정상으로 돌아온다 — 판정을 새로 쌓는다."""
    c = SafetyController()
    c.dock_state(True)
    for t in (-0.8, -0.6, -0.4, -0.2, 0.0):
        c.scan(t, 0.10, 0.10, 0.10, 0.10)
        c.odometry(t, 0, 0, 0)
        c.tick(t, True)
    assert not c.estop

    c.dock_state(False)                        # 언독 완료
    assert c.clear == 0, '도크를 벗어나면 해제 표본을 다시 쌓아야 한다'
    for t in (0.2, 0.4):                       # 이제 0.10 m 는 진짜 장애물이다
        c.scan(t, 0.10, 0.10, 0.10, 0.50)
        c.odometry(t, 0, 0, 0)
        c.tick(t, True)
    assert c.estop, '도크 밖에서는 전방 0.10 m 가 estop 이어야 한다'


def _scan(ranges, angle_min=-math.pi, angle_max=math.pi):
    from sensor_msgs.msg import LaserScan
    m = LaserScan()
    m.angle_min, m.angle_max = angle_min, angle_max
    m.angle_increment = (angle_max - angle_min) / len(ranges)
    m.range_min, m.range_max = 0.15, 12.0
    m.ranges = ranges
    return m


def test_front_survives_realistic_lidar_dropout():
    """실물 RPLIDAR 처럼 빔 25 % 가 0 이어도 전방 거리가 나온다.

    9/10 실주행에서 전방 140 빔 중 26~36 개가 항상 0 이었다. 빔 하나라도 결측이면 구역을
    '모름' 으로 치던 규칙 때문에 언독하는 순간 STUCK 이 걸려 로봇이 영영 굳었다.
    """
    from mro_navigation.core.safety import scan_sectors
    n = 360
    ranges = [1.0] * n
    for i in range(0, n, 4):          # 25 % 결측
        ranges[i] = 0.0
    ranges[n // 2] = 0.44             # 정면에 0.44 m
    front, sweep, left, right = scan_sectors(_scan(ranges))
    assert front is not None and abs(front - 0.44) < 1e-6
    assert sweep is not None, '360° 도 결측 25 % 로 모름이 되면 SAFE_ROTATE 가 영영 불가하다'


def test_front_is_unknown_when_most_beams_are_missing():
    """유효 빔이 절반 미만이면 진짜 '모름' 이다 — 센서 이상."""
    from mro_navigation.core.safety import scan_sectors
    n = 360
    ranges = [0.0] * n
    for i in range(0, n, 3):          # 33 % 만 유효
        ranges[i] = 1.0
    front, sweep, _, _ = scan_sectors(_scan(ranges))
    assert front is None and sweep is None
