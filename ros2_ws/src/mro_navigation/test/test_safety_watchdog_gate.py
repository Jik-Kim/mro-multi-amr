"""최종 속도 출력의 ESTOP 차단·오래된 명령 폐기를 검증한다."""
from types import SimpleNamespace

from geometry_msgs.msg import TwistStamped
import pytest
import rclpy

from mro_navigation.nodes.safety_watchdog_node import SafetyWatchdog


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/robot3'])
    node = SafetyWatchdog()
    node.sent = []
    node.pub_vel = SimpleNamespace(publish=node.sent.append)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def command(node, angular=1.5):
    msg = TwistStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.twist.angular.z = angular
    return msg


def test_estop_blocks_both_sources(node):
    for source in ('search', 'navigation'):
        node.command_cb(source, command(node))
    assert not node.sent


def test_clear_drops_old_commands_and_preserves_hint_profile(node):
    import time
    old = command(node)
    node.control.state = 'CLEAR'
    node.clear_at = time.monotonic()           # CLEAR 전이 직후 — 불감 시간 안
    node.command_cb('search', old)
    assert not node.sent
    node.clear_at = time.monotonic() - 1.0     # 불감 시간 지남
    new = command(node)
    node.command_cb('search', new)
    assert node.sent == [new]
    assert node.sent[-1].twist.angular.z == 1.5
    node.command_cb('navigation', command(node, 0.2))
    assert node.sent == [new]


def test_rotation_output_has_zero_translation(node):
    node.control.tick = lambda now, paused: 0.25
    node.tick()
    msg = node.sent[-1]
    assert (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z) == (0, 0, 0)
    assert msg.twist.angular.z == 0.25


def test_old_pause_ack_is_not_reused_for_new_estop(node):
    import time
    from std_msgs.msg import Header
    old = Header(stamp=node.get_clock().now().to_msg())
    node.pause_epoch = node.get_clock().now().nanoseconds
    for name in node.participants:
        node.pause_cb(name, old)
    assert not node.all_paused(time.monotonic())
    fresh = Header(stamp=node.get_clock().now().to_msg())
    for name in node.participants:
        node.pause_cb(name, fresh)
    assert node.all_paused(time.monotonic())


def _skewed(msg, seconds):
    """헤더 시각을 seconds 만큼 과거로 — 로봇 시계가 PC 보다 늦은 상황."""
    from builtin_interfaces.msg import Time
    sec = max(0, msg.header.stamp.sec - int(seconds))
    msg.header.stamp = Time(sec=sec, nanosec=msg.header.stamp.nanosec)
    return msg


def test_clock_skew_does_not_blind_the_scan(node):
    """로봇 시계가 10 s 어긋나도 스캔은 유효하다.

    종전에는 헤더 시각을 PC 시계와 비교해 0.5 s 넘게 어긋나면 스캔 전체를 버렸다. 그러면 front 가
    영원히 None → STUCK('유효한 LiDAR 스캔 없음') 영구. 신선도는 수신 시각으로만 잰다.
    """
    import math
    from sensor_msgs.msg import LaserScan
    m = LaserScan()
    m.header.stamp = node.get_clock().now().to_msg()
    m.angle_min, m.angle_max = -math.pi, math.pi
    m.angle_increment = 2 * math.pi / 360
    m.range_min, m.range_max = 0.15, 12.0
    m.ranges = [1.0] * 360
    node.scan_cb(_skewed(m, 10))
    assert node.control.front is not None, '시계 어긋남으로 스캔을 버리면 안 된다'


def test_clock_skew_does_not_invalidate_odometry(node):
    from nav_msgs.msg import Odometry
    m = Odometry()
    m.header.stamp = node.get_clock().now().to_msg()
    m.pose.pose.orientation.w = 1.0
    node.odom_cb(_skewed(m, 10))
    assert node.control.odom_at is not None, '시계 어긋남으로 odom 을 버리면 정지 확인이 영영 안 된다'


def test_clock_skew_does_not_drop_commands_in_clear(node):
    import time
    node.control.state = 'CLEAR'
    node.clear_at = time.monotonic() - 1.0
    node.command_cb('navigation', _skewed(command(node, 0.2), 10))
    assert node.sent, '시계 어긋남으로 명령을 버리면 CLEAR 인데도 로봇이 안 움직인다'


def test_silent_participant_is_forgotten(node):
    """ACK 를 보내다 5 s 넘게 침묵한 노드는 떠난 것으로 보고 더 기다리지 않는다."""
    import time
    from mro_navigation.nodes.safety_watchdog_node import PAUSE_FORGET_S
    node.paused['follow_target'] = (node.pause_epoch, time.monotonic() - PAUSE_FORGET_S - 1)
    assert node.all_paused(time.monotonic()) is True
    assert 'follow_target' not in node.paused
