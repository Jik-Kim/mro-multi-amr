"""I-037 재현: 실제 DDS/TF에 20 Hz 주행을 공급하고 navigator 대기 중 지연을 잰다.

로봇 명령은 보내지 않는다. 격리 ROS_DOMAIN_ID에서만 실행할 것.
"""
import threading
import time

from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster

from test_shuttle_mission_fsm import make_node, ros  # noqa: F401


pytestmark = pytest.mark.usefixtures('ros')


def test_tf_stays_current_during_navigator_wait():
    """블로킹 언독과 navigator 폴링이 있어도 TF가 0.3초 이상 밀리지 않아야 한다."""
    node = make_node(simulate_pose=False)
    source = Node('shuttle_tf_test_source')
    broadcaster = TransformBroadcaster(source)
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    started = source.get_clock().now().nanoseconds * 1e-9

    def publish():
        stamp = source.get_clock().now()
        elapsed = stamp.nanoseconds * 1e-9 - started
        odom = TransformStamped()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.transform.translation.x = elapsed * 0.26
        odom.transform.rotation.w = 1.0
        localization = TransformStamped()
        localization.header.stamp = stamp.to_msg()
        localization.header.frame_id = 'map'
        localization.child_frame_id = 'odom'
        localization.transform.rotation.w = 1.0
        broadcaster.sendTransform([localization, odom])

    source.create_timer(0.05, publish)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert node.wait(lambda: node.tf_pose(warn=False) is not None, timeout=5.0)
        # undock()가 셔틀 콜백을 막는 구간을 재현한다. TF 발행자는 계속 돈다.
        time.sleep(1.0)
        deadline = time.monotonic() + 2.0

        def navigation_complete():
            # BasicNavigator.isTaskComplete()와 같은 별도 Node의 0.1초 spin 대기.
            rclpy.spin_once(node.navigator, timeout_sec=0.1)
            return time.monotonic() >= deadline

        node.wait(navigation_complete)
        transform = node.tf_buffer.lookup_transform('map', 'base_link', Time())
        stamp = Time.from_msg(transform.header.stamp).nanoseconds * 1e-9
        age = node.now() - stamp
        position_error = abs((node.now() - started) * 0.26
                             - transform.transform.translation.x)
        print(f'\nTF_EXPERIMENT age_s={age:.3f} position_error_m={position_error:.3f}')
        assert age < 0.3, f'navigator 대기 중 TF가 {age:.3f}초 밀렸다'
        assert position_error < 0.26 * 0.3
    finally:
        executor.shutdown()
        thread.join()
        source.destroy_node()
        node.destroy_node()
