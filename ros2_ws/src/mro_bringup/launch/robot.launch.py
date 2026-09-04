"""로봇 1대분 온보드 노드 (PC-로봇 제어 PC 에서 실행). localization·nav2 는 별도 터미널 (docs/setup.md 규칙).

ros2 launch mro_bringup robot.launch.py namespace:=/robot3
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    params_dir = os.path.join(get_package_share_directory('mro_bringup'), 'params')
    params = PythonExpression(["'", params_dir, "/' + '", ns, "'.strip('/') + '.yaml'"])
    remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]   # TB4 는 TF 를 네임스페이스 안으로 내보낸다

    def n(pkg, exe, name):
        return Node(package=pkg, executable=exe, name=name, namespace=ns, output='screen',
                    parameters=[params], remappings=remaps)

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/robot3'),
        n('mro_navigation', 'safety_watchdog_node', 'safety_watchdog'),   # 가장 먼저
        n('mro_perception', 'amr_detection_node', 'amr_detection'),
        n('mro_perception', 'aruco_detector_node', 'aruco_detector'),
        n('mro_navigation', 'shuttle_mission_node', 'shuttle_mission'),
        n('mro_navigation', 'follow_target_node', 'follow_target'),
        n('mro_navigation', 'return_dock_node', 'return_dock'),
    ])
