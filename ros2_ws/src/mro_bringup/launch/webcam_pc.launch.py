"""웹캠 PC: 웹캠1 등록, 웹캠2 힌트, KPI 로거."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('mro_webcam'), 'config')
    return LaunchDescription([
        Node(package='mro_webcam', executable='webcam_register_node', name='webcam_register', output='screen'),
        Node(package='mro_webcam', executable='webcam_hint_node', name='webcam_hint', output='screen',
             parameters=[{'homography_json': os.path.join(cfg, 'homography_cam2.json')}]),
        Node(package='mro_webcam', executable='kpi_logger_node', name='kpi_logger', output='screen'),
    ])
