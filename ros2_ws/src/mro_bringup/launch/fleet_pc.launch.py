"""관제 PC (PC-F): fleet_manager, zone_semaphore, control_ui (http://<PC>:5000), kpi_logger.

kpi_logger 는 fleet 토픽을 가장 많이 구독하므로 관제 PC 에 둔다 (웹캠 PC 분리 9/5).
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='mro_fleet', executable='zone_semaphore_node', name='zone_semaphore', output='screen'),
        Node(package='mro_fleet', executable='fleet_manager_node', name='fleet_manager', output='screen'),
        Node(package='mro_fleet', executable='control_ui_node', name='control_ui', output='screen'),
        Node(package='mro_webcam', executable='kpi_logger_node', name='kpi_logger', output='screen'),
    ])
