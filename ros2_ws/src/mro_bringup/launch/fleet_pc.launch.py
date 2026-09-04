"""관제 PC: fleet_manager, zone_semaphore, control_ui (http://<PC>:5000)."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='mro_fleet', executable='zone_semaphore_node', name='zone_semaphore', output='screen'),
        Node(package='mro_fleet', executable='fleet_manager_node', name='fleet_manager', output='screen'),
        Node(package='mro_fleet', executable='control_ui_node', name='control_ui', output='screen'),
    ])
