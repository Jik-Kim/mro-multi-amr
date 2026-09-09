"""웹캠2 PC (PC-W2): 재탐지 방위 힌트 + 계측 (E-1).

ros2 launch mro_bringup webcam2_pc.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('mro_webcam'), 'config')
    show_window = LaunchConfiguration('show_window')
    # worker.pt 는 mro_perception 이 단일 출처다 (models/README.md, PR #19). yaml 은 패키지 경로를 못 구하므로 여기서 넘긴다
    models = os.path.join(get_package_share_directory('mro_perception'), 'models')
    return LaunchDescription([
        DeclareLaunchArgument(
            'show_window',
            default_value='false',
            description='webcam_hint OpenCV 미리보기 창 표시 여부',
        ),
        Node(package='mro_webcam', executable='webcam_hint_node', name='webcam_hint', output='screen',
             parameters=[os.path.join(cfg, 'webcam.yaml'),   # 노드 이름별 ros__parameters (interfaces.md 5절)
                         {'homography_json': os.path.join(cfg, 'WebCam2', 'homography_cam2.json'),
                          'model_path': os.path.join(models, 'worker.pt'),
                          'show_window': ParameterValue(show_window, value_type=bool)}]),
    ])
