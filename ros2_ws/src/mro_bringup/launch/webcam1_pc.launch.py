"""웹캠1 PC (PC-W1): 관제센터 작업자 등록 (S1).

ros2 launch mro_bringup webcam1_pc.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('mro_webcam'), 'config')
    # worker.pt 는 mro_perception 이 단일 출처다 (models/README.md, PR #19). yaml 은 패키지 경로를 못 구하므로 여기서 넘긴다
    models = os.path.join(get_package_share_directory('mro_perception'), 'models')
    return LaunchDescription([
        Node(package='mro_webcam', executable='webcam_register_node', name='webcam_register', output='screen',
             parameters=[os.path.join(cfg, 'webcam.yaml'),   # 노드 이름별 ros__parameters (interfaces.md 5절)
                         {'homography_json': os.path.join(cfg, 'WebCam1', 'homography_cam1.json'),
                          'model_path': os.path.join(models, 'worker.pt')}]),
    ])
