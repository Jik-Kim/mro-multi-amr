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
    # 인지 소유 파라미터 (계약 5절). robotN.yaml 뒤에 넘기므로 겹치는 키(reid_threshold)는 이쪽이 이긴다 — 값은 같다
    perception_yaml = os.path.join(get_package_share_directory('mro_perception'), 'config', 'perception.yaml')
    # arena.yaml 은 데이터 yaml 이라 파라미터 파일로 못 넘긴다 — 경로만 준다.
    #   shuttle_mission 이 스스로 mro_bringup share 를 런타임 조회하는 대체값을 갖고 있지만,
    #   mro_navigation 이 mro_bringup 에 의존을 걸 수 없어(순환) 런치가 넘기는 쪽이 정본이다
    arena_yaml = {'arena_yaml': os.path.join(params_dir, 'arena.yaml')}
    remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]   # TB4 는 TF 를 네임스페이스 안으로 내보낸다

    def n(pkg, exe, name, extra=()):
        return Node(package=pkg, executable=exe, name=name, namespace=ns, output='screen',
                    parameters=[params, *extra], remappings=remaps)

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/robot3'),
        n('mro_navigation', 'safety_watchdog_node', 'safety_watchdog'),   # 가장 먼저
        n('mro_perception', 'amr_detection_node', 'amr_detection', [perception_yaml]),
        n('mro_perception', 'aruco_detector_node', 'aruco_detector', [perception_yaml]),   # camera_fx_scale·max_distance_m (PR #30)
        n('mro_navigation', 'shuttle_mission_node', 'shuttle_mission', [arena_yaml]),   # 베이 waypoint·대기공간 (PR #36)
        n('mro_navigation', 'follow_target_node', 'follow_target'),
        n('mro_navigation', 'return_dock_node', 'return_dock'),
    ])
