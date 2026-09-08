"""로봇 1대분 온보드 노드 (PC-로봇 제어 PC 에서 실행). localization·nav2 는 별도 터미널 (docs/setup.md 규칙).

ros2 launch mro_bringup robot.launch.py namespace:=/robot3

로봇을 추가·교체할 때 이 파일은 건드리지 않는다 — params/robotN.yaml 을 만들면 namespace 로 찾아 쓴다.
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
    # 공통값(모든 로봇 동일)과 로봇별 값(dock_pose·depth_k)을 분리해 둔다 — 로봇을 늘려도 공통값이 갈라지지 않게.
    common = os.path.join(params_dir, 'common.yaml')
    # 로봇별 파일은 이름으로 찾는다: namespace:=/robot8 → params/robot8.yaml
    robot_params = PythonExpression(["'", params_dir, "/' + '", ns, "'.strip('/') + '.yaml'"])
    # 인지 소유 파라미터 (계약 5절). 공용값(/**)만 들었고 로봇별 depth_k 는 robotN.yaml 로 옮겼다 (9/8)
    perception_yaml = os.path.join(get_package_share_directory('mro_perception'), 'config', 'perception.yaml')
    # arena.yaml 은 데이터 yaml 이라 파라미터 파일로 못 넘긴다 — 경로만 준다.
    #   shuttle_mission 이 스스로 mro_bringup share 를 런타임 조회하는 대체값을 갖고 있지만,
    #   mro_navigation 이 mro_bringup 에 의존을 걸 수 없어(순환) 런치가 넘기는 쪽이 정본이다
    arena_yaml = {'arena_yaml': os.path.join(params_dir, 'arena.yaml')}
    remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]   # TB4 는 TF 를 네임스페이스 안으로 내보낸다

    def n(pkg, exe, name, extra=()):
        # ★ 파일 순서가 우선순위를 정한다 — 다만 규칙이 직관과 다르다 (9/8 실측 확인).
        #   rcl_yaml_param_parser 는 와일드카드 패턴별로 묶음을 만들고, **패턴이 처음 나온 순서**로
        #   묶음을 적용한다. 같은 패턴이 뒤 파일에 또 나오면 처음 만든 묶음에 합쳐진다.
        #   → 광역(/**) 파일을 먼저, 로봇별(/robotN/**) 파일을 마지막에 둬야 로봇별 값이 이긴다.
        #   순서를 뒤집으면 /** 묶음이 나중에 적용돼 로봇별 값을 덮는다. 실제로 변경 전 순서
        #   [robotN.yaml, perception.yaml] 에서 robot3 의 depth_k 0.024 가 perception.yaml 의
        #   /** depth_k 0.0 에 덮여 **I-031 보정이 노드에 안 들어가고 있었다** (I-040).
        return Node(package=pkg, executable=exe, name=name, namespace=ns, output='screen',
                    parameters=[common, *extra, robot_params], remappings=remaps)

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/robot3'),
        n('mro_navigation', 'safety_watchdog_node', 'safety_watchdog'),   # 가장 먼저
        n('mro_perception', 'amr_detection_node', 'amr_detection', [perception_yaml]),
        n('mro_perception', 'aruco_detector_node', 'aruco_detector', [perception_yaml]),   # camera_fx_scale·max_distance_m (PR #30)
        n('mro_navigation', 'shuttle_mission_node', 'shuttle_mission', [arena_yaml]),   # 베이 waypoint·대기공간 (PR #36)
        n('mro_navigation', 'follow_target_node', 'follow_target'),
        n('mro_navigation', 'return_dock_node', 'return_dock'),
    ])
