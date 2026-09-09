"""관제 PC (PC-F): fleet_manager(웹 UI 포함), zone_semaphore, kpi_logger.

kpi_logger 는 fleet 토픽을 가장 많이 구독하므로 관제 PC 에 둔다 (웹캠 PC 분리 9/5).

로봇 명단의 단일 출처는 params/fleet.yaml 의 robot_namespaces 다 — 대수·이름을 바꾸려면 그 파일을
고친다 (--symlink-install 이라 리빌드 없이 반영된다). 한 번만 다르게 띄울 때만 인자로 덮어쓴다:

  ros2 launch mro_bringup fleet_pc.launch.py                                  # fleet.yaml 그대로
  ros2 launch mro_bringup fleet_pc.launch.py robots:="[/robot3,/robot8,/robot9]"

zone_semaphore 는 요청에 실려 온 robot_ns 로만 동작해서 명단이 필요 없다.
fleet_manager 프로세스에 포함된 RobotControlNode가 로봇별 target/pair_state를 구독한다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def _launch_setup(context, *args, **kwargs):
    params_dir = os.path.join(get_package_share_directory('mro_bringup'), 'params')
    # common.yaml 은 로봇 파라미터가 대부분이지만 **로봇과 관제가 같아야 하는 계약값**(t_lim_s)도 들었다.
    #   관제가 자기 사본을 갖고 있으면 한쪽만 고쳤을 때 미션 실패 시점이 어긋난다 — 그래서 여기서도 넘긴다.
    #   관제 노드가 선언하지 않은 키는 그냥 무시되고, fleet.yaml 이 뒤에 와서 관제 전용 값이 이긴다.
    common_yaml = os.path.join(params_dir, 'common.yaml')
    fleet_yaml = os.path.join(params_dir, 'fleet.yaml')

    # 인자를 안 주면(기본) fleet.yaml 의 robot_namespaces 를 그대로 쓴다. 빈 덮어쓰기를 항상 얹으면
    #   yaml 값이 죽어 명단이 두 곳에 생긴다 — 그래서 값이 있을 때만 덮어쓴다.
    params = [common_yaml, fleet_yaml]
    robots = LaunchConfiguration('robots').perform(context).strip()
    if robots:
        namespaces = yaml.safe_load(robots)
        if not isinstance(namespaces, list) or not all(isinstance(n, str) for n in namespaces):
            raise RuntimeError(
                f'robots 인자는 문자열 목록이어야 한다 — 예: robots:="[/robot3,/robot8]" (받은 값: {robots!r})')
        if not namespaces:
            raise RuntimeError('robots 인자가 빈 목록이다 — 관제가 로봇을 하나도 잡지 않는다. '
                               '인자를 빼면 fleet.yaml 의 명단을 쓴다')
        params = [common_yaml, fleet_yaml, {'robot_namespaces': [str(n) for n in namespaces]}]

    def n(pkg, exe, name, extra=()):
        return Node(package=pkg, executable=exe, name=name, output='screen',
                    parameters=list(extra))

    return [
        n('mro_fleet', 'zone_semaphore_node', 'zone_semaphore'),
        n('mro_fleet', 'fleet_manager_node', 'fleet_manager', params),
        n('mro_webcam', 'kpi_logger_node', 'kpi_logger', params),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robots', default_value='',
            description='관제가 다룰 로봇 네임스페이스 목록. 비우면 params/fleet.yaml 을 쓴다. '
                        '예: robots:="[/robot3,/robot8,/robot9]"'),
        OpaqueFunction(function=_launch_setup),
    ])
