"""Nav2의 최종 출력을 워치독 입력으로 바꾼다. 원본 TB4 설정은 보존한다."""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


SPIN_MAX_RAD_S = 0.3    # Nav2 Spin 상한. TB4 기본 1.0 — 아래 주석
SPIN_MIN_RAD_S = 0.15   # Nav2 Spin 하한. TB4 기본 0.4
INFLATION_RADIUS_M = 0.12   # 주행 결정 9/10 오후 (이시율): 0.25·0.20 은 rviz 가 뚱뚱하다. **내접 0.186 아래라**
                            #   Nav2 가 'inflation smaller than inscribed' 경고를 내고 벽에 붙어 달릴 수 있다 —
                            #   통로에서 STUCK 이 재현되면 이 값을 0.20 으로 되돌린다 (9/10 오전 P1 벽 0.20 m 사례)


def _setup(context):
    ns = LaunchConfiguration('namespace').perform(context)
    if not ns.strip('/'):
        raise RuntimeError('로봇 namespace가 필요하다')
    source = LaunchConfiguration('params_file').perform(context)
    with open(source, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    params = config['collision_monitor']['ros__parameters']
    params['cmd_vel_out_topic'] = 'cmd_vel_navigation'
    # **Nav2 제자리 회전 속도를 낮춘다 (I-051 — odom 지연 과도).** Create3 의 odom 이 0.3~0.4 s
    #   늦게 와서 Spin 이 목표를 지나친 뒤에야 멈춘다. 9/10 `full_0910_1039`: 베이 정렬 스핀
    #   +10.6° 명령에 **+30.9° 실제** — 카메라 시야(±19°) 밖으로 마커가 나가 접근 2/2 가 실패하고,
    #   옆벽을 본 채 180° 탈출 스핀이 Collision Ahead 로 거부됐다. 1.0 rad/s 에서 지연 0.35 s 는
    #   20° 다. 0.3 rad/s 면 6°, 하한 0.15 면 마지막 구간 관성이 3° 안이다. 180° 는 10.5 s 로
    #   bay_align_timeout_s 20 안이다. 원본 TB4 설정은 건드리지 않는다 (임시 복사본만)
    behavior = config['behavior_server']['ros__parameters']
    behavior['max_rotational_vel'] = SPIN_MAX_RAD_S
    behavior['min_rotational_vel'] = SPIN_MIN_RAD_S
    # **벽에서 떨어져 다니게 한다 (I-009).** TB4 기본 inflation 0.15(설치본 파싱값 0.05)는 로봇
    #   내접 반지름 0.186 보다 작아 Nav2 가 "inflation radius smaller than inscribed radius" 를 내고
    #   벽에 붙어 달린다. 9/10 `full_0910_1054`: P1 통로를 대각선으로 질러 서쪽 벽 0.20 m 에서
    #   estop, 360° 최소 0.20 < 회전 여유 0.21 이라 탈출 회전도 거부 → STUCK. 0.25 는 9/6 주행
    #   합의·PM 승인값(arena.yaml nav2_inflation_radius)인데 저장소에서 Nav2 로 넘길 통로가 없어
    #   미적용이었다 — 이 런치가 그 통로다. 0.79 m 통로에서 양쪽 0.25 를 빼도 lethal 이 아니라
    #   비용일 뿐이라 통과는 되고, 가운데를 선호하게 된다
    for costmap in ('local_costmap', 'global_costmap'):
        config[costmap][costmap]['ros__parameters']['inflation_layer']['inflation_radius'] = INFLATION_RADIUS_M
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(config, stream)
        target = stream.name

    def cleanup(context):
        if os.path.exists(target):
            os.unlink(target)
        return []

    package = get_package_share_directory('turtlebot4_navigation')
    return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(package, 'launch', 'nav2.launch.py')),
                launch_arguments={'namespace': ns, 'params_file': target,
                                  'use_sim_time': LaunchConfiguration('use_sim_time')}.items())]


def generate_launch_description():
    package = get_package_share_directory('turtlebot4_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/robot3'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(package, 'config', 'nav2.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_setup)])
