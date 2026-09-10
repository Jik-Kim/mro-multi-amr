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
INFLATION_RADIUS_M = 0.25   # arena.yaml nav2_inflation_radius (9/6 합의). 9/10 P1 세 번: 0.15·0.12 는 벽 0.20 m STUCK,
                            #   0.25 만 통과(13 s). 워치독 stop_dist 0.30 안으로 Nav2 가 안 들어가게 하는 값이다.
                            #   낮추면 '여유' 가 생기는 게 아니라 Nav2 가 그 여유를 다 써서 벽에 붙는다
COST_SCALING = 10.0         # TB4 기본 4.0. 값이 클수록 비용이 벽 근처에서만 진하다 — rviz 가 얇아 보인다. 행동은 거의 같다


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
    #   **로컬만 따로 낮춰 볼 수 있다** (`local_inflation:=0.10`, 9/10 16:14 복귀 회차 — 작업공간 서쪽
    #   0.8 m 출입구 모서리에서 DWB 가 vx 0 으로 2 분 헤맴). 기본은 글로벌과 같은 0.25 — 시험용이지
    #   확정값이 아니다. 내접 반지름 0.186 아래로 내리면 Nav2 가 경고를 내고 벽에 붙을 수 있다
    local_inflation = float(LaunchConfiguration('local_inflation').perform(context) or INFLATION_RADIUS_M)
    for costmap, radius in (('local_costmap', local_inflation), ('global_costmap', INFLATION_RADIUS_M)):
        layer = config[costmap][costmap]['ros__parameters']['inflation_layer']
        layer['inflation_radius'] = radius
        layer['cost_scaling_factor'] = COST_SCALING
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
        DeclareLaunchArgument('local_inflation', default_value=str(INFLATION_RADIUS_M),
                              description='local_costmap 만의 inflation_radius [m]. 글로벌은 INFLATION_RADIUS_M 고정'),
        OpaqueFunction(function=_setup)])
