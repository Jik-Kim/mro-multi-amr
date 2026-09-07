"""셔틀 실주행 시험용 '리그' — 관찰할 것이 없는 보조 노드 셋을 한 터미널로 묶는다.

docs/shuttle_test_guide.md B-3 의 T4(세마포어) · T5(안전 감시) · T6(마커 주입) 에 해당한다.
셋 다 시험 중에 볼 로그가 거의 없다 (T4 는 토큰 2줄, T6 은 0줄). 그래서 묶어도 잃는 것이 없다.

**셔틀 노드(T7)·Nav2(T3)는 여기에 넣지 않는다.** 셔틀은 판정 7항목이 전부 나오는 주 관찰
대상이고(가이드 C-1), Nav2 는 실패 원인이 자기 로그에 있다. 한 터미널에 합치면 초당 수십 줄의
costmap 경고에 판정 로그가 묻혀 시험 자체가 성립하지 않는다.

쓰는 법
  ros2 launch mro_navigation shuttle_step1_rig.launch.py
  ros2 launch mro_navigation shuttle_step1_rig.launch.py namespace:=/robot4 bay:=B
  ros2 launch mro_navigation shuttle_step1_rig.launch.py inject_aruco:=false   # 3단계(실물 마커)

인자
  namespace      기본 /robot3
  bay            A/B/C. arena.yaml 의 bays.<bay>.aruco_id 를 읽어 주입 ID 를 정한다
  aruco_id       0 이면 bay 로부터 자동. 직접 주고 싶을 때만 지정 (2단계 오도착 시험 등)
  aruco_distance 기본 0.9 (m). 셔틀의 aruco_max_dist_m 1.5 안쪽이어야 한다
  aruco_rate     기본 5.0 (Hz). 셔틀 신선도 판정이 1.0 s 라 2 Hz 밑으로 내리지 말 것
  inject_aruco   기본 true. 실물 마커를 쓸 때 false
  semaphore      기본 true. false 로 두면 셔틀이 '단독 모드' 가 되어 판정 5 를 못 한다
  watchdog       기본 true

주의 — Ctrl+C 에 세마포어·워치독이 트레이스백을 뱉는다. 정상이다 (가이드 B-6). 두 노드는
  아직 종료 예외를 잡지 않는다. 리그를 내리면 P1 토큰도 같이 사라지므로 재시작하면 깨끗해진다.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import yaml

# 마커 주입 프로세스는 발행마다 'publishing #N' 을 찍는다. 5 Hz 씩 둘이면 초당 10줄이라
#   같은 터미널의 세마포어 토큰 2줄이 묻힌다. 화면 대신 로그 파일로 보낸다.
ARUCO_OUTPUT = 'log'


def _aruco_id_for_bay(bay):
    """arena.yaml 에서 베이의 aruco_id 를 읽는다. 값의 단일 출처는 params 다 (계약 5절).

    여기서 A=1·B=2·C=3 을 하드코딩하면 arena.yaml 과 두 곳이 되어 조용히 어긋난다.
    """
    try:
        arena_yaml = os.path.join(get_package_share_directory('mro_bringup'), 'params', 'arena.yaml')
    except PackageNotFoundError:
        raise RuntimeError('mro_bringup 을 찾지 못했다 — ws source 누락 (가이드 B-2). '
                           'aruco_id:=1 처럼 직접 넘기면 우회할 수 있다')
    with open(arena_yaml, encoding='utf-8') as f:
        bays = (yaml.safe_load(f) or {}).get('arena', {}).get('bays', {})
    if bay not in bays:
        raise RuntimeError(f'arena.yaml 에 베이 {bay} 가 없다 (있는 것: {sorted(bays)})')
    return int(bays[bay]['aruco_id'])


def _setup(context, *_args, **_kwargs):
    ns = LaunchConfiguration('namespace').perform(context)
    bay = LaunchConfiguration('bay').perform(context).strip().upper()
    aruco_id = int(LaunchConfiguration('aruco_id').perform(context))
    distance = float(LaunchConfiguration('aruco_distance').perform(context))
    rate = float(LaunchConfiguration('aruco_rate').perform(context))
    inject = LaunchConfiguration('inject_aruco').perform(context).lower() in ('true', '1', 'yes')
    use_sem = LaunchConfiguration('semaphore').perform(context).lower() in ('true', '1', 'yes')
    use_wd = LaunchConfiguration('watchdog').perform(context).lower() in ('true', '1', 'yes')

    if not ns.startswith('/'):
        ns = '/' + ns
    if aruco_id == 0:
        aruco_id = _aruco_id_for_bay(bay)

    actions = []

    # 세마포어 — 네임스페이스를 주지 않는다. /fleet/zone_token 은 절대 이름이고,
    #   로봇 네임스페이스 아래로 들어가면 다른 로봇이 같은 토큰을 못 본다.
    if use_sem:
        actions.append(Node(package='mro_fleet', executable='zone_semaphore_node',
                            name='zone_semaphore', output='screen'))
    else:
        actions.append(LogInfo(msg='[리그] 세마포어 꺼짐 — 셔틀이 단독 모드가 되어 판정 5(토큰 시점)를 못 한다'))

    # 안전 감시 — 셔틀보다 먼저 떠 있어야 한다. estop 은 전이 시점에만 발행되기 때문이다.
    #   파라미터가 없는 노드라 params 파일을 넘기지 않는다.
    if use_wd:
        actions.append(Node(package='mro_navigation', executable='safety_watchdog_node',
                            name='safety_watchdog', namespace=ns, output='screen',
                            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]))

    if inject:
        actions.append(LogInfo(msg=f'[리그] 마커 주입: {ns}/aruco/id={aruco_id} (베이 {bay}) · '
                                   f'distance={distance} m · {rate} Hz → 로그 파일로만 출력'))
        actions.append(ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-r', str(rate), f'{ns}/aruco/id',
                 'std_msgs/msg/Int32', f'{{data: {aruco_id}}}'],
            name='aruco_id_pub', output=ARUCO_OUTPUT))
        actions.append(ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-r', str(rate), f'{ns}/aruco/distance',
                 'std_msgs/msg/Float32', f'{{data: {distance}}}'],
            name='aruco_distance_pub', output=ARUCO_OUTPUT))
    else:
        actions.append(LogInfo(msg='[리그] 마커 주입 꺼짐 — 실물 ArUco 를 쓴다 (3단계)'))

    actions.append(LogInfo(msg='[리그] 준비됨. 다음: tools/shuttle_preflight.sh mission ' + ns))
    return actions


def generate_launch_description():
    """리그 인자를 선언하고 실제 구성은 OpaqueFunction 에 맡긴다 (bay → aruco_id 조회에 값이 필요)."""
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/robot3'),
        DeclareLaunchArgument('bay', default_value='A', description='A/B/C — arena.yaml 에서 aruco_id 를 읽는다'),
        DeclareLaunchArgument('aruco_id', default_value='0', description='0 이면 bay 로부터 자동'),
        DeclareLaunchArgument('aruco_distance', default_value='0.9'),
        DeclareLaunchArgument('aruco_rate', default_value='5.0', description='2 Hz 밑으로 내리지 말 것 (신선도 1.0 s)'),
        DeclareLaunchArgument('inject_aruco', default_value='true'),
        DeclareLaunchArgument('semaphore', default_value='true'),
        DeclareLaunchArgument('watchdog', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
