"""추종 실주행 시험용 '리그' — 작업자 추종에 필요한 것을 한 터미널로 묶는다.

박병후님의 `shuttle_step1_rig.launch.py` 와 같은 발상이다. 그쪽이 셔틀 주변(세마포어·안전
감시·마커)을 묶었고, 이쪽은 **추종 주변(작업자 검출·추종 노드·수동 신호)** 을 묶는다.
둘을 같이 띄우면 셔틀 → 추종 통합 시험이 터미널 6개로 끝난다 (종전 9개).

**Nav2(T3)·셔틀(T5)은 여기에 넣지 않는다.** Nav2 는 초당 수십 줄의 costmap 경고를 내고,
셔틀은 판정 7항목이 전부 나오는 주 관찰 대상이다. 한 터미널에 합치면 추종 로그가 묻힌다.

쓰는 법
  ros2 launch mro_navigation follow_rig.launch.py                      # 게이트 주입 + 손으로 시작
  ros2 launch mro_navigation follow_rig.launch.py arrive:=true         # 시작 신호까지 자동
  ros2 launch mro_navigation follow_rig.launch.py paired:=real         # 웹캠1·관제까지 띄웠을 때
  ros2 launch mro_navigation follow_rig.launch.py namespace:=/robot8 show_window:=false

인자
  namespace     기본 /robot3
  detection     기본 true. `amr_detection` 을 여기서 띄운다. 이미 다른 터미널에 띄웠으면 false
                — **카메라 구독자는 로봇당 검출 노드 1개** 규칙 때문에 두 번 띄우면 안 된다
                (AGENTS.md · RPi 디스커버리 부하)
  show_window   기본 true. SSH 로 붙었으면 false — 미리보기 창을 못 만들면 검출 노드가 죽는다
  paired        inject(기본) | real
                inject : 추종의 `target/paired` 를 `target/paired_manual` 로 돌리고 참을 주입한다.
                         웹캠1·관제가 없어 재식별 게이트를 열 수 없을 때 쓴다
                real   : 인지의 게이트를 그대로 쓴다. 웹캠1(작업자 등록)과 관제(담당 배정)가
                         떠 있어야 열린다 — 둘 중 하나만 없으면 영원히 거짓이다
  arrive        기본 false. true 면 `arrive_after_s` 뒤에 `shuttle/arrived` 를 세 번 보낸다.
                셔틀을 같이 띄웠으면 **false 로 두라** — 셔틀이 진짜 신호를 낸다
  arrive_after_s 기본 10.0 (s). 노드가 뜨고 측위가 잡힐 시간

**왜 시작 신호를 `-r 1 -t 3` 으로 보내나 (9/9 실주행에서 두 번 다 틀렸다)**
  · `--once` 는 유실된다. 구독자가 '보인다' 는 신호를 받으면 즉시 쏘는데 그때 DDS 연결
    악수가 안 끝났을 수 있고, `shuttle/arrived` 는 VOLATILE 이라 그 메시지는 버려진다.
  · `-r 1` 을 켜 둔 채 두면 **매초 사이클이 다시 시작된다.** 추종은 도착 신호마다 재탐지
    시계를 초기화하므로(`begin_cycle` → `rearm`) 좌표 표본이 영원히 쌓이지 않아
    `SEARCHING` 에서 못 나온다.
  · `-r 1 -t 3` 은 세 번 쏘고 스스로 죽는다 — 유실도 반복도 없다.
  게이트(`target/paired`)는 반대다. 계속 참이어야 하므로 `-r 2` 로 죽지 않게 둔다.

주의 — 주입 발행은 'publishing #N' 을 초당 두 줄씩 찍는다. 추종 로그가 묻히므로 화면 대신
  로그 파일로 보낸다 (셔틀 리그가 마커 주입에 쓴 것과 같은 방식).
"""
import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction,
                            TimerAction)
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# 주입 발행 로그를 화면에서 뺀다 — 초당 두 줄이면 추종 판정 로그가 묻힌다
INJECT_OUTPUT = 'log'
PAIRED_RATE = 2.0        # 인지가 1 Hz 로 거짓을 재발행하므로 그보다 빨라야 이긴다
ARRIVE_TIMES = 3         # 유실 대비. 세 번이면 충분하고 사이클을 다시 시작시키지 않는다


def _params(ns, extra_pkg=None):
    """넘길 params 파일 목록. 값의 단일 출처는 params 다 (계약 5절).

    **순서가 우선순위다** — 광역(`/**`) 파일이 먼저, 로봇별(`/robotN/**`) 이 마지막이어야
    robot3 의 `depth_k` 0.024 가 이긴다. 뒤집으면 조용히 0.0 으로 돈다 (I-040).
    `robot.launch.py` 와 같은 순서를 유지한다.
    """
    try:
        bringup = os.path.join(get_package_share_directory('mro_bringup'), 'params')
    except PackageNotFoundError as e:
        raise RuntimeError(
            f'mro_bringup 을 못 찾았다 — ws source 누락 '
            f'(source tools/shuttle_env.sh): {e}')
    files = [os.path.join(bringup, 'common.yaml')]
    if extra_pkg:
        try:
            files.append(os.path.join(
                get_package_share_directory(extra_pkg), 'config', 'perception.yaml'))
        except PackageNotFoundError as e:
            raise RuntimeError(f'{extra_pkg} 을 못 찾았다 — colcon build 를 다시 할 것: {e}')
    files.append(os.path.join(bringup, f'{ns.strip("/")}.yaml'))
    missing = [f for f in files if not os.path.exists(f)]
    if missing:
        raise RuntimeError(f'params 파일이 없다: {missing} — colcon build 를 다시 할 것')
    return files


def _setup(context, *_args, **_kwargs):
    ns = LaunchConfiguration('namespace').perform(context)
    if not ns.startswith('/'):
        ns = '/' + ns

    def flag(name):
        return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')

    detection = flag('detection')
    show_window = flag('show_window')
    arrive = flag('arrive')
    paired = LaunchConfiguration('paired').perform(context).strip().lower()
    after = float(LaunchConfiguration('arrive_after_s').perform(context))

    if paired not in ('inject', 'real'):
        raise RuntimeError(f'paired 는 inject 또는 real 이어야 한다 (받은 값: {paired})')

    tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    actions = []

    # ── 작업자 검출 (인지 소관 노드. 우리는 띄우기만 한다) ──
    if detection:
        actions.append(Node(
            package='mro_perception', executable='amr_detection_node',
            name='amr_detection', namespace=ns, output='screen',
            parameters=[*_params(ns, 'mro_perception'), {'show_window': show_window}],
            remappings=tf))
    else:
        actions.append(LogInfo(
            msg='[추종 리그] 검출 노드 꺼짐 — 다른 터미널에 이미 떠 있어야 한다. '
                '카메라 구독자는 로봇당 하나뿐이다'))

    # ── 추종 (우리 노드) ──
    #   게이트를 주입할 때만 토픽을 가른다. 같은 토픽에 인지의 거짓과 우리의 참이 함께
    #   들어가면 게이트가 초당 여러 번 열렸다 닫혀 추종이 시작조차 못 한다 (9/9 실주행)
    follow_remap = list(tf)
    if paired == 'inject':
        follow_remap.append(('target/paired', 'target/paired_manual'))
    actions.append(Node(
        package='mro_navigation', executable='follow_target_node',
        name='follow_target', namespace=ns, output='screen',
        parameters=_params(ns), remappings=follow_remap))

    # ── 게이트 주입 ──
    if paired == 'inject':
        actions.append(LogInfo(
            msg=f'[추종 리그] 재식별 게이트 주입: {ns}/target/paired_manual = true · '
                f'{PAIRED_RATE} Hz → 로그 파일로만 출력. '
                f'웹캠1·관제를 띄웠으면 paired:=real 로 쓸 것'))
        actions.append(ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-r', str(PAIRED_RATE),
                 f'{ns}/target/paired_manual', 'std_msgs/msg/Bool', '{data: true}'],
            name='paired_pub', output=INJECT_OUTPUT))
    else:
        actions.append(LogInfo(
            msg='[추종 리그] 재식별 게이트는 인지가 낸다 — 웹캠1(작업자 등록)과 '
                '관제(담당 배정)가 둘 다 떠 있어야 열린다. 하나라도 없으면 영원히 거짓이다'))

    # ── 시작 신호 ──
    if arrive:
        actions.append(LogInfo(
            msg=f'[추종 리그] {after:.0f} s 뒤 {ns}/shuttle/arrived 를 {ARRIVE_TIMES} 번 보낸다. '
                f'셔틀을 같이 띄웠으면 arrive:=false 로 둘 것 — 셔틀이 진짜 신호를 낸다'))
        actions.append(TimerAction(period=after, actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-r', '1', '-t', str(ARRIVE_TIMES),
                 f'{ns}/shuttle/arrived', 'std_msgs/msg/Bool', '{data: true}'],
            name='arrived_pub', output=INJECT_OUTPUT)]))
    else:
        actions.append(LogInfo(
            msg=f'[추종 리그] 시작 신호는 손으로 — 셔틀이 없으면 이렇게 보낸다 '
                f'(세 번 쏘고 스스로 죽는다):\n'
                f'  ros2 topic pub -r 1 -t {ARRIVE_TIMES} {ns}/shuttle/arrived '
                f'std_msgs/Bool "{{data: true}}"'))

    actions.append(LogInfo(msg=f'[추종 리그] 준비됨. 상태 관찰: ros2 topic echo {ns}/approach/status'))
    return actions


def generate_launch_description():
    """인자를 선언하고 구성은 OpaqueFunction 에 맡긴다 (params 경로 조회에 값이 필요하다)."""
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/robot3'),
        DeclareLaunchArgument('detection', default_value='true',
                              description='amr_detection 을 여기서 띄운다. 카메라 구독자는 로봇당 하나뿐이다'),
        DeclareLaunchArgument('show_window', default_value='true',
                              description='SSH 면 false — 창을 못 만들면 검출 노드가 죽는다'),
        DeclareLaunchArgument('paired', default_value='inject',
                              description='inject: 게이트를 주입 · real: 인지의 게이트를 그대로'),
        DeclareLaunchArgument('arrive', default_value='false',
                              description='true 면 시작 신호를 자동으로. 셔틀을 띄웠으면 false'),
        DeclareLaunchArgument('arrive_after_s', default_value='10.0'),
        OpaqueFunction(function=_setup),
    ])
