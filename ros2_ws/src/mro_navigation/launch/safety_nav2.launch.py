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


def _setup(context):
    ns = LaunchConfiguration('namespace').perform(context)
    if not ns.strip('/'):
        raise RuntimeError('로봇 namespace가 필요하다')
    source = LaunchConfiguration('params_file').perform(context)
    with open(source, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    params = config['collision_monitor']['ros__parameters']
    params['cmd_vel_out_topic'] = 'cmd_vel_navigation'
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
