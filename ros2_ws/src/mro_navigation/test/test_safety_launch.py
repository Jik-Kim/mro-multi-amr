"""Nav2 출력 변경이 다른 제어 파라미터에 영향을 주지 않는지 확인한다."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
import yaml


def test_only_collision_monitor_output_changes(tmp_path, monkeypatch):
    path = Path(__file__).parents[1] / 'launch' / 'safety_nav2.launch.py'
    spec = importlib.util.spec_from_file_location('safety_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config = {'collision_monitor': {'ros__parameters': {'cmd_vel_out_topic': 'cmd_vel',
              'source_timeout': 1.0}}, 'behavior_server': {'ros__parameters': {'max_rotational_vel': 1.0}}}
    source = tmp_path / 'nav2.yaml'
    source.write_text(yaml.safe_dump(config))
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: '/tmp/test-tb4')
    original = module.tempfile.NamedTemporaryFile
    files = []

    def temporary(**kwargs):
        file = original(dir=tmp_path, **kwargs)
        files.append(Path(file.name))
        return file

    monkeypatch.setattr(module.tempfile, 'NamedTemporaryFile', temporary)
    context = LaunchContext()
    context.launch_configurations.update(namespace='/robot3', params_file=str(source), use_sim_time='false')
    module._setup(context)
    output = yaml.safe_load(files[0].read_text())
    assert output['collision_monitor']['ros__parameters']['cmd_vel_out_topic'] == 'cmd_vel_navigation'
    output['collision_monitor']['ros__parameters']['cmd_vel_out_topic'] = 'cmd_vel'
    assert output == config
    assert yaml.safe_load(source.read_text()) == config
