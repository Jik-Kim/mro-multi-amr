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
              'source_timeout': 1.0}},
              'behavior_server': {'ros__parameters': {'max_rotational_vel': 1.0, 'min_rotational_vel': 0.4,
                                                      'rotational_acc_lim': 3.2}},
              'local_costmap': {'local_costmap': {'ros__parameters': {
                  'inflation_layer': {'inflation_radius': 0.15, 'cost_scaling_factor': 4.0}, 'resolution': 0.06}}},
              'global_costmap': {'global_costmap': {'ros__parameters': {
                  'inflation_layer': {'inflation_radius': 0.15, 'cost_scaling_factor': 4.0}}}}}
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
    # 회전 속도(I-051)와 inflation(I-009)도 이 런치가 정한다 — 그 셋 말고는 아무것도 안 바뀐다
    bs = output['behavior_server']['ros__parameters']
    assert (bs['max_rotational_vel'], bs['min_rotational_vel']) == (module.SPIN_MAX_RAD_S, module.SPIN_MIN_RAD_S)
    assert module.SPIN_MAX_RAD_S <= 0.3, '1.0 rad/s 는 odom 지연 0.35 s 에 20° 를 지나친다'
    for cm in ('local_costmap', 'global_costmap'):
        assert output[cm][cm]['ros__parameters']['inflation_layer']['inflation_radius'] == module.INFLATION_RADIUS_M
        assert module.INFLATION_RADIUS_M > 0.186, 'inflation 이 내접 반지름보다 작으면 벽에 붙어 달린다 (9/10 세 회차)'
        assert output[cm][cm]['ros__parameters']['inflation_layer']['cost_scaling_factor'] == module.COST_SCALING
        output[cm][cm]['ros__parameters']['inflation_layer']['inflation_radius'] = 0.15
        output[cm][cm]['ros__parameters']['inflation_layer']['cost_scaling_factor'] = 4.0
    bs['max_rotational_vel'], bs['min_rotational_vel'] = 1.0, 0.4
    output['collision_monitor']['ros__parameters']['cmd_vel_out_topic'] = 'cmd_vel'
    assert output == config
    assert yaml.safe_load(source.read_text()) == config
