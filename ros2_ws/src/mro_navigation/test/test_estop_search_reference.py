"""ROS 통신 없이 실제 복구 분기와 탐색 방위 계산을 검증한다."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest


def methods():
    source = Path(__file__).parents[1] / 'mro_navigation/nodes/follow_target_node.py'
    tree = ast.parse(source.read_text())
    selected = [n for cls in tree.body if isinstance(cls, ast.ClassDef)
                for n in cls.body if isinstance(n, ast.FunctionDef)
                and n.name in ('step', 'last_seen_bearing')]
    namespace = {'math': math, 'robot_xy': lambda buffer: (0, 0)}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(source), 'exec'), namespace)
    return namespace


def test_clear_keeps_search_reference_but_discards_old_follow_goal():
    node = SimpleNamespace(finish_req=False, estop=False, safety_resume=True,
                           motion_pause=SimpleNamespace(ready=True), state='FOLLOWING',
                           recovery_worker_id='worker_a', worker_id='worker_a',
                           follow_est=(0, 2), follow_goal=(0, 1), last_target=(0, 2),
                           recovery_search_point=None, tf_buffer=None,
                           map_yaw=lambda: math.pi / 2,
                           begin_reacquire=lambda reason: 'SEARCHING')
    code = methods()
    assert code['step'](node) == 'SEARCHING'
    assert node.worker_id == node.recovery_worker_id == 'worker_a'
    assert node.follow_est is node.follow_goal is node.last_target is None
    assert node.recovery_search_point == (0, 2)
    assert code['last_seen_bearing'](node) == pytest.approx(0)
    node.map_yaw = lambda: 0
    assert code['last_seen_bearing'](node) == pytest.approx(90)


def test_new_observation_takes_priority_over_recovery_reference():
    node = SimpleNamespace(follow_est=(2, 0), follow_goal=None,
                           recovery_search_point=(0, 2), tf_buffer=None, map_yaw=lambda: 0)
    assert methods()['last_seen_bearing'](node) == pytest.approx(0)
