"""I-037 회귀 검증: Nav2 성공과 신선한 위치 검증이 함께 성립해야 다음 단계로 간다."""
import math

from geometry_msgs.msg import TransformStamped
import pytest
from rclpy.time import Time

from test_shuttle_mission_fsm import (
    assigned, fresh_marker, make_node, stub_token,
)
from test_shuttle_mission_fsm import ros  # noqa: F401

pytestmark = pytest.mark.usefixtures('ros')


def feed_pose(node, x, y, yaw=0.0, age=0.0):
    """좌표와 시각을 독립적으로 주입한다. 반환 함수는 매 조회 때 새 stamp를 만든다."""
    def lookup(*args, **kwargs):
        transform = TransformStamped()
        transform.header.stamp = Time(seconds=node.now() - age).to_msg()
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2)
        transform.transform.rotation.w = math.cos(yaw / 2)
        return transform
    node.tf_buffer.lookup_transform = lookup


@pytest.mark.parametrize('offset', [0.39, 0.55])
def test_success_outside_tolerance_does_not_advance_mission(offset):
    """신선한 TF라도 39/55 cm 오차이면 AT_BAY·WAITING·arrived를 내지 않는다."""
    node = make_node(arrival_xy_tolerance_m=0.25)
    stub_token(node, [True])
    fresh_marker(node)
    feed_pose(node, -0.78 + offset, 0.138)
    try:
        node.run_mission(assigned())
        assert node.status_log == ['SHUTTLE', 'FAILED', 'READY']
        assert node.arrived_log == []
        assert len(node.navigator.goals()) == 1
        assert node.token_log == []
    finally:
        node.destroy_node()


@pytest.mark.parametrize('age', [2.0, -2.0])
def test_stale_or_future_pose_is_not_arrival(age):
    """목표 좌표라도 오래된 TF·미래 TF이면 성공하지 않고 제한시간에 포기한다."""
    node = make_node(tf_pose_timeout_s=0.1, tf_max_age_s=0.3)
    feed_pose(node, -0.78, 0.138, age=age)
    try:
        from mro_navigation.nodes.shuttle_mission_node import MissionAbort
        started = node.now()
        with pytest.raises(MissionAbort, match='신선한 TF'):
            node.goto(node.pose_of(-0.78, 0.138, 0.0), '베이 A')
        assert node.now() - started < 2.0
    finally:
        node.destroy_node()


def test_missing_pose_cannot_start_advance():
    """TF가 없으면 베이 waypoint로 대신 계산해 전진시키지 않는다."""
    node = make_node(simulate_pose=False, tf_pose_timeout_s=0.1)
    try:
        from mro_navigation.nodes.shuttle_mission_node import MissionAbort
        with pytest.raises(MissionAbort, match='신선한 TF'):
            node.advance_into_bay(node.arena['bays']['A'])
        assert node.navigator.goals() == []
    finally:
        node.destroy_node()


def test_waits_for_observation_after_success():
    """오래되지 않았더라도 액션 완료 이전 관측만으로 도착을 확정하지 않는다."""
    node = make_node(tf_pose_timeout_s=0.1, tf_max_age_s=10.0)
    feed_pose(node, -0.78, 0.138, age=0.05)
    try:
        from mro_navigation.nodes.shuttle_mission_node import MissionAbort
        with pytest.raises(MissionAbort, match='신선한 TF'):
            node.goto(node.pose_of(-0.78, 0.138, 0.0), '베이 A')
    finally:
        node.destroy_node()


def test_fresh_pose_within_tolerance_is_accepted():
    """기존 허용오차 안의 20 cm 정차는 통과한다. Nav2 정밀도 정책은 바꾸지 않는다."""
    node = make_node(arrival_xy_tolerance_m=0.25)
    feed_pose(node, -0.58, 0.138)
    try:
        node.goto(node.pose_of(-0.78, 0.138, 0.0), '베이 A')
    finally:
        node.destroy_node()


def test_advance_uses_updated_start_pose():
    """지연 좌표가 새 좌표로 갱신된 뒤 그 자리에서 0.3 m 목표를 만든다."""
    node = make_node(tf_pose_timeout_s=1.0)
    feed_pose(node, -0.60, -0.20, age=2.0)
    timer = node.create_timer(0.05, lambda: feed_pose(node, -0.65, -0.02, yaw=0.0))
    # 전진 후 TF는 navigator 대역의 도착 좌표를 반환한다.
    original_goto = node.navigator.goToPose

    def goto(pose):
        node.destroy_timer(timer)
        feed_pose(node, pose.pose.position.x, pose.pose.position.y)
        return original_goto(pose)

    node.navigator.goToPose = goto
    try:
        node.advance_into_bay(node.arena['bays']['A'])
        assert node.navigator.goals() == [('goto', -0.35, -0.02)]
    finally:
        node.destroy_node()


@pytest.mark.parametrize('name', ['arrival_xy_tolerance_m', 'tf_pose_timeout_s', 'tf_max_age_s'])
@pytest.mark.parametrize('value', [0.0, -1.0, float('nan'), float('inf')])
def test_invalid_tolerance_does_not_send_goal(name, value):
    """무효 허용오차로 도착 검증이 무력화되는 것을 막는다."""
    node = make_node(**{name: value})
    try:
        from mro_navigation.nodes.shuttle_mission_node import MissionAbort
        with pytest.raises(MissionAbort, match='유한한 양수'):
            node.goto(node.pose_of(-0.78, 0.138, 0.0), '베이 A')
        assert node.navigator.goals() == []
    finally:
        node.destroy_node()


def test_waiting_position_error_does_not_publish_arrived():
    """베이를 통과했어도 대기공간 오차가 크면 배송 성공을 내지 않는다."""
    node = make_node()
    stub_token(node, [True])
    fresh_marker(node)
    original_goto = node.navigator.goToPose

    def goto(pose):
        accepted = original_goto(pose)
        if len(node.navigator.goals()) == 3:
            feed_pose(node, pose.pose.position.x + 0.55, pose.pose.position.y)
        return accepted

    node.navigator.goToPose = goto
    try:
        node.run_mission(assigned())
        assert node.status_log == ['SHUTTLE', 'AT_BAY', 'SHUTTLE', 'FAILED', 'READY']
        assert node.arrived_log == []
        assert node.token_log == ['acquire', 'release']
    finally:
        node.destroy_node()


def test_tlim_expiring_inside_completion_poll_is_rejected():
    """완료 확인을 기다리는 사이 T_lim이 지나도 늦은 성공을 내지 않는다."""
    node = make_node(t_lim_s=0.1)
    node.t0 = node.now()

    def complete_late():
        node.t0 = node.now() - 1.0
        return True

    try:
        from mro_navigation.nodes.shuttle_mission_node import MissionAbort
        with pytest.raises(MissionAbort, match='T_lim'):
            node.wait(complete_late)
    finally:
        node.destroy_node()


def test_expired_mission_cannot_pass_already_true_condition():
    """T_lim 이후 완료 콜백이 준비돼 있어도 성공을 내지 않는다."""
    node = make_node(t_lim_s=0.1)
    node.t0 = node.now() - 1.0
    try:
        from mro_navigation.nodes.shuttle_mission_node import MissionAbort
        with pytest.raises(MissionAbort, match='T_lim'):
            node.wait(lambda: True)
    finally:
        node.destroy_node()
