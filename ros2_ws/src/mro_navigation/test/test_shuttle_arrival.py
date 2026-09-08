"""I-037 회귀 검증: Nav2 성공과 신선한 위치 검증이 함께 성립해야 다음 단계로 간다."""
import math

from geometry_msgs.msg import TransformStamped
import pytest
from rclpy.time import Time

from test_shuttle_mission_fsm import (
    CAMERA_FRAME, OPTICAL_TO_BASE_Q, assigned, fresh_marker, make_node, marker_frames, stub_token,
)
from test_shuttle_mission_fsm import ros  # noqa: F401

pytestmark = pytest.mark.usefixtures('ros')


def _camera_transform():
    """base_link ← 카메라 광학 프레임. 마커 표본을 base_link 로 옮길 때 쓴다 (고정 회전)."""
    transform = TransformStamped()
    q = transform.transform.rotation
    q.x, q.y, q.z, q.w = OPTICAL_TO_BASE_Q
    return transform


def feed_pose(node, x, y, yaw=0.0, age=0.0):
    """좌표와 시각을 독립적으로 주입한다. 반환 함수는 매 조회 때 새 stamp를 만든다.

    카메라 TF 는 이 주입과 무관하게 늘 성립한다 — map→base_link 의 신선도만 시험하려는
    것이지, 마커를 base_link 로 옮기는 고정 변환까지 없애려는 것이 아니다.
    """
    def lookup(target, source, *args, **kwargs):
        if source == CAMERA_FRAME:
            return _camera_transform()
        transform = TransformStamped()
        transform.header.stamp = Time(seconds=node.now() - age).to_msg()
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2)
        transform.transform.rotation.w = math.cos(yaw / 2)
        return transform
    node.tf_buffer.lookup_transform = lookup


def camera_tf_only(node):
    """카메라 TF 만 주고 map→base_link 는 없앤다 — 위치를 모르는 상태를 만든다."""
    def lookup(target, source, *args, **kwargs):
        if source == CAMERA_FRAME:
            return _camera_transform()
        raise LookupError(f'{target}←{source} 없음')
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


def test_camera_tf_arriving_late_does_not_lose_the_run():
    """tf_static 이 늦게 와도 기다렸다가 접근한다 — 9/8 실측에서 처음 몇 초는 조회가 실패했다.

    기다리지 않으면 그 창에 걸린 회차가 '마커 표본 없음' 으로 통째로 날아간다.
    """
    yaw_ref = -3.050
    node = make_node(bay_align_enable=True, bay_align_max_iter=1, tf_pose_timeout_s=2.0)
    late = {'ready': False}
    real = _camera_transform

    def lookup(target, source, *args, **kwargs):
        if source == CAMERA_FRAME:
            if not late['ready']:
                raise LookupError('frame does not exist')     # tf_static 아직 안 옴
            return real()
        transform = TransformStamped()
        transform.header.stamp = Time(seconds=node.now()).to_msg()
        transform.transform.translation.x = -0.65
        transform.transform.translation.y = -0.02
        transform.transform.rotation.z = math.sin(yaw_ref / 2)
        transform.transform.rotation.w = math.cos(yaw_ref / 2)
        return transform

    node.tf_buffer.lookup_transform = lookup
    node.create_timer(0.3, lambda: late.__setitem__('ready', True))
    fresh_marker(node)                # aruco/pose 는 ID 가 없다 — id·거리로 걸러진다
    marker_frames(node, fwd=1.0, lat=0.0)
    try:
        node.approach_and_align(node.arena['bays']['A'])
        assert node.navigator.moves() == [
            ('drive', round(1.0 - node.p('bay_marker_range_m'), 3))]
    finally:
        node.destroy_node()


def test_missing_pose_cannot_start_approach():
    """자기 위치를 모르면 마커가 보여도 접근을 시작하지 않는다.

    접근은 마커 상대 이동이지만 회전량 φ 를 '측량된 베이 방위 − 지금 방위' 로 잡으므로
    map→base_link 가 필요하다. 없는 채로 움직이면 방위를 엉뚱하게 돌린다.
    """
    node = make_node(bay_align_enable=True, tf_pose_timeout_s=0.1)
    camera_tf_only(node)
    fresh_marker(node)                # aruco/pose 는 ID 가 없다 — id·거리로 걸러진다
    marker_frames(node, fwd=1.0, lat=0.0)
    try:
        from mro_navigation.nodes.shuttle_mission_node import MissionAbort
        with pytest.raises(MissionAbort, match='신선한 TF'):
            node.approach_and_align(node.arena['bays']['A'])
        assert node.navigator.goals() == []
        assert node.navigator.moves() == []
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


def test_approach_waits_for_updated_pose():
    """지연 좌표가 새 좌표로 갱신되면 포기하지 않고 그 방위로 접근을 계산한다.

    옛 '0.3 m 전진' 시험을 접근으로 옮긴 것이다 (I-037). 전진은 사라졌지만 '신선한
    map→base_link 없이는 다음 단계로 가지 않는다' 는 요구는 접근에 그대로 남아 있다.
    베이 A 의 측량 방위(-3.050 rad)로 서 있으면 회전은 0 이고, 마커까지 1.0 m 에서
    목표 거리까지 남는 만큼만 직진이 나와야 한다.
    """
    yaw_ref = -3.050
    node = make_node(bay_align_enable=True, bay_align_max_iter=1, tf_pose_timeout_s=1.0)
    feed_pose(node, -0.65, -0.02, yaw=yaw_ref, age=2.0)          # 처음엔 2 초 지연된 좌표뿐
    node.create_timer(0.05, lambda: feed_pose(node, -0.65, -0.02, yaw=yaw_ref))
    fresh_marker(node)                # aruco/pose 는 ID 가 없다 — id·거리로 걸러진다
    marker_frames(node, fwd=1.0, lat=0.0)
    try:
        node.approach_and_align(node.arena['bays']['A'])
        assert node.navigator.moves() == [
            ('drive', round(1.0 - node.p('bay_marker_range_m'), 3))]
        assert node.navigator.goals() == []
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
        if len(node.navigator.goals()) == 2:      # 대기공간 목표 (전진은 이제 상대 이동이다)
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
