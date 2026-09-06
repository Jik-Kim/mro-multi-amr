"""ROS 네트워크 없이 관제 콜백의 분기 규칙을 검증한다."""
from types import SimpleNamespace

from mro_fleet.nodes.fleet_manager_node import FleetManager


class FakeDatabase:
    """콜백이 기록한 이벤트를 보관하는 테스트 대역."""

    def __init__(self):
        """빈 이벤트 목록을 준비한다."""
        self.events = []

    def event(self, *values):
        """전달된 이벤트 값을 순서대로 보관한다."""
        self.events.append(values)


class FakePublisher:
    """발행된 ROS 메시지를 보관하는 테스트 대역."""

    def __init__(self):
        """빈 메시지 목록을 준비한다."""
        self.messages = []

    def publish(self, message):
        """발행 요청된 메시지를 보관한다."""
        self.messages.append(message)


def test_return_all_accepts_empty_robot_namespace():
    """RETURN_ALL이 빈 robot_ns여도 두 로봇에 발행되는지 확인한다."""
    fake = SimpleNamespace(
        robots=['/robot3', '/robot4'],
        robot={'/robot3': {}, '/robot4': {}},
        db=FakeDatabase(),
        pub_finish={
            '/robot3': FakePublisher(),
            '/robot4': FakePublisher(),
        },
        _now_s=lambda: 10.0,
    )
    request = SimpleNamespace(robot_ns='', command='RETURN_ALL')
    response = SimpleNamespace(ok=False, message='')

    result = FleetManager._operator_cb(fake, request, response)

    assert result.ok is True
    assert len(fake.pub_finish['/robot3'].messages) == 1
    assert len(fake.pub_finish['/robot4'].messages) == 1
    assert all(event[2] == 'OPERATOR' for event in fake.db.events)


def test_low_battery_rejects_mission_and_records_reason():
    """배터리 25% 미만 요청이 거부되고 사유가 남는지 확인한다."""
    fake = SimpleNamespace(
        worker_robot={'W1': '/robot3'},
        robot={'/robot3': {'battery': 0.24}},
        battery_min=0.25,
        db=FakeDatabase(),
        _now_s=lambda: 20.0,
    )
    request = SimpleNamespace(worker_id='W1', bay='A')
    response = SimpleNamespace(
        accepted=True,
        mission_id='',
        message='',
    )

    result = FleetManager._request_cb(fake, request, response)

    assert result.accepted is False
    assert '배터리 임계 미만' in result.message
    assert fake.db.events == [
        (20.0, '/robot3', 'ASSIGN_REFUSED_BATTERY', 0.24)
    ]
