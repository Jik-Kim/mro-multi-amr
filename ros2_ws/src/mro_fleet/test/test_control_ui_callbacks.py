"""ROS 네트워크 없이 FR-13 관제 UI 콜백을 검증한다."""
import sys
import threading
from types import ModuleType
from types import SimpleNamespace

# 개발 PC에 Flask 실행 의존성이 없어도 ROS 콜백 단위 테스트는 수행한다.
try:
    import flask  # noqa: F401
except ModuleNotFoundError:
    flask_stub = ModuleType('flask')
    flask_stub.Flask = object
    flask_stub.jsonify = lambda **values: values
    flask_stub.render_template = lambda *_args, **_kwargs: ''
    flask_stub.request = SimpleNamespace(form={})
    sys.modules['flask'] = flask_stub

from mro_fleet.nodes.control_ui_node import ControlUI


class FakeLogger:
    """UI 콜백 경고를 보관하는 로거 대역."""

    def __init__(self):
        """빈 경고 목록을 준비한다."""
        self.warnings = []

    def warn(self, message):
        """경고 로그를 보관한다."""
        self.warnings.append(message)


class FakePublisher:
    """발행한 Intervention 메시지를 보관한다."""

    def __init__(self):
        """빈 메시지 목록을 준비한다."""
        self.messages = []

    def publish(self, message):
        """발행 메시지를 순서대로 보관한다."""
        self.messages.append(message)


def make_clock():
    """Intervention 헤더에 사용할 고정 ROS 시각 대역을 만든다."""
    stamp = SimpleNamespace(sec=10, nanosec=20)
    now = SimpleNamespace(to_msg=lambda: stamp)
    return SimpleNamespace(now=lambda: now)


def test_pair_state_is_kept_per_robot_when_status_updates():
    """pair_state와 FleetStatus가 같은 로봇 상태에 합쳐지는지 확인한다."""
    fake = SimpleNamespace(
        status={},
        status_lock=threading.Lock(),
        get_logger=lambda: FakeLogger(),
    )
    ControlUI._pair_state_cb(
        fake, '/robot3', SimpleNamespace(data='GATE_FAIL')
    )
    status_message = SimpleNamespace(
        robot_ns='/robot3',
        state='SEARCHING',
        battery=0.8,
        x=1.0,
        y=2.0,
        yaw=0.1,
        mission_id='M001',
        paired_worker='',
        estop=False,
    )

    ControlUI._status_cb(fake, status_message)

    assert fake.status['/robot3']['pair_state'] == 'GATE_FAIL'
    assert fake.status['/robot3']['state'] == 'SEARCHING'


def test_pair_states_of_two_robots_do_not_mix():
    """robot3와 robot8의 페어링 상태가 서로 덮이지 않는지 확인한다."""
    fake = SimpleNamespace(
        status={},
        status_lock=threading.Lock(),
        get_logger=lambda: FakeLogger(),
    )

    ControlUI._pair_state_cb(
        fake, '/robot3', SimpleNamespace(data='GATE_FAIL')
    )
    ControlUI._pair_state_cb(
        fake, '/robot8', SimpleNamespace(data='PAIRED')
    )

    assert fake.status['/robot3']['pair_state'] == 'GATE_FAIL'
    assert fake.status['/robot8']['pair_state'] == 'PAIRED'


def test_accepted_manual_pair_records_one_intervention():
    """인지 노드가 승인한 명시 페어링만 개입으로 한 번 기록하는지 확인한다."""
    publisher = FakePublisher()
    fake = SimpleNamespace(
        cli_manual_pair=object(),
        manual_pair_timeout_s=3.0,
        pub_int=publisher,
        get_clock=make_clock,
        _call=lambda _client, _request, timeout_sec: SimpleNamespace(
            accepted=True,
            reason='',
        ),
    )

    accepted, message = ControlUI._request_manual_pair(
        fake, '/robot3', 'W1'
    )

    assert accepted is True
    assert message == '명시 페어링 완료'
    assert len(publisher.messages) == 1
    intervention = publisher.messages[0]
    assert intervention.robot_ns == '/robot3'
    assert intervention.kind == 'MANUAL_PAIR'
    assert intervention.note == 'W1'


def test_rejected_manual_pair_does_not_record_intervention():
    """인지 노드가 거절한 요청은 IPH 개입으로 기록하지 않는지 확인한다."""
    publisher = FakePublisher()
    fake = SimpleNamespace(
        cli_manual_pair=object(),
        manual_pair_timeout_s=3.0,
        pub_int=publisher,
        get_clock=make_clock,
        _call=lambda _client, _request, timeout_sec: SimpleNamespace(
            accepted=False,
            reason='NO_TARGET',
        ),
    )

    accepted, message = ControlUI._request_manual_pair(
        fake, '/robot8', 'W2'
    )

    assert accepted is False
    assert message == 'NO_TARGET'
    assert publisher.messages == []


def test_missing_manual_pair_response_does_not_report_success():
    """서비스 응답이 없을 때 성공 또는 개입으로 처리하지 않는지 확인한다."""
    publisher = FakePublisher()
    fake = SimpleNamespace(
        cli_manual_pair=object(),
        manual_pair_timeout_s=3.0,
        pub_int=publisher,
        get_clock=make_clock,
        _call=lambda _client, _request, timeout_sec: None,
    )

    accepted, message = ControlUI._request_manual_pair(
        fake, '/robot3', 'W1'
    )

    assert accepted is False
    assert '응답 없음' in message
    assert publisher.messages == []
