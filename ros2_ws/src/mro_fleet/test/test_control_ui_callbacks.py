"""ROS 네트워크 없이 FR-13 관제 UI 콜백을 검증한다."""
import os
import sys
import threading
from types import ModuleType
from types import SimpleNamespace
from unittest.mock import patch

# 개발 PC에 Flask 실행 의존성이 없어도 ROS 콜백 단위 테스트는 수행한다.
try:
    import flask  # noqa: F401
except ModuleNotFoundError:
    flask_stub = ModuleType('flask')
    flask_stub.Flask = object
    flask_stub.jsonify = lambda **values: values
    flask_stub.render_template = lambda *_args, **_kwargs: ''
    flask_stub.request = SimpleNamespace(form={})
    flask_stub.send_file = lambda *_args, **_kwargs: None
    sys.modules['flask'] = flask_stub

from mro_fleet.nodes import robot_control_node
from mro_fleet.nodes.MonitorWebApp import MonitorWebApp
from mro_fleet.nodes.MonitorWebApp import load_dock_landmarks
from mro_fleet.nodes.MonitorWebApp import load_map_config
from mro_fleet.nodes.robot_control_node import RobotControlNode


TEMPLATE_PATH = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), 'templates', 'index.html'
)


def test_dashboard_keeps_existing_control_endpoints():
    """대시보드가 기존 요청·복귀·명시 페어링 API를 계속 호출하는지 확인한다."""
    with open(TEMPLATE_PATH, encoding='utf-8') as template_file:
        html = template_file.read()

    assert "postForm('/request'" in html
    assert "postForm('/command'" in html
    assert "postForm('/manual_pair'" in html
    assert "command:'RETURN'" in html
    assert "command:'RETURN_ALL'" in html


def test_monitor_web_app_registers_dashboard_routes():
    """웹 파트가 기존 관제 URL을 기능별 메서드에 연결하는지 확인한다."""
    control = SimpleNamespace(
        namespace='/robot3',
        map_yaml_path='',
        arena_yaml_path='',
        get_logger=lambda: SimpleNamespace(warn=lambda _message: None),
    )
    monitor = MonitorWebApp([control], os.path.dirname(TEMPLATE_PATH))

    routes = {rule.rule for rule in monitor.app.url_map.iter_rules()}
    assert routes.issuperset({
        '/', '/request', '/command', '/intervention', '/manual_pair',
        '/status', '/map/metadata', '/map/image',
    })


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
        namespace='/robot3',
        status={},
        status_lock=threading.Lock(),
        get_logger=lambda: FakeLogger(),
    )
    RobotControlNode._pair_state_cb(
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

    RobotControlNode._status_cb(fake, status_message)

    assert fake.status['/robot3']['pair_state'] == 'GATE_FAIL'
    assert fake.status['/robot3']['state'] == 'SEARCHING'


def test_approach_status_is_kept_per_robot_when_fleet_status_updates():
    """approach/status가 FleetStatus 갱신 후에도 유지되는지 확인한다."""
    fake = SimpleNamespace(
        namespace='/robot3',
        status={},
        status_lock=threading.Lock(),
        get_logger=lambda: FakeLogger(),
    )
    RobotControlNode._receive_robot_status(
        fake, '/robot3', SimpleNamespace(data='APPROACH1')
    )
    status_message = SimpleNamespace(
        robot_ns='/robot3',
        state='ACTIVE',
        battery=0.8,
        x=1.0,
        y=2.0,
        yaw=0.1,
        mission_id='M001',
        paired_worker='W1',
        estop=False,
    )

    RobotControlNode._status_cb(fake, status_message)

    assert fake.status['/robot3']['approach_state'] == 'APPROACH_1'


def test_invalid_approach_status_is_ignored():
    """계약에 없는 로봇 상태는 저장하지 않고 경고를 남긴다."""
    logger = FakeLogger()
    fake = SimpleNamespace(
        status={},
        status_lock=threading.Lock(),
        get_logger=lambda: logger,
    )

    RobotControlNode._receive_robot_status(
        fake, '/robot8', SimpleNamespace(data='UNKNOWN')
    )

    assert fake.status == {}
    assert len(logger.warnings) == 1


def test_battery_status_is_stored_by_instance_namespace():
    """배터리 잔량이 인스턴스의 로봇 namespace에 저장된다."""
    fake = SimpleNamespace(
        namespace='/robot3',
        status={},
        status_lock=threading.Lock(),
    )

    with patch.object(
        robot_control_node.time, 'monotonic', return_value=10.0
    ):
        RobotControlNode._receive_battery_status(
            fake, SimpleNamespace(percentage=0.73)
        )

    assert fake.status['/robot3']['battery'] == 0.73
    assert fake.status['/robot3']['_received_monotonic'] == 10.0


def test_pair_states_of_two_robots_do_not_mix():
    """robot3와 robot8의 페어링 상태가 서로 덮이지 않는지 확인한다."""
    fake = SimpleNamespace(
        status={},
        status_lock=threading.Lock(),
        get_logger=lambda: FakeLogger(),
    )

    RobotControlNode._pair_state_cb(
        fake, '/robot3', SimpleNamespace(data='GATE_FAIL')
    )
    RobotControlNode._pair_state_cb(
        fake, '/robot8', SimpleNamespace(data='PAIRED')
    )

    assert fake.status['/robot3']['pair_state'] == 'GATE_FAIL'
    assert fake.status['/robot8']['pair_state'] == 'PAIRED'


def test_robot_control_ignores_other_robot_fleet_status():
    """로봇별 통신 노드가 다른 로봇의 FleetStatus를 무시한다."""
    fake = SimpleNamespace(
        namespace='/robot3', status={}, status_lock=threading.Lock()
    )
    robot3 = SimpleNamespace(
        robot_ns='/robot3', state='SHUTTLE', battery=0.31,
        x=1.0, y=2.0, yaw=0.1, mission_id='M003',
        paired_worker='W1', estop=False,
    )
    robot8 = SimpleNamespace(
        robot_ns='/robot8', state='FOLLOWING', battery=0.88,
        x=8.0, y=9.0, yaw=0.8, mission_id='M008',
        paired_worker='W2', estop=True,
    )

    with patch.object(
        robot_control_node.time, 'monotonic', return_value=10.0
    ):
        RobotControlNode._status_cb(fake, robot3)
        RobotControlNode._status_cb(fake, robot8)

    assert fake.status['/robot3']['state'] == 'SHUTTLE'
    assert fake.status['/robot3']['battery'] == 0.31
    assert fake.status['/robot3']['worker'] == 'W1'
    assert fake.status['/robot3']['mission'] == 'M003'
    assert fake.status['/robot3']['estop'] is False
    assert '/robot8' not in fake.status


def test_status_snapshot_reports_age_and_stale_per_robot():
    """웹 API가 로봇별 수신 경과 시간과 지연 여부를 계산하는지 확인한다."""
    fake = SimpleNamespace(
        status={
            '/robot3': {'state': 'SHUTTLE', '_received_monotonic': 98.0},
            '/robot8': {'state': 'FOLLOWING', '_received_monotonic': 95.0},
        },
        status_lock=threading.Lock(),
        status_stale_s=2.5,
    )

    with patch.object(
        robot_control_node.time, 'monotonic', return_value=100.0
    ):
        snapshot = RobotControlNode._status_snapshot(fake)

    assert snapshot['/robot3']['age_s'] == 2.0
    assert snapshot['/robot3']['stale'] is False
    assert snapshot['/robot8']['age_s'] == 5.0
    assert snapshot['/robot8']['stale'] is True
    assert '_received_monotonic' not in snapshot['/robot3']


def test_map_config_loads_relative_pgm_and_metadata(tmp_path):
    """Nav2 YAML 기준으로 실제 PGM 경로와 좌표 메타데이터를 읽는지 확인한다."""
    image_path = tmp_path / 'test_map.pgm'
    image_path.write_bytes(b'P5\n# test map\n3 2\n255\n' + bytes(range(6)))
    yaml_path = tmp_path / 'test_map.yaml'
    yaml_path.write_text(
        'image: test_map.pgm\n'
        'resolution: 0.05\n'
        'origin: [-3.1, -1.7, 0.0]\n'
        'negate: 0\n',
        encoding='utf-8',
    )

    config = load_map_config(str(yaml_path))

    assert config['image_path'] == str(image_path)
    assert config['width'] == 3
    assert config['height'] == 2
    assert config['resolution'] == 0.05
    assert config['origin'] == [-3.1, -1.7, 0.0]


def test_dock_landmarks_are_loaded_per_robot(tmp_path):
    """화면 대조용 도크 자세가 robot3와 robot8별로 분리되는지 확인한다."""
    yaml_path = tmp_path / 'arena.yaml'
    yaml_path.write_text(
        'arena:\n'
        '  dock_zone:\n'
        '    pose:\n'
        '      robot3: [0.019, -0.795, 0.133]\n'
        '      robot8: [-0.023, 0.034, -0.052]\n',
        encoding='utf-8',
    )

    landmarks = load_dock_landmarks(str(yaml_path))

    by_robot = {item['robot']: item for item in landmarks}
    assert by_robot['robot3']['y'] == -0.795
    assert by_robot['robot3']['yaw'] == 0.133
    assert by_robot['robot8']['y'] == 0.034
    assert by_robot['robot8']['yaw'] == -0.052


def test_accepted_manual_pair_records_one_intervention():
    """인지 노드가 승인한 명시 페어링만 개입으로 한 번 기록하는지 확인한다."""
    publisher = FakePublisher()
    fake = SimpleNamespace(
        namespace='/robot3',
        cli_manual_pair=object(),
        manual_pair_timeout_s=3.0,
        pub_int=publisher,
        get_clock=make_clock,
        _call=lambda _client, _request, timeout_sec: SimpleNamespace(
            accepted=True,
            reason='',
        ),
    )

    accepted, message = RobotControlNode._request_manual_pair(fake, 'W1')

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
        namespace='/robot8',
        cli_manual_pair=object(),
        manual_pair_timeout_s=3.0,
        pub_int=publisher,
        get_clock=make_clock,
        _call=lambda _client, _request, timeout_sec: SimpleNamespace(
            accepted=False,
            reason='NO_TARGET',
        ),
    )

    accepted, message = RobotControlNode._request_manual_pair(fake, 'W2')

    assert accepted is False
    assert message == 'NO_TARGET'
    assert publisher.messages == []


def test_missing_manual_pair_response_does_not_report_success():
    """서비스 응답이 없을 때 성공 또는 개입으로 처리하지 않는지 확인한다."""
    publisher = FakePublisher()
    fake = SimpleNamespace(
        namespace='/robot3',
        cli_manual_pair=object(),
        manual_pair_timeout_s=3.0,
        pub_int=publisher,
        get_clock=make_clock,
        _call=lambda _client, _request, timeout_sec: None,
    )

    accepted, message = RobotControlNode._request_manual_pair(fake, 'W1')

    assert accepted is False
    assert '응답 없음' in message
    assert publisher.messages == []
