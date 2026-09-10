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
from mro_fleet.nodes.MonitorWebApp import (
    MonitorWebApp, load_dock_landmarks, load_map_config,
)
from mro_fleet.nodes.fleet_client import FleetUICommands
from mro_fleet.nodes.robot_control_node import RobotStatus
from mro_fleet.nodes.robot_state_manager import RobotStateManager


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
    robot_state_manager = SimpleNamespace(
        robots={'/robot3': control},
        get_robot=lambda namespace: {'/robot3': control}.get(namespace),
    )
    monitor = MonitorWebApp(
        robot_state_manager, SimpleNamespace(), os.path.dirname(TEMPLATE_PATH)
    )

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


def make_robot_control_fake(namespace='/robot3'):
    """로봇 하나의 상태만 소유하는 RobotStatus 대역을 만든다."""
    fake = SimpleNamespace(
        namespace=namespace,
        status_lock=threading.Lock(),
        get_logger=lambda: FakeLogger(),
    )
    fake.status = RobotStatus._initial_status()
    fake._record_status_received_time = lambda: (
        RobotStatus._record_status_received_time(fake)
    )
    fake._update_status_value = lambda key, value: (
        RobotStatus._update_status_value(fake, key, value)
    )
    fake._return_has_priority = RobotStatus._return_has_priority
    fake.get_status = lambda: RobotStatus.get_status(fake)
    return fake


def test_pair_state_is_stored_by_robot_control_instance():
    """pair_state를 담당 RobotStatus의 상태에 저장한다."""
    fake = make_robot_control_fake()

    RobotStatus._receive_pair_state(
        fake, SimpleNamespace(data='GATE_FAIL')
    )

    assert fake.status['pair_state'] == 'GATE_FAIL'


def test_approach_status_is_normalized_and_stored():
    """approach/status 별칭을 계약 값으로 변환해 저장한다."""
    fake = make_robot_control_fake()

    RobotStatus._receive_approach_status(
        fake, SimpleNamespace(data='APPROACH1')
    )

    status = fake.status
    assert status['approach_state'] == 'APPROACH_1'
    assert status['state'] == 'APPROACH_1'


def test_invalid_approach_status_is_ignored():
    """계약에 없는 로봇 상태는 저장하지 않고 경고를 남긴다."""
    fake = make_robot_control_fake('/robot8')
    logger = FakeLogger()
    fake.get_logger = lambda: logger
    before = fake.status.copy()

    RobotStatus._receive_approach_status(
        fake, SimpleNamespace(data='UNKNOWN')
    )

    assert fake.status == before
    assert len(logger.warnings) == 1


def test_battery_status_is_stored_by_instance_namespace():
    """배터리 잔량이 인스턴스의 로봇 namespace에 저장된다."""
    fake = make_robot_control_fake()

    with patch.object(
        robot_control_node.time, 'monotonic', return_value=10.0
    ):
        RobotStatus._receive_battery_status(
            fake, SimpleNamespace(percentage=0.73)
        )

    assert fake.status['battery'] == 0.73
    assert fake.status['_received_monotonic'] == 10.0


def test_pair_states_are_owned_by_separate_instances():
    """로봇별 인스턴스의 페어링 상태가 서로 섞이지 않는다."""
    robot3 = make_robot_control_fake('/robot3')
    robot8 = make_robot_control_fake('/robot8')

    RobotStatus._receive_pair_state(
        robot3, SimpleNamespace(data='GATE_FAIL')
    )
    RobotStatus._receive_pair_state(
        robot8, SimpleNamespace(data='PAIRED')
    )

    assert robot3.status['pair_state'] == 'GATE_FAIL'
    assert robot8.status['pair_state'] == 'PAIRED'


def test_status_snapshot_contains_only_instance_robot():
    """웹에는 해당 RobotStatus가 소유한 로봇만 노출한다."""
    fake = make_robot_control_fake('/robot3')
    fake.status_overtime_sec = 5.0
    fake.status.update(state='SHUTTLE', _received_monotonic=96.0)

    with patch(
        'mro_fleet.nodes.robot_state_manager.time.monotonic',
        return_value=100.0,
    ):
        manager = SimpleNamespace(robots={'/robot3': fake})
        snapshot = RobotStateManager.get_all_robot_status(manager)

    assert list(snapshot) == ['/robot3']
    assert snapshot['/robot3']['age_s'] == 4.0
    assert snapshot['/robot3']['stale'] is False
    assert '_received_monotonic' not in snapshot['/robot3']


def test_return_state_has_priority_over_late_approach_status():
    """복귀 중에 뒤늦게 온 접근 상태가 대표 상태를 덮지 않는다."""
    fake = make_robot_control_fake()

    RobotStatus._receive_shuttle_status(
        fake, SimpleNamespace(data='RETURNING')
    )
    RobotStatus._receive_approach_status(
        fake, SimpleNamespace(data='FOLLOWING')
    )

    assert fake.status['state'] == 'RETURNING'


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
        manual_pair_client=object(),
        manual_pair_timeout_s=3.0,
        intervention_publisher=publisher,
        get_clock=make_clock,
        _call_service=lambda _client, _request, timeout_sec: SimpleNamespace(
            accepted=True,
            reason='',
        ),
    )
    fake.publish_intervention = lambda namespace, kind, note='': (
        FleetUICommands.publish_intervention(fake, namespace, kind, note)
    )

    accepted, message = FleetUICommands.request_manual_pair(
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
        manual_pair_client=object(),
        manual_pair_timeout_s=3.0,
        intervention_publisher=publisher,
        get_clock=make_clock,
        _call_service=lambda _client, _request, timeout_sec: SimpleNamespace(
            accepted=False,
            reason='NO_TARGET',
        ),
    )

    accepted, message = FleetUICommands.request_manual_pair(
        fake, '/robot8', 'W2'
    )

    assert accepted is False
    assert message == 'NO_TARGET'
    assert publisher.messages == []


def test_missing_manual_pair_response_does_not_report_success():
    """서비스 응답이 없을 때 성공 또는 개입으로 처리하지 않는지 확인한다."""
    publisher = FakePublisher()
    fake = SimpleNamespace(
        manual_pair_client=object(),
        manual_pair_timeout_s=3.0,
        intervention_publisher=publisher,
        get_clock=make_clock,
        _call_service=lambda _client, _request, timeout_sec: None,
    )

    accepted, message = FleetUICommands.request_manual_pair(
        fake, '/robot3', 'W1'
    )

    assert accepted is False
    assert '응답 없음' in message
    assert publisher.messages == []
