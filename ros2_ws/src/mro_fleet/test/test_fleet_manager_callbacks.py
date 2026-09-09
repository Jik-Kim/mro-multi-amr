"""ROS 네트워크 없이 관제 콜백의 분기 규칙을 검증한다."""
import asyncio
import threading
from types import MethodType, SimpleNamespace
from unittest.mock import patch

from mro_fleet.nodes.fleet_manager_node import FleetManager
from mro_fleet.nodes.robot_state_manager import RobotStateManager

from mro_interfaces.msg import Mission


def test_robot_state_manager_creates_one_robot_object_per_namespace():
    """Namespace를 키로 삼아 로봇 객체를 관리한다."""
    executor = SimpleNamespace(add_node=lambda _node: None)
    with patch(
        'mro_fleet.nodes.robot_state_manager.RobotStatus',
        side_effect=lambda namespace: SimpleNamespace(namespace=namespace),
    ):
        manager = RobotStateManager(['/robot3', '/robot8'], executor)

    assert list(manager.robots) == ['/robot3', '/robot8']
    assert manager.robots['/robot3'].namespace == '/robot3'
    assert manager.robots['/robot8'].namespace == '/robot8'


def test_robot_state_manager_adds_and_removes_one_robot_during_execution():
    """실행 중 로봇 한 대를 executor에 추가하고 제거한다."""
    added = []
    removed = []
    destroyed = []
    executor = SimpleNamespace(
        add_node=lambda node: added.append(node),
        remove_node=lambda node: removed.append(node),
    )
    with patch(
        'mro_fleet.nodes.robot_state_manager.RobotStatus',
        side_effect=lambda namespace: SimpleNamespace(
            namespace=namespace,
            destroy_node=lambda: destroyed.append(namespace),
        ),
    ):
        manager = RobotStateManager([], executor)
        assert manager.add_robot('/robot3') is True
        assert manager.add_robot('/robot3') is False
        assert manager.remove_robot('/robot3') is True
        assert manager.remove_robot('/robot3') is False

    assert [robot.namespace for robot in added] == ['/robot3']
    assert [robot.namespace for robot in removed] == ['/robot3']
    assert destroyed == ['/robot3']


def make_clock(seconds):
    """고정된 ROS 시각을 반환하는 테스트 대역을 만든다."""
    nanoseconds = int(seconds * 1_000_000_000)
    stamp = SimpleNamespace(
        sec=nanoseconds // 1_000_000_000,
        nanosec=nanoseconds % 1_000_000_000,
    )
    current_time = SimpleNamespace(
        nanoseconds=nanoseconds,
        to_msg=lambda: stamp,
    )
    return SimpleNamespace(now=lambda: current_time)


class FakeDatabase:
    """콜백이 기록한 이벤트를 보관하는 테스트 대역."""

    def __init__(self):
        """빈 이벤트 목록을 준비한다."""
        self.events = []
        self.overdue = []
        self.workers = []

    def event(self, *values):
        """전달된 이벤트 값을 순서대로 보관한다."""
        self.events.append(values)

    def expire_overdue(self, now, t_lim_s):
        """타이머에 전달할 기한 초과 미션 목록을 반환한다."""
        return self.overdue

    def upsert_worker(self, *values):
        """확정된 작업자 배정 값을 순서대로 보관한다."""
        self.workers.append(values)


class FakeRobot:
    """로봇 상태 조회용 RobotStatus 대역."""

    def __init__(self, namespace, status):
        """테스트할 namespace와 상태를 저장한다."""
        self.namespace = namespace
        self.status = status
        self.status_lock = threading.Lock()

    def get_status(self):
        """현재 상태를 복사해 반환한다."""
        return self.status.copy()


class FakeRobotStateManager:
    """가용 로봇 조회와 미션 배정을 테스트하는 대역."""

    def __init__(self, robot_states):
        """로봇별 테스트 상태와 RobotStatus 대역을 만든다."""
        self.robot_states = robot_states
        self.robots = {}
        self.last_received_status = {}
        for namespace, state in robot_states.items():
            status = {
                'battery': state.get('battery', 1.0),
                'status_received': state.get('status_received', True),
                'mission': state.get('mission_id', ''),
                'mission_state': state.get('mission_state'),
                'shuttle_state': state.get('shuttle_state', 'UNKNOWN'),
                'estop': state.get('estop', False),
            }
            self.robots[namespace] = FakeRobot(namespace, status)
            self.last_received_status[namespace] = {
                'approach_state': state.get('approach_state', 'UNKNOWN'),
                'shuttle_state': state.get('shuttle_state', 'UNKNOWN'),
                'battery': state.get('battery'),
                'estop': state.get('estop'),
                'paired_worker': state.get('paired_worker', ''),
            }

    def get_available_robots(self):
        """미션·E-stop이 없고 상태를 받은 로봇을 반환한다."""
        available = []
        for robot in self.robots.values():
            status = robot.get_status()
            if (
                status['status_received']
                and not status['mission']
                and not status['estop']
            ):
                available.append(robot)
        return available

    def assign_mission(self, namespace, mission_id, worker_id, mission_state):
        """배정 결과를 로봇 대역과 기존 검증 상태에 반영한다."""
        self.robots[namespace].status['mission'] = mission_id
        state = self.robot_states[namespace]
        state['mission_id'] = mission_id
        state['mission_state'] = mission_state
        state['target_worker'] = worker_id

    def get_robot(self, namespace):
        """namespace에 해당하는 로봇 대역을 반환한다."""
        return self.robots.get(namespace)

    def update_fleet_status(self, message):
        """실제 RobotStateManager의 FleetStatus 반영 로직을 사용한다."""
        return RobotStateManager.update_fleet_status(self, message)

    def update_source_state(self, namespace, source, state):
        """출처별 상태를 로봇 대역에 반영한다."""
        changed = RobotStateManager.update_source_state(
            self, namespace, source, state
        )
        if changed:
            self.robot_states[namespace][f'{source}_state'] = state
        return changed

    def is_mission_transition_allowed(self, namespace, source, new_state):
        """실제 RobotStateManager의 미션 전이 규칙을 사용한다."""
        return RobotStateManager.is_mission_transition_allowed(
            self, namespace, source, new_state
        )

    def update_mission_state(self, namespace, mission_state):
        """로봇 대역의 미션 상태를 변경한다."""
        self.robots[namespace].status['mission_state'] = mission_state

    def clear_mission(self, namespace):
        """로봇 대역의 미션 정보를 비운다."""
        self.robots[namespace].status['mission'] = ''


def make_status_manager(mission_state=Mission.FOLLOWING):
    """상태 콜백 검증에 필요한 최소 관제 객체를 만든다."""
    namespace = '/robot3'
    fake = SimpleNamespace(
        robot={
            namespace: {
                'approach_state': 'UNKNOWN',
                'shuttle_state': 'UNKNOWN',
                'mission_state': mission_state,
                'mission_id': 'M001',
            }
        },
        db=FakeDatabase(),
        get_clock=lambda: make_clock(30.0),
    )
    fake.robot_state_manager = FakeRobotStateManager(fake.robot)

    def transition(namespace_arg, state):
        fake.robot[namespace_arg]['mission_state'] = state
        fake.robot_state_manager.update_mission_state(namespace_arg, state)

    fake._update_mission_state = transition
    return fake


def test_fleet_status_callback_writes_changed_values_to_database():
    """수신 콜백이 반환되기 전 변경 이벤트를 DB에 넘긴다."""
    fake = make_status_manager(mission_state=Mission.WAITING)
    robot = fake.robot['/robot3']
    robot.update({
        'battery': 0.5,
        'estop': False,
        'x': 0.0,
        'y': 0.0,
        'yaw': 0.0,
        'pair_state': 'NO_TARGET',
        'paired_worker': '',
    })
    fake.get_logger = lambda: FakeLogger()
    for method_name in ('_process_robot_state_for_mission',):
        setattr(
            fake,
            method_name,
            MethodType(getattr(FleetManager, method_name), fake),
        )

    message = SimpleNamespace(
        robot_ns='/robot3',
        shuttle_state='WAITING',
        approach_state='SEARCHING',
        battery=0.75,
        estop=True,
        x=1.2,
        y=-0.4,
        yaw=0.3,
        pair_state='GATE_FAIL',
        paired_worker='',
    )

    FleetManager._receive_fleet_status(fake, message)

    assert fake.db.events == [
        (30.0, '/robot3', 'STATE', 'WAITING', 'shuttle'),
        (30.0, '/robot3', 'STATE', 'SEARCHING', 'approach'),
        (30.0, '/robot3', 'ESTOP', True),
    ]
    stored_status = fake.robot_state_manager.get_robot('/robot3').get_status()
    assert stored_status['battery'] == 0.75
    assert stored_status['x'] == 1.2
    assert stored_status['pair_state'] == 'GATE_FAIL'


class FakePublisher:
    """발행된 ROS 메시지를 보관하는 테스트 대역."""

    def __init__(self):
        """빈 메시지 목록을 준비한다."""
        self.messages = []

    def publish(self, message):
        """발행 요청된 메시지를 보관한다."""
        self.messages.append(message)


class FakeLogger:
    """관제 로그 메시지를 보관하는 테스트 대역."""

    def __init__(self):
        """로그 목록을 준비한다."""
        self.infos = []
        self.warnings = []
        self.errors = []

    def info(self, message):
        """정보 로그를 보관한다."""
        self.infos.append(message)

    def warn(self, message):
        """경고 로그를 보관한다."""
        self.warnings.append(message)

    def error(self, message):
        """오류 로그를 보관한다."""
        self.errors.append(message)


class ImmediateFuture:
    """등록 즉시 완료 콜백을 부르는 서비스 Future 대역."""

    def __init__(self, result):
        """반환할 서비스 응답을 저장한다."""
        self._result = result

    def add_done_callback(self, callback):
        """실제 서비스가 완료된 것처럼 콜백을 즉시 부른다."""
        callback(self)

    def result(self):
        """저장한 서비스 응답을 반환한다."""
        return self._result


class PendingFuture:
    """완료되지 않아 시간 초과를 재현하는 Future 대역."""

    def add_done_callback(self, _callback):
        """완료 콜백을 호출하지 않는다."""


class FakeTimeoutTimer:
    """필요하면 생성 즉시 시간 초과 콜백을 실행하는 타이머 대역."""

    def __init__(self, callback, fire):
        """콜백 실행 여부와 취소 상태를 저장한다."""
        self.cancelled = False
        if fire:
            callback()

    def cancel(self):
        """타이머가 취소됐음을 기록한다."""
        self.cancelled = True


class FakeManualPairClient:
    """전달된 명시 페어링 요청을 보관하는 클라이언트 대역."""

    def __init__(self, result=None, ready=True, pending=False):
        """준비 상태와 반환할 응답을 저장한다."""
        self.ready = ready
        self.result = result
        self.pending = pending
        self.requests = []

    def service_is_ready(self):
        """설정한 서비스 준비 상태를 반환한다."""
        return self.ready

    def call_async(self, request):
        """요청을 보관하고 완료 또는 대기 Future를 반환한다."""
        self.requests.append(request)
        if self.pending:
            return PendingFuture()
        return ImmediateFuture(self.result)


def make_worker_manager(batteries=None):
    """작업자 등록·AMR 배정 검증용 최소 관제 객체를 만든다."""
    robots = ['/robot3', '/robot8']
    database = FakeDatabase()
    logger = FakeLogger()
    fake = SimpleNamespace(
        robot_namespaces=robots,
        robot={
            namespace: {
                'battery': (batteries or {}).get(namespace, 1.0),
                'status_received': True,
                'mission_id': '',
                'estop': False,
            }
            for namespace in robots
        },
        registered_workers=set(),
        db=database,
        get_clock=lambda: make_clock(15.0),
        get_logger=lambda: logger,
    )

    fake.robot_state_manager = FakeRobotStateManager(fake.robot)
    return fake, database, logger


def worker_descriptor(worker_id):
    """배정 테스트용 작업자 서술자를 만든다."""
    outfit_class = {
        'W1': 'worker_a',
        'W2': 'worker_b',
    }[worker_id]
    return SimpleNamespace(
        worker_id=worker_id,
        outfit_class=outfit_class,
    )


def register_workers(worker_ids):
    """지정 순서로 작업자를 등록하고 등록 결과를 반환한다."""
    fake, database, _logger = make_worker_manager()
    for worker_id in worker_ids:
        FleetManager._register_worker(fake, worker_descriptor(worker_id))
    return fake.registered_workers, database


def test_worker_registration_does_not_assign_robot():
    """작업자 등록은 AMR을 미리 배정하지 않고 DB에만 남긴다."""
    workers, database = register_workers(['W1'])

    assert workers == {'W1'}
    assert database.workers == [('W1', 'worker_a', 15.0)]


def test_repeated_worker_descriptor_is_stored_once():
    """1 Hz로 재발행된 같은 작업자를 중복 저장하지 않는다."""
    workers, database = register_workers(['W1', 'W1'])

    assert workers == {'W1'}
    assert database.workers == [('W1', 'worker_a', 15.0)]


def test_request_rejects_when_all_available_robots_have_low_battery():
    """등록된 작업자 요청이어도 가용 AMR 배터리가 낮으면 거부한다."""
    fake, database, _logger = make_worker_manager(
        batteries={'/robot3': 0.24, '/robot8': 0.20}
    )
    fake.registered_workers.add('W1')
    fake.battery_min = 0.25
    request = SimpleNamespace(worker_id='W1', bay='A')
    response = SimpleNamespace(accepted=True, mission_id='', message='')

    result = FleetManager._handle_mission_request(fake, request, response)

    assert result.accepted is False
    assert '배터리 임계 미만' in result.message
    assert database.events == [
        (15.0, '/robot3', 'ASSIGN_REFUSED_BATTERY', 0.24)
    ]


def test_request_assigns_first_available_robot_and_stores_mission():
    """자재 요청 시점에 가용 AMR을 골라 인스턴스와 DB에 반영한다."""
    database = FakeDatabase()
    database.states = []
    database.create_mission = lambda *_args: 'M001'
    database.set_state = lambda *args: database.states.append(args)
    robot3 = {
        'battery': 0.9, 'status_received': True,
        'mission_id': 'M000', 'mission_state': Mission.SHUTTLE,
        'estop': False, 'target_worker': 'W0',
        'approach_state': 'UNKNOWN', 'shuttle_state': 'SHUTTLE',
    }
    robot8 = {
        'battery': 0.8, 'status_received': True,
        'mission_id': '', 'mission_state': None,
        'estop': False, 'target_worker': '',
        'approach_state': 'UNKNOWN', 'shuttle_state': 'READY',
    }
    fake = SimpleNamespace(
        robot_namespaces=['/robot3', '/robot8'],
        registered_workers={'W1'},
        robot={'/robot3': robot3, '/robot8': robot8},
        battery_min=0.25,
        db=database,
        pub_mission=FakePublisher(),
        pub_assigned={
            '/robot3': FakePublisher(), '/robot8': FakePublisher()
        },
        get_clock=lambda: make_clock(20.0),
    )
    fake.robot_state_manager = FakeRobotStateManager(fake.robot)
    request = SimpleNamespace(worker_id='W1', bay='B')
    response = SimpleNamespace(accepted=False, mission_id='', message='')

    result = FleetManager._handle_mission_request(fake, request, response)

    assert result.accepted is True
    assert robot8['mission_id'] == 'M001'
    assert robot8['target_worker'] == 'W1'
    assert database.states == [('M001', Mission.ASSIGNED, 20.0)]
    assert len(fake.pub_assigned['/robot8'].messages) == 1
    assert fake.pub_assigned['/robot3'].messages == []


def test_return_all_accepts_empty_robot_namespace():
    """RETURN_ALL이 빈 robot_ns여도 두 로봇에 발행되는지 확인한다."""
    fake = SimpleNamespace(
        robot_namespaces=['/robot3', '/robot8'],
        robot={'/robot3': {}, '/robot8': {}},
        db=FakeDatabase(),
        pub_finish={
            '/robot3': FakePublisher(),
            '/robot8': FakePublisher(),
        },
        get_clock=lambda: make_clock(10.0),
    )
    request = SimpleNamespace(robot_ns='', command='RETURN_ALL')
    response = SimpleNamespace(ok=False, message='')

    result = FleetManager.return_immediately(fake, request, response)

    assert result.ok is True
    assert len(fake.pub_finish['/robot3'].messages) == 1
    assert len(fake.pub_finish['/robot8'].messages) == 1
    assert all(event[2] == 'OPERATOR' for event in fake.db.events)


def test_manual_pair_is_forwarded_to_selected_robot():
    """UI 요청과 인지 응답이 선택한 로봇 서비스로 전달되는지 확인한다."""
    robot3_client = FakeManualPairClient(
        SimpleNamespace(accepted=True, reason='')
    )
    robot8_client = FakeManualPairClient()
    fake = SimpleNamespace(
        manual_pair_clients={
            '/robot3': robot3_client,
            '/robot8': robot8_client,
        },
        manual_pair_timeout_s=0.01,
        get_logger=lambda: FakeLogger(),
        create_timer=lambda _period, callback, callback_group: (
            FakeTimeoutTimer(callback, False)
        ),
        destroy_timer=lambda _timer: True,
        manual_pair_client_group=object(),
        _complete_manual_pair_wait_once=(
            FleetManager._complete_manual_pair_wait_once
        ),
    )
    request = SimpleNamespace(robot_ns='/robot3', worker_id='W1')
    response = SimpleNamespace(accepted=False, reason='')

    result = asyncio.run(
        FleetManager._forward_manual_pair_request(fake, request, response)
    )

    assert result.accepted is True
    assert result.reason == ''
    assert len(robot3_client.requests) == 1
    assert robot3_client.requests[0].robot_ns == '/robot3'
    assert robot3_client.requests[0].worker_id == 'W1'
    assert robot8_client.requests == []


def test_manual_pair_rejects_unknown_robot_without_forwarding():
    """계약에 없는 로봇 이름은 하위 서비스를 부르지 않고 거부하는지 확인한다."""
    robot3_client = FakeManualPairClient()
    fake = SimpleNamespace(
        manual_pair_clients={'/robot3': robot3_client},
        manual_pair_timeout_s=0.01,
        get_logger=lambda: FakeLogger(),
        _complete_manual_pair_wait_once=(
            FleetManager._complete_manual_pair_wait_once
        ),
    )
    request = SimpleNamespace(robot_ns='/robot5', worker_id='W1')
    response = SimpleNamespace(accepted=True, reason='')

    result = asyncio.run(
        FleetManager._forward_manual_pair_request(fake, request, response)
    )

    assert result.accepted is False
    assert result.reason == 'WRONG_ROBOT'
    assert robot3_client.requests == []


def test_manual_pair_timeout_does_not_report_success():
    """인지 서비스 응답이 없을 때 관제가 성공으로 답하지 않는지 확인한다."""
    client = FakeManualPairClient(pending=True)
    fake = SimpleNamespace(
        manual_pair_clients={'/robot3': client},
        manual_pair_timeout_s=0.001,
        get_logger=lambda: FakeLogger(),
        create_timer=lambda _period, callback, callback_group: (
            FakeTimeoutTimer(callback, True)
        ),
        destroy_timer=lambda _timer: True,
        manual_pair_client_group=object(),
        _complete_manual_pair_wait_once=(
            FleetManager._complete_manual_pair_wait_once
        ),
    )
    request = SimpleNamespace(robot_ns='/robot3', worker_id='W1')
    response = SimpleNamespace(accepted=True, reason='old')

    result = asyncio.run(
        FleetManager._forward_manual_pair_request(fake, request, response)
    )

    assert result.accepted is False
    assert result.reason == ''


def test_request_rejects_when_robot_status_has_not_arrived():
    """상태를 한 번도 받지 못한 AMR은 가용 로봇에서 제외한다."""
    fake = SimpleNamespace(
        robot_namespaces=['/robot3'],
        registered_workers={'W1'},
        robot={
            '/robot3': {
                'battery': 1.0,
                'status_received': False,
                'mission_id': '',
                'estop': False,
            }
        },
        battery_min=0.25,
        db=FakeDatabase(),
        get_clock=lambda: make_clock(20.0),
    )
    fake.robot_state_manager = FakeRobotStateManager(fake.robot)
    request = SimpleNamespace(worker_id='W1', bay='A')
    response = SimpleNamespace(
        accepted=True,
        mission_id='',
        message='',
    )

    result = FleetManager._handle_mission_request(fake, request, response)

    assert result.accepted is False
    assert result.message == '가용 AMR 없음'


def test_overdue_timer_records_failure_without_stopping_robot():
    """T_lim 타이머가 초과 이벤트만 남기고 주행 명령은 건드리지 않는다."""
    database = FakeDatabase()
    database.overdue = [{
        'mission_id': 'M001',
        'robot_ns': '/robot3',
        'created_at': 10.0,
    }]
    logger = SimpleNamespace(warn=lambda _: None)
    fake = SimpleNamespace(
        db=database,
        t_lim_s=120.0,
        get_clock=lambda: make_clock(131.0),
        get_logger=lambda: logger,
    )

    FleetManager._mark_overdue_missions(fake)

    assert database.events == [
        (131.0, '/robot3', 'TLIM_EXCEEDED', 'M001')
    ]


def test_intervention_records_message_timestamp_and_fields():
    """개입 메시지의 발행 시각과 필드가 DB 이벤트로 전달되는지 확인한다."""
    fake = SimpleNamespace(db=FakeDatabase())
    message = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=12, nanosec=500_000_000)
        ),
        robot_ns='/robot3',
        kind='TOW',
        note='주행 불가로 작업자가 이동',
    )

    FleetManager._record_intervention_event(fake, message)

    assert fake.db.events == [
        (
            12.5,
            '/robot3',
            'INTERVENTION',
            'TOW',
            '주행 불가로 작업자가 이동',
        )
    ]


def test_intervention_uses_receive_time_for_invalid_stamp():
    """발행 시각이 0이거나 비정상이면 수신 시각을 사용하는지 확인한다."""
    fake = SimpleNamespace(
        db=FakeDatabase(), get_clock=lambda: make_clock(99.0)
    )
    invalid_stamps = (
        SimpleNamespace(sec=0, nanosec=0),
        SimpleNamespace(sec=-1, nanosec=0),
        SimpleNamespace(sec=1, nanosec=1_000_000_000),
    )

    for stamp in invalid_stamps:
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=stamp),
            robot_ns='/robot8',
            kind='RESTART',
            note='노드 재시작',
        )
        FleetManager._record_intervention_event(fake, message)

    assert fake.db.events == 3 * [
        (99.0, '/robot8', 'INTERVENTION', 'RESTART', '노드 재시작')
    ]


def test_safety_policy_is_recorded_without_filtering():
    """SAFETY_POLICY도 DB에는 원래 종류 그대로 기록하는지 확인한다."""
    fake = SimpleNamespace(db=FakeDatabase())
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=20, nanosec=0)),
        robot_ns='/robot3',
        kind='SAFETY_POLICY',
        note='E-STOP 자동 정지',
    )

    FleetManager._record_intervention_event(fake, message)

    assert fake.db.events == [
        (
            20.0,
            '/robot3',
            'INTERVENTION',
            'SAFETY_POLICY',
            'E-STOP 자동 정지',
        )
    ]


def test_zone_events_are_recorded_with_contract_fields():
    """구역 이벤트 4종이 계약의 DB 필드로 기록되는지 확인한다."""
    database = FakeDatabase()
    fake = SimpleNamespace(
        robot={'/robot3': {}, '/robot8': {}},
        db=database,
        get_clock=lambda: make_clock(42.5),
        get_logger=lambda: FakeLogger(),
    )
    fake.robot_state_manager = FakeRobotStateManager(fake.robot)
    messages = (
        'P1 /robot3 REJECT',
        'P1 /robot3 GRANT',
        'DOCK /robot8 RELEASE',
        'DOCK /robot8 RECLAIM',
    )

    for data in messages:
        FleetManager._record_zone_event(fake, SimpleNamespace(data=data))

    assert database.events == [
        (42.5, '/robot3', 'ZONE', 'REJECT', 'P1'),
        (42.5, '/robot3', 'ZONE', 'GRANT', 'P1'),
        (42.5, '/robot8', 'ZONE', 'RELEASE', 'DOCK'),
        (42.5, '/robot8', 'ZONE', 'RECLAIM', 'DOCK'),
    ]


def test_invalid_zone_events_are_not_recorded():
    """형식·구역·로봇·동작이 계약과 다르면 DB에 기록하지 않는다."""
    database = FakeDatabase()
    logger = FakeLogger()
    fake = SimpleNamespace(
        robot={'/robot3': {}, '/robot8': {}},
        db=database,
        get_clock=lambda: make_clock(42.5),
        get_logger=lambda: logger,
    )
    fake.robot_state_manager = FakeRobotStateManager(fake.robot)
    invalid_messages = (
        'P1 /robot3',
        'P2 /robot3 GRANT',
        'P1 /robot5 GRANT',
        'P1 /robot3 WAIT',
    )

    for data in invalid_messages:
        FleetManager._record_zone_event(fake, SimpleNamespace(data=data))

    assert database.events == []
    assert len(logger.warnings) == len(invalid_messages)


def test_docked_is_not_overwritten_by_periodic_following():
    """DOCKED 뒤 FOLLOWING 재발행이 화면과 미션 완료를 못 덮는지 확인한다."""
    fake = make_status_manager()

    FleetManager._process_robot_state_for_mission(
        fake, '/robot3', 'shuttle', SimpleNamespace(data='RETURNING')
    )
    FleetManager._process_robot_state_for_mission(
        fake, '/robot3', 'shuttle', SimpleNamespace(data='DOCKED')
    )
    FleetManager._process_robot_state_for_mission(
        fake, '/robot3', 'approach', SimpleNamespace(data='FOLLOWING')
    )

    robot = fake.robot['/robot3']
    assert robot['mission_state'] == Mission.DONE
    assert robot['shuttle_state'] == 'DOCKED'


def test_repeated_source_state_does_not_duplicate_event():
    """같은 출처의 동일 상태 재발행이 이벤트를 늘리지 않는지 확인한다."""
    fake = make_status_manager(mission_state=Mission.WAITING)
    message = SimpleNamespace(data='FOLLOWING')

    FleetManager._process_robot_state_for_mission(
        fake, '/robot3', 'approach', message
    )
    FleetManager._process_robot_state_for_mission(
        fake, '/robot3', 'approach', message
    )

    assert fake.db.events == [
        (30.0, '/robot3', 'STATE', 'FOLLOWING', 'approach')
    ]


def test_old_shuttle_state_does_not_regress_following_mission():
    """추종 중 늦게 온 WAITING이 미션 상태와 화면을 되돌리지 않는지 확인한다."""
    fake = make_status_manager()
    robot = fake.robot['/robot3']
    robot['approach_state'] = 'FOLLOWING'

    FleetManager._process_robot_state_for_mission(
        fake, '/robot3', 'shuttle', SimpleNamespace(data='WAITING')
    )

    assert robot['mission_state'] == Mission.FOLLOWING
    assert robot['approach_state'] == 'FOLLOWING'


def test_transient_approach_failure_recovers_and_reaches_done():
    """접근 시도 실패 후 복구하면 미션이 최종 DONE에 도달하는지 확인한다."""
    fake = make_status_manager(mission_state=Mission.WAITING)
    states = [
        ('approach', 'SEARCHING'),
        ('approach', 'FAILED'),
        ('approach', 'SEARCHING'),
        ('approach', 'FOLLOWING'),
        ('shuttle', 'RETURNING'),
        ('shuttle', 'DOCKED'),
    ]

    for source, state in states:
        FleetManager._process_robot_state_for_mission(
            fake,
            '/robot3',
            source,
            SimpleNamespace(data=state),
        )

    robot = fake.robot['/robot3']
    assert robot['mission_state'] == Mission.DONE
    assert robot['shuttle_state'] == 'DOCKED'


def test_transient_approach_failure_is_only_recorded_as_event():
    """접근 FAILED가 상태 이벤트만 남기고 미션을 실패시키지 않는지 확인한다."""
    fake = make_status_manager(mission_state=Mission.WAITING)

    FleetManager._process_robot_state_for_mission(
        fake,
        '/robot3',
        'approach',
        SimpleNamespace(data='FAILED'),
    )

    assert fake.robot['/robot3']['mission_state'] == Mission.WAITING
    assert fake.db.events == [
        (30.0, '/robot3', 'STATE', 'FAILED', 'approach')
    ]


def test_failed_mission_state_can_receive_later_shuttle_progress():
    """FAILED 뒤 정상 셔틀 상태가 오면 미션 상태 전이가 재개되는지 확인한다."""
    fake = make_status_manager(mission_state=Mission.FAILED)

    FleetManager._process_robot_state_for_mission(
        fake,
        '/robot3',
        'shuttle',
        SimpleNamespace(data='SHUTTLE'),
    )
    FleetManager._process_robot_state_for_mission(
        fake,
        '/robot3',
        'shuttle',
        SimpleNamespace(data='WAITING'),
    )

    assert fake.robot['/robot3']['mission_state'] == Mission.WAITING
