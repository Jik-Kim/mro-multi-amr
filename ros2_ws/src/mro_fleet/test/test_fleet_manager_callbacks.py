"""ROS 네트워크 없이 관제 콜백의 분기 규칙을 검증한다."""
from types import MethodType, SimpleNamespace

from mro_fleet.nodes.fleet_manager_node import FleetManager

from mro_interfaces.msg import Mission


class FakeDatabase:
    """콜백이 기록한 이벤트를 보관하는 테스트 대역."""

    def __init__(self):
        """빈 이벤트 목록을 준비한다."""
        self.events = []
        self.overdue = []

    def event(self, *values):
        """전달된 이벤트 값을 순서대로 보관한다."""
        self.events.append(values)

    def expire_overdue(self, now, t_lim_s):
        """타이머에 전달할 기한 초과 미션 목록을 반환한다."""
        return self.overdue


def make_status_manager(mission_state=Mission.FOLLOWING):
    """상태 콜백 검증에 필요한 최소 관제 객체를 만든다."""
    namespace = '/robot3'
    fake = SimpleNamespace(
        robot={
            namespace: {
                'approach_state': 'UNKNOWN',
                'shuttle_state': 'UNKNOWN',
                'status_owner': None,
                'mission_state': mission_state,
                'mission_id': 'M001',
            }
        },
        db=FakeDatabase(),
        _now_s=lambda: 30.0,
    )
    for method_name in (
        '_set_source_state',
        '_can_transition_from_status',
    ):
        setattr(
            fake,
            method_name,
            MethodType(getattr(FleetManager, method_name), fake),
        )
    fake._is_return_state = FleetManager._is_return_state
    fake._shuttle_return_has_priority = (
        FleetManager._shuttle_return_has_priority
    )
    fake._composed_state = FleetManager._composed_state

    def transition(namespace_arg, state):
        fake.robot[namespace_arg]['mission_state'] = state

    fake._transition_mission = transition
    return fake


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
        _now_s=lambda: 131.0,
        get_logger=lambda: logger,
    )

    FleetManager._expire_overdue_missions(fake)

    assert database.events == [
        (131.0, '/robot3', 'TLIM_EXCEEDED', 'M001')
    ]


def test_intervention_records_message_timestamp_and_fields():
    """개입 메시지의 발행 시각과 필드가 DB 이벤트로 전달되는지 확인한다."""
    fake = SimpleNamespace(db=FakeDatabase(), _now_s=lambda: 99.0)
    message = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=12, nanosec=500_000_000)
        ),
        robot_ns='/robot3',
        kind='TOW',
        note='주행 불가로 작업자가 이동',
    )

    FleetManager._intervention_cb(fake, message)

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
    fake = SimpleNamespace(db=FakeDatabase(), _now_s=lambda: 99.0)
    invalid_stamps = (
        SimpleNamespace(sec=0, nanosec=0),
        SimpleNamespace(sec=-1, nanosec=0),
        SimpleNamespace(sec=1, nanosec=1_000_000_000),
    )

    for stamp in invalid_stamps:
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=stamp),
            robot_ns='/robot4',
            kind='RESTART',
            note='노드 재시작',
        )
        FleetManager._intervention_cb(fake, message)

    assert fake.db.events == 3 * [
        (99.0, '/robot4', 'INTERVENTION', 'RESTART', '노드 재시작')
    ]


def test_safety_policy_is_recorded_without_filtering():
    """SAFETY_POLICY도 DB에는 원래 종류 그대로 기록하는지 확인한다."""
    fake = SimpleNamespace(db=FakeDatabase(), _now_s=lambda: 99.0)
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=20, nanosec=0)),
        robot_ns='/robot3',
        kind='SAFETY_POLICY',
        note='E-STOP 자동 정지',
    )

    FleetManager._intervention_cb(fake, message)

    assert fake.db.events == [
        (
            20.0,
            '/robot3',
            'INTERVENTION',
            'SAFETY_POLICY',
            'E-STOP 자동 정지',
        )
    ]


def test_docked_is_not_overwritten_by_periodic_following():
    """DOCKED 뒤 FOLLOWING 재발행이 화면과 미션 완료를 못 덮는지 확인한다."""
    fake = make_status_manager()

    FleetManager._status_cb(
        fake, '/robot3', 'shuttle', SimpleNamespace(data='RETURNING')
    )
    FleetManager._status_cb(
        fake, '/robot3', 'shuttle', SimpleNamespace(data='DOCKED')
    )
    FleetManager._status_cb(
        fake, '/robot3', 'approach', SimpleNamespace(data='FOLLOWING')
    )

    robot = fake.robot['/robot3']
    assert robot['mission_state'] == Mission.DONE
    assert FleetManager._composed_state(robot) == 'DOCKED'


def test_repeated_source_state_does_not_duplicate_event():
    """같은 출처의 동일 상태 재발행이 이벤트를 늘리지 않는지 확인한다."""
    fake = make_status_manager(mission_state=Mission.WAITING)
    message = SimpleNamespace(data='FOLLOWING')

    FleetManager._status_cb(fake, '/robot3', 'approach', message)
    FleetManager._status_cb(fake, '/robot3', 'approach', message)

    assert fake.db.events == [
        (30.0, '/robot3', 'STATE', 'FOLLOWING', 'approach')
    ]


def test_old_shuttle_state_does_not_regress_following_mission():
    """추종 중 늦게 온 WAITING이 미션 상태와 화면을 되돌리지 않는지 확인한다."""
    fake = make_status_manager()
    robot = fake.robot['/robot3']
    robot['approach_state'] = 'FOLLOWING'
    robot['status_owner'] = 'approach'

    FleetManager._status_cb(
        fake, '/robot3', 'shuttle', SimpleNamespace(data='WAITING')
    )

    assert robot['mission_state'] == Mission.FOLLOWING
    assert FleetManager._composed_state(robot) == 'FOLLOWING'


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
        FleetManager._status_cb(
            fake,
            '/robot3',
            source,
            SimpleNamespace(data=state),
        )

    robot = fake.robot['/robot3']
    assert robot['mission_state'] == Mission.DONE
    assert FleetManager._composed_state(robot) == 'DOCKED'


def test_transient_approach_failure_is_only_recorded_as_event():
    """접근 FAILED가 상태 이벤트만 남기고 미션을 실패시키지 않는지 확인한다."""
    fake = make_status_manager(mission_state=Mission.WAITING)

    FleetManager._status_cb(
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

    FleetManager._status_cb(
        fake,
        '/robot3',
        'shuttle',
        SimpleNamespace(data='SHUTTLE'),
    )
    FleetManager._status_cb(
        fake,
        '/robot3',
        'shuttle',
        SimpleNamespace(data='WAITING'),
    )

    assert fake.robot['/robot3']['mission_state'] == Mission.WAITING
