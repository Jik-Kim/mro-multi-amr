"""관제 화면에 필요한 로봇 ROS 통신을 담당한다."""
import os
import threading
import time
from functools import partial

from ament_index_python.packages import get_package_share_directory

from mro_interfaces.msg import FleetStatus, Intervention
from mro_interfaces.srv import ManualPair, OperatorCommand, RequestMission

from rclpy.node import Node

from sensor_msgs.msg import BatteryState

from std_msgs.msg import String


DEFAULT_ROBOTS = ['/robot3', '/robot8']
PAIR_STATES = {'PAIRED', 'GATE_FAIL', 'NO_TARGET'}
APPROACH_STATES = {
    'IDLE',
    'SEARCHING',
    'APPROACH_1',
    'APPROACH_2',
    'ARRIVED',
    'STANDBY',
    'FOLLOWING',
    'FAILED',
}
APPROACH_STATE_ALIASES = {
    'APPROACH1': 'APPROACH_1',
    'APPROACH2': 'APPROACH_2',
}

# RobotControlNode 통신 계약.
#   수신: fleet_manager의 로봇 상태 + 로봇별 인지 페어링 상태
#   요청: 자재 요청·복귀·명시 페어링을 fleet_manager에 서비스로 전달
#   발행: 운영자 개입을 KPI·DB 기록용 토픽으로 전달
FLEET_STATUS_TOPIC = '/fleet/status'
PAIR_STATE_TOPIC = 'target/pair_state'
APPROACH_STATUS_TOPIC = 'approach/status'
BATTERY_STATE_TOPIC = 'battery_state'
INTERVENTION_TOPIC = '/fleet/intervention'
REQUEST_MISSION_SERVICE = '/fleet/request_mission'
OPERATOR_COMMAND_SERVICE = '/fleet/operator_command'
MANUAL_PAIR_SERVICE = '/fleet/manual_pair'


def _bringup_path(*parts):
    """통합 워크스페이스에 설치된 mro_bringup 자원 경로를 만든다."""
    try:
        share = get_package_share_directory('mro_bringup')
    except Exception:  # 설치 전에는 빈 경로로 UI를 계속 실행한다.
        return ''
    return os.path.join(share, *parts)


class RobotControlNode(Node):
    """한 대의 로봇 상태와 관제 명령을 ROS로 주고받는 노드.

    ROS executor와 Flask는 서로 다른 스레드에서 실행된다. 수신 콜백은
    ``status_lock``으로 공유 상태를 보호하고, 웹 파트에는 복사본만 넘긴다.
    """

    def __init__(self, namespace):
        """담당할 로봇 namespace로 노드와 통신을 구성한다."""
        node_suffix = namespace.strip('/').replace('/', '_')
        super().__init__(f'robot_control_{node_suffix}')
        if namespace not in DEFAULT_ROBOTS:
            raise ValueError(f'지원하지 않는 로봇 namespace: {namespace!r}')
        self.namespace = namespace
        self._load_parameters()
        self.status = {}
        self.status_lock = threading.Lock()
        self._create_ros_interfaces()

    def _load_parameters(self):
        """로봇 통신과 관제 화면에 필요한 실행 값을 읽는다."""
        self.declare_parameter('manual_pair_timeout_s', 3.0)
        self.declare_parameter('status_stale_s', 2.5)
        self.declare_parameter(
            'map_yaml_path', _bringup_path('maps', 'auto_map.yaml')
        )
        self.declare_parameter(
            'arena_yaml_path', _bringup_path('params', 'arena.yaml')
        )
        self.manual_pair_timeout_s = float(
            self.get_parameter('manual_pair_timeout_s').value
        )
        self.status_stale_s = float(self.get_parameter('status_stale_s').value)
        self.map_yaml_path = str(self.get_parameter('map_yaml_path').value)
        self.arena_yaml_path = str(self.get_parameter('arena_yaml_path').value)
        if self.status_stale_s <= 0.0:
            raise ValueError('status_stale_s must be positive')

    def _create_ros_interfaces(self):
        """로봇 통신에 사용하는 topic·service 연결을 만든다."""
        self.create_subscription(
            FleetStatus, FLEET_STATUS_TOPIC, self._status_cb, 10
        )
        self.cli_req = self.create_client(
            RequestMission, REQUEST_MISSION_SERVICE
        )
        self.cli_op = self.create_client(
            OperatorCommand, OPERATOR_COMMAND_SERVICE
        )
        self.cli_manual_pair = self.create_client(
            ManualPair, MANUAL_PAIR_SERVICE
        )
        self.pub_int = self.create_publisher(
            Intervention, INTERVENTION_TOPIC, 10
        )
        self.create_subscription(
            String,
            f'{self.namespace}/{APPROACH_STATUS_TOPIC}',
            partial(self._receive_robot_status, self.namespace),
            10,
        )
        self.create_subscription(
            String,
            f'{self.namespace}/{PAIR_STATE_TOPIC}',
            partial(self._pair_state_cb, self.namespace),
            10,
        )
        self.create_subscription(
            BatteryState,
            f'{self.namespace}/{BATTERY_STATE_TOPIC}',
            self._receive_battery_status,
            10,
        )

    def _status_cb(self, message):
        """FleetStatus를 웹 응답에 사용할 로봇별 상태로 변환한다."""
        if message.robot_ns != self.namespace:
            return
        status = {
            'state': message.state,
            'battery': message.battery,
            'x': message.x,
            'y': message.y,
            'yaw': message.yaw,
            'mission': message.mission_id,
            'worker': message.paired_worker,
            'estop': message.estop,
            '_received_monotonic': time.monotonic(),
        }
        with self.status_lock:
            previous = self.status.get(message.robot_ns, {})
            status['pair_state'] = previous.get('pair_state', 'NO_TARGET')
            status['approach_state'] = previous.get('approach_state', 'IDLE')
            self.status[message.robot_ns] = status

    def _receive_robot_status(self, namespace, message):
        """로봇의 접근·추종 상태를 검사해 네임스페이스별로 저장한다."""
        state = APPROACH_STATE_ALIASES.get(message.data, message.data)
        if state not in APPROACH_STATES:
            self.get_logger().warn(
                f'{namespace}: 알 수 없는 approach/status {message.data!r}'
            )
            return

        with self.status_lock:
            self.status.setdefault(namespace, {})['approach_state'] = state

    def _receive_battery_status(self, message):
        """담당 로봇의 배터리 잔량을 화면용 상태에 저장한다."""
        with self.status_lock:
            status = self.status.setdefault(self.namespace, {})
            status['battery'] = message.percentage
            status['_received_monotonic'] = time.monotonic()

    def _pair_state_cb(self, namespace, message):
        """인지 노드의 페어링 상태를 로봇별 UI 상태에 합친다."""
        if message.data not in PAIR_STATES:
            self.get_logger().warn(
                f'{namespace}: 알 수 없는 pair_state {message.data!r}'
            )
            return
        with self.status_lock:
            self.status.setdefault(namespace, {})['pair_state'] = message.data

    def _status_snapshot(self):
        """웹 UI에 로봇별 최신 상태와 수신 경과 시간을 넘긴다."""
        now = time.monotonic()
        with self.status_lock:
            snapshot = {}
            for namespace, values in self.status.items():
                copied = values.copy()
                received = copied.pop('_received_monotonic', None)
                if received is None:
                    copied['age_s'] = None
                    copied['stale'] = True
                else:
                    age_s = max(0.0, now - received)
                    copied['age_s'] = round(age_s, 3)
                    copied['stale'] = age_s > self.status_stale_s
                snapshot[namespace] = copied
            return snapshot

    def _call(self, client, service_request, timeout_sec=2.0):
        """Flask 스레드에서 ROS 서비스 결과를 유한 시간만 기다린다."""
        if not client.wait_for_service(timeout_sec=1.0):
            return None
        future = client.call_async(service_request)
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout_sec) or future.exception():
            return None
        return future.result()

    def _request_mission(self, worker_id, bay):
        """자재 요청을 fleet_manager에 전달한다."""
        service_request = RequestMission.Request()
        service_request.worker_id = worker_id
        service_request.bay = bay
        return self._call(self.cli_req, service_request)

    def _request_command(self, command):
        """작업 종료·복귀 명령을 fleet_manager에 전달한다."""
        service_request = OperatorCommand.Request()
        service_request.robot_ns = self.namespace
        service_request.command = command
        return self._call(self.cli_op, service_request)

    def _publish_intervention(self, kind, note=''):
        """운영자 개입을 ROS 시각과 함께 발행한다."""
        message = Intervention()
        message.header.stamp = self.get_clock().now().to_msg()
        message.robot_ns = self.namespace
        message.kind = kind
        message.note = note
        self.pub_int.publish(message)

    def _request_manual_pair(self, worker_id):
        """명시 페어링을 요청하고 성공한 개입만 기록한다."""
        service_request = ManualPair.Request()
        service_request.robot_ns = self.namespace
        service_request.worker_id = worker_id
        response = self._call(
            self.cli_manual_pair,
            service_request,
            timeout_sec=self.manual_pair_timeout_s,
        )
        if response is None:
            return False, 'fleet_manager 또는 인지 서비스 응답 없음'
        if not response.accepted:
            return False, response.reason or '인지 서비스 응답 없음'
        RobotControlNode._publish_intervention(
            self, 'MANUAL_PAIR', worker_id
        )
        return True, '명시 페어링 완료'
