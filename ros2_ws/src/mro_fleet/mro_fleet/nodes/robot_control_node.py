"""로봇별 ROS 통신과 `/fleet/status` 상태 집계를 담당한다."""
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from mro_interfaces.msg import FleetStatus, Intervention, Mission
from mro_interfaces.srv import ManualPair, OperatorCommand, RequestMission
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String


DEFAULT_ROBOTS = ['/robot3', '/robot8']
PAIR_STATES = {'PAIRED', 'GATE_FAIL', 'NO_TARGET'}
APPROACH_STATES = {
    'IDLE', 'SEARCHING', 'APPROACH_1', 'APPROACH_2',
    'ARRIVED', 'STANDBY', 'FOLLOWING', 'FAILED',
}
SHUTTLE_STATES = {
    'READY', 'SHUTTLE', 'AT_BAY', 'WAITING', 'RETURNING',
    'DOCK_WAIT', 'DOCKING', 'DOCKED', 'FAILED',
}
APPROACH_STATE_ALIASES = {
    'APPROACH1': 'APPROACH_1',
    'APPROACH2': 'APPROACH_2',
}

FLEET_STATUS_TOPIC = '/fleet/status'
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
    """하나의 namespace에 속한 AMR 상태와 관제 명령을 담당한다."""

    def __init__(self, namespace):
        """담당할 로봇 namespace로 노드와 통신을 구성한다."""
        node_suffix = namespace.strip('/').replace('/', '_')
        super().__init__(f'robot_control_{node_suffix}')
        if namespace not in DEFAULT_ROBOTS:
            raise ValueError(f'지원하지 않는 로봇 namespace: {namespace!r}')
        self.namespace = namespace
        self._load_parameters()
        self.status_lock = threading.Lock()
        self.status = {namespace: self._initial_status()}
        self._create_ros_interfaces()

    @staticmethod
    def _initial_status():
        """아직 토픽을 받지 않은 AMR의 기본 상태를 만든다."""
        return {
            'state': 'IDLE',
            'approach_state': 'IDLE',
            'shuttle_state': 'READY',
            'pair_state': 'NO_TARGET',
            'battery': 0.0,
            'x': 0.0,
            'y': 0.0,
            'yaw': 0.0,
            'mission': '',
            'worker': '',
            'estop': False,
        }

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
        self.status_stale_s = float(
            self.get_parameter('status_stale_s').value
        )
        self.map_yaml_path = str(self.get_parameter('map_yaml_path').value)
        self.arena_yaml_path = str(
            self.get_parameter('arena_yaml_path').value
        )
        if self.status_stale_s <= 0.0:
            raise ValueError('status_stale_s must be positive')

    def _create_ros_interfaces(self):
        """담당 AMR 상태 토픽과 관제 요청 인터페이스를 만든다."""
        subscriptions = (
            (String, 'approach/status', self._receive_approach_status),
            (String, 'shuttle/status', self._receive_shuttle_status),
            (BatteryState, 'battery_state', self._receive_battery_status),
            (PoseWithCovarianceStamped, 'amcl_pose', self._receive_pose),
            (Bool, 'safety/estop', self._receive_estop),
            (Bool, 'target/paired', self._receive_paired),
            (String, 'target/worker_id', self._receive_worker_id),
            (String, 'target/pair_state', self._receive_pair_state),
            (Mission, 'mission/assigned', self._receive_mission),
        )
        for message_type, topic, callback in subscriptions:
            self.create_subscription(
                message_type, f'{self.namespace}/{topic}', callback, 10
            )
        self.pub_status = self.create_publisher(
            FleetStatus, FLEET_STATUS_TOPIC, 10
        )
        self.pub_int = self.create_publisher(
            Intervention, INTERVENTION_TOPIC, 10
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
        self.create_timer(1.0, self._publish_status)

    def _touch(self, status):
        """마지막 AMR 상태 수신 시각을 갱신한다."""
        status['_received_monotonic'] = time.monotonic()

    @staticmethod
    def _return_has_priority(status):
        """복귀·도킹 상태가 접근 상태보다 우선인지 판별한다."""
        return status['shuttle_state'] in {
            'RETURNING', 'DOCK_WAIT', 'DOCKING', 'DOCKED', 'FAILED'
        }

    def _receive_approach_status(self, message):
        """접근·추종 상태를 검사해 담당 AMR 상태에 저장한다."""
        state = APPROACH_STATE_ALIASES.get(message.data, message.data)
        if state not in APPROACH_STATES:
            self.get_logger().warn(
                f'{self.namespace}: 알 수 없는 approach/status '
                f'{message.data!r}'
            )
            return
        with self.status_lock:
            status = self.status[self.namespace]
            status['approach_state'] = state
            if state != 'IDLE' and not self._return_has_priority(status):
                status['state'] = state
            self._touch(status)

    def _receive_shuttle_status(self, message):
        """셔틀·복귀 상태를 검사해 담당 AMR 상태에 저장한다."""
        if message.data not in SHUTTLE_STATES:
            self.get_logger().warn(
                f'{self.namespace}: 알 수 없는 shuttle/status '
                f'{message.data!r}'
            )
            return
        with self.status_lock:
            status = self.status[self.namespace]
            status['shuttle_state'] = message.data
            if message.data != 'READY' or status['approach_state'] == 'IDLE':
                status['state'] = message.data
            self._touch(status)

    def _receive_battery_status(self, message):
        """담당 AMR의 배터리 잔량을 저장한다."""
        self._set_status_value('battery', message.percentage)

    def _receive_pose(self, message):
        """담당 AMR의 map 위치와 yaw를 저장한다."""
        pose = message.pose.pose
        quaternion = pose.orientation
        yaw = math.atan2(
            2.0 * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0 - 2.0 * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )
        with self.status_lock:
            status = self.status[self.namespace]
            status.update(x=pose.position.x, y=pose.position.y, yaw=yaw)
            self._touch(status)

    def _receive_estop(self, message):
        """담당 AMR의 안전 정지 여부를 저장한다."""
        self._set_status_value('estop', message.data)

    def _receive_worker_id(self, message):
        """담당 AMR이 인식한 작업자 ID를 저장한다."""
        self._set_status_value('worker', message.data)

    def _receive_paired(self, message):
        """페어링이 해제되면 작업자 ID를 비운다."""
        if not message.data:
            self._set_status_value('worker', '')

    def _receive_pair_state(self, message):
        """인지 노드의 페어링 판정 상태를 저장한다."""
        if message.data not in PAIR_STATES:
            self.get_logger().warn(
                f'{self.namespace}: 알 수 없는 pair_state '
                f'{message.data!r}'
            )
            return
        self._set_status_value('pair_state', message.data)

    def _receive_mission(self, message):
        """담당 AMR에 배정된 미션 ID를 저장한다."""
        self._set_status_value('mission', message.mission_id)

    def _set_status_value(self, key, value):
        """담당 AMR의 단일 상태 값과 수신 시각을 갱신한다."""
        with self.status_lock:
            status = self.status[self.namespace]
            status[key] = value
            self._touch(status)

    def _publish_status(self):
        """담당 AMR의 최신 상태를 FleetStatus로 1 Hz 발행한다."""
        with self.status_lock:
            status = self.status[self.namespace].copy()
        message = FleetStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.robot_ns = self.namespace
        message.state = status['state']
        message.approach_state = status['approach_state']
        message.shuttle_state = status['shuttle_state']
        message.pair_state = status['pair_state']
        message.battery = float(status['battery'])
        message.x = float(status['x'])
        message.y = float(status['y'])
        message.yaw = float(status['yaw'])
        message.mission_id = status['mission']
        message.paired_worker = status['worker']
        message.estop = bool(status['estop'])
        self.pub_status.publish(message)

    def _status_snapshot(self):
        """웹 UI에 담당 AMR의 최신 상태와 수신 경과 시간을 넘긴다."""
        now = time.monotonic()
        with self.status_lock:
            copied = self.status[self.namespace].copy()
        received = copied.pop('_received_monotonic', None)
        if received is None:
            copied['age_s'] = None
            copied['stale'] = True
        else:
            age_s = max(0.0, now - received)
            copied['age_s'] = round(age_s, 3)
            copied['stale'] = age_s > self.status_stale_s
        return {self.namespace: copied}

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
        self._publish_intervention('MANUAL_PAIR', worker_id)
        return True, '명시 페어링 완료'
