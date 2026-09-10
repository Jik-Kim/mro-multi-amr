"""로봇별 ROS 통신과 `/fleet/status` 상태 집계를 담당한다."""
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseWithCovarianceStamped

from mro_interfaces.msg import FleetStatus, Mission

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)

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

# 상태 칸마다 어느 토픽에서 오는지 (I-055). 배터리 하나만 들어와도 전체가
#   신선해 보이던 구멍을 막는다.
#
# ★ 판정 기준은 「값이 오래됐나」가 아니라 **「소스가 아직 살아 있나」** 다.
#   이 토픽들 중 여럿이 **바뀔 때만** 발행한다 — `safety/estop` 은 estop 이
#   걸리거나 풀릴 때만 내고(`safety_watchdog_node.scan_cb`), `amcl_pose` 는
#   로봇이 서면 발행을 멈춘다. 나이로 재면 **멀쩡한 로봇이 계속 「확인 불가」**
#   가 되어 화면이 더 못 쓰게 된다. 그래서 발행자 수로 본다 — 0 이면 소스가
#   죽은 것이고, 그때만 마지막 값이 현재값이 아니다.
STATUS_SOURCES = {
    'pose': {'fields': ('x', 'y', 'yaw'), 'topic': 'amcl_pose'},
    'estop': {'fields': ('estop',), 'topic': 'safety/estop'},
    'battery': {'fields': ('battery',), 'topic': 'battery_state'},
    'shuttle': {'fields': ('shuttle_state',), 'topic': 'shuttle/status'},
    'approach': {'fields': ('approach_state',), 'topic': 'approach/status'},
    'pair': {'fields': ('pair_state',), 'topic': 'target/pair_state'},
    'worker': {'fields': ('worker',), 'topic': 'target/worker_id'},
    'mission': {'fields': ('mission',), 'topic': 'mission/assigned'},
}


def _bringup_path(*parts):
    """통합 워크스페이스에 설치된 mro_bringup 자원 경로를 만든다."""
    try:
        share = get_package_share_directory('mro_bringup')
    except Exception:  # 설치 전에는 빈 경로로 UI를 계속 실행한다.
        return ''
    return os.path.join(share, *parts)


class RobotStatus(Node):
    """하나의 namespace에 속한 AMR 상태 통신을 담당한다."""

    def __init__(self, namespace):
        """담당할 로봇 namespace로 노드와 통신을 구성한다."""
        node_suffix = namespace.strip('/').replace('/', '_')
        super().__init__(f'robot_control_{node_suffix}')
        if namespace not in DEFAULT_ROBOTS:
            raise ValueError(f'지원하지 않는 로봇 namespace: {namespace!r}')
        self.namespace = namespace
        self._load_parameters()
        self.status_lock = threading.Lock()
        self.status = self._initial_status()
        # 소스 토픽별 마지막 수신 시각. 비어 있으면 「한 번도 안 받았다」다 (I-055).
        self.source_times = {}
        # 소스 토픽별 발행자 수. 0 이면 소스가 죽은 것이다 (I-055).
        #   Flask 스레드에서 rmw 를 부르지 않도록 노드 타이머로만 갱신한다.
        self.source_publishers = {}
        self.create_subscription(
            String,
            f'{self.namespace}/approach/status',
            self._receive_approach_status,
            10,
        )
        self.create_subscription(
            String,
            f'{self.namespace}/shuttle/status',
            self._receive_shuttle_status,
            10,
        )
        self.create_subscription(
            BatteryState,
            f'{self.namespace}/battery_state',
            self._receive_battery_status,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            f'{self.namespace}/amcl_pose',
            self._receive_pose,
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(
            Bool,
            f'{self.namespace}/safety/estop',
            self._receive_estop,
            10,
        )
        self.create_subscription(
            Bool,
            f'{self.namespace}/target/paired',
            self._receive_paired,
            10,
        )
        self.create_subscription(
            String,
            f'{self.namespace}/target/worker_id',
            self._receive_worker_id,
            10,
        )
        self.create_subscription(
            String,
            f'{self.namespace}/target/pair_state',
            self._receive_pair_state,
            10,
        )
        self.create_subscription(
            Mission,
            f'{self.namespace}/mission/assigned',
            self._receive_mission,
            10,
        )
        self.fleet_status_publisher = self.create_publisher(
            FleetStatus, FLEET_STATUS_TOPIC, 10
        )
        self.create_timer(1.0, self._publish_status)
        self.create_timer(2.0, self._count_source_publishers)

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
            'mission_state': None,
            'target_worker': '',
            'worker': '',
            'estop': False,
            'status_received': False,
        }

    def _load_parameters(self):
        """로봇 통신과 관제 화면에 필요한 실행 값을 읽는다."""
        self.declare_parameter('status_stale_s', 5.0)
        self.declare_parameter(
            'map_yaml_path', _bringup_path('maps', 'auto_map.yaml')
        )
        self.declare_parameter(
            'arena_yaml_path', _bringup_path('params', 'arena.yaml')
        )
        self.status_overtime_sec = float(
            self.get_parameter('status_stale_s').value
        )
        self.map_yaml_path = str(self.get_parameter('map_yaml_path').value)
        self.arena_yaml_path = str(
            self.get_parameter('arena_yaml_path').value
        )
        if self.status_overtime_sec <= 0.0:
            raise ValueError('status_stale_s must be positive')

    def _record_status_received_time(self):
        """마지막 AMR 상태 수신 시각을 갱신한다."""
        self.status['_received_monotonic'] = time.monotonic()
        self.status['status_received'] = True

    def _record_source(self, source):
        """소스 토픽의 마지막 수신 시각을 남긴다 (status_lock 안에서 부른다)."""
        self.source_times[source] = time.monotonic()

    def _count_source_publishers(self):
        """소스 토픽마다 발행자가 남아 있는지 센다 (I-055)."""
        counts = {
            source: self.count_publishers(
                f'{self.namespace}/{spec["topic"]}'
            )
            for source, spec in STATUS_SOURCES.items()
        }
        with self.status_lock:
            self.source_publishers = counts

    def get_status(self):
        """담당 로봇의 현재 상태와 소스별 수신 시각을 복사해 반환한다."""
        with self.status_lock:
            status = self.status.copy()
            status['_source_monotonic'] = dict(self.source_times)
            status['_source_publishers'] = dict(self.source_publishers)
        return status

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
            self.status['approach_state'] = state
            if state != 'IDLE' and not self._return_has_priority(self.status):
                self.status['state'] = state
            self._record_source('approach')
            self._record_status_received_time()

    def _receive_shuttle_status(self, message):
        """셔틀·복귀 상태를 검사해 담당 AMR 상태에 저장한다."""
        if message.data not in SHUTTLE_STATES:
            self.get_logger().warn(
                f'{self.namespace}: 알 수 없는 shuttle/status '
                f'{message.data!r}'
            )
            return
        with self.status_lock:
            self.status['shuttle_state'] = message.data
            if (
                message.data != 'READY'
                or self.status['approach_state'] == 'IDLE'
            ):
                self.status['state'] = message.data
            self._record_source('shuttle')
            self._record_status_received_time()

    def _receive_battery_status(self, message):
        """담당 AMR의 배터리 잔량을 저장한다."""
        self._update_status_value('battery', message.percentage, 'battery')

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
            self.status['x'] = pose.position.x
            self.status['y'] = pose.position.y
            self.status['yaw'] = yaw
            self._record_source('pose')
            self._record_status_received_time()

    def _receive_estop(self, message):
        """담당 AMR의 안전 정지 여부를 저장한다."""
        self._update_status_value('estop', message.data, 'estop')

    def _receive_worker_id(self, message):
        """담당 AMR이 인식한 작업자 ID를 저장한다."""
        self._update_status_value('worker', message.data, 'worker')

    def _receive_paired(self, message):
        """페어링이 해제되면 작업자 ID를 비운다."""
        if not message.data:
            self._update_status_value('worker', '', 'worker')

    def _receive_pair_state(self, message):
        """인지 노드의 페어링 판정 상태를 저장한다."""
        if message.data not in PAIR_STATES:
            self.get_logger().warn(
                f'{self.namespace}: 알 수 없는 pair_state '
                f'{message.data!r}'
            )
            return
        self._update_status_value('pair_state', message.data, 'pair')

    def _receive_mission(self, message):
        """담당 AMR에 배정된 미션 ID를 저장한다."""
        self._update_status_value('mission', message.mission_id, 'mission')

    def _update_status_value(self, key, value, source):
        """담당 AMR의 단일 상태 값과 수신 시각을 갱신한다."""
        with self.status_lock:
            self.status[key] = value
            self._record_source(source)
            self._record_status_received_time()

    def _publish_status(self):
        """담당 AMR의 최신 상태를 FleetStatus로 1 Hz 발행한다."""
        with self.status_lock:
            status = self.status.copy()
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
        self.fleet_status_publisher.publish(message)
