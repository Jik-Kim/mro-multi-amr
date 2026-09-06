"""미션 생성·배정, 상태 집계, DB 기록, 종료 명령을 담당한다.

서비스: /fleet/request_mission, /fleet/operator_command
구독: /webcam1/worker_descriptor, 로봇별 상태·배터리·pose·estop
발행: /fleet/mission, 로봇별 mission/assigned·mission/finish,
      /fleet/status
"""
import math
import os
from functools import partial

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseWithCovarianceStamped

from mro_fleet.core.db import FleetDB

from mro_interfaces.msg import FleetStatus, Mission, WorkerDescriptor
from mro_interfaces.srv import OperatorCommand, RequestMission

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import BatteryState

from std_msgs.msg import Bool, String


DEFAULT_ROBOTS = ['/robot3', '/robot4']

SHUTTLE_MISSION_STATES = {
    'SHUTTLE': Mission.SHUTTLE,
    'AT_BAY': Mission.AT_BAY,
    'WAITING': Mission.WAITING,
    'FAILED': Mission.FAILED,
    'DOCKED': Mission.DONE,
}
APPROACH_MISSION_STATES = {
    'FOLLOWING': Mission.FOLLOWING,
    'STANDBY': Mission.STANDBY,
    'FAILED': Mission.FAILED,
}


class FleetManager(Node):
    """두 AMR의 미션과 관제 상태를 관리하는 ROS 2 노드."""

    def __init__(self):
        """파라미터, DB, ROS 통신 객체를 초기화한다."""
        super().__init__('fleet_manager')
        share = get_package_share_directory('mro_fleet')
        self.declare_parameter(
            'db_path', os.path.expanduser('~/mro_fleet.db')
        )
        self.declare_parameter('robot_namespaces', DEFAULT_ROBOTS)
        self.declare_parameter('battery_min', 0.25)
        self.declare_parameter('t_lim_s', 120.0)

        self.robots = list(self.get_parameter('robot_namespaces').value)
        self.battery_min = float(self.get_parameter('battery_min').value)
        self.t_lim_s = float(self.get_parameter('t_lim_s').value)
        schema_path = os.path.join(share, 'config', 'schema.sql')
        self.db = FleetDB(
            self.get_parameter('db_path').value,
            schema_path,
        )
        self.robot = {
            namespace: {
                'state': 'UNKNOWN',
                'mission_state': None,
                'battery': 1.0,
                'x': 0.0,
                'y': 0.0,
                'yaw': 0.0,
                'mission_id': '',
                'paired_worker': '',
                'estop': False,
                'target_worker': '',
            }
            for namespace in self.robots
        }
        self.worker_robot = {}

        self.pub_mission = self.create_publisher(
            Mission, '/fleet/mission', 10
        )
        self.pub_assigned = {
            namespace: self.create_publisher(
                Mission, f'{namespace}/mission/assigned', 10
            )
            for namespace in self.robots
        }
        self.pub_finish = {
            namespace: self.create_publisher(
                Bool, f'{namespace}/mission/finish', 10
            )
            for namespace in self.robots
        }
        self.pub_status = self.create_publisher(
            FleetStatus, '/fleet/status', 10
        )
        self.create_subscription(
            WorkerDescriptor,
            '/webcam1/worker_descriptor',
            self._worker_cb,
            10,
        )
        for namespace in self.robots:
            self._create_robot_subscriptions(namespace)

        self.create_service(
            RequestMission,
            '/fleet/request_mission',
            self._request_cb,
        )
        self.create_service(
            OperatorCommand,
            '/fleet/operator_command',
            self._operator_cb,
        )
        self.create_timer(1.0, self._publish_status)

    def _create_robot_subscriptions(self, namespace):
        self.create_subscription(
            String,
            f'{namespace}/approach/status',
            partial(self._status_cb, namespace, 'approach'),
            10,
        )
        self.create_subscription(
            String,
            f'{namespace}/shuttle/status',
            partial(self._status_cb, namespace, 'shuttle'),
            10,
        )
        self.create_subscription(
            BatteryState,
            f'{namespace}/battery_state',
            partial(self._battery_cb, namespace),
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            f'{namespace}/amcl_pose',
            partial(self._pose_cb, namespace),
            10,
        )
        self.create_subscription(
            Bool,
            f'{namespace}/safety/estop',
            partial(self._estop_cb, namespace),
            10,
        )
        self.create_subscription(
            String,
            f'{namespace}/target/worker_id',
            partial(self._target_worker_cb, namespace),
            10,
        )
        self.create_subscription(
            Bool,
            f'{namespace}/target/paired',
            partial(self._paired_cb, namespace),
            10,
        )

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _set_robot_value(self, namespace, key, value):
        if self.robot[namespace].get(key) != value:
            event_kind = {
                'state': 'STATE',
                'estop': 'ESTOP',
                'paired_worker': 'PAIRED_WORKER',
            }.get(key)
            if event_kind:
                self.db.event(
                    self._now_s(), namespace, event_kind, value
                )
        self.robot[namespace][key] = value

    def _status_cb(self, namespace, source, message):
        status = message.data
        self._set_robot_value(namespace, 'state', status)
        if source == 'shuttle':
            state_map = SHUTTLE_MISSION_STATES
        else:
            state_map = APPROACH_MISSION_STATES
        mission_state = state_map.get(status)
        if mission_state is not None:
            self._transition_mission(namespace, mission_state)

    def _transition_mission(self, namespace, state):
        mission_id = self.robot[namespace]['mission_id']
        previous_state = self.robot[namespace]['mission_state']
        if not mission_id or previous_state == state:
            return

        now = self._now_s()
        if state == Mission.WAITING:
            arrival = self.db.mark_waiting(
                mission_id, now, self.t_lim_s
            )
            if (
                arrival
                and arrival['first_arrival']
                and arrival['arrived_late']
            ):
                self.db.event(
                    now,
                    namespace,
                    'arrived_late',
                    f"{arrival['lead_time_s']:.3f}",
                )
        elif state == Mission.FAILED:
            self.db.set_state(
                mission_id,
                state,
                now,
                result='FAIL_ROBOT',
            )
        else:
            self.db.set_state(mission_id, state, now)

        self.robot[namespace]['mission_state'] = state
        self._publish_mission(namespace, state)

    def _publish_mission(self, namespace, state):
        mission_id = self.robot[namespace]['mission_id']
        row = self.db.mission(mission_id)
        if row is None:
            return

        message = Mission()
        message.header.stamp = self.get_clock().now().to_msg()
        message.mission_id = mission_id
        message.worker_id = row['worker_id']
        message.robot_ns = namespace
        message.bay = row['bay']
        message.state = state
        message.note = row['note'] or ''
        self.pub_mission.publish(message)

    def _target_worker_cb(self, namespace, message):
        self.robot[namespace]['target_worker'] = message.data

    def _paired_cb(self, namespace, message):
        if message.data:
            worker_id = self.robot[namespace]['target_worker']
            if not worker_id:
                mission_id = self.robot[namespace]['mission_id']
                mission = self.db.mission(mission_id)
                worker_id = mission['worker_id'] if mission else ''
            self._set_robot_value(
                namespace, 'paired_worker', worker_id
            )
            self._transition_mission(namespace, Mission.PAIRED)
        else:
            self._set_robot_value(namespace, 'paired_worker', '')

    def _battery_cb(self, namespace, message):
        self._set_robot_value(
            namespace, 'battery', message.percentage
        )

    def _estop_cb(self, namespace, message):
        self._set_robot_value(namespace, 'estop', message.data)

    def _pose_cb(self, namespace, message):
        pose = message.pose.pose
        self.robot[namespace]['x'] = pose.position.x
        self.robot[namespace]['y'] = pose.position.y
        quaternion = pose.orientation
        self.robot[namespace]['yaw'] = math.atan2(
            2.0 * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0 - 2.0 * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )

    def _publish_status(self):
        for namespace, robot in self.robot.items():
            status = FleetStatus()
            status.header.stamp = self.get_clock().now().to_msg()
            status.robot_ns = namespace
            status.state = robot['state']
            status.battery = float(robot['battery'])
            status.x = robot['x']
            status.y = robot['y']
            status.yaw = robot['yaw']
            status.mission_id = robot['mission_id']
            status.paired_worker = robot['paired_worker']
            status.estop = bool(robot['estop'])
            self.pub_status.publish(status)

    def _worker_cb(self, descriptor):
        if descriptor.worker_id in self.worker_robot:
            return

        assigned = set(self.worker_robot.values())
        free = [
            namespace
            for namespace in self.robots
            if namespace not in assigned
        ]
        if not free:
            self.get_logger().warn(
                f'{descriptor.worker_id}: 배정 가능한 로봇 없음'
            )
            return

        namespace = free[0]
        self.worker_robot[descriptor.worker_id] = namespace
        self.db.upsert_worker(
            descriptor.worker_id,
            descriptor.outfit_class,
            self._now_s(),
            namespace,
        )
        self.get_logger().info(
            f'{descriptor.worker_id}({descriptor.outfit_class}) '
            f'→ {namespace}'
        )

    def _request_cb(self, request, response):
        namespace = self.worker_robot.get(request.worker_id)
        if namespace is None:
            response.accepted = False
            response.message = '미등록 작업자'
            return response

        battery = self.robot[namespace]['battery']
        if battery < self.battery_min:
            response.accepted = False
            response.message = (
                f'{namespace} 배터리 임계 미만 — 배정 거부 (BR-T14)'
            )
            self.db.event(
                self._now_s(),
                namespace,
                'ASSIGN_REFUSED_BATTERY',
                battery,
            )
            return response

        now = self._now_s()
        mission_id = self.db.create_mission(
            request.worker_id,
            request.bay,
            now,
            namespace,
        )
        mission = Mission()
        mission.header.stamp = self.get_clock().now().to_msg()
        mission.mission_id = mission_id
        mission.worker_id = request.worker_id
        mission.robot_ns = namespace
        mission.bay = request.bay
        mission.state = Mission.ASSIGNED
        self.robot[namespace]['mission_id'] = mission_id
        self.robot[namespace]['mission_state'] = Mission.ASSIGNED
        self.db.set_state(mission_id, Mission.ASSIGNED, now)
        self.pub_mission.publish(mission)
        self.pub_assigned[namespace].publish(mission)

        response.accepted = True
        response.mission_id = mission_id
        response.message = (
            f'{mission_id} · {request.worker_id} · 창고 {request.bay} · '
            f'{namespace.lstrip("/")} 배정 완료'
        )
        return response

    def _operator_cb(self, request, response):
        if request.command not in ('RETURN', 'RETURN_ALL'):
            response.ok = False
            response.message = (
                f'{request.command} 는 지원하지 않음 '
                '(RETURN / RETURN_ALL 만 허용)'
            )
            return response
        if (
            request.command == 'RETURN'
            and request.robot_ns not in self.robot
        ):
            response.ok = False
            response.message = '알 수 없는 로봇'
            return response

        # P1 통과 뒤 DOCK 토큰이 순차 도킹을 보장한다.
        if request.command == 'RETURN_ALL':
            targets = list(self.robots)
        else:
            targets = [request.robot_ns]
        for namespace in targets:
            self.db.event(
                self._now_s(), namespace, 'OPERATOR', request.command
            )
            self.pub_finish[namespace].publish(Bool(data=True))

        response.ok = True
        response.message = (
            f"{', '.join(targets)} 작업 종료 → 복귀 전달"
        )
        return response


def main():
    """fleet_manager 노드를 실행한다."""
    rclpy.init()
    node = FleetManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.db.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
