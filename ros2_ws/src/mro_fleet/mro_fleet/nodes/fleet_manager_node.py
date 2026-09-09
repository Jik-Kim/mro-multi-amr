"""미션 생성·배정, DB 기록, 종료 명령을 담당한다.

서비스: /fleet/request_mission, /fleet/operator_command, /fleet/manual_pair
구독: /webcam1/worker_descriptor, /fleet/intervention, /fleet/zone_event,
      /fleet/status
발행: /fleet/mission, 로봇별 mission/assigned·mission/finish,
"""
import os
import threading

from ament_index_python.packages import get_package_share_directory

from mro_fleet.core.db import FleetDB
from mro_fleet.nodes.MonitorWebApp import MonitorWebApp
from mro_fleet.nodes.robot_control_node import DEFAULT_ROBOTS, RobotControlNode

from mro_interfaces.msg import (
    FleetStatus,
    Intervention,
    Mission,
    WorkerDescriptor,
)
from mro_interfaces.srv import ManualPair, OperatorCommand, RequestMission

import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

from std_msgs.msg import Bool, String


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
}
SHUTTLE_OUTBOUND_STATES = {'READY', 'SHUTTLE', 'AT_BAY', 'WAITING'}
SHUTTLE_RETURN_STATES = {'RETURNING', 'DOCK_WAIT', 'DOCKED'}
APPROACH_ACTIVE_STATES = {
    'SEARCHING',
    'APPROACH_1',
    'APPROACH_2',
    'ARRIVED',
    'FOLLOWING',
    'STANDBY',
}
APPROACH_MISSION_PHASES = {
    Mission.PAIRED,
    Mission.FOLLOWING,
    Mission.STANDBY,
}
TERMINAL_MISSION_STATES = {Mission.DONE}
ZONE_IDS = {'P1', 'DOCK'}
ZONE_EVENT_ACTIONS = {'REJECT', 'GRANT', 'RELEASE', 'RECLAIM'}


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
        self.declare_parameter('worker_assignment_window_s', 1.0)
        self.declare_parameter('manual_pair_timeout_s', 2.0)

        self.robots = list(self.get_parameter('robot_namespaces').value)
        self.battery_min = float(self.get_parameter('battery_min').value)
        self.t_lim_s = float(self.get_parameter('t_lim_s').value)
        self.worker_assignment_window_s = float(
            self.get_parameter('worker_assignment_window_s').value
        )
        self.manual_pair_timeout_s = float(
            self.get_parameter('manual_pair_timeout_s').value
        )
        if self.worker_assignment_window_s <= 0.0:
            raise ValueError('worker_assignment_window_s must be positive')
        if self.manual_pair_timeout_s <= 0.0:
            raise ValueError('manual_pair_timeout_s must be positive')
        schema_path = os.path.join(share, 'config', 'schema.sql')
        self.db = FleetDB(
            self.get_parameter('db_path').value,
            schema_path,
        )
        self.robot = {
            namespace: {
                'approach_state': 'UNKNOWN',
                'shuttle_state': 'UNKNOWN',
                'status_owner': None,
                'mission_state': None,
                'battery': 1.0,
                'x': 0.0,
                'y': 0.0,
                'yaw': 0.0,
                'mission_id': '',
                'paired_worker': '',
                'estop': False,
                'target_worker': '',
                'pair_state': 'NO_TARGET',
            }
            for namespace in self.robots
        }
        self.worker_robot = {}
        self.pending_workers = {}
        self.worker_assignment_timer = None
        # 중계 서비스가 하위 서비스 응답을 기다리는 동안 다른 ROS 콜백이
        # 실행될 수 있도록 서버와 클라이언트 그룹을 분리한다.
        self.manual_pair_service_group = MutuallyExclusiveCallbackGroup()
        self.manual_pair_client_group = ReentrantCallbackGroup()

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
        self.manual_pair_clients = {
            namespace: self.create_client(
                ManualPair,
                f'{namespace}/manual_pair',
                callback_group=self.manual_pair_client_group,
            )
            for namespace in self.robots
        }
        self.create_subscription(
            WorkerDescriptor,
            '/webcam1/worker_descriptor',
            self._worker_cb,
            10,
        )
        self.create_subscription(
            Intervention,
            '/fleet/intervention',
            self._intervention_cb,
            10,
        )
        self.create_subscription(
            String,
            '/fleet/zone_event',
            self._zone_event_cb,
            10,
        )
        self.create_subscription(
            FleetStatus,
            '/fleet/status',
            self._fleet_status_cb,
            10,
        )

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
        self.create_service(
            ManualPair,
            '/fleet/manual_pair',
            self._manual_pair_cb,
            callback_group=self.manual_pair_service_group,
        )
        self.create_timer(1.0, self._expire_overdue_missions)

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _intervention_cb(self, message):
        """개입 메시지의 원 시각과 내용을 SQLite 이벤트로 기록한다."""
        stamp = message.header.stamp
        stamp_is_valid = (
            stamp.sec >= 0
            and 0 <= stamp.nanosec < 1_000_000_000
            and (stamp.sec != 0 or stamp.nanosec != 0)
        )
        if stamp_is_valid:
            event_at = stamp.sec + stamp.nanosec / 1_000_000_000.0
        else:
            event_at = self._now_s()
        self.db.event(
            event_at,
            message.robot_ns,
            'INTERVENTION',
            message.kind,
            message.note,
        )

    def _zone_event_cb(self, message):
        """구역 세마포어 이벤트를 SQLite의 ZONE 이벤트로 기록한다."""
        parts = message.data.split()
        if len(parts) != 3:
            self.get_logger().warn(
                f'zone_event 형식이 계약과 다르다: {message.data!r}'
            )
            return

        zone_id, robot_ns, action = parts
        if (
            zone_id not in ZONE_IDS
            or robot_ns not in self.robot
            or action not in ZONE_EVENT_ACTIONS
        ):
            self.get_logger().warn(
                f'zone_event 값이 계약과 다르다: {message.data!r}'
            )
            return

        self.db.event(
            self._now_s(),
            robot_ns,
            'ZONE',
            action,
            zone_id,
        )

    def _set_robot_value(self, namespace, key, value):
        if self.robot[namespace].get(key) != value:
            event_kind = {
                'estop': 'ESTOP',
                'paired_worker': 'PAIRED_WORKER',
            }.get(key)
            if event_kind:
                self.db.event(
                    self._now_s(), namespace, event_kind, value
                )
        self.robot[namespace][key] = value

    def _fleet_status_cb(self, message):
        """RobotControlNode의 요약 상태를 미션 판단과 DB 기록에 반영한다."""
        namespace = message.robot_ns
        if namespace not in self.robot:
            self.get_logger().warn(
                f'등록되지 않은 FleetStatus 로봇: {namespace!r}'
            )
            return

        robot = self.robot[namespace]
        if message.shuttle_state != robot['shuttle_state']:
            self._status_cb(
                namespace, 'shuttle', String(data=message.shuttle_state)
            )
        if message.approach_state != robot['approach_state']:
            self._status_cb(
                namespace, 'approach', String(data=message.approach_state)
            )

        self._set_robot_value(namespace, 'battery', message.battery)
        self._set_robot_value(namespace, 'estop', message.estop)
        robot['x'] = message.x
        robot['y'] = message.y
        robot['yaw'] = message.yaw
        robot['pair_state'] = message.pair_state

        previous_worker = robot['paired_worker']
        self._set_robot_value(
            namespace, 'paired_worker', message.paired_worker
        )
        if message.paired_worker and not previous_worker:
            self._transition_mission(namespace, Mission.PAIRED)

    def _status_cb(self, namespace, source, message):
        status = message.data
        self._set_source_state(namespace, source, status)
        if source == 'shuttle':
            state_map = SHUTTLE_MISSION_STATES
        else:
            state_map = APPROACH_MISSION_STATES
        mission_state = state_map.get(status)
        if (
            mission_state is not None
            and self._can_transition_from_status(
                namespace, source, mission_state
            )
        ):
            self._transition_mission(namespace, mission_state)

    def _set_source_state(self, namespace, source, status):
        """상태 출처별 최신값을 저장하고 화면에 쓸 출처를 고른다."""
        robot = self.robot[namespace]
        key = f'{source}_state'
        if robot[key] != status:
            self.db.event(
                self._now_s(), namespace, 'STATE', status, source
            )
            robot[key] = status

        if source == 'shuttle':
            is_return = self._is_return_state(status)
            is_outbound = status in SHUTTLE_OUTBOUND_STATES
            mission_phase = robot['mission_state']
            if is_return or status == 'FAILED':
                if mission_phase not in TERMINAL_MISSION_STATES:
                    robot['status_owner'] = 'shuttle'
            elif (
                is_outbound
                and mission_phase not in APPROACH_MISSION_PHASES
                and mission_phase not in TERMINAL_MISSION_STATES
            ):
                robot['status_owner'] = 'shuttle'
        elif (
            not self._shuttle_return_has_priority(robot)
            and robot['mission_state'] not in TERMINAL_MISSION_STATES
            and (
                status in APPROACH_ACTIVE_STATES
                or not robot['mission_id']
            )
        ):
            robot['status_owner'] = 'approach'

    @staticmethod
    def _is_return_state(status):
        """복귀·도킹 상태인지 판별한다."""
        return (
            status in SHUTTLE_RETURN_STATES
            or status.startswith('DOCKING')
        )

    @staticmethod
    def _shuttle_return_has_priority(robot):
        """복귀가 시작돼 shuttle 상태를 유지해야 하는지 반환한다."""
        return (
            robot['status_owner'] == 'shuttle'
            and (
                FleetManager._is_return_state(robot['shuttle_state'])
                or robot['shuttle_state'] == 'FAILED'
            )
        )

    def _can_transition_from_status(
        self, namespace, source, mission_state
    ):
        """서로 다른 상태 출처가 미션 단계를 되돌리지 않게 한다."""
        robot = self.robot[namespace]
        current = robot['mission_state']
        if current in TERMINAL_MISSION_STATES:
            return False
        if source == 'approach':
            if self._shuttle_return_has_priority(robot):
                return False
            return current in {
                Mission.WAITING,
                Mission.PAIRED,
                Mission.FOLLOWING,
                Mission.STANDBY,
            }
        if mission_state in {
            Mission.SHUTTLE,
            Mission.AT_BAY,
            Mission.WAITING,
        }:
            return current not in APPROACH_MISSION_PHASES
        return True

    @staticmethod
    def _composed_state(robot):
        """현재 단계의 담당 출처가 보낸 상태를 FleetStatus에 사용한다."""
        owner = robot['status_owner']
        if owner:
            return robot[f'{owner}_state']
        if robot['approach_state'] != 'UNKNOWN':
            return robot['approach_state']
        return robot['shuttle_state']

    def _transition_mission(self, namespace, state):
        mission_id = self.robot[namespace]['mission_id']
        previous_state = self.robot[namespace]['mission_state']
        if (
            not mission_id
            or previous_state == state
            or previous_state in TERMINAL_MISSION_STATES
        ):
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

    def _expire_overdue_missions(self):
        """T_lim을 넘긴 미도착 미션을 실패 처리하되 운행은 유지한다."""
        now = self._now_s()
        for mission in self.db.expire_overdue(now, self.t_lim_s):
            self.db.event(
                now,
                mission['robot_ns'],
                'TLIM_EXCEEDED',
                mission['mission_id'],
            )
            self.get_logger().warn(
                f"{mission['mission_id']}: T_lim 초과 — 운행 계속"
            )

    def _worker_cb(self, descriptor):
        worker_id = descriptor.worker_id
        if worker_id in self.worker_robot:
            return

        if not self._available_robots():
            self.get_logger().warn(
                f'{worker_id}: 배정 가능한 로봇 없음'
            )
            return

        if worker_id not in self.pending_workers:
            self.pending_workers[worker_id] = (
                descriptor,
                self._now_s(),
            )

        if len(self.pending_workers) >= len(self.robots):
            self._flush_pending_workers()
        elif self.worker_assignment_timer is None:
            self.worker_assignment_timer = self.create_timer(
                self.worker_assignment_window_s,
                self._flush_pending_workers,
            )

    def _available_robots(self):
        """기존 순서를 유지하며 아직 배정되지 않은 로봇을 반환한다."""
        assigned = set(self.worker_robot.values())
        return [
            namespace
            for namespace in self.robots
            if namespace not in assigned
        ]

    def _flush_pending_workers(self):
        """동시 등록 후보를 worker_id 순서로 로봇에 확정 배정한다."""
        timer = self.worker_assignment_timer
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
            self.worker_assignment_timer = None

        pending = self.pending_workers
        self.pending_workers = {}
        for worker_id in sorted(pending):
            if worker_id in self.worker_robot:
                continue

            free = self._available_robots()
            if not free:
                self.get_logger().warn(
                    f'{worker_id}: 배정 가능한 로봇 없음'
                )
                continue

            descriptor, registered_at = pending[worker_id]
            namespace = free[0]
            self.worker_robot[worker_id] = namespace
            self.db.upsert_worker(
                worker_id,
                descriptor.outfit_class,
                registered_at,
                namespace,
            )
            self.get_logger().info(
                f'{worker_id}({descriptor.outfit_class}) '
                f'→ {namespace}'
            )

    def _request_cb(self, request, response):
        if request.worker_id in self.pending_workers:
            self._flush_pending_workers()

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
        self.robot[namespace]['approach_state'] = 'UNKNOWN'
        self.robot[namespace]['shuttle_state'] = 'UNKNOWN'
        self.robot[namespace]['status_owner'] = None
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

    @staticmethod
    def _resolve_manual_pair_wait(completion, result):
        """응답과 시간 초과 중 먼저 도착한 결과 하나만 확정한다."""
        if not completion.done():
            completion.set_result(result)

    async def _manual_pair_cb(self, request, response):
        """UI 요청을 선택된 로봇의 amr_detection 서비스로 전달한다."""
        client = self.manual_pair_clients.get(request.robot_ns)
        if client is None:
            response.accepted = False
            response.reason = 'WRONG_ROBOT'
            return response

        if not client.service_is_ready():
            self.get_logger().warn(
                f'{request.robot_ns}: manual_pair 서비스 준비 안 됨'
            )
            response.accepted = False
            response.reason = ''
            return response

        forwarded_request = ManualPair.Request()
        forwarded_request.robot_ns = request.robot_ns
        forwarded_request.worker_id = request.worker_id
        future = client.call_async(forwarded_request)
        completion = Future()
        timeout_timer = self.create_timer(
            self.manual_pair_timeout_s,
            lambda: self._resolve_manual_pair_wait(completion, None),
            callback_group=self.manual_pair_client_group,
        )
        future.add_done_callback(
            lambda done: self._resolve_manual_pair_wait(completion, done)
        )
        completed_future = await completion
        timeout_timer.cancel()
        self.destroy_timer(timeout_timer)

        if completed_future is None:
            self.get_logger().warn(
                f'{request.robot_ns}: manual_pair 응답 시간 초과'
            )
            response.accepted = False
            response.reason = ''
            return response

        try:
            forwarded_response = completed_future.result()
        except Exception as error:
            self.get_logger().error(
                f'{request.robot_ns}: manual_pair 호출 실패: {error}'
            )
            response.accepted = False
            response.reason = ''
            return response

        if forwarded_response is None:
            response.accepted = False
            response.reason = ''
            return response

        response.accepted = bool(forwarded_response.accepted)
        response.reason = forwarded_response.reason
        return response


def main():
    """fleet_manager와 관제 웹·로봇 통신 파트를 함께 실행한다."""
    rclpy.init()
    fleet_manager = FleetManager()
    robot_controls = [
        RobotControlNode(namespace) for namespace in DEFAULT_ROBOTS
    ]
    share = get_package_share_directory('mro_fleet')
    monitor = MonitorWebApp(robot_controls, os.path.join(share, 'templates'))
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(fleet_manager)
    for robot_control in robot_controls:
        executor.add_node(robot_control)

    def spin_ros_nodes():
        """웹 서버와 동시에 fleet·UI ROS 콜백을 처리한다."""
        try:
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass

    executor_thread = threading.Thread(target=spin_ros_nodes, daemon=True)
    executor_thread.start()
    try:
        monitor.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        executor_thread.join(timeout=2.0)
        for robot_control in robot_controls:
            executor.remove_node(robot_control)
        executor.remove_node(fleet_manager)
        executor.shutdown()
        fleet_manager.db.close()
        for robot_control in robot_controls:
            robot_control.destroy_node()
        fleet_manager.destroy_node()


if __name__ == '__main__':
    main()
