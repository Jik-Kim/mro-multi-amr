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
from mro_fleet.nodes.fleet_client import FleetUICommands
from mro_fleet.nodes.robot_control_node import (
    DEFAULT_ROBOTS, FLEET_STATUS_TOPIC,
)
from mro_fleet.nodes.robot_state_manager import RobotStateManager

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
ZONE_IDS = {'P1', 'DOCK'}
ZONE_EVENT_ACTIONS = {'REJECT', 'GRANT', 'RELEASE', 'RECLAIM'}


class FleetManager(Node):
    """두 AMR의 미션 생성·배정·전이와 DB 기록을 담당한다."""

    def __init__(self):
        """파라미터, DB, ROS 통신 객체를 초기화한다."""
        super().__init__('fleet_manager')
        package_share_directory = get_package_share_directory('mro_fleet')
        self.declare_parameter(
            'db_path', os.path.expanduser('~/mro_fleet.db')
        )
        self.declare_parameter('robot_namespaces', DEFAULT_ROBOTS)
        self.declare_parameter('battery_min', 0.25)
        self.declare_parameter('t_lim_s', 120.0)
        self.declare_parameter('manual_pair_timeout_s', 2.0)

        self.robot_namespaces = list(
            self.get_parameter('robot_namespaces').value
        )
        self.battery_min = float(self.get_parameter('battery_min').value)
        self.t_lim_s = float(self.get_parameter('t_lim_s').value)
        self.manual_pair_timeout_s = float(
            self.get_parameter('manual_pair_timeout_s').value
        )
        if self.manual_pair_timeout_s <= 0.0:
            raise ValueError('manual_pair_timeout_s must be positive')
        schema_path = os.path.join(
            package_share_directory, 'config', 'schema.sql'
        )
        self.db = FleetDB(
            self.get_parameter('db_path').value,
            schema_path,
        )
        self.registered_workers = set()
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
            for namespace in self.robot_namespaces
        }
        self.pub_finish = {
            namespace: self.create_publisher(
                Bool, f'{namespace}/mission/finish', 10
            )
            for namespace in self.robot_namespaces
        }
        self.manual_pair_clients = {
            namespace: self.create_client(
                ManualPair,
                f'{namespace}/manual_pair',
                callback_group=self.manual_pair_client_group,
            )
            for namespace in self.robot_namespaces
        }
        self.create_subscription(
            WorkerDescriptor,
            '/webcam1/worker_descriptor',
            self._register_worker,
            10,
        )
        self.create_subscription(
            Intervention,
            '/fleet/intervention',
            self._record_intervention_event,
            10,
        )
        self.create_subscription(
            String,
            '/fleet/zone_event',
            self._record_zone_event,
            10,
        )
        self.create_subscription(
            FleetStatus,
            '/fleet/status',
            self._receive_fleet_status,
            10,
        )

        self.create_service(
            RequestMission,
            '/fleet/request_mission',
            self._handle_mission_request,
        )
        self.create_service(
            OperatorCommand,
            '/fleet/operator_command',
            self.return_immediately,
        )
        self.create_service(
            ManualPair,
            '/fleet/manual_pair',
            self._forward_manual_pair_request,
            callback_group=self.manual_pair_service_group,
        )
        self.create_timer(1.0, self._mark_overdue_missions)
        # 첫 판정은 10 초 뒤 — 그 전에는 디스커버리가 덜 끝나 발행자가 덜 보인다.
        self.create_timer(10.0, self._check_duplicate_monitor)

    def _record_intervention_event(self, message):
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
            event_at = (
                self.get_clock().now().nanoseconds / 1_000_000_000.0
            )
        self.db.event(
            event_at,
            message.robot_ns,
            'INTERVENTION',
            message.kind,
            message.note,
        )

    def _record_zone_event(self, message):
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
            or self.robot_state_manager.get_robot(robot_ns) is None
            or action not in ZONE_EVENT_ACTIONS
        ):
            self.get_logger().warn(
                f'zone_event 값이 계약과 다르다: {message.data!r}'
            )
            return

        self.db.event(
            self.get_clock().now().nanoseconds / 1_000_000_000.0,
            robot_ns,
            'ZONE',
            action,
            zone_id,
        )

    def _receive_fleet_status(self, message):
        """수신한 로봇 상태를 반영하고 변경 이벤트를 DB에 동기 저장한다.

        `FleetDB.event()`와 미션 갱신은 SQLite 트랜잭션을 사용하므로
        이 콜백이 정상 반환할 때는 해당 DB 커밋도 완료된다.
        """
        namespace = message.robot_ns
        if self.robot_state_manager.get_robot(namespace) is None:
            self.get_logger().warn(
                f'등록되지 않은 FleetStatus 로봇: {namespace!r}'
            )
            return

        changes = self.robot_state_manager.update_fleet_status(message)
        if 'shuttle_state' in changes:
            self._process_robot_state_for_mission(
                namespace, 'shuttle', String(data=message.shuttle_state)
            )
        if 'approach_state' in changes:
            self._process_robot_state_for_mission(
                namespace, 'approach', String(data=message.approach_state)
            )
        if 'estop' in changes:
            self.db.event(
                self.get_clock().now().nanoseconds / 1_000_000_000.0,
                namespace,
                'ESTOP',
                message.estop,
            )
        if 'paired_worker' in changes:
            self.db.event(
                self.get_clock().now().nanoseconds / 1_000_000_000.0,
                namespace,
                'PAIRED_WORKER',
                message.paired_worker,
            )
        previous_worker = changes.get('paired_worker', ('', ''))[0]
        if message.paired_worker and not previous_worker:
            self._update_mission_state(namespace, Mission.PAIRED)

    def _process_robot_state_for_mission(self, namespace, source, message):
        """RobotStatus가 요약한 상태로 미션 전이를 판정한다."""
        status = message.data
        if not self.robot_state_manager.update_source_state(
            namespace, source, status
        ):
            return
        self.db.event(
            self.get_clock().now().nanoseconds / 1_000_000_000.0,
            namespace,
            'STATE',
            status,
            source,
        )
        if source == 'shuttle':
            state_map = SHUTTLE_MISSION_STATES
        else:
            state_map = APPROACH_MISSION_STATES
        mission_state = state_map.get(status)
        if (
            mission_state is not None
            and self.robot_state_manager.is_mission_transition_allowed(
                namespace, source, mission_state
            )
        ):
            self._update_mission_state(namespace, mission_state)

    def _update_mission_state(self, namespace, state):
        robot_status = self.robot_state_manager.get_robot(namespace).get_status()
        mission_id = robot_status['mission']
        previous_state = robot_status['mission_state']
        if (
            not mission_id
            or previous_state == state
            or previous_state == Mission.DONE
        ):
            return

        now = self.get_clock().now().nanoseconds / 1_000_000_000.0
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

        self.robot_state_manager.update_mission_state(namespace, state)
        self._publish_mission_update(namespace, state)
        if state == Mission.DONE:
            self.robot_state_manager.clear_mission(namespace)

    def _publish_mission_update(self, namespace, state):
        robot = self.robot_state_manager.get_robot(namespace)
        mission_id = robot.get_status()['mission']
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

    def _check_duplicate_monitor(self):
        """관제가 두 벌 떠 있는지 보고 조용히 틀리지 않게 한다 (I-054).

        ``/fleet/status`` 는 전역 토픽이라 다른 PC 의 관제도 **같은 robot_ns 로**
        발행한다. 받는 쪽은 나중에 온 것을 그대로 쓰므로, 이쪽이 amcl 에서 막 받은
        pose 가 저쪽 값에 1 초 안에 덮인다. 값 자체는 늘 그럴듯해서 화면만 보면
        무엇이 틀렸는지 알 수 없다 — 그래서 세어서 말해 준다.

        기대 발행자 수는 이 프로세스의 ``RobotStatus`` 개수, 즉 로봇 수다.
        """
        expected = len(self.robot_namespaces)
        found = self.count_publishers(FLEET_STATUS_TOPIC)
        manager = getattr(self, 'robot_state_manager', None)
        if found <= expected:
            if manager is not None:
                manager.duplicate_monitor_warning = ''
            return
        warning = (
            f'{FLEET_STATUS_TOPIC} 발행자가 {found} 개다 (이 관제가 내는 것은 '
            f'{expected} 개) — 다른 PC 에서 관제가 함께 돌고 있다. 두 인스턴스가 '
            f'서로의 로봇 상태를 덮어쓴다. 관제는 한 대에서만 띄운다 (I-054)'
        )
        if manager is not None:
            manager.duplicate_monitor_warning = warning
        self.get_logger().error(warning)

    def _mark_overdue_missions(self):
        """T_lim을 넘긴 미도착 미션을 실패 처리하되 운행은 유지한다."""
        now = self.get_clock().now().nanoseconds / 1_000_000_000.0
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

    def _register_worker(self, descriptor):
        """작업자를 등록하되 AMR은 자재 요청이 올 때 배정한다."""
        worker_id = descriptor.worker_id
        if worker_id in self.registered_workers:
            return
        if not worker_id:
            self.get_logger().warn('worker_id가 빈 작업자 등록 무시')
            return
        self.registered_workers.add(worker_id)
        self.db.upsert_worker(
            worker_id,
            descriptor.outfit_class,
            self.get_clock().now().nanoseconds / 1_000_000_000.0,
        )
        self.get_logger().info(
            f'{worker_id}({descriptor.outfit_class}) 등록 완료'
        )

    def _handle_mission_request(self, request, response):
        if request.worker_id not in self.registered_workers:
            response.accepted = False
            response.message = '미등록 작업자'
            return response

        available_robots = self.robot_state_manager.get_available_robots()
        if not available_robots:
            response.accepted = False
            response.message = '가용 AMR 없음'
            return response

        selected_robot = next(
            (
                robot for robot in available_robots
                if robot.get_status()['battery'] >= self.battery_min
            ),
            None,
        )
        if selected_robot is None:
            rejected_robot = available_robots[0]
            battery = rejected_robot.get_status()['battery']
            response.accepted = False
            response.message = '가용 AMR 배터리 임계 미만 — 배정 거부'
            self.db.event(
                self.get_clock().now().nanoseconds / 1_000_000_000.0,
                rejected_robot.namespace,
                'ASSIGN_REFUSED_BATTERY',
                battery,
            )
            return response

        namespace = selected_robot.namespace
        now = self.get_clock().now().nanoseconds / 1_000_000_000.0
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
        self.robot_state_manager.assign_mission(
            namespace, mission_id, request.worker_id, Mission.ASSIGNED
        )
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

    def return_immediately(self, request, response):
        """현재 작업을 즉시 종료하고 대상 AMR을 복귀시킨다."""
        if request.command not in ('RETURN', 'RETURN_ALL'):
            response.ok = False
            response.message = (
                f'{request.command} 는 지원하지 않음 '
                '(RETURN / RETURN_ALL 만 허용)'
            )
            return response

        if (
            request.command == 'RETURN'
            and self.robot_state_manager.get_robot(request.robot_ns) is None
        ):
            response.ok = False
            response.message = '알 수 없는 로봇'
            return response

        # P1 통과 뒤 DOCK 토큰이 순차 도킹을 보장한다.
        if request.command == 'RETURN_ALL':
            targets = list(self.robot_namespaces)
        else:
            targets = [request.robot_ns]
        for namespace in targets:
            self.db.event(
                self.get_clock().now().nanoseconds / 1_000_000_000.0,
                namespace,
                'OPERATOR',
                request.command,
            )
            self.pub_finish[namespace].publish(Bool(data=True))

        response.ok = True
        response.message = (
            f"{', '.join(targets)} 작업 종료 → 복귀 전달"
        )
        return response

    @staticmethod
    def _complete_manual_pair_wait_once(completion, result):
        """응답과 시간 초과 중 먼저 도착한 결과 하나만 확정한다."""
        if not completion.done():
            completion.set_result(result)

    async def _forward_manual_pair_request(self, request, response):
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
            lambda: self._complete_manual_pair_wait_once(completion, None),
            callback_group=self.manual_pair_client_group,
        )
        future.add_done_callback(
            lambda done: self._complete_manual_pair_wait_once(
                completion, done
            )
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
    executor = MultiThreadedExecutor(num_threads=4)
    fleet_manager = FleetManager()
    robot_state_manager = RobotStateManager(
        fleet_manager.robot_namespaces, executor
    )
    fleet_ui_commands = FleetUICommands()
    executor.add_node(fleet_ui_commands)
    fleet_manager.robot_state_manager = robot_state_manager
    package_share_directory = get_package_share_directory('mro_fleet')
    template_directory = os.path.join(package_share_directory, 'templates')
    monitor = MonitorWebApp(
        robot_state_manager, fleet_ui_commands, template_directory
    )
    executor.add_node(fleet_manager)

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
        for namespace in list(robot_state_manager.robots):
            robot_state_manager.remove_robot(namespace)
        executor.remove_node(fleet_manager)
        executor.remove_node(fleet_ui_commands)
        executor.shutdown()
        fleet_manager.db.close()
        fleet_manager.destroy_node()
        fleet_ui_commands.destroy_node()


if __name__ == '__main__':
    main()
