"""관제 UI의 ROS 서비스 요청과 개입 이벤트 발행을 담당한다."""
import threading

from mro_interfaces.msg import Intervention
from mro_interfaces.srv import ManualPair, OperatorCommand, RequestMission

from rclpy.node import Node


class FleetClient(Node):
    """웹 UI와 FleetManager 사이의 ROS 통신을 담당한다."""

    def __init__(self):
        """관제 UI가 사용할 ROS 서비스 클라이언트와 발행자를 만든다."""
        super().__init__('fleet_client')
        self.declare_parameter('manual_pair_timeout_s', 3.0)
        self.manual_pair_timeout_s = float(
            self.get_parameter('manual_pair_timeout_s').value
        )
        self.request_mission_client = self.create_client(
            RequestMission, '/fleet/request_mission'
        )
        self.return_client = self.create_client(
            OperatorCommand, '/fleet/operator_command'
        )
        self.manual_pair_client = self.create_client(
            ManualPair, '/fleet/manual_pair'
        )
        self.intervention_publisher = self.create_publisher(
            Intervention, '/fleet/intervention', 10
        )

    def _call_service(self, client, request, timeout_sec=2.0):
        """서비스 응답을 제한 시간 동안 기다린다."""
        if not client.wait_for_service(timeout_sec=1.0):
            return None
        future = client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout_sec) or future.exception():
            return None
        return future.result()

    def request_mission(self, worker_id, bay):
        """작업자의 자재 운반 미션을 요청한다."""
        request = RequestMission.Request()
        request.worker_id = worker_id
        request.bay = bay
        return self._call_service(self.request_mission_client, request)

    def return_immediately(self, namespace, command):
        """해당 로봇 또는 전체 로봇의 즉시 복귀를 요청한다."""
        request = OperatorCommand.Request()
        request.robot_ns = namespace
        request.command = command
        return self._call_service(self.return_client, request)

    def publish_intervention(self, namespace, kind, note=''):
        """운영자 개입 이벤트를 발행한다."""
        message = Intervention()
        message.header.stamp = self.get_clock().now().to_msg()
        message.robot_ns = namespace
        message.kind = kind
        message.note = note
        self.intervention_publisher.publish(message)

    def request_manual_pair(self, namespace, worker_id):
        """해당 로봇과 작업자의 명시 페어링을 요청한다."""
        request = ManualPair.Request()
        request.robot_ns = namespace
        request.worker_id = worker_id
        response = self._call_service(
            self.manual_pair_client,
            request,
            timeout_sec=self.manual_pair_timeout_s,
        )
        if response is None:
            return False, 'fleet_manager 또는 인지 서비스 응답 없음'
        if not response.accepted:
            return False, response.reason or '인지 서비스 응답 없음'
        self.publish_intervention(namespace, 'MANUAL_PAIR', worker_id)
        return True, '명시 페어링 완료'
