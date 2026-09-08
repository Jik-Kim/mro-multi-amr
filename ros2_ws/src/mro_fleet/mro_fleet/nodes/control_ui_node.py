"""자재 요청, 상태 표시, 작업 종료를 제공하는 관제 UI 노드.

rclpy executor는 별도 스레드에서 돌고 Flask는 메인 스레드에서 돈다.
"""
import os
import threading

from ament_index_python.packages import get_package_share_directory

from flask import Flask, jsonify, render_template, request

from mro_interfaces.msg import FleetStatus, Intervention
from mro_interfaces.srv import ManualPair, OperatorCommand, RequestMission

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String


DEFAULT_ROBOTS = ['/robot3', '/robot8']
PAIR_STATES = {'PAIRED', 'GATE_FAIL', 'NO_TARGET'}


class ControlUI(Node):
    """ROS 서비스와 Flask 화면을 연결하는 노드."""

    def __init__(self):
        """상태 구독자, 서비스 클라이언트와 공유 상태를 만든다."""
        super().__init__('control_ui')
        self.declare_parameter('robot_namespaces', DEFAULT_ROBOTS)
        self.declare_parameter('manual_pair_timeout_s', 3.0)
        self.robots = list(self.get_parameter('robot_namespaces').value)
        self.manual_pair_timeout_s = float(
            self.get_parameter('manual_pair_timeout_s').value
        )
        self.status = {}
        self.status_lock = threading.Lock()
        self.create_subscription(
            FleetStatus, '/fleet/status', self._status_cb, 10
        )
        self.cli_req = self.create_client(
            RequestMission, '/fleet/request_mission'
        )
        self.cli_op = self.create_client(
            OperatorCommand, '/fleet/operator_command'
        )
        self.cli_manual_pair = self.create_client(
            ManualPair, '/fleet/manual_pair'
        )
        self.pub_int = self.create_publisher(
            Intervention, '/fleet/intervention', 10
        )
        for namespace in self.robots:
            self.create_subscription(
                String,
                f'{namespace}/target/pair_state',
                lambda message, ns=namespace: self._pair_state_cb(
                    ns, message
                ),
                10,
            )

    def _status_cb(self, message):
        status = {
            'state': message.state,
            'battery': message.battery,
            'x': message.x,
            'y': message.y,
            'yaw': message.yaw,
            'mission': message.mission_id,
            'worker': message.paired_worker,
            'estop': message.estop,
        }
        with self.status_lock:
            status['pair_state'] = self.status.get(
                message.robot_ns, {}
            ).get('pair_state', 'NO_TARGET')
            self.status[message.robot_ns] = status

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
        with self.status_lock:
            return {
                namespace: values.copy()
                for namespace, values in self.status.items()
            }

    def _call(self, client, service_request, timeout_sec=2.0):
        if not client.wait_for_service(timeout_sec=1.0):
            return None

        future = client.call_async(service_request)
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout_sec):
            return None
        if future.exception():
            return None
        return future.result()

    def _request_manual_pair(self, robot_ns, worker_id):
        """명시 페어링을 요청하고 성공한 개입만 한 번 기록한다."""
        service_request = ManualPair.Request()
        service_request.robot_ns = robot_ns
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

        message = Intervention()
        message.header.stamp = self.get_clock().now().to_msg()
        message.robot_ns = robot_ns
        message.kind = 'MANUAL_PAIR'
        message.note = worker_id
        self.pub_int.publish(message)
        return True, '명시 페어링 완료'


def main():
    """ROS executor와 Flask 관제 서버를 실행한다."""
    rclpy.init()
    node = ControlUI()
    share = get_package_share_directory('mro_fleet')
    app = Flask(
        __name__, template_folder=os.path.join(share, 'templates')
    )

    @app.route('/')
    def index():
        """관제 메인 화면을 반환한다."""
        return render_template(
            'index.html', status=node._status_snapshot()
        )

    @app.route('/request', methods=['POST'])
    def request_mission():
        """자재 요청을 fleet_manager에 전달한다."""
        service_request = RequestMission.Request()
        service_request.worker_id = request.form['worker_id']
        service_request.bay = request.form['bay']
        response = node._call(node.cli_req, service_request)
        return jsonify(
            ok=bool(response and response.accepted),
            mission_id=(response.mission_id if response else ''),
            message=(
                response.message
                if response
                else 'fleet_manager 응답 없음'
            ),
        )

    @app.route('/command', methods=['POST'])
    def command():
        """작업 종료 명령을 fleet_manager에 전달한다."""
        service_request = OperatorCommand.Request()
        service_request.robot_ns = request.form['robot_ns']
        service_request.command = request.form['command']
        response = node._call(node.cli_op, service_request)
        return jsonify(
            ok=bool(response and response.ok),
            message=(response.message if response else '응답 없음'),
        )

    @app.route('/intervention', methods=['POST'])
    def intervention():
        """계약에 남아 있는 개입 이벤트를 발행한다."""
        message = Intervention()
        message.header.stamp = node.get_clock().now().to_msg()
        message.robot_ns = request.form['robot_ns']
        message.kind = request.form['kind']
        message.note = request.form.get('note', '')
        node.pub_int.publish(message)
        return jsonify(ok=True)

    @app.route('/manual_pair', methods=['POST'])
    def manual_pair():
        """운영자의 명시 페어링 요청과 인지 판단 결과를 전달한다."""
        accepted, reason = node._request_manual_pair(
            request.form.get('robot_ns', ''),
            request.form.get('worker_id', ''),
        )
        return jsonify(ok=accepted, message=reason)

    @app.route('/status')
    def status():
        """가장 최근 로봇 상태의 복사본을 반환한다."""
        return jsonify(node._status_snapshot())

    def spin_node():
        """종료 신호로 ROS 컨텍스트가 닫히면 executor를 정상 종료한다."""
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass

    executor_thread = threading.Thread(target=spin_node, daemon=True)
    executor_thread.start()
    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        executor_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
