"""관제 UI — 자재 요청 입력(S3), 로봇 상태 표시(BR-T15, 단순 뷰어), 작업자별/전체 종료→순차 복귀(BR-T10, S9·S10), 개입 버튼(BR-T26). 정지·재개 없음(9/4 결정).

Flask(day5 9_TBL_on_Web.py 방식) + rclpy 를 한 프로세스에서: rclpy 는 별도 스레드에서 spin, Flask 는 메인 스레드.
"""
import os
import threading

import rclpy
from rclpy.node import Node
from flask import Flask, render_template, request, jsonify
from ament_index_python.packages import get_package_share_directory
from mro_interfaces.msg import FleetStatus, Intervention
from mro_interfaces.srv import RequestMission, OperatorCommand


class ControlUI(Node):
    def __init__(self):
        super().__init__('control_ui')
        self.status = {}
        self.create_subscription(FleetStatus, '/fleet/status', lambda m: self.status.__setitem__(m.robot_ns, {
            'state': m.state, 'battery': m.battery, 'x': m.x, 'y': m.y, 'mission': m.mission_id,
            'worker': m.paired_worker, 'estop': m.estop}), 10)
        self.cli_req = self.create_client(RequestMission, '/fleet/request_mission')
        self.cli_op = self.create_client(OperatorCommand, '/fleet/operator_command')
        self.pub_int = self.create_publisher(Intervention, '/fleet/intervention', 10)

    def call(self, cli, req):
        if not cli.wait_for_service(timeout_sec=1.0):
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)   # 접수 표시 ≤ 2 s (BR-T05)
        return fut.result()


def main():
    rclpy.init()
    node = ControlUI()
    share = get_package_share_directory('mro_fleet')
    app = Flask(__name__, template_folder=os.path.join(share, 'templates'))

    @app.route('/')
    def index():
        return render_template('index.html', status=node.status)

    @app.route('/request', methods=['POST'])
    def request_mission():
        r = RequestMission.Request()
        r.worker_id, r.bay = request.form['worker_id'], request.form['bay']
        res = node.call(node.cli_req, r)
        return jsonify(ok=bool(res and res.accepted), message=(res.message if res else 'fleet_manager 응답 없음'))

    @app.route('/command', methods=['POST'])
    def command():
        r = OperatorCommand.Request()
        r.robot_ns, r.command = request.form['robot_ns'], request.form['command']
        res = node.call(node.cli_op, r)
        return jsonify(ok=bool(res and res.ok), message=(res.message if res else '응답 없음'))

    @app.route('/intervention', methods=['POST'])
    def intervention():
        m = Intervention()
        m.header.stamp = node.get_clock().now().to_msg()
        m.robot_ns, m.kind, m.note = request.form['robot_ns'], request.form['kind'], request.form.get('note', '')
        node.pub_int.publish(m)
        return jsonify(ok=True)

    @app.route('/status')
    def status():
        return jsonify(node.status)

    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    app.run(host='0.0.0.0', port=5000, debug=False)


if __name__ == '__main__':
    main()
