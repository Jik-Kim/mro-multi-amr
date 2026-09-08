"""자재 요청, 상태 표시, 작업 종료를 제공하는 관제 UI 노드.

rclpy executor는 별도 스레드에서 돌고 Flask는 메인 스레드에서 돈다.
"""
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory

from flask import Flask, jsonify, render_template, request, send_file

from mro_interfaces.msg import FleetStatus, Intervention
from mro_interfaces.srv import ManualPair, OperatorCommand, RequestMission

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

import yaml


DEFAULT_ROBOTS = ['/robot3', '/robot8']
PAIR_STATES = {'PAIRED', 'GATE_FAIL', 'NO_TARGET'}


def _default_map_yaml_path():
    """통합 워크스페이스에 설치된 기본 아레나 맵을 찾는다."""
    try:
        share = get_package_share_directory('mro_bringup')
    except Exception:  # 설치 전에는 빈 값으로 UI를 계속 실행한다.
        return ''
    return os.path.join(share, 'maps', 'auto_map.yaml')


def _default_arena_yaml_path():
    """통합 워크스페이스에 설치된 기본 아레나 설정을 찾는다."""
    try:
        share = get_package_share_directory('mro_bringup')
    except Exception:  # 설치 전에는 빈 값으로 UI를 계속 실행한다.
        return ''
    return os.path.join(share, 'params', 'arena.yaml')


def _pgm_size(path):
    """P2/P5 PGM 헤더에서 이미지 크기만 읽는다."""
    with open(path, 'rb') as stream:
        tokens = []
        while len(tokens) < 4:
            line = stream.readline()
            if not line:
                break
            line = line.split(b'#', 1)[0]
            tokens.extend(line.split())
    if len(tokens) < 4 or tokens[0] not in {b'P2', b'P5'}:
        raise ValueError('지원하지 않는 PGM 형식')
    width, height = int(tokens[1]), int(tokens[2])
    if width <= 0 or height <= 0:
        raise ValueError('PGM 크기는 양수여야 함')
    return width, height


def load_map_config(yaml_path):
    """Nav2 맵 YAML과 PGM을 검증해 웹용 메타데이터로 바꾼다."""
    if not yaml_path:
        raise ValueError('map_yaml_path가 비어 있음')
    yaml_path = os.path.realpath(os.path.expanduser(yaml_path))
    with open(yaml_path, encoding='utf-8') as stream:
        source = yaml.safe_load(stream) or {}
    image = source.get('image', '')
    image_path = os.path.realpath(
        image if os.path.isabs(image)
        else os.path.join(os.path.dirname(yaml_path), image)
    )
    resolution = float(source['resolution'])
    origin = [float(value) for value in source['origin']]
    if resolution <= 0.0 or len(origin) != 3:
        raise ValueError('맵 resolution 또는 origin 형식 오류')
    width, height = _pgm_size(image_path)
    return {
        'yaml_path': yaml_path,
        'image_path': image_path,
        'width': width,
        'height': height,
        'resolution': resolution,
        'origin': origin,
        'negate': int(source.get('negate', 0)),
        'occupied_thresh': float(source.get('occupied_thresh', 0.65)),
        'free_thresh': float(source.get('free_thresh', 0.196)),
    }


def load_dock_landmarks(yaml_path):
    """arena.yaml에서 로봇별 도킹 기준 자세를 읽는다."""
    if not yaml_path:
        raise ValueError('arena_yaml_path가 비어 있음')
    with open(
        os.path.realpath(os.path.expanduser(yaml_path)), encoding='utf-8'
    ) as stream:
        source = yaml.safe_load(stream) or {}
    poses = source['arena']['dock_zone']['pose']
    landmarks = []
    for robot, values in poses.items():
        pose = [float(value) for value in values]
        if len(pose) != 3:
            raise ValueError(f'{robot} dock pose 형식 오류')
        landmarks.append({
            'robot': str(robot), 'x': pose[0], 'y': pose[1], 'yaw': pose[2]
        })
    return landmarks


class ControlUI(Node):
    """ROS 서비스와 Flask 화면을 연결하는 노드."""

    def __init__(self):
        """상태 구독자, 서비스 클라이언트와 공유 상태를 만든다."""
        super().__init__('control_ui')
        self.declare_parameter('robot_namespaces', DEFAULT_ROBOTS)
        self.declare_parameter('manual_pair_timeout_s', 3.0)
        self.declare_parameter('status_stale_s', 2.5)
        self.declare_parameter('map_yaml_path', _default_map_yaml_path())
        self.declare_parameter('arena_yaml_path', _default_arena_yaml_path())
        self.robots = list(self.get_parameter('robot_namespaces').value)
        self.manual_pair_timeout_s = float(
            self.get_parameter('manual_pair_timeout_s').value
        )
        self.status_stale_s = float(
            self.get_parameter('status_stale_s').value
        )
        if self.status_stale_s <= 0.0:
            raise ValueError('status_stale_s must be positive')
        self.map_config = None
        self.map_error = ''
        try:
            self.map_config = load_map_config(
                str(self.get_parameter('map_yaml_path').value)
            )
        except (
            KeyError, OSError, TypeError, ValueError, yaml.YAMLError
        ) as error:
            self.map_error = str(error)
            self.get_logger().warn(f'웹 맵을 불러오지 못함: {error}')
        self.dock_landmarks = []
        try:
            self.dock_landmarks = load_dock_landmarks(
                str(self.get_parameter('arena_yaml_path').value)
            )
        except (
            KeyError, OSError, TypeError, ValueError, yaml.YAMLError
        ) as error:
            self.get_logger().warn(f'도크 기준 좌표를 불러오지 못함: {error}')
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
            '_received_monotonic': time.monotonic(),
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
        """웹 UI에 로봇별 최신 상태와 수신 경과 시간을 반환한다."""
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

    @app.route('/map/metadata')
    def map_metadata():
        """웹 좌표 변환에 필요한 실제 맵 메타데이터를 반환한다."""
        if node.map_config is None:
            return jsonify(ok=False, message=node.map_error), 503
        return jsonify(
            ok=True,
            width=node.map_config['width'],
            height=node.map_config['height'],
            resolution=node.map_config['resolution'],
            origin=node.map_config['origin'],
            negate=node.map_config['negate'],
            occupied_thresh=node.map_config['occupied_thresh'],
            free_thresh=node.map_config['free_thresh'],
            image_url='/map/image',
            docks=node.dock_landmarks,
        )

    @app.route('/map/image')
    def map_image():
        """브라우저 canvas가 읽을 실제 PGM 파일을 반환한다."""
        if node.map_config is None:
            return jsonify(ok=False, message=node.map_error), 503
        return send_file(
            node.map_config['image_path'],
            mimetype='image/x-portable-graymap',
            conditional=True,
        )

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
