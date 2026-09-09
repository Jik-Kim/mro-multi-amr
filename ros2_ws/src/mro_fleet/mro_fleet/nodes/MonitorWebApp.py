"""관제 화면·HTTP API를 ROS 로봇 통신 노드와 연결한다."""
import os

from flask import Flask, jsonify, render_template, request, send_file

import yaml


def _pgm_size(path):
    """P2/P5 PGM 헤더에서 이미지 크기만 읽는다."""
    with open(path, 'rb') as stream:
        tokens = []
        while len(tokens) < 4:
            line = stream.readline()
            if not line:
                break
            tokens.extend(line.split(b'#', 1)[0].split())
    if len(tokens) < 4 or tokens[0] not in {b'P2', b'P5'}:
        raise ValueError('지원하지 않는 PGM 형식')
    width, height = int(tokens[1]), int(tokens[2])
    if width <= 0 or height <= 0:
        raise ValueError('PGM 크기는 양수여야 함')
    return width, height


def load_map_config(yaml_path):
    """Nav2 맵 YAML과 PGM을 검증해 웹용 메타데이터로 바꾸다."""
    if not yaml_path:
        raise ValueError('map_yaml_path가 비어 있음')
    yaml_path = os.path.realpath(os.path.expanduser(yaml_path))
    with open(yaml_path, encoding='utf-8') as stream:
        source = yaml.safe_load(stream) or {}
    image = source.get('image', '')
    image_path = (
        image if os.path.isabs(image)
        else os.path.join(os.path.dirname(yaml_path), image)
    )
    image_path = os.path.realpath(image_path)
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
    path = os.path.realpath(os.path.expanduser(yaml_path))
    with open(path, encoding='utf-8') as stream:
        source = yaml.safe_load(stream) or {}
    landmarks = []
    for robot, values in source['arena']['dock_zone']['pose'].items():
        pose = [float(value) for value in values]
        if len(pose) != 3:
            raise ValueError(f'{robot} dock pose 형식 오류')
        landmarks.append({
            'robot': str(robot),
            'x': pose[0],
            'y': pose[1],
            'yaw': pose[2],
        })
    return landmarks


class MonitorWebApp:
    """관제 화면과 HTTP API를 제공하는 웹 파트.

    ROS 통신은 직접 처리하지 않고 ``RobotControlNode``에 요청한다.
    """

    def __init__(self, robot_controls, template_folder):
        """로봇별 통신 노드와 Flask 템플릿 경로를 받는다."""
        self.robot_controls = {
            control.namespace: control for control in robot_controls
        }
        self.primary_control = next(iter(self.robot_controls.values()))
        self.map_config = None
        self.map_error = ''
        self.dock_landmarks = []
        self._load_map_resources()
        self.app = Flask(__name__, template_folder=template_folder)
        self._register_routes()

    def _load_map_resources(self):
        """대시보드에 표시할 맵과 도크 기준점을 읽는다."""
        try:
            self.map_config = load_map_config(
                self.primary_control.map_yaml_path
            )
        except (
            KeyError, OSError, TypeError, ValueError, yaml.YAMLError
        ) as error:
            self.map_error = str(error)
            self.primary_control.get_logger().warn(
                f'웹 맵을 불러오지 못함: {error}'
            )
        try:
            self.dock_landmarks = load_dock_landmarks(
                self.primary_control.arena_yaml_path
            )
        except (
            KeyError, OSError, TypeError, ValueError, yaml.YAMLError
        ) as error:
            self.primary_control.get_logger().warn(
                f'도크 기준 좌표를 불러오지 못함: {error}'
            )

    def _robot_status_snapshot(self):
        """로봇별 통신 노드의 상태를 하나의 화면 응답으로 합친다."""
        snapshot = {}
        for control in self.robot_controls.values():
            snapshot.update(control._status_snapshot())
        return snapshot

    def _control_for(self, namespace):
        """요청 대상 로봇을 담당하는 ROS 통신 노드를 반환한다."""
        return self.robot_controls.get(namespace)

    def _register_routes(self):
        """브라우저 URL을 화면 기능별 메서드와 연결한다."""
        self.app.add_url_rule('/', view_func=self.show_dashboard)
        self.app.add_url_rule(
            '/request', methods=['POST'], view_func=self.request_mission
        )
        self.app.add_url_rule(
            '/command', methods=['POST'], view_func=self.request_command
        )
        self.app.add_url_rule(
            '/intervention',
            methods=['POST'],
            view_func=self.record_intervention,
        )
        self.app.add_url_rule(
            '/manual_pair',
            methods=['POST'],
            view_func=self.request_manual_pair,
        )
        self.app.add_url_rule('/status', view_func=self.get_robot_status)
        self.app.add_url_rule('/map/metadata', view_func=self.get_map_metadata)
        self.app.add_url_rule('/map/image', view_func=self.get_map_image)

    def show_dashboard(self):
        """관제 메인 화면을 보여준다."""
        return render_template(
            'index.html', status=self._robot_status_snapshot()
        )

    def request_mission(self):
        """화면의 자재 요청을 ROS 서비스로 전달한다."""
        response = self.primary_control._request_mission(
            request.form['worker_id'], request.form['bay']
        )
        return jsonify(
            ok=bool(response and response.accepted),
            mission_id=(response.mission_id if response else ''),
            message=(response.message if response else 'fleet_manager 응답 없음'),
        )

    def request_command(self):
        """화면의 작업 종료·복귀 명령을 ROS 서비스로 전달한다."""
        namespace = request.form['robot_ns']
        control = self._control_for(namespace)
        response = (
            control._request_command(request.form['command'])
            if control else None
        )
        return jsonify(
            ok=bool(response and response.ok),
            message=(response.message if response else '응답 없음'),
        )

    def record_intervention(self):
        """화면에서 발생한 운영자 개입을 ROS 토픽으로 기록한다."""
        namespace = request.form['robot_ns']
        control = self._control_for(namespace)
        if control is None:
            return jsonify(ok=False, message='알 수 없는 로봇'), 404
        control._publish_intervention(
            request.form['kind'], request.form.get('note', '')
        )
        return jsonify(ok=True)

    def request_manual_pair(self):
        """운영자의 명시 페어링 요청 결과를 보여준다."""
        namespace = request.form.get('robot_ns', '')
        control = self._control_for(namespace)
        if control is None:
            return jsonify(ok=False, message='알 수 없는 로봇'), 404
        accepted, reason = control._request_manual_pair(
            request.form.get('worker_id', '')
        )
        return jsonify(ok=accepted, message=reason)

    def get_robot_status(self):
        """로봇별 최신 상태를 JSON으로 보여준다."""
        return jsonify(self._robot_status_snapshot())

    def get_map_metadata(self):
        """맵 좌표 변환에 필요한 메타데이터와 도크 위치를 보여준다."""
        if self.map_config is None:
            return jsonify(ok=False, message=self.map_error), 503
        return jsonify(
            ok=True,
            width=self.map_config['width'],
            height=self.map_config['height'],
            resolution=self.map_config['resolution'],
            origin=self.map_config['origin'],
            negate=self.map_config['negate'],
            occupied_thresh=self.map_config['occupied_thresh'],
            free_thresh=self.map_config['free_thresh'],
            image_url='/map/image',
            docks=self.dock_landmarks,
        )

    def get_map_image(self):
        """브라우저 캔버스에 그릴 PGM 맵 파일을 보여준다."""
        if self.map_config is None:
            return jsonify(ok=False, message=self.map_error), 503
        return send_file(
            self.map_config['image_path'],
            mimetype='image/x-portable-graymap',
            conditional=True,
        )

    def run(self):
        """관제 웹 서버를 실행한다."""
        self.app.run(host='0.0.0.0', port=5000, debug=False)
