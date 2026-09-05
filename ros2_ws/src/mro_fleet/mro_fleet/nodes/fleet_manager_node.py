"""fleet_manager — 미션 생성·배정, 로봇 상태 집계(≤ 2 s), 운영자 명령, 배터리 임계 배정 거부. BRD v3.0 BR-T05/T08/T10/T14.

서비스: /fleet/request_mission, /fleet/operator_command
구독:   /webcam1/worker_descriptor, /robotN/approach/status, /robotN/shuttle/status, /robotN/battery_state, /robotN/amcl_pose, /robotN/safety/estop
발행:   /fleet/mission (Mission), /robotN/mission/assigned (Mission), /robotN/mission/finish (Bool), /fleet/status (FleetStatus, 1 Hz)
"""
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseWithCovarianceStamped
from ament_index_python.packages import get_package_share_directory
from mro_interfaces.msg import Mission, FleetStatus, WorkerDescriptor
from mro_interfaces.srv import RequestMission, OperatorCommand

from mro_fleet.core.db import FleetDB

ROBOTS = ['/robot3', '/robot4']       # TODO: 파라미터화
BATTERY_MIN = 0.25                    # BR-T14 임계 [팀 가정]


class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')
        share = get_package_share_directory('mro_fleet')
        self.declare_parameter('db_path', os.path.expanduser('~/mro_fleet.db'))
        self.db = FleetDB(self.get_parameter('db_path').value, os.path.join(share, 'config', 'schema.sql'))
        self.robot = {ns: {'state': 'UNKNOWN', 'battery': 1.0, 'x': 0.0, 'y': 0.0, 'yaw': 0.0,
                           'mission_id': '', 'paired_worker': '', 'estop': False} for ns in ROBOTS}
        self.worker_robot = {}        # worker_id -> robot_ns (등록 순서대로 배정)
        self.pub_mission = self.create_publisher(Mission, '/fleet/mission', 10)
        self.pub_assigned = {ns: self.create_publisher(Mission, f'{ns}/mission/assigned', 10) for ns in ROBOTS}
        self.pub_finish = {ns: self.create_publisher(Bool, f'{ns}/mission/finish', 10) for ns in ROBOTS}
        self.pub_status = self.create_publisher(FleetStatus, '/fleet/status', 10)
        self.create_subscription(WorkerDescriptor, '/webcam1/worker_descriptor', self.worker_cb, 10)
        for ns in ROBOTS:
            self.create_subscription(String, f'{ns}/approach/status', lambda m, ns=ns: self.set(ns, 'state', m.data), 10)
            self.create_subscription(String, f'{ns}/shuttle/status', lambda m, ns=ns: self.set(ns, 'state', m.data), 10)
            self.create_subscription(BatteryState, f'{ns}/battery_state', lambda m, ns=ns: self.set(ns, 'battery', m.percentage), 10)
            self.create_subscription(PoseWithCovarianceStamped, f'{ns}/amcl_pose', lambda m, ns=ns: self.pose_cb(ns, m), 10)
            self.create_subscription(Bool, f'{ns}/safety/estop', lambda m, ns=ns: self.set(ns, 'estop', m.data), 10)
        self.create_service(RequestMission, '/fleet/request_mission', self.request_cb)
        self.create_service(OperatorCommand, '/fleet/operator_command', self.operator_cb)
        self.create_timer(1.0, self.publish_status)

    # ---- 상태 집계
    def set(self, ns, key, val):
        if self.robot[ns].get(key) != val:
            self.db.event(ns, 'STATE' if key == 'state' else key.upper(), val)
        self.robot[ns][key] = val

    def pose_cb(self, ns, m):
        p = m.pose.pose
        self.robot[ns]['x'], self.robot[ns]['y'] = p.position.x, p.position.y
        # TODO: quaternion → yaw

    def publish_status(self):
        for ns, r in self.robot.items():
            s = FleetStatus()
            s.header.stamp = self.get_clock().now().to_msg()
            s.robot_ns = ns
            s.state, s.battery, s.x, s.y, s.yaw = r['state'], float(r['battery']), r['x'], r['y'], r['yaw']
            s.mission_id, s.paired_worker, s.estop = r['mission_id'], r['paired_worker'], bool(r['estop'])
            self.pub_status.publish(s)

    # ---- S1 등록 → 로봇 배정 (S2 undock 트리거는 shuttle_mission_node 가 mission/assigned 로 받는다)
    def worker_cb(self, d):
        if d.worker_id not in self.worker_robot:
            free = [ns for ns in ROBOTS if ns not in self.worker_robot.values()]
            if not free:
                self.get_logger().warn(f'{d.worker_id}: 배정 가능한 로봇 없음')
                return
            self.worker_robot[d.worker_id] = free[0]
            self.db.upsert_worker(d.worker_id, d.outfit_class, free[0])
            self.get_logger().info(f'{d.worker_id}({d.outfit_class}) → {free[0]}')

    # ---- S3 요청 → 미션 생성·배정
    def request_cb(self, req, res):
        ns = self.worker_robot.get(req.worker_id)
        if ns is None:
            res.accepted, res.message = False, '미등록 작업자'
            return res
        if self.robot[ns]['battery'] < BATTERY_MIN:
            res.accepted, res.message = False, f'{ns} 배터리 임계 미만 — 배정 거부 (BR-T14)'
            self.db.event(ns, 'ASSIGN_REFUSED_BATTERY', self.robot[ns]['battery'])
            return res
        mid = self.db.create_mission(req.worker_id, req.bay, ns)
        m = Mission()
        m.header.stamp = self.get_clock().now().to_msg()
        m.mission_id, m.worker_id, m.robot_ns, m.bay, m.state = mid, req.worker_id, ns, req.bay, Mission.ASSIGNED
        self.robot[ns]['mission_id'] = mid
        self.db.set_state(mid, Mission.ASSIGNED)
        self.pub_mission.publish(m)
        self.pub_assigned[ns].publish(m)
        res.accepted, res.mission_id, res.message = True, mid, f'{ns} 배정'
        return res

    # ---- 운영자 명령
    def operator_cb(self, req, res):
        if req.robot_ns not in self.robot:
            res.ok, res.message = False, '알 수 없는 로봇'
            return res
        if req.command not in ('RETURN', 'RETURN_ALL'):
            # 9/4 결정: 관제는 단순 뷰어. 정지는 온보드 safety_watchdog·물리 E-stop 전담
            res.ok, res.message = False, f'{req.command} 는 지원하지 않음 (RETURN / RETURN_ALL 만 허용)'
            return res
        # S9 종료 시퀀스 발행. 두 대가 동시에 복귀해도 Z-09·Z-03 구역 토큰이 순서를 만든다 (S10, BR-T31)
        targets = list(ROBOTS) if req.command == 'RETURN_ALL' else [req.robot_ns]
        for ns in targets:
            self.db.event(ns, 'OPERATOR', req.command)
            self.pub_finish[ns].publish(Bool(data=True))
        res.ok, res.message = True, f"{', '.join(targets)} 작업 종료 → 복귀 전달"
        return res


def main():
    rclpy.init()
    node = FleetManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
