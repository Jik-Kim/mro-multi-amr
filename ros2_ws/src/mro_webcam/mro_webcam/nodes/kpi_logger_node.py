"""KPI 로거 — 상태 전이·미션·개입·웹캠2 좌표를 ROS 시각 기준 CSV 한 파일에 기록. BRD v3.0 4.1 / BR-T13 / BR-T26.

후처리(추종거리 p95, IPH, 성공률)는 tools/kpi_report.py (TODO). 영상 원본은 세션 판정 후 폐기 (R-10).
"""
import csv
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Bool
from mro_interfaces.msg import TargetHint, Mission, Intervention, FleetStatus

ROBOTS = ['/robot3', '/robot4']   # TODO: 파라미터화 (bringup params)


class KpiLogger(Node):
    def __init__(self):
        super().__init__('kpi_logger')
        self.declare_parameter('out_dir', os.path.expanduser('~/mro_kpi'))
        out = self.get_parameter('out_dir').value
        os.makedirs(out, exist_ok=True)
        self.path = os.path.join(out, time.strftime('kpi_%Y%m%d_%H%M%S.csv'))
        self.f = open(self.path, 'w', newline='', encoding='utf-8')
        self.wr = csv.writer(self.f)
        self.wr.writerow(['t_ros', 'source', 'robot', 'key', 'value', 'x', 'y', 'extra'])
        for ns in ROBOTS:
            self.create_subscription(String, f'{ns}/approach/status', lambda m, ns=ns: self.row('status', ns, 'approach', m.data), 10)
            self.create_subscription(String, f'{ns}/shuttle/status', lambda m, ns=ns: self.row('status', ns, 'shuttle', m.data), 10)
            self.create_subscription(Bool, f'{ns}/target/visible', lambda m, ns=ns: self.row('visible', ns, 'target', int(m.data)), 10)
            self.create_subscription(Bool, f'{ns}/safety/estop', lambda m, ns=ns: self.row('estop', ns, 'safety', int(m.data)), 10)
        self.create_subscription(TargetHint, '/webcam2/target_hint',
                                 lambda m: self.row('hint', '', m.worker_id, m.confidence, m.point.point.x, m.point.point.y),
                                 qos_profile_sensor_data)
        self.create_subscription(Mission, '/fleet/mission', lambda m: self.row('mission', m.robot_ns, m.mission_id, m.state, extra=m.bay), 10)
        self.create_subscription(Intervention, '/fleet/intervention', lambda m: self.row('intervention', m.robot_ns, m.kind, 1, extra=m.note), 10)
        self.create_subscription(FleetStatus, '/fleet/status', lambda m: self.row('fleet', m.robot_ns, m.state, m.battery, m.x, m.y), 10)
        self.get_logger().info(f'KPI 로그 → {self.path}')

    def row(self, source, robot, key, value, x='', y='', extra=''):
        t = self.get_clock().now().nanoseconds / 1e9
        self.wr.writerow([f'{t:.3f}', source, robot, key, value, x, y, extra])
        self.f.flush()


def main():
    rclpy.init()
    node = KpiLogger()
    try:
        rclpy.spin(node)
    finally:
        node.f.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
