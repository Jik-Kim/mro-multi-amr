"""KPI 로거 — 상태 전이·미션·개입·추종거리·힌트를 ROS 시각 기준 CSV 한 파일에 기록.

관제 PC(PC-F)에서 실행한다 — fleet 토픽을 가장 많이 구독하므로 (fleet_pc.launch.py, 9/5 결정).
후처리(추종거리 p95, IPH, 리드타임, 성공률)는 tools/kpi_report.py.
영상 원본은 세션 판정 후 폐기 (R-10).

계약 (docs/interfaces.md)
  - 모든 시각은 ROS 시각. 벽시계를 섞지 않는다 — 로봇·웹캠·관제 로그를 합쳐야 하므로
  - target/distance 가 추종거리 판정의 기준값이다 (FR-29). 좌표로 역산하지 않는다
  - QoS: 명령·상태·미션은 RELIABLE, 카메라·scan·힌트는 BEST_EFFORT.
    구독을 잘못 잡으면 발행자와 안 맞아 해당 항목이 통째로 빈다

CSV 열
  t_ros, source, robot, key, value, x, y, extra
  구역 이벤트는 source=zone, robot=로봇 네임스페이스, key=zone_id, value=REJECT|GRANT|RELEASE|RECLAIM
"""
import csv
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Bool, Float32
from mro_interfaces.msg import TargetHint, Mission, Intervention, FleetStatus

# 명단의 단일 출처는 mro_bringup/params/fleet.yaml 의 robot_namespaces 다 (fleet_pc.launch.py 가 넘긴다).
#   여기 값은 런치 없이 노드만 띄웠을 때의 대체값이다. 목록에 없는 로봇은 KPI 에 한 줄도 안 남는다
ROBOT_NAMESPACES = ['/robot3', '/robot8']


class KpiLogger(Node):
    def __init__(self):
        super().__init__('kpi_logger')
        self.declare_parameter('out_dir', os.path.expanduser('~/mro_kpi'))
        self.declare_parameter('robot_namespaces', ROBOT_NAMESPACES)
        out = self.get_parameter('out_dir').value
        robots = list(self.get_parameter('robot_namespaces').value)

        os.makedirs(out, exist_ok=True)
        self.path = os.path.join(out, time.strftime('kpi_%Y%m%d_%H%M%S.csv'))
        self.f = open(self.path, 'w', newline='', encoding='utf-8')
        self.wr = csv.writer(self.f)
        self.wr.writerow(['t_ros', 'source', 'robot', 'key', 'value', 'x', 'y', 'extra'])

        for ns in robots:
            # 상태 전이 — RELIABLE. 놓치면 리드타임·Lost 구간 계산이 어긋난다
            self.create_subscription(String, f'{ns}/approach/status',
                                     lambda m, ns=ns: self.row('status', ns, 'approach', m.data), 10)
            self.create_subscription(String, f'{ns}/shuttle/status',
                                     lambda m, ns=ns: self.row('status', ns, 'shuttle', m.data), 10)
            self.create_subscription(Bool, f'{ns}/safety/estop',
                                     lambda m, ns=ns: self.row('estop', ns, 'safety', int(m.data)), 10)
            # 검출 결과 — 발행 측이 sensor_data 일 수 있어 BEST_EFFORT 로 받는다.
            # BEST_EFFORT 구독자는 RELIABLE 발행자도 받을 수 있으나 그 반대는 성립하지 않는다
            self.create_subscription(Float32, f'{ns}/target/distance',
                                     lambda m, ns=ns: self.row('distance', ns, 'target', f'{m.data:.3f}'),
                                     qos_profile_sensor_data)   # FR-29 추종거리 판정 기준값
            self.create_subscription(Bool, f'{ns}/target/visible',
                                     lambda m, ns=ns: self.row('visible', ns, 'target', int(m.data)),
                                     qos_profile_sensor_data)

        # 힌트 — 계약상 BEST_EFFORT (interfaces.md QoS 절)
        self.create_subscription(TargetHint, '/webcam2/target_hint',
                                 lambda m: self.row('hint', '', m.worker_id, m.confidence,
                                                    m.point.point.x, m.point.point.y),
                                 qos_profile_sensor_data)
        # 미션·개입·집계 상태 — RELIABLE
        self.create_subscription(Mission, '/fleet/mission',
                                 lambda m: self.row('mission', m.robot_ns, m.mission_id, m.state, extra=m.bay), 10)
        self.create_subscription(Intervention, '/fleet/intervention',
                                 lambda m: self.row('intervention', m.robot_ns, m.kind, 1, extra=m.note), 10)
        self.create_subscription(FleetStatus, '/fleet/status',
                                 lambda m: self.row('fleet', m.robot_ns, m.state, m.battery, m.x, m.y), 10)
        # 구역 토큰 이벤트 — K-06 원천 (계약 v1.11, I-030). "<zone> <robot> <REJECT|GRANT|RELEASE|RECLAIM>"
        # 개입이 아니다. Intervention 과 섞으면 IPH 분자가 부풀어 AC-13 이 실제보다 나빠 보인다
        self.create_subscription(String, '/fleet/zone_event', self.zone_cb, 10)

        self.get_logger().info(f'KPI 로그 → {self.path} | 로봇 {robots}')

    def zone_cb(self, message):
        """세마포어 이벤트 한 줄을 쪼개 CSV 에 적는다. 형식이 어긋나면 원문을 남긴다.

        판정하지 않는다 — K-06 집계는 tools/kpi_report.py 가 한다 (계약 9.2).
        """
        parts = message.data.split()
        if len(parts) == 3:
            zone, robot, action = parts
            self.row('zone', robot, zone, action)
        else:
            # 발행 측 형식이 바뀌면 여기서 조용히 버리지 않고 원문을 남겨 추적할 수 있게 한다
            self.row('zone', '', '?', '?', extra=message.data)
            self.get_logger().warn(f'zone_event 형식이 계약과 다르다: {message.data!r}',
                                   throttle_duration_sec=10.0)

    def row(self, source, robot, key, value, x='', y='', extra=''):
        t = self.get_clock().now().nanoseconds / 1e9
        self.wr.writerow([f'{t:.3f}', source, robot, key, value, x, y, extra])
        self.f.flush()


def main():
    rclpy.init()
    node = KpiLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
