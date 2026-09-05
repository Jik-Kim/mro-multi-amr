#!/usr/bin/env python3
"""[이관] Kn1_mini/nav/goto_goal_node.py → ros2_ws/src/mro_navigation/mro_navigation/nodes/shuttle_mission_node.py
역할 변경: '웹캠 차 좌표 앞 1.5 m 로 이동' → '셔틀 미션 (S2 undock → S4 베이 waypoint → ArUco 확인 → N초 체류 → Z-05)'.
유지: 도크 초기자세, Nav2 준비 대기, undock, goToPose, 콜백 밖 실행 규칙.
TODO([주행]): ① 트리거를 webcam/detected 대신 mission/assigned(mro_interfaces/Mission) 로 ② 목표를 bringup/params/arena.yaml 의 bay waypoint → Z-05 순서로
③ 베이 도착 후 aruco/id == 요청 베이 확인 → N초 체류 → shuttle/status 발행 ④ Z-08 진입 전 /fleet/zone_token 획득, 통과 후 반납 (E-3)
⑤ 도착 시 goto_goal/arrived → shuttle/arrived 로 이름 변경 (follow_target_node 도 같이).
"""
"""
goto_goal_node.py — [Goto Goal]
  /robot3/webcam/detected (Bool) 수신 → 도킹 상태에서 (0,0) 초기자세 → Nav2 준비 확인 → 언독
  → 목표로 이동 → /robot3/goto_goal/arrived (Bool true) 발행

  목표 우선순위:
    1) /robot3/webcam/car_map_point (PointStamped, map) 가 최근 WEBCAM_POINT_MAX_AGE_S 안에 왔으면
       그 차 위치 앞 WEBCAM_STANDOFF_M 지점 (웹캠 호모그래피 → 차 근처까지 바로 감)
    2) 아니면 고정 GOAL_X
    3) 둘 다 없으면 언독만 하고 arrived

실행: python3 goto_goal_node.py --ros-args -r __ns:=/robot3 -r /tf:=tf -r /tf_static:=tf_static
수동 트리거: ros2 topic pub --times 10 -r 2 /robot3/webcam/detected std_msgs/msg/Bool "{data: true}"
"""
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from lifecycle_msgs.srv import GetState
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

from mro_perception.core.target_utils import robot_xy, standoff_goal, yaw_to_quat

ROBOT_NS = '/robot3'
EVENT_TOPIC = 'webcam/detected'        # 네임스페이스 상대 → /robot3/webcam/detected
ARRIVED_TOPIC = 'goto_goal/arrived'
GOAL_X = None                          # (x, y, yaw_deg). 웹캠 좌표가 없을 때의 고정 목표. None 이면 언독만
WEBCAM_POINT_TOPIC = 'webcam/car_map_point'   # 팀원 웹캠 노드가 호모그래피로 낸 차 위치 (map)
WEBCAM_POINT_MAX_AGE_S = 30.0          # 이보다 오래된 웹캠 좌표는 무시
WEBCAM_STANDOFF_M = 1.5                # 웹캠 좌표 앞 몇 m 를 목표로 (오차 ±30 cm 감안해 넉넉히)
# 9/5: 로봇 2대가 각자 도크에서 (0,0) 으로 초기화하면 두 map 프레임의 원점이 물리적으로 다른 점이 되어
#   구역 세마포어 폴리곤·waypoint·FleetStatus 가 전부 어긋난다. 1대로 시험할 때는 드러나지 않는다.
#   → 맵 원점은 SLAM 작성 시점의 단일 기준점으로 고정하고, 로봇은 **자신의 도크 map 좌표**로 초기화한다.
USE_DOCK_INIT = True                   # 도킹 상태에서 아래 DOCK_POSE 로 초기 pose 설정
DOCK_POSE = None                       # (x, y, yaw_rad). None 이면 ROS2 파라미터 dock_pose 에서 읽는다
                                       # 값은 mro_bringup/params/arena.yaml 의 dock_zone.pose.<robot_ns> [팀 입력 필요]
NAV_TIMEOUT_S = 120.0


class GotoGoal(Node):
    def __init__(self):
        super().__init__('goto_goal')
        ns = self.get_namespace()
        self.ns = ROBOT_NS if ns == '/' else ns
        self.navigator = TurtleBot4Navigator(namespace=self.ns)
        self.pub_arrived = self.create_publisher(Bool, ARRIVED_TOPIC, 1)
        self.create_subscription(Bool, EVENT_TOPIC, self.event_cb, 1)
        self.create_subscription(PointStamped, WEBCAM_POINT_TOPIC, self.car_cb, 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.car_point = None          # (x, y, 수신 시각)
        self.triggered = False
        self.state = 'IDLE'
        self.get_logger().info(f"대기: {self.ns}/{EVENT_TOPIC} 에 true 오면 출발 "
                               f"(웹캠 좌표 {self.ns}/{WEBCAM_POINT_TOPIC} 우선, 없으면 GOAL_X={GOAL_X})")

    def car_cb(self, msg):
        if msg.header.frame_id == 'map':
            self.car_point = (msg.point.x, msg.point.y, time.time())

    def event_cb(self, msg):
        if msg.data and self.state == 'IDLE':
            self.triggered = True          # 실제 동작은 메인 루프에서 (콜백 안에서 navigator 호출 금지)

    def nav2_ready(self):
        cli = self.create_client(GetState, f'{self.ns}/bt_navigator/get_state')
        if not cli.wait_for_service(timeout_sec=2.0):
            return False
        fut = cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        return fut.result() is not None and fut.result().current_state.label == 'active'

    def dock_pose(self):
        """자신의 도크의 map 좌표 (x, y, yaw). 두 로봇이 같은 맵 좌표계를 쓰게 하는 핵심."""
        if DOCK_POSE is not None:
            return DOCK_POSE
        p = self.get_parameter_or('dock_pose', None)
        v = list(p.value) if p is not None and p.value else []
        if len(v) == 3:
            return float(v[0]), float(v[1]), float(v[2])
        self.get_logger().error(
            'dock_pose 파라미터가 없다 — (0,0,0) 으로 대체한다. '
            '로봇 2대를 동시에 쓰면 맵 좌표계가 어긋난다. arena.yaml 의 dock_zone.pose 를 채울 것')
        return 0.0, 0.0, 0.0

    def run_sequence(self):
        self.state = 'INIT'
        if USE_DOCK_INIT:
            if not self.navigator.getDockedStatus():
                self.get_logger().info('초기자세 설정을 위해 도킹')
                self.navigator.dock()
            x, y, yaw = self.dock_pose()
            pose = self.navigator.getPoseStamped([x, y], TurtleBot4Directions.NORTH)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.navigator.setInitialPose(pose)
            self.get_logger().info(f'초기자세 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°) 전송 — 자신의 도크 map 좌표')
        while not self.nav2_ready():
            self.get_logger().warn('Nav2(bt_navigator) 미준비 — 복구 스크립트 Step 1 후 자동 진행')
            time.sleep(5.0)
        self.get_logger().info('언독')
        self.navigator.undock()

        pose, label = self.pick_goal()
        if pose is not None:
            self.state = 'NAV_TO_X'
            self.get_logger().info(f'{label} 로 이동')
            if not self.navigator.goToPose(pose):
                return self.finish(False, 'X 목표 거부됨')
            t0 = time.time()
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)
                if time.time() - t0 > NAV_TIMEOUT_S:
                    self.navigator.cancelTask()
                    return self.finish(False, 'X 이동 타임아웃')
            if self.navigator.getResult().name != 'SUCCEEDED':
                return self.finish(False, f'X 이동 실패: {self.navigator.getResult().name}')
        self.finish(True, '목표 도착' if pose is not None else '언독 완료 (목표 없음)')

    def pick_goal(self):
        """(PoseStamped 또는 None, 설명)"""
        if self.car_point is not None and time.time() - self.car_point[2] <= WEBCAM_POINT_MAX_AGE_S:
            cx, cy, _ = self.car_point
            rob = robot_xy(self.tf_buffer, self.get_logger()) or (0.0, 0.0)
            goal, dist, yaw = standoff_goal(*rob, cx, cy, WEBCAM_STANDOFF_M, self.get_clock().now().to_msg())
            return goal, (f'웹캠 차 위치 ({cx:.2f},{cy:.2f}) {dist:.2f} m → 앞 {WEBCAM_STANDOFF_M} m '
                          f'({goal.pose.position.x:.2f},{goal.pose.position.y:.2f},{math.degrees(yaw):.0f}°)')
        if GOAL_X is not None:
            x, y, yaw = GOAL_X
            pose = self.navigator.getPoseStamped([x, y], TurtleBot4Directions.NORTH)
            pose.pose.orientation = yaw_to_quat(math.radians(yaw))
            return pose, f'고정 X ({x:.2f}, {y:.2f}, {yaw:.0f}°)'
        return None, ''

    def finish(self, ok, msg):
        self.state = 'DONE' if ok else 'FAILED'
        self.get_logger().info(f'[{self.state}] {msg}')
        self.pub_arrived.publish(Bool(data=ok))


def main():
    rclpy.init()
    node = GotoGoal()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.triggered:
                node.triggered = False
                node.run_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()      # Ctrl+C 시 이미 내려간 경우 중복 호출 방지


if __name__ == '__main__':
    main()
