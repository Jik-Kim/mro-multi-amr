#!/usr/bin/env python3
"""[이관] Kn1_mini/nav/goto_goal_node.py → mro_navigation/nodes/shuttle_mission_node.py.

역할 변경: '웹캠 차 좌표 앞 1.5 m 로 이동' → 'S4 셔틀 미션'.
  **기동 시** 도크 초기자세 → (미션 대기) → mission/assigned 수신 → Nav2 준비 → undock
  → 베이 waypoint → ArUco 로 도착 확정(못 보면 제자리 회전 재탐지) → **마커까지 접근·정렬**
  → 적재 체류 → P1 토큰 → 대기공간 → WAITING

기동 순서가 중요하다 (I-035) — **이 노드를 Nav2(T3) 보다 먼저 띄운다.** Nav2 의 global_costmap 은
  기동할 때 map→base_link TF 를 요구하고, AMCL 은 초기자세를 받아야 map→odom 을 낸다. 초기자세가
  늦으면 costmap 활성화가 실패하는데 lifecycle_manager 는 **나중에 TF 가 생겨도 재시도하지 않는다**
  (9/7 실측). 그래서 초기자세를 미션이 아니라 기동 시에 보낸다.

유지한 골격: 도크 초기자세(dock_pose), Nav2 준비 폴링, undock, goToPose 폴링,
  그리고 **콜백 안에서 navigator 를 부르지 않는다** — 콜백은 값만 저장하고 메인 루프가 실행한다.
폐기한 것: webcam/detected 트리거, webcam/car_map_point 목표, goto_goal/arrived
  (계약 4절 '쓰지 않는 옛 토픽'), standoff_goal 기반 목표 선택.

**베이 접근·정렬 (9/8 추가) — '창고 안쪽 0.3 m 전진' 을 대체한다.**
  옛 절차는 두 단계였다: waypoint 에 도착한 뒤 그 자리에서 앞으로 0.3 m. 앞의 0.3 m 는 **출발점을
  모르는 상대량**이라 AMCL 이 0.4 m 틀리면 최종 정차 위치도 그대로 0.4 m 틀렸다 (실측 0.38~0.53 m).
  지금은 한 단계다: **마커에서 bay_marker_range_m(0.9 m) 인 자리로 간다.** 목적지가 마커 기준
  절대 위치라 출발점이 어디든 도착점이 같다 — `aruco/pose` 는 카메라→마커의 상대 측정이라
  AMCL 오차와 무관하기 때문이다. 그래서 별도의 전진 단계가 필요 없다.

**마커 재탐지 (9/8 추가)** — 베이에서 마커를 못 보면 **제자리 회전만으로** 좌우를 훑는다.
  기준 방위에서 가까운 쪽부터 좌우 교대로(−Δ, +Δ, −2Δ, +2Δ …) 돌며 정지점마다 확인한다.
  · **병진하지 않는다.** 마커를 못 본다는 것은 자기 위치를 못 믿는다는 뜻이라 어디로 갈지
    모른다. 제자리 회전이 footprint 밖으로 쓸고 가는 폭은 내접 0.175 / 외접 0.189 라 1.4 cm
    뿐이다 (safety_watchdog 과 같은 근거).
  · 스텝은 카메라 수평 FOV 보다 확실히 작아야 정지점 **사이로 마커가 빠지지 않는다.**
  · 옛 '정차 자세 재정렬'(같은 waypoint 로 goToPose 재전송)을 대체한다 — Nav2 허용오차
    0.25 m 안이면 로봇이 아예 안 움직여 사실상 무동작이었다.
  · 찾으면 **원래 방위로 되돌리지 않는다.** 접근 단계가 어차피 측량 베이 방위로 돌려세운다.
  · **Nav2 목표로는 못 한다.** SimpleGoalChecker 가 xy 0.25 m · yaw 0.25 rad 라 12 cm 보정 목표를
    보내면 로봇이 움직이지 않고 곧장 SUCCEEDED 가 난다 (nav2.yaml 9/8 확인). 그래서 보정과 전진은
    behavior_server 의 Spin·DriveOnHeading(상대 이동, 허용오차 없음) 으로 실행한다.
  · 차동구동이라 옆으로 못 간다 → **회전–직진–회전–직진**. 직진은 항상 전진이다 (후진 금지, 계약 2절).
  · **기준은 마커 법선선이다 (9/10).** 마커를 지나고 법선으로 뻗는 직선 L(s)=p+s·n 위로 먼저
    옮기고(횡정렬), 마커를 정면으로 보고(방위정렬), 법선방향 거리를 맞춘다(거리정렬).
    세 목적이 독립이라 각각 측정·판정된다 — 예전에는 반경 이동 하나에 뭉뚱그려 어느 것이
    안 맞는지 로그로 갈리지 않았다.
  · **AMCL 방위 오차가 지워진다.** 각도 기준이 측량값(AMCL 유래)이 아니라 마커라서다.
    측량값은 대조용으로만 남기고 어긋나면 경고한다 — 어느 쪽이 틀렸는지 그 비교로는 못 가린다.
  · **계약 v1.13 은 「aruco/pose 의 회전(yaw)은 정렬에 쓰지 않는다」고 적고 있다.** 이 노드는
    9/10 부터 쓴다. 근거는 R0·R1 실측이다 — 기울기 잡음 σ≈2°, 회차 안 α 일치 0.1°.
    계약 개정을 PM 에 요청했다. 방어는 표본 산포(MAD)와 횡이동 상한 두 가지다.

입력  mission/assigned(Mission) · aruco/id(Int32) · aruco/distance(Float32) · aruco/pose(PoseStamped)
      safety/estop(Bool) · TF map→base_link · TF base_link→카메라 광학 프레임
      arena.yaml(베이 waypoint·aruco_id, 대기공간) · robotN.yaml(dock_pose 등)

출력  shuttle/status(String: READY/SHUTTLE/AT_BAY/WAITING/FAILED) · shuttle/arrived(Bool)

협조  /fleet/zone_token 의 P1 토큰 — 베이를 떠나기 전 획득, 대기공간 도착 후 반납 (계약 3절)
      **순서 기반 판정 (방법 B, 9/7 팀 회의).** 경로가 베이 → P1 → 대기공간으로 고정돼 있으므로
      좌표를 매 tick 폴리곤과 대조하지 않는다. 획득 시점(베이)과 반납 시점(대기공간)이 둘 다
      폴리곤 밖이면 '진입 전 획득·이탈 후 반납' 이 성립한다 — 그 전제는 arena.yaml 좌표로
      test_shuttle_mission_fsm.py 가 확인한다. 좌표 감시(방법 A)는 계산이 복잡하고 경로가
      폴리곤 가장자리를 스치면 진입을 놓쳐 버그가 나기 쉬워 폐기했다

판정 관련 규칙 (어기면 KPI 가 조용히 틀어진다)
  · shuttle/status = WAITING 은 **대기공간 도착이 확정된 시점에 한 번만**. fleet_manager 가
    이것으로 mark_waiting() 을 불러 AC-08 배송 성공/실패와 K-05 리드타임을 확정한다 (I-024).
    베이에서 P1 토큰을 기다리는 동안에는 AT_BAY 를 유지한다 — 로봇이 아직 창고에 있기 때문이다.
  · FAILED 는 **포기할 때만** 낸다. 재시도 중에는 직전 상태를 유지한다 (I-026).
  · T_lim 초과로 포기한 미션은 폐기한다 — 그 뒤 늦게 도착해도 WAITING·arrived 를 내지 않는다
    (계약 7절, 이중 계상 금지). 늦은 도착은 관제가 arrived_late 로 따로 남긴다.

TODO([주행]): ① 파라미터 7개와 arena_yaml 경로는 PM 이 robotN.yaml·robot.launch.py 로 옮겼다
  (PR #36 후속, 9/8). **I-035 로 추가한 nav2_poll_max·dock_status_wait_s 두 개는 아직 코드 기본값뿐이라
  robotN.yaml 이관을 PM 에 요청해야 한다** — 값의 단일 출처는 params 다 (계약 5절). 런타임 share 조회는
  런치가 안 넘겨줄 때의 대체값으로만 남아 있다 (의존을 선언하면 순환이라 막을 수 없다)
② follow_target_node.py 가 아직 옛 goto_goal/arrived 를 구독한다. shuttle/arrived 로 바꾸지 않으면
  S5 합류가 시작되지 않는다 (docs/todo.md 에 등록)
③ ArUco '정면 ±30°'(계약 6절)는 **기계로 검사하지 않기로 확정됐다** — 기운각이 두 해 문제로
  흔들려(1.0 m 에서 −30° 가 33° 로) 게이트로 쓸 수 없다. 시험 절차에서 사람이 지킨다.
  거리는 다르다: aruco_detector 의 `max_distance_m` 은 **2.0 m** 이고(1.5 로 두면 정확히 1.5 m 에
  둔 마커가 1.52 로 걸러진다, I-031), 계약 6절의 검출 조건 1.5 m 는 **소비자인 우리가** 자른다 —
  아래 aruco_max_dist_m 는 이중 방어가 아니라 **주 게이트**다
④ 대기공간 정차 중에도 로봇 몸통이 P1 폴리곤에 살짝 걸친다 — robot3 정차점이 폴리곤에서
  0.20 m 인데 반지름은 0.22 m 다. 반납 시점이 계약상 '완전 이탈' 인지 실주행에서 확인할 것
  (2대 시험 때 다른 로봇이 P1 을 지날 수 있는지가 실제 판단 기준)
⑤ navigator.undock()/dock() 은 블로킹이라 그 동안 이 노드의 콜백·T_lim 검사가 멈춘다
  (수 초 규모라 그대로 둔다)
"""
from collections import deque
from dataclasses import dataclass
import math
import os
from statistics import median
from threading import Thread

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from lifecycle_msgs.srv import GetState
from mro_interfaces.msg import Mission
from mro_interfaces.srv import ZoneToken
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, Int32, String
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
import yaml

# 네임스페이스 대체값은 두지 않는다 (9/8) — ns 가 arena.yaml 의 로봇별 좌표 조회 키라
#   틀리면 남의 대기공간·도크로 간다. 아래에서 없으면 죽인다
MISSION_TOPIC = 'mission/assigned'
STATUS_TOPIC = 'shuttle/status'
ARRIVED_TOPIC = 'shuttle/arrived'
ARUCO_ID_TOPIC = 'aruco/id'
ARUCO_DIST_TOPIC = 'aruco/distance'
ARUCO_POSE_TOPIC = 'aruco/pose'
ESTOP_TOPIC = 'safety/estop'
TOKEN_SRV = '/fleet/zone_token'   # 관제 PC 의 zone_semaphore. 전역 이름
ZONE_P1 = 'P1'                    # arena.yaml semaphore_zones · ZoneToken.srv 와 ID 일치
MAP_FRAME = 'map'
BASE_FRAME = 'base_link'
# aruco/pose 의 header.frame_id 가 비어 올 때만 쓰는 대체값 (aruco_detector 와 같은 이름)
CAMERA_FRAME_FALLBACK = 'oakd_rgb_camera_optical_frame'
TICK_S = 0.1                      # spin_once 주기


class MissionAbort(Exception):
    """미션을 포기하는 유일한 경로. 사유를 담아 FAILED 로 빠진다."""


@dataclass(frozen=True)
class PoseSample:
    """한 번의 TF 조회에서 얻은 좌표와 시각. 실측 위치가 아닌 map 추정값이다."""

    x: float
    y: float
    yaw: float
    stamp_s: float

    @property
    def pose(self):
        """목표 계산에 쓰는 좌표와 방향."""
        return self.x, self.y, self.yaw


@dataclass(frozen=True)
class MarkerSample:
    """마커를 base_link 로 옮긴 값. AMCL 과 무관한 **상대** 측정이다."""

    fwd: float          # base_link x [m] — 앞이 +
    lat: float          # base_link y [m] — 왼쪽이 +
    normal_yaw: float   # 마커 법선의 base_link 방위 [rad]. 정면이면 π (뒤에서 로봇을 본다)
    stamp_s: float
    normal_mad: float = 0.0   # 표본 창 안 법선 방위의 중앙값절대편차 [rad] — 두 해 오감을 잡는다

    @property
    def bearing(self):
        """로봇 정면에서 마커까지의 방위각 [rad]."""
        return math.atan2(self.lat, self.fwd)

    @property
    def yaw_error(self):
        """벽 법선 기준 로봇 방위 오차 [rad]. 이만큼 돌면 벽을 정면으로 본다."""
        return wrap_pi(self.normal_yaw - math.pi)

    # ── 법선선 기하 (9/10) ──────────────────────────────────────────────
    # 마커를 지나고 법선 방향으로 뻗는 직선 L(s) = p + s·n 을 기준으로 삼는다.
    # 로봇을 **먼저 이 선 위로 옮기고**(횡정렬), **마커를 정면으로 보고**(방위정렬),
    # **법선방향 거리를 맞춘다**(거리정렬). 세 목적이 서로 독립이라 각각 측정·판정된다.

    @property
    def normal(self):
        """마커에서 로봇 쪽을 향하는 단위 법선 n."""
        return math.cos(self.normal_yaw), math.sin(self.normal_yaw)

    @property
    def tangent(self):
        """법선에 수직인 단위벡터 t = (−n_y, n_x). 마커 벽과 나란하다."""
        nx, ny = self.normal
        return -ny, nx

    @property
    def lat_error(self):
        """로봇이 법선선에서 벗어난 **부호 있는** 거리 [m]. 왼쪽이 +.

        선까지의 변위는 (원점 − F)·t = −(p·t) 다. **마커 위치의 t 성분과 부호가 반대**라
        여기서 한 번만 뒤집어 두고 밖에서는 이 값을 그대로 쓴다.
        """
        tx, ty = self.tangent
        return -(self.fwd * tx + self.lat * ty)

    @property
    def normal_range(self):
        """마커 **평면**까지의 법선거리 d_n [m] = −(p·n).

        지금까지 쓰던 반경거리 hypot(fwd, lat) 와 다르다. 선반·벽까지의 여유를 정하는 것은
        반경이 아니라 이 값이다 — 옆으로 치우쳐 있으면 반경이 같아도 벽에는 더 가깝다.
        """
        nx, ny = self.normal
        return -(self.fwd * nx + self.lat * ny)

    @property
    def foot(self):
        """법선선에 내린 수선의 발 F = p + d_n·n (base_link).

        ‖F‖ = |lat_error| 이다 ({n, t} 가 정규직교이고 F ⊥ n 이므로). 그래서 횡정렬의
        직진거리를 따로 계산하지 않고 lat_error 를 쓴다.
        """
        nx, ny = self.normal
        d = self.normal_range
        return self.fwd + d * nx, self.lat + d * ny

    @property
    def face_yaw(self):
        """마커를 정면으로 보려면 향해야 할 base_link 방위 [rad] = atan2(−n_y, −n_x).

        yaw_error 와 같은 값이다(법선 방위 − π). 이름을 따로 둔 것은 쓰이는 자리가
        달라서다 — 이쪽은 '어느 쪽을 볼까', 저쪽은 '얼마나 틀어졌나'.
        """
        nx, ny = self.normal
        return math.atan2(-ny, -nx)

    @property
    def alpha(self):
        """마커 법선과 '마커→로봇' 선 사이 각 [rad] = 방위 − 법선오차.

        **제자리 회전과 반경방향 이동에 불변**이라, 회차 사이에 비교하면 법선이 튀었는지
        사후에 가릴 수 있다. 9/9 R1b 의 두 측정이 0.1° 안에서 일치했고(회차 안 자기일관),
        같은 자리에서 잰 R1a −10.8° · R1c −12.0° 와는 21° 달랐다(회차 사이 이탈).
        판정에는 쓰지 않고 로그로만 남긴다 — 표본이 쌓이면 법선을 믿을지 답이 나온다.
        """
        return wrap_pi(self.bearing - self.yaw_error)


def wrap_pi(a):
    """각을 (−π, π] 로 접는다. 보정량을 항상 짧은 쪽으로 돌리기 위한 것이다."""
    return math.atan2(math.sin(a), math.cos(a))


class ShuttleMission(Node):
    """S4 배송을 순차 실행하는 노드.

    ``mission_cb``은 배정 1건을 보관할 뿐이며, :meth:`run_mission`이 메인 루프에서
    도크 초기화·Nav2·ArUco·P1 토큰 순서를 실제로 수행한다.
    """

    def __init__(self):
        super().__init__('shuttle_mission')
        self.ns = self.get_namespace().rstrip('/')
        if not self.ns:
            raise RuntimeError(
                'shuttle_mission: 네임스페이스 없이는 뜰 수 없다 — 로봇 노드는 ns 로 자기 로봇을 정한다.\n'
                '  ros2 launch mro_bringup robot.launch.py namespace:=/robotN\n'
                '  (단독 실행이면 --ros-args -r __ns:=/robotN)\n'
                'ns 가 arena.yaml 의 waiting_zone·dock_zone 조회 키다 — 틀리면 남의 자리로 간다.')
        self.robot = self.ns.strip('/')          # arena.yaml 의 키는 'robot3' (슬래시 없음)
        self.navigator = TurtleBot4Navigator(namespace=self.ns)

        # 파라미터 — 값의 단일 출처는 mro_bringup/params (계약 5절). 여기 기본값은 params 가
        #   아직 안 넘어올 때의 대체값이며, 코드가 값을 소유하지 않는다.
        # arena.yaml 은 mro_bringup 소유지만 **의존을 선언할 수 없다** — mro_bringup 이 이미
        #   mro_navigation 을 의존해서(런치가 노드를 띄운다) 선언하면 colcon 이 순환으로 막는다.
        #   그래서 런타임 조회로만 기본값을 잡고, 못 찾으면 arena_yaml 파라미터로 받는다.
        default_arena = ''
        try:
            default_arena = os.path.join(
                get_package_share_directory('mro_bringup'), 'params', 'arena.yaml')
        except Exception as e:
            self.get_logger().warn(f'mro_bringup 을 찾지 못했다 — arena_yaml 을 직접 넘길 것: {e}')
        # 기본값 (0,0,0) 은 '주입 안 됨' 을 뜻한다. 빈 리스트는 BYTE_ARRAY 로 추론되어
        #   robotN.yaml 의 실제 값을 되레 막으므로 쓸 수 없다 (9/7 확인).
        self.declare_parameter('dock_pose', [0.0, 0.0, 0.0])   # robotN.yaml (필수)
        self.declare_parameter('bay_dwell_s', 5.0)             # robotN.yaml
        self.declare_parameter('t_lim_s', 120.0)               # robotN.yaml
        self.declare_parameter('arena_yaml', default_arena)
        self.declare_parameter('aruco_max_dist_m', 1.5)        # 계약 6절 검출 조건
        self.declare_parameter('aruco_valid_s', 1.0)
        self.declare_parameter('aruco_confirm_s', 3.0)
        self.declare_parameter('aruco_retry', 1)          # 재탐지 스윕 횟수 (9/8 의미 변경)
        self.declare_parameter('aruco_search_enable', True)
        self.declare_parameter('aruco_search_step_deg', 15.0)   # 카메라 수평 FOV 보다 작게
        self.declare_parameter('aruco_search_max_deg', 60.0)    # 이보다 틀렸으면 측위가 깨진 것
        self.declare_parameter('aruco_search_dwell_s', 0.8)     # 정지점마다 확인하는 시간
        self.declare_parameter('token_retry_s', 2.0)           # 계약 3절 '거절되면 2초 후 재요청'
        self.declare_parameter('nav2_poll_s', 5.0)
        # I-035 로 추가 (9/8) — robotN.yaml 이관은 PM 요청 필요 (위 TODO ①)
        self.declare_parameter('nav2_poll_max', 3)          # 이 횟수 안에 active 가 아니면 포기
        self.declare_parameter('dock_status_wait_s', 15.0)   # dock_status 를 기다리는 상한
        # I-037 실험값 — common.yaml 이 단일 출처. 실물 검증 전 팀 확정값이 아니다.
        self.declare_parameter('tf_pose_timeout_s', 3.0)
        self.declare_parameter('tf_max_age_s', 0.3)
        self.declare_parameter('arrival_xy_tolerance_m', 0.25)
        # 베이 정렬 보정 (9/8 추가) — 값의 단일 출처는 common.yaml. 여기 기본값은 대체값이다.
        self.declare_parameter('bay_align_enable', True)
        # 접근이 끝났을 때 마커가 base_link 에서 있어야 할 자리 = 이 미션의 최종 정차 위치다.
        #   **0.9 m 는 추정이 아니라 실측에서 나온 값이다 (9/8).** 접근 2회차에 마커까지
        #   0.715 m 였을 때 워치독이 `ESTOP: 전방 0.29 m` 를 냈다 — 빼면 **선반 앞면이 마커
        #   벽에서 0.425 m 앞**이다. 여기서 바닥이 계산된다:
        #     워치독이 안 울리려면   0.425 + clear_dist 0.38  = 0.805 m
        #     Nav2 충돌 예측을 피하려면 0.425 + 0.379          = 0.804 m
        #   즉 **0.80 m 가 하드 바닥**이고 0.6·0.7 은 그 아래라 실패한 것이 당연했다. 0.9 는
        #   양쪽에 10 cm 씩 여유를 둔 값이다. 옛 '0.3 m 전진' 이 세우던 자리가 0.87 m 였는데,
        #   우연히 이 바닥 바로 위여서 지금까지 이 문제가 안 드러났다.
        #   좌우 치우침 목표는 두지 않는다 — approach_translation 의 설명 참조 (9/8 실주행).
        self.declare_parameter('bay_marker_range_m', 0.9)
        # 목표 거리 자체의 하한. footprint 외접 0.189 라 0.35 면 벽까지 여유 0.16 m 다.
        #   이보다 가까운 목표는 설정 오류로 보고 포기한다 — 선반에 밀어 넣는 것을 막는다
        self.declare_parameter('bay_align_min_range_m', 0.80)
        # 주행 중 **측정된** 법선거리의 하한. 위(목표 검증)와 다른 일을 한다 — 이건 매 측정에
        #   걸어서, 이미 이보다 가까이 있으면 전진하지 않는다. 측정이 깨졌을 때 선반으로
        #   밀고 들어가는 것을 막는 마지막 지점이다
        self.declare_parameter('bay_align_min_dn_m', 0.70)
        self.declare_parameter('bay_align_max_iter', 2)
        self.declare_parameter('bay_align_tol_yaw_deg', 5.0)
        # 법선선 이탈(횡오차)의 수렴 허용치와 상한.
        #   **상한을 넘으면 횡정렬만 건너뛴다 — 미션은 죽이지 않는다.** 베이 waypoint 는
        #   마커에서 약 1.2 m 로 베이 앞 열린 통로라(9/10 확인) 0.3 m 치우침이 실제로 가능하다.
        #   그보다 크면 측정을 의심한다. 슬롯 안이었다면 편측 여유가 0.173 m 뿐이라 값이 달라진다
        self.declare_parameter('bay_align_tol_lat_m', 0.05)
        self.declare_parameter('bay_align_max_lat_m', 0.30)
        self.declare_parameter('bay_align_tol_dn_m', 0.05)   # 법선거리 수렴 허용치
        # 표본 창 안 법선 방위의 중앙값절대편차(MAD) 상한 — **정확도 검사이지 회전 임계가 아니다.**
        #   두 해 사이를 프레임마다 오갈 때 잡는다. solvePnP 기울기 잡음이 σ≈2° 라(마커 0.10 m,
        #   거리 1.2 m, 코너잡음 0.25 px) 정상 측정의 MAD 는 0.674σ ≈ 1.35° 다. 6° 는 그 4.4 배라
        #   정상 측정을 거를 위험이 없다. **올리면 느슨해진다**
        self.declare_parameter('bay_align_normal_spread_deg', 6.0)
        # 90° 급 회전 뒤에는 마커가 FOV(38.5°) 밖에 있다가 돌아온다. 기존 2.0 s 로는 짧아
        #   9/9 R1a 가 11° 회전 뒤에도 재획득을 못 했다
        self.declare_parameter('bay_align_reacquire_s', 3.0)
        # DriveOnHeading 계통 오버슛 보정 [m] — 명령한 것보다 이만큼 더 간다.
        #   9/9 실측: 줄자 −0.065 · 노드 자체 측정 −0.071 (방법이 독립인데 6 mm 안에서 일치).
        #   **상수인지 비례인지는 아직 모른다** — 두 표본의 명령값이 0.270·0.257 로 거의 같았다.
        #   ratio 를 남겨 두었으니 짧은 이동(0.10 m 급) 회차 하나면 갈린다. 횡정렬은 0.15 m 급
        #   이동이라 이 구분이 크게 걸린다 (상수면 +45 %, 비례면 +26 %)
        self.declare_parameter('bay_align_drive_bias_m', 0.068)
        self.declare_parameter('bay_align_drive_bias_ratio', 0.0)
        # 이보다 작은 회전은 **명령하지 않는다.** Nav2 의 Spin 은 min_rotational_vel 0.4 rad/s ·
        #   10 Hz 라 한 사이클에 최소 2.3° 를 돌고 그만큼 지나친다 — 0.4° 를 시키면 6배가
        #   실행된다 (9/8 실측). 수렴 판정(tol_yaw)과 같은 값으로 두어야 앞뒤가 맞는다
        self.declare_parameter('bay_align_min_turn_deg', 5.0)
        self.declare_parameter('bay_align_samples', 5)        # 중앙값을 낼 표본 수
        self.declare_parameter('bay_align_sample_s', 2.0)     # 표본을 모으는 상한
        # 한 번에 움직일 상한. **접근 거리 전체가 여기 들어온다** — 잔차 보정만 하던 때보다 커야
        #   한다 (waypoint 가 마커에서 1.2 m 면 보정이 0.7 m 다). 넘으면 측정을 의심하고 포기한다
        self.declare_parameter('bay_align_max_step_m', 1.0)
        # 체류 뒤 칸 밖을 향해 제자리에서 도는 각. 0 이면 끈다 (turn_out_of_bay 설명 참조)
        self.declare_parameter('bay_exit_turn_deg', 180.0)
        # 돌고 나서 칸 밖으로 스스로 빠져나올 거리 [m]. 0 이면 바로 Nav2 에 넘긴다
        self.declare_parameter('bay_exit_drive_m', 0.5)
        self.declare_parameter('bay_align_speed_mps', 0.10)   # DriveOnHeading 속도
        self.declare_parameter('bay_align_timeout_s', 20)     # Spin·DriveOnHeading time_allowance
        # 마커 법선과 측량값(arena.yaml 방위 − AMCL 방위)이 이보다 어긋나면 **경고만** 남긴다.
        #   **기각하지 않는다 (9/10 결정) — 어긋날 때 마커를 신뢰한다.**
        #   측량값은 AMCL 방위에서 나오므로, 둘이 다를 때 어느 쪽이 틀렸는지 이 비교로는
        #   가릴 수 없다. 9/9 R1b 에서 방위 β 와 법선 ε 가 **정확히 같은 양(+19.8·+19.9°)만큼**
        #   함께 움직였는데, 이는 마커 측정이 자기일관적이고 로봇 방위 쪽이 어긋났다는 신호다.
        #   그래서 기각의 근거로 쓰지 않고 회차마다 기록만 남겨 표본을 모은다
        self.declare_parameter('bay_align_yaw_gate_deg', 10.0)

        self.arena = self._load_arena(self.p('arena_yaml'))

        # navigator의 블로킹 호출 동안에도 TF 수신은 계속한다 (I-037).
        # 셔틀 자신을 다른 executor에 넣으면 기존 spin_once와 충돌하므로 TF 전용 Node를 쓴다.
        # 전역 __node 리맵은 상속하지 않고, 로봇별 TF 토픽과 시계만 명시적으로 맞춘다.
        self.tf_node = Node(
            'shuttle_tf', namespace=self.ns, context=self.context, use_global_arguments=False,
            cli_args=['--ros-args', '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'],
            parameter_overrides=[Parameter(
                'use_sim_time', value=self.get_parameter('use_sim_time').value)])
        self.tf_buffer = Buffer(node=self.tf_node)
        self.tf_listener = TransformListener(self.tf_buffer, self.tf_node)
        self.tf_executor = SingleThreadedExecutor(context=self.context)
        self.tf_executor.add_node(self.tf_node)
        self.tf_thread = Thread(target=self._spin_tf, name='shuttle_tf', daemon=True)
        self.tf_thread.start()

        # 콜백이 채우는 값들 — 실행은 전부 메인 루프에서 한다
        self.pending = None      # 채택 대기 중인 Mission
        self.mission = None      # 수행 중인 Mission (None 이면 IDLE)
        self.t0 = None           # T_lim 기준 ROS 시각
        self.aruco_id = None     # (id, 수신 시각)
        self.aruco_dist = None   # (m, 수신 시각)
        # aruco/pose 원본을 그대로 쌓는다 — TF 변환은 메인 루프에서 한다 (콜백은 값만 저장)
        self.aruco_poses = deque(maxlen=30)
        self.cam_tf = None       # (프레임 이름, base_link←카메라 변환) — 고정이라 한 번만 찾는다
        self.estop = False
        self.last_status = None
        self.token_held = False

        self.create_subscription(Mission, MISSION_TOPIC, self.mission_cb, 10)
        self.create_subscription(Int32, ARUCO_ID_TOPIC, self.aruco_id_cb, 1)
        self.create_subscription(Float32, ARUCO_DIST_TOPIC, self.aruco_dist_cb, 1)
        self.create_subscription(PoseStamped, ARUCO_POSE_TOPIC, self.aruco_pose_cb, 10)
        self.create_subscription(Bool, ESTOP_TOPIC, self.estop_cb, 1)
        self.pub_status = self.create_publisher(String, STATUS_TOPIC, 10)
        self.pub_arrived = self.create_publisher(Bool, ARRIVED_TOPIC, 1)
        self.cli_token = self.create_client(ZoneToken, TOKEN_SRV)
        # 클라이언트는 한 번만 만든다 — 폴링마다 만들면 5 초마다 하나씩 쌓인다
        self.cli_state = self.create_client(GetState, f'{self.ns}/bt_navigator/get_state')

        self.set_status('READY')
        self.get_logger().info(f'대기: {self.ns}/{MISSION_TOPIC} 에 Mission(ASSIGNED) 오면 출발')

    # ── 파라미터·설정 ────────────────────────────────────────────────
    def _spin_tf(self):
        """TF 전용 실행기는 navigator나 미션 상태를 건드리지 않는다."""
        try:
            self.tf_executor.spin()
        except ExternalShutdownException:
            pass

    def destroy_node(self):
        """콜백 종료를 기다린 뒤 TF·navigator 자원을 회수한다."""
        self.tf_executor.shutdown()
        self.tf_thread.join()
        self.tf_listener.unregister()
        self.tf_node.destroy_node()
        self.navigator.destroy_node()
        return super().destroy_node()

    def p(self, name):
        return self.get_parameter(name).value

    def positive_parameter(self, name):
        """도착 검증을 우회하는 0·음수·NaN 설정은 미션 실패로 처리한다."""
        value = float(self.p(name))
        if not math.isfinite(value) or value <= 0.0:
            raise MissionAbort(f'{name} 은 유한한 양수여야 한다: {value}')
        return value

    def _load_arena(self, path):
        """arena.yaml 은 ROS 파라미터 형식이 아닌 데이터 yaml 이라 파일로 직접 읽는다."""
        if not path:
            self.get_logger().error(
                'arena_yaml 경로가 비었다 — 미션을 받아도 수행할 수 없다. '
                '런치나 --ros-args -p arena_yaml:=<경로> 로 넘길 것')
            return None
        try:
            with open(path, encoding='utf-8') as f:
                return yaml.safe_load(f)['arena']
        except Exception as e:
            self.get_logger().error(
                f'arena.yaml 적재 실패 — 미션을 받아도 수행할 수 없다 ({path}): {e}')
            return None

    # ── 콜백: 값만 저장한다 (navigator 호출 금지) ────────────────────
    def mission_cb(self, m):
        if m.state != Mission.ASSIGNED:
            return                                  # 그 밖의 전이는 관제 내부용이다
        if m.robot_ns not in ('', self.ns):
            return                                  # 단독 시험 시 오수신 방지
        if self.mission is not None or self.pending is not None:
            # 로봇당 미션은 1건이다. 큐를 두면 T_lim 시계를 언제부터 재야 할지가 모호해진다
            self.get_logger().warn(f'이미 미션이 있다 — 새 배정을 버린다: {m.mission_id}')
            return
        self.pending = m                            # 실행은 메인 루프에서

    def aruco_id_cb(self, msg):
        self.aruco_id = (int(msg.data), self.now())

    def aruco_dist_cb(self, msg):
        self.aruco_dist = (float(msg.data), self.now())

    def aruco_pose_cb(self, msg):
        """마커 자세. 여기서는 TF 를 만지지 않는다 — 변환은 메인 루프의 marker_sample() 이 한다."""
        self.aruco_poses.append((msg, self.now()))

    def estop_cb(self, msg):
        self.estop = bool(msg.data)

    # ── 공용 헬퍼 ────────────────────────────────────────────────────
    def now(self):
        """ROS 시각. 로그·CSV·DB 를 합쳐야 하므로 벽시계를 섞지 않는다 (계약 2절)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def set_status(self, s):
        if s == self.last_status:
            return                                  # 같은 값을 연속으로 내지 않는다
        self.last_status = s
        self.pub_status.publish(String(data=s))
        self.get_logger().info(f'[{s}]')

    def over_tlim(self):
        return self.t0 is not None and self.now() - self.t0 > self.p('t_lim_s')

    def elapsed(self):
        """미션 시작 이후 경과 초. 미션 밖(기동 시)에는 t0 가 없으므로 0.0 을 준다."""
        return 0.0 if self.t0 is None else self.now() - self.t0

    def docked_status(self, timeout_s):
        """도킹 여부를 돌려준다. 시간 안에 알 수 없으면 None.

        navigator.getDockedStatus() 는 dock_status 가 올 때까지 **무한히** 기다린다
        (turtlebot4_navigator.py: `while self.is_docked is None`). 로봇이 안 떠 있으면 거기서
        영영 멈추고, 미션 경로에서는 T_lim 조차 돌지 않아 FAILED 도 안 나온다 (가이드 D-2).
        구독은 navigator 의 것을 그대로 쓰되 기다리는 시간에 상한을 둔다.

        **먼저 무조건 한 번 spin 한다** — 원본과 같은 순서다. `is_docked` 는 `_dockCallback`
        으로만 바뀌고 `dock()`·`undock()` 은 건드리지 않으므로(turtlebot4_navigator.py 확인),
        조건부로 건너뛰면 묵은 값을 돌려준다. 특히 `startup()` 과 첫 미션 사이에는 navigator 를
        spin 하는 곳이 하나도 없어, 그 사이에 사람이 로봇을 도크에서 내리면 캐시된 True 를 믿고
        `dock()` 을 건너뛴 채 도크 밖에서 dock_pose 를 초기자세로 보내게 된다 (I-013, PR #44 PM 지적).
        """
        deadline = self.now() + float(timeout_s)
        rclpy.spin_once(self.navigator, timeout_sec=0.1)   # 원본과 같게: 먼저 한 번 받아온다
        while self.navigator.is_docked is None and self.now() < deadline:
            rclpy.spin_once(self.navigator, timeout_sec=0.1)
        return self.navigator.is_docked

    def wait(self, cond, timeout=None, on_tick=None):
        """유일한 대기 수단. 모든 대기가 여기를 지나므로 T_lim 검사가 한 곳에 모인다."""
        t_start = self.now()
        while rclpy.ok():
            # 완료와 제한시간 초과가 같은 tick에 와도 늦은 성공을 내지 않는다.
            if self.over_tlim():
                raise MissionAbort(f"T_lim {self.p('t_lim_s')} s 초과")
            done = cond()
            # navigator 완료 폴링 자체가 블로킹이므로 그 사이 지난 제한시간도 확인한다.
            if self.over_tlim():
                raise MissionAbort(f"T_lim {self.p('t_lim_s')} s 초과")
            if done:
                return True
            rclpy.spin_once(self, timeout_sec=TICK_S)   # 콜백은 대기 중에도 살아 있다
            if on_tick:
                on_tick()
            if self.over_tlim():
                raise MissionAbort(f"T_lim {self.p('t_lim_s')} s 초과")
            if timeout is not None and self.now() - t_start > timeout:
                return False
        raise MissionAbort('노드 종료')

    def tf_sample(self, warn=True):
        """최신 공통 시각의 map→base_link 좌표와 stamp를 함께 읽는다."""
        try:
            t = self.tf_buffer.lookup_transform(MAP_FRAME, BASE_FRAME, Time())
        except Exception as e:
            if warn:
                self.get_logger().warn(f'TF: {e}')
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return PoseSample(t.transform.translation.x, t.transform.translation.y, yaw,
                          Time.from_msg(t.header.stamp).nanoseconds * 1e-9)

    def tf_pose(self, warn=True):
        """관측용 좌표. 도착 판정과 목표 계산은 신선도를 검사하는 fresh_pose를 쓴다."""
        sample = self.tf_sample(warn=warn)
        return None if sample is None else sample.pose

    def fresh_pose(self, label, not_before=None):
        """새 TF를 제한시간 안에 얻지 못하면 좌표를 추측하지 않고 포기한다."""
        timeout = self.positive_parameter('tf_pose_timeout_s')
        max_age = self.positive_parameter('tf_max_age_s')
        sample = None

        def ready():
            nonlocal sample
            sample = self.tf_sample(warn=False)
            return (sample is not None
                    and all(math.isfinite(v) for v in (*sample.pose, sample.stamp_s))
                    and 0.0 <= self.now() - sample.stamp_s <= max_age
                    and (not_before is None or sample.stamp_s >= not_before))

        if not self.wait(ready, timeout=timeout):
            detail = ('TF 없음' if sample is None else
                      f'TF 시각 {sample.stamp_s:.3f}, 지연 {self.now() - sample.stamp_s:.3f} s')
            raise MissionAbort(f'{label}: 신선한 TF를 {timeout:.1f} s 안에 받지 못함 ({detail})')
        return sample

    def pose_of(self, x, y, yaw):
        """회전 인자는 도 단위다 — getPoseStamped 규약이라 라디안을 넘기면 조용히 틀어진다."""
        return self.navigator.getPoseStamped([float(x), float(y)], math.degrees(yaw))

    def goto(self, pose, label):
        """목표 1건. 콜백 밖에서만 불린다."""
        tolerance = self.positive_parameter('arrival_xy_tolerance_m')
        self.positive_parameter('tf_pose_timeout_s')
        self.positive_parameter('tf_max_age_s')
        if self.estop:
            self.get_logger().warn(f'estop 중 — 해제까지 {label} 보류')
            self.wait(lambda: not self.estop)        # 계약 4절: estop 동안 주행 명령 금지
        gx, gy = pose.pose.position.x, pose.pose.position.y
        self.get_logger().info(f'{label} → ({gx:.2f}, {gy:.2f}) 이동 시작')
        if not self.navigator.goToPose(pose):
            raise MissionAbort(f'{label} 목표 거부됨')
        self.wait(self.navigator.isTaskComplete)
        result = self.navigator.getResult().name
        if result != 'SUCCEEDED':
            raise MissionAbort(f'{label} 이동 실패: {result}')
        completed = self.now()
        # SUCCEEDED 직전 캐시가 아닌 완료 이후 관측으로 검증한다. 목표 전송 시각으로
        # 조회하면 출발 위치를 읽게 되므로 쓰지 않는다. TF는 물리적 실측과 구분해 표시한다.
        at = self.fresh_pose(label, not_before=completed)
        error = math.hypot(at.x - gx, at.y - gy)
        measurement = (f'목표 ({gx:.2f}, {gy:.2f}) / TF 추정 ({at.x:.2f}, {at.y:.2f}), '
                       f'오차 {error:.3f} m, TF 시각 {at.stamp_s:.3f}, '
                       f'지연 {self.now() - at.stamp_s:.3f} s')
        if error > tolerance:
            raise MissionAbort(
                f'{label} 도착 검증 실패 — Nav2 SUCCEEDED, {measurement}, 허용 {tolerance:.3f} m')
        self.get_logger().info(f'{label} 도착 검증 통과 — {measurement}')
        return at

    def token(self, acquire):
        """P1 토큰 획득/반납. 서비스가 없으면 (관제 미기동) 통과시켜 단독 시험을 막지 않는다."""
        if not self.cli_token.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('zone_token 서비스 없음 — 단독 모드로 진행')
            return True
        req = ZoneToken.Request()
        req.robot_ns, req.zone_id, req.acquire = self.ns, ZONE_P1, acquire
        fut = self.cli_token.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        res = fut.result()
        if res is None:
            self.get_logger().warn('zone_token 응답 없음')
            return False
        if acquire and not res.granted and res.holder:
            self.get_logger().info(f'P1 점유 중 (holder={res.holder})')
        return bool(res.granted)

    def release_token(self, reason):
        """성공·실패와 무관하게 반드시 반납한다 (계약 3절). 두 번 불려도 안전하다."""
        if not self.token_held:
            return
        self.token(False)
        self.token_held = False
        self.get_logger().info(f'P1 반납 ({reason})')

    def aruco_ok(self, expect_id):
        """마커 ID 와 거리는 헤더가 없어 짝을 맞출 수 없다 → 각자 수신 시각으로 신선도를 본다."""
        if self.aruco_id is None or self.aruco_dist is None:
            return False
        valid = self.p('aruco_valid_s')
        now = self.now()
        if now - self.aruco_id[1] > valid or now - self.aruco_dist[1] > valid:
            return False
        if self.aruco_id[0] == -1:                   # 미검출
            return False
        return (self.aruco_id[0] == expect_id
                and self.aruco_dist[0] <= self.p('aruco_max_dist_m'))

    def nav2_state(self):
        """bt_navigator 의 준비 여부를 (준비됨, 사유) 로 돌려준다.

        사유까지 남기는 이유 — '미준비' 한 줄만으로는 서비스가 아예 없는 것인지, 응답이
        없는 것인지, 라이프사이클이 active 가 아닌 것인지 구분되지 않아 원인을 짚는 데
        시간이 걸린다 (9/7 실주행에서 120 초를 그렇게 썼다).
        """
        if not self.cli_state.wait_for_service(timeout_sec=2.0):
            return False, f'{self.ns}/bt_navigator/get_state 서비스가 없다 — Nav2 미기동'
        fut = self.cli_state.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        res = fut.result()
        if res is None:
            return False, ('get_state 가 응답하지 않는다 — 같은 네임스페이스에 Nav2 가 '
                           '중복으로 떠 있는지 확인할 것 (ros2 node list)')
        label = res.current_state.label
        if label != 'active':
            return False, (f"bt_navigator 가 '{label}' 이다 — 라이프사이클 활성화가 끝나지 "
                           '않았다. lifecycle_manager 로그를 볼 것')
        return True, label

    # ── 미션 시퀀스 (메인 루프에서만 호출) ───────────────────────────
    def run_mission(self, m):
        self.mission = m
        self.t0 = self.now()
        self.token_held = False
        try:
            bay, waiting = self.mission_targets(m)
            self.init_pose()
            self.wait_nav2()
            self.get_logger().info('언독')
            self.navigator.undock()
            self.set_status('SHUTTLE')

            self.goto(self.pose_of(*bay['waypoint']), f'베이 {m.bay}')
            self.set_status('AT_BAY')                # Mission.msg: 'ArUco 확인, 체류 중'
            self.confirm_aruco(bay)
            self.approach_and_align(bay)
            self.dwell()
            self.turn_out_of_bay()   # 토큰보다 먼저 — 회전에 쓰는 몇 초가 60 s 강제 회수를 깎지 않게
            self.acquire_p1()

            self.set_status('SHUTTLE')
            self.goto(self.pose_of(*waiting), '대기공간')
            # 방법 B — 대기공간 도착이 곧 'P1 을 빠져나왔다' 는 뜻이다 (경로가 고정돼 있다)
            self.release_token('대기공간 도착 — P1 통과 완료')

            self.set_status('WAITING')               # AC-08 판정 기준점, 미션당 한 번
            self.pub_arrived.publish(Bool(data=True))
            self.get_logger().info(f'대기공간 도착 — 소요 {self.now() - self.t0:.1f} s')
        except MissionAbort as e:
            self.navigator.cancelTask()
            self.set_status('FAILED')                # 포기할 때만 (I-024·I-026)
            self.get_logger().error(f'미션 {m.mission_id} 포기: {e}')
        finally:
            self.release_token('미션 종료')
            self.mission = None
            self.t0 = None                           # 폐기 — 늦은 도착을 계상하지 않는다 (계약 7절)
            self.aruco_id = self.aruco_dist = None
            self.aruco_poses.clear()
            self.set_status('READY')

    def mission_targets(self, m):
        """요청 베이와 자기 대기공간 좌표를 arena.yaml 에서 찾는다."""
        if self.arena is None:
            raise MissionAbort('arena.yaml 미적재')
        bay = (self.arena.get('bays') or {}).get(m.bay)
        if not bay or len(bay.get('waypoint', [])) != 3:
            raise MissionAbort(f'arena.yaml 에 베이 {m.bay} waypoint 가 없다')
        waiting = (self.arena.get('waiting_zone') or {}).get(self.robot)
        if not waiting or len(waiting) != 3:
            raise MissionAbort(f'arena.yaml waiting_zone 에 {self.robot} 이 없다')
        self.get_logger().info(
            f"미션 {m.mission_id} · 베이 {m.bay} · 마커 {bay['aruco_id']} · 작업자 {m.worker_id}")
        return bay, waiting

    def dock_pose(self):
        """robotN.yaml 의 dock_pose 를 검증해 [x, y, yaw] 로 돌려준다.

        각 로봇이 (0,0) 으로 초기화하면 두 map 프레임의 원점이 물리적으로 다른 점이 되어
        세마포어 폴리곤·waypoint·FleetStatus 가 전부 어긋난다 — 1대로 시험하면 안 드러난다 (I-013).
        """
        v = [float(a) for a in (self.p('dock_pose') or [])]
        if len(v) != 3 or all(a == 0.0 for a in v):
            # (0,0,0) 은 파라미터가 안 들어왔다는 뜻으로 본다. 실제 도크는 robot3 [0.019,-0.795,0.133],
            #   robot8 [-0.023,0.034,-0.052] 라 원점과 겹치지 않는다. 원점으로 초기화하는 것이
            #   바로 두 map 프레임을 어긋나게 하는 실수다 (I-013) — 조용히 진행하지 않고 포기한다.
            raise MissionAbort(
                f'dock_pose 가 유효하지 않다 ({v}) — robotN.yaml 의 dock_pose 를 확인할 것. '
                '이대로 초기화하면 두 로봇의 맵 좌표계가 어긋난다 (I-013)')
        return v

    def send_initial_pose(self, v):
        """검증된 dock_pose 를 AMCL 에 보낸다. 이 시점부터 map→odom TF 가 나온다."""
        x, y, yaw = float(v[0]), float(v[1]), float(v[2])
        self.navigator.setInitialPose(self.pose_of(x, y, yaw))
        self.get_logger().info(
            f'초기자세 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°) 전송 — 자신의 도크 map 좌표')

    def startup(self):
        """미션을 받기 전에 초기자세를 보낸다 — 메인 루프에서만 부른다 (I-035).

        왜 여기서 보내나 — Nav2 의 global_costmap 은 기동할 때 map→base_link TF 를 요구하고,
        AMCL 은 초기자세를 받아야 map→odom 을 낸다. 초기자세가 늦으면 costmap 활성화가 실패하는데
        lifecycle_manager 는 나중에 TF 가 생겨도 재시도하지 않는다 (9/7 robot3 실측: initialpose
        발행 8 초 뒤에도 global_costmap 은 inactive). 미션을 받은 뒤에 보내면 이미 늦어
        T_lim 120 초를 통째로 태운다.

        기동 시에는 로봇을 움직이지 않는다 — 도킹돼 있지 않으면 초기자세를 보내지 않고 미션
        경로에 맡긴다. 도크 밖에서 dock_pose 로 초기화하면 측위가 대놓고 틀어지기 때문이다.
        여기서는 상태를 바꾸지 않는다 — 미션이 없으므로 FAILED 를 낼 수 없다 (I-024·I-026).
        """
        try:
            v = self.dock_pose()
        except MissionAbort as e:
            # 같은 검사가 미션 경로에서 다시 걸려 그때 FAILED 로 처리된다
            self.get_logger().error(f'기동 시 초기자세 보류: {e}')
            return
        wait_s = self.p('dock_status_wait_s')
        docked = self.docked_status(wait_s)
        if docked is None:
            self.get_logger().warn(
                f'기동 시 초기자세 보류 — dock_status 를 {wait_s} s 안에 받지 못했다. '
                '로봇 전원·네임스페이스·디스커버리를 확인할 것 (가이드 D-2)')
            return
        if not docked:
            self.get_logger().warn(
                '기동 시 초기자세 보류 — 도크에 물려 있지 않다. 도크 밖에서 dock_pose 로 '
                '초기화하면 측위가 틀어진다. 로봇을 도크에 올리고 다시 띄울 것')
            return
        self.send_initial_pose(v)
        self.get_logger().info(
            'Nav2 를 아직 띄우지 않았다면 지금 띄울 것 — 이 TF 로 global_costmap 이 활성화된다 (I-035)')

    def init_pose(self):
        """미션 경로의 초기자세. 기동 시 이후 로봇이 옮겨졌을 수 있어 다시 잡는다."""
        v = self.dock_pose()
        wait_s = self.p('dock_status_wait_s')
        docked = self.docked_status(wait_s)
        if docked is None:
            # 예전에는 여기서 무한 대기라 T_lim 도 안 돌고 FAILED 도 안 나왔다 (가이드 D-2)
            raise MissionAbort(
                f'dock_status 를 {wait_s} s 안에 받지 못했다 — '
                '로봇 전원·네임스페이스·디스커버리를 확인할 것')
        if not docked:
            self.get_logger().info('초기자세 설정을 위해 도킹')
            self.navigator.dock()
        self.send_initial_pose(v)

    def wait_nav2(self):
        """Nav2 가 active 가 될 때까지 **정해진 횟수만** 확인한다 (I-035).

        무한히 기다리면 T_lim 120 초를 통째로 태운다. 그런데 초기자세 없이 뜬 Nav2 는 스스로
        활성화를 재시도하지 않으므로 더 기다린다고 낫지 않는다 — 사유와 함께 빨리 포기해야
        '120 초 뒤 실패' 와 '10 초 만의 Nav2 미기동' 이 회차 기록에서 구분된다.
        경과 시간은 elapsed() 로 찍는다 — t0 가 없는 경로에서도 안전해야 한다.
        """
        tries = max(1, int(self.p('nav2_poll_max')))
        reason = ''
        for attempt in range(1, tries + 1):
            ready, reason = self.nav2_state()
            if ready:
                self.get_logger().info('Nav2 준비됨 (bt_navigator active)')
                return
            self.get_logger().warn(
                f'Nav2 미준비 {attempt}/{tries} ({self.elapsed():.0f} s 경과 / '
                f"T_lim {self.p('t_lim_s'):.0f} s): {reason}")
            if attempt < tries:
                self.wait(lambda: False, timeout=self.p('nav2_poll_s'))
        raise MissionAbort(f'Nav2 미준비 — {tries}회 확인 동안 active 가 아니다: {reason}')

    def confirm_aruco(self, bay):
        """베이 도착 확정. 못 보면 제자리 회전으로 재탐지하고, 그래도 없으면 포기한다 (FR-08).

        포기하면 접근·체류를 하지 않는다 — 오도착 0건이 절대 기준이다.
        """
        sweeps = max(0, int(self.p('aruco_retry')))
        seen = None
        for attempt in range(sweeps + 1):
            if self.wait(lambda: self.aruco_ok(bay['aruco_id']),
                         timeout=self.p('aruco_confirm_s')):
                self.get_logger().info(
                    f"마커 {bay['aruco_id']} 확인 (거리 {self.aruco_dist[0]:.2f} m)")
                return
            seen = self.aruco_id[0] if self.aruco_id else None
            if attempt == sweeps:
                break
            self.get_logger().warn(
                f'마커 미확인 {attempt + 1}/{sweeps + 1} (관측 {seen}) — 제자리 회전으로 재탐지')
            if self.search_marker(bay):     # 아직 포기 아님 → AT_BAY 를 유지한다 (I-026)
                return
        raise MissionAbort(f"ArUco 불일치 — 오도착 (기대 {bay['aruco_id']}, 관측 {seen})")

    def search_offsets(self):
        """훑을 상대 각도들 — 가까운 쪽부터 좌우 교대 [−Δ, +Δ, −2Δ, +2Δ …].

        AMCL 방위 오차는 대개 작아 가까운 쪽이 먼저 맞을 확률이 높다. 좌우를 교대하면 한쪽으로
        편향되지 않고, 못 찾고 끝나도 총 회전량이 최소다.
        """
        step = math.radians(self.positive_parameter('aruco_search_step_deg'))
        limit = math.radians(self.positive_parameter('aruco_search_max_deg'))
        offsets, k = [], 1
        while k * step <= limit + 1e-9:
            offsets.extend((-k * step, k * step))
            k += 1
        return offsets

    def search_marker(self, bay):
        """제자리 회전만으로 마커를 다시 찾는다. 찾으면 True.

        **병진하지 않는다.** 마커를 못 본다는 것은 자기 위치를 못 믿는다는 뜻이라 어디로 갈지
        모른다. 제자리 회전이 footprint 밖으로 쓸고 가는 폭은 1.4 cm 뿐이다.

        찾은 뒤 원래 방위로 되돌리지 않는다 — 접근 단계가 측량 베이 방위로 돌려세우므로
        되돌리면 같은 회전을 두 번 한다.
        """
        if not self.p('aruco_search_enable'):
            return False
        if not self.behavior_server_ready():
            raise MissionAbort('behavior_server 의 spin 이 없다 — Nav2(T3) 를 확인할 것')
        dwell = self.positive_parameter('aruco_search_dwell_s')
        allow = max(1, int(self.p('bay_align_timeout_s')))
        offsets = self.search_offsets()
        self.get_logger().info(
            f'마커 재탐지 — 제자리 회전 {len(offsets)}회, '
            f"±{self.p('aruco_search_max_deg'):.0f}° 를 "
            f"{self.p('aruco_search_step_deg'):.0f}° 간격으로")
        at = 0.0
        for i, target in enumerate(offsets, start=1):
            delta = wrap_pi(target - at)
            self.run_behavior(lambda d=delta: self.navigator.spin(d, allow),
                              f'마커 재탐지 회전 {math.degrees(target):+.0f}°')
            at = target
            if self.wait(lambda: self.aruco_ok(bay['aruco_id']), timeout=dwell):
                self.get_logger().info(
                    f"마커 {bay['aruco_id']} 재탐지 성공 — 기준 방위에서 "
                    f'{math.degrees(at):+.0f}° ({i}/{len(offsets)}), '
                    f'거리 {self.aruco_dist[0]:.2f} m')
                return True
        self.get_logger().warn(
            f"마커 재탐지 실패 — ±{self.p('aruco_search_max_deg'):.0f}° 안에 없다 "
            f'(지금 방위는 기준에서 {math.degrees(at):+.0f}°)')
        return False

    # ── 베이 접근·정렬 (마커 기준 상대 이동) ─────────────────────────
    def camera_transform(self, frame):
        """base_link ← 카메라 광학 프레임. 고정 변환이라 **한 번만 찾아 들고 있는다**.

        기다리는 이유 — `tf_static` 은 늦게 도착한다. 9/8 실측에서 `tf2_echo` 도 처음 몇 초는
        `Invalid frame ID "base_link" ... frame does not exist` 를 내다가 그 뒤에 값을 냈다.
        기다리지 않으면 그 창에 걸린 회차가 '마커 표본 없음' 으로 미션을 포기한다.
        robot3 실측값: 병진 [-0.060, 0, 0.244] · 회전 (xyzw) [-0.5, 0.5, -0.5, 0.5].
        """
        if self.cam_tf is not None and self.cam_tf[0] == frame:
            return self.cam_tf[1]
        found = None

        def ready():
            nonlocal found
            try:
                found = self.tf_buffer.lookup_transform(BASE_FRAME, frame, Time())
            except Exception:
                return False
            return True

        if not self.wait(ready, timeout=self.positive_parameter('tf_pose_timeout_s')):
            self.get_logger().warn(
                f'카메라 TF({BASE_FRAME}←{frame}) 를 못 찾았다 — 프레임 이름과 '
                'tf_static 발행을 확인할 것 (ros2 run tf2_ros tf2_echo)')
            return None
        t = found.transform.translation
        self.get_logger().info(
            f'카메라 TF 확보 ({BASE_FRAME}←{frame}) — '
            f'병진 [{t.x:.3f}, {t.y:.3f}, {t.z:.3f}]')
        self.cam_tf = (frame, found)
        return found

    def marker_in_base(self, msg, tf, distance=None):
        """aruco/pose 한 장을 base_link 로 옮겨 (앞, 왼쪽, 법선 방위) 로 돌려준다.

        **거리를 주면 그 값으로 다시 재서 쓴다 — 방향만 pose 에서 가져온다 (9/8).**
        두 값은 계산 방법이 다르다: `aruco/distance` 는 depth 에 로봇별 보정(k)을 건 값이라
        줄자 대조 ±1.5 cm 인 반면, `aruco/pose` 의 위치는 solvePnP 의 tvec 에 depth/pnp 비
        **중앙값**을 곱한 것이라 프레임별 오차가 그대로 남는다. 실측에서 같은 순간에
        1.188 m 대 0.943 m 로 26 % 어긋났고, 그 때문에 목표 0.9 m 인데 0.66 m 까지 들어가
        하드 바닥(0.80 m) 아래로 내려갔다. 크기는 정확한 쪽을, 방향은 pose 를 쓴다.

        마커 법선은 마커 좌표계의 +z 를 base_link 로 옮긴 것이다(회전행렬 3열). 로봇이 벽을
        정면으로 보면 법선은 로봇 쪽 = base_link −x 를 향하므로 방위가 π 가 된다.
        solvePnP 의 두 해 중 법선이 반대로 나온 해는 부호를 되돌려 흡수한다 — 이건 뒤집힘이
        분명해 안전하지만, 남는 기울기 모호성까지 지우지는 못한다 (그래서 기본은 안 쓴다).
        """
        pose = msg.pose
        if distance is not None:
            pos = pose.position
            reach = math.sqrt(pos.x ** 2 + pos.y ** 2 + pos.z ** 2)
            if reach < 1e-6:
                return None
            k = distance / reach          # 카메라 프레임에서 크기만 갈아끼운다 (방향 보존)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pos.x * k, pos.y * k, pos.z * k
            pose.orientation = msg.pose.orientation
        p = do_transform_pose(pose, tf)
        q = p.orientation
        nx = 2.0 * (q.x * q.z + q.w * q.y)
        ny = 2.0 * (q.y * q.z - q.w * q.x)
        if not all(math.isfinite(v) for v in (p.position.x, p.position.y, nx, ny)):
            return None
        if p.position.x <= 0.0:
            return None                       # 마커가 로봇 뒤에 있다 — 변환이나 검출이 틀렸다
        if nx > 0.0:
            nx, ny = -nx, -ny                 # 법선은 로봇 쪽을 향한다 (두 해 중 뒤집힌 것 복원)
        if math.hypot(nx, ny) < 1e-3:
            return None                       # 수평 성분이 없다 — 방위를 낼 수 없다
        return p.position.x, p.position.y, math.atan2(ny, nx)

    def marker_sample(self, expect_id=None, window_s=None):
        """여러 프레임의 중앙값으로 한 점을 만든다. 표본을 못 모으면 None.

        **expect_id 를 주면 그 마커가 지금 보일 때만 쓴다.** `aruco/pose` 에는 ID 가 없어서
        (계약 4절) 이 토픽만 보면 어느 마커의 자세인지 알 수 없다. 검출기는 여럿 보이면 가장
        가까운 하나만 내므로, 옆 베이 마커로 바뀌는 순간의 자세를 그대로 쓰면 엉뚱한 곳으로
        간다. 같은 프레임에서 함께 나오는 `aruco/id`·`aruco/distance` 로 걸러 준다.

        중앙값을 쓰는 이유 — 한 프레임이 튀어도(두 해 뒤집힘·ROI 오염) 보정량이 끌려가지 않는다.
        법선 방위는 π 근처라 그대로 중앙값을 내면 π 와 −π 가 섞이므로 첫 표본 기준으로 접어서 낸다.
        """
        want = max(1, int(self.p('bay_align_samples')))
        window = window_s or self.positive_parameter('bay_align_sample_s')
        self.aruco_poses.clear()
        # ID 일치를 **기다리는 조건에 넣는다.** 미리 한 번 보고 없다고 곧장 포기하면, 회전 직후
        #   한두 프레임 놓친 것만으로 미션이 죽는다 (9/8 T15·T16 이 그랬다)
        self.wait(lambda: (len(self.aruco_poses) >= want
                           and (expect_id is None or self.aruco_ok(expect_id))),
                  timeout=window)
        if expect_id is not None and not self.aruco_ok(expect_id):
            self.get_logger().warn(
                f'마커 {expect_id} 를 {window:.1f} s 안에 못 봤다 — 자세 표본을 쓰지 않는다 '
                '(aruco/pose 에는 ID 가 없어 옆 베이 마커일 수 있다)')
            return None
        frames = list(self.aruco_poses)
        if not frames:
            return None
        tf = self.camera_transform(frames[0][0].header.frame_id or CAMERA_FRAME_FALLBACK)
        if tf is None:
            return None
        # 거리는 검출기가 vetting 한 aruco/distance 를 쓴다 (위 marker_in_base 설명)
        distance = None
        if self.aruco_dist is not None:
            value, stamp = self.aruco_dist
            if self.now() - stamp <= self.p('aruco_valid_s') and math.isfinite(value):
                distance = float(value)
        if distance is None:
            self.get_logger().warn('aruco/distance 가 신선하지 않다 — pose 크기를 그대로 쓴다')
        else:
            # 9/8 미해결: 검출기가 0.899 m 를 찍은 순간 우리 계산이 1.104 m 였다. 어느 쪽이
            #   어긋났는지 로그로 바로 가리려고 쓴 값을 남긴다 (내일 첫 항목)
            self.get_logger().info(
                f'마커 표본 {len(frames)}개 · 자로 쓴 aruco/distance {distance:.3f} m '
                f'(수신 {self.now() - self.aruco_dist[1]:.2f} s 전)')
        rows, last = [], None
        for msg, recv in frames:
            row = self.marker_in_base(msg, tf, distance)
            if row is not None:
                rows.append(row)
                last = recv
        if not rows:
            return None
        ref = rows[0][2]
        folded = [wrap_pi(r[2] - ref) for r in rows]
        centre = median(folded)
        # 중앙값절대편차. 두 해 사이를 프레임마다 오가면 여기가 크게 뛴다 —
        #   중앙값만 보면 그 오감이 안 보인다 (표본의 절반 이상이 한쪽이면 그쪽으로 정해진다)
        mad = median([abs(f - centre) for f in folded])
        return MarkerSample(
            median([r[0] for r in rows]), median([r[1] for r in rows]),
            ref + centre, last, mad)

    def behavior_server_ready(self, timeout_s=2.0):
        """Spin·DriveOnHeading 액션 서버가 떠 있는지.

        BasicNavigator 의 spin()·driveOnHeading() 은 서버가 없으면 **무한히** 기다린다
        (`while not wait_for_server(...)`). getDockedStatus 와 같은 함정이라 먼저 확인한다.
        대역(시험)에는 액션 클라이언트가 없으므로 준비된 것으로 본다.
        """
        for attr in ('spin_client', 'drive_on_heading_client'):
            client = getattr(self.navigator, attr, None)
            if client is not None and not client.wait_for_server(timeout_sec=timeout_s):
                return False
        return True

    def normal_trustworthy(self, sample, surveyed):
        """법선을 정렬 기준으로 써도 되는지 본다. 못 쓰면 사유 문자열, 쓰면 None.

        **검사는 하나뿐이다 — 표본 창 안의 산포(MAD).** 정사각 마커의 두 해가 프레임마다
        오가면 여기가 크게 뛴다. 중앙값만 보면 그 오감이 안 보인다: 표본의 절반 이상이
        한쪽이면 중앙값은 조용히 그쪽으로 정해진다.

        **측량값과의 어긋남은 기각 사유가 아니다 (9/10 결정).** 측량값은 AMCL 방위에서
        나오므로 둘이 다를 때 어느 쪽이 틀렸는지 이 비교로는 가릴 수 없다. 9/9 R1b 에서
        방위 β 와 법선 ε 가 **같은 양(+19.8·+19.9°)만큼 함께 움직였고**, 이는 마커 측정이
        자기일관적이고 로봇 방위 쪽이 어긋났다는 신호다. 경고만 남기고 표본을 모은다.
        """
        spread = math.radians(self.positive_parameter('bay_align_normal_spread_deg'))
        if sample.normal_mad > spread:
            return (f'법선 산포 {math.degrees(sample.normal_mad):.1f}° 가 상한 '
                    f'{math.degrees(spread):.0f}° 를 넘는다 — 두 해 사이를 오가고 있다')
        gate = math.radians(self.positive_parameter('bay_align_yaw_gate_deg'))
        off = wrap_pi(sample.yaw_error - surveyed)
        if abs(off) > gate:
            self.get_logger().warn(
                f'법선 {math.degrees(sample.yaw_error):+.1f}° 와 측량 '
                f'{math.degrees(surveyed):+.1f}° 가 {math.degrees(abs(off)):.1f}° 어긋난다 '
                f'(게이트 {math.degrees(gate):.0f}°) — **마커를 쓴다.** 측량값은 AMCL 방위에서 '
                '나오므로 어느 쪽이 틀렸는지 이 비교로는 못 가린다 (9/10)')
        return None

    def drive_command(self, want):
        """원하는 거리를 실제로 가려면 얼마를 명령해야 하는지 [m].

        `DriveOnHeading` 은 명령보다 **계통적으로 더 간다** — 9/9 실측 −0.065(줄자)와
        −0.071(노드 자체 측정)로, 방법이 독립인데 6 mm 안에서 일치했다. 그만큼 덜 명령한다.

        **상수인지 비례인지는 아직 모른다.** 두 표본의 명령값이 0.270·0.257 로 거의 같아서
        구분이 안 됐다. 두 항을 다 두었으니 짧은 이동(0.10 m 급) 회차 하나면 갈린다.
        보정 뒤 값이 0 이하가 되면 움직이지 않는다 — 되레 지나치는 것보다 낫다.
        """
        bias = self.p('bay_align_drive_bias_m') + self.p('bay_align_drive_bias_ratio') * want
        return want - bias

    def drive(self, want, label):
        """오버슛을 보정해 직진한다. 보정 뒤 남는 것이 없으면 명령하지 않는다."""
        cmd = self.drive_command(want)
        if cmd <= 0.0:
            self.get_logger().info(
                f'{label} 생략 — 원하는 {want:.3f} m 가 오버슛 보정 '
                f'{want - cmd:+.3f} m 안에 들어간다')
            return
        speed = self.positive_parameter('bay_align_speed_mps')
        allow = max(1, int(self.p('bay_align_timeout_s')))
        self.get_logger().info(f'{label} — 원하는 {want:.3f} m, 명령 {cmd:.3f} m')
        self.run_behavior(lambda: self.navigator.driveOnHeading(cmd, speed, allow), label)

    def turn(self, angle, label):
        """분해능 위일 때만 제자리 회전한다. 실제로 돈 각을 돌려준다."""
        if not self.turn_worth_doing(angle):
            self.get_logger().info(
                f'{label} 생략 — {math.degrees(angle):+.1f}° 는 Spin 분해능 미만')
            return 0.0
        allow = max(1, int(self.p('bay_align_timeout_s')))
        self.run_behavior(lambda: self.navigator.spin(angle, allow), label)
        return angle

    def lateral_align(self, sample, label):
        """① 횡정렬 — 마커 법선선 위로 옮긴다. 회전 θ_F → 직진 |e_lat|.

        실제로 돈 각을 돌려준다 (안 움직였으면 0.0). 이 값은 다음 단계에서 정면을 향할
        회전량을 내는 데 쓴다.

        **여기서 실패해도 미션을 죽이지 않는다.** θ_F 는 법선에 수직이라 ±90° 급이고,
        Nav2 `Spin` 의 충돌 예측에 걸릴 확률이 접근 동작 중 가장 높다 — 9/9 R1 은 3회 중
        2회가 Spin 취소로 죽었다. 여기서 또 죽으면 횡정렬을 넣은 대가로 성공률이 떨어진다.
        실패하면 그 자리에서 다음 단계로 넘어가고 잔여 횡오차는 로그로 남는다. 지금까지의
        동작(횡오차를 아예 보정하지 않던 것)과 같은 수준일 뿐 더 나빠지지 않는다.
        """
        e_lat = sample.lat_error
        tol = self.positive_parameter('bay_align_tol_lat_m')
        cap = self.positive_parameter('bay_align_max_lat_m')
        if abs(e_lat) <= tol:
            self.get_logger().info(
                f'{label} 횡정렬 불필요 — 이탈 {e_lat:+.3f} m 가 허용 {tol:.3f} m 안이다')
            return 0.0
        if abs(e_lat) > cap:
            self.get_logger().warn(
                f'{label} 횡정렬 생략 — 이탈 {e_lat:+.3f} m 가 상한 {cap:.3f} m 를 넘는다. '
                '이 크기는 실제 치우침보다 법선 오류일 때가 많다 (법선이 δ 만큼 틀리면 '
                f'수선의 발이 거리×sin δ 만큼 밀린다). 잔여 이탈은 그대로 둔다')
            return 0.0
        fx, fy = sample.foot
        theta = math.atan2(fy, fx)
        self.get_logger().info(
            f'{label} 횡정렬 — 법선선에서 {e_lat:+.3f} m 벗어나 있다. '
            f'수선의 발 ({fx:+.3f}, {fy:+.3f}) 로 {math.degrees(theta):+.1f}° 돌아 '
            f'{abs(e_lat):.3f} m 간다')
        try:
            turned = self.turn(theta, f'{label} 횡정렬 회전')
            self.drive(abs(e_lat), f'{label} 횡정렬 직진')
            return turned
        except MissionAbort as exc:
            self.get_logger().warn(
                f'{label} 횡정렬 실패({exc}) — 생략하고 방위·거리 정렬로 넘어간다. '
                '미션은 계속한다 (횡오차는 원래 보정하지 않던 값이다)')
            return 0.0

    def face_marker(self, sample, turned, label):
        """② 방위정렬 — 마커를 정면으로 본다. 회전 (θ_face − 이미 돈 각).

        **이것은 명령 누적이 아니다.** 평행이동은 base_link 에서 마커 법선의 *방향* 을
        바꾸지 않는다 — 바뀌는 것은 마커의 위치뿐이다. 로봇 방위는 우리가 θ_F 만큼 돌린
        것을 알고 있으므로 남은 회전 (θ_face − θ_F) 는 이동 거리와 무관하게 기하학적으로
        정확하다. 남는 오차는 Spin·Drive 의 실행 오차뿐이고, 그것은 다음 재측정이 잡는다.

        횡정렬 자세에서는 마커가 카메라 FOV(38.5°) 밖이라 잴 수 없다. 그래서 재측정을
        '동작 사이' 가 아니라 **'마커가 보이는 자세가 된 뒤'** 에 넣는다.
        """
        remain = wrap_pi(sample.face_yaw - turned)
        self.get_logger().info(
            f'{label} 방위정렬 — 정면 {math.degrees(sample.face_yaw):+.1f}° 중 '
            f'{math.degrees(turned):+.1f}° 는 이미 돌았다. 남은 {math.degrees(remain):+.1f}°')
        self.turn(remain, f'{label} 방위정렬 회전')

    def advance_to_range(self, bay, r_target, label, before=None):
        """③ 거리정렬 — 법선방향 거리를 목표에 맞춘다. 재측정 → 소각 보정 → 직진.

        **반경거리가 아니라 법선거리(d_n)를 맞춘다.** 선반·벽까지의 여유를 정하는 것은
        반경이 아니라 벽에 수직인 거리다 — 옆으로 치우쳐 있으면 반경이 같아도 벽에는 더
        가깝다. 목표보다 이미 가까우면 **전진하지 않는다** (후진 금지, 계약 2절).

        **재측정에 실패하면 이전 측정의 법선거리를 쓴다 — 이것은 근사가 아니다.**
        앞의 두 단계는 d_n 을 바꾸지 않는다: 방위정렬은 제자리 회전이고, 횡정렬은 수선의
        발로 가는 이동인데 그 방향은 t 라 법선 n 에 수직이다. 수직 방향으로 움직이면
        평면까지의 거리가 변하지 않는다. 그래서 회전 뒤 한두 프레임을 놓쳤다고 접근을
        통째로 버릴 이유가 없다 (9/8 T15·T16 · 9/9 R1a 가 이 자리에서 죽었다).
        다만 잔여 방위 보정은 못 하므로 그때는 건너뛴다.
        """
        window = self.positive_parameter('bay_align_reacquire_s')
        sample = self.marker_sample(bay['aruco_id'], window_s=window)
        if sample is None:
            if before is None:
                self.get_logger().warn(
                    f'{label} 거리정렬 — 회전 뒤 마커를 {window:.1f} s 안에 못 봤고 '
                    '이전 측정도 없다. 그 자리에 선다')
                return None
            self.get_logger().warn(
                f'{label} 거리정렬 — 회전 뒤 마커를 {window:.1f} s 안에 못 봤다. '
                f'이전 측정의 법선거리 {before.normal_range:.3f} m 를 쓴다 '
                '(앞 두 단계는 법선거리를 바꾸지 않는다 — 회전이거나 법선에 수직인 이동이다)')
            sample = before
        floor = self.positive_parameter('bay_align_min_dn_m')
        if sample.normal_range < floor:
            self.get_logger().warn(
                f'{label} 거리정렬 생략 — 법선거리 {sample.normal_range:.3f} m 가 하한 '
                f'{floor:.3f} m 아래다. 전진하지 않는다')
            return sample
        if sample is not before:
            self.turn(sample.face_yaw, f'{label} 정면 잔여 회전')
        want = sample.normal_range - r_target
        if want <= 0.0:
            self.get_logger().info(
                f'{label} 거리정렬 불필요 — 법선거리 {sample.normal_range:.3f} m 가 이미 목표 '
                f'{r_target:.3f} m 안이다. 후진하지 않는다 (계약 2절)')
            return sample
        self.drive(want, f'{label} 거리정렬 직진')
        return sample

    def run_behavior(self, start, label):
        """상대 이동 1건. goto() 와 같은 폴링 규약을 쓰되 좌표 검증은 마커가 대신한다."""
        if self.estop:
            self.get_logger().warn(f'estop 중 — 해제까지 {label} 보류')
            self.wait(lambda: not self.estop)      # 계약 4절: estop 동안 주행 명령 금지
        if not start():
            raise MissionAbort(f'{label} 거부됨')
        self.wait(self.navigator.isTaskComplete)
        result = self.navigator.getResult().name
        if result != 'SUCCEEDED':
            raise MissionAbort(f'{label} 실패: {result}')

    def turn_worth_doing(self, angle):
        """분해능보다 작은 회전은 명령하지 않는다 — 명령하면 되레 나빠진다.

        Nav2 의 Spin 은 `min_rotational_vel` 0.4 rad/s 를 밑돌지 않고 제어 주기가 10 Hz 라
        **한 사이클에 최소 0.04 rad = 2.3° 를 돌고, 정지 판정도 사이클 단위라 그만큼 지나친다.**
        9/8 실주행에서 0.37° 를 명령했는데 실제로는 그 몇 배가 실행됐고, 그 오차가 매 회전마다
        쌓여 법선 오차가 8.3° → 16.4° → −14.1° 로 **발산**했다. 작은 각은 그냥 두는 편이 낫다.
        """
        return abs(angle) >= math.radians(self.positive_parameter('bay_align_min_turn_deg'))

    def relative_move(self, tx, ty, phi, label):
        """회전–직진–회전. 차동구동이라 옆으로 못 가고, 직진은 항상 전진이다 (후진 금지)."""
        speed = self.positive_parameter('bay_align_speed_mps')
        allow = max(1, int(self.p('bay_align_timeout_s')))
        step = math.hypot(tx, ty)
        turn1 = 0.0
        if step >= 1e-3:
            heading = math.atan2(ty, tx)           # 이동 방향을 정면으로 — 그래야 전진만으로 간다
            if self.turn_worth_doing(heading):
                turn1 = heading
                self.run_behavior(lambda: self.navigator.spin(turn1, allow), f'{label} 회전①')
            else:
                # 안 돌았다. 지금 방위 그대로 가면 옆으로 step·sin(heading) 만큼 어긋나는데,
                #   불감대(5°)에서 그 값은 수렴 허용치와 같은 자리수다. 회전 오차보다 낫다
                self.get_logger().info(
                    f'{label} 회전① 생략 — {math.degrees(heading):+.1f}° 는 Spin 분해능 미만')
            self.run_behavior(
                lambda: self.navigator.driveOnHeading(step, speed, allow), f'{label} 전진')
        turn2 = wrap_pi(phi - turn1)
        if self.turn_worth_doing(turn2):
            self.run_behavior(lambda: self.navigator.spin(turn2, allow), f'{label} 회전②')
        elif abs(turn2) > 1e-6:
            self.get_logger().info(
                f'{label} 회전② 생략 — {math.degrees(turn2):+.1f}° 는 Spin 분해능 미만')

    def turn_out_of_bay(self):
        """체류 뒤 제자리에서 돌아 칸 밖을 향한다 — Nav2 가 좁은 칸에서 방향을 못 바꾼다.

        **실주행 2회 연속으로 여기서 죽었다** (마커까지 0.6 m·0.7 m 둘 다). 체류를 마치고
        대기공간 목표를 받은 Nav2 가 90 초 넘게 진전을 못 내고 T_lim 을 넘겼다. 두 번째
        회차에는 워치독 estop 조차 없었다 — 장애물이 가까워서가 아니라 **경로를 못 푼 것**이다.

        로봇이 **벽을 마주 본 채** 출발 명령을 받기 때문이다. 후진이 금지돼 있어(계약 2절)
        좁은 칸 안에서 스스로 180° 를 돌아야 하는데 DWB 가 그걸 풀지 못한다.

        그래서 우리가 대신 돌려 준다. 돌고 나면 Nav2 는 앞으로 나오기만 하면 된다.
          · 제자리 회전이 footprint 밖으로 쓸고 가는 폭은 1.4 cm 다 (safety_watchdog 과 같은
            근거) — 칸 안에서도 안전하고, '제자리 회전 뒤 전진' 은 계약이 정한 탈출 방식이다.
          · **AMCL 을 쓰지 않는다.** 목표 방위를 계산하지 않고 그냥 뒤로 돈다 — 칸 밖은 무조건
            로봇 뒤쪽이고, 그 판단에 측위는 필요 없다.
          · 제자리 회전이라 위치가 바뀌지 않으므로 방법 B 의 전제(획득 시점이 P1 폴리곤 밖)를
            건드리지 않는다.
        """
        turn = math.radians(float(self.p('bay_exit_turn_deg')))
        if abs(turn) < 1e-6:
            self.get_logger().info('탈출 회전 꺼짐 (bay_exit_turn_deg=0)')
            return
        if not self.behavior_server_ready():
            raise MissionAbort('behavior_server 의 spin 이 없다 — Nav2(T3) 를 확인할 것')
        allow = max(1, int(self.p('bay_align_timeout_s')))
        self.get_logger().info(
            f'칸 밖을 향해 제자리 {math.degrees(turn):.0f}° 회전')
        self.run_behavior(lambda: self.navigator.spin(wrap_pi(turn), allow), '탈출 회전')
        # 돌기만 하고 Nav2 에 넘겼더니 좁은 칸 안에서 DWB 가 앞뒤로 헤매다 진행하지 못했다
        #   (9/8 T17: 35 초 넘게 제자리에서 흔들리고 P1 이 강제 회수됐다). 칸을 벗어나는
        #   구간까지 우리가 곧게 빼 준 뒤 넘긴다 — 열린 곳에서는 Nav2 가 잘 한다
        out = float(self.p('bay_exit_drive_m'))
        if out <= 0.0:
            return
        speed = self.positive_parameter('bay_align_speed_mps')
        self.get_logger().info(f'칸 밖으로 {out:.2f} m 전진 — 여기까지만 우리가 뺀다')
        self.run_behavior(lambda: self.navigator.driveOnHeading(out, speed, allow), '탈출 전진')

    def approach_target(self):
        """접근을 끝낼 마커까지의 거리를 검증해 돌려준다.

        이것이 **미션의 최종 정차 위치**다 (옛 '0.3 m 전진' 을 대신한다). 그래서 0 이나 음수를
        '거리 보정 안 함' 으로 조용히 넘기지 않는다 — 설정 누락으로 보고 포기한다.
        하한(bay_align_min_range_m)은 선반·벽에 밀어 넣는 설정을 막는 유일한 방어다.

        하한을 정할 때 Nav2 쪽 바닥도 같이 봐야 한다: DriveOnHeading 은 앞을
        `속도 × simulate_ahead_time`(0.1 × 1.9 = 0.19 m) 내다보고 footprint 정면 0.189 m 를
        더해, **중심에서 약 0.38 m 안에 벽이 들어오면 스스로 중단한다.** 목표 거리가 0.45 m
        밑이면 정상 접근이 매번 `Collision Ahead` 로 끝난다 (9/8 실측).
        """
        r_target = self.positive_parameter('bay_marker_range_m')
        floor = self.positive_parameter('bay_align_min_range_m')
        if r_target < floor:
            raise MissionAbort(
                f'bay_marker_range_m {r_target:.3f} m 가 하한 {floor:.3f} m 보다 가깝다 — '
                'footprint 외접 0.189 m 를 빼면 벽까지 여유가 없다')
        return r_target

    def approach_and_align(self, bay):
        """마커 법선선 위로 옮기고, 마커를 정면으로 보고, 법선거리를 목표에 맞춘다.

        **이 단계가 옛 '창고 안쪽 0.3 m 전진' 을 대신한다.** 전진은 출발점을 모르는 상대량이라
        AMCL 오차가 그대로 남았지만, 여기는 목적지가 마커 기준 절대 위치라 출발점이 어디든
        도착점이 같다.

        **세 목적을 따로 세운다 (9/10).** 이전에는 마커를 향해 곧장 들어가는 반경방향 이동
        하나로 처리했고, 좌우 치우침은 아예 보정하지 않았다 — 좁은 칸에서 옆걸음이 옆벽으로
        민 적이 있어서다. 그런데 9/9 R1 에서 **보정하지 않은 그 횡오차가 실패의 공통 원인**
        으로 나왔다 (도착 오차 0.188·0.198 인 두 회차가 Spin 취소로 죽고, 0.145 인 회차만
        완주). 그래서 횡정렬을 독립 단계로 되살리되, **법선선이라는 명확한 기준**과 **실패를
        흡수하는 구조**를 함께 둔다.

            ① 횡정렬  회전 θ_F → 직진 |e_lat|      법선선 위로. 실패하면 생략(미션 유지)
            ② 방위정렬 회전 θ_face − θ_F            마커를 정면으로
            ③ 거리정렬 재측정 → 직진 d_n − r_target 법선방향 거리를 목표로

        Nav2 목표로는 못 한다: SimpleGoalChecker 가 xy 0.25 m · yaw 0.25 rad 라 이 크기의 이동을
        '이미 도착' 으로 처리해 로봇이 안 움직인다. 그래서 behavior_server 의 상대 이동을 쓴다.

        **aruco/pose 가 없으면 포기한다.** 정렬만 하던 때는 건너뛰어도 미션이 성립했지만
        (베이가 맞다는 것은 confirm_aruco 가 확인한다), 지금은 그 pose 가 **유일한 목적지 정보**라
        없으면 갈 곳이 없다. 주입 리그로 시험할 때는 bay_align_enable:=false 로 끈다.
        """
        if not self.p('bay_align_enable'):
            self.get_logger().warn(
                '베이 접근·정렬 꺼짐 (bay_align_enable=false) — 베이 waypoint 에 선 채로 둔다. '
                '적재 위치까지 가지 않으므로 판정 3(도달 오차)은 이 회차에 매기지 않는다')
            return
        r_target = self.approach_target()
        yaw_ref = float(bay['waypoint'][2])
        iters = max(1, int(self.p('bay_align_max_iter')))
        max_step = self.positive_parameter('bay_align_max_step_m')
        self.get_logger().info(
            f'베이 접근 — 마커 법선선 위로 옮기고, 정면으로 보고, 법선거리 {r_target:.2f} m 에 선다')
        for i in range(1, iters + 1):
            label = f'접근 {i}/{iters}'
            sample = self.marker_sample(bay['aruco_id'])
            if sample is None:
                if i > 1:
                    # 이미 접근은 했다. 다듬지 못할 뿐이라 미션을 죽일 이유가 없다 (I-026)
                    self.get_logger().warn(
                        f'{label} 에서 마커를 못 봤다 — {i - 1}회 접근한 자리에서 멈춘다')
                    break
                raise MissionAbort(
                    f'{ARUCO_POSE_TOPIC} 표본 없음 — 접근할 기준이 없다. aruco_detector 가 떠 '
                    '있는지, publish_pose 가 true 인지, 카메라 TF 가 있는지 확인할 것')
            if i == 1 and not self.behavior_server_ready():
                raise MissionAbort(
                    'behavior_server 의 spin·drive_on_heading 이 없다 — Nav2(T3) 를 확인할 것')
            here = self.fresh_pose('접근 기준 자세', not_before=self.now())
            surveyed = wrap_pi(yaw_ref - here.yaw)
            self.log_geometry(label, sample, surveyed)

            bad = self.normal_trustworthy(sample, surveyed)
            if bad is not None:
                # 법선을 못 믿으면 **움직이지 않는다.** 법선이 δ 만큼 틀리면 수선의 발이
                #   거리×sin δ 만큼 밀리므로, 잘못된 법선으로 옆으로 가는 것이 가장 나쁘다
                self.get_logger().warn(f'{label} 접근 보류 — {bad}')
                break
            if self.aligned(sample, r_target):
                self.get_logger().info(f'{label} 접근 완료 — 세 항목이 모두 허용 안이다')
                return
            budget = abs(sample.lat_error) + max(0.0, sample.normal_range - r_target)
            if budget > max_step:
                raise MissionAbort(
                    f'접근 이동량 {budget:.3f} m 가 상한 {max_step:.3f} m 를 넘는다 — 마커 측정이나 '
                    'bay_marker_range_m 를 의심할 것 (이대로 움직이면 선반에 부딪힐 수 있다)')

            turned = self.lateral_align(sample, label)
            self.face_marker(sample, turned, label)
            self.advance_to_range(bay, r_target, label, before=sample)

        final = self.marker_sample(bay['aruco_id'])
        if final is None:
            self.get_logger().warn('접근 후 마커를 다시 못 봤다 — 잔차를 남기지 못한다')
            return
        ok = self.aligned(final, r_target)
        self.get_logger().info(
            f'접근 종료 [{"충족" if ok else "미달"}] — '
            f'법선거리 {final.normal_range:.3f} m (목표 {r_target:.3f}, '
            f'잔차 {final.normal_range - r_target:+.3f}) · '
            f'법선선 이탈 {final.lat_error:+.3f} m · '
            f'방위 오차 {math.degrees(final.yaw_error):+.1f}° · '
            f'α {math.degrees(final.alpha):+.1f}°')

    def aligned(self, sample, r_target):
        """세 항목이 모두 허용치 안인가 — 법선거리 · 법선선 이탈 · 방위.

        **셋을 따로 본다.** 예전에는 반경거리 하나만 봐서, 옆으로 치우친 채 반경만 맞아도
        '도착' 이 됐다. 그 상태로 체류를 마치고 180° 를 돌면 엉뚱한 방향을 보게 되고
        `Spin` 이 충돌 예측으로 회전을 거부한다 — 9/9 R1a 가 거기서 죽었다.
        """
        return (abs(sample.normal_range - r_target)
                <= self.positive_parameter('bay_align_tol_dn_m')
                and abs(sample.lat_error) <= self.positive_parameter('bay_align_tol_lat_m')
                and abs(sample.yaw_error)
                <= math.radians(self.positive_parameter('bay_align_tol_yaw_deg')))

    def log_geometry(self, label, sample, surveyed):
        """측정 한 건의 기하를 한 줄로. 판정에 쓰는 값과 진단용 값을 함께 남긴다."""
        fx, fy = sample.foot
        self.get_logger().info(
            f'{label} — 마커 base_link ({sample.fwd:.3f}, {sample.lat:.3f}) m, '
            f'방위 {math.degrees(sample.bearing):+.1f}°, '
            f'법선 {math.degrees(sample.yaw_error):+.1f}° '
            f'(산포 {math.degrees(sample.normal_mad):.1f}°, 측량 '
            f'{math.degrees(surveyed):+.1f}°, α {math.degrees(sample.alpha):+.1f}°) / '
            f'법선거리 {sample.normal_range:.3f} · 이탈 {sample.lat_error:+.3f} · '
            f'수선의 발 ({fx:+.3f}, {fy:+.3f})')

    def dwell(self):
        dwell_s = self.p('bay_dwell_s')
        self.get_logger().info(f'적재 체류 {dwell_s} s')
        self.wait(lambda: False, timeout=dwell_s)

    def acquire_p1(self):
        """베이를 떠나기 전에 획득한다 — 여기서부터 대기공간까지가 P1 을 지나는 구간이다.

        거절은 정상이므로 실패로 세지 않는다 (계약 3절). 무한 대기는 T_lim 이 끊고,
        세마포어에도 60 초 강제 회수가 있다.
        """
        while not self.token(True):
            self.get_logger().info(f"P1 대기 — {self.p('token_retry_s')} s 후 재요청")
            self.wait(lambda: False, timeout=self.p('token_retry_s'))  # 무한 대기는 T_lim 이 끊는다
        self.token_held = True
        self.get_logger().info('P1 획득')


def main():
    rclpy.init()
    node = ShuttleMission()
    try:
        try:
            node.startup()       # 초기자세를 미션보다 먼저 (I-035). 콜백 밖 메인 루프다
        except Exception as e:   # 기동 초기화 실패로 노드를 죽이면 미션조차 못 받는다
            node.get_logger().error(f'기동 초기화 실패 — 미션 경로에서 다시 시도한다: {e}')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=TICK_S)
            if node.pending is not None and node.mission is None:
                m, node.pending = node.pending, None
                node.run_mission(m)      # 콜백 밖에서만 navigator 를 부른다
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C 는 KeyboardInterrupt, launch·systemd 의 SIGTERM 은 ExternalShutdownException 이다.
        #   둘 다 잡아야 종료 경로가 같아지고, 아래 finally 가 P1 토큰을 반납한다
        pass
    finally:
        try:
            node.release_token('노드 종료')   # Ctrl+C 로 죽어도 P1 을 물고 있지 않게
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()          # Ctrl+C 시 이미 내려간 경우 중복 호출 방지


if __name__ == '__main__':
    main()
