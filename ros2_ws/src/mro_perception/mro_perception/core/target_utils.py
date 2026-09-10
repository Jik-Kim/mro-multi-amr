"""픽셀 → 카메라 3D → map 좌표 → standoff 목표. click_to_goal / amr_control 공용."""
import math

from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from rclpy.duration import Duration
from rclpy.time import Time
import tf2_geometry_msgs  # noqa: F401  (PointStamped 변환 등록)

MAP_FRAME = 'map'
BASE_FRAME = 'base_link'
TF_TIMEOUT = Duration(seconds=1.0)


def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def back_project(K, u, v, z, frame_id, stamp):
    """픽셀 (u,v) + 거리 z(m) → 광학 프레임 PointStamped (X 우, Y 하, Z 전방)."""
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    pt = PointStamped()
    pt.header.frame_id = frame_id
    pt.header.stamp = stamp
    pt.point.x = (u - cx) * z / fx
    pt.point.y = (v - cy) * z / fy
    pt.point.z = z
    return pt


def to_map(tf_buffer, pt, logger=None, timeout=TF_TIMEOUT):
    """카메라 좌표를 map 좌표로. 메시지 시각 TF 가 없으면(시계 어긋남) 최신 TF 로 재시도. 실패 시 None.

    timeout — tf2_ros 는 기다리는 동안 호출 스레드를 잠재운다(sleep 루프, 실행기 정지). 단일 스레드 노드에서는
    그 사이 /tf 콜백이 돌지 않아 아무리 기다려도 변환이 도착하지 않고 시간만 잃는다 — amr_detection 에서 탐지된
    프레임마다 1 s × 2회 = 2 s 가 멈춰 렉의 원인이었다 (9/8 실측 proc max 2036 ms). 그런 노드는 Duration()(0) 을
    넘겨 즉시 판정하고 그 프레임을 건너뛴다. 기본값 1 s 는 다른 호출자(follow_target)의 이관 당시 동작 유지.
    """
    last_err = None
    for stamp in (pt.header.stamp, Time().to_msg()):
        pt.header.stamp = stamp
        try:
            return tf_buffer.transform(pt, MAP_FRAME, timeout=timeout)
        except Exception as e:
            last_err = e
    if logger:
        logger.warn(f'TF: {last_err}')
    return None


def robot_xy(tf_buffer, logger=None):
    try:
        t = tf_buffer.lookup_transform(MAP_FRAME, BASE_FRAME, Time(), timeout=TF_TIMEOUT)
        return t.transform.translation.x, t.transform.translation.y
    except Exception as e:
        if logger:
            logger.warn(f'TF: {e}')
        return None


def standoff_goal(rx, ry, tx, ty, standoff, stamp):
    """물체 앞 standoff m 지점을 바라보는 목표 자세를 만든다.

    로봇 (rx,ry) → 물체 (tx,ty) 방향으로 잡되, 이미 그보다 가까우면 제자리에서 물체를 바라보게만 한다.
    반환: (PoseStamped, 거리, yaw)
    """
    dx, dy = tx - rx, ty - ry
    dist = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    if dist > standoff:
        gx, gy = tx - standoff * dx / dist, ty - standoff * dy / dist
    else:
        gx, gy = rx, ry
    goal = PoseStamped()
    goal.header.frame_id = MAP_FRAME
    goal.header.stamp = stamp
    goal.pose.position.x, goal.pose.position.y = gx, gy
    goal.pose.orientation = yaw_to_quat(yaw)
    return goal, dist, yaw
