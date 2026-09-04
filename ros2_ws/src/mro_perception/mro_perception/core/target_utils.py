"""
target_utils.py — 픽셀 → 카메라 3D → map 좌표 → standoff 목표. click_to_goal / amr_control 공용.
"""
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


def to_map(tf_buffer, pt, logger=None):
    """camera → map. 메시지 시각 TF 가 없으면(시계 어긋남) 최신 TF 로 재시도. 실패 시 None."""
    last_err = None
    for stamp in (pt.header.stamp, Time().to_msg()):
        pt.header.stamp = stamp
        try:
            return tf_buffer.transform(pt, MAP_FRAME, timeout=TF_TIMEOUT)
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
    """
    로봇 (rx,ry) → 물체 (tx,ty) 방향으로, 물체 앞 standoff m 지점을 목표로.
    이미 그보다 가까우면 제자리에서 물체를 바라보게만.
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
