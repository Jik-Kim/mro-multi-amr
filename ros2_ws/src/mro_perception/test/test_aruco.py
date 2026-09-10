"""core/aruco.py 단위 테스트 — 검출·자세·거리. ROS 없이 합성 코너로 돈다.

무엇을 지키나
  · solvePnP 거리가 참값을 되찾고, 기울여도 흔들리지 않는다 (핀홀 근사를 버린 이유)
  · camera_info 의 fx 가 r 배 작으면 pnp 거리도 정확히 r 배 작다 — I-031 의 증상 그대로
  · 허용 목록 밖 ID 는 검출 단계에서 빠진다 (예약 ID 10~19 가 베이로 오인되면 안 된다)
  · 쿼터니언 변환이 회전행렬과 같은 회전이다 (aruco/pose 를 켤 때 쓴다)

실기 정확도는 여기서 보장하지 않는다 — 그건 줄자 대조(TS-U05)가 한다. 9/9 체커보드로 fx 를 실측한 뒤
depth 경로를 걷어냈으므로 depth 관련 시험도 함께 뺐다.

실행: ROS 없이 돈다
  cd ros2_ws/src/mro_perception && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_aruco.py
  (venv 의 launch_testing 플러그인이 pytest 9 와 충돌해 자동 로드를 끈다. colcon test 는 그대로 된다)
"""
import cv2
import numpy as np
import pytest

from mro_perception.core import aruco

SIZE = 0.10
K = np.array([[1038.0, 0.0, 352.5],
              [0.0, 1038.0, 352.5],
              [0.0, 0.0, 1.0]])                  # 9/9 체커보드 실측 fx (robot3 1037.9 · robot8 1038.9)
D = np.zeros((5, 1))


def marker_rotation(tilt_deg):
    """카메라를 정면으로 보는 마커의 회전. OpenCV 관례상 마커 z 가 카메라 쪽을 향하므로 x 축 180° 를 깔고
    그 위에 y 축 기울기를 얹는다. 이렇게 해야 투영된 코너 순서가 ArUco 가 주는 순서(좌상·우상·우하·좌하)와 같다."""
    rx = cv2.Rodrigues(np.array([np.pi, 0.0, 0.0]))[0]
    ry = cv2.Rodrigues(np.array([0.0, np.deg2rad(tilt_deg), 0.0]))[0]
    return rx @ ry


def corners_at(x=0.0, y=0.0, z=1.0, tilt_deg=0.0, K=K):
    rvec = cv2.Rodrigues(marker_rotation(tilt_deg))[0]
    pts, _ = cv2.projectPoints(aruco.object_points(SIZE), rvec, np.array([x, y, z]), K, D)
    return pts.reshape(4, 2).astype(np.float32)


def test_corner_order_matches_aruco_convention():
    """테스트 생성기의 코너 0 이 영상 좌상단이어야 solve_pose 검증이 의미가 있다."""
    c = corners_at()
    assert c[0, 0] < K[0, 2] and c[0, 1] < K[1, 2]          # 좌상
    assert c[2, 0] > K[0, 2] and c[2, 1] > K[1, 2]          # 우하


@pytest.mark.parametrize('z', [0.5, 1.0, 1.5])
def test_solve_pose_recovers_distance(z):
    pose = aruco.solve_pose(corners_at(z=z), K, D, SIZE)
    assert pose is not None
    assert abs(pose['dist'] - z) < 0.005
    assert abs(pose['z'] - z) < 0.005


@pytest.mark.parametrize('tilt', [0, 30, 60])
def test_solve_pose_is_tilt_invariant(tilt):
    """기울어도 거리는 그대로, 기운각은 맞게 — 핀홀 근사(한 변 픽셀)로는 30° 에서 10 cm 틀렸다."""
    pose = aruco.solve_pose(corners_at(z=1.0, tilt_deg=tilt), K, D, SIZE)
    assert abs(pose['dist'] - 1.0) < 0.01
    assert abs(pose['angle_deg'] - tilt) < 3.0


def test_solve_pose_lateral_offset():
    pose = aruco.solve_pose(corners_at(x=0.20, z=1.0), K, D, SIZE)
    assert abs(pose['tvec'][0] - 0.20) < 0.005
    assert abs(pose['dist'] - np.hypot(0.20, 1.0)) < 0.005


def test_fx_error_scales_pnp_distance_by_same_ratio():
    """I-031: camera_info 의 fx 가 1.7778 배 작으면 pnp 거리도 1.7778 배 작다. 보정 계수의 근거."""
    K_bad = K.copy()
    K_bad[0, 0] /= 16 / 9
    K_bad[1, 1] /= 16 / 9
    pose = aruco.solve_pose(corners_at(z=1.35), K_bad, D, SIZE)
    assert abs(pose['dist'] - 1.35 / (16 / 9)) < 0.01


def test_solve_pose_rejects_garbage():
    bad = np.array([[0, 0], [0, 0], [0, 0], [0, 0]], np.float32)
    assert aruco.solve_pose(bad, K, D, SIZE) is None


def _scene_with_marker(mid, px=120):
    """회색 배경 704×704 에 마커 하나를 붙인 합성 영상."""
    img = np.full((704, 704), 110, np.uint8)
    tex = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(aruco.DICT), mid, px)
    q = px // 2
    y0, x0 = 300, 300
    img[y0 - q:y0 + px + q, x0 - q:x0 + px + q] = 255
    img[y0:y0 + px, x0:x0 + px] = tex
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def test_detect_filters_by_allowed_ids():
    det = aruco.make_detector()
    img = _scene_with_marker(4)
    found = aruco.detect(det, img, {1, 2, 3, 4})
    assert [f[0] for f in found] == [4]
    assert found[0][1].shape == (4, 2)
    assert aruco.detect(det, img, {1, 2, 3}) == []          # ID 4 를 빼면 안 보인다 (관제 FR-35 가 죽는 상황)
    assert aruco.detect(det, _scene_with_marker(12), {1, 2, 3, 4}) == []   # 예약 ID 는 통과 못 한다


def test_prev_rvec_breaks_tie_between_mirror_solutions():
    """기울어진 마커의 두 해(앞으로/뒤로 기움)가 오차 동점일 때 이전 프레임 자세에 가까운 쪽을 고른다."""
    rng = np.random.default_rng(3)
    corners = corners_at(z=1.0, tilt_deg=20) + rng.normal(0, 0.15, (4, 2)).astype(np.float32)
    true_rvec = cv2.Rodrigues(marker_rotation(20))[0].ravel()
    mirror_rvec = cv2.Rodrigues(marker_rotation(-20))[0].ravel()
    a = aruco.solve_pose(corners, K, D, SIZE, prev_rvec=true_rvec, tie_px=5.0)
    b = aruco.solve_pose(corners, K, D, SIZE, prev_rvec=mirror_rvec, tie_px=5.0)
    assert aruco.rotation_gap_deg(a['rvec'], true_rvec) < 8
    assert aruco.rotation_gap_deg(b['rvec'], mirror_rvec) < 8
    assert abs(a['dist'] - b['dist']) < 0.01              # 거리는 두 해가 같다 — 그래서 거리에는 영향이 없다


def test_rvec_to_quat_matches_rotation_matrix():
    rng = np.random.default_rng(0)
    for _ in range(20):
        rvec = rng.normal(size=3)
        R, _ = cv2.Rodrigues(rvec)
        x, y, z, w = aruco.rvec_to_quat(rvec)
        assert abs(np.hypot(np.hypot(x, y), np.hypot(z, w)) - 1.0) < 1e-6
        # 쿼터니언으로 돌린 벡터가 회전행렬로 돌린 벡터와 같아야 한다
        v = rng.normal(size=3)
        q = np.array([x, y, z])
        v_q = v + 2 * np.cross(q, np.cross(q, v) + w * v)
        assert np.allclose(v_q, R @ v, atol=1e-6)
