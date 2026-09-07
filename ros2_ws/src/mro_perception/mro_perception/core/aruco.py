"""ArUco 마커 — 검출·자세·거리의 순수 계산. ROS 에 의존하지 않는다 (단위 테스트·오프라인 검증 대상).

노드(aruco_detector_node)는 메시지 변환과 호출 순서만 맡고, 계산은 전부 여기서 한다.
tools/aruco_offline_check.py 도 같은 함수를 import 한다 — 노드와 검증 도구가 서로 다른 계산을
갖고 있으면 한쪽만 고쳐져 검증이 검증이 아니게 된다.

거리를 두 가지 방법으로 낸다. 서로 원리가 달라서 하나가 다른 하나의 자가 된다.

  depth   스테레오 시차 → 미터. 초점거리를 쓰지 않아 camera_info 결함(I-031)과 무관하다.
          보정식(depth_distance.correct_depth)은 0.8~2.0 m 에서 맞췄다. 마커 안쪽 40 % 만 읽어
          가장자리에 배경 depth 가 섞이지 않게 한다.
  pnp     마커 네 꼭짓점으로 평면 자세(solvePnP). 초점거리에 정비례하므로 camera_fx_scale 이
          틀리면 그만큼 틀린다. 대신 좌우·상하 위치와 기운각이 나온다 — depth 는 이걸 못 낸다.

발행하는 거리는 **카메라 광학중심 → 마커 중심 직선거리**다. depth 는 광축 방향(Z)이라
pnp 가 준 방향으로 환산한다(radial_from_depth). 방향은 픽셀 위치에서 오므로 fx 오차에 거의
둔감하다(기울기 15° 에서 fx 4 % 오차가 0.3 % 로 준다).

solvePnP 는 IPPE_SQUARE 로 푼다 — 정사각 평면은 해가 둘 나올 수 있어서(앞으로 기운 것과
뒤로 기운 것이 같은 그림), 재투영 오차가 작은 쪽을 고른다. 거리·위치는 두 해가 거의 같고
기운각만 갈리므로, 각도를 쓰는 쪽은 이 점을 알고 써야 한다.
"""
import math

import cv2
import numpy as np

from .depth_distance import roi_distance

DICT = cv2.aruco.DICT_4X4_50   # 계약 6절. ID 1/2/3 = 베이 A/B/C, 4 = 측위 검증, 10~19 = Phase 2 예약
DEPTH_INNER = 0.4              # 거리 측정에 쓰는 마커 안쪽 비율. 바깥은 흰 여백·배경이 섞인다
MAX_CORE_GAP_MM = 150          # amr_detection 과 같은 품질 게이트 — ROI 핵심부(p20~median)가 이보다 벌어지면 불신


def make_detector(refine=True):
    """refine=True 면 코너를 서브픽셀로 다듬는다 — 회전(yaw) 추정이 코너 1 px 에 크게 흔들려서다."""
    params = cv2.aruco.DetectorParameters()
    if refine:
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICT), params)


def detect(detector, frame, allowed):
    """허용 ID 의 마커를 큰 것(가까운 것)부터. → [(id, corners(4,2) float32)]

    허용 목록 밖은 여기서 버린다 — 예약 ID(10~19)가 베이로 오인되면 안 되고, ID 4 를 빼면
    관제의 측위 검증이 죽는다(계약 9.2 ①). 목록은 파라미터(marker_ids)가 정한다.
    """
    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None:
        return []
    found = [(int(i[0]), c.reshape(4, 2).astype(np.float32))
             for i, c in zip(ids, corners) if int(i[0]) in allowed]
    found.sort(key=lambda p: -cv2.contourArea(p[1]))
    return found


def object_points(size_m):
    """solvePnP 용 마커 평면 좌표. ArUco 꼭짓점 순서(좌상·우상·우하·좌하)와 같아야 한다."""
    h = size_m / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)


def rotation_gap_deg(rvec_a, rvec_b):
    """두 회전 사이의 각 (deg). 이전 프레임과 얼마나 다른 자세인지 잰다."""
    Ra, _ = cv2.Rodrigues(np.asarray(rvec_a, dtype=np.float64).reshape(3, 1))
    Rb, _ = cv2.Rodrigues(np.asarray(rvec_b, dtype=np.float64).reshape(3, 1))
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def solve_pose(corners, K, D, size_m, prev_rvec=None, tie_px=3.0):
    """마커 자세. → dict(tvec, rvec, dist, z, angle_deg, reproj_px) · 실패하면 None

    dist  카메라 광학중심 → 마커 중심 직선거리 (m)
    z     광축 방향 성분 (m). depth 와 같은 의미
    angle 마커 법선과 카메라 시선의 각 (deg). 계약 6절 '정면 ±30°' 확인용

    prev_rvec 를 주면, 재투영 오차가 tie_px 안으로 비슷한 후보들 사이에서는 **이전 프레임 자세에
    가까운 것**을 고른다. 정사각 마커는 앞으로 기운 것과 뒤로 기운 것이 거의 같은 그림이라 두 해의
    오차가 1 px 안에서 엎치락뒤치락하고, 오차만 보면 프레임마다 기울기 부호가 뒤집힌다. 거리·위치는
    두 해가 같지만 aruco/pose 의 회전을 정렬에 쓰려면 이 흔들림을 잡아야 한다.
    """
    obj = object_points(size_m)
    corners = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    cands = []
    try:
        n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, corners, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        cands += list(zip(rvecs, tvecs))
        # IPPE 는 마커가 광축 정중앙에 정면으로 놓이면(잡음도 왜곡도 없는 합성 입력) 퇴화해 엉뚱한 자세를
        # 내기도 한다. 반복법은 그런 대칭 배치에서도 정확하므로 후보에 같이 넣고 재투영 오차로 고른다
        ok, rvec, tvec = cv2.solvePnP(obj, corners, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
        if ok:
            cands.append((rvec, tvec))
    except cv2.error:
        return None

    scored = []
    for rvec, tvec in cands:
        tvec = np.asarray(tvec, dtype=np.float64).ravel()
        rvec = np.asarray(rvec, dtype=np.float64).ravel()
        if not (np.all(np.isfinite(tvec)) and np.all(np.isfinite(rvec))) or tvec[2] <= 0:
            continue
        R, _ = cv2.Rodrigues(rvec)
        if R[2, 2] >= 0:
            continue          # 마커 면이 카메라 반대쪽을 향한 해 — 보이는 마커일 수 없다 (거울상 해)
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
        err = float(np.sqrt(((proj.reshape(4, 2) - corners) ** 2).sum(axis=1)).mean())
        scored.append((err, rvec, tvec, R))
    if not scored:
        return None
    best_err = min(e for e, *_ in scored)
    near = [c for c in scored if c[0] <= best_err + tie_px]
    if prev_rvec is not None and len(near) > 1:
        near.sort(key=lambda c: rotation_gap_deg(c[1], prev_rvec))
    else:
        near.sort(key=lambda c: c[0])
    err, rvec, tvec, R = near[0]
    return {
        'tvec': tvec, 'rvec': rvec,
        'dist': float(np.linalg.norm(tvec)),
        'z': float(tvec[2]),
        'angle_deg': float(np.degrees(np.arccos(np.clip(abs(R[2, 2]), -1.0, 1.0)))),
        'reproj_px': err,
    }


def marker_depth(depth_mm, corners, scale=1.0):
    """마커 안쪽 정사각 ROI 의 depth 거리. → roi_distance 의 dict · 부족하면 None

    scale : RGB 픽셀 → depth 픽셀 배율 (둘의 해상도가 같으면 1.0)
    핵심부 간격(med − q)이 MAX_CORE_GAP_MM 을 넘으면 None — 가장자리에 배경이 섞인 프레임이다.
    """
    cx, cy = corners.mean(axis=0) * scale
    side = float(np.mean([np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)])) * scale
    half = max(2, int(side * DEPTH_INNER / 2.0))
    r = roi_distance(depth_mm, int(cx) - half, int(cy) - half, int(cx) + half + 1, int(cy) + half + 1)
    if r is None or (r['med'] - r['q']) > MAX_CORE_GAP_MM:
        return None
    return r


def radial_from_depth(z_depth, tvec):
    """depth 의 광축 방향 거리 → 카메라→마커 직선거리. 방향은 pnp 의 tvec 에서 가져온다."""
    return float(z_depth * np.linalg.norm(tvec) / tvec[2])


def rvec_to_quat(rvec):
    """Rodrigues 벡터 → (x, y, z, w). geometry_msgs 순서."""
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    tr = float(np.trace(R))
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)
