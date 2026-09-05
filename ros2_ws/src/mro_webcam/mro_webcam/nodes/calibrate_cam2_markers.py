#!/usr/bin/env python3
import json
from pathlib import Path

import cv2
import numpy as np


# =========================
# 사용자 설정
# =========================
PACKAGE_DIR = Path(__file__).resolve().parents[2]
IMAGE_PATH = PACKAGE_DIR / "resource" / "기준이미지.png"
OUTPUT_JSON = PACKAGE_DIR / "config" / "webcam2_homography.json"
OUTPUT_PREVIEW = PACKAGE_DIR / "config" / "webcam2_homography_points.png"

# 민트색 3x3 마커 검출용 HSV 범위
# 현재 기준이미지의 민트색 사각형에 맞춘 초기값.
HSV_LOWER = np.array([80, 150, 150], dtype=np.uint8)
HSV_UPPER = np.array([100, 255, 255], dtype=np.uint8)

# 3x3 정사각형이면 이상적으로 area=9.
# 압축/이미지 변환을 고려하여 약간의 여유를 둔다.
MIN_AREA = 5
MAX_AREA = 20

# 같은 행으로 묶을 y좌표 허용 오차
ROW_TOLERANCE_PX = 8


def unit_scale(unit: str) -> float:
    unit = unit.strip().lower()
    table = {
        "m": 1.0,
        "cm": 0.01,
        "mm": 0.001,
    }
    if unit not in table:
        raise ValueError("단위는 m, cm, mm 중 하나여야 합니다.")
    return table[unit]


def detect_mint_markers(image):
    """민트색 3x3 사각형의 중심 픽셀 좌표를 검출한다."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

    n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        mask, connectivity=8
    )

    points = []

    for i in range(1, n_labels):  # 0은 background
        area = int(stats[i, cv2.CC_STAT_AREA])
        width = int(stats[i, cv2.CC_STAT_WIDTH])
        height = int(stats[i, cv2.CC_STAT_HEIGHT])

        if not (MIN_AREA <= area <= MAX_AREA):
            continue

        # 작은 정사각형 성분만 허용
        if width > 6 or height > 6:
            continue

        cx, cy = centroids[i]
        points.append((float(cx), float(cy)))

    return points, mask


def sort_points_by_rows(points, row_tolerance=ROW_TOLERANCE_PX):
    """
    점들을 화면 위->아래, 각 행에서는 왼쪽->오른쪽 순으로 정렬한다.
    원근 때문에 같은 행의 y가 몇 px 차이날 수 있으므로 row clustering을 수행한다.
    """
    points = sorted(points, key=lambda p: p[1])

    rows = []
    for point in points:
        placed = False

        for row in rows:
            row_y = np.mean([p[1] for p in row])
            if abs(point[1] - row_y) <= row_tolerance:
                row.append(point)
                placed = True
                break

        if not placed:
            rows.append([point])

    rows.sort(key=lambda row: np.mean([p[1] for p in row]))

    sorted_points = []
    for row in rows:
        row.sort(key=lambda p: p[0])
        sorted_points.extend(row)

    return sorted_points


def draw_points(image, points):
    preview = image.copy()

    for i, (u, v) in enumerate(points, start=1):
        u_i = int(round(u))
        v_i = int(round(v))

        cv2.circle(preview, (u_i, v_i), 6, (0, 0, 255), 1)
        cv2.putText(
            preview,
            f"P{i}",
            (u_i + 7, v_i - 7),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )

    return preview


def collect_map_points(image_points):
    print()
    print("=== 실측 좌표 입력 ===")
    print("각 점의 실제 바닥 좌표 (X, Y)를 같은 기준 원점/축으로 입력하세요.")
    print("예: ROS map 좌표를 직접 쓸 경우 map 기준 X,Y를 입력합니다.")
    print()

    while True:
        unit = input("실측 단위 [m/cm/mm]: ").strip().lower()
        try:
            scale = unit_scale(unit)
            break
        except ValueError as e:
            print(e)

    map_points_m = []

    for i, (u, v) in enumerate(image_points, start=1):
        while True:
            raw = input(
                f"P{i:02d} pixel=({u:.1f}, {v:.1f}) -> 실제 X Y [{unit}]: "
            ).strip()

            try:
                parts = raw.replace(",", " ").split()
                if len(parts) != 2:
                    raise ValueError

                x = float(parts[0]) * scale
                y = float(parts[1]) * scale
                map_points_m.append((x, y))
                break

            except ValueError:
                print("  입력 형식 오류. 예: 125.0 340.0")

    return map_points_m, unit


def compute_homography(image_points, map_points):
    src = np.asarray(image_points, dtype=np.float64)
    dst = np.asarray(map_points, dtype=np.float64)

    if len(src) < 4:
        raise RuntimeError("Homography 계산에는 최소 4개의 대응점이 필요합니다.")

    # 모든 대응점을 최소제곱으로 사용
    H, mask = cv2.findHomography(src, dst, method=0)

    if H is None:
        raise RuntimeError("Homography 계산에 실패했습니다.")

    # reprojection error
    src_h = src.reshape(-1, 1, 2)
    projected = cv2.perspectiveTransform(src_h, H).reshape(-1, 2)

    errors = np.linalg.norm(projected - dst, axis=1)
    rmse = float(np.sqrt(np.mean(errors ** 2)))

    return H, projected, errors, rmse


def main():
    image_path = Path(IMAGE_PATH)
    image = cv2.imread(str(image_path))

    if image is None:
        raise FileNotFoundError(f"이미지를 열 수 없습니다: {image_path}")

    points, mask = detect_mint_markers(image)
    points = sort_points_by_rows(points)

    print(f"검출된 민트색 마커 수: {len(points)}")

    for i, (u, v) in enumerate(points, start=1):
        print(f"P{i:02d}: pixel=({u:.1f}, {v:.1f})")

    if len(points) < 4:
        raise RuntimeError(
            "마커가 4개 미만입니다. HSV 범위 또는 마커 크기 조건을 확인하세요."
        )

    preview = draw_points(image, points)
    cv2.imwrite(str(OUTPUT_PREVIEW), preview)

    # 화면으로 번호 확인
    cv2.imshow("Calibration points - press any key", preview)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    map_points, input_unit = collect_map_points(points)

    H, projected, errors, rmse = compute_homography(points, map_points)

    print()
    print("=== Homography ===")
    print(H)

    print()
    print("=== Reprojection error ===")
    for i, err in enumerate(errors, start=1):
        print(f"P{i:02d}: {err:.4f} m")

    print(f"RMSE: {rmse:.4f} m")

    output = {
        "image_size": {
            "width": int(image.shape[1]),
            "height": int(image.shape[0]),
        },
        "input_unit": input_unit,
        "coordinate_unit": "m",
        "image_points": [
            {
                "id": f"P{i}",
                "u": float(u),
                "v": float(v),
            }
            for i, (u, v) in enumerate(points, start=1)
        ],
        "map_points": [
            {
                "id": f"P{i}",
                "x": float(x),
                "y": float(y),
            }
            for i, (x, y) in enumerate(map_points, start=1)
        ],
        "homography": H.tolist(),
        "reprojection_error_m": {
            f"P{i}": float(err)
            for i, err in enumerate(errors, start=1)
        },
        "rmse_m": rmse,
    }

    with open(OUTPUT_JSON, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2, ensure_ascii=False)

    print()
    print(f"저장 완료: {OUTPUT_JSON}")
    print(f"포인트 번호 이미지: {OUTPUT_PREVIEW}")


if __name__ == "__main__":
    main()
