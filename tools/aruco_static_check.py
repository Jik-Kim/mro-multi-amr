#!/usr/bin/env python3
"""R0 정지 측정 — 로봇을 세워 두고 줄자값과 검출기 값을 한 지점씩 대조한다 (I-044).

  python3 aruco_static_check.py --ns /robot3 --front 0.60 --rear 0.98
  python3 aruco_static_check.py --ns /robot8 --front 1.20 --rear 1.58 --n 80

로봇을 움직이지 않는다. 듣기만 하고 파라미터를 바꾸지 않으며 임시 노드는 종료와 함께 사라진다.
결과는 화면과 --csv(기본 aruco_static.csv) 양쪽에 남는다 — 지점마다 한 줄씩 쌓인다.

**줄자는 앞뒤 두 번 잰다.** `base_link` 는 바닥 원판의 중심이라 자를 댈 데가 없다. 앞 끝과
뒤 끝을 재서 평균하면 중심이 나오고, **로봇 상수를 하나도 안 써도 된다.**

    d_base = (front + rear) / 2          ← 이것이 base_link 까지의 참값
    radius = (rear - front) / 2          ← 덤. Nav2 footprint 가 전제하는 0.189 와 맞는지

반지름이 어긋나면 워치독 `clear_dist` 와 `DriveOnHeading` 충돌 예측이 전부 그 위에 서 있으므로
같이 흔들린다. 공짜로 얻는 검증이니 매 지점 기록한다.

**렌즈면을 기준으로 재지 않는다.** 기준점이 셋이라 헷갈리기 쉽다:
    ① base_link                     Nav2 footprint · 워치독 · 우리 접근 계산
    ② oakd_rgb_camera_optical_frame TF. URDF 마지막 joint 가 xyz="0 0 0" 이라 OAK-D 의
                                    **CAD 장착점**이지 렌즈 광학중심이 아니다
    ③ 물리 렌즈면                    인지가 depth_k 를 여기 맞춰 적합했다 (robot8.yaml)
②↔③ 틈은 아무도 모델링하지 않았고 그대로 base_x 에 실린다. 이 도구는 ① 로 통일해서 잰다.

내는 값
  d_base    (front+rear)/2 [m] — 기준
  dist      aruco/distance 중앙값 — depth 에 로봇별 k 를 건 값 (③ 기준으로 보정됨)
  pose_r    aruco/pose 위치 크기 √(x²+y²+z²) — solvePnP tvec × (depth/pnp 비)
  lat       base_link 기준 좌우 치우침 (왼쪽 +). TF 를 안 탄다 — URDF 상 카메라 y 병진이
            정확히 0 이라 -pose.position.x 가 그대로 좌우다. 정면으로 세웠으면 0 이어야 한다

읽는 법 (가이드 F-3) — 기준점 모호성은 **절편에만** 들어가고 기울기는 못 건드린다
  dist - d_base 가 네 지점에서 일정          → (b) 기준점. 그 상수를 빼면 끝난다
  거리 따라 커지거나 0.8 m 에서 꺾인다        → (c) depth_k 유효구간 밖. 절편과 무관하게 결론
  일정한데 그 값이 0.2 m 급                   → ②③ 로는 설명 안 된다. (d) 또는 검출기 자체
"""
import argparse
import csv
import math
import os
import statistics as st

from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32

FOOTPRINT_RADIUS = 0.189   # nav2.yaml 의 footprint 외접. 이것과 어긋나면 안전 여유가 전부 흔들린다


class Collector(Node):
    """검출기 토픽(id·distance·pose)을 N 프레임 모으는 임시 구독 노드."""

    def __init__(self, ns, want):
        """주어진 네임스페이스 아래에서 want 프레임을 모을 준비를 한다."""
        super().__init__('aruco_static_check_%d' % os.getpid(), namespace=ns)
        self.want = want
        self.ids, self.dists, self.poses = [], [], []
        self.create_subscription(Int32, f'{ns}/aruco/id', lambda m: self.ids.append(m.data), 10)
        self.create_subscription(Float32, f'{ns}/aruco/distance', lambda m: self.dists.append(m.data), 10)
        self.create_subscription(PoseStamped, f'{ns}/aruco/pose', self.poses.append, 10)

    def done(self):
        """거리·자세 둘 다 목표 프레임 수를 채웠나."""
        return len(self.dists) >= self.want and len(self.poses) >= self.want

    def lateral(self):
        """base_link 기준 좌우 치우침 [m]. 왼쪽이 +.

        **TF 를 타지 않는다.** URDF 상 카메라의 y 병진이 정확히 0 이고 광학→base 회전이
        고정([-0.5,0.5,-0.5,0.5], 즉 광학 (px,py,pz) -> base (pz,-px,-py))이라 좌우는
        그냥 -pose.position.x 다. 전방거리는 tf 의 x 병진 -0.060 만 빼면 되므로,
        이 도구는 tf_static 을 아예 구독하지 않는다 (9/9 — 네임스페이스 밖에서 못 받았고,
        받아 봐야 산술로 이미 정해진 값이었다).
        """
        return st.median(-m.pose.position.x for m in self.poses)


def main():
    """한 지점을 재서 화면과 csv 에 남긴다."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='/robot3')
    ap.add_argument('--front', type=float, required=True, help='마커 벽 → 로봇 앞 끝 [m]')
    ap.add_argument('--rear', type=float, required=True, help='마커 벽 → 로봇 뒤 끝 [m]')
    ap.add_argument('--n', type=int, default=50, help='모을 프레임 수')
    ap.add_argument('--timeout', type=float, default=20.0)
    ap.add_argument('--csv', default='aruco_static.csv')
    a = ap.parse_args()

    d_base = (a.front + a.rear) / 2.0
    radius = (a.rear - a.front) / 2.0

    rclpy.init()
    node = Collector(a.ns.rstrip('/'), a.n)
    t0 = node.get_clock().now().nanoseconds / 1e9
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds / 1e9 - t0 > a.timeout:
            break

    n = min(len(node.dists), len(node.poses))
    if n == 0:
        print(f'프레임을 하나도 못 받았다 — {a.ns}/aruco/distance 와 aruco/pose 를 확인할 것')
        print('  마커가 시야에 있는가 · 검출기가 떠 있는가 · ROS_SUPER_CLIENT 는 True 인가')
        rclpy.shutdown()
        return 1

    dist = st.median(node.dists)
    rs = [math.hypot(math.hypot(m.pose.position.x, m.pose.position.y), m.pose.position.z)
          for m in node.poses]
    pose_r = st.median(rs)
    lat = node.lateral()
    marker_id = st.mode(node.ids) if node.ids else -1

    print(f'\n== {a.ns}  앞 {a.front:.3f} / 뒤 {a.rear:.3f}  ->  base_link {d_base:.3f} m '
          f'(프레임 {n}, ID {marker_id}) ==')
    print(f'  반지름 검증     {radius:7.3f}   (footprint 전제 {FOOTPRINT_RADIUS:.3f}, '
          f'차 {radius - FOOTPRINT_RADIUS:+.3f})'
          + ('   <- 어긋난다. 안전 여유가 전부 이 값 위에 있다'
             if abs(radius - FOOTPRINT_RADIUS) > 0.02 else ''))
    print(f'  aruco/distance  {dist:7.3f}   (base 대비 {dist - d_base:+.3f})')
    print(f'  pose 크기       {pose_r:7.3f}   (base 대비 {pose_r - d_base:+.3f})')
    print(f'  좌우 치우침     {lat:+7.3f}   정면으로 세웠다면 0 이어야 한다 (I-039)')
    print(f'  distance 산포    {min(node.dists):.3f} ~ {max(node.dists):.3f}'
          f'   pose 산포 {min(rs):.3f} ~ {max(rs):.3f}')

    new = not os.path.exists(a.csv)
    with open(a.csv, 'a', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        if new:
            w.writerow(['ns', 'front', 'rear', 'd_base', 'radius', 'id', 'n',
                        'dist', 'pose_r', 'lat', 'dist-base', 'pose_r-base'])
        w.writerow([a.ns, a.front, a.rear, round(d_base, 4), round(radius, 4), marker_id, n,
                    round(dist, 4), round(pose_r, 4), round(lat, 4),
                    round(dist - d_base, 4), round(pose_r - d_base, 4)])
    print(f'  → {a.csv} 에 한 줄 추가')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
