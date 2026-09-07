#!/usr/bin/env python3
"""amr_detection 렉 진단 — 노드가 도는 PC 에서 실행. 추측 대신 숫자로 원인 자리를 가른다 (README 7절).

  python3 tools/diag_lag.py                # /robot3, 10 초  (docs/setup.md '터미널마다 맨 처음' 을 한 터미널에서)
  python3 tools/diag_lag.py --ns /robot4 --seconds 15

재는 것 (노드는 그대로 두고 옆에서 듣기만 한다)
  입력  oakd/rgb·depth        Hz · 대역폭 · 지연(카메라 stamp → 지금)     ← RPi·Wi-Fi 쪽
  경쟁  /tf                   Hz                                            ← 카메라 콜백과 같은 실행기에서 경쟁하는 양
  노드  target/visible        Hz  (= 실제 처리 속도)
        [diag] 로그           /rosout 에서 수집 — 루프·콜백·나이·process 시간
  출력  target/image          Hz · 대역폭                                    ← rqt 가 다른 PC 면 Wi-Fi 부담
  환경  SSH·DISPLAY·GPU·프로세스 CPU

찌꺼기 없음 — 파일을 쓰지 않고, 파라미터를 바꾸지 않으며, 임시 노드(diag_lag_<pid>)는 종료와 함께 사라진다.
지연값 주의 — 카메라 stamp 는 RPi 시계, 지금은 이 PC 시계다. 음수거나 수 초면 두 시계가 어긋난 것이지 렉이 아니다.
"""
import argparse
import os
import shutil
import subprocess
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Log
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


def sh(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception as e:  # noqa: BLE001
        return f'(실패: {e})'


class Diag(Node):
    def __init__(self, ns, seconds):
        super().__init__(f'diag_lag_{os.getpid()}')
        self.ns, self.seconds = ns.rstrip('/'), seconds
        self.n = defaultdict(int)          # 토픽 -> 메시지 수
        self.b = defaultdict(int)          # 토픽 -> 바이트
        self.delay = defaultdict(list)     # 토픽 -> [stamp→now 초]
        self.diag_lines = []
        self.t0 = None
        qos = qos_profile_sensor_data     # BEST_EFFORT 구독자는 RELIABLE 발행자도 받는다
        self.create_subscription(CompressedImage, f'{self.ns}/oakd/rgb/image_raw/compressed',
                                 lambda m: self._img('rgb', m, len(m.data)), qos)
        self.create_subscription(CompressedImage, f'{self.ns}/oakd/stereo/image_raw/compressedDepth',
                                 lambda m: self._img('depth', m, len(m.data)), qos)
        self.create_subscription(TFMessage, f'{self.ns}/tf', lambda m: self._cnt('tf', m, 0), qos)
        self.create_subscription(Bool, f'{self.ns}/target/visible', lambda m: self._cnt('visible', m, 1), qos)
        self.create_subscription(Image, f'{self.ns}/target/image',
                                 lambda m: self._img('out_image', m, len(m.data)), qos)
        self.create_subscription(Log, '/rosout', self._rosout, 10)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _cnt(self, k, m, nbytes):
        if self.t0 is None:
            self.t0 = time.monotonic()
        self.n[k] += 1
        self.b[k] += nbytes

    def _img(self, k, m, nbytes):
        self._cnt(k, m, nbytes)
        st = m.header.stamp
        if st.sec or st.nanosec:
            self.delay[k].append(self._now() - (st.sec + st.nanosec / 1e9))

    def _rosout(self, m):
        if 'amr_detection' in m.name and '[diag' in m.msg:
            self.diag_lines.append(m.msg)


def stats(v):
    if not v:
        return '—'
    v = sorted(v)
    return f'avg {sum(v) / len(v) * 1000:.0f}  p95 {v[int(len(v) * 0.95) - 1] * 1000:.0f}  max {v[-1] * 1000:.0f} ms'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='/robot3')
    ap.add_argument('--seconds', type=float, default=10.0)
    a = ap.parse_args()

    print('=== 환경 ===')
    env = os.environ.get
    print(f"host {sh('hostname')} | ROS_DOMAIN_ID={env('ROS_DOMAIN_ID')} "
          f"| DISCOVERY={env('ROS_DISCOVERY_SERVER', '')} | SUPER_CLIENT={env('ROS_SUPER_CLIENT', '')}")
    print(f"SSH 접속={'예' if os.environ.get('SSH_CONNECTION') else '아니오'} | DISPLAY={os.environ.get('DISPLAY', '(없음)')}"
          f" — SSH 로 창을 띄우면 X 포워딩이라 imshow 가 느리다")
    # python 프로세스만, 그리고 이 스크립트와 그 부모 셸(명령줄에 같은 문자열이 들어감)은 제외
    me = {str(os.getpid()), str(os.getppid())}
    pid = next((x for x in sh("pgrep -f 'python.*amr_detection_node'").split() if x not in me), '')
    if pid:
        print(f"amr_detection PID {pid} | " + sh(f"ps -o %cpu=,%mem=,nlwp= -p {pid}").strip() + '  (%CPU %MEM 스레드수)')
    else:
        print('amr_detection 프로세스가 이 PC 에 없다 — 노드가 도는 PC 에서 실행할 것')
    if shutil.which('nvidia-smi'):
        print('GPU: ' + sh('nvidia-smi --query-gpu=name,utilization.gpu,memory.used --format=csv,noheader'))
        apps = sh('nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv,noheader')
        print('GPU 프로세스: ' + (apps or '(없음 — YOLO 가 CPU 로 돌고 있을 가능성)'))
    else:
        print('nvidia-smi 없음 — GPU 미탑재 또는 드라이버 없음 → YOLO 가 CPU')

    rclpy.init()
    node = Diag(a.ns, a.seconds)
    time.sleep(0.5)                                   # 디스커버리가 노드 목록을 채울 시간
    names = sorted({f'{ns}/{n}'.replace('//', '/') for n, ns in node.get_node_names_and_namespaces()})
    print('보이는 노드: ' + ', '.join(x for x in names if 'diag_lag' not in x)[:400])
    print(f'\n=== {a.seconds:.0f} 초 측정 중 ... ===')
    end = time.monotonic() + a.seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        dt = a.seconds
        n, b, d = node.n, node.b, node.delay
        print('\n=== 결과 ===')
        print(f"{'토픽':<22}{'Hz':>7}{'MB/s':>8}   stamp→지금 지연")
        for k, label in (('rgb', '입력 rgb(jpeg)'), ('depth', '입력 depth(png)'), ('tf', '/tf'),
                         ('visible', '노드 처리(visible)'), ('out_image', '출력 target/image')):
            print(f'{label:<22}{n[k] / dt:>7.1f}{b[k] / dt / 1e6:>8.2f}   {stats(d[k]) if k in d else ""}')
        print('\n노드 [diag] 로그 (최근 3줄):')
        for line in node.diag_lines[-3:] or ['  (없음 — 노드가 이 빌드가 아니거나 diag_log_s=0)']:
            print('  ' + line)

        print('\n=== 판정 ===')
        rgb_hz, vis_hz, tf_hz = n['rgb'] / dt, n['visible'] / dt, n['tf'] / dt
        out_mb = b['out_image'] / dt / 1e6
        rgb_delay = sorted(d['rgb'])[len(d['rgb']) // 2] if d['rgb'] else None
        if rgb_hz == 0:
            print('입력이 0 Hz — 이 PC 에 카메라 토픽이 안 온다. 네임스페이스(--ns)·Discovery 확인')
        else:
            if rgb_delay is not None and (rgb_delay < -0.5 or rgb_delay > 5):
                print(f'입력 지연 {rgb_delay * 1000:.0f} ms 는 시계 어긋남(RPi vs PC) — 지연값은 무시하고 Hz 만 본다')
            elif rgb_delay is not None and rgb_delay > 0.3:
                print(f'③ 입력이 늦게 온다 — 카메라 stamp→도착 중앙값 {rgb_delay * 1000:.0f} ms. RPi·Wi-Fi 쪽')
            if vis_hz < rgb_hz * 0.7:
                print(f'노드가 입력을 못 따라간다 — 입력 {rgb_hz:.1f} Hz vs 처리 {vis_hz:.1f} Hz. '
                      f'[diag] 의 proc 가 크면 ① 계산(GPU 미사용/SSH 창), cb rgb 가 낮으면 ② 콜백 굶음(tf {tf_hz:.0f}/s)')
            else:
                print(f'노드는 입력을 따라간다 — 입력 {rgb_hz:.1f} Hz, 처리 {vis_hz:.1f} Hz. '
                      f'그래도 늦어 보이면 [diag] age 를 본다')
            if out_mb > 5:
                print(f'④ 출력 target/image {out_mb:.1f} MB/s ({out_mb * 8:.0f} Mbit/s) — rqt 를 다른 PC 에서 보면 Wi-Fi 포화. '
                      f'rqt 끄고 다시 재보면 갈린다')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
