"""camera_info 초점거리 보정 — OAK-D 가 잘라낸 영상에 자르기 전 초점거리를 실어 보낸다.

발행 영상은 704×704 정사각인데 `camera_info` 의 fx 는 자르기 전 16:9 전체 폭(약 1252) 기준이다.
자르기는 화각만 좁힐 뿐 초점거리를 바꾸지 않는데 폭 비율로 줄인 값이 들어가, fx 가 실제보다
16/9 = 1.778 배 작다. 주점은 정상이다 — cx 353.4 · cy 354.1 ≈ 704/2 이므로 "가운데를 잘랐다"는
것이 오히려 확인된다.

**fx 와 fy 를 함께 곱한다.** 9/7 실측에서 마커 네 변의 *평균* 픽셀로 역산한 비가 1.728·1.744 였다.
fy 는 정상이고 fx 만 틀렸다면(= 가로로 찌그러진 영상이라면) 같은 계산이 (1013+570)/2/570 = 1.389
로 나왔어야 한다. 등방이 맞고, camera_info 도 fx = fy = 569.95 로 같다.

영향 (9/7 robot4 실측):
  aruco/distance     1.74 배 작게       주행 FR-08 도착 확정 · 관제 FR-35 측위 검증 (계약 9.2 ①)
  target/map_point   좌우 1.76 배 크게  1.13 m 지점에서 11.7 cm. 주행 추종 목표가 옆으로 밀린다
  target/distance    영향 없음          depth(스테레오 시차)는 미터를 직접 내므로 초점거리를 안 쓴다

**보정은 두 노드가 이 함수 하나로만 한다.** 각자 곱하면 나중에 한쪽만 고쳐진다 — 계약 2절이
`hsv_hist` 를 공용 함수로 못박은 것과 같은 이유다.

근본 해결은 로봇이 영상과 맞는 `camera_info` 를 내보내는 것이다(depthai/turtlebot4 설정이라 저장소
밖. 각 제어 PC 에서 nav2.yaml 을 고치기로 한 I-009 와 같은 부류). 그때 계수를 1.0 으로 되돌린다.
"""
import numpy as np

# 정사각 crop 이 16:9 원본의 전체 높이를 쓴다는 기하에서 나오는 값.
# 실측은 1.728(62점) · 1.744(144점) · 1.761(좌우 위치 대조) 로 2 % 낮은데, depth 를 기준자로 썼기에
# depth 자체 캘리브 오차(Kn1_mini 검증 1.06 %)가 섞인 것으로 본다. TS-U05 줄자 대조로 확정한다.
FX_SCALE_CROP_16_9 = 16.0 / 9.0

PARAM = 'camera_fx_scale'


def read_fx_scale(node):
    """`camera_fx_scale` 을 선언·검증해 돌려준다. 두 노드가 같은 이름·같은 기본값을 쓰도록.

    기본값이 곧 정답이어야 한다 — robot.launch.py 는 `mro_bringup/params/robotN.yaml` 만 넘기고
    `perception.yaml` 은 아직 넘기지 않는다 (todo.md, 마감 9/8). 연결되기 전까지 yaml 값은 안 먹는다.
    """
    node.declare_parameter(PARAM, FX_SCALE_CROP_16_9)
    s = float(node.get_parameter(PARAM).value)
    if not 0.1 <= s <= 10.0:
        # 0 이나 음수가 조용히 먹으면 거리가 통째로 0 이 되고 원인을 찾는 데 하루가 든다
        node.get_logger().error(f'{PARAM}={s} 는 범위 밖 — 보정 없이(1.0) 진행한다')
        return 1.0
    return s


def corrected_K(msg, fx_scale):
    """CameraInfo → 보정한 3×3 카메라 행렬. fx_scale=1.0 이면 원본 그대로."""
    K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
    K[0, 0] *= fx_scale
    K[1, 1] *= fx_scale
    return K
