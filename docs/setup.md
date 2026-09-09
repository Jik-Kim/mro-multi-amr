# 개발 환경과 실행 절차

Kn1_mini QUICKSTART 의 검증된 규칙을 그대로 가져왔다. 바뀐 것은 launch 로 묶은 것뿐이다.

## 준비 (PC 마다)

- Ubuntu 24.04, ROS 2 Jazzy, `~/turtlebot4_ws` 빌드 (turtlebot4, turtlebot4_desktop, turtlebot4_navigation)
- venv(`rokey_venv`): `ultralytics`, `opencv-python`, `numpy`, `flask`
- `ros-jazzy-tf2-geometry-msgs`, `ros-jazzy-message-filters`, `ros-jazzy-cv-bridge`
- 저장소: `git clone <repo> ~/mro-multi-amr && cd ~/mro-multi-amr/ros2_ws && colcon build --symlink-install && source install/setup.bash`

## 터미널마다 맨 처음

```bash
echo $ROS_DOMAIN_ID $ROS_DISCOVERY_SERVER $ROS_SUPER_CLIENT
```

기대값은 **PC 역할에 따라 다르다.** `ROS_DISCOVERY_SERVER` 는 세미콜론 위치가 서버 ID 다 (4번째 칸 = ID 3 = robot3, **9번째 칸 = ID 8 = robot8**).
> **9/8 robot4 → robot8 교체.** robot8 의 onboard 서버는 **ID `8` · IP `192.168.108.108`** (9/9 주행 확인). 옛 robot4 는 ID 4 로 적혀 있었으나 그것은 「로봇 번호 = ID」 규칙으로 미리 채운 값이었고 IP 도 끝내 비어 있었다 — robot8 값은 RPi 에서 직접 읽었다.
> 지금 쓰는 구성(PC-F 단일 서버, 9/5 확정)에서는 PC 가 로봇 onboard 서버를 보지 않으므로 당장 막히지는 않는다.

| PC | `ROS_DOMAIN_ID` | `ROS_DISCOVERY_SERVER` | `ROS_SUPER_CLIENT` |
|---|---|---|---|
| **offboard 단일 서버 채택 시 (1순위)** | `3` | **모든 PC 동일:** `;192.168.108.41:11811` | nav2 터미널만 `False`, 웹캠 PC 는 `False` |
| *(대안 ② 2서버 동시 접속 시)* robot3 제어 PC | `3` | `;;;192.168.108.103:11811;` | `True` (nav2 터미널만 `False`) |
| *(대안 ②)* robot8 제어 PC | `3` | `;;;;;;;;192.168.108.108:11811;` | `True` (nav2 터미널만 `False`) |
| *(대안 ②)* 웹캠 PC ×2 · 관제 PC | `3` | `;;;192.168.108.103:11811;;;;;192.168.108.108:11811;` | 웹캠 `False`, 관제 `True` |

값이 다르면 `source /etc/turtlebot4_discovery/setup.bash`. 이 파일은 PC 마다 한 번 위 표대로 맞춘다 (`turtlebot4_setup` 의 discovery 설정 스크립트에 서버 ID·IP 를 넣거나 파일을 직접 편집).
`ROS_SUPER_CLIENT` 는 setup.bash 가 대화형 터미널에서만 `True` 로 주므로, launch 를 스크립트로 띄우면 `False` 가 된다 — nav2 외 노드는 `True` 가 필요하니 주의.

**offboard 단일 서버 구성은 9/5 에 검증했다** (SOT Q-18 해소) — PC 5대·로봇 2대가 PC-F 서버 하나에 붙어 양쪽 로봇 토픽이 보이는 것을 확인. 위 대안 ② 행은 되돌릴 때를 위해 남긴다.

## Discovery Server 구성 — **offboard 단일 서버 (9/5 확정·검증)**

Kn1_mini 에서 Nav2 lifecycle 이 자꾸 멈춘 원인이 **RPi 의 디스커버리 부담**이었다. 로봇 2대·PC 5대가 되면 그 부하가 두 배다.
서버를 노트북 한 대로 옮기면 RPi 는 클라이언트로만 남고, **2서버 동시 접속 문제도 함께 사라진다.**

**PC-F = `192.168.108.41`** (Wi-Fi `turtle08`, 고정 설정 완료 9/5. 게이트웨이 `.1`, DNS `168.126.63.1/2`). **공유기 예약은 필요 없다 (9/7 확인)** — PC-F 는 학원 지원 노트북이라 재부팅해도 `.41` 이 유지된다고 학원이 확답. 시연 전 `ip a` 로 `.41` 인지만 확인한다.
**서버 ID 는 1 로 통일한다** — `turtlebot4-setup` 의 Offboard 기본값이 1 이라 도구와 싸우지 않는다.

**① PC-F 에서 서버 실행** (플래그는 `fastdds discovery --help` 로 버전 확인)
```bash
fastdds discovery -i 1 -l 0.0.0.0 -p 11811
```

**② 모든 PC 의 환경 변수** — `/etc/turtlebot4_discovery/setup.bash` 수정
```bash
export ROS_DISCOVERY_SERVER=";192.168.108.41:11811"
```
세미콜론 위치가 서버 ID 다. **ID 1 이면 앞에 세미콜론 하나** (기존 `;;;IP:11811;` 은 ID 3). **로봇 서버는 PC 목록에서 뺀다** — 넣으면 다시 RPi 에 부하가 간다.

**③ 로봇 2대** — RPi 에서 `turtlebot4-setup` → Discovery Server 메뉴. 이 메뉴는 "onboard **또는** offboard" 가 아니라 **"onboard + offboard 둘 다"** 구조라, 로봇 노드가 자기 서버와 PC-F 서버 양쪽에 등록된다.

| 항목 | 값 |
|---|---|
| Enabled | **True 그대로** (False 면 멀티캐스트로 바뀐다) |
| Onboard Server - Port / Server ID | 11811 / **3 그대로** (robot8 은 **8**) |
| **Offboard Server - IP** | **`192.168.108.41`** ← 이것만 채운다. 비어 있으면 offboard 미사용 |
| Offboard Server - Port / Server ID | 11811 / **1 그대로** |

Save → 재부팅. **되돌리기:** Offboard IP 를 비우고 Save 하면 원래대로.

부하가 줄어드는 이유: PC 들이 더 이상 RPi 서버를 보지 않는다. RPi 서버에는 로봇 자신의 노드만 남고, 로봇 노드와 PC 노드는 PC-F 를 통해 서로를 찾는다.

**지켜야 할 것 (NR-08)**
- **PC-F 의 IP 를 고정한다.** DHCP 로 바뀌면 전부 한 번에 끊긴다
- **PC-F 서버를 먼저 띄우고 로봇을 켠다.** 부팅 순서가 중요하다
- PC-F 가 단일 실패 지점이 된다. 시연 중 그 PC 에서 다른 작업을 하지 않는다
- **변경 전 원래 설정을 기록**한다. 로봇이 다른 조와 공용이면 되돌리는 절차까지 준비할 것

## 네트워크 고정값

| 항목 | 값 |
|---|---|
| `ROS_DOMAIN_ID` | **3** (모든 PC·로봇 공통) |
| 로봇 네임스페이스 | **`/robot3`**, **`/robot8`** (robot4 하드웨어 이슈로 9/8 교체) |
| Discovery Server | **PC-F `192.168.108.41:11811`, ID 1** (offboard 단일 서버, 9/5 확정). 로봇 onboard 서버(ID 3·8)는 남아 있으나 PC 는 보지 않는다 |
| nav2 launch 터미널 | `ROS_SUPER_CLIENT=False` |

두 로봇이 같은 도메인·같은 Discovery Server 를 쓰므로 네임스페이스로만 구분된다. 노드는 상대 토픽 이름을 쓰고 launch 가 `namespace:=` 를 붙인다.

## 로봇 제어 PC (로봇당 1대, 예: /robot3)

```bash
# T1 localization
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot3 map:=$(ros2 pkg prefix mro_bringup)/share/mro_bringup/maps/auto_map.yaml
# T2 RViz (2D Pose Estimate 클릭 금지 — 도크 기준 초기자세를 노드가 잡는다)
ros2 launch turtlebot4_viz view_navigation.launch.py namespace:=/robot3
# T3 nav2 — 이 터미널만
export ROS_SUPER_CLIENT=False
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot3
# T4 온보드 노드
ros2 launch mro_bringup robot.launch.py namespace:=/robot3
```

두 번째 로봇은 다른 PC 에서 `/robot8` 로 동일하게. **Nav2 launch 는 로봇당 한 PC 에서만** (`controller_server` 중복 = `/cmd_vel` 퍼블리셔 2개).

로봇을 추가·교체할 때 코드는 건드리지 않는다 — `namespace:=/robotN` 이 `mro_bringup/params/robotN.yaml` 을 이름으로 찾아 쓴다.
새 로봇이면 ① `params/robot3.yaml` 을 복사해 `dock_pose`·`depth_k` 실측을 채우고 ② `params/arena.yaml` 의
`waiting_zone`·`dock_zone.pose`·`dock_zone.approach` 에 항목을 넣고 ③ `params/fleet.yaml` 의 `robot_namespaces` 에 추가한다.
네임스페이스 없이 노드를 띄우면 예전처럼 조용히 robot3 인 척하지 않고 **바로 죽는다** (9/8).

## 웹캠 PC — **2대로 분리 (9/5 변경)**

```bash
# PC-W1 (관제센터 시야)
ros2 launch mro_bringup webcam1_pc.launch.py
```
```bash
# PC-W2 (작업공간·병목·더미 시야)
ros2 launch mro_bringup webcam2_pc.launch.py
```
호모그래피는 PC 별로 자기 json 을 쓴다 (`config/WebCam1/homography_cam1.json` /
`config/WebCam2/homography_cam2.json`). **카메라마다 폴더를 하나씩 둔다** — 행렬 외에 대응점·기준 이미지·재투영 오차가 같이 남아야 재캘리브레이션을 반복할 수 있기 때문이다. 새 아레나에서 반드시 재캘리브레이션
(`mro_webcam/config/README.md`).

웹캠 PC 는 주로 **발행만** 하므로 `ROS_SUPER_CLIENT=False` 로 띄워 디스커버리 부하를 줄인다. 띄운 뒤 `/webcam1/worker_descriptor`, `/webcam2/target_hint` 가 로봇·관제에서 보이는지 확인할 것.

웹캠2 압축 영상의 수신률·대역폭·발행→구독 지연을 확인할 때는 PC-W2와 같은 ROS 환경에서 아래 측정 노드를 실행한다. 출력에는 FPS, Mbps, 지연 평균·p95·최대와 2초 이상 수신 중단 경고가 포함된다. 동일한 `webcam_hint`를 둘 이상 띄우면 각 노드가 같은 토픽에 10 Hz씩 발행해 측정값이 합산되므로 먼저 중복 노드가 없는지 확인한다.

```bash
ros2 run mro_webcam test_subscription \
  --ros-args -p show_window:=false -p log_interval_s:=5.0
```

## 관제 PC

```bash
ros2 launch mro_bringup fleet_pc.launch.py     # http://localhost:5000, kpi_logger 포함
                                               #   로봇 명단은 params/fleet.yaml 의 robot_namespaces
# 한 번만 다른 명단으로 띄울 때 (파일은 그대로 두고)
ros2 launch mro_bringup fleet_pc.launch.py robots:="[/robot3,/robot8,/robot9]"
```

## 안 될 때

- Nav2 lifecycle 멈춤 (`Configuring X`, `change_state timeout`): `tools/nav_recovery_fetch.sh` (Kn1_mini 절차 동일)
- 노드가 토픽 0개: 그 터미널의 Discovery 환경 확인
- PC 를 바꿔 쓸 때 이전 PC 노드를 반드시 내린다 (`pkill -INT -f "ros2 launch|nav2_|rviz2|_node"`)
- 2대 운용 시 카메라 노드 소실·디스커버리 지연 → 카메라 구독자 수 확인 (로봇당 1개)
