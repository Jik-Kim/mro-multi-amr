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

기대값은 **PC 역할에 따라 다르다.** `ROS_DISCOVERY_SERVER` 는 세미콜론 위치가 서버 ID 다 (4번째 칸 = ID 3 = robot3, 5번째 칸 = ID 4 = robot4).

| PC | `ROS_DOMAIN_ID` | `ROS_DISCOVERY_SERVER` | `ROS_SUPER_CLIENT` |
|---|---|---|---|
| robot3 제어 PC | `3` | `;;;192.168.108.103:11811;` | `True` (nav2 터미널만 `False`) |
| robot4 제어 PC | `3` | `;;;;<robot4 IP>:11811;` `[IP 확인 필요]` | `True` (nav2 터미널만 `False`) |
| 웹캠 PC ×2 (PC-W1, PC-W2) | `3` | `;;;192.168.108.103:11811;<robot4 IP>:11811;` | `False` (주로 발행만) |
| 관제 PC (PC-F) | `3` | `;;;192.168.108.103:11811;<robot4 IP>:11811;` | `True` |

값이 다르면 `source /etc/turtlebot4_discovery/setup.bash`. 이 파일은 PC 마다 한 번 위 표대로 맞춘다 (`turtlebot4_setup` 의 discovery 설정 스크립트에 서버 ID·IP 를 넣거나 파일을 직접 편집).
`ROS_SUPER_CLIENT` 는 setup.bash 가 대화형 터미널에서만 `True` 로 주므로, launch 를 스크립트로 띄우면 `False` 가 된다 — nav2 외 노드는 `True` 가 필요하니 주의.

**2대 동시 접속은 미검증이다** (SOT Q-18, 9/6). 두 서버에 동시에 붙는 웹캠·관제 PC 에서 양쪽 로봇 토픽이 다 보이는지가 첫 확인 항목이다.

## Discovery Server 구성 — **offboard 단일 서버 (1순위, 9/6 결정 예정)**

Kn1_mini 에서 Nav2 lifecycle 이 자꾸 멈춘 원인이 **RPi 의 디스커버리 부담**이었다. 로봇 2대·PC 5대가 되면 그 부하가 두 배다.
서버를 노트북 한 대로 옮기면 RPi 는 클라이언트로만 남고, **2서버 동시 접속 문제도 함께 사라진다.**

**① PC-F 에서 서버 실행** (플래그는 `fastdds discovery --help` 로 버전 확인)
```bash
fastdds discovery -i 0 -l 0.0.0.0 -p 11811
```

**② 모든 PC 의 환경 변수** — `/etc/turtlebot4_discovery/setup.bash` 수정
```bash
export ROS_DISCOVERY_SERVER="<PC-F IP>:11811"
```
세미콜론 위치가 서버 ID 다. **ID 0 이면 앞에 세미콜론이 없다** (기존 `;;;IP:11811;` 은 ID 3).

**③ 로봇 2대** — RPi 에서 `turtlebot4-setup` → Discovery Server 항목 → **onboard 에서 offboard 로 변경**, 서버 IP 를 PC-F, 포트 11811, 서버 ID 0. 적용 후 재부팅. 파일을 직접 고치기보다 이 도구를 쓴다.

**지켜야 할 것 (NR-08)**
- **PC-F 의 IP 를 고정한다.** DHCP 로 바뀌면 전부 한 번에 끊긴다
- **PC-F 서버를 먼저 띄우고 로봇을 켠다.** 부팅 순서가 중요하다
- PC-F 가 단일 실패 지점이 된다. 시연 중 그 PC 에서 다른 작업을 하지 않는다
- **변경 전 원래 설정을 기록**한다. 로봇이 다른 조와 공용이면 되돌리는 절차까지 준비할 것

## 네트워크 고정값

| 항목 | 값 |
|---|---|
| `ROS_DOMAIN_ID` | **3** (모든 PC·로봇 공통) |
| 로봇 네임스페이스 | **`/robot3`**, **`/robot4`** |
| Discovery Server | 로봇별 RPi — robot3 = ID 3 (`192.168.108.103`), robot4 = ID 4 (`[IP 확인 필요]`). PC 역할별 값은 아래 '터미널마다 맨 처음' 표 |
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

두 번째 로봇은 다른 PC 에서 `/robot4` 로 동일하게. **Nav2 launch 는 로봇당 한 PC 에서만** (`controller_server` 중복 = `/cmd_vel` 퍼블리셔 2개).

## 웹캠 PC — **2대로 분리 (9/5 변경)**

```bash
# PC-W1 (관제센터 시야)
ros2 launch mro_bringup webcam1_pc.launch.py
```
```bash
# PC-W2 (작업공간·병목·더미 시야)
ros2 launch mro_bringup webcam2_pc.launch.py
```
호모그래피는 PC 별로 자기 json 을 쓴다 (`homography_cam1.json` / `homography_cam2.json`). 새 아레나에서 반드시 재캘리브레이션 (`mro_webcam/config/README.md`).

웹캠 PC 는 주로 **발행만** 하므로 `ROS_SUPER_CLIENT=False` 로 띄워 디스커버리 부하를 줄인다. 띄운 뒤 `/webcam1/worker_descriptor`, `/webcam2/target_hint` 가 로봇·관제에서 보이는지 확인할 것.

## 관제 PC

```bash
ros2 launch mro_bringup fleet_pc.launch.py     # http://localhost:5000, kpi_logger 포함
```

## 안 될 때

- Nav2 lifecycle 멈춤 (`Configuring X`, `change_state timeout`): `tools/nav_recovery_fetch.sh` (Kn1_mini 절차 동일)
- 노드가 토픽 0개: 그 터미널의 Discovery 환경 확인
- PC 를 바꿔 쓸 때 이전 PC 노드를 반드시 내린다 (`pkill -INT -f "ros2 launch|nav2_|rviz2|_node"`)
- 2대 운용 시 카메라 노드 소실·디스커버리 지연 → 카메라 구독자 수 확인 (로봇당 1개)
