# 시연 실행 절차 — PC 5대 · 로봇 2대 (9/10 실주행으로 검증)

9/10 로봇8 회차(`merge_r8_0910_1614`·`1715`)로 끝까지(관제 미션 → 언독 → 베이 → 탈출 회전 → 대기공간 → 추종 →
관제 RETURN → 도킹) 검증한 순서다. **명령어는 이 파일이 정본**이고, 값이 바뀌면 여기부터 고친다.

## 0. 역할과 순서

| 순서 | PC | 띄우는 것 | 기다릴 것 |
|---|---|---|---|
| 1 | PC-F (관제, `192.168.108.41`) | 디스커버리 서버 → `fleet_pc.launch.py` | UI http://localhost:5000 |
| 2 | PC-W1 (웹캠1) | `webcam1_pc.launch.py` | 작업자 입장 → 로그 `W1` |
| 3 | PC-W2 (웹캠2) | `webcam2_pc.launch.py` | 영상 창 |
| 4 | PC-R3 / PC-R8 (로봇 제어) | 아래 T0~T13 | T6 `[READY]` · T8 `Managed nodes are active` · 관제 UI 에 로봇 등장 |
| 5 | 사람 | 인형 차량을 웹캠1 앞에 **정지 2~3 s** → 관제 UI RequestMission(W1, 베이) | T13 에 `mission/assigned` |
| 6 | 관제 | 작업 종료 시 RETURN | T7 `[DOCKED]` |

종료는 역순: 로봇 PC 창 → 웹캠 → 관제 → 디스커버리 서버(마지막).

규칙
- 관제는 **전체에서 한 대만** (I-054). 두 대면 UI 에 빨간 배너.
- 안 쓰는 로봇의 스택은 띄우지 않는다 (디스커버리 부하 — 9/10 13:46 링크 끊김 회차에 robot3 풀스택이 같이 떠 있었다).
- **2대 운용은 노드 창을 `node` 모드로 띄운다 (9/11 기본).** T3·T5·T6·T7·T9·T10 — 아래 명령어에
  이미 들어 있다. 이유와 되돌리는 법은 6-1. 1대만 돌릴 때는 종전대로 `node` 없이 띄워도 된다.
- **2대는 겹쳐서 올리지 않는다 (9/10 추가).** 로봇 A 가 T8 `Managed nodes are active` 를 낼 때까지
  기다린 뒤 로봇 B 의 T3 를 시작한다. 그리고 **T2(bag) 과 T4(rviz) 는 양쪽 T8 이 다 뜬 뒤에 켠다** —
  둘 다 `scan`·`tf`·`odom` 을 로봇에서 **구독자마다 따로** 끌어오는데, 기동 구간엔 필요가 없다.
  5절 판정은 관제 미션부터 시작하므로 **bag 을 늦게 켜도 잃는 기록이 없다.**
- **T8 앞에 `map→odom` 을 확인한다 (하드 게이트 · 9/10 추가).** T6 의 `초기자세 … 전송` 로그만으로는
  부족하다 — 9/10 21:08 로봇3 은 셔틀 노드 로그가 **0건**이었고 AMCL 이 890 초 내내 초기자세를 못
  받았다. TF 가 실제로 생긴 것을 보고 T8 을 띄운다 (T6 절 참조). **확인 못 하면 T8 을 띄우지 않는다** —
  초기자세 없이 뜬 Nav2 는 ③에서 멈추고 **매니저가 재시도하지 않는다** (I-035).
- **두 대가 대기공간에 동시에 서면 안 된다.** `arena.yaml` 의 대기 좌표가 로봇3 `x=-0.998`, 로봇8
  `x=-1.593` 로 중심 간 0.595 m — 내접 반지름 0.186 씩 빼면 **옆 간격 0.22 m** 다. 워치독 `stop_dist`
  0.30 보다 좁고 **Nav2 inflation 0.25 보다도 좁아서** 두 번째는 목표점이 첫 번째의 팽창 영역 안이라
  도착 자체가 안 되고, 탐색 회전이 시작되면 서로 estop 이다. **두 번째 미션의 타이밍은 관제가
  조정한다** (첫 로봇이 대기공간을 떠난 뒤 — T12 `FOLLOWING`).
- `teleop_twist_keyboard` 창은 주행 중 절대 열어 두지 않는다 (키 하나가 로봇을 움직인다).
- 미션 시작·종료 토픽은 **관제만** 낸다. 로봇 PC 에서 `mission/assigned` 를 수동 발행하지 않는다.

## 0-0. 9/11 아침 — 목표와 순서

**목표는 2대 동시 운용을 녹화하는 것이다.** 단독 완주 영상은 이미 있다. 그래서 **부하를 줄여
Nav2 가 2벌 `active` 가 되는 것이 첫 번째**이고, 그다음이 bag + 영상으로 2대 회차를 남기는 것이다.
11:30 제출·발표라 **로봇 작업은 10:00 에 끊는다.** 그때까지 2대 완주가 없으면 단독 영상 + 양쪽
`active` 화면 + 관제 UI 에 2대 뜬 화면으로 간다.

**깨끗하게 시작한다.** 9/10 밤 회차엔 유령 등록 · `talker` · 관제 두 벌 · rviz 로봇당 3개가 함께
떠 있었다. 순서 규칙만으론 유령이 남아 있으면 첫 번째부터 느리다.

| 순서 | 무엇 | 왜 |
|---|---|---|
| ① | **PC-F 디스커버리 서버 재시작** — 서버 창 Ctrl-C 후 `fastdds discovery -i 1 -l 0.0.0.0 -p 11811` 다시. **아무것도 띄우기 전에** | 어제 밤의 유령 등록을 지운다. 노드가 떠 있는 채로 재시작하면 전부 다시 등록해야 하니 맨 처음에 한다 |
| ② | RPi 2대 `sudo systemctl restart turtlebot4.service` (안 되면 재부팅) | 로봇 쪽 참가자도 새로 등록시킨다 |
| ③ | PC 5대 `git pull --ff-only` → 해시 대조 (0-1절) | 9/10 밤 「추종 안 됨」이 안 받은 PC 였다 |
| ④ | 로봇 PC 에서 `tools/shuttle_preflight.sh pre /ROBOT` | `talker`·rviz·이전 세션 잔재가 없는지 |
| ⑤ | **로봇8 단독** — 노드 창 `node` 모드(6-1) · T6 → `preflight pose` → T8 `active` → 미션 1회 완주 | 2대 가기 전에 1대가 새 구성으로 도는지 |
| ⑥ | **로봇3 추가** — 같은 순서. 양쪽 `active` · 관제 UI 에 2대 | 여기까지가 첫 번째 목표 |
| ⑦ | T2 bag 켜고 **2대 회차** — 폰 영상 동시 촬영 | 두 번째 미션 타이밍은 관제가 조정한다 (아래 규칙) |
| ⑧ | 성공이면 한 번 더. 실패면 **원인 하나만** 고쳐 재시도. **10:00 중단** | |

## 0-1. 모든 PC 공통 — **먼저 같은 코드인지 맞춘다** (9/10 밤 추가)

**9/10 에 PR 이 19 건 들어갔다. 그리고 그날 밤 「추종이 안 된다」의 원인이 정확히 이것이었다** —
코드는 고쳐져 있는데 로봇 PC 가 안 받은 상태였다 (`reid_threshold` 0.6 → 0.34).

**PC 5대 전부에서** 돌리고 **해시가 같은지 눈으로 대조한다.**

```bash
cd ~/mro-multi-amr && git pull --ff-only && git log --oneline -1
```

다르면 그 PC 만 뒤처진 것이다. `--ff-only` 가 거부하면 그 PC 에 커밋되지 않은 변경이 있다는 뜻이니
`git status` 로 확인하고 정리한 뒤 다시 받는다 — **시연 아침에 그 자리에서 고치지 않는다.**

## 1. 모든 PC 공통 — 터미널 첫 줄

로봇 제어 PC 는 `source tools/shuttle_env.sh` 가 아래를 대신한다.

```bash
source /etc/turtlebot4_discovery/setup.bash && source ~/turtlebot4_ws/install/setup.bash && source ~/mro-multi-amr/ros2_ws/install/setup.bash && echo "$ROS_DOMAIN_ID $ROS_DISCOVERY_SERVER $ROS_SUPER_CLIENT"
```

기대값 `3 ;192.168.108.41:11811 True`. 웹캠 PC · Nav2 창 · **2대 운용 시 노드 창(T3·T5·T6·T7·T9·T10, `node` 모드)** 은 `False`. 다르면 `/etc/turtlebot4_discovery/setup.bash` 의
`ROS_DISCOVERY_SERVER=";192.168.108.41:11811"` 을 맞춘다. **스크립트·SSH 로 띄우면 `ROS_SUPER_CLIENT` 가 조용히 False 가 되어
토픽이 거의 안 보인다** — 관제 창은 `export ROS_SUPER_CLIENT=True` 를 명시한다.

## 2. PC-F · 관제

```bash
ip a | grep 192.168.108.41 || echo "IP 가 .41 이 아님"
```
```bash
fastdds discovery -i 1 -l 0.0.0.0 -p 11811
```
```bash
export ROS_SUPER_CLIENT=True && ros2 launch mro_bringup fleet_pc.launch.py
```
로봇이 보이는지 (로봇 PC 의 T3·T6·T9 가 떠야 1 Hz 로 찍힌다):
```bash
export ROS_SUPER_CLIENT=True && ros2 topic echo /fleet/status --filter "m.robot_ns == '/robot8'"
```

`fleet_pc.launch.py` 하나가 zone_semaphore · fleet_manager(웹 UI :5000 · SQLite `~/mro_fleet.db` 자동 생성 · 로봇별 상태 수집) ·
kpi_logger 를 띄운다. 따로 켤 것은 없다. 로봇 명단은 `params/fleet.yaml`.

## 3. PC-W1 · 웹캠1 / PC-W2 · 웹캠2

```bash
export ROS_SUPER_CLIENT=False && ros2 launch mro_bringup webcam1_pc.launch.py
```
```bash
export ROS_SUPER_CLIENT=False && ros2 launch mro_bringup webcam2_pc.launch.py
```
로봇 PC 에서 보이는지 (둘 다 떠야 함):
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 node list | grep -E "webcam_register|webcam_hint"; ros2 topic hz /webcam2/target_hint
```
> 9/10 17:15 회차는 `webcam_hint` 가 이 서버에 없어 힌트가 한 건도 안 왔다 — 추종은 규약대로 20° 스윕을 했다.

## 4. 로봇 제어 PC — 터미널 14개

로봇별 값. 아래 명령어의 `ROBOT`·`BAY` 를 바꾼다.

| | robot3 | robot8 |
|---|---|---|
| `ROBOT` | `robot3` | `robot8` |
| `BAY` (마커) | `B` (2) | `A` (1) |
| 초기자세 로그 | `(0.02, -0.80, 8°)` | `(-0.02, 0.03, -3°)` |
| 대기공간 | `(-1.00, 0.94)` | `(-1.59, 0.94)` |

### T0 · 빌드 + 청소 — 임시 (모든 ROS 창 내린 뒤)
> **`mro_navigation` 을 반드시 넣는다 (9/10 밤 추가).** `setup.py:15` 가 `launch/*.launch.py` 를
> **빌드 시점 glob** 으로 설치한다. 재빌드하지 않으면 PR #88 이 새로 만든 **`safety_nav2.launch.py` 가
> 로봇 PC 에 아예 없다** — T8 에서 「그런 런치 파일 없음」으로 막힌다. `core/safety.py`·`motion_pause.py` 도
> 같은 날 새로 생긴 모듈이다. **관제 PC(PC-F)는 `mro_fleet` 도 함께 넣는다** — 9/10 에 노드 구성이 바뀌었다.
```bash
cd ~/mro-multi-amr/ros2_ws && colcon build --packages-select mro_bringup mro_perception mro_navigation --symlink-install
```
```bash
rm -f /dev/shm/sem.fastrtps_* /dev/shm/fastrtps_* /dev/shm/fast_datasharing_* /tmp/launch_params_* && rm -rf ~/.ros/log/* && echo "shm $(ls /dev/shm | grep -ci fastrtps)"
```
```bash
pkill -f teleop_twist_keyboard; pgrep -af teleop_twist_keyboard || echo "텔레옵 없음"
```

### T1 · 확인 — 임시
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 topic echo --once /ROBOT/dock_status --field is_docked
```
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 topic hz /ROBOT/scan
```
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 node list | grep -E "fleet_manager|zone_semaphore|webcam_register|webcam_hint" | sort | uniq -c
```
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 node list | sort | uniq -d | grep -E "bt_navigator|controller_server|amcl|shuttle_mission|follow_target|safety_watchdog|return_dock|amr_detection"
```
도크 `True` · scan 7.5 Hz · 관제/웹캠 각 1개 · 중복 없음.

### T2 · bag — 유지 · **양쪽 T8 이 뜬 뒤에 켠다** (0절 규칙)
```bash
mkdir -p ~/bags && cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 bag record -o ~/bags/merge_ROBOT_$(date +%m%d_%H%M) /rosout /webcam2/target_hint /webcam1/worker_descriptor /fleet/zone_event /fleet/status /fleet/mission /ROBOT/mission/assigned /ROBOT/mission/finish /ROBOT/shuttle/status /ROBOT/approach/status /ROBOT/shuttle/arrived /ROBOT/safety/estop /ROBOT/safety/state /ROBOT/cmd_vel /ROBOT/cmd_vel_navigation /ROBOT/cmd_vel_search /ROBOT/scan /ROBOT/amcl_pose /ROBOT/odom /ROBOT/dock_status /ROBOT/battery_state /ROBOT/hazard_detection /ROBOT/tf /ROBOT/tf_static /ROBOT/target/map_point /ROBOT/target/visible /ROBOT/target/paired /ROBOT/target/pair_state /ROBOT/target/distance /ROBOT/target/worker_id /ROBOT/aruco/id /ROBOT/aruco/distance /ROBOT/aruco/pose /ROBOT/goal_pose /ROBOT/local_costmap/costmap
```

### T3 · localization — 유지
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh node && ros2 launch turtlebot4_navigation localization.launch.py namespace:=/ROBOT map:=$(ros2 pkg prefix mro_bringup)/share/mro_bringup/maps/auto_map.yaml
```

### T4 · rviz — 유지 · **양쪽 T8 이 뜬 뒤에 켠다** (2D Pose Estimate 누르지 않음)
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 launch turtlebot4_viz view_navigation.launch.py namespace:=/ROBOT
```

### T5 · 셔틀 리그 — 유지 (워치독 · ArUco 검출기 · 세마포어는 관제 것)
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh node && ros2 launch mro_navigation shuttle_step1_rig.launch.py namespace:=/ROBOT bay:=BAY real_aruco:=true semaphore:=false safety_participants:="shuttle_mission,follow_target,return_dock"
```
띄운 뒤 로봇이 도크에 그대로 있어야 한다 (워치독은 dock_status 를 받기 전·도킹 중엔 cmd_vel 을 내지 않는다).

### T6 · 셔틀 노드 — 유지 · Nav2 보다 먼저
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh node && ros2 run mro_navigation shuttle_mission_node --ros-args -r __ns:=/ROBOT -r /tf:=tf -r /tf_static:=tf_static --params-file $(ros2 pkg prefix mro_bringup)/share/mro_bringup/params/common.yaml --params-file $(ros2 pkg prefix mro_bringup)/share/mro_bringup/params/ROBOT.yaml
```
`초기자세 … 전송` 확인. T3 에 `AMCL cannot publish` 가 계속되거나 `dock_status 15 s` 경고면 이 창만 재시작 (첫 발행이 디스커버리 매칭 전에 나가 유실된 것).

**그리고 T8 로 넘어가기 전에 `map→odom` 이 실제로 생겼는지 본다 (하드 게이트).** 로그는 '보냈다' 만
말하지 '도착했다' 를 말하지 않는다.

```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 run tf2_ros tf2_echo map odom --ros-args -r __ns:=/ROBOT -r /tf:=tf -r /tf_static:=tf_static
```

또는 점검 스크립트가 이 셋(셔틀 노드 존재 · `map→odom` · `amcl_pose`)을 한 번에 본다:

```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && tools/shuttle_preflight.sh pose /ROBOT
```

변환이 찍히면 Ctrl-C 로 닫고 T7·T8 로 간다. 안 찍히면 **T8 을 띄우지 않는다** — 이 창을 다시 띄우고
`ros2 topic echo /ROBOT/amcl_pose --once` 로 AMCL 이 받았는지 확인한다. 초기자세 없이 뜬 Nav2 는
`global_costmap` 활성화에서 멈추고 **매니저가 재시도하지 않는다** (I-035 · 6-2절).

### T7 · 복귀 노드 — 유지 (**finish 전에 떠 있는지 확인** — 9/10 16:14 에 6번 놓쳤다)
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh node && ros2 run mro_navigation return_dock_node --ros-args -r __ns:=/ROBOT -r /tf:=tf -r /tf_static:=tf_static --params-file $(ros2 pkg prefix mro_bringup)/share/mro_bringup/params/common.yaml --params-file $(ros2 pkg prefix mro_bringup)/share/mro_bringup/params/ROBOT.yaml
```

### T8 · Nav2 — 유지 · **T6 의 `map→odom` 확인 뒤에** (`Managed nodes are active` 까지 기다림)
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh nav2 && ros2 launch mro_navigation safety_nav2.launch.py namespace:=/ROBOT dwb_sim_time:=1.0
```
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 param get /ROBOT/controller_server FollowPath.sim_time
```
`dwb_sim_time:=1.0` — 9/10 17:15 회차에서 출입구 모서리 `Failed to make progress` 9회 → 0회. 글로벌·로컬 inflation 0.25.

### T9 · 추종 리그 — 유지 (재식별 = 인지 실물)
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh node && ros2 launch mro_navigation follow_rig.launch.py namespace:=/ROBOT show_window:=false paired:=real
```

### T10 · 로봇 시점 창 — 유지
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh node && python3 tools/detection_view.py --ns /ROBOT
```

### T11 · 안전 상태 · T12 · 추종 상태 · T13 · 관제 미션 수신 — 유지
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 topic echo /ROBOT/safety/state
```
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 topic echo /ROBOT/approach/status
```
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 topic echo /ROBOT/mission/assigned
```
준비 완료: T11 `CLEAR` · T12 `IDLE` · T10 마커 사각형 · T7 `[대기]` · 관제 UI 에 로봇 등장.

### T14 · 예비 — 임시 (관제 UI 가 못 누를 때만 · 관제 서비스 경유)
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 service call /fleet/request_mission mro_interfaces/srv/RequestMission "{worker_id: 'W1', bay: 'BAY'}"
```
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 service call /ROBOT/manual_pair mro_interfaces/srv/ManualPair "{robot_ns: '/ROBOT', worker_id: 'W1'}"
```
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 service call /fleet/operator_command mro_interfaces/srv/OperatorCommand "{robot_ns: '/ROBOT', command: 'RETURN'}"
```
manual_pair 는 SEARCHING 중 차량이 **1 m 안**에 보일 때만 받는다 (30 s 게이트 미달 뒤 관제 UI 에도 버튼이 뜬다).

## 5. 판정

```
T5   도크 유지 → 관제 미션 → T6 언독 → [SHUTTLE] → [AT_BAY] → 정렬 → "탈출 회전 완료" → P1 → [WAITING]
T9   "worker_b conf=0.9x sim=0.4x paired"  (sim < 0.34 면 30 s 뒤 GATE_FAIL → 수동 페어링)
T12  IDLE → SEARCHING → FOLLOWING
관제 RETURN → T7 [RETURNING] → [DOCKING] → [DOCKED]
T11  CLEAR 유지. 막히면 STOPPING → SAFE_ROTATE → CLEAR
```
끝나면 T2 를 Ctrl-C 로 닫는다. bag 폴더 이름이 분석 단위다.

## 6. 안 될 때 (9/10 에 실제로 겪은 것)

| 증상 | 원인 | 조치 |
|---|---|---|
| 관제 UI 에 로봇이 없음 | 로봇 토픽이 관제에 안 닿음 | 로봇 PC `ros2 topic hz /ROBOT/approach/status` (T9) → 관제 창 `ROS_SUPER_CLIENT` → 관제 2대 여부 |
| T6 뒤에도 `AMCL cannot publish` | 초기자세 첫 발행 유실 (디스커버리 느림) | T6 재시작 |
| T8 이 `bond` 타임아웃으로 안 뜸 | 디스커버리 과부하 (talker·bag 재생·다른 로봇 스택) | 불필요한 노드 내리고 T8 재시작 |
| 2대 운용 중 T8 이 안 뜨고 핑이 튐 | 디스커버리 명단 배포 폭주 — 아래 6-1 | 0절의 2대 기동 순서 → 그래도 안 되면 6-1 |
| `Timed out waiting for transform from base_link to map` 으로 활성화가 3번(`planner_server`)에서 멈춤 | 둘 중 하나다 — 초기자세가 **애초에 안 갔거나**, 갔는데 Nav2 가 TF 보다 먼저 활성화됐거나 | **먼저 `amcl_pose` 로 어느 쪽인지 가른다** — 아래 6-2 |
| 로봇이 스캔 없이 멈춤 (`STUCK: 유효한 LiDAR 스캔 없음`) | 로봇↔PC 링크 끊김 (13:46) | `ros2 topic hz /ROBOT/scan` → RPi `sudo systemctl restart turtlebot4.service` |
| 리그를 띄우자 로봇이 도크에서 빠짐 | Create3 가 cmd_vel 수신에 반응 (14:34·14:55) | 워치독이 도킹 중 발행 금지 — 재발 시 `ros2 topic info -v /ROBOT/cmd_vel` 로 다른 발행자 확인 |
| 작업자를 봐도 SEARCHING 만 반복 | 재식별 sim < 0.34 (`unpaired`) | 인형을 웹캠1 앞에 정지 2~3 s 로 재등록, 또는 manual_pair |
| RETURN 눌러도 즉시 FAILED | 추종 취소와 복귀 첫 목표 경합 (17:15) | PR #88 에서 수정 (IDLE 대기·1회 재전송). 그 전 판이면 RETURN 한 번 더 |
| 힌트 방향으로 안 돌고 20° 씩 | `webcam_hint` 가 서버에 없음 | PC-W2 `echo $ROS_DISCOVERY_SERVER`, 로봇 PC `ros2 node list | grep webcam_hint` |

## 6-1. 2대 운용의 `node` 모드 — 왜 기본인가

**9/11 부터 2대 운용의 기본이다.** T3·T5·T6·T7·T9·T10 명령어에 이미 들어 있다.

실주행 검증은 아직 전이다 (9/10 밤엔 시간이 없었다). 그런데도 기본으로 두는 이유 —
**검증된 2대 경로 자체가 없다.** 9/10 밤 두 번(`20:53`·`21:08`) 다 실패했다. 그렇다면 부하가 낮은
쪽으로 첫 시도를 하는 게 맞고, 중간에 창 여섯 개를 내리고 모드를 바꿔 다시 띄우는 것은 **그 자체가
디스커버리 폭풍**이다. Nav2 창(T8)은 하루 종일 `False` 로 돌았고, 노드 창을 `False` 로 두는 것은
같은 성질이라 위험이 낮다. **7시 로봇8 단독 회차(0-0 ⑤)가 곧 검증이다.**

1대만 돌릴 때는 종전대로 `node` 없이 띄워도 된다 — 그 경로는 9/10 17:15 에 완주로 검증됐다.

### 왜 이걸 의심하는가

`ROS_SUPER_CLIENT=True` 인 참가자는 자기에게 필요한 것이 아니라 디스커버리 서버가 가진
**전체 참가자 명단**을 받는다. 절차서대로 띄우면 로봇 PC 한 대에 **15벌**이다 —
T2 bag 1 · T3 localization 3 · T4 rviz 1 · T5 리그 2 · T6·T7 2 · T9 리그 2 · T10 1 ·
T11~T13 echo 3. **2대면 30벌**이고, 시스템 전체 참가자는 약 75벌이다.

Nav2 는 `turtlebot4_navigation/nav2.launch.py` 가 `use_composition` 을 `'False'` 로
하드코딩해서 **프로세스가 11개**다. 두 번째 Nav2 를 켜는 순간이 *명단이 제일 클 때* 와
*참가자 11개가 한꺼번에 들어올 때* 가 겹치는 지점이다.

**다만 이것이 T8 실패의 유일한 원인은 아니다.** 9/10 밤 회차(`20:53`·`21:08`) 를 보면
20:53 은 **먼저 띄운** 로봇8 이 bond 실패했고(그때 로봇3 Nav2 는 아직 안 떠 있었다),
21:08 로봇3 은 **초기자세가 아예 안 가서** 실패했다 (셔틀 노드 로그 0건 · AMCL 890 초 내내
미수신). '항상 두 번째가 실패한다' 는 성립하지 않는다 — **부하는 실패를 쉽게 만들 뿐이고,
그날 실패의 직접 원인은 따로 있을 수 있다. 6-2 로 먼저 가른다.**

그리고 그 회차는 **절차대로 돈 것이 아니었다** — 관제가 두 벌(I-054 위반) · rviz 가 로봇당
3개(T4 는 1개) · `talker` · 유령 등록이 함께 떠 있었다. 부하가 병목인지 판단하려면 **먼저
절차대로 한 번 돌려야 한다.**

### 적용 대상 — 노드만 띄우는 터미널

**T3 · T5 · T6 · T7 · T9 · T10** 이 `source tools/shuttle_env.sh node` 다. 나머지는 손대지 않는다.

```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh node && ros2 launch turtlebot4_navigation localization.launch.py namespace:=/ROBOT map:=$(ros2 pkg prefix mro_bringup)/share/mro_bringup/maps/auto_map.yaml
```

PC 당 9벌, 2대면 **18벌**이 명단 배포 대상에서 빠진다.

**바꾸지 않는 것** — 전체 그래프를 조회하는 터미널은 `True` 라야 한다:
T1(점검) · **T2(bag — 타입 확인에 `get_topic_names_and_types` 를 쓴다)** ·
T11·T12·T13(`topic echo`) · T14(`service call`) · 관제 PC · T8 은 원래 `nav2`.

### 되돌리기와 판정

틀렸다면 **기동 2분 안에 드러난다** — T6 이 `초기자세 … 전송` 을 못 내거나, T11·T12 에
상태가 안 찍힌다. 그때는 `node` 를 빼고 그 창만 다시 띄우면 원래대로다. 안 당긴 PC 에서 `node`
를 치면 스크립트가 경고를 낸다 (모르는 모드는 조용히 넘어가지 않는다, 9/10). 바꾼 창 말고는
아무것도 달라지지 않으므로 **원인 변수가 하나로 유지된다.**

> `use_composition` 을 `True` 로 돌리면 Nav2 프로세스가 11 → 2 벌이 된다. 걸림돌로 본
> `SetRemap`(`global_costmap/scan`→`scan`)·`PushRosNamespace` 는 둘 다 composable 노드에
> 전달되는 것을 확인했다 (`launch_ros/actions/load_composable_nodes.py:297,304`).
> 다만 컨테이너(`component_container_isolated`)를 직접 띄워야 하고 **실주행 재검증이
> 필요하다** — Phase 1 시연 뒤에 볼 것.

## 6-2. 활성화가 `planner_server` 에서 멈췄을 때 — RESUME

`Managed nodes are active` 가 안 나오고 아래가 뜬 채로 멈춘 경우다. **T8 을 껐다 켜지 않는다.**

```
[planner_server-N] [global_costmap.global_costmap]: Timed out waiting for transform
    from base_link to map to become available, tf error: ...
```

### 이게 3번이다

T8 의 활성화 순서는 `navigation_launch.py:43` 에 있다 — ① `controller_server` ② `smoother_server`
③ **`planner_server`** ④ `route_server` ⑤ `behavior_server` ⑥ `velocity_smoother`
⑦ `collision_monitor` ⑧ `bt_navigator` ⑨ `waypoint_follower` ⑩ `docking_server` (+ 매니저 = 11 프로세스).

- **`local_costmap` 은 `controller_server`(①) 것이고 `odom` 프레임이라 map 없이도 뜬다.** 그래서 ①은 통과한다.
- **map 이 필요한 것은 `planner_server`(③) 의 `global_costmap` 이다.** 위 메시지는 여기서만 나온다 —
  `route_server`(④) 에는 map 관련 메시지가 없고, TB4 `nav2.yaml` 에 설정 자체가 없다.
  **줄 수로 세지 말고 메시지 본문으로 판정한다.**

### 먼저 가른다 — 원인이 둘이고 조치가 다르다

**RESUME 을 먼저 치지 않는다.** 초기자세가 애초에 안 갔으면 RESUME 은 아무것도 못 고친다.

```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 topic echo /ROBOT/amcl_pose --once
```

| `amcl_pose` | 무슨 일이 있었나 | 조치 |
|---|---|---|
| **안 나옴** (멈춰 있음) | **초기자세가 아예 안 갔다** — T6 이 안 떴거나 즉사했다 | **A. T6 부터** |
| 나옴 | 갔고 AMCL 도 받았다. Nav2 가 TF 보다 **먼저** 활성화된 것뿐이다 | **B. RESUME** |

### A. 초기자세가 안 간 경우

9/10 21:08 로봇3 이 이 경우였다 — **셔틀 노드 로그 0건 · AMCL 890 초 내내 미수신.** 늦게 온 것이
아니라 **한 번도 안 왔다.** 이건 디스커버리 부하와 무관하며, 부하를 아무리 줄여도 안 고쳐진다.

T6 을 띄우고 `초기자세 … 전송` 을 눈으로 확인한다. 그다음 **`map→odom` 이 실제로 생겼는지 본다:**

```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 run tf2_ros tf2_echo map odom --ros-args -r __ns:=/ROBOT -r /tf:=tf -r /tf_static:=tf_static
```

변환이 찍히면 그때 T8 을 띄운다. **이 확인이 0절의 하드 게이트다** — I-035 의 PM 결정 ⓒ 가
요구한 것이다. 확인 없이 T8 을 띄우면 ③에서 멈추고, **매니저는 나중에 TF 가 생겨도 재시도하지
않으므로** 잠깐의 어긋남이 영구 실패가 된다.

### B. Nav2 가 TF 보다 먼저 활성화된 경우

이쪽은 타이밍이다. 디스커버리 서버를 쓰면 **같은 PC 안의 두 프로세스도 서로를 찾으려면 PC-F 를
거친다** (멀티캐스트 탐색이 꺼진다). 데이터는 그다음에 로컬로 흐르지만 **짝을 맺는 것 자체가 무선
왕복**이라, 서버가 막히면 AMCL→`planner_server` 매칭이 늦어진다. 6-1 이 겨냥하는 것이 이쪽이다.

활성화를 다시 건다 (`command: 2` = RESUME). **I-035 에 9/7 실측 확인 기록이 있는 명령이다** —
`global_costmap`·`planner_server`·`behavior_server`·`bt_navigator`·`velocity_smoother`·
`waypoint_follower` 가 모두 `active [3]` 이 되는 것을 확인했다.

```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 service call /ROBOT/lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 2}"
```

확인:

```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 service call /ROBOT/bt_navigator/get_state lifecycle_msgs/srv/GetState
```

**T8 을 껐다 켜지 않는 이유** — 재시작은 참가자 11개를 내렸다가 다시 올리는 것이라 **디스커버리
폭풍을 한 번 더 만든다.** RESUME 은 떠 있는 노드의 상태만 전이시키므로 디스커버리가 다시 일어나지
않는다. 다만 **A 인 경우엔 RESUME 이 소용없다** — 위 표로 먼저 가르는 이유다.
