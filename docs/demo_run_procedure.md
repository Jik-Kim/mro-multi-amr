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
- `teleop_twist_keyboard` 창은 주행 중 절대 열어 두지 않는다 (키 하나가 로봇을 움직인다).
- 미션 시작·종료 토픽은 **관제만** 낸다. 로봇 PC 에서 `mission/assigned` 를 수동 발행하지 않는다.

## 1. 모든 PC 공통 — 터미널 첫 줄

로봇 제어 PC 는 `source tools/shuttle_env.sh` 가 아래를 대신한다.

```bash
source /etc/turtlebot4_discovery/setup.bash && source ~/turtlebot4_ws/install/setup.bash && source ~/mro-multi-amr/ros2_ws/install/setup.bash && echo "$ROS_DOMAIN_ID $ROS_DISCOVERY_SERVER $ROS_SUPER_CLIENT"
```

기대값 `3 ;192.168.108.41:11811 True`. 웹캠 PC 와 Nav2 창만 `False`. 다르면 `/etc/turtlebot4_discovery/setup.bash` 의
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
```bash
cd ~/mro-multi-amr/ros2_ws && colcon build --packages-select mro_bringup mro_perception --symlink-install
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

### T2 · bag — 유지
```bash
mkdir -p ~/bags && cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 bag record -o ~/bags/merge_ROBOT_$(date +%m%d_%H%M) /rosout /webcam2/target_hint /webcam1/worker_descriptor /fleet/zone_event /fleet/status /fleet/mission /ROBOT/mission/assigned /ROBOT/mission/finish /ROBOT/shuttle/status /ROBOT/approach/status /ROBOT/shuttle/arrived /ROBOT/safety/estop /ROBOT/safety/state /ROBOT/cmd_vel /ROBOT/cmd_vel_navigation /ROBOT/cmd_vel_search /ROBOT/scan /ROBOT/amcl_pose /ROBOT/odom /ROBOT/dock_status /ROBOT/battery_state /ROBOT/hazard_detection /ROBOT/tf /ROBOT/tf_static /ROBOT/target/map_point /ROBOT/target/visible /ROBOT/target/paired /ROBOT/target/pair_state /ROBOT/target/distance /ROBOT/target/worker_id /ROBOT/aruco/id /ROBOT/aruco/distance /ROBOT/aruco/pose /ROBOT/goal_pose /ROBOT/local_costmap/costmap
```

### T3 · localization — 유지
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 launch turtlebot4_navigation localization.launch.py namespace:=/ROBOT map:=$(ros2 pkg prefix mro_bringup)/share/mro_bringup/maps/auto_map.yaml
```

### T4 · rviz — 유지 (2D Pose Estimate 누르지 않음)
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 launch turtlebot4_viz view_navigation.launch.py namespace:=/ROBOT
```

### T5 · 셔틀 리그 — 유지 (워치독 · ArUco 검출기 · 세마포어는 관제 것)
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 launch mro_navigation shuttle_step1_rig.launch.py namespace:=/ROBOT bay:=BAY real_aruco:=true semaphore:=false safety_participants:="shuttle_mission,follow_target,return_dock"
```
띄운 뒤 로봇이 도크에 그대로 있어야 한다 (워치독은 dock_status 를 받기 전·도킹 중엔 cmd_vel 을 내지 않는다).

### T6 · 셔틀 노드 — 유지 · Nav2 보다 먼저
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 run mro_navigation shuttle_mission_node --ros-args -r __ns:=/ROBOT -r /tf:=tf -r /tf_static:=tf_static --params-file $(ros2 pkg prefix mro_bringup)/share/mro_bringup/params/common.yaml --params-file $(ros2 pkg prefix mro_bringup)/share/mro_bringup/params/ROBOT.yaml
```
`초기자세 … 전송` 확인. T3 에 `AMCL cannot publish` 가 계속되거나 `dock_status 15 s` 경고면 이 창만 재시작 (첫 발행이 디스커버리 매칭 전에 나가 유실된 것).

### T7 · 복귀 노드 — 유지 (**finish 전에 떠 있는지 확인** — 9/10 16:14 에 6번 놓쳤다)
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 run mro_navigation return_dock_node --ros-args -r __ns:=/ROBOT -r /tf:=tf -r /tf_static:=tf_static --params-file $(ros2 pkg prefix mro_bringup)/share/mro_bringup/params/common.yaml --params-file $(ros2 pkg prefix mro_bringup)/share/mro_bringup/params/ROBOT.yaml
```

### T8 · Nav2 — 유지 (`Managed nodes are active` 까지 기다림)
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh nav2 && ros2 launch mro_navigation safety_nav2.launch.py namespace:=/ROBOT dwb_sim_time:=1.0
```
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 param get /ROBOT/controller_server FollowPath.sim_time
```
`dwb_sim_time:=1.0` — 9/10 17:15 회차에서 출입구 모서리 `Failed to make progress` 9회 → 0회. 글로벌·로컬 inflation 0.25.

### T9 · 추종 리그 — 유지 (재식별 = 인지 실물)
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && ros2 launch mro_navigation follow_rig.launch.py namespace:=/ROBOT show_window:=false paired:=real
```

### T10 · 로봇 시점 창 — 유지
```bash
cd ~/mro-multi-amr && source tools/shuttle_env.sh && python3 tools/detection_view.py --ns /ROBOT
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
| 로봇이 스캔 없이 멈춤 (`STUCK: 유효한 LiDAR 스캔 없음`) | 로봇↔PC 링크 끊김 (13:46) | `ros2 topic hz /ROBOT/scan` → RPi `sudo systemctl restart turtlebot4.service` |
| 리그를 띄우자 로봇이 도크에서 빠짐 | Create3 가 cmd_vel 수신에 반응 (14:34·14:55) | 워치독이 도킹 중 발행 금지 — 재발 시 `ros2 topic info -v /ROBOT/cmd_vel` 로 다른 발행자 확인 |
| 작업자를 봐도 SEARCHING 만 반복 | 재식별 sim < 0.34 (`unpaired`) | 인형을 웹캠1 앞에 정지 2~3 s 로 재등록, 또는 manual_pair |
| RETURN 눌러도 즉시 FAILED | 추종 취소와 복귀 첫 목표 경합 (17:15) | PR #88 에서 수정 (IDLE 대기·1회 재전송). 그 전 판이면 RETURN 한 번 더 |
| 힌트 방향으로 안 돌고 20° 씩 | `webcam_hint` 가 서버에 없음 | PC-W2 `echo $ROS_DISCOVERY_SERVER`, 로봇 PC `ros2 node list | grep webcam_hint` |
