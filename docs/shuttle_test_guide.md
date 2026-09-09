# 셔틀 미션 실주행 시험 가이드

`shuttle_mission_node`(S4 셔틀)를 실물 로봇으로 시험하는 절차다. 담당 주행 박병후·이시율.
공통 실행 환경은 `docs/setup.md`, 토픽·상태 문자열의 뜻은 `docs/interfaces.md` 4절이 기준이다.

> **왜 따로 두나** — 셔틀은 관제(`fleet_manager`)·인지(`aruco_detector`)·세마포어가 다 붙어야
> 온전히 돌지만, 그것들이 준비되기 전에도 주행 골격만 떼어 확인할 수 있다. 무엇을 대체하고
> 무엇을 못 보는지 미리 적어두지 않으면 "돌았다"와 "검증했다"가 섞인다.

---

## A. 단계별 시험 설계

| 단계 | 목적 — 무엇이 참인지 확인하나 | 전제 | 합격 조건 |
|---|---|---|---|
| **1** | **정상 경로가 끝까지 돈다** — 상태열, 목표 도달, 토큰 획득·반납 시점 | 마커 주입, 선반 없음, 1대 | B-5 판정 7항목 |
| **2** | **실패 경로가 옳게 실패한다** — 오도착 거부, T_lim 포기, estop 보류 | 1단계와 같은 환경 | `FAILED` 뒤 `WAITING`·`arrived` 미발행 |
| 3 | ArUco 실물 판정, **마커 기준 접근 잔차**, 재탐지, 선반 여유 | 마커·선반 설치 | 오도착 0건 (FR-08), 접근 잔차 ≤ 0.05 m |
| 4 | P1 FIFO·E-3 교착, 대기공간 정차 중 타 로봇 통과 | 로봇 2대 | 교착 자동 해소, 접촉 0 |
| 5 | AC-08 판정 사슬 (TS-07 배송 20회) | `fleet_manager` | `missions.result` 일치 |

**2단계를 3단계보다 앞에 둔다.** 실패 경로는 마커·선반 없이 전부 확인할 수 있고, 잘못된 실패
판정은 정상 동작 오류보다 조용히 번진다 (I-024·I-026).

**대체 가능한 것과 아닌 것**

| 없는 것 | 대체 | 못 보게 되는 것 |
|---|---|---|
| `fleet_manager` | `mission/assigned` 를 터미널에서 직접 발행 | DB 기록·`missions.result`·리드타임 (5단계로 미룸) |
| ArUco 마커·`aruco_detector` | `aruco/id`·`aruco/distance` 를 터미널에서 주입 | 실제 마커 판정·거리 정확도 (3단계) |
| `aruco/pose` (주입으로는 못 만든다) | **대체 없음.** 셔틀에 `bay_align_enable:=false` 를 주고 접근 단계를 끈다 | **베이 접근 전체** — 로봇이 waypoint 에 선 채로 끝나 판정 4 를 못 한다 |
| `zone_semaphore` | **대체 없음.** 안 띄우면 노드가 '단독 모드'로 통과시켜 토큰 로직이 검증되지 않는다 | 판정 5 전체 |
| 로봇 2대 | 1대로 진행 | P1 경합·교착 (4단계) |

---

## B. 1단계 시험 절차

### B-0. 사전 점검

- [ ] **아레나 정리** — 베이 A `(-0.78, 0.14)` 주변과 **그 서쪽 0.5 m**(마커 쪽 접근 구간), P1 통로
      (x −0.8~0 / y 0.45~0.9), 대기공간 `(-1.00, 0.94)` 경로
- [ ] **로봇을 도크에 물려서 시작** — 초기자세 설정이 도킹 상태를 전제한다
- [ ] E-stop 손 닿는 위치, 감시자 배치
- [ ] **PC-F 디스커버리 서버를 로봇 전원보다 먼저** — `fastdds discovery -i 1 -l 0.0.0.0 -p 11811`
- [ ] **사전 점검 스크립트** — 위를 사람이 눈으로 확인하는 대신 돌린다 (B-1·B-2 를 마친 터미널에서)

```bash
tools/shuttle_preflight.sh pre
```

`is_docked` 실측·중복 노드·네임스페이스 없는 스택·이전 세션 잔재까지 본다. **FAIL 이 있으면
고치기 전에 T1 로 넘어가지 않는다** — 9/7 실패 세 번이 전부 여기서 걸렸을 것들이다.

### B-1 · B-2. 모든 터미널의 첫 줄 — 빌드와 source 를 한 줄로

**시험에 쓰는 모든 터미널에서 이 한 줄로 시작한다.** 빌드까지 겸한다.

```bash
cd ~/rokey_intel1/main_project/mro-multi-amr && source tools/shuttle_env.sh build
```

빌드가 이미 끝났으면 `build` 를 뺀다. **T3(Nav2) 터미널만** `nav2` 를 붙인다 —
그 터미널만 `ROS_SUPER_CLIENT=False` 여야 한다.

```bash
source tools/shuttle_env.sh          # T1·T2·T4·T7·T8 및 명령을 치는 모든 터미널
source tools/shuttle_env.sh nav2     # T3 전용
source tools/shuttle_env.sh build    # 코드를 고친 뒤 (빌드 후 source 까지)
```

`환경 준비됨 — ROS_DOMAIN_ID=3 · …` 이 찍히면 된 것이다.

> **`source` 로 실행해야 한다.** `./tools/shuttle_env.sh` 처럼 실행하면 자식 프로세스의
> 환경만 바뀌고 지금 터미널에는 아무 효과가 없다. 같은 이유로 `tools/shuttle_preflight.sh` 는
> 환경을 **검사만** 하고 고치지 못한다 — 둘을 분리한 이유다.

<details><summary>이 스크립트가 하는 일 — 손으로 할 때</summary>

```bash
source /etc/turtlebot4_discovery/setup.bash && source /home/hv-04/turtlebot4_ws/install/setup.bash && source ~/rokey_intel1/main_project/mro-multi-amr/ros2_ws/install/setup.bash
```

| 빠뜨리면 | 증상 |
|---|---|
| `/etc/turtlebot4_discovery/setup.bash` | `ROS_DOMAIN_ID`·`ROS_DISCOVERY_SERVER` 가 없어 **다른 터미널의 노드를 영원히 못 찾는다** (`Waiting for at least 1 matching subscription(s)...`) |
| `turtlebot4_ws` | 셔틀 노드가 `turtlebot4_navigation` 임포트에 실패해 즉사 |
| `mro-multi-amr/ros2_ws` | `Package 'mro_navigation' not found` |

빌드는 `colcon build --symlink-install --packages-select mro_interfaces mro_bringup mro_navigation mro_fleet`.
</details>

### B-3. 터미널 기동 — 순서를 지킬 것

> **기동 순서가 9/8 에 바뀌었다 (I-035).** 셔틀 노드(T7)를 **Nav2(T3) 보다 먼저** 띄운다.
> 셔틀이 기동 시 초기자세를 보내고, 그 TF 위에서 Nav2 의 `global_costmap` 이 정상 활성화된다.
> 거꾸로 띄우면 costmap 활성화가 실패하고 `lifecycle_manager` 는 **나중에 TF 가 생겨도 재시도하지
> 않는다** — 예전 순서로는 매 회차 T_lim 120 초를 통째로 태웠다.
>
> ```
> T1 localization → T2 RViz → T4 리그 → T7 셔틀(초기자세) → T3 Nav2 → 게이트 → 미션
> ```

> **⚠ 아래 명령마다 `source` 를 앞에 붙여 놓았다. 떼지 말 것 (9/8 실주행).** B-2 를 따로 치는
> 구조였더니 T3 에서 실제로 빠졌고, `failed to send response to …/change_state (timeout)` 로
> 라이프사이클 기동이 중간에 멈췄다 (D-7).

**T1 · localization**
```bash
cd ~/rokey_intel1/main_project/mro-multi-amr && source tools/shuttle_env.sh && ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot3 map:=$(ros2 pkg prefix mro_bringup)/share/mro_bringup/maps/auto_map.yaml
```

**T2 · RViz** — **2D Pose Estimate 클릭 금지.** 초기자세는 셔틀 노드가 기동 시 `dock_pose` 로 잡는다
```bash
cd ~/rokey_intel1/main_project/mro-multi-amr && source tools/shuttle_env.sh && ros2 launch turtlebot4_viz view_navigation.launch.py namespace:=/robot3
```

**T3 · Nav2** — **T7 을 먼저 띄운 뒤에** 온다. 이 터미널만 `shuttle_env.sh nav2`
(`ROS_SUPER_CLIENT=False`). **`nav2` 를 빼거나 source 를 건너뛰면 기동이 중간에 멈춘다 (D-7).**
```bash
cd ~/rokey_intel1/main_project/mro-multi-amr && source tools/shuttle_env.sh nav2 && ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot3
```

T7 이 이미 초기자세를 보냈으므로 `map→odom` TF 가 있고, `global_costmap` 이 곧바로 활성화된다.
T7 에 `초기자세 (0.02, -0.80, 8°) 전송` 이 안 보였다면 **T3 를 띄우기 전에 원인을 잡는다** —
그 상태로 띄우면 아래 복구 절차를 타게 된다.

<details><summary>순서를 거꾸로 띄웠을 때 — 복구 (I-035)</summary>

`bt_navigator` 가 `inactive [2]` 로 남아 있고 **초기자세를 나중에 보내도 낫지 않는다.**
`lifecycle_manager` 가 재시도하지 않기 때문이다 (9/7 실측: `initialpose` 발행 8 초 뒤에도
`global_costmap` 은 `inactive`). 활성화를 사람이 다시 걸어야 한다.

```bash
ros2 topic pub --once /robot3/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.019, y: -0.795}, orientation: {z: 0.066451, w: 0.997790}}}}"
ros2 service call /robot3/lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 2}"
```

좌표는 `robot3.yaml` 의 `dock_pose` `[0.019, -0.795, 0.133]` 이다 (yaw 0.133 rad → z 0.066451,
w 0.997790). **RViz 의 2D Pose Estimate 로 대신하지 않는다** — 사람이 찍은 값이 들어가면
두 로봇의 map 프레임이 어긋난다 (I-013).
</details>

**T4 · 시험 리그** — 세마포어 + 안전 감시 + 마커 주입을 한 터미널로 (옛 T4·T5·T6)
```bash
cd ~/rokey_intel1/main_project/mro-multi-amr && source tools/shuttle_env.sh && ros2 launch mro_navigation shuttle_step1_rig.launch.py
```

세 노드는 시험 중에 볼 로그가 사실상 없어서 묶었다. 화면에는 워치독 기동 한 줄과 세마포어의
토큰 2줄만 나온다 — **마커 주입의 `publishing #N` 은 로그 파일로 보내 화면을 비웠다.**
베이를 바꾸려면 `bay:=B` (주입 ID 는 `arena.yaml` 의 `bays.<bay>.aruco_id` 에서 읽는다).

**실물 마커를 쓰는 3단계 — 진짜 검출기를 띄운다** (주입은 자동으로 꺼진다)
```bash
cd ~/rokey_intel1/main_project/mro-multi-amr && source tools/shuttle_env.sh && ros2 launch mro_navigation shuttle_step1_rig.launch.py real_aruco:=true
```
`aruco_detector_node` 가 `aruco/id`·`aruco/distance` 에 더해 **`aruco/pose`** 를 낸다.
**베이 접근(마커까지 0.6 m)은 이것 없이는 못 한다** — 주입은 pose 를 만들 수 없어 셔틀이
목적지를 몰라 미션을 포기한다. 주입으로만 돌리려면 셔틀에 `-p bay_align_enable:=false`.

> **셔틀(T7)과 Nav2(T3)는 묶지 않았다.** 셔틀은 판정 7항목이 전부 나오는 주 관찰 대상이고,
> Nav2 는 실패 원인이 자기 로그에 있다. 합치면 costmap 경고에 판정 로그가 묻힌다.

<details><summary>리그가 안 뜰 때 — 옛 방식 (터미널 3개)</summary>

```bash
ros2 run mro_fleet zone_semaphore_node                                        # 기동 로그 없음이 정상
ros2 run mro_navigation safety_watchdog_node --ros-args -r __ns:=/robot3      # 셔틀보다 먼저
ros2 topic pub -r 5 /robot3/aruco/id std_msgs/msg/Int32 "{data: 1}" & ros2 topic pub -r 5 /robot3/aruco/distance std_msgs/msg/Float32 "{data: 0.9}"
```
</details>

**T7 · 셔틀 미션** — **T3 보다 먼저.** `robot.launch.py` 대신 단독 실행(그쪽은 아직 준비 안 된
인지 노드까지 띄운다)
```bash
ros2 run mro_navigation shuttle_mission_node --ros-args -r __ns:=/robot3 -r /tf:=tf -r /tf_static:=tf_static --params-file $(ros2 pkg prefix mro_bringup)/share/mro_bringup/params/common.yaml --params-file $(ros2 pkg prefix mro_bringup)/share/mro_bringup/params/robot3.yaml 2>&1 | tee ~/shuttle_step1_$(date +%m%d_%H%M).log
```

**T8 · 상태 관찰**
```bash
cd ~/rokey_intel1/main_project/mro-multi-amr && source tools/shuttle_env.sh && ros2 topic echo /robot3/shuttle/status
```

### B-4. 미션 발행 — 확인하고 보낼 것

**⓪ 하드 게이트 — 통과하지 못하면 회차를 시작하지 않는다 (I-035 PM 결정 9/8)**

```bash
tools/shuttle_preflight.sh nav       # bt_navigator = active [3] 인가
tools/shuttle_preflight.sh mission   # 위 + 토큰·마커 주기·미션 구독
```

**`bt_navigator` 가 `active [3]` 가 아니면 미션을 발행하지 않는다.** 그 상태로 보내면 회차가
"Nav2 미준비" 로 끝나 아레나·측위에 대해 아무것도 알려주지 않는다. 회차 하나를 버리는 것보다
1 분 먼저 확인하는 편이 싸다. 스크립트는 FAIL 이 있으면 **종료 코드 1** 을 낸다.

**FAIL 이 없으면 ④ 발행으로 간다.** 손으로 확인하려면 아래를 차례로 한다.

**① 노드·세마포어가 떴는지**

T7 에 아래가 보여야 한다. `arena.yaml`·`dock_pose` 적재와 **기동 시 초기자세 전송**을 함께 뜻한다.
```
[robot3.shuttle_mission]: [READY]
[robot3.shuttle_mission]: 대기: /robot3/mission/assigned 에 Mission(ASSIGNED) 오면 출발
[robot3.shuttle_mission]: 초기자세 (0.02, -0.80, 8°) 전송 — 자신의 도크 map 좌표
[robot3.shuttle_mission]: Nav2 를 아직 띄우지 않았다면 지금 띄울 것 — 이 TF 로 global_costmap 이 활성화된다 (I-035)
```
**초기자세 줄이 없으면 T3 를 띄우지 않는다.** 대신 이유가 로그에 적혀 있다 —
`기동 시 초기자세 보류 — 도크에 물려 있지 않다` (로봇을 도크에 올린다) ·
`… dock_status 를 N s 안에 받지 못했다` (로봇 전원·디스커버리) ·
`기동 시 초기자세 보류: dock_pose 가 유효하지 않다` (`--params-file` 경로).
T4 는 로그가 없으므로 서비스로 확인한다.
```bash
ros2 service list | grep zone_token
```

**② Nav2 가 '떴다' 가 아니라 'active' 인지** — 셔틀 노드는 후자만 인정한다

> **⚠ 아래 `grep -c` 만으로는 부족하다 (9/7 저녁 발견).** 이 패턴은 이름이 `robot3/` 로 시작하고
> **살아 있는** 노드만 센다. **죽은 세션이 디스커버리 서버에 남긴 유령 노드**는 `ros2 node list` 에
> 그대로 보이므로 `1` 이 나와 통과하지만, T3 를 띄우면 이름 충돌로 활성화가 끝나지 않는다.
> 위 스크립트가 이것을 본다 (D-5).

```bash
ros2 node list | grep -c "robot3/lifecycle_manager_localization"
```
**`1` 이어야 한다.** 2 이상이면 다른 PC 나 이전 세션이 같은 로봇을 물고 있다. 그 상태에서는
라이프사이클 활성화가 끝나지 않아 `bt_navigator` 가 `active` 에 도달하지 못한다
(9/7 실주행에서 이것 때문에 T_lim 120 초를 통째로 날렸다). `docs/setup.md` 의
**"Nav2 launch 는 로봇당 한 PC 에서만"** 이 이 뜻이다.

```bash
ros2 lifecycle get /robot3/bt_navigator
```
**`active [3]`** 이어야 한다.

**③ 구독이 붙었는지**
```bash
ros2 topic info /robot3/mission/assigned --verbose
```
`Subscription count: 1` 과 `Node name: shuttle_mission` 을 확인한다.

**④ 발행**
```bash
ros2 topic pub --once /robot3/mission/assigned mro_interfaces/msg/Mission "{mission_id: 'T1', worker_id: 'W1', robot_ns: '/robot3', bay: 'A', state: 1}"
```
`Waiting for at least 1 matching subscription(s)...` 뒤에 `publishing #1` 이 나오면 **전달된 것**이다.
2 초 안에 T7 에 `미션 T1 · 베이 A · 마커 1 · 작업자 W1` 이 뜨지 않으면 한 번 더 보낸다.

### B-5. 판정 기준

| # | 확인 | 어디서 | 합격 |
|---|---|---|---|
| 1 | 상태열 | T8 | `SHUTTLE` → `AT_BAY` → `SHUTTLE` → `WAITING` → `READY`. **`WAITING` 은 1회** |
| 2 | 초기자세 | T7 | `초기자세 (0.02, -0.80, 8°)` — `robot3.yaml` 의 `dock_pose` 와 일치 |
| 3 | 베이 도달 오차 | T7 | `베이 A 도착 — … 오차 N m`, N ≤ 0.25 (Nav2 허용치, I-029) |
| 4 | **접근 잔차** | T7 | `접근 종료 — … 잔차 +N m`. \|N\| ≤ 0.05 (`bay_align_tol_xy_m`). **AMCL 과 무관한 마커 상대 측정이라 판정 3 보다 엄격하다** (아래 D-3) |
| 5 | **토큰 시점** | T7 + T4 | `적재 체류` 다음 줄이 `P1 획득`, `대기공간 도착` 다음 줄이 `P1 반납 (대기공간 도착 — P1 통과 완료)`. T4 에 획득·반납 각 1줄 |
| 6 | 소요 시간 | T7 | `대기공간 도착 — 소요 N s`, N < 120 |
| 7 | 합류 신호 | `ros2 topic echo /robot3/shuttle/arrived` | `data: true` 1회 |

### B-6. 중단·복구

- **T7 중단**: `Ctrl+C` → `P1 반납 (노드 종료)` 후 깔끔히 종료된다
- **T4 리그 중단**: `Ctrl+C` 에 **세마포어·워치독이 트레이스백을 뱉는다. 정상이다** — 두 노드는
  아직 종료 예외를 잡지 않는다 (셔틀만 처리했다). 동작에는 지장 없다
- **토큰이 남았을 때**: T4 리그 재시작. 또는 60 초 뒤 세마포어가 `강제 회수` 로그와 함께 회수한다
- **PC·세션을 바꿀 때**: 이전 노드를 반드시 내린다 — `pkill -INT -f "ros2 launch|nav2_|rviz2"`

---

## C. 로그 해석 매뉴얼

### C-1. 노드 이름과 관찰 대상

로그 앞머리에 **네임스페이스가 붙는다.**

| 터미널 | 로그의 노드 이름 | 볼 것 |
|---|---|---|
| **T7 셔틀** | `[robot3.shuttle_mission]` | **주 관찰 대상.** 모든 판정이 여기서 나온다 |
| T4 리그 | `[zone_semaphore]` (네임스페이스 없음) | 토큰 왕복 2줄 — **판정 5** |
| T4 리그 | `[robot3.safety_watchdog]` | `ESTOP:` 이 뜨면 즉시 |
| T3 Nav2 | `[robot3.*]` | **실패했을 때만.** 목표 거부·경로 실패 원인이 여기 있다 |

### C-2. 정상 진행 로그 (T7)

```
[INFO] [robot3.shuttle_mission]: [READY]
[INFO] [robot3.shuttle_mission]: 대기: /robot3/mission/assigned 에 Mission(ASSIGNED) 오면 출발
```
기동 완료. `arena.yaml`·`dock_pose` 적재 성공을 뜻한다.

```
[INFO] [robot3.shuttle_mission]: 미션 T1 · 베이 A · 마커 1 · 작업자 W1
```
미션 채택. **T_lim 120 초 시계가 여기서 시작한다.**

```
[INFO] [robot3.shuttle_mission]: 초기자세 (0.02, -0.80, 8°) 전송 — 자신의 도크 map 좌표
[INFO] [robot3.shuttle_mission]: Nav2 준비됨 (bt_navigator active)
[INFO] [robot3.shuttle_mission]: 언독
[INFO] [robot3.shuttle_mission]: [SHUTTLE]
```
초기자세를 보내야 AMCL 이 `map→odom` TF 를 내기 시작한다. 그 전까지 Nav2 costmap 이
`StaticLayer: "map" ... does not exist` 를 뱉는 것은 **정상**이며, 이 줄 직후 멎어야 한다.
언독은 블로킹이라 몇 초간 로그가 멈춘다 — 멈춘 것이 아니다.

```
[INFO] [robot3.shuttle_mission]: 베이 A → (-0.78, 0.14) 이동 시작
[INFO] [robot3.shuttle_mission]: 베이 A 도착 검증 통과 — 목표 (-0.78, 0.14) / TF 추정 (-0.71, 0.19), 오차 0.090 m, TF 시각 ..., 지연 0.020 s
[INFO] [robot3.shuttle_mission]: [AT_BAY]
[INFO] [robot3.shuttle_mission]: 마커 1 확인 (거리 0.90 m)
```
**판정 3.** `베이 A 도착 검증 통과`의 **TF 추정 좌표·시각·지연을 적어둔다** — 판정 4에 쓴다. 물리적 실제 좌표는 줄자로 별도 확인한다.

```
[INFO] [robot3.shuttle_mission]: 베이 접근 — 마커 정면 0.60 m · 좌우 +0.00 m 를 목표로 한다
[INFO] [robot3.shuttle_mission]: 접근 1/2 — 마커 base_link (0.980, +0.070) m, 방위 +4.1°, 마커 법선 오차 +2.3° / 이동 앞 +0.380 · 좌 +0.070 m (합 0.386), 회전 +1.2° [측량]
[INFO] [robot3.shuttle_mission]: 접근 2/2 — 마커 base_link (0.615, +0.012) m, … (합 0.019), 회전 +0.3° [측량]
[INFO] [robot3.shuttle_mission]: 접근 완료 (2회) — 더 움직이지 않는다
```
**판정 4.** `이동 앞/좌` 는 **회전–직진–회전**으로 실행된다 (`Spin` → `DriveOnHeading` → `Spin`).
차동구동이라 옆으로 못 가서다. 마지막 줄이 `접근 완료` 면 잔차가 `bay_align_tol_xy_m`(0.05 m)
안이라는 뜻이고, 2회를 다 쓰고도 못 맞추면 `접근 종료 — … 잔차 +N m` 로 실측이 남는다.

**`마커 법선 오차` 는 참고값이다** — 마커 자세로 각도를 잡는 경로(`bay_align_use_marker_yaw`)는
기본으로 꺼져 있다. 이 값이 회차마다 안정적이면 켤 근거가 된다 (정사각 마커 두 해 문제, 계약 6절).

**`[측량]` 표시**는 회전 기준이 `arena.yaml` 의 베이 방위라는 뜻이다. 그래서 **AMCL 의 방위
오차는 이 보정으로 지워지지 않는다** — 위치만 지워진다.

<details><summary>마커를 못 봐서 재탐지가 돈 경우</summary>

```
[WARN] [robot3.shuttle_mission]: 마커 미확인 1/2 (관측 None) — 제자리 회전으로 재탐지
[INFO] [robot3.shuttle_mission]: 마커 재탐지 — 제자리 회전 8회, ±60° 를 15° 간격으로
[INFO] [robot3.shuttle_mission]: 마커 1 재탐지 성공 — 기준 방위에서 +30° (4/8), 거리 0.98 m
```
가까운 쪽부터 좌우 교대(−15° → +15° → −30° → +30° …)로 훑는다. **병진하지 않는다** — 마커를
못 본다는 것은 자기 위치를 못 믿는다는 뜻이라 어디로 갈지 모르기 때문이다.
찾은 각도를 적어 둘 것: 매 회차 같은 쪽으로 크게 나오면 `arena.yaml` 의 베이 방위나 마커
설치 위치를 재실측해야 한다는 신호다.
</details>

```
[INFO] [robot3.shuttle_mission]: 적재 체류 5.0 s
[INFO] [robot3.shuttle_mission]: P1 획득
[INFO] [robot3.shuttle_mission]: [SHUTTLE]
```
**판정 5 전반.** `적재 체류` 바로 다음이 `P1 획득` 이어야 한다 — 베이를 아직 떠나지 않은
시점이라는 뜻이다 (순서 기반 판정 = 방법 B).

```
[INFO] [robot3.shuttle_mission]: 대기공간 → (-1.00, 0.94) 이동 시작
[INFO] [robot3.shuttle_mission]: 대기공간 도착 검증 통과 — 목표 (-1.00, 0.94) / TF 추정 (-1.02, 0.91), 오차 0.036 m, TF 시각 ..., 지연 0.020 s
[INFO] [robot3.shuttle_mission]: P1 반납 (대기공간 도착 — P1 통과 완료)
[INFO] [robot3.shuttle_mission]: [WAITING]
[INFO] [robot3.shuttle_mission]: 대기공간 도착 — 소요 62.4 s
[INFO] [robot3.shuttle_mission]: [READY]
```
**판정 5 후반·6.** `대기공간 도착` 으로 시작하는 줄이 **둘**이다 — 앞은 주행 도착(좌표),
뒤는 미션 완료(소요 시간).

**`[WAITING]` 이 딱 한 번, 대기공간 도착 뒤에만** 나오는 것이 이 시험의 가장 중요한 확인이다.
이 시각이 AC-08 배송 판정과 K-05 리드타임의 기준점이다 (계약 9.2, I-024).

**T4(세마포어) 정상 출력 — 이 두 줄뿐**
```
[INFO] [zone_semaphore]: P1 획득 → /robot3
[INFO] [zone_semaphore]: P1 반납 ← /robot3
```

### C-3. 상태 문자열 (T8)

| 값 | 의미 | 언제 |
|---|---|---|
| `READY` | 미션 대기 | 기동 시, 미션 종료 후 |
| `SHUTTLE` | 셔틀 주행 중 | undock 직후 / P1 획득 직후 — **두 번이 정상** |
| `AT_BAY` | 베이 도착. ArUco 확인(필요하면 재탐지 회전)·접근·체류·토큰 대기 | 베이 도착 시 |
| `WAITING` | **대기공간 도착 확정** | 미션당 **1회** |
| `FAILED` | **포기** | 재시도 중에는 나오지 않는다 |

`RETURNING`/`DOCK_WAIT`/`DOCKING`/`DOCKED` 는 `return_dock` 소관이라 이 시험에서 나오면 안 된다.

### C-4. 경고(WARN) — 미션은 계속된다

| 로그 | 뜻 | 조치 |
|---|---|---|
| `Nav2 미준비 (N s 경과 / T_lim 120 s): /robot3/bt_navigator/get_state 서비스가 없다 — Nav2 미기동` | T3 이 안 떴다 | T3 확인 |
| `… get_state 가 응답하지 않는다 — 같은 네임스페이스에 Nav2 가 중복으로 떠 있는지 확인할 것` | **노드 중복** | B-4 ② 를 볼 것 |
| `… bt_navigator 가 'inactive' 이다 — 라이프사이클 활성화가 끝나지 않았다` | 활성화 미완 | `lifecycle_manager` 로그 확인 |
| `mro_bringup 을 찾지 못했다 — arena_yaml 을 직접 넘길 것` | ws source 누락 | B-2 |
| `zone_token 서비스 없음 — 단독 모드로 진행` | **T4 미기동** | **판정 5 를 못 한다.** T4 띄우고 재시험 |
| `P1 점유 중 (holder=…)` | 다른 로봇이 보유 | 1대 시험에선 나오면 안 된다 — 이전 토큰 잔존. T4 재시작 |
| `마커 미확인 1/2 (관측 None) — 제자리 회전으로 재탐지` | 3 초 안에 마커 확인 실패 | 주입/실물 검출기 확인. `관측 None` 은 토픽이 아예 안 온다는 뜻. 이어서 재탐지 회전이 돈다 |
| `마커 1 재탐지 성공 — 기준 방위에서 +30° (4/8)` | 회전 스윕으로 찾았다 | **정상이지만 각도를 적어 둘 것.** 매 회차 같은 쪽으로 크게 나오면 베이 방위·마커 위치 재실측 신호 |
| `마커 재탐지 실패 — ±60° 안에 없다` | 스윕을 다 돌고도 못 찾음 | 다음 줄이 `ArUco 불일치` 포기다. 마커 설치·조명·거리(≤1.5 m)를 볼 것 |
| `aruco/pose 표본 없음` (포기 사유로도 나온다) | 검출기가 pose 를 안 낸다 | `real_aruco:=true` 로 띄웠는지, `publish_pose: true` 인지, 카메라 TF 가 있는지 |
| `estop 중 — 해제까지 … 보류` | 워치독 정지 | 전방 0.3 m 장애물 제거 |
| `TF: …` / `TF 없음 — 베이 waypoint 기준으로…` | `map→base_link` 조회 실패 | AMCL 수렴 확인. 판정 4 의 의미가 약해진다 |
| `이미 미션이 있다 — 새 배정을 버린다` | 발행을 두 번 했다 | 정상 방어. 무시 |

### C-5. 포기(ERROR) — `미션 T1 포기: <사유>`

| 사유 | 원인 | 조치 |
|---|---|---|
| `T_lim 120.0 s 초과` | 전체 시간 초과 | D-1·D-2 |
| `ArUco 불일치 — 오도착 (기대 1, 관측 2)` | 요청 베이와 다른 마커 | T6 값을 베이에 맞춘다 (A=1, B=2, C=3) |
| `ArUco 불일치 — 오도착 (기대 1, 관측 None)` | 마커 토픽이 없다 | T6 미기동·주기 부족 |
| `Nav2 미준비 — N회 확인 동안 active 가 아니다: …` | Nav2 가 `active` 가 아니다 | **T_lim 을 태우지 않고 곧장 포기한다** (9/8, I-035). 기동 순서를 B-3 대로 (T7 → T3) |
| `dock_status 를 N s 안에 받지 못했다` | 로봇이 안 보인다 | 예전에는 여기서 무한 대기라 `FAILED` 도 안 났다 (D-2). 로봇 전원·디스커버리 |
| `dock_pose 가 유효하지 않다 ([0.0, 0.0, 0.0])` | params 미전달 | `--params-file` 경로 확인 (I-013) |
| `arena.yaml 미적재` / `arena_yaml 경로가 비었다` | 파일을 못 읽음 | B-1 빌드, B-2 source |
| `arena.yaml 에 베이 X waypoint 가 없다` | 미션 `bay` 오타 | A/B/C 만 유효 |
| `arena.yaml waiting_zone 에 robot3 이 없다` | 네임스페이스 불일치 | `-r __ns:=/robot3` 확인 |
| `베이 A 목표 거부됨` | Nav2 가 goal 을 반려 | **T3 로그.** 목표가 lethal 안이거나 액션 서버 미기동 |
| `베이 A 이동 실패: FAILED` | 경로 실패 | T3 로그. 통로 폭·inflation·장애물 |
| `aruco/pose 표본 없음 — 접근할 기준이 없다` | 마커 자세 토픽이 없다 | 주입 리그만 띄웠다면 정상이다 — `real_aruco:=true` 를 쓰거나 셔틀에 `bay_align_enable:=false` |
| `접근 이동량 N m 가 상한 1.0 m 를 넘는다` | 마커까지가 너무 멀거나 측정이 튐 | 베이 waypoint 가 마커에서 1.6 m 넘게 떨어졌는지, 거리값이 튀는지 확인 |
| `bay_marker_range_m N m 가 하한 0.35 m 보다 가깝다` | 목표 거리 설정 오류 | `common.yaml`. 0·음수도 여기서 걸린다 |
| `접근 1 전진 실패: FAILED` / `… 회전① 실패` | 상대 이동이 중간에 깨졌다 | **T3 behavior_server 로그.** estop 이 걸렸는지, `time_allowance`(20 s) 를 넘겼는지 |
| `behavior_server 의 spin·drive_on_heading 이 없다` | Nav2 미기동·미활성 | T3 확인. 이 액션이 없으면 접근·재탐지가 전부 불가능하다 |

**포기한 미션은 폐기된다.** 그 뒤 로봇이 늦게 도착해도 `WAITING`·`arrived` 를 내지 않는다
(계약 7절 이중 계상 금지). 이것을 확인하는 것이 2단계다.

---

## D. 증상별 판별

### D-1. 미션을 보냈는데 T7 이 무반응

`미션 T1 · 베이 A…` 가 안 뜬다. 순서대로 본다.

1. 발행 터미널에 `env | grep ROS_DOMAIN_ID` — 없으면 B-2 (가장 흔한 원인)
2. `ros2 topic info /robot3/mission/assigned --verbose` 로 `Subscription count: 1`
3. 디스커버리 지연으로 첫 발행이 유실될 수 있으니 한 번 더 보낸다

### D-2. `미션 T1 · 베이 A…` 뒤 아무것도 안 나온다

**9/8 에 고쳤다.** 예전에는 `navigator.getDockedStatus()` 가 `dock_status` 를 **무한 대기**해
(타임아웃이 없다) T_lim 조차 돌지 않아 `FAILED` 도 안 나왔다. 지금은 셔틀 노드가 같은 구독을
쓰되 `dock_status_wait_s`(기본 5 s) 상한을 두고, 시간 안에 못 받으면
`dock_status 를 N s 안에 받지 못했다` 로 **포기한다.**

그래도 로그가 멎는다면 그 구간은 `undock()`·`dock()` 블로킹이다 (수 초). 그보다 오래 멎으면
로봇 전원·네임스페이스·디스커버리를 확인한다.

### D-3. 접근 잔차·도착 검증 실패 (I-037)

**9/8 로 절차가 바뀌었다.** 옛 '창고 안쪽 0.3 m 전진' 은 없어졌다. 그것은 출발점을 모르는
상대량이라 AMCL 오차 0.38~0.53 m 가 최종 정차 위치에 그대로 남았고, `bay_advance_m` 을 키워도
덮이지 않는 문제였다. 지금은 **마커에서 0.6 m 인 자리로 간다** — 목적지가 마커 기준 절대
위치라 출발점이 어디든 도착점이 같다.

**두 오차를 섞지 말 것.**

| | 판정 3 `베이 A 도착 검증` | 판정 4 `접근 종료 … 잔차` |
|---|---|---|
| 기준 | map 좌표 (AMCL) | 마커 (카메라 상대 측정) |
| 허용 | 0.25 m (`arrival_xy_tolerance_m`) | 0.05 m (`bay_align_tol_xy_m`) |
| 무엇이 틀리면 나빠지나 | AMCL 수렴, Nav2 | 마커 검출 거리, 실행 오차 |

판정 3 이 나쁜데 판정 4 가 좋으면 **정상이다** — 그게 이 기능의 목적이다 (AMCL 이 틀려도
마커 기준으로는 제자리에 선다). 반대로 판정 4 가 나쁘면 마커 쪽을 본다.

- `신선한 TF를 … 안에 받지 못함`: TF 발행·시각·로봇 네임스페이스를 점검한다. 접근은 회전량을
  '측량 베이 방위 − 지금 방위' 로 잡으므로 `map→base_link` 없이는 시작하지 않는다.
- `도착 검증 실패 — Nav2 SUCCEEDED`: 새 TF의 오차가 `arrival_xy_tolerance_m`를 넘었다.
  Nav2 feedback·AMCL·물리 위치를 대조한다.
- `접근 종료 … 잔차`: 2회 반복하고도 못 맞춘 값이다. 줄자 실측과 대조해 **마커 거리 자체가
  틀렸는지**(`depth_k`, 0.8 m 아래는 pnp 로 떨어진다) **실행이 틀렸는지**를 가른다.

`common.yaml`의 `tf_pose_timeout_s=3.0`, `tf_max_age_s=0.3`,
`arrival_xy_tolerance_m=0.25`는 실험 초깃값이다. 단독 실행에도 common과 robot3 YAML을 둘 다 전달한다.
허용오차를 낮추는 실험은 기존 I-029 진동과 함께 별도로 검증한다.

**접근 이동은 `progress_checker` 에 안 걸린다** — `Spin`·`DriveOnHeading` 은 behavior_server 의
상대 이동이라 controller 의 `SimpleProgressChecker`(10 초 안에 0.5 m)를 지나지 않는다.
대신 `time_allowance`(`bay_align_timeout_s`, 20 s)가 상한이다.

### D-7. 라이프사이클 기동이 중간에 멈춘다 — `failed to send response to …/change_state (timeout)`

**증상.** `lifecycle_manager` 가 어느 노드까지는 `Configuring …` 을 찍다가 거기서 멈춘다.
멈춘 노드 쪽 로그에는 이렇게 남는다.

```
[smoother_server] [WARN] failed to send response to /robot3/smoother_server/change_state
                         (timeout): client will not receive response
```

**무슨 일인가.** 요청은 갔고 노드는 **실제로 수행했다**(`Created smoother …` 까지 찍힌다).
돌아오는 **응답만** 못 왔다. `lifecycle_manager` 는 영영 안 올 응답을 기다리며 멈추고,
그 뒤 순서(`planner_server`·`bt_navigator`·`behavior_server` …)는 configure 조차 시작되지 않는다.
그래서 **T7 은 `Nav2 미준비` 로 미션을 포기**하게 된다.

**9/8 실주행에서 확인된 원인 — `source` 누락.** T3 를 `source tools/shuttle_env.sh nav2` 없이
띄웠을 때 났고, 붙여서 다시 띄우니 정상 기동했다. 같은 날 아침 T1(`map_server/change_state`)에서도
같은 문장이 났고 그때는 터미널 재시작으로 풀렸다.

**조치.**
1. **멈춘 터미널만** 내렸다가 `source` 를 붙여 다시 띄운다. T1·T7 은 건드리지 않는다 —
   초기자세는 이미 나갔고 다시 띄우면 순서(I-035)를 처음부터 다시 만들어야 한다.
2. T3 는 반드시 `shuttle_env.sh **nav2**` 다 (`ROS_SUPER_CLIENT=False`). 나머지는 인자 없이.
3. 다시 떠도 같은 곳에서 멈추면 디스커버리 문제다 — D-5·D-5-1 로 간다.

**같이 나오지만 무관한 것.**
`The configured inflation radius (0.100) is smaller than the computed inscribed radius (0.186)`
는 **의도한 설정**이다 (local 0.10 / global 0.30 분리, 9/8 P1 통과 설정). ERROR 로 찍히지만
기동 실패와 무관하니 여기서 원인을 찾지 말 것.

### D-5. 노드 이름은 보이는데 서비스가 응답하지 않는다 (유령 노드)

`ros2 node list` 에 `/robot3/bt_navigator` 가 있는데 `ros2 lifecycle get /robot3/bt_navigator` 가
**아무것도 출력하지 않는** 경우다. 9/7 저녁에 실제로 있었다.

- **원인** — 죽은 세션의 등록이 디스커버리 서버에 남아 있다. `controller_server`·`planner_server`
  같은 나머지 Nav2 서버가 함께 안 보이면 유령이 거의 확실하다.
- **왜 위험한가** — **같은 네임스페이스에 같은 이름**이 둘이 되어 라이프사이클 활성화가 끝나지
  않는다. **9/7 에 T_lim 120 초를 날린 것이 이 증상이다.**
- **판별** — `tools/shuttle_preflight.sh pre` 가 잡는다. 손으로 하면 `ros2 lifecycle get
  /robot3/bt_navigator` 가 **아무것도 출력하지 않는지**를 본다.
- **조치** — ① 그 노드를 띄운 PC 를 찾아 내린다 ② 못 찾으면 **PC-F 의 `fastdds discovery` 를
  재시작**해 등록을 비우고 로봇을 재부팅한다 ③ 그냥 두면 시간이 지나 사라지기도 한다
  (9/7 저녁 유령 `bt_navigator` 가 그렇게 없어졌다).

**반대로, 네임스페이스가 다른 노드는 무해하다 (9/7 저녁 실측).** 이름 없이 뜬
`/amcl`·`/map_server`·`/lifecycle_manager_localization` 한 벌이 있어도 `/robot3` 시험에 영향이 없다.

| 확인한 것 | 결과 |
|---|---|
| 그쪽 `lifecycle_manager` 의 `node_names` | `['map_server', 'amcl']` — **상대 이름**이라 `/map_server`·`/amcl` 로만 풀린다. `/robot3/*` 를 관리 대상으로 잡지 않는다 |
| 토픽 | 그쪽은 전부 루트(`/tf`·`/map`·`/scan`·`/bond`), robot3 은 `/robot3/tf`·`/robot3/scan` — 겹치는 것이 없다 |
| 루트 `/scan` | **Publisher count: 0** — 그쪽 amcl 은 아무도 발행하지 않는 토픽을 듣고 있어 영원히 `inactive` 다 |

그래서 preflight 는 이것을 **WARN 으로만** 낸다. 위험한 것은 '네임스페이스가 없는 것' 이 아니라
**'완전한 이름이 같은 것'** 이다. 단, T1·T3 를 `namespace:=` 없이 띄우면 그때는 진짜로 충돌한다 —
**항상 네임스페이스를 준다.**

### D-5-1. 노드가 '있다/없다' 를 볼 때 — 데몬 캐시와 spin-time (9/8 실측)

**둘 다 거짓말을 한다. 방향이 반대다.**

| 방법 | 문제 | 9/8 에 실제로 본 것 |
|---|---|---|
| `ros2 node list` (데몬) | **죽은 노드를 오래 들고 있다** (과다 보고) | 로봇이 안 뜬 상태에서 robot3 노드 13 개를 보여줬다 |
| `ros2 node list --no-daemon` | 기본 `--spin-time` 이 **1 초**뿐이라 디스커버리 서버가 응답할 시간이 없다 (과소 보고) | **0 개**를 돌려줬다 |

디스커버리 서버 방식에서는 `ros2` CLI 호출마다 새 참가자가 만들어지고, 상대를 찾는 데
**3~10 초**가 걸린다. 그래서 `--no-daemon` 만 붙이고 짧게 끊으면 "아무것도 없다" 가 나온다.
**9/8 에 이것을 보고 디스커버리 서버가 죽었다고 오진해 로봇을 헛되이 재부팅했다.**

```bash
ros2 node list --no-daemon --spin-time 8     # 이렇게 본다
```

`--spin-time` 8 과 20 이 같은 결과(35 개)를 주고, 데몬은 죽은 launch 프로세스 하나를 더
들고 있었다. `tools/shuttle_preflight.sh` 는 이 방식으로 노드 목록을 받는다.

> **이름이 보이는 것과 데이터가 흐르는 것은 다르다.** 가장 확실한 생존 확인은 실제 데이터다 —
> `dock_status` 는 약 1 Hz 로 계속 오므로 preflight 는 이것으로 로봇 생존을 판정한다.
> 유령 판별(D-5)의 `lifecycle get` 무응답도 같은 원리다.

### D-6. `bt_navigator` 가 `inactive [2]` 에서 안 올라온다 (I-035)

**노드별로 찍어보면 어디서 막혔는지 바로 보인다.**

```bash
for n in amcl controller_server planner_server bt_navigator \
         global_costmap/global_costmap local_costmap/local_costmap; do
  printf '%-32s ' "$n"; ros2 lifecycle get /robot3/$n
done
```

9/7 저녁 robot3 의 실제 출력:

| 노드 | 상태 | 뜻 |
|---|---|---|
| `amcl` | `active [3]` | 떠 있지만 **초기자세를 못 받아 `map→odom` 을 안 낸다** |
| `local_costmap` | `active [3]` | `odom` 프레임만 쓰므로 영향 없음 |
| **`global_costmap`** | **`inactive [2]`** | **`map→base_link` 를 못 얻어 여기서 막혔다** |
| `planner_server` 이하 | `inactive [2]` | 앞이 막혀 순서가 진행되지 않음 |

**초기자세를 나중에 보내도 저절로 낫지 않는다** — `lifecycle_manager` 가 재시도하지 않는 것을
실측으로 확인했다(초기자세 발행 후 `amcl_pose` 는 나왔으나 8 초 뒤에도 `global_costmap` 은
`inactive`). 위 B-3 의 `RESUME` 으로 복구한다.

### D-4. 그 밖

| 증상 | 판별 | 대응 |
|---|---|---|
| `P1 획득` 이 `적재 체류` 보다 먼저 | 순서 뒤바뀜 | 코드 버그. 로그를 남길 것 |
| `[WAITING]` 이 두 번 | 상태 중복 발행 | 코드 버그. 관제 연동 시 리드타임이 깨진다 |
| `P1 반납` 이 없다 | T4 에도 반납 로그 없음 | Ctrl+C 후 `P1 반납 (노드 종료)` 확인 |
| 소요 시간 100 s 초과 | 판정 6 아슬아슬 | 도크→베이 1.23 m, 베이→대기공간 0.83 m 라 여유가 크지 않다. Nav2 재계획이 잦은지 T3 로그 확인 |
| 언독에서 멈춘 듯 | `언독` 뒤 몇 초 정지 | 정상. `undock()` 이 블로킹이다 |

---

## F. 회차 기록과 다음 회차 계획

### F-1. 9/7 실주행 (robot3, 1단계)

| 회차 | 결과 | 베이 오차 | 대기공간 | 소요 | 비고 |
|---|---|---|---|---|---|
| 1 | 완주 | **0.72 m** | 도착 (오차 0.26 m · **줄자 실측 0.32 m**) | 40.4 s | 판정 3 불합격. 나머지는 노드 로그 기준 충족 |
| 2 | **T_lim 포기** | **0.84 m** | 미도달 | 120 s 초과 | `(-0.92, 0.27)` 벽 앞 교착 95 s (I-036) |

**2회차에서 토픽으로 확인한 것** — `shuttle/status` 는 `SHUTTLE→AT_BAY→SHUTTLE→FAILED→READY` 로 **`WAITING` 을 내지 않았고**, `shuttle/arrived` 도 **발행되지 않았다.** 포기한 미션이 도착으로 계상되지 않는다는 계약 7절이 실물에서 지켜졌다 — **2단계 합격 조건을 부수적으로 충족했다.**
`/fleet/zone_event` 는 `P1 /robot3 GRANT`(21:22:29) → `P1 /robot3 RECLAIM`(21:23:28)으로, 세마포어의 60 초 강제 회수가 설계대로 동작했다.

**오늘 새로 등록한 이슈** — I-035(Nav2 초기자세 선후) · I-036(estop 중 복구 불가) · I-037(AMCL 과 TF 0.65 m 불일치). I-009 에는 P1 통과 실주행 결과를 더했다.

### F-1-1. 9/8 실주행 (robot3, 3단계 · 실물 마커) — 18 회차

코드가 회차마다 바뀌었으므로 **회차끼리 비교하지 않는다.** 이 절은 그 18 회에서 나온
**값**만 남긴다 — 다음 회차의 전제가 되는 것들이다.

| 잰 것 | 값 | 어떻게 나왔나 |
|---|---|---|
| **선반 앞면 ↔ 마커 벽** | **0.425 m** | 마커까지 0.715 m 인 순간 워치독 `ESTOP: 전방 0.29 m` → 뺀 값. **ESTOP 1 회에서 나온 역산이라 베이별 재확인이 필요하다** |
| **접근 거리 하드 바닥** | **0.80 m** | 워치독 0.425 + `clear_dist` 0.38 = 0.805 · Nav2 0.425 + 0.379 = 0.804 |
| 카메라 수평 FOV | **38.5°** | `camera_info` fx 566.7 → 1007.4 (×1.7778), 704 px crop |
| `base_link←카메라` 병진 | **[−0.060, 0, 0.244]** | `tf2_echo` — 카메라가 중심보다 **6 cm 뒤**다 |
| Nav2 도착 편차 | **0.11 m 부족 · 0.14 m 좌** (합 0.178 m), 8 회 부호 일정 | `amcl_pose` · 줄자 |
| Nav2 `Spin` 분해능 | **2.3°/사이클** | `min_rotational_vel` 0.4 ÷ `cycle_frequency` 10 |

- **T18 이 전 구간을 완주했다** — 대기공간 오차 0.143 m, 44.4 s, `[WAITING]` 1 회.
- **남은 것이 I-044** — T18 접근 잔차 **−0.275 m**(목표 0.900, 실제 0.625). 같은 순간
  검출기 0.899 · 노드 계산 1.104. **여유 0.095 m 가 관측 오차 0.275 m 보다 작다.**
- **마커 법선(ε)은 아직 못 믿는다** — 4 회 중 3 회 명령한 회전량을 따라오지 않았고
  검출기 기운각이 9 → 17 → 22° 로 발산했다. 인지에 전달됨 (I-039 와 같은 뿌리로 추정).

### F-2. 다음 회차 계획 — 변인통제

**한 번에 하나만 바꾼다.** 두 개를 같이 바꾸면 어느 것이 효과를 냈는지 영원히 모른다.
저장소 밖 파일(`turtlebot4_ws`)을 고칠 때는 **먼저 `.orig_<날짜시각>` 로 백업**한다.

> **파일이 아니라 노드에서 확인한다 (9/9 규칙).** 9/8 의 `stateful` 실험은 "바꿨는데
> 효과가 없었다" 로 기록했지만, 9/9 확인 결과 `nav2.yaml` 이 백업본과 **바이트까지 같았다**
> — 그 설정이 실행 중이었다는 증거가 없다. `install/` 과 `src/` 도 같아서 어디서 되돌아왔는지
> 알 수 없다. **바꾼 값은 띄운 뒤 반드시 노드에 물어보고, 그 출력을 회차 기록의 첫 줄로 남긴다.**
>
> ```bash
> ros2 param get /robot3/controller_server general_goal_checker.xy_goal_tolerance
> ros2 param get /robot3/controller_server general_goal_checker.stateful
> ros2 param get /robot3/controller_server FollowPath.sim_time
> ```

**끝난 것**

| 무엇 | 결과 |
|---|---|
| `inflation_radius` **global 0.25~0.30 / local 0.10 분리** (9/8) | **P1 통과 성공.** 경로는 통로 중앙으로 밀고 실행은 벽 가까이 허용한 것이 답이었다. 단일 값(0.25 도 0.15 도)으로는 안 됐다 |
| I-036 estop 탈출 (9/8) | 갇힘 76~95 s → **2.1~4.5 s**, 매번 스스로 회전으로 탈출 |
| 3단계 완주 (9/8 T18) | 상태열·토큰·소요 전부 충족. 남은 것은 접근 잔차뿐 (I-044) |
| robot8 `dock_pose` 재확인 | **하지 않는다 (9/9 PM).** 기체만 바꿨고 도크는 그대로다. 다만 robot8 이 기동부터 측위가 어긋나면 이 값을 첫 번째로 의심한다 (I-013) |

**지금 풀려는 문제 셋** — 아래 R0~R5 는 전부 이 셋 중 하나에 붙는다.

| # | 증상 | 지금 세운 가설 |
|---|---|---|
| ① | 목표에 **0.2~0.3 m 못 미치고** 완료 판정 | **설정값 그대로다.** `xy_goal_tolerance` 가 `general_goal_checker`(39행)·DWB(64행) 둘 다 **0.25 m** 인데 관측 편차가 0.178 m 다. `stateful: true` 가 첫 만족에서 **래치**하므로 로봇은 허용원의 **가까운 쪽 가장자리**에서 멈추고, 접근 방향이 늘 같아 부호도 일정하다 |
| ② | 마커 접근·정렬의 정확도·안정성 (I-044) | **R0 이 좁혔다 (F-3-1).** **(c) 는 기각** — 0.57~1.19 m 에서 기울기 0. **(b) 는 확인됐고 크기가 +0.0135 m 로 작다.** `aruco/distance` 는 1.4 cm 수준에서 맞고 `pose` 크기와도 소수 셋째 자리까지 같다. **남은 것은 (d) 와, T18 의 「실제 0.625」를 어디까지 쟀는가**다 — 앞 끝까지였다면 0.625+0.173=0.798 로 검출기 기준 0.826 과 3 cm 차이라 PM 이 제기한 모순이 풀린다 |
| ③ | P1 에서 **찔끔찔끔** 가거나 선다 | **DWB 가 궤적을 비용으로 깎지 않고 무효로 버린다.** 내다보는 거리가 `sim_time` 1.7 × `max_vel_x` 0.26 = **0.44 m** 인데 통로 실폭이 0.79 m 다. 회전 성분이 섞인 궤적이 전부 벽에 닿아 사라지고 저속 직진 몇 개만 남는다. `BaseObstacle.scale` 이 0.02 라 **점수가 아니라 표본이 문제**다 |

> **선반 여유 — 3단계 전에 확인할 것.** 마커는 선반 뒤 벽면에 있으므로 **선반 앞면이 마커보다 로봇에 가깝다.** 주행 보고로 최근접점이 0.32 m 라 워치독 `stop_dist` 0.30 m 에는 안 걸리지만 여유가 2 cm 뿐이다. `bay_marker_range_m` 을 0.5 → 0.6 으로 올렸던 이유이고, 9/8 실주행에서 **0.9 m** 로 다시 올렸다 (PR #52 — 선반 앞면이 마커 벽에서 0.425 m 앞이라 바닥이 0.80 m 다). 선반 위치가 바뀌면 이 값을 먼저 다시 잡는다. **0.425 m 는 ESTOP 1 회에서 역산한 값이라, 베이를 바꾸거나 선반을 옮기면 R0(F-3) 에서 줄자로 다시 잡는다.**

**순서**

| 순서 | 무엇 | 바꾸는 것 | 관측 · 합격선 |
|---|---|---|---|
| **R0** | **정지 측정 (주행 없음, 두 로봇 다)** — ② 를 가른다 | 없음 | F-3 참조. **주행으로는 이 셋을 못 가린다** — 매 회차 세 값이 같이 움직인다 |
| **R1** | 기준선 (robot3, 무변경, 3 회) | 없음 | 도착 오차·접근 잔차·P1 소요. **비교 기준을 만드는 회차** — 9/8 의 18 회는 코드가 계속 바뀌어 기준이 안 된다. **접근 잔차는 반드시 앞뒤 두 번 재서 `base_link` 로 환산한다** — I-044 가 여기서 닫힌다 (F-3-1) |
| **R2** | ① — `xy_goal_tolerance` **0.25 → 0.10** (39·64행, 3 회) · **R1 뒤 1순위** | 이거 하나 | 도착 오차 **≤ 0.10 m**. 목표 근처 진동이나 `progress_checker`(0.5 m / 10 s) 실패가 나면 0.15 로 되물린다. 그래도 0.2 대면 허용오차가 아니라 **측위 편향**이고 R5-b 로 간다 |
| **R3** | ③ — DWB `sim_time` **1.7 → 1.0** (61행) | 이거 하나 | **P1 왕복만** 3 회. 통과 소요와 **속도가 0 이 된 횟수·길이**. 전체 미션을 돌리면 다른 실패에 가린다. 실패하면 컨트롤러를 RPP 로 바꾸는 건을 따로 설계한다 |
| **R4** | robot8 복제 — R1 과 **완전히 같은 설정** 3 회 | 로봇만 | ② 가 개체 문제인가 공통인가. R0 과 교차 검증된다. `bay:=A`, `-r __ns:=/robot8`, `robot8.yaml` |
| **R5-a** | ~~목표 거리 상향~~ **불필요** | — | **R0 이 (c) 를 기각했다** (F-3-1). `bay_marker_range_m` 0.9 를 유지한다. 다만 `bay_align_min_range_m` **0.45 → 0.80** 은 그대로 한다 — 잘못된 설정을 거르는 유일한 방어가 자기가 잰 바닥보다 낮다 (I-044, PM 요청) |
| **R5-b** | R0 이 **(b)/(d)** 로 나오거나 R2 가 실패하면 | 기준면 보정·선반 재실측 · 또는 `localization.yaml` | `update_min_a`(0.2) · `resample_interval`(1) · `recovery_alpha_slow/fast`(0.0). `update_min_d` 는 9/7 에 0.25→0.05 로 이미 바꿨다 (I-029) |

**아직 순서에 넣지 않은 것**

- **재탐지가 실제로 도는지** — 일부러 15~30° 틀어 놓고 시작하면 `마커 N 재탐지 성공 — 기준
  방위에서 +X°` 가 나와야 한다. R1 회차에 **끼워서** 본다 (설정을 안 바꾸므로 변인이 아니다).
- **`bay_align_use_marker_yaw: true`** — 위에서 모은 법선 오차가 안정될 때까지 켜지 않는다.
  9/8 에 켰다가 되돌렸고, 그 근거는 `common.yaml` 주석에 남아 있다.

**매 회차 공통** — 기록 5종(`shuttle/status`·`shuttle/arrived`·`/fleet/zone_event`·`amcl_pose`·`tf2_echo map base_link`)을 걸고 시작하고, **위의 `ros2 param get` 출력을 첫 줄에 남긴다.**

### F-3. R0 절차 — 정지 측정 (주행 없음)

**로봇을 움직이지 않는다.** 손으로 밀어 세우고 값을 읽는 것이 전부라 위험이 0 이고,
estop·`Collision Ahead` 로 회차를 날릴 일이 없다. 15 분이면 끝난다.

**왜 이걸 먼저 하나** — I-044 는 숫자 셋이 안 맞는 문제다. 주행 중에는 로봇 위치·검출값·
계산값이 **동시에** 움직여서 어느 것이 틀렸는지 영원히 안 갈린다. 세워 놓고 재면 갈린다.

#### 기준점이 셋이다 — 줄자는 `base_link` 로 통일한다

| | 무엇 | 누가 쓰나 |
|---|---|---|
| ① | **`base_link`** | Nav2 footprint · 워치독 · **우리 접근 계산**. 여기로 통일한다 |
| ② | `oakd_rgb_camera_optical_frame` 원점 | TF. URDF 마지막 joint 가 `xyz="0 0 0"` 이라 **OAK-D 의 CAD 장착점**이지 렌즈 광학중심이 아니다 |
| ③ | 물리 렌즈면 | **인지가 `depth_k` 를 여기 맞춰 적합했다** (`robot8.yaml` 「카메라 렌즈면→마커 줄자」) |

TF 는 depth 가 ②에서 나온다고 보고 변환하는데 `aruco/distance` 는 ③ 기준으로 보정돼 있다.
**②↔③ 틈은 아무도 모델링하지 않았고 그대로 `base_x` 에 실린다.** 이것이 후보 (b) 다.

URDF 체인을 더하면 `base_link → shell(0,0,0.030) → bracket(−0.118,0,0.05257) →
oakd_link(+0.0584,0,0.09676) → optical(0,0,0)` 이라 x 가 **−0.0596** — `tf2_echo` 실측
**−0.060** 과 맞는다. **이 6 cm 는 TF 가 이미 처리한다.** 줄자를 렌즈면 기준으로 재면
같은 6 cm 를 두 번 세게 된다.

#### 앞뒤 두 번 재면 로봇 상수가 필요 없다

`base_link` 는 바닥 원판의 중심이라 자를 댈 데가 없다. 앞 끝과 뒤 끝을 재서 평균한다.

```
d_base = (front + rear) / 2        ← base_link 까지의 참값
radius = (rear − front) / 2        ← 덤. Nav2 footprint 전제 0.189 와 맞는지
```

**반지름은 그냥 얻는 값이 아니다.** 워치독 `clear_dist` 0.38 도 `DriveOnHeading` 충돌
예측(0.19 + 0.189)도 전부 이 0.189 위에 서 있다. 어긋나면 **베이 하드 바닥 0.80 이 같이
움직인다.** 매 지점 기록한다.

#### 준비

마커(베이 A ID)를 평소 자리에 두고, 로봇 정면이 마커를 향하게 세운다.
**`base_link` 기준 0.6 / 0.8 / 1.0 / 1.2 m** 네 지점 — 즉 앞 끝이 대략 0.41 / 0.61 /
0.81 / 1.01 m 에 오게 놓고, 매번 **앞뒤를 다 잰다.**

#### 띄우는 것 — 검출기 하나 (Nav2·셔틀 불필요)

```bash
cd ~/rokey_intel1/main_project/mro-multi-amr && source tools/shuttle_env.sh && ros2 run mro_perception aruco_detector_node --ros-args -r __ns:=/robot3 --params-file ros2_ws/install/mro_perception/share/mro_perception/config/perception.yaml --params-file ros2_ws/install/mro_bringup/share/mro_bringup/params/robot3.yaml
```

**`--params-file` 순서가 중요하다** — `robotN.yaml` 이 뒤에 와야 `depth_k` 가 이긴다 (I-040).

#### 각 지점에서

```bash
cd ~/rokey_intel1/main_project/mro-multi-amr && source tools/shuttle_env.sh && python3 ~/aruco_static_check.py --ns /robot3 --front 0.41 --rear 0.79
```

줄자(앞·뒤) · `aruco/distance` · `pose` 크기 · **`base_link` 전방거리**(셔틀이 실제로 쓰는 자)
넷을 한 화면에 내고 `aruco_static.csv` 에 한 줄씩 쌓는다.

#### 읽는 법 — 기울기와 절편을 나눠 본다

**기준점 모호성은 절편에만 들어가고 기울기는 못 건드린다.** 그래서 ②↔③ 틈을 몰라도
(c) 는 판정할 수 있다.

| 나온 모양 | 결론 |
|---|---|
| `base_x − d_base` 가 **네 지점에서 일정** | **(b) 기준점.** ②↔③ 틈이다. 그 상수를 빼면 끝난다 |
| **거리 따라 커진다 / 0.8 m 에서 꺾인다** | **(c) `depth_k` 유효구간 밖.** k 는 0.8 m 이상 점으로만 적합됐고 그 아래는 pnp 대체다. 목표 거리를 그 위로 올린다 (R5-a) |
| 일정한데 그 값이 **0.2 m 급** | 카메라 내부 치수로는 설명이 안 된다 → **(d)** 선반·마커 재실측, 또는 검출기 자체 |
| `dist` 는 맞는데 `pose_r` 만 어긋난다 | 9/8 에 넣은 **거리 교체가 옳았다**는 확인. 남은 어긋남은 (a) 신선도이므로 `수신 X s 전` 로그를 본다 |
| `radius` 가 0.189 에서 2 cm 넘게 벗어난다 | **먼저 이것부터.** 하드 바닥 0.80 의 근거가 무너진다 |

#### 같이 해 두면 좋은 것 (각 15 초, 다른 파트가 기다리는 것들)

- **robot8 도 같은 4 지점을 잰다.** 인지 todo 가 «`depth_k` 0.028 은 줄자 3점을 **전부 적합에
  썼다** — 독립 검증점이 없어 **FR-35 는 이 값을 신뢰하지 않는다**» 로 열려 있다 (I-031).
  R0 이 그 독립점을 **네 개** 만들어 준다.
- **`camera_info` 를 받아 둔다** — `ros2 topic echo /robot8/oakd/rgb/camera_info --once` 와
  `/robot8/oakd/stereo/camera_info --once`. `tools/aruco_offline_check.py` 의 상수가 아직
  **9/7 robot4 값**이라 지금 상태로 robot8 을 검증하면 robot4 광학으로 합격을 매긴다 (I-031).
  **받는 김에 `cx`·`cy` 가 353 ≈ 704/2 인지 본다** — 아니면 16/9 전제가 robot8 엔 안 맞는다.

### F-3-1. R0 결과 (9/9, robot3 · 베이 A 마커 ID 1)

**8 지점. 로봇은 움직이지 않았다.**

| `d_base` | `aruco/distance` | 차 | 반지름 |
|---|---|---|---|
| 0.570 | 0.641 | +0.0713 | 0.170 |
| 0.780 | 0.850 | +0.0702 | 0.170 |
| 0.985 | 1.052 | +0.0675 | 0.170 |
| 0.995 ×3 | 1.070 ×3 | +0.0753 | 0.175 |
| 1.188 | 1.265 | +0.0779 | 0.172 |
| 1.188 | 1.263 | +0.0751 | 0.177 |

**평균 +0.0735 · 표준편차 3.2 mm · 범위 +0.0675~+0.0779.**

#### 판정

- **(c) `depth_k` 유효구간 — 기각.** 0.57 → 1.19 m 로 두 배 넘게 벌어지는 동안 차이가
  일정하고 **0.8 m 에서 꺾이지 않는다.** k 는 그 구간에서 잘 듣는다.
  **→ `bay_marker_range_m` 을 1.1 m 로 올릴 이유가 없다** (I-044 의 PM 지시 2번 불필요).
- **(b) 기준점 — 확인, 그리고 크기가 작다.** 상수 +0.0735 에서 TF 의 카메라 x 병진
  **0.060** 을 빼면 **+0.0135 m** 가 남는다. 인지가 밝힌 `depth_k` 잔차 ±1.5 cm 안이고,
  ②(URDF 광학원점)와 ③(물리 렌즈면) 사이 틈으로도 맞는 크기다.
  **`aruco/distance` 는 1.4 cm 수준에서 맞는다.**
- **`aruco/pose` 크기와 `aruco/distance` 가 소수 셋째 자리까지 같다** (8 지점 전부).
  검출기 docstring 의 「같은 자다」가 맞다. **그러면 9/8 에 관측한 「같은 순간 1.188 대
  0.943, 26 % 어긋남」은 두 토픽의 성질이 아니다** — 9/8 의 거리 교체 코드는 해가 되지
  않지만, 그것으로 문제가 풀렸다고 볼 근거는 이 측정으로 사라졌다.
- **재현성** — 같은 지점 3 회 전부 1.070, 산포 5 mm.

#### 로봇 반지름 0.173 — footprint 0.189 는 1.6 cm 보수적

Create 3 규격(지름 0.339 m → 반지름 0.170)과 맞는다. `nav2.yaml` 의 footprint 가 **로봇보다
크다** — 어긋난 것이 아니라 안전 쪽이므로 **그대로 둔다.** 하드 바닥 0.80 도 이 보수적인 값
위에서 계산됐으니 실제 여유는 계산보다 1.6 cm 더 있다.

#### 남은 것 — I-044 는 R0 만으로 안 닫힌다

전방 거리는 깨끗한데 **T18 의 「실제 0.625」가 어디까지 잰 값인지**가 남았다.

```
검출기 0.899 → base_link 기준  0.899 − 0.0735 = 0.826 m
기록된 0.625 가 앞 끝까지였다면  0.625 + 0.173 = 0.798 m
```

**3 cm 차이다.** 앞 끝까지 잰 값을 `base_link` 기준 목표 0.900 과 비교했을 가능성이 크다.
그렇다면 PM 이 제기한 모순 — 「0.625 는 하드 바닥 0.805 안쪽인데 왜 estop 이 안 걸렸나」 —
이 저절로 풀린다. **로봇은 0.80 바로 위에 있었다.** 잔차 −0.275 m 는 제어 오차가 아니라
자를 잘못 댄 것이 된다.

**아직 가설이다.** R1 에서 앞뒤 방식으로 다시 재서 확정한다 (R1 판정 항목에 넣었다).

#### 인지에 넘길 것 — robot8 `camera_info` (9/9 수신)

```
fx = fy = 566.0106   cx = 352.383   cy = 367.613   704 x 704
robot3 566.654 · robot4(옛 2호기) 569.951  →  robot8 은 robot3 과 0.11 % 차이
```

1. **`tools/aruco_offline_check.py` 의 `K_INFO`·`D` 를 이 값으로 갱신하면 I-031 todo 가 닫힌다.**
   지금은 9/7 robot4 값이라 robot8 을 robot4 광학으로 검증하게 된다.
2. **`cx` 는 맞고 `cy` 는 안 맞는다.** `cx` 352.38 ≈ 704/2 = 352 이라 **가로 16/9 crop 전제는
   성립**한다 (`camera_fx_scale` 1.7778 유지 가능). 그런데 `cy` 367.61 은 352 에서 **15.6 px**
   아래로 치우쳤다 — 정사각 crop 이 세로로는 가운데가 아니다. **세로 오차는 마커의 기운각·
   법선 추정에 직접 들어간다.** 9/8 에 법선이 4 회 중 3 회 안 맞고 기운각이 9→17→22° 로
   튄 것과 같은 자리라 I-039 와 엮어 볼 값이다.
3. **stereo `camera_info` 가 RGB 것과 완전히 같다** — `frame_id` 도 `oakd_rgb_camera_optical_frame`,
   K 도 동일, `d` 는 전부 0. depth 를 RGB 에 정합하는 드라이버면 정상이지만, `amr_detection` 이
   이것을 스테레오 고유값으로 알고 쓰고 있다면 문제다. **단정하지 않는다 — 확인 요청이다.**

### F-3-2. R1 결과 (9/9, robot3 · 기준선 3회 · 파라미터 무변경)

| 회차 | 베이 도착 오차 | 접근 | 결과 |
|---|---|---|---|
| R1a | **0.188 m** | 1/2 만 (2회차에서 마커 놓침) | **`탈출 회전 실패: FAILED`** — 180° Spin 이 1.8 s 만에 취소 |
| R1b | **0.145 m** | 2/2 | **완주 39.9 s** · 대기공간 오차 0.179 m |
| R1c | **0.198 m** | 1/2 | **`접근 1 회전② 실패: FAILED`** — 10.1° Spin 이 0.3 s 만에 취소 |

#### I-044 종결 — 잔차는 −0.064 m 이고 계통 오차다

**R1a 체류 중 줄자 실측: 앞 0.66 / 뒤 1.01 → `base_link` 0.835 m** (반지름 0.175 로 F-3-1 의
0.173 과 일치 — 측정 자체가 건전하다).

```
접근 1 시작   마커 base_link 1.169 m
이동 명령     +0.270 m
예상 도착     0.899 m       ← 목표 0.900 과 사실상 같다
줄자 실측     0.835 m
잔차          −0.064 m
```

**R1b 가 독립적으로 같은 값을 낸다** — 접근 1 뒤 예상 0.896 인데 접근 2 에서 노드가 잰 값이
0.829 로 **−0.067**. 두 회차가 3 mm 안에서 일치한다.

- **`DriveOnHeading` 이 일관되게 6.5 cm 더 간다.** 무작위가 아니라 계통 오차라 뺄 수 있다.
  0.1 m/s 에서의 감속 오버슛으로 보인다 (`bay_align_speed_mps`).
- **T18 의 「잔차 −0.275」는 기준점 착오였다** (F-3-1 의 가설이 맞았다). 0.625 는 **앞 끝까지**
  잰 값이고 `base_link` 로는 0.798 이다. **PM 이 제기한 모순 — 「0.625 는 하드 바닥 0.805
  안쪽인데 왜 estop 이 안 걸렸나」 — 는 로봇이 실제로 바닥 위에 있었기 때문이다.**
- **여유는 여전히 얇다** — 0.835 는 바닥 0.805 위로 3 cm 다. 오버슛 6.5 cm 를 보정하면
  0.899 가 되어 여유가 9.4 cm 로 회복된다. **이것이 TS-07 전 조치 1순위다.**

#### 새 발견 — 문제 ①과 베이 실패는 같은 문제였다

3회 중 2회가 죽었고 **둘 다 `Spin` 이 취소당했다.** 그리고 **도착 오차가 큰 두 회차가 죽고
작은 회차가 살았다** (0.188·0.198 실패 / 0.145 완주). 3점이라 단정하지 않지만 사슬이 맞는다.

```
xy_goal_tolerance 0.25  →  목표에서 0.15~0.20 m 덜 간 채 완료 판정
                        →  접근 방향 벽에 붙은 채로 정렬 시작
                        →  Spin 의 충돌 예측이 footprint 0.189 로 회전을 거부
                        →  미션 포기
```

**문제 ①(덜 주행)과 베이 접근 실패가 별개가 아니라 하나다.** R2 가 둘을 같이 칠 수 있으므로
**다음 회차 1순위를 R2 로 올린다** (원래 R1 다음이었다 — 순서는 그대로지만 근거가 세졌다).

#### 그 밖

- **마커 법선 오차 +8.6 / −15.2 / +10.3°** — 여전히 요동친다. `bay_align_use_marker_yaw: false`
  유지가 맞다. 인지에 넘긴 `cy` 15.6 px 치우침(F-3-1)과 같은 자리일 수 있다.
- **`aruco/distance` 와 노드 계산의 차가 0.061 m** (1.230 vs 1.169) — F-3-1 의 상수 0.0735 와
  1.2 cm 차이로, 주행 중에도 R0 의 관계가 유지된다.
- R1a 는 접근 2 에서 마커를 2 s 안에 못 봤다. 회전 뒤 재획득이 아슬아슬하다 —
  `bay_align_sample_s` 2.0 을 늘리는 것이 후보지만, **R2 와 섞지 않는다.**

---

## E. 이 시험으로 확인되지 않는 것

단위 검증(`test/test_shuttle_mission_fsm.py`)도 실주행 1단계도 아래는 못 본다. 헷갈리지 않게 적어둔다.

- **Nav2 가 실제로 P1 폴리곤을 지나는 경로를 그리는지** — 방법 B 는 경로 순서를 전제로 한다.
  전제가 깨지면(예: 우회로가 생기면) 토큰이 무의미해진다
- **대기공간 정차 중 로봇 몸통이 P1 을 막는지** — robot3 정차점이 폴리곤에서 0.20 m 인데
  반지름이 0.22 m 다. 2대 시험에서 다른 로봇이 지나갈 수 있는지로 판단한다
- **`fleet_manager` 가 이 상태열로 `missions.result` 를 옳게 확정하는지** — TS-07 (5단계)
- **ArUco '정면 ±30°'(계약 6절)** — 아무도 검사하지 않는다. `aruco_detector` 가 기운각을
  계산은 하지만 로그로만 남기고 발행하지 않는다 (PR #30)
