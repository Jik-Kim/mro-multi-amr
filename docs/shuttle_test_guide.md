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
| **1** | **정상 경로가 끝까지 돈다** — 상태열, 목표 도달, 전진 실제 이동량, 토큰 획득·반납 시점 | 마커 주입, 선반 없음, 1대 | B-5 판정 7항목 |
| **2** | **실패 경로가 옳게 실패한다** — 오도착 거부, T_lim 포기, estop 보류 | 1단계와 같은 환경 | `FAILED` 뒤 `WAITING`·`arrived` 미발행 |
| 3 | ArUco 실물 판정, 전진 후 선반 여유 | 마커·선반 설치 | 오도착 0건 (FR-08) |
| 4 | P1 FIFO·E-3 교착, 대기공간 정차 중 타 로봇 통과 | 로봇 2대 | 교착 자동 해소, 접촉 0 |
| 5 | AC-08 판정 사슬 (TS-07 배송 20회) | `fleet_manager` | `missions.result` 일치 |

**2단계를 3단계보다 앞에 둔다.** 실패 경로는 마커·선반 없이 전부 확인할 수 있고, 잘못된 실패
판정은 정상 동작 오류보다 조용히 번진다 (I-024·I-026).

**대체 가능한 것과 아닌 것**

| 없는 것 | 대체 | 못 보게 되는 것 |
|---|---|---|
| `fleet_manager` | `mission/assigned` 를 터미널에서 직접 발행 | DB 기록·`missions.result`·리드타임 (5단계로 미룸) |
| ArUco 마커·`aruco_detector` | `aruco/id`·`aruco/distance` 를 터미널에서 주입 | 실제 마커 판정·거리 정확도 (3단계) |
| `zone_semaphore` | **대체 없음.** 안 띄우면 노드가 '단독 모드'로 통과시켜 토큰 로직이 검증되지 않는다 | 판정 5 전체 |
| 로봇 2대 | 1대로 진행 | P1 경합·교착 (4단계) |

---

## B. 1단계 시험 절차

### B-0. 사전 점검

- [ ] **아레나 정리** — 베이 A `(-0.78, 0.14)` 주변과 **그 서쪽 0.5 m**(전진 목표), P1 통로
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

**T1 · localization**
```bash
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot3 map:=$(ros2 pkg prefix mro_bringup)/share/mro_bringup/maps/auto_map.yaml
```

**T2 · RViz** — **2D Pose Estimate 클릭 금지.** 초기자세는 셔틀 노드가 `dock_pose` 로 잡는다
```bash
ros2 launch turtlebot4_viz view_navigation.launch.py namespace:=/robot3
```

**T3 · Nav2** — 이 터미널만 `ROS_SUPER_CLIENT=False` (`shuttle_env.sh nav2` 가 해준다)
```bash
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot3
```

**T4 · 시험 리그** — 세마포어 + 안전 감시 + 마커 주입을 한 터미널로 (옛 T4·T5·T6)
```bash
ros2 launch mro_navigation shuttle_step1_rig.launch.py
```

세 노드는 시험 중에 볼 로그가 사실상 없어서 묶었다. 화면에는 워치독 기동 한 줄과 세마포어의
토큰 2줄만 나온다 — **마커 주입의 `publishing #N` 은 로그 파일로 보내 화면을 비웠다.**
베이를 바꾸려면 `bay:=B` (주입 ID 는 `arena.yaml` 의 `bays.<bay>.aruco_id` 에서 읽는다).
실물 마커를 쓰는 3단계에서는 `inject_aruco:=false`.

> **셔틀(T7)과 Nav2(T3)는 묶지 않았다.** 셔틀은 판정 7항목이 전부 나오는 주 관찰 대상이고,
> Nav2 는 실패 원인이 자기 로그에 있다. 합치면 costmap 경고에 판정 로그가 묻힌다.

<details><summary>리그가 안 뜰 때 — 옛 방식 (터미널 3개)</summary>

```bash
ros2 run mro_fleet zone_semaphore_node                                        # 기동 로그 없음이 정상
ros2 run mro_navigation safety_watchdog_node --ros-args -r __ns:=/robot3      # 셔틀보다 먼저
ros2 topic pub -r 5 /robot3/aruco/id std_msgs/msg/Int32 "{data: 1}" & ros2 topic pub -r 5 /robot3/aruco/distance std_msgs/msg/Float32 "{data: 0.9}"
```
</details>

**T7 · 셔틀 미션** — `robot.launch.py` 대신 단독 실행(그쪽은 아직 준비 안 된 인지 노드까지 띄운다)
```bash
ros2 run mro_navigation shuttle_mission_node --ros-args -r __ns:=/robot3 -r /tf:=tf -r /tf_static:=tf_static --params-file $(ros2 pkg prefix mro_bringup)/share/mro_bringup/params/robot3.yaml 2>&1 | tee ~/shuttle_step1_$(date +%m%d_%H%M).log
```

**T8 · 상태 관찰**
```bash
ros2 topic echo /robot3/shuttle/status
```

### B-4. 미션 발행 — 확인하고 보낼 것

**⓪ 한 번에 확인** — 아래 ①②③ 과 마커 주기까지 전부 본다

```bash
tools/shuttle_preflight.sh mission
```

**FAIL 이 없으면 ④ 발행으로 간다.** 손으로 확인하려면 아래를 차례로 한다.

**① 노드·세마포어가 떴는지**

T7 에 아래 두 줄이 보여야 한다. 이게 곧 `arena.yaml` 과 `dock_pose` 가 정상 적재됐다는 뜻이다.
```
[robot3.shuttle_mission]: [READY]
[robot3.shuttle_mission]: 대기: /robot3/mission/assigned 에 Mission(ASSIGNED) 오면 출발
```
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
| 4 | **전진 실제 이동량** | T7 | 베이 도착 실제 좌표 → 전진 도착 실제 좌표 거리. **0.3 m 에 못 미쳐도 정상일 수 있다** (아래 D-3) |
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
[INFO] [robot3.shuttle_mission]: 베이 A 도착 — 목표 (-0.78, 0.14) / 실제 (-0.71, 0.19), 오차 0.09 m
[INFO] [robot3.shuttle_mission]: [AT_BAY]
[INFO] [robot3.shuttle_mission]: 마커 1 확인 (거리 0.90 m)
```
**판정 3.** `베이 A 도착` 의 **실제 좌표를 적어둔다** — 판정 4 에 쓴다.

```
[INFO] [robot3.shuttle_mission]: 창고 안쪽 0.3 m 전진 → (-1.01, 0.18) 이동 시작
[INFO] [robot3.shuttle_mission]: 창고 안쪽 0.3 m 전진 도착 — 목표 (-1.01, 0.18) / 실제 (-0.95, 0.19), 오차 0.06 m
```
**판정 4.** 전진 목표는 **베이 도착 실제 좌표**에서 계산되므로 매번 다르다. 위 예시의 실제
이동량은 `(-0.71,0.19) → (-0.95,0.19)` = **0.24 m** 로 0.3 에 못 미친다 (D-3 참조).

```
[INFO] [robot3.shuttle_mission]: 적재 체류 5.0 s
[INFO] [robot3.shuttle_mission]: P1 획득
[INFO] [robot3.shuttle_mission]: [SHUTTLE]
```
**판정 5 전반.** `적재 체류` 바로 다음이 `P1 획득` 이어야 한다 — 베이를 아직 떠나지 않은
시점이라는 뜻이다 (순서 기반 판정 = 방법 B).

```
[INFO] [robot3.shuttle_mission]: 대기공간 → (-1.00, 0.94) 이동 시작
[INFO] [robot3.shuttle_mission]: 대기공간 도착 — 목표 (-1.00, 0.94) / 실제 (-1.02, 0.91), 오차 0.04 m
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
| `AT_BAY` | 베이 도착. ArUco 확인·전진·체류·토큰 대기 | 베이 도착 시 |
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
| `마커 미확인 1/2 (관측 None) — 정차 자세 재정렬` | 3 초 안에 마커 확인 실패 | T6 주입 확인. `관측 None` 은 토픽이 아예 안 온다는 뜻 |
| `estop 중 — 해제까지 … 보류` | 워치독 정지 | 전방 0.3 m 장애물 제거 |
| `TF: …` / `TF 없음 — 베이 waypoint 기준으로…` | `map→base_link` 조회 실패 | AMCL 수렴 확인. 판정 4 의 의미가 약해진다 |
| `이미 미션이 있다 — 새 배정을 버린다` | 발행을 두 번 했다 | 정상 방어. 무시 |

### C-5. 포기(ERROR) — `미션 T1 포기: <사유>`

| 사유 | 원인 | 조치 |
|---|---|---|
| `T_lim 120.0 s 초과` | 전체 시간 초과 | D-1·D-2 |
| `ArUco 불일치 — 오도착 (기대 1, 관측 2)` | 요청 베이와 다른 마커 | T6 값을 베이에 맞춘다 (A=1, B=2, C=3) |
| `ArUco 불일치 — 오도착 (기대 1, 관측 None)` | 마커 토픽이 없다 | T6 미기동·주기 부족 |
| `dock_pose 가 유효하지 않다 ([0.0, 0.0, 0.0])` | params 미전달 | `--params-file` 경로 확인 (I-013) |
| `arena.yaml 미적재` / `arena_yaml 경로가 비었다` | 파일을 못 읽음 | B-1 빌드, B-2 source |
| `arena.yaml 에 베이 X waypoint 가 없다` | 미션 `bay` 오타 | A/B/C 만 유효 |
| `arena.yaml waiting_zone 에 robot3 이 없다` | 네임스페이스 불일치 | `-r __ns:=/robot3` 확인 |
| `베이 A 목표 거부됨` | Nav2 가 goal 을 반려 | **T3 로그.** 목표가 lethal 안이거나 액션 서버 미기동 |
| `베이 A 이동 실패: FAILED` | 경로 실패 | T3 로그. 통로 폭·inflation·장애물 |
| `창고 안쪽 0.3 m 전진 이동 실패: FAILED` | 전진 실패 | D-3 |

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

`getDockedStatus()` 가 `dock_status` 를 **무한 대기**한다 (타임아웃이 없다). 로봇 전원·
네임스페이스·디스커버리를 확인한다. **이 구간은 T_lim 도 돌지 않으므로** `FAILED` 도 안 나온다 —
Ctrl+C 로 내리고 원인을 고친 뒤 다시 한다.

### D-3. 전진이 0.3 m 에 한참 못 미친다

Nav2 의 `xy_goal_tolerance` 가 **0.25 m 로 고정**이라(I-029 — 낮추면 목표 근처에서 진동해
도착 판정을 못 한다) 0.3 m 목표를 줘도 실제 이동은 **0.05~0.3 m** 사이다. 더 확실히 전진시켜야
하면 파라미터를 올린다 (재빌드 불필요).

```bash
ros2 run mro_navigation shuttle_mission_node --ros-args -r __ns:=/robot3 -p bay_advance_m:=0.55 --params-file $(ros2 pkg prefix mro_bringup)/share/mro_bringup/params/robot3.yaml
```

전진 목표에서만 `이동 실패: FAILED` 가 나면 `progress_checker` 를 의심한다 — 10 초 안에
0.5 m 이동을 요구하는데 목표가 0.3 m 뿐이다.

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

### D-4. 그 밖

| 증상 | 판별 | 대응 |
|---|---|---|
| `P1 획득` 이 `적재 체류` 보다 먼저 | 순서 뒤바뀜 | 코드 버그. 로그를 남길 것 |
| `[WAITING]` 이 두 번 | 상태 중복 발행 | 코드 버그. 관제 연동 시 리드타임이 깨진다 |
| `P1 반납` 이 없다 | T4 에도 반납 로그 없음 | Ctrl+C 후 `P1 반납 (노드 종료)` 확인 |
| 소요 시간 100 s 초과 | 판정 6 아슬아슬 | 도크→베이 1.23 m, 베이→대기공간 0.83 m 라 여유가 크지 않다. Nav2 재계획이 잦은지 T3 로그 확인 |
| 언독에서 멈춘 듯 | `언독` 뒤 몇 초 정지 | 정상. `undock()` 이 블로킹이다 |

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
