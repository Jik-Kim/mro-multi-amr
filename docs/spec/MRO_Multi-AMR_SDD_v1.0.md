# System Design Document (SDD) v1.0
## 비정형 항공 정비고(MRO) 환경 대응형 Multi-AMR 정비 지원 시스템 — Phase 1 실증

| 항목 | 내용 |
|---|---|
| 문서 번호 | SDD-MRO-001 |
| 버전 | **v1.0** (2026-09-11) — 제출본. 개정 이력은 `SDD_changelog.md` |
| 상위 문서 | BRD v3.6 (`MRO_Multi-AMR_BRD_v3.5.md`) — 요구사항 ID(BR-T · NFR · AC · TS)를 그대로 인용한다 |
| 구현 기준 | 저장소 `mro-multi-amr` — `docs/interfaces.md`(계약 v1.23) · `docs/SOT.md` · `docs/setup.md` |
| 작성 | PM 김병직 · 5장 파트별: 인지 정정인·민범진 / 웹캠 한세교·김학민 / 주행 박병후·이시율 / 관제 김민준 |
| 구성 | 강의안 Ex3 「System Design Document」 11절 구성을 따른다 |

---

## 1. Overview

정비사(스케일 인형+RC카)가 관제센터에서 자재를 요청하면 AMR(TurtleBot4 ×2)이 창고(A/B/C)에서 가져와 대기공간에서 만나고, 작업공간까지 따라다니며, 두 대가 사람 개입 없이 공존한다.

**Phase 1 이 답해야 할 질문**
1. 작업별 복장이 구분되는 다수 작업자 중 지정된 정비사를 신뢰성 있게 추종하는가 (동일 작업복은 Phase 2)
2. 두 대의 로봇이 좁은 통로에서 교착 없이 운용되는가
3. 사람·구조물에 대해 무사고가 성립하는가 — 인지·웹캠·네트워크 어느 것이 죽어도

**시나리오** (BRD 2.4): S1 진입·등록 → S2 준비(undock) → S3 요청 → S4 셔틀(베이 ArUco 확인 → Z-05) → S5 합류·재식별 → S6 추종 이동 → S7 예외 대응 → S8 작업공간 내 추종 → **S9 종료 발행 → S10 순차 복귀·도킹**

### 1.1 범위

- **포함** — 웹캠1 등록·외형 재식별 페어링, 고정 Waypoint 셔틀, 추종·재획득·Standby, 온보드 안전 정지, 2대 관제(뷰어)·교착 해결, **종료 시퀀스 → 즉시 추종 중단 → 순차 복귀·도킹**, KPI 로깅
- **제외 (구현·시험하지 않는다)** — **KOZ(접근 금지 구역, 이관)**, 동일 작업복·실제 사람, 동적 랑데부, 자동 적재, 공구 추적, 장갑 UI, 운영자 정지·재개, costmap 웹캠 융합, 호스·저조도 조건

### 1.2 제약·가정

- 축소 아레나·인형 대체물·복장 2종 조건이므로 결과는 실제 현장 성능의 **상한**이다 (BRD 2.2).
- 보정식·호모그래피·arena.yaml 좌표는 **동일 맵·동일 카메라 위치**에서만 유효하며 변경 시 재보정한다.
- TB4 2대 동시 통신과 **2대의 맵 좌표계 일치는 검증됐다** — 도킹 상태 `amcl_pose` 의 밀림 방향·크기가 두 대에서 5.1 cm 안에서 같고 도크 간격 0.803 vs 줄자 0.840 m. 베이 B ArUco 교차 확인은 TS-U06 때 한 번 더.
- **TurtleBot4 는 후진시키지 않는다 (계약 2절).** Create3 의 **범퍼와 클리프 센서가 전방에만** 있어 후진은 접촉·낙하 감지가 없는 무센서 주행이고, `safety_watchdog` 도 전방 ±35° 만 본다. 무사고 0건(NF-06)이 절대 기준인 프로젝트에서 감지 없는 방향으로 움직이지 않는다. 뒤로 빠져야 하는 상황은 **제자리 회전 뒤 전진**으로 푼다. Nav2 `behavior_plugins` 의 `BackUp` 도 뺀다 (강사 지침).
- 관제는 뷰어이며 운영자 정지·재개 기능은 없다. 정지는 온보드·물리 E-stop 전담.
- **대기공간 도착은 로봇의 자기 보고다.** 웹캠1·2 모두 시야 밖이다. 별도 마커로 보강하지 않는다 (FR-35 폐지) — 로봇이 매 미션 창고 베이에서 마커를 읽어 측위가 그 지점에서 대조되고, 실주행 5회에서 대기공간 정위치가 확인됐다. 대기공간 로봇은 서향(작업공간)으로 서므로 S5 페어링 때 RC카를 로봇 정면으로 돌아 들어오게 조종한다.

### 1.3 용어

BRD 부록 용어를 따른다. 본 문서 추가: **PC-R3/PC-R8**(로봇 제어 PC), **PC-W1/PC-W2**(웹캠 PC), **PC-F**(관제 PC), **Z-01~Z-09**(아레나 구역, BRD 2.3).

---

## 2. System Architecture

PC 5대와 로봇 2대가 PC-F 의 단일 offboard Discovery Server(ID 1, `192.168.108.41:11811`)에 붙는다. 디스커버리 트래픽만 PC-F 를 거치고 **토픽 데이터는 노드 간 P2P** 다. 로봇 온보드는 인지·주행·안전 노드가 로봇별 네임스페이스(`/robot3`, `/robot8`)로 돌고, 웹캠 PC 둘은 등록과 힌트를, 관제 PC 는 미션·상태·기록을 맡는다.

### 2.1 고레벨 아키텍처 다이어그램

```
 Wi-Fi SSID turtle08 / ROS_DOMAIN_ID = 3
 ┌──────────────┬──────────────┬────────────────────────┬───────────────┬───────────────┐
 │ [PC-W1] │ [PC-W2] │ [PC-F] 192.168.108.41 │ [PC-R3] │ [PC-R8] │
 │ Ubuntu 24.04 │ Ubuntu 24.04 │ Ubuntu 24.04 / Jazzy │ Jazzy + GPU │ Jazzy + GPU │
 │ Jazzy │ Jazzy │ ★ Discovery Server │ localization │ localization │
 │ mro_webcam │ mro_webcam │ ID 1 :11811 │ nav2 · RViz │ nav2 · RViz │
 │ webcam_ │ webcam_hint │ mro_fleet + SQLite │ mro_perception│ mro_perception│
 │ register │ (재탐지 힌트)│ Flask control_ui │ mro_navigation│ mro_navigation│
 │ SUPER_CLIENT │ SUPER_CLIENT │ kpi_logger │ ns=/robot3 │ ns=/robot8 │
 │ = False │ = False │ │ │ │
 └──────┬───────┴──────┬───────┴───────────┬────────────┴───────┬───────┴───────┬───────┘
 └──────────────┴──── ROS_DISCOVERY_SERVER=";192.168.108.41:11811" ──────┘
 │
 ┌──────────────────┴──────────────────┐
 [robot3 TurtleBot4] [robot8 TurtleBot4]
 RPi4 / Jazzy RPi4 / Jazzy
 create3 · depthai(OAK-D) · rplidar (동일)
 onboard Discovery ID 3 (유지, 미사용) onboard ID 8 (유지, 미사용)
 Offboard IP → 192.168.108.41 Offboard IP → 192.168.108.41
```

**핵심:** 디스커버리 트래픽만 PC-F 를 경유하고, **토픽 데이터는 노드 간 P2P** 로 흐른다. 로봇 온보드 서버는 남겨두되 PC 쪽 `ROS_DISCOVERY_SERVER` 에 넣지 않는다 — RPi 가 디스커버리 허브를 겸하던 부하 문제(Q-24)가 이 구조로 사라진다.

### 2.2 Nav2 스택

로봇당 Nav2 구성: `bt_navigator`, `planner_server`, `controller_server`, `behavior_server`, `amcl` + `map_server`, global/local costmap, `opennav_docking`, `velocity_smoother`. **keepout costmap_filter 는 KOZ 와 함께 Phase 2 로 이관** 되어 Phase 1 스택에 넣지 않는다.

**우리 노드가 Nav2 에 붙는 지점은 넷뿐이다** — `navigate_to_pose`(action), `dock`/`undock`(action), `amcl_pose`(구독), `cmd_vel`(TwistStamped, estop 시 덮어쓰기). 경로 계획과 지역 제어는 Nav2 에 맡기고 우리는 "어디로, 언제" 만 정한다.

**플러그인 방침**

| 플러그인 | 결정 | 이유 |
|---|---|---|
| `behavior_plugins` 의 **`BackUp` 제거** | **한다** | 계약 v1.13 2절 후진 금지. `Spin`·`Wait` 만 남긴다. 우리 코드에는 `linear.x < 0` 이 없지만 **Nav2 기본 복구가 후진을 낸다** |
| **`RotationShimController`** | **검토한다** | 경로 방향으로 제자리 회전한 뒤 전진한다. '회전 뒤 전진' 이 후진 금지 아래 우리의 유일한 기동 방식이라 성격이 맞는다 |
| `SmacPlanner`(Hybrid-A\*) · `MPPI` 로 교체 | **하지 않는다** | footprint 인식 계획이 좁은 통로에 유리한 것은 맞지만 **P1 실패의 원인은 플래너가 아니라 측위**다. 3일 남은 일정에 튜닝 리스크가 크다. Phase 2 후보 |
| `KeepoutFilter` · 지도 덧칠로 받침대 처리 | **하지 않는다** | 받침대는 이미 통로 바깥으로 뺀 것으로 안다. 설령 남아 있어도 얻는 것이 12 cm 대라 P1 을 풀지 못한다 |

### 2.3 기능 프로세스 흐름 — 전체 플로우차트

**그림:** `docs/diagrams/project_interface_flowchart.svg` — **전체 플로우차트 한 장**으로 통일했다.
편집용 원본은 같은 폴더의 `project_interface_flowchart.drawio` (draw.io 로 연다).
읽는 법 — PC 를 두꺼운 테두리와 옅은 바탕으로 나누고(PC-W1 웹캠1 · PC-W2 웹캠2 · PC-F 관제 ·
PC-R3/PC-R8 로봇 온보드), 그 안에 노드를 **카드**로 놓았다. **카드마다 「입력 → 처리 로직 → 출력」
세 칸**이고, 화살표는 **포트 한 줄에서 포트 한 줄로만** 잇는다 — 어느 토픽이 어느 칸에서 나와
어느 칸으로 들어가는지가 선 하나로 드러난다. 로봇 온보드는 두 대가 같은 구성이라 **한 벌만 그리고
`/robotN` 으로 표기**했다.
세로 축이 정상 흐름 S1~S10, 왼쪽이 그 단계를 수행하는 노드, 오른쪽이 예외와 그때의 처리다. 판정 분기 일곱 개 — 베이 ArUco 일치, P1 토큰 획득(창고에서 나올 때), 재식별 게이트, Target Lost, 전방 장애물, 복귀 토큰(P1 + DOCK), 도킹 성공 — 각각에 도착 판정 실패 · E-3 · E-2 · E-1 · E-4 · E-7 · 도킹 재시도가 붙는다.
---

---

## 3. Component Design

### 3.1 하드웨어 구성

| ID | 대상 | 요구사항 | 근거 |
|---|---|---|---|
| HW-01 | AMR ×2 | TurtleBot4 Standard (Create3 + RPi4, RPLIDAR A1, OAK-D Pro, 범퍼·클리프, 충전 도크). 네임스페이스 `/robot3`, `/robot8` | BRD 2.3 |
| HW-02 | AMR 카메라 | OAK-D RGB 704×704 @10 Hz, aligned depth, subpixel, 압축 전송. 렌즈 높이 **23.5 cm** — RPLIDAR 스캔 높이 18.5 cm 보다 위라 벽(41 cm)이 두 센서 모두를 가린다 | 거리 보정식 유효 조건, 가림 판정 기준, **ArUco 거리·`map_point` 의 초점거리 원천** |
| HW-03 | AMR 적재부 | 시험용 고정 적재물 상시 탑재, 급정지·회전 시 이탈 없는 고정 구조 | BR-T21 |
| HW-04 | 도크 | Create3 도크 ×2, Z-03. 도킹점 간격 **0.84 m** (실측). 좌우 여유가 0.42 m 뿐이라 **로봇별 도크 정면 접근 지점**을 거쳐 도킹한다. IR 차폐판은 두지 않는다 | S2/S10 |
| HW-05 | PC-R3, PC-R8 | Ubuntu 24.04, ROS 2 Jazzy, NVIDIA GPU(OAK-D 영상 YOLO 추론), 로봇 Wi-Fi ping ≤ 10 ms | 온보드 인지는 로봇 제어 PC 에서 수행 |
| HW-06 | PC-W1 / PC-W2 | Ubuntu 24.04, ROS 2 Jazzy, USB 웹캠 1대씩 (640×480, 삼각대). 캡처는 장치 상한(웹캠1 25 fps · 웹캠2 30 fps), YOLO 추론·발행은 10 Hz. 캡처 버퍼를 1 로 두어 묵은 프레임을 버린다 | 웹캠1·2 |
| HW-07 | PC-F | Ubuntu 24.04, ROS 2 Jazzy, SQLite, Flask. GPU 불필요 | 관제 |
| HW-08 | 웹캠 배치 | 웹캠1 — 남쪽 외벽 출입구 옆, **출입구만 본다**. 웹캠2 — 남쪽 외벽 P3 개구부 정면, 삼각대 최대 높이, 작업공간(Z-07)과 Z-09 를 본다 | S1, E-1 |
| HW-09 | 작업자 대체물 | **원통 3단 인형** ×2(몸통 원통 + 어깨판 + 목 + 머리 구) + 원격 비례 조종 RC카 ×2(≤ 0.3 m/s). 회전 대칭이라 어느 각도에서도 실루엣과 bbox 폭이 같다. 전고 worker_a 265 / worker_b 285 mm (실측, RC카 포함) — 20 mm 차이는 사람 간 차이 재현. 작업별 복장 2종은 상의 색으로 구분, 벨크로 탈부착 | BRD 2.3 |
| HW-10 | 아레나 | **MDF 슬롯 조립 벽** (높이 41 cm 실측). 통로 **P1 0.79 m**(병목) / **P2 0.86 m** / **P3 0.93 m** (실측, 받침대 제외 실통과 P2 0.80 · P3 0.865). 물리 병목은 P1 하나 | BRD 2.3, E-3 |
| HW-11 | 마커 | ArUco DICT_4X4_50 10 cm, 베이 A/B/C = **ID 1/2/3**. **선반 뒤 벽면, 높이 20 cm** (광축 23.5 cm 보다 3.5 cm 아래 — 올리면 선반에 가리고, 바닥은 가까워지면 시야 아래로 사라진다). 무광 인쇄, 검은 테두리 바깥 **흰 여백 ≥ 한 칸**. 검출 조건 거리 ≤ 1.5 m · 정면 ±30°. ID 4 는 폐지 | BR-T06 |
| HW-15 | 자재 베이 구조 | 베이는 **우드락 선반**으로 Tool Crib 형태. 선반 높이 **≥ 25 cm** — 로봇 눈높이에 마커, 그 위에 선반. 선반 앞면은 마커 벽에서 0.425 m 앞 | BR-T06, 시연 |
| HW-13 | 가림 수단 | 가림 수단 — ① **교차하는 타 작업자(RC카 2호)** 가 주 수단 ② 기존 벽 모서리 ③ 더미는 웹캠2 각도가 가파른 근거리에만. 추종 경로에 40 cm 벽을 추가하지 않는다 | BRD 2.3, E-1 |
| HW-14 | 바닥 테이프 | 색 구분 마스킹 테이프 — 구역 경계(파랑) · waypoint(초록) · 호모그래피 기준점(십자). 폭 ≥ 3 cm | BRD 2.3, 호모그래피 |
| HW-12 | 비상정지 | 로봇별 물리 정지 수단 (Create3 전원/범퍼 + 운영자 수동 이탈 가능) | NFR-S.6, OR-3.4 |

### 3.2 소프트웨어 구성

**패키지·스택**

| ID | 대상 | 요구사항 |
|---|---|---|
| SW-01 | 공통 | ROS 2 Jazzy, Python 3.12, `rclpy`, `rosidl` 커스텀 인터페이스(`mro_interfaces`), `colcon` 워크스페이스 `mro-multi-amr/ros2_ws` |
| SW-02 | AMR 스택 | TurtleBot4 표준 이미지(`turtlebot4_bringup`, `depthai_ros_driver`), Discovery Server 는 PC-F offboard (NR-01) |
| SW-03 | 주행 | Nav2 + AMCL (`turtlebot4_navigation` localization/nav2 launch), `TurtleBot4Navigator` API(dock/undock/goToPose) |
| SW-04 | 인지 | ultralytics **YOLO26s @ 640** 커스텀 모델 `worker.pt` — 클래스 `amr` / `worker_b` / `worker_a`(코드는 이름으로 거른다). `amr` 은 상대 로봇을 후보에서 빼는 네거티브 클래스. `mro_perception/models/` 가 단일 출처. OpenCV(HSV 히스토그램 · ArUco), `message_filters` RGB-D 동기화, `tf2` |
| SW-05 | 웹캠 | YOLO + ByteTrack, 바닥 호모그래피(json), 재탐지 힌트 발행, KPI CSV 로거. 추론·발행 주기는 파라미터 `process_hz`(기본 10). 시간 상수(히스테리시스·힌트 유효기간)는 초 단위 |
| SW-06 | 관제 | `fleet_manager`(등록·배정·미션·상태 집계·종료 시퀀스 + Flask 관제 화면 모듈), `zone_semaphore`(구역 토큰 `P1`·`DOCK` — 주행 소관), SQLite 스키마(workers / missions / events) |
| SW-07 | 안전 | `safety_watchdog`: `/scan` 만 구독, 전방 ±35° 0.3 m 이내 → `safety/estop` + `cmd_vel` 0 (20 Hz). YOLO·웹캠·Wi-Fi 비의존 |
| SW-08 | 파라미터 | 변경 가능한 값은 `mro_bringup/params/` 로 관리, 코드 하드코딩 금지. 공통 `common.yaml` · 로봇별 `robotN.yaml` · 아레나 `arena.yaml` · 관제 명단 `fleet.yaml`. 로봇 추가는 `robotN.yaml` 1개 + `arena.yaml`·`fleet.yaml` 항목 |
| SW-09 | 저장소 | 담당 단위 패키지, PR 리뷰, `docs/SOT.md`·`interfaces.md` 동기화 |

**노드 (패키지 → 노드)**

| 패키지 | 노드 | PC | 입력 | 출력 |
|---|---|---|---|---|
| mro_perception | amr_detection | PC-R | oakd rgb/depth/camera_info, /webcam1/worker_descriptor | target/map_point, visible, paired, worker_id, image |
| mro_perception | aruco_detector | PC-R | oakd rgb | aruco/id, aruco/distance |
| mro_webcam | webcam_register | PC-W | 웹캠1 | /webcam1/worker_descriptor, image |
| mro_webcam | webcam_hint | PC-W | 웹캠2, worker_descriptor | /webcam2/target_hint |
| mro_webcam | kpi_logger | PC-F | 상태·미션·개입·힌트·FleetStatus | CSV |
| mro_navigation | shuttle_mission | PC-R | mission/assigned, aruco/id, zone_token | undock/goToPose, shuttle/status, shuttle/arrived |
| mro_navigation | follow_target | PC-R | shuttle/arrived, target/*, /webcam2/target_hint, safety/estop | goToPose, cmd_vel(탐색 회전), approach/status |
| mro_navigation | safety_watchdog | PC-R | /scan | safety/estop, cmd_vel(0) |
| mro_navigation | return_dock | PC-R | mission/finish | dock, shuttle/status |
| mro_fleet | fleet_manager | PC-F | worker_descriptor, 로봇 상태·배터리·amcl·estop, 서비스 3종 | /fleet/mission, /robotN/mission/assigned·finish, /fleet/status |
| mro_fleet | zone_semaphore | PC-F | /fleet/zone_token | granted/holder |
| mro_fleet | *(관제 화면 — `fleet_manager` 안의 모듈)* | PC-F | /fleet/status, HTTP | RequestMission, OperatorCommand(RETURN), /fleet/intervention |

관제 화면(Flask)·UI 명령·로봇 상태 집계는 `fleet_manager_node` 안의 모듈(`monitor_web_app` · `fleet_ui_commands` · `robot_status` · `robot_state_manager`)이다. 실행 가능한 노드는 `fleet_manager_node` 와 `zone_semaphore_node` 둘이다.

### 3.3 네트워크 구성

| ID | 요구사항 |
|---|---|
| NR-01 | `ROS_DOMAIN_ID=3` 통일. **Discovery Server 는 PC-F `192.168.108.41` 한 대** (offboard, ID 1). 로봇 2대는 Offboard IP 를 PC-F 로 설정하고, 모든 PC 는 `ROS_DISCOVERY_SERVER=";192.168.108.41:11811"` |
| NR-08 | offboard 구성의 전제 — ① PC-F 의 IP 를 고정한다(DHCP 로 바뀌면 전부 끊긴다) ② PC-F 의 서버를 먼저 띄우고 로봇을 켠다 ③ PC-F 는 단일 실패 지점이므로 시연 중 그 PC 에서 다른 작업을 하지 않는다 |
| NR-02 | nav2 launch 터미널은 `ROS_SUPER_CLIENT=False`, 그 외 노드·CLI 는 `True` |
| NR-03 | 카메라 스트림은 압축 전송(JPEG, compressedDepth), **로봇당 구독자 1개**(검출 노드). 모니터링은 `target/image` 재발행 구독 |
| NR-04 | 로봇–PC 시계 차 ≤ 0.1 s (TF 조회). 초과 시 최신 TF 폴백 |
| NR-05 | Wi-Fi 단절 시 AMR 은 추종·회피·정지를 온보드로 지속하고 신규 미션 배정만 중단 (E-5) |
| NR-06 | PC 사이를 건너는 정보는 토픽·서비스뿐. PC 간 직접 소켓 통신 없음 |

**주기 정책** — 각 카메라의 주기는 하는 일의 시간 상수로 정한다. 맞춰야 하는 것은 주기가 아니라 시각이다(모든 메시지·로그가 ROS 시각).

| 대상 | 캡처 | 추론·발행 | 그 값인 이유 |
|---|---|---|---|
| AMR OAK-D | 10 Hz (장치 프로파일) | 10 Hz | 추종 제어 루프의 입력. v_max 0.3 m/s 에서 프레임당 3 cm 이동이라 standoff 0.5 m 제어에 충분하다. 더 올리면 RPi 대역폭과 압축 전송이 먼저 한계에 닿는다 (NR-03, 3.3) |
| 웹캠1 (등록) | 장치 상한 (Jieli 25 fps) | **10 Hz** | 사람이 걸어와 서는 이벤트라 시간 상수가 초 단위다. 등록 완료 기준이 ≤ 10 s 이므로 10 Hz 로 충분하다 |
| 웹캠2 (힌트) | 장치 상한 (30 fps `[실측 필요]`) | **10 Hz** | 힌트 유효기간 2 s 안에 20프레임. 재획득 병목은 힌트 주기가 아니라 Lost 판정 1.0 s (FR-16) 다 |

---

## 4. Data Flow Design

### 4.1 수집

- **로봇 온보드** — OAK-D 컬러·depth·camera_info(10 Hz, 압축 전송), RPLIDAR `/scan`, Create3 `battery_state`·`dock_status`·`odom`. 로봇당 카메라 구독자는 검출 노드 하나뿐이다 (NR-03, 3.3)
- **웹캠1** — 출입구 영상(캡처 25 fps · 추론 10 Hz) → 작업자 등록 서술자
- **웹캠2** — 작업공간 영상(캡처 30 fps · 추론 10 Hz) → 재탐지 방위 힌트, KPI 지상 진실

### 4.2 처리

인지(검출·재식별·ArUco) → 주행(셔틀·추종·안전·복귀) → 관제(배정·미션·집계)로 흐른다. 노드 간 토픽·서비스·QoS 는 **2.3 의 전체 플로우차트**가 카드의 입력·출력 칸에 담고 있고, 계약 원문은 `docs/interfaces.md` 다. 정상 흐름 S1~S10 의 단계별 트리거·동작·성공 조건은 BRD 2.4 를 따른다.

### 4.3 저장

| 데이터 | 저장소 | 보존 | 근거 |
|---|---|---|---|
| workers / missions / events | PC-F SQLite (`~/mro_fleet.db`) | 프로젝트 기간 | FR-03, FR-28 |
| KPI CSV (상태·힌트·개입·위치) | PC-F `~/mro_kpi/` → Google Drive | 프로젝트 기간 | FR-28, 29 |
| 웹캠·로봇 영상 원본 | 세션 판정용 임시 | **판정 후 폐기** | NF-08 |
| 모델·맵·호모그래피 | 저장소 | 버전 관리 | SW-08 |

### 4.4 인터페이스·알림

| ID | 요구사항 |
|---|---|
| IR-01 | 노드 간 계약은 `mro_interfaces` (msg 5: WorkerDescriptor, Mission, FleetStatus, TargetHint, Intervention / srv 3: RequestMission, ZoneToken, OperatorCommand) 와 `docs/interfaces.md` 토픽 표를 따른다 |
| IR-02 | 로봇 노드는 상대 토픽 이름을 쓰고 launch 가 `namespace:=/robotN` 과 TF 리맵을 붙인다. 전역(웹캠·관제) 토픽은 `/webcam1/*`, `/webcam2/*`, `/fleet/*` |
| IR-03 | QoS: 명령·상태·미션·서비스 RELIABLE, 카메라·scan·힌트 BEST_EFFORT |
| IR-04 | TB4 기본 토픽(`oakd/*`, `scan`, `battery_state`, `amcl_pose`, `cmd_vel`(TwistStamped)) 과 Nav2 액션(`navigate_to_pose`, dock/undock)은 변경하지 않는다 |
| IR-05 | 관제 UI 는 HTTP(Flask, :5000). 외부 시스템(ERP/MRO) 연동 없음 |
| IR-06 | 서술자 히스토그램 bin(16×8), 재식별 임계, 구역 폴리곤은 파라미터로 노출한다 |

관제 화면은 `FleetStatus`(1 Hz)를 받아 로봇별 상태·배터리·위치·짝지은 작업자를 표시하고, 재식별 게이트 30 s 미달(`GATE_FAIL`)은 명시 페어링 버튼으로 운영자에게 알린다.

---

## 5. Detailed Design

각 절은 **요구사항 → 아키텍처(모듈) → 데이터 흐름 → 파라미터 → 실패 시 동작** 순이다. 값의 단일 출처는 `mro_bringup/params/` 와 각 패키지 `config/` 이며, 본문의 숫자는 그 파일을 옮긴 것이다.

### 5.1 인지 (Perception) — FR-11~14, 24

#### 5.1.1 요구사항

| FR | 요구 |
|---|---|
| FR-11 | RGB-D 동기 쌍에서 작업자를 검출하고 거리를 재서 map 좌표를 발행한다 |
| FR-12 | 웹캠1 서술자와 대조하는 재식별 게이트를 통과해야 「내 담당」이다 |
| FR-13 | 게이트 미달 30 s 지속 시 명시 페어링 폴백 창구를 연다 |
| FR-14 | 명시 페어링은 1 m 안에서만 받는다 |
| FR-24 | ArUco 마커로 베이 도착 근거가 되는 값을 만든다 (FR-08 의 원천) |

**역할 경계.** 인지는 **값을 만들 뿐 판정하지 않는다** (계약 9.1). 「도착했다」·「측위가 맞다」는 판정은
소비자인 주행과 관제가 각자 건다. 인지는 안전 경로에 없다 — 카메라가 죽어도 `safety_watchdog` 는 돈다.

#### 5.1.2 아키텍처 (모듈)

| 모듈 | 파일 | 역할 |
|---|---|---|
| `amr_detection` | `nodes/amr_detection_node.py` | 작업자 검출 · 재식별 게이트 · 거리 · map 좌표 |
| `aruco_detector` | `nodes/aruco_detector_node.py` | 마커 번호 · 거리 · 자세 |
| 재식별 | `core/reid.py` | 색 지문 · 유사도 · 게이트 상태기계 · 명시 페어링 판정 |
| 카메라 | `core/camera.py` | `camera_info` 초점거리 보정 |
| 마커 계산 | `core/aruco.py` | 검출 · solvePnP · 자세 |
| 거리 | `core/depth_distance.py` | 압축 depth 해제 · ROI 거리 |

두 노드 모두 **로봇 제어 PC** 에서 로봇별 네임스페이스로 돈다. 시작 신호는 없다 — 켜지면 곧바로
검출·발행하고, 결과를 언제부터 쓸지는 소비자가 정한다.

#### 5.1.3 작업자 검출과 거리 (FR-11)

```
oakd/rgb/image_raw/compressed ──┐
oakd/stereo/image_raw/compressedDepth ──┼─ 시각 맞춤(허용 시차 0.1 s) ─▶ 처리
oakd/stereo/camera_info ────────┘
```

처리 순서 — ① YOLO 로 작업자 후보를 전부 찾는다 ② 후보마다 색 지문을 뜬다 ③ 게이트로 「내 담당」을
고른다 ④ `target/visible` 발행 ⑤ bbox **안쪽 50 %** 의 depth 로 거리를 낸다 ⑥ 거리 품질 게이트
⑦ 픽셀+거리 → 카메라 3차원 점 → **직선거리**를 `target/distance` 로 발행 ⑧ TF 로 map 좌표를 만들어
`target/map_point` 발행.

**bbox 가장자리를 쓰지 않는 이유** — 가장자리에는 배경 depth 가 섞인다. 안쪽 50 % 네모에서 하위 20 %
값을 그 사람까지의 거리로 쓴다.

**거리 품질 게이트** — ROI 핵심부(하위 20 %~중앙값)가 150 mm 이상 벌어지면 배경이 섞인 것으로 보고
**map 좌표를 버린다.** `target/visible` 은 품질과 무관하게 매 프레임 내고, `map_point`·`distance` 는
통과분만 낸다 — 앞의 것은 Lost 판정(K-04)에, 뒤의 것은 추종 목표와 추종거리(AC-05) 채점에 쓰이기 때문이다.

**초점거리 보정.** OAK-D 는 704×704 로 자른 영상을 내면서 **자르기 전 16:9 폭 기준의 `fx`** 를
실어 보낸다. 그대로 쓰면 픽셀↔3차원 환산이 틀어진다. 보정 배율 `camera_fx_scale` 은 확정했다 —
체커보드로 두 대 모두 `fx ≈ 1038` 이 나왔지만 같은 사진을 다시 풀면 배율이 13~28 % 폭으로 흔들려
**줄자로 판정했다** (robot3 0.575~1.455 m 전 구간 1.7 cm 안 · robot8 0.98~1.53 m 에서 +0.5~+3.2 cm).
운용 구간(0.9~1.5 m)에서 두 대 모두 판정 기준 10 cm 의 1/3 안이라 **로봇 공용**으로 쓴다.
주점(cx·cy)은 곱하지 않는다 — 로봇이 싣고 오는 값이 참값이다.

**영상은 압축으로만 낸다.** 원본 영상은 2.97 MB × 9 Hz = 214 Mbit/s 라 Wi-Fi 를 막는다.

#### 5.1.4 재식별 게이트 (FR-12)

**색 지문**은 bbox 안의 색 분포표(H16×S8 = 128차원)다. **웹캠1 과 반드시 같은 함수를 쓴다** — 다른
함수를 쓰면 두 값이 같은 자가 아니라 비교 자체가 성립하지 않는다 (계약 2절). 한 장의 잡음을 씻기 위해
최근 10프레임 평균을 쓰고, 평균을 낸 뒤 다시 정규화해 웹캠1 이 발행하는 척도와 맞춘다.

**판정** — 복장 클래스가 같고 유사도가 임계 이상이면 그 프레임은 통과다. 프레임 판정을 그대로 쓰지 않고
**연속 시간**으로 켜고 끈다: `pair_on_s`(1.0 s) 연속 통과면 `paired=True`, `pair_off_s`(3.0 s) 연속
실패·미검출이면 `False`.

**임계값 (Q-13) —.** `reid_threshold` 를 **0.34** 로 정했다. 실측 유사도 분포로 결정했다
(계약 2절 「눈대중 금지」). 종전 잠정값 0.6 은 **자동 페어링이 걸릴 수 없는 값**이었다 — 서술자 추출
영역을 인형 전체로 넓히면서(계약 v1.3) 하의·머리가 두 인형에서 같아 유사도 범위가 위로 압축됐기 때문이다.

**「대상」이 누구인가**
- 서술자·미션이 아직 없으면(S1~S3 전) — 보이는 작업자 중 확신도 최고. `paired` 는 거짓 (단독 시험용)
- 서술자·미션이 들어오면(S5 이후) — **내 담당 복장**의 후보 중 색 지문이 가장 닮은 것. 남의 담당은 대상이 아니다

#### 5.1.5 명시 페어링 폴백 (FR-13·14)

게이트를 못 넘어 로봇이 영영 서 있지 않게 운영자가 짝을 정해 주는 길이다.

| 단계 | 내용 |
|---|---|
| 손 들기 | 작업자가 **보이는데** 게이트 미달이 30 s 이어지면 `target/pair_state = GATE_FAIL` → 관제 화면에 버튼 |
| 창구 | 서비스 요청 (로봇 · 작업자 번호) |
| 검사 | 남의 로봇 · 이미 짝 있음 · 안 보임 · **1 m 밖** · 그새 스스로 통과 → 거부 + 사유 |
| 확정 | 수락하면 **이 노드가** `paired`·`worker_id` 를 낸다 — 발행자는 하나뿐이어야 로그로 가릴 수 있다 |
| 해제 | `mission/finish` 가 오면 잠금과 게이트를 함께 푼다 |

**사람이 안 보이는 동안(`NO_TARGET`)은 30 s 를 세지 않는다** — 그건 못 알아보는 것이 아니라 없는 것이다.

**발동률에 상한을 두는 이유** — 상한이 없으면 게이트를 극단적으로 빡빡하게 잡아 매번 폴백으로 넘기는
「판단하지 않는 시스템」이 만점을 받는다. 그래서 폴백 발동을 **개입으로 세어 IPH 에 넣는다.**

#### 5.1.6 ArUco 검출 (FR-24 → FR-08)

**같은 두 토픽을 두 곳이 다르게 쓴다.**

| 번호 | 어디 | 무엇 | 쓰는 쪽 |
|---|---|---|---|
| 1 · 2 · 3 | 창고 베이 A/B/C 선반 뒤 벽 (높이 20 cm) | 베이 **도착 확정** — 번호 일치 AND 거리 ≤ 1.5 m | 주행 `shuttle_mission` (FR-08) |

**허용 번호는 1·2·3 뿐이다.** 대기공간 측위 검증 마커(옛 ID 4)는 **폐지했다** —
실주행 5회에서 그것 없이도 대기공간 정위치가 확인됐고, 로봇이 **매 미션 창고 베이에서 마커를 읽으므로**
AMCL 발산은 그 지점에서 드러난다. 대기공간에 마커를 하나 더 두는 것은 같은 확인의 중복이었다.

**거리는 마커 네 꼭짓점만으로 낸다.** 원리는 한 줄이다 —
`거리 = fx × 마커 한 변 ÷ 화면에서의 마커 픽셀 크기`.
`fx` 와 마커 크기가 곱으로만 들어가므로 **`fx` 가 틀리면 거리가 그 비율 그대로 틀린다.**

`fx` 는 `camera_info` 값이 704×704 crop 에 맞지 않아 **체커보드와 줄자로 실측한 배율**(`camera_fx_scale`)을 곱해 쓴다. 줄자 대조는 전 구간 1.6 cm 안이다. 스테레오 depth 는 마커 거리에 쓰지 않고, 마커가 없는 인형 추종(5.1.3)에만 쓴다.

**발행 한계와 수용 조건을 나눈다.** 여기서는 2.0 m 를 넘으면 「못 봤다」(번호 −1)로 튼다. 계약 6절의
수용 조건 1.5 m 는 **소비자가 건다** — 발행을 1.5 로 자르면 정확히 1.5 m 에 둔 마커가 1.52 로 걸러진다
. 자세를 못 푼 프레임도 「못 봤다」로 낸다 — 유효한 번호와 NaN 거리를 함께 내보내면
관제의 비교가 조용히 깨진다.

**「정면 ±30°」는 기계로 검사하지 않는다.** 마커 자세의 기운각이 정사각 마커의 두 해 문제로 거리가 멀수록
흔들려(1.0 m 에서 −30° 가 33°, 2.0 m 에서 20°) 게이트로 쓸 수 없다. **시험 절차에서 사람이 지킨다.**

**마커에 흰 여백이 없으면 보정값이 맞아도 거리가 틀린다**. 검은 테두리가 벽
경계선과 붙으면 꼭짓점이 바깥으로 끌려가 사각형이 커지고 거리가 가깝게 나오는데, 재투영 오차는 깨끗해서
드러나지 않는다. 계약 6절이 요구하는 여백(한 칸 이상 ≈ 2 cm)을 반드시 둔다.

**마커 자세(`aruco/pose`)** 는 주행의 베이 정렬용으로 선택 발행한다. 좌우 치우침과 방위는 쓰고
**회전(yaw)은 정렬에 쓰지 않는다** — 위와 같은 두 해 문제 때문이다.

#### 5.1.7 파라미터 (단일 출처: `mro_perception/config/perception.yaml`)

| 값 | 현재 | 근거 |
|---|---|---|
| `camera_fx_scale` | 1.8336 | 줄자 확정. 체커보드는 출발점일 뿐 |
| `detect_conf` | 0.5 | 학습 스윕의 F1 최적은 0.4 지만 올려 쓴다 |
| `reid_threshold` | **0.34** | **Q-13 확정** — 실측 유사도 분포 |
| `descriptor_frames` | 10 | 색 지문 평균 창. 웹캠1 과 같아야 같은 척도 |
| `pair_on_s` / `pair_off_s` | 1.0 s / 3.0 s | 연속 통과·실패 시간. **`pair_off_s` 는 `[팀 확정 필요]`** |
| `gate_fail_s` | 30 s | FR-13 폴백 발동 |
| `manual_pair_max_m` | 1.0 m | FR-14 — 멀면 좌표 오차가 커서 수동도 소용없다 |
| `max_core_gap_mm` | 150 mm | 거리 품질 게이트 |
| `bbox_inner` | 0.5 | 거리 측정에 쓰는 bbox 중심부 비율 |
| `sync_slop_s` | 0.1 s | RGB·depth 시각 맞춤 허용 시차 |
| `marker_size_m` | 0.10 m | 인쇄된 검은 사각형 한 변 실측 |
| `marker_ids` | 1, 2, 3 | 베이 A/B/C. **ID 4 는 폐지** |
| `max_distance_m` | 2.0 m | **발행** 한계. 수용 조건 1.5 m 는 소비자가 건다 |

#### 5.1.8 실패 시 동작

| 상황 | 동작 |
|---|---|
| `camera_info` 미수신 | 그 프레임은 버린다 (픽셀↔3차원 환산 불가) |
| depth 짝을 못 찾음 | `visible` 은 내고 `map_point`·`distance` 는 내지 않는다 |
| ROI 품질 게이트 미달 | map 좌표 폐기 — 배경이 섞인 거리로 목표를 잡지 않는다 |
| 마커 미검출·자세 풀이 실패·2.0 m 초과 | 번호 −1 로 발행. 거리·자세는 내지 않는다 |
| 게이트 30 s 미달 | `GATE_FAIL` 발행 → 관제 폴백 버튼 |
| 명시 페어링 요청이 조건 미달 | 거부 + 사유 (남의 로봇 / 이미 짝 / 안 보임 / 1 m 밖) |
| TF 미준비 | map 좌표를 내지 않는다 — 잘못된 좌표보다 없는 편이 낫다 |

---

### 5.2 웹캠 (Webcam) — FR-01, 02, 17, 29

#### 5.2.1 요구사항

| FR | 요구 |
|---|---|
| FR-01 | 웹캠1 은 출입구를 지나는 작업자를 검출하고 트랙 번호를 준다 |
| FR-02 | 트랙별 복장 클래스와 색 지문을 `WorkerDescriptor` 로 발행한다 |
| FR-17 | 웹캠2 는 재탐지용 방위 힌트를 낸다 (쓰는 쪽은 주행 5.3.4) |
| FR-29 | 추종거리·미션·개입·구역 사건을 ROS 시각 기준 CSV 로 남긴다 |

**역할 경계.** **웹캠은 안전 판단에 관여하지 않는다.** 웹캠1 은 등록만, 웹캠2 는 힌트와 계측만 한다.
둘 다 죽어도 로봇의 정지·추종은 그대로 돈다.

#### 5.2.2 아키텍처 (모듈)

| 모듈 | 파일 | PC | 역할 |
|---|---|---|---|
| `webcam_register` | `nodes/webcam_register_node.py` | PC-W1 | 출입구 작업자 등록 (S1) |
| `webcam_hint` | `nodes/webcam_hint_node.py` | PC-W2 | 재탐지 힌트 + 압축 영상 |
| `kpi_logger` | `nodes/kpi_logger_node.py` | **PC-F** | KPI CSV |
| 좌표 변환 | `core/homography.py` | PC-W2 | 픽셀 → map 좌표 |

`kpi_logger` 가 웹캠 담당이면서 관제 PC 에서 도는 이유 — fleet 토픽을 가장 많이 구독하기 때문이다.

#### 5.2.3 웹캠1 — 작업자 등록 (FR-01·02)

```
카메라 ─▶ YOLO 추적(ByteTrack) ─▶ 트랙 번호 ─▶ bbox 잘라내기
 ─▶ 색 지문 ─▶ W1/W2 배정 ─▶ /webcam1/worker_descriptor (1 Hz 재발행)
```

**시야는 출입구만 본다**. 관제탑 구역 전경이 아니다 — 그 거리에서는 인형 bbox 가
작아 색 히스토그램이 뭉개진다. 출입구에서는 실측 3,640 px² 가 유지되고 배경 오검출이 줄며, 등록 시점이
앞당겨져 언독(S2)이 빨라진다. **등록 구역 게이트는 두지 않는다 — 시야 전체가 곧 등록 구역이다.**

**히스테리시스는 프레임 수가 아니라 시간으로 정의한다** (계약 1절). 0.3 s 연속 검출이면 있음,
0.5 s 연속 미검출이면 없음, 트랙이 1.0 s 안정되면 작업자 번호를 배정한다. 프레임 수로 두면 처리 주기가
바뀔 때 뜻이 조용히 달라진다.

**발행하는 서술자**

1. 원래는 최근 10프레임 **이동평균**을 계속 재발행했다. 그러면 등록 뒤 사람이 움직이거나 조명이 바뀔 때
 서술자가 따라 흔들려 AMR 쪽 재식별 기준이 계속 변한다.
2. 1차 수정 — **등록 순간의 평균으로 고정**했다. 이동평균은 재등록 판정에만 쓴다.
3. 2차 수정 — 지금은 **미리 구운 인형별 기준 지문**(`models/worker_hist.json`, 데이터셋 평균)을 먼저 쓴다.
 인형은 둘뿐이고 생김새가 고정이라 등록 때 본 한 장면보다 데이터셋 평균이 낫다. 기준 지문을 못 읽으면
 등록 순간 고정본으로 떨어진다.

**색 지문 함수는 인지와 같은 것을 쓴다** (계약 2절). 다른 함수를 쓰면 비교 자체가 성립하지 않는다.

**트랙이 끊겼다 다시 잡히면** 기존 등록자와 유사도를 비교해 같은 사람이면 **등록 슬롯을 재사용**한다
(`rematch_threshold` 0.45). 안 그러면 한 사람이 두 번 등록돼 로봇이 두 대 배정된다.

**캡처 fps 와 처리 Hz 는 다른 값이다.** 캡처가 처리보다 빠르면 큐에 오래된 프레임이 쌓이므로 버퍼를 1로 둔다
. 자동 노출이 길어지면 캡처가 처리 주기 밑으로 떨어져 루프까지 느려진다.

**웹캠1 은 map 좌표를 내지 않는다.** 그래서 호모그래피가 필요 없다.

#### 5.2.4 웹캠2 — 재탐지 힌트 (FR-17)

```
카메라 ─▶ YOLO 추적 ─▶ bbox 하단 중앙 픽셀 ─▶ 호모그래피 ─▶ map 좌표
 ─▶ /webcam2/target_hint (작업자 번호 포함)
 ─▶ /webcam2/image_raw/compressed (오버레이, 10 Hz)
```

**bbox 하단 중앙을 쓰는 이유** — 그 점이 바닥과 닿는 지점이라 평면 호모그래피가 성립한다.

**호모그래피는 로봇 AMCL 로 잰 기준점 13개로 캘리브레이션했다**. 재투영 RMSE 2.8 cm,
leave-one-out 평균 4.0 cm · 최대 8.1 cm 로 판정 기준 0.3 m 의 1/3 안이다.
**Nav2 클릭 주행의 도착 위치를 자로 쓰지 않는다** — Nav2 는 허용오차 0.25 m 안이면 어디서든 멈추므로
그 방식으로는 25 cm 이하 오차를 판정할 수 없다. 검증은 held-out 지점에 로봇을 세우고
`amcl_pose` 와 픽셀 변환값을 직접 대조한다.

**힌트는 방향을 정하는 데만 쓴다.** 주행은 힌트 방위로 회전 방향만 정하고, 목표 좌표는 로봇 자기 카메라가
낸 값을 쓴다. 그래서 호모그래피 오차가 추종 정확도에 직접 들어가지 않는다. 힌트에 **작업자 번호를 실어
보내** 주행이 남의 담당 힌트를 거를 수 있게 한다 — 토픽이 전역이라 이 검사가 없으면 2대 운용에서
상대 로봇의 작업자 힌트로 회전 방향을 정하게 된다.

**압축 영상은 오버레이로 통일한다**. 오버레이본과 원본이 섞여 나가면 받는 쪽이 무엇을 보는지 모른다.
발행 주기는 YOLO 처리 주기와 독립이다.

#### 5.2.5 KPI 기록 (FR-29)

한 CSV 파일에 `t_ros, source, robot, key, value, x, y, extra` 로 남긴다. 후처리는 `tools/kpi_report.py` 다.

**규칙 셋** — 어기면 해당 항목이 통째로 빈다.
1. **모든 시각은 ROS 시각.** 벽시계를 섞지 않는다 — 로봇·웹캠·관제 로그를 합쳐야 하기 때문이다.
2. **추종거리 판정의 기준값은 검출 노드의 `target/distance` 다** (FR-29). 좌표로 역산하지 않는다.
3. **QoS 를 발행자와 맞춘다.** 명령·상태·미션은 RELIABLE, 카메라·스캔·힌트는 BEST_EFFORT.

**로봇 명단의 단일 출처는 `fleet.yaml`** 이다. 명단에 없는 로봇은 KPI 에 한 줄도 안 남는다.

**배송 판정(AC-08)과 리드타임(K-05)은 CSV 가 아니라 관제 DB 에서 읽는다**.
CSV 만 보면 ① T_lim 을 넘겨 도착한 미션도 성공으로 세고 ② 끝내 도착하지 못한 미션은 흔적이 없어
분모에서 빠진다 — 둘 다 **실제보다 좋게** 나오는 방향이라 위험하다. 대조에서 리포트 50.0 %, 참값 0.0 % 가 나왔다.

**영상 원본은 세션 판정 후 폐기한다** (NF-08 · R-10).

#### 5.2.6 파라미터 (단일 출처: `mro_webcam/config/webcam.yaml`)

| 값 | 현재 | 근거 |
|---|---|---|
| `process_hz` (양쪽) | 10 Hz | 계약값. 추론·발행 주기 |
| `capture_fps` (웹캠1) | 25 | 장치 상한 실측. 계약 v1.2 가 「장치 상한 그대로」로 완화 |
| `on_time_s` / `off_time_s` | 0.3 s / 0.5 s | 계약값 히스테리시스 |
| `register_s` | 1.0 s | 트랙 안정 → 작업자 번호 배정 (TS-U01 실측 0.90 s 통과) |
| `descriptor_frames` | 10 | 인지 `descriptor_frames` 와 같아야 같은 척도 |
| `rematch_threshold` | 0.45 | 끊긴 트랙의 동일인 판정 |
| `max_workers` | 2 | SOT 확정 |
| `conf` / `confidence` | 0.5 | 인지 `detect_conf` 와 같은 값 |
| `detection_confirm_s` / `_hold_s` / `lost_confirm_s` (웹캠2) | 0.3 / 0.5 / 1.0 s | 트랙 확정·연속성·Lost |
| `jpeg_quality` | 80 | 인지 `target/image` 와 같은 값 |
| `out_dir` (KPI) | `~/mro_kpi` | 세션 종료 후 Google Drive |

#### 5.2.7 실패 시 동작

| 상황 | 동작 |
|---|---|
| 기준 지문 파일을 못 읽음 | 경고 후 **등록 순간 고정본**으로 발행 (동작은 계속) |
| 트랙 끊김 | 유사도로 동일인 판정 → 같으면 슬롯 재사용, 다르면 새 번호 |
| 작업자 3명 이상 | 상한 2명까지만 등록 |
| 호모그래피 파일 없음 | 힌트를 내지 않는다 — 틀린 좌표보다 없는 편이 낫다 |
| 웹캠2 힌트 끊김 | 주행이 유효기간 2 s 로 걸러 옛 힌트를 쓰지 않는다. 힌트 없는 재탐색으로 돌아간다 |
| KPI 구독 QoS 불일치 | 그 항목이 통째로 빈다 — **발행자와 반드시 맞춘다** |
| 자동 노출로 캡처 지연 | 처리 주기 밑으로 떨어지면 루프가 함께 느려진다 (알려진 한계) |

---

### 5.3 주행 (Navigation) — FR-04, 07~10, 15~22, 25, 30~32

#### 5.3.1 요구사항

| FR | 요구 |
|---|---|
| FR-04 | 도크 기준 초기자세 → Nav2 준비 확인 → undock → `READY` |
| FR-07 | `mission/assigned` 수신 → 베이 waypoint 이동 (도크~창고는 토큰 없음) |
| FR-08 | `aruco/id == 요청 베이` 로 도착 확정, 5 s 체류 후 대기공간 이동 |
| FR-09 | 요청 접수 → Z-05 도착 리드타임, T_lim 초과 시 미션 폐기 (이동은 계속) |
| FR-10 | 구역 토큰 FIFO, 보유 60 s 초과 시 강제 회수 |
| FR-15 | standoff 0.5 m 유지, 대상이 충분히 움직였을 때만 재목표 |
| FR-16 | 미검출 1.0 s → Lost → 1 s 안에 감속·정지 |
| FR-17 | Lost 후 회전 방향을 웹캠2 힌트 방위로 결정 |
| FR-18 | 재획득 시 재식별 게이트를 다시 통과해야 재개 |
| FR-19 | Lost 30 s → STANDBY |
| FR-20 | 작업공간 내 추종 지속 |
| FR-21 | `/scan` 전방 ±35° 0.3 m 2스캔 연속 → `safety/estop` |
| FR-22 | 모든 주행 노드는 estop 중 goal 취소·병진 명령 금지 |
| FR-25 | v_max 0.3 m/s 공통 |
| FR-30 | `mission/finish` → 즉시 페어링 해제·추종 중단 |
| FR-31 | 복귀 시 P1·DOCK 토큰 |
| FR-32 | 도크 정면 접근 지점 경유 후 도킹, 실패 시 최대 2회 재시도 |

**설계 원칙 (BRD 5장 안전 계층).** 정지 판단은 로봇 온보드에서만 한다. 관제·웹캠·인지가 전부 죽어도
`safety_watchdog` 는 `/scan` 만으로 계속 돈다. **후진은 어느 노드도 쓰지 않는다** (계약 2절 — Create 3 의
범퍼·낭떠러지 감지가 전방에만 있어 후진은 감지 없는 주행이다).

#### 5.3.2 아키텍처 (모듈)

| 모듈 | 파일 | 역할 |
|---|---|---|
| `shuttle_mission` | `nodes/shuttle_mission_node.py` | S4 셔틀 상태기계 (READY→SHUTTLE→AT_BAY→WAITING) |
| `follow_target` | `nodes/follow_target_node.py` | S5~S8 추종 상태기계 (SEARCHING→APPROACH_1/2→ARRIVED→FOLLOWING↔STANDBY) |
| `safety_watchdog` | `nodes/safety_watchdog_node.py` | LiDAR 독립 정지 + **최종 `cmd_vel` 단일 중재자** |
| `return_dock` | `nodes/return_dock_node.py` | S9·S10 복귀 상태기계 (RETURNING→DOCK_WAIT→DOCKING→DOCKED) |
| `zone_semaphore` | `mro_fleet/nodes/zone_semaphore_node.py` | 구역 토큰 서버 (패키지는 `mro_fleet`, 설계·담당은 주행) |
| 안전 상태기계 | `core/safety.py` | 시각·센서를 호출자가 넣어 주는 순수 상태기계 (`SafetyController`) |
| 정지 확인 | `nodes/motion_pause.py` | 주행 노드가 "내 액션은 멈췄다" 를 워치독에 알리는 공통 부품 |

**노드 4개가 한 로봇 제어 PC 에서 같이 돈다.** 기동 순서는 `shuttle_mission` → Nav2 다 —
Nav2 의 `global_costmap` 이 기동할 때 map→base_link 변환을 요구하는데 그것은 초기자세가 있어야 생기고,
활성화가 한 번 실패하면 나중에 변환이 생겨도 **재시도하지 않는다**.

#### 5.3.3 셔틀 미션 (FR-04·07·08·09)

**상태** — `shuttle/status` 로 `READY / SHUTTLE / AT_BAY / WAITING / FAILED` 를 낸다.

| 단계 | 하는 일 | 나가는 상태 |
|---|---|---|
| 기동 | 도크 좌표로 초기자세 발행 → Nav2 준비 대기 | `READY` |
| 배정 수신 | `mission/assigned` → undock | `SHUTTLE` |
| 베이 이동 | `arena.yaml` 의 베이 waypoint 로 `goToPose` | `SHUTTLE` |
| 도착 확정 | `aruco/id` 가 배정 베이와 같고 거리 ≤ 1.5 m | `AT_BAY` |
| 접근·정렬 | 마커에서 `bay_marker_range_m`(0.9 m) 인 자리로 붙고 측량 베이 방위로 선다 | `AT_BAY` |
| 체류 | 적재 시뮬레이션 5 s | `AT_BAY` |
| P1 토큰 | 베이를 떠나기 전 획득, 못 얻으면 그 자리에서 2 s 뒤 재요청 | `AT_BAY` 유지 |
| 대기공간 | 이동 후 P1 반납 | `WAITING` (한 번만) |

**판정과 직결되는 규칙 넷.** 어기면 KPI 가 조용히 틀어진다.

1. **`WAITING` 은 대기공간 도착이 확정된 시점에 한 번만.** 관제가 이것으로 AC-08 성공/실패와 K-05
 리드타임을 확정한다. 베이에서 토큰을 기다리는 동안 `WAITING` 을 내면 창고에 있는 로봇이 도착으로
 기록돼 리드타임이 짧게 찍힌다.
2. **`FAILED` 는 포기할 때만.** 재시도 중에는 직전 상태를 유지한다.
3. **T_lim 초과 미션은 폐기한다** — 그 뒤 늦게 도착해도 `WAITING`·`shuttle/arrived` 를 내지 않는다
 (계약 7절 이중 계상 금지). 늦은 도착은 관제가 `arrived_late` 로 따로 남긴다.
4. **T_lim 시계의 기준점은 요청 접수 시각이다.** Nav2 를 기다린 시간도 포함한다 — 뒤로 옮기면
 인프라 대기가 리드타임에서 빠져 AC-08 이 실제보다 좋게 나온다.

**베이 접근을 마커 기준 절대 위치로 바꾼 이유.** 옛 절차는 waypoint 도착 뒤 「그 자리에서 앞으로 0.3 m」
였는데, 전진은 **출발점을 모르는 상대량**이라 AMCL 이 0.4 m 틀리면 최종 정차도 그대로 0.4 m 틀렸다
(실측 0.38~0.53 m). 지금은 목적지가 마커 기준 절대 위치라 출발점이 어디든 도착점이 같다 —
`aruco/pose` 가 카메라→마커의 상대 측정이라 AMCL 오차와 무관하기 때문이다.

목표 거리 0.9 m 는 **추정이 아니라 실측에서 나왔다**. 접근 중 마커까지 0.715 m 인 순간 워치독이
전방 0.29 m 로 정지를 걸었다 — 빼면 **선반 앞면이 마커 벽에서 0.425 m 앞**이다. 여기서 바닥이 나온다:
워치독이 안 울리려면 `0.425 + 해제거리 0.38 = 0.805 m`, Nav2 충돌 예측을 피하려면 `0.425 + 0.379 = 0.804 m`.
**0.80 m 가 하드 바닥**이고 0.5·0.6·0.7 은 전부 그 아래라 실패했다(옆벽 충돌 / 정지 / 빠져나오지 못함).

**Nav2 목표로는 이 보정을 못 한다.** `SimpleGoalChecker` 가 xy 0.25 m · yaw 0.25 rad 라 그보다 작은
이동을 보내면 로봇이 움직이지 않고 곧장 성공이 난다. 그래서 보정은 `behavior_server` 의 Spin·
DriveOnHeading(상대 이동, 허용오차 없음)으로 실행하고, 차동구동이라 옆으로 못 가므로 **회전–직진–회전**
으로 푼다. 직진은 항상 전진이다.

**마커 재탐지.** 베이에서 마커가 안 보이면 **제자리 회전만으로** 기준 방위에서 좌우 교대로 훑는다
(−Δ, +Δ, −2Δ, +2Δ …). 병진하지 않는 이유는 마커를 못 본다는 것이 곧 자기 위치를 못 믿는다는 뜻이라
어디로 갈지 모르기 때문이다. 제자리 회전이 footprint 밖으로 쓸고 가는 폭은 1.4 cm 뿐이다
(내접 0.175 / 외접 0.189 m). 스텝은 카메라 수평 화각보다 확실히 작아야 정지점 사이로 마커가 빠지지 않는다.

#### 5.3.4 추종 (FR-15~20, 30)

**대기공간에서 시작한다.** 셔틀이 로봇을 대기공간에 세우고 `shuttle/arrived` 를 내면 그때부터 이 노드가
잡는다. **로봇이 작업자에게 가는 것이 아니라 작업자가 로봇에게 온다** — 정차 자세는 서향으로 두고
RC카를 로봇 정면으로 돌아 들어오게 조종한다 (계약 8절). 그래서 도착 직후에는 돌지 않는다.

**상태** — `approach/status` 로 계약 4절의 8개 값만 낸다. 관제와 `kpi_logger` 가 이 값으로 구간을 자른다.

| 상태 | 뜻 |
|---|---|
| `SEARCHING` | 대상을 찾는 중 (기동 직후·Lost 후) |
| `APPROACH_1` / `APPROACH_2` | 중간 목표 / 최종 목표로 접근 중 |
| `ARRIVED` | 최종 이격까지 붙음 |
| `FOLLOWING` | 추종 중 — **추종거리(AC-05) 채점 구간은 여기뿐이다** |
| `STANDBY` | Lost 30 s 지속 |
| `IDLE` | `mission/finish` 로 정상 종료 (**실패가 아니다**) |
| `FAILED` | 포기 |

**두 겹 게이트.** 좌표를 목표로 삼기 전에 두 가지를 본다 — ① 인지가 준 map 좌표가 두 번 연속 30 cm 안에서
일관되는가 ② `target/paired` 가 참인가. 움직인 직후 2 초 동안 온 좌표는 버린다.

**점진 접근.** 남은 거리가 `approach_skip_m`(1.0 m) 안이면 곧장 FOLLOWING 이고, 멀면 목표를
`max(follow_standoff_m, gap/2)` 로 두어 **남은 거리의 절반씩** 좁힌다. 회차 수를 정해두지 않는 이유는
gap 이 반씩 줄어 하한에서 저절로 끝나기 때문이다. 멀리서 잰 좌표가 0.5~0.7 m 틀린다는 것이 이 설계의
전제이고, 그 오차가 회차마다 줄어드는지가 전제의 확인이다.

**Lost 3단 시계 (FR-16·17·19).** 같은 시계(마지막으로 본 뒤 지난 시간) 안의 세 지점이고 더해지지 않는다.

| 경과 | 동작 |
|---|---|
| `lost_s` 1.0 s | 주행 취소 · 제자리 대기 |
| `research_s` | 회전 탐색 시작 |
| `standby_s` 30 s | STANDBY 로 물러남 — **대기로 끝난 재획득은 실패로 센다** |

**회전 방향은 아무 쪽이나 정하지 않는다** — 웹캠2 힌트 > 마지막으로 본 쪽 > 기본 30° 순이다. 천장 카메라가
없어도 놓치기 직전 위치는 우리가 들고 있다. 힌트는 **작업자 번호가 내 담당과 같을 때만** 쓴다 —
힌트 토픽이 전역이라 그 검사가 비면 상대 로봇의 작업자 힌트로 회전 방향을 정하게 된다.

**재획득은 게이트를 다시 통과해야 한다** (FR-18) — 다시 보였다고 바로 따라가지 않는다.

**종료 (FR-30).** `mission/finish` 는 1 초 안에 페어링을 풀고 goal 을 취소한 뒤 **`IDLE`** 을 낸다.
여기서 `FAILED` 를 내면 관제가 미션을 영구 실패로 굳힌다.

**비상정지 중에는** 주행을 멈추고 기다렸다가, 풀리면 **하던 단계부터 이어서** 한다. 처음으로 되돌리지 않고,
실패로도 내지 않는다.

#### 5.3.5 안전 감시 — 최종 속도의 단일 중재자 (FR-21·22)

**워치독을 최종 속도의 단일 중재자로 바꿨다.** 이전에는 각 주행 노드가 `cmd_vel` 을 내고 워치독이 정지 중에만
덮어썼다. 지금은 **워치독만 `cmd_vel` 을 발행한다.** 다른 노드는 목적지별 입력 토픽으로 낸다.

```
Nav2 최종 출력 ──▶ cmd_vel_navigation ─┐
추종 탐색 회전 ──▶ cmd_vel_search ─────┴─▶ safety_watchdog ──▶ cmd_vel ──▶ 로봇
 ▲
 /scan · odom · dock_status
 safety/paused/<노드> (정지 확인)
```

발행자를 하나로 모은 이유 — estop 중에 외부 명령을 통과시키는 방식이면 **estop 을 무시한 노드가 로봇을
움직이는 구조**가 된다. 지금은 estop 동안 들어온 명령을 그냥 버린다.

**상태기계** (`core/safety.SafetyController`, 20 Hz)

| 상태 | 뜻 | 나가는 조건 |
|---|---|---|
| `STOPPING` | 장애물 감지 — 멈추는 중 | 실제 정지 확인 + 회전 여유 확보 → `SAFE_ROTATE` |
| `SAFE_ROTATE` | **워치독이 직접 만드는 탈출 회전** (0.25 rad/s) | 시간 7 s · 각도 90° · 여유 부족 → `STUCK` |
| `STUCK` | 스스로 못 푼다 — 정지 유지 | 360° 여유가 5 프레임 회복 + 정지 확인 → 재시도 (최대 2회) |
| `CLEAR` | 정상 | 전방 0.30 m → `STOPPING` |

**거리 두 개를 나눈 이유.** 세우는 거리 0.30 m, 푸는 거리 0.38 m — 8 cm 여유는 라이다 오차의 네 배다.
종전 0.45 m 는 **P1 반폭 0.395 m 보다 커서** 통로 안에서는 어느 방향으로 돌아도 해제가 안 됐다.

**각도 한도가 시간 한도보다 먼저 걸려야 한다.** 반각 35° 부채꼴에서 정면 벽 `d` 를 풀려면
`d / cos(θ − 35°) > 해제거리` 가 되는 θ 까지 돌아야 한다 — 0.30 m 면 72.9°, 실측 0.29 m 면 75.3° 다.
종전 `0.25 rad/s × 5 s = 71.6°` 는 **그 어느 것도 못 풀었고**, 각도 상한 75° 는 5 s 가 항상 먼저 걸려
죽은 값이었다. 지금 값 `0.25 × 7 s = 100° > 90°` 라야 각도가 먼저 걸리고 0.22 m 벽까지 덮는다.
생성자가 이 부등식을 검사해서, 어기면 노드가 아예 뜨지 않는다.

**정지 확인을 받기 전에는 회전하지 않는다.** 각 주행 노드가 `motion_pause` 로 「내 액션은 멈췄다」를
0.1 s 마다 보내고, 워치독은 그것과 `odom` 실측 속도가 둘 다 정지를 가리킬 때만 탈출 회전을 시작한다.
액션이 살아 있는 채로 돌리면 Nav2 와 워치독이 동시에 바퀴를 잡는다.

**도크에 물려 있는 동안은 전방 판정을 하지 않는다.** Create 3 는 도크에 코를 박고 들어가므로 도킹 상태의
전방 ±35° 안에 있는 것은 도크와 그 뒤 벽이고 0.30 m 안에 반드시 들어온다. 그것을 장애물로 세면
**estop 이 항상 걸려 언독 자체가 불가능해진다**. 도크를 벗어나면 자동으로 복귀한다.

**스캔이 없으면 `STUCK` 이다** — 「모름」을 「비었음」으로 치지 않는다. 빔 유효율이 50 % 미만이면
센서가 가려졌거나 고장난 것으로 보고 회전을 허용하지 않는다 (실측 유효율 74~81 %).

#### 5.3.6 구역 토큰 (FR-10·31)

`zone_semaphore` 는 **잠금 서버일 뿐이다.** 시나리오도, 로봇 위치도, 아레나 좌표도 모른다.

- **획득** — 비어 있고 대기열 선두면 부여, 이미 내가 보유 중이면 그대로 부여(보유 시계는 리셋하지 않는다),
 남이 보유 중이면 거절하고 FIFO 대기열에 등록.
- **반납** — 보유자면 비우고, 대기 중이었으면 대기열에서 뺀다. **토큰을 받았든 못 받았든 반드시 반납을 부른다.**
- **감시** — 보유가 60 s 를 넘으면 강제 회수한다 (교착 자동 복구).

**진입 판정은 좌표로 하지 않는다 (방법 B,).** 경로가 베이 → P1 → 대기공간으로 고정돼 있으므로
획득 시점과 반납 시점이 둘 다 폴리곤 밖이면 「진입 전 획득·이탈 후 반납」이 성립한다. 좌표 감시는 계산이
복잡하고 경로가 폴리곤 가장자리를 스치면 진입을 놓친다.

**이벤트 중복 발행 금지 규칙 셋** — K-06(교착 자동 해결률)이 이것으로 매겨지므로 하나만 어겨도 값이 틀어진다.
`GRANT` 는 새로 획득할 때만 · `REJECT` 는 대기가 시작될 때 1건만(2 초 주기 재요청은 발행 안 함) ·
`RELEASE` 는 실제 보유자의 반납만.

#### 5.3.7 복귀·도킹 (FR-31·32)

**토큰 순서가 DOCK → P1 이다.** 홀딩 포인트가 **P1 바깥**(중앙 복도, 최근접 도크에서 1.52 m)이라,
DOCK 을 못 얻은 로봇은 애초에 P1 에 들어가지 않아야 하기 때문이다.

**접근 지점을 반드시 거친다.** 도크 간격이 0.84 m(실측) 뿐이라 멀리서 도킹을 부르면 **옆 도크로 향할 수 있다.**
IR 차폐판을 두지 않기로 했으므로 접근 지점 경유가 오도킹을 막는 유일한 방어다.
실패하면 **접근 지점으로 되돌아가** 최대 2회 재시도한다 — 후진이 아니라 계약이 정한 「제자리 회전 뒤 전진」이다.

**`DOCKED` 는 AC-13 운용시간의 끝이다.** 성공 직후 `READY` 를 내지 않는다 — 내면 운용시간이 도크에서
한 번 더 끊긴다. 결과가 성공이든 실패든 **DOCK 토큰은 반드시 반납한다** — 안 하면 그 구역이 영영 잠긴다.

#### 5.3.8 파라미터 (단일 출처: `mro_bringup/params/common.yaml` · `robotN.yaml` · `arena.yaml`)

| 값 | 현재 | 근거 |
|---|---|---|
| `t_lim_s` | 120 s | 미션 제한시간. **로봇·관제 공유 계약값** — 갈라지면 로봇은 가는데 관제만 초과를 찍는다. `[Q-14 실측 확정 대기]` |
| `t_ret_s` | 180 s | 복귀 제한시간 `[팀 가정]` |
| `bay_dwell_s` | 5 s | 적재 시뮬레이션 |
| `bay_marker_range_m` | 0.9 m | 하드 바닥 0.80 m 에 10 cm 여유 (실측) |
| `bay_align_min_range_m` | 0.80 m | 목표 거리의 하한 — 선반에 밀어 넣는 설정을 거르는 마지막 방어 |
| `aruco_max_dist_m` | 1.5 m | 계약 6절 수용 조건. 인지의 발행 한계 2.0 m 와 별개인 **주 게이트** |
| `follow_standoff_m` | 0.5 m | 유지 거리 |
| `approach_skip_m` | 1.0 m | 이 안이면 접근 단계를 건너뛴다 |
| `lost_s` / `standby_s` | 1.0 s / 30 s | 계약값 (SOT) |
| `v_max` | 0.3 m/s | FR-25 공통 상한 (실제로는 Nav2 쪽 값이 이긴다) |
| `stop_dist` / `clear_dist` | 0.30 / 0.38 m | 8 cm 여유는 라이다 오차의 네 배 |
| `safe_rotate_rate` / `_timeout_s` / `_max_angle_deg` | 0.25 rad/s / 7 s / 90° | 각도가 시간보다 먼저 걸려야 한다 |
| `rotation_clearance_m` | 0.21 m | 360° 회전 여유. 외접 0.189 m + 여유 |
| `dock_retry` | 2 | FR-32 |

#### 5.3.9 실패 시 동작

| 상황 | 동작 |
|---|---|
| Nav2 가 정해진 횟수 안에 활성화되지 않음 | T_lim 을 태우지 않고 곧장 `FAILED` + 사유 |
| 도킹 여부를 시간 안에 알 수 없음 | 무한 대기하지 않고 상한(5 s)에서 포기 — 무한 대기면 T_lim 조차 안 돌아 `FAILED` 도 안 나온다 |
| 베이에서 마커 미확인 | 제자리 회전 스윕 → 횟수 초과 시 오도착으로 포기 (FR-08) |
| P1 토큰 거절 | 그 자리에서 2 s 뒤 재요청, `AT_BAY` 유지 |
| T_lim 초과 | 미션 폐기. 늦은 도착은 계상하지 않는다 |
| 추종 중 Lost | 1 s 정지 → 회전 탐색 → 30 s 면 STANDBY |
| `mission/finish` | 1 s 안에 페어링 해제·goal 취소 → `IDLE` (실패 아님) |
| estop | goal 취소 · 병진 명령 금지 · 풀리면 하던 단계부터 재개 |
| 도킹 실패 | 접근 지점 복귀 후 2회 재시도 → 그래도 실패면 `FAILED`. **토큰은 반드시 반납** |
| 토큰 서비스 미기동 | ⚠ 현재 `True` 를 돌려 획득한 것으로 치고 진행한다 — 2대 운용에서 관제가 잠깐 죽으면 두 로봇이 동시에 P1 에 들어간다 |

---

### 5.4 관제 (Fleet) — FR-03, 05, 06, 26~29, 33

#### 5.4.1 요구사항

| FR | 요구 |
|---|---|
| FR-03 | 등록 순서대로 로봇 배정, DB `workers` 기록. 동시 검출 시 결정론적 순서 |
| FR-05 | UI 에서 작업자·베이 입력 → `RequestMission` → 접수 표시 ≤ 2 s |
| FR-06 | 배터리 임계 미만이면 미션 거부, 사유 기록 |
| FR-26 | 로봇별 상태를 `FleetStatus` 로 1 Hz 발행 (**뷰어**) |
| FR-27 | `RETURN` / `RETURN_ALL` 제공. **STOP·RESUME 은 거부** |
| FR-28 | 상태 전이·개입·실패를 DB `events` 에 ROS 시각으로 기록 |
| FR-33 | 종료 발행 → 도킹 완료 시간·재시도 기록 |
| FR-13 | 게이트 미달 30 s 지속 시 명시 페어링 폴백 |

**역할 경계 (BRD 5장).** 관제는 **단순 뷰어**다. 값을 만들지 않고 모은다.
경로 원격 조종·운영자 정지/재개는 Phase 1 범위 밖이며, **관제 PC 가 죽어도 로봇의 정지 동작은 유지된다**
(안전 계층 원칙). 관제는 안전 경로에 없다.

---

#### 5.4.2 아키텍처 (모듈)

**한 프로세스로 합쳐져 있다.** `control_ui` 는 더 이상 별도 노드가 아니다 —
`fleet_manager_node` 가 아래 네 모듈을 안고 돈다. 콘솔 진입점은 `fleet_manager_node` 와
`zone_semaphore_node` 둘뿐이다.

| 모듈 | 파일 | 역할 |
|---|---|---|
| 미션·배정 | `nodes/fleet_manager_node.py` | 작업자 배정 · 미션 생성과 전이 · 서비스 3종 · DB 기록 |
| 로봇 상태 | `nodes/robot_status.py` · `nodes/robot_state_manager.py` | 로봇별 구독과 `/fleet/status` 집계 |
| 화면 | `nodes/monitor_web_app.py` | Flask 대시보드 · HTTP API · 지도 표시 |
| UI 명령 | `nodes/fleet_ui_commands.py` | 화면의 요청을 ROS 서비스로 · 개입 이벤트 발행 |
| DB | `core/db.py` · `config/schema.sql` | SQLite `~/mro_fleet.db` |

**`zone_semaphore` 는 이 절이 아니다.** 패키지는 `mro_fleet` 안에 있지만 담당은 주행이고 설계는 5.3 이다
. 관제는 그 결과인 `/fleet/zone_event` 를 **받아 DB 에 적기만** 한다.

---

#### 5.4.3 데이터 흐름 (입력 · 처리 · 출력)

**입력**

| 토픽·서비스 | 타입 | 쓰임 |
|---|---|---|
| `/webcam1/worker_descriptor` | WorkerDescriptor | 작업자 등록 → 로봇 배정 |
| `robotN/shuttle/status` | String | 셔틀 단계 미션 전이 · 화면 상태 |
| `robotN/approach/status` | String | 추종 단계 미션 전이 · 화면 상태 |
| `robotN/target/paired` | Bool | `PAIRED` 전이 |
| `robotN/target/worker_id` | String | 페어링된 작업자 이름 |
| `robotN/target/pair_state` | String | `PAIRED`/`GATE_FAIL`/`NO_TARGET` 보관 |
| `robotN/battery_state` | BatteryState | FR-06 배정 거부 판정 |
| `robotN/amcl_pose` | PoseWithCovarianceStamped | `FleetStatus` 의 x·y·yaw |
| `robotN/safety/estop` | Bool | 화면 표시 · `ESTOP` 이벤트 |
| `/fleet/intervention` | Intervention | `INTERVENTION` 이벤트 |
| `/fleet/zone_event` | String | `ZONE` 이벤트 (K-06 원천) |
| `/fleet/request_mission` | srv RequestMission | 미션 생성 |
| `/fleet/operator_command` | srv OperatorCommand | RETURN / RETURN_ALL |
| `/fleet/manual_pair` | srv ManualPair | FR-13 폴백 입구 |

**출력**

| 토픽·서비스 | 타입 | 시점 |
|---|---|---|
| `/fleet/mission` | Mission | 미션 상태가 바뀔 때마다 |
| `/fleet/status` | FleetStatus | **1 Hz 타이머** |
| `robotN/mission/assigned` | Mission | `RequestMission` 수락 시 1회 |
| `robotN/mission/finish` | Bool | `RETURN`/`RETURN_ALL` 시 |
| `robotN/manual_pair` | srv ManualPair | 폴백 확정 요청 |

---

#### 5.4.4 작업자 등록과 로봇 배정 (FR-03)

**등록과 배정은 분리돼 있다.** `/webcam1/worker_descriptor` 를 받으면 작업자만 등록하고(DB `workers` 에 `worker_id`·복장 클래스·등록 시각 upsert, 이미 등록된 `worker_id` 는 무시), 로봇은 그 시점에 붙이지 않는다.

**배정은 `RequestMission` 을 수락할 때** 한다. 조건은 셋이다 — 진행 중인 미션이 없고, E-stop 이 아니며, 배터리가 기준(FR-06) 이상인 AMR. 후보가 여럿이면 `robot_namespaces` 목록 순서로 고른다. **코드는 로봇 대수를 가정하지 않는다** — 배정·집계·`RETURN_ALL` 이 모두 이 목록(`params/fleet.yaml`)을 순회한다.

두 작업자가 웹캠1 에 동시에 잡히면 트랙 ID 가 작은 쪽이 먼저 등록되므로(FR-03), 같은 순서로 요청이 들어오는 한 배정 순서도 재현된다.

---

#### 5.4.5 미션 생성과 거부 (FR-05 · FR-06)

`RequestMission(worker_id, bay)` 처리 순서:

1. 그 작업자가 배치 창에 대기 중이면 먼저 확정한다(배정 없이 미션이 생기는 것을 막는다).
2. 배정된 로봇이 없으면 `accepted=False`, `"미등록 작업자"`.
3. **배터리 < `battery_min`(0.25) 이면 거부** — `accepted=False`,
 events 에 `ASSIGN_REFUSED_BATTERY`(value=배터리) 기록. (FR-06)
4. 통과하면 `create_mission` → **`Mission.ASSIGNED` 로 즉시 set_state** →
 `/fleet/mission` 과 `robotN/mission/assigned` 발행.
5. 로봇의 상태 캐시를 초기화한다 — `approach_state`·`shuttle_state` = `UNKNOWN`,
 `status_owner` = None. **이전 미션의 잔상이 새 미션에 새지 않게 하는 장치다.**

> `Mission.CREATED`(0) 는 **쓰지 않는다.** 생성 즉시 `ASSIGNED` 로 간다.
> (`core/db.py:41` 주석이 "CREATED 상태의 미션을 만들고" 라고 되어 있으나 코드와 다르다 — 5.4.11 ②)

---

#### 5.4.6 미션 상태 전이 — **이 절이 핵심이다**

미션 상태는 **9개 중 8개**를 쓴다(`CREATED` 제외). 전이를 일으키는 입력은 셋뿐이다.

##### 전이표

| 입력 | 값 | → 미션 상태 | 함께 하는 일 |
|---|---|---|---|
| `RequestMission` | 수락 | `ASSIGNED` | DB `create_mission`+`set_state` · `mission/assigned` 발행 |
| `shuttle/status` | `SHUTTLE` | `SHUTTLE` | DB `set_state` |
| `shuttle/status` | `AT_BAY` | `AT_BAY` | DB `set_state` |
| `shuttle/status` | `WAITING` | `WAITING` | DB **`mark_waiting`** (리드타임·T_lim 판정). 늦었으면 `arrived_late` 이벤트 |
| `target/paired` | `True` | `PAIRED` | `paired_worker` 갱신 · `PAIRED_WORKER` 이벤트 |
| `approach/status` | `FOLLOWING` | `FOLLOWING` | DB `set_state` |
| `approach/status` | `STANDBY` | `STANDBY` | DB `set_state` |
| `shuttle/status` | `DOCKED` | **`DONE`** (종결) | DB `set_state` |
| `shuttle/status` | `FAILED` | **`FAILED`** (종결) | DB `set_state(result='FAIL_ROBOT')` |

**전이를 일으키지 않는 입력** — 화면 표시와 이벤트 기록만 한다:
`READY` · `RETURNING` · `DOCK_WAIT` · `DOCKING` (셔틀) ·
`IDLE` · `SEARCHING` · `APPROACH_1` (접근) ·
`RETURN`/`RETURN_ALL` 명령 · T_lim 초과.

> **접근 상태는 8개 중 5개만 실제로 온다** (계약 v1.16·v1.17). `APPROACH_2`·`ARRIVED` 는 기본
> 파라미터에서 조건이 겹치지 않아 발행되지 않고, `FAILED` 는 `follow_target` 이 내지 않는다
>.
> **셋 다 '안 나오는 것이 고장이 아니다'.** 이 값을 기다리는 분기를 관제에 두면 영원히 안 온다.
> 값 목록에서 뺀 것은 아니다 — `approach_skip_m` 한 줄이면 되살아난다.

##### 전이 가드 — 되돌림 방지

두 출처(셔틀·접근)가 동시에 상태를 보내므로, 늦게 온 옛 상태가 단계를 되돌리지 않게 막는다.

1. 현재가 **`DONE`** 이면 어떤 전이도 없다.
2. **접근발 전이**는 현재가 `WAITING`·`PAIRED`·`FOLLOWING`·`STANDBY` 일 때만 허용.
 또 **복귀 우선권**(셔틀이 `RETURNING`/`DOCK_WAIT`/`DOCKING`/`DOCKED`/`FAILED`)이면 거부.
 → 복귀 시작 후 늦게 도착한 `FOLLOWING` 이 미션을 되살리지 못한다.
3. **셔틀발 전이** 중 `SHUTTLE`·`AT_BAY`·`WAITING` 은, 현재가 이미 접근 단계
 (`PAIRED`/`FOLLOWING`/`STANDBY`)면 거부.

##### 화면에 무엇을 보일지 (`status_owner`)

`FleetStatus.state` 는 미션 상태가 아니라 **두 상태 출처 중 하나의 원문**이다. 고르는 규칙:

- 셔틀이 복귀 상태이거나 `FAILED` → `shuttle` (미션이 `DONE` 이 아닌 한)
- 셔틀이 출고 상태(`READY`/`SHUTTLE`/`AT_BAY`/`WAITING`)이고 미션이 접근 단계가 아니면 → `shuttle`
- 그 밖에, 복귀 우선권이 없고 미션이 `DONE` 이 아니며 접근 상태가 활성이면 → `approach`
- 아무 출처도 안 정해졌으면 `approach_state`, 그것도 없으면 `shuttle_state`

---

#### 5.4.7 상태 집계와 발행 (FR-26)

**1 Hz 타이머**가 로봇마다 `FleetStatus` 를 하나씩 낸다:
`state`(5.4.6 규칙) · `battery` · `x`·`y`·`yaw`(amcl_pose) · `mission_id` · `paired_worker` · `estop`.

BR-T08 의 "갱신 ≤ 2 s" 를 1 Hz 로 만족시킨다. **집계만 하고 판단하지 않는다.**

---

#### 5.4.8 운영자 명령 (FR-27)

- 허용은 **`RETURN`**(해당 로봇) 과 **`RETURN_ALL`**(전체) 뿐. 그 밖은 `ok=False` 로 **거부**한다
 (STOP·RESUME 은 결정으로 범위 밖).
- `RETURN` 은 알 수 없는 `robot_ns` 면 거부.
- 처리: 대상마다 events 에 `OPERATOR` 기록 → `robotN/mission/finish` 에 `True` 발행.
- **미션 상태는 바꾸지 않는다.** `DONE` 은 로봇이 실제로 도킹해 `shuttle/status = DOCKED` 를 낼 때만 온다.
 → 종료 버튼은 "복귀 지시" 이지 "미션 종결" 이 아니다.

---

#### 5.4.9 DB 반영 시점 (FR-28 · FR-33)

**`workers`** — 배정 확정 시 1회.

**`missions`** — `create_mission`(ASSIGNED) → 전이마다 `set_state` →
`WAITING` 에서 `mark_waiting`(리드타임·`result` 확정) → `FAILED` 는 `result='FAIL_ROBOT'`.

**T_lim 처리 (계약 7절).** 1 Hz 타이머가 `expire_overdue` 를 돌려 `created_at + t_lim < now` 이고
`result` 가 비어 있는 미션을 **`result='FAIL_TLIM'` 으로 한 번만** 확정하고 `TLIM_EXCEEDED` 이벤트를 남긴다.
**미션 `state` 는 바꾸지 않고 운행도 멈추지 않는다** — 초과 후 도착하면 `arrived_late` 이벤트로 따로 남긴다.
계약이 요구한 "이중 계상하지 않는다" 가 이 구조로 지켜진다.

**`events`** — `kind` 별 기록 시점:

| kind | 언제 | value / note |
|---|---|---|
| `STATE` | 셔틀·접근 상태 문자열이 **바뀔 때마다** | value=상태, note=출처(`shuttle`/`approach`) |
| `ESTOP` | estop 값 변화 | value=bool |
| `PAIRED_WORKER` | 페어링 작업자 변화 | value=worker_id |
| `INTERVENTION` | `/fleet/intervention` 수신 | **메시지의 원 시각**을 쓴다(유효하지 않으면 현재 시각) |
| `ZONE` | `/fleet/zone_event` 수신 | value=REJECT/GRANT/RELEASE/RECLAIM, note=zone_id |
| `ASSIGN_REFUSED_BATTERY` | FR-06 거부 | value=배터리 |
| `TLIM_EXCEEDED` | T_lim 초과 확정 | value=mission_id |
| `OPERATOR` | RETURN/RETURN_ALL | value=명령 |
| `arrived_late` | T_lim 후 도착 | value=리드타임(s) |

`/fleet/zone_event` 는 형식(3토막)·구역 ID·로봇 이름·동작 값을 **모두 검증**하고, 어긋나면 경고만 남기고
버린다. **개입으로 계상하지 않는다** — 섞으면 IPH 분자가 부풀어 AC-13 이 실제보다 나빠 보인다 (계약 v1.11).

---

#### 5.4.10 명시 페어링 폴백 (FR-13)

**FR-13** — 30 s 는 관제가 세지 않는다. `amr_detection` 이 재서 `target/pair_state = GATE_FAIL` 을 낸다
(계약 10.1: 관제는 "사람이 없는 것"과 "못 알아보는 것"을 구분할 수 없다).
`control_ui` 가 `robotN/target/pair_state` 를 **직접 구독**해 버튼을 띄우고,
`/fleet/manual_pair` → `fleet_manager` → `robotN/manual_pair` 로 전달한다. 폴백 1건은 개입으로 계상한다.

---

#### 5.4.11 파라미터

`mro_bringup/params/fleet.yaml` (관제 전용) + `common.yaml`(로봇과 공유하는 계약값):

| 이름 | 값 | 출처 | 뜻 |
|---|---|---|---|
| `robot_namespaces` | `[/robot3, /robot8]` | fleet.yaml | **로봇 명단의 단일 출처** |
| `battery_min` | 0.25 | fleet.yaml | 이 미만이면 배정 거부 (FR-06) |
| `worker_assignment_window_s` | 1.0 | fleet.yaml | 동시 등록으로 볼 시간창 |
| `t_lim_s` | 120.0 | **common.yaml** | 미션 제한시간. **로봇과 같은 값이어야 한다** — 갈라지면 로봇은 계속 가는데 관제만 `TLIM_EXCEEDED` 를 찍는다 |
| `db_path` | `~/mro_fleet.db` | 코드 기본값 | 런치가 주는 경로 |

---

#### 5.4.12 실패 시 동작

| 상황 | 동작 |
|---|---|
| 로봇 상태 토픽이 끊김 | 마지막 값을 유지한다. 별도 타임아웃 없음 `[검토 필요]` |
| `zone_event` 형식 오류 | 경고 로그 후 버린다. DB 에 적지 않는다 |
| `pair_state` 값이 셋 중 하나가 아님 | 경고 로그 후 무시 |
| 미등록 작업자로 미션 요청 | `accepted=False`, DB 기록 없음 |
| DB 쓰기 실패 | `[검토 필요]` — 현재 예외 처리 없음 |

---

---

## 6. Security Design

- **개인정보** — 웹캠·로봇 영상은 서술자 생성과 시험 계측 목적으로만 처리한다. 원본은 세션 판정 후 폐기하고, 얼굴 인식을 쓰지 않으며, 촬영 사실 표시를 부착한다 (NF-08, BRD NFR-C)
- **외형 서술자**는 색 히스토그램(H16×S8)이라 얼굴·신원을 담지 않는다. 세션 종료 시 폐기한다
- **네트워크 경계** — PC 사이를 건너는 정보는 ROS 2 토픽·서비스뿐이고 PC 간 직접 소켓 통신은 없다 (NR-06). 관제 UI 는 아레나 내부 Wi-Fi 의 HTTP(:5000)이며 외부 시스템(ERP/MRO) 연동이 없다 (IR-05)
- **안전 경로의 독립** — 정지·회피는 재식별·웹캠·Wi-Fi·관제와 무관한 온보드 경로다. 어느 것이 뚫리거나 죽어도 로봇은 멈춘다 (NF-07)

---

## 7. Performance Requirements

목표 수치는 BRD 1.2 KPI 표의 값이고, 측정 정의는 BRD 7.1 을 따른다.

| ID | 구분 | 요구사항 | 목표 | BRD |
|---|---|---|---|---|
| NF-01 | 성능 | 추종 미션 성공률 / 교차 후 유지율 / ID Swap | ≥ 90 % / ≥ 95 % / ≤ 2 per 100 | K-01~03 |
| NF-02 | 성능 | 재획득 성공률·시간 (힌트 有) | ≥ 90 %, p95 ≤ 5 s | K-04 |
| NF-03 | 성능 | 배송 성공률(T_lim 이내) / 교착 자동 해결률 | ≥ 95 % / ≥ 95 % | K-05, K-06 |
| NF-04 | 성능 | 위험 감지 → 정지 / 상태 갱신 / 요청 접수 | ≤ 1 s / ≤ 2 s / ≤ 2 s | NFR-P |
| NF-05 | 운용 | 개입률 IPH / 추종 유지율 (FR-19 와 **동일 정의** — Standby 구간을 분모에서 제외) | ≤ 1.0 회/h / **≥ 95 % AND Standby 점유 ≤ 15 %** | K-07, K-08 |
| NF-06 | 안전 | 사람·AMR 간·구조물 접촉, 적재물 낙하 | **0건 (절대)** | K-09, K-11 |
| NF-07 | 신뢰성 | 한 로봇 실패가 다른 로봇에 영향 없음, 관제·네트워크 장애 시 온보드 안전 정지 성립 | 설계 검증 + TS-14 | NFR-R |
| NF-08 | 보안·개인정보 | 영상은 서술자·계측 목적만, 원본 세션 후 폐기, 얼굴 인식 없음, 촬영 표시 부착 | 설계 검토 | NFR-C |
| NF-09 | 관측성 | 상태·좌표·개입이 로그와 `target/image` 로 확인 가능 | — | NFR-R.4 |
| NF-10 | 재현성 | `docs/setup.md` 만으로 다른 PC 에서 실행 재현 | 5 PC | — |

---

## 8. Error Handling and Recovery

BRD 2.5 의 예외 흐름 E-1~E-7 에 설계 단계에서 드러난 E-8·E-9 를 더했다.

| ID | 상황 | 감지 | 대응 | 복구 | 기록 |
|---|---|---|---|---|---|
| E-1 | 교차·가림 → Lost | `target/visible` 1 s | 감속·정지 → 힌트 방향 회전 탐색 | 게이트 통과 후 재개 | LOST/REACQUIRED |
| E-2 | 경로 차단 | Nav2 재계획 실패 | 10 s 이내 재계획 또는 대기 | 경로 회복 시 재개 | STATE |
| E-3 | 2대 교착 | 토큰 거절 / 60 s 타임아웃 | 후행 대기, 타임아웃 시 강제 회수 | 선행 통과 후 재요청 | ZONE |
| E-4 | 전방 진입 | `/scan` 0.3 m | estop, cmd_vel 0, goal 취소 | 0.45 m 밖 5스캔 후 해제 | ESTOP |
| E-5 | 웹캠·Wi-Fi 단절 | 힌트·관제 토픽 타임아웃 | 온보드 추종·회피·정지 지속, 신규 미션 중단 | 재연결 시 상태 재발행 | STATE |
| E-6 | 작업자 우선 통행 | 경합 감지(costmap) | AMR 양보·대기 | — | — |
| **E-7** | **도크 경합·도킹 실패** | 토큰 거절 / dock 액션 실패 | 후행은 홀딩 포인트 대기. 실패 시 이탈 후 2회 재시도 | 선행 도킹·토큰 반납 후 진입 | ZONE / DOCK_RETRY |
| E-9 | Nav2 lifecycle 멈춤 | `change_state timeout` | `tools/nav_recovery_fetch.sh` | 5 분 내 | 개입(RESTART) |
| E-8 | 재식별 게이트 30 s 미달 | `target/pair_state = GATE_FAIL` (인지가 30 s 계측, 계약 10.1) | 대기 + 관제 UI 명시 페어링 폴백 (`srv/ManualPair`) | 폴백 후 추종 — 현재 미션에서만 유효, `mission/finish` 로 해제 | **개입 `MANUAL_PAIR`** (v1.12, IPH 포함) |

---

## 9. Testing and Validation

단위 → 통합 → 운영 순으로 올린다. 시험 ID(TS)와 표본은 BRD 7장 수락 기준과 같다.

| 단계 | 대상 | 시험 | 판정 |
| --- | --- | --- | --- |
| 단위 | 노드·core 모듈 | TS-U01~U11 (등록, 호모그래피, 검출, 측면·후면, 거리·ArUco, 셔틀, standoff, watchdog, 힌트 회전, 접수, 종료 명령 — **TS-U03·U04 판정 기준은 FR-11·FR-18**, v0.10), **TS-U12 도킹 반복**(로봇별 10회, 오도킹 0·성공률 기록 — 0.84 m 간격 검증) | FR 수락 기준 |
| 통합 | PC 간·노드 간 | TS-I01~I05 (등록→미션, 미션→셔틀, 재식별 페어링, 힌트 재탐지, 2대 교착·정지) | AC-06~09, AC-12 |
| 선행 | 추종 비의존 | **TS-07 배송 20회 · TS-08 교착 20회 · TS-U12 도킹** | AC-08, AC-09 |
| 운영 | 시나리오 | TS-01 완주 **5회**, TS-02/03 추종 **15회**·교차 **40회**, TS-13 급정지 **10회**, **TS-20 종료 20회**, TS-14 결함 주입 **15회(유지)**, TS-17 연속 2 h | AC-01~05, 10, 11, 13, **14** |

**판정 원칙**: 성공 정의 사전 고정, 최악 조건(후면·측면 + 협소 통로 + 교차) 포함, 결함 주입, **안전 지표 위반 시 즉시 중단**. **추종거리는 검출 노드 로그로 판정**(FR-29, FR-34), 오추종·접촉은 웹캠2 영상 판독.

**통합 순서**: ① 로봇 1대 단독(S2→S4→S5→S6→S9·S10) → ② 웹캠·관제 연결(S1, S3, 힌트) → ③ 로봇 2대(E-3, E-7 순차 도킹) → ④ 결함 주입·연속 운용.

---

## 10. Deployment and Maintenance Plan

- **기동 순서 (로봇당)** — ① 위치 추정(localization) ② `shuttle_mission`(도크 좌표로 초기자세 발행) ③ Nav2. Nav2 의 `global_costmap` 은 기동 시 map→base_link 변환을 요구하고 활성화가 한 번 실패하면 재시도하지 않으므로, 초기자세가 먼저 있어야 한다
- **PC별 launch** — `mro_bringup` 의 `webcam1_pc.launch.py` · `webcam2_pc.launch.py` · `fleet_pc.launch.py` · `robot.launch.py`(`namespace:=/robotN`). 한 PC 에 한 launch 다
- **파라미터 단일 출처** — 공통값 `params/common.yaml`(로봇·관제 공유 계약값 포함), 로봇별 `robotN.yaml`, 아레나 `arena.yaml`, 관제 명단 `fleet.yaml`, 인지 `mro_perception/config/perception.yaml`, 웹캠 `mro_webcam/config/webcam.yaml`. 코드에 값을 박지 않는다 (SW-08)
- **네트워크 전제** — PC-F IP 고정(`192.168.108.41`), 모든 PC 와 로봇의 `ROS_DOMAIN_ID=3`, `ROS_DISCOVERY_SERVER=";192.168.108.41:11811"`. Nav2 launch 터미널만 `ROS_SUPER_CLIENT=False`
- **재현** — `docs/setup.md` 만으로 다른 PC 에서 5대 구성을 재현할 수 있어야 한다 (NF-10). 모델(`worker.pt`)·맵·호모그래피·기준 지문은 저장소에서 버전 관리한다
- **유지보수** — 계약(`docs/interfaces.md`)을 바꿀 때는 PM 에게 먼저 알리고 `mro_interfaces` 와 함께 고친다. 담당 단위 패키지, PR 리뷰, `docs/SOT.md` 동기화 (SW-09)
- **지원 도구** — `tools/kpi_report.py`(KPI 산출), `tools/md2docx.js`(문서 변환), Nav2 lifecycle 복구 스크립트(E-9)

---

## 11. Appendix

### A. 요구사항 추적표 — FR → 구현 노드 → BRD → AC → TS

요구사항 하나가 **어느 노드에서 구현되고 어느 시험으로 확인되는가.** 기능 요구사항의 전문과 수락 기준은 이 표가 단일 출처다. 결번은 의도된 것이다 — FR-24 결번, FR-35 폐지, TS-10~12 는 KOZ 시험이라 Phase 2 이관과 함께 빠졌다.

#### 등록·요청 (S1~S3)

| FR | 요구사항 | 수락 기준 | 구현 노드 | BRD | AC | TS |
|---|---|---|---|---|---|---|
| FR-01 | 웹캠1이 출입구 통과자를 검출하고 트랙 ID 부여 | 2명 별개 ID, 0.2 s 이하 오검출에 반응 없음 | `webcam_register` | BR-T28 | AC-02 | TS-U01, TS-U02 |
| FR-02 | 복장 클래스 + HSV 서술자를 `WorkerDescriptor` 로 발행 | 등록 ≤ 10 s, 서술자 길이 128 . 실기동 등록 0.90 s · 재등록 15/15 | `webcam_register` + `core/reid.hsv_hist` | BR-T28, T16 | AC-02 | TS-U01, TS-U02 |
| FR-03 | 등록 순서대로 로봇 배정, DB(workers) 기록 | 1:1 배정, 중복 배정 0, 동시 검출 시 순서 재현 | `fleet_manager` | S1 | AC-02 | TS-I01 |
| FR-04 | 초기자세 설정 → Nav2 확인 → undock → READY | Nav2 미준비 시 대기 후 자동 진행 | `shuttle_mission` | S2, BR-T14 | AC-02 | TS-I01 |
| FR-05 | UI 에서 작업자·베이 입력 → 미션 생성 (접수 ≤ 2 s) | 접수 표시 ≤ 2 s | `control_ui` → `fleet_manager` | BR-T05 | AC-07 | TS-U10 |
| FR-06 | 배터리 25 % 미만이면 미션 거부·사유 기록 | 임계 배정 0건 | `fleet_manager` | BR-T14 | AC-07 | TS-15 |

#### 셔틀 (S4)

| FR | 요구사항 | 수락 기준 | 구현 노드 | BRD | AC | TS |
|---|---|---|---|---|---|---|
| FR-07 | `mission/assigned` 수신 → 베이 waypoint 이동 → 창고에서 나올 때 P1 토큰 획득 | 토큰 없이 P1 진입 0건 | `shuttle_mission` + `zone_semaphore` | BR-T09 | AC-09 | TS-U06, TS-08 |
| FR-08 | `aruco/id` 일치로 도착 확정, 5 s 체류 후 대기공간 이동 | 오도착 0건 | `aruco_detector` → `shuttle_mission` | BR-T06 | AC-08 | TS-U05, TS-U06 |
| FR-09 | 리드타임 기록, T_lim 초과 시 실패 확정 (`arrived_late` 별기) | 성공률 ≥ 95 % | `fleet_manager` + `kpi_logger` | K-05, AC-08 | AC-08 | TS-07 |
| FR-10 | 토큰 FIFO, 보유 60 s 초과 시 강제 회수·기록 | 교착 자동 해결 ≥ 95 %, AMR 간 접촉 0 | `zone_semaphore` | BR-T09, AC-09 | AC-09 | TS-08 |

#### 합류·재식별 (S5)

| FR | 요구사항 | 수락 기준 | 구현 노드 | BRD | AC | TS |
|---|---|---|---|---|---|---|
| FR-11 | RGB-D 동기화 검출 + 역깊이 보정 → map 좌표 발행 | 0.8~2 m 에서 거리 오차 ≤ 2 % (TS-U05). **TS-U03 검출 :** 0.8 m·1.5 m 정면에서 인형 2체 각각 30 프레임 중 conf ≥ `detect_conf`(0.5) 검출 ≥ 90 %, 클래스 일치 100 %, 상대 로봇을 worker 로 오검출 0건 — 데이터셋이 포화(mAP 0.99)라 실물로만 볼 수 있다 | `amr_detection` | BR-T01 | AC-05 | TS-U05, TS-U03, TS-U04 |
| FR-12 | 재식별 게이트 통과 시에만 `target/paired=True` | 오페어링 0건 | `amr_detection` + `core/reid.gate` | BR-T16, R-05 | AC-04 | TS-I03 |
| FR-13 | 게이트 미달 30 s 지속 시 명시 페어링 폴백 제공 | **오페어링 0건 AND 폴백 발동률 ≤ 20 %** `[팀 가정]` — 상한이 없으면 게이트를 극단적으로 빡빡하게 잡아 매번 폴백으로 넘기는 '판단하지 않는 시스템' 이 만점을 받는다. **그래서 발동을 개입(`MANUAL_PAIR`)으로 세어 IPH 에 넣는다** — 발동률과 IPH 가 같은 원천(`events`)을 쓴다 | `control_ui` + `amr_detection`(서버) | BR-T16 | AC-04 | TS-I03 |
| FR-14 | 페어링 판정은 1 m 이내 정지 상태에서 수행 | — | `amr_detection` | Kn1_mini 실측 | AC-04 | TS-I03 |

#### 추종·재탐지·Standby (S6~S8)

| FR | 요구사항 | 수락 기준 | 구현 노드 | BRD | AC | TS |
|---|---|---|---|---|---|---|
| FR-15 | standoff 0.5 m 목표, 0.35 m 이상 이동 시에만 재목표 | 추종거리 p95 오차 ≤ 0.2 m, 최소 이격 ≥ 0.3 m | `follow_target` | BR-T02 | AC-03, AC-05 | TS-U07, TS-02 |
| FR-16 | 미검출 1.0 s → Lost → 1 s 이내 감속·정지 | 반응 ≤ 1 s | `follow_target` | BR-T03 | AC-06 | TS-05 |
| FR-17 | Lost 후 회전 방향을 `/webcam2/target_hint` 방위로 결정 | 재획득 ≥ 90 %, p95 ≤ 5 s (힌트 有) | `follow_target` ← `webcam_hint` | BR-T27 | AC-06 | TS-05, TS-19, TS-U09 |
| FR-18 | 재획득 시 FR-12 게이트 재통과 후에만 추종 재개 | 오추종(ID Swap) ≤ 2/100 교차. **TS-U04 측면·후면 :** 1 m 에서 인형을 0°/90°/180°/270° 로 돌려 각 30 프레임 검출 ≥ 90 %·`outfit_class` 유지, 게이트 미통과 방향에서는 `target/paired` 가 True 로 바뀌지 않는다. **더해서 «붙어야 할 때 붙는가» 도 본다 — 각 방향에서 같은 인형의 유사도를 기록하고, 정면(0°)에서는 `target/paired` 가 True 로 바뀌어야 하며, 방향별 유사도 최솟값을 표로 남긴다** ( 추가: 미통과만 보면 임계를 0.99 로 둬도 통과한다. pass/fail 만 남기면 `reid_threshold` 0.34 가 깨졌을 때 얼마나 깨졌는지 몰라 다시 찍어야 한다) — AC-10(상의 교체) 의 단위 근거이자 **`reid_threshold` 0.34 의 검증** | `amr_detection` + `follow_target` | BR-T16, K-03 | AC-04, AC-10 | TS-02, TS-03, TS-U04 |
| FR-19 | Lost 30 s → STANDBY, 동선 비점유 위치로 이탈 | **추종 유지율 ≥ 95 % AND Standby 점유 시간 비율 ≤ 15 %** `[팀 가정]` — Standby 구간을 분모에서 빼므로, 자주 Standby 로 빠질수록 유지율이 좋아지는 구멍을 막는다 | `follow_target` | BR-T17, T18 | AC-06 | TS-06 |
| FR-20 | 작업공간 내 추종 지속, 정지 시 STANDBY 규칙 | 동선 간섭 0건 | `follow_target` | S8 | AC-03 | TS-02 |

#### 안전 (E-4, E-5)

| FR | 요구사항 | 수락 기준 | 구현 노드 | BRD | AC | TS |
|---|---|---|---|---|---|---|
| FR-21 | `/scan` ±35° 0.3 m 2스캔 → estop · **병진 0 · 회전만 통과 · CLEAR 0.38** | 감지 후 정지 ≤ 1 s, 접촉 0. **변경 뒤 TS-04·TS-14 재실행** — 해제가 일러 재진입이 반복되지 않는지 본다 | `safety_watchdog` | BR-T04, T20 | AC-12 | TS-04, TS-14, TS-U08 |
| FR-22 | 모든 주행 노드는 estop 중 goal 취소·속도 명령 금지 | estop 중 이동 0 | `shuttle_mission`·`follow_target`·`return_dock` | BR-T20 | AC-12 | TS-14 |
| FR-23 | 재식별·웹캠2·Wi-Fi 어느 조합이 죽어도 FR-21 동작 | 3종 × 5회 정지 | `safety_watchdog` | AC-12 | AC-12 | TS-14 |
| FR-25 | v_max 0.3 m/s 공통 제한 | — | `params/robotN.yaml` + 주행 노드 | BR-T04 | AC-05 | TS-U07 |

#### 관제·기록 (S9 이전)

| FR | 요구사항 | 수락 기준 | 구현 노드 | BRD | AC | TS |
|---|---|---|---|---|---|---|
| FR-26 | 로봇별 상태를 `FleetStatus` 1 Hz 발행, UI 표시 (뷰어) | 갱신 ≤ 2 s, 2대 100 % 조회 | `fleet_manager` → `control_ui` | BR-T08, T15 | AC-07 | TS-16 |
| FR-27 | UI 가 RETURN / RETURN_ALL 제공, STOP·RESUME 거부 | 발행 반영 ≤ 2 s | `control_ui` → `fleet_manager` | BR-T10 | AC-07, AC-14 | TS-16, TS-20, TS-U11 |
| FR-28 | 상태 전이·개입·estop 을 DB·CSV 에 ROS 시각으로 기록 | 실패 미션 100 % 사유 기록, IPH 산출 가능 | `core/db` + `kpi_logger` | BR-T13, T26 | AC-13 | TS-18, TS-17 |

#### 종료·순차 복귀 (S9~S10)

| FR | 요구사항 | 수락 기준 | 구현 노드 | BRD | AC | TS |
|---|---|---|---|---|---|---|
| FR-30 | `mission/finish` 수신 → 즉시 페어링 해제·추종 중단 (≤ 1 s) | 종료 발행 → 추종 중단 **≤ 1 s**, 종료 후 추종 재개 0건 | `follow_target` | BR-T29 | AC-14 | TS-20, TS-U11 |
| FR-31 | 복귀 협소 구간은 토큰 통과, DOCK 은 한 번에 한 대 | **AMR 간 접촉 0건**, 두 대 동시 도크 진입 0건 | `return_dock` + `zone_semaphore` | BR-T31 | AC-14 | TS-20 |
| FR-32 | 도크 정면 접근 지점 경유 후 `dock`, 실패 시 2회 재시도 | 두 대 도킹 성공, **오도킹(옆 도크) 0건**, T_ret(180 s) 이내, 사람 개입 0 | `return_dock` | BR-T29, T31 | AC-14 | TS-U12, TS-20 |
| FR-33 | 종료 → 도킹 완료까지 소요 시간·재시도 횟수 기록 | 측정 전용(목표는 Pilot) | `kpi_logger` | BR-T26 | AC-13 | TS-20 |
| FR-29 | 추종거리 p95 는 검출 노드의 `target/distance` 로 판정 | — | `amr_detection` → `kpi_logger` | AC-05 | AC-05 | TS-02, TS-U05 |
| FR-34 | 계측 오차가 판정 기준의 1/3 미만인 수단으로만 채점 | 채점 수단의 오차 < 기준의 1/3 | 판정 규칙 + `tools/kpi_report.py` | AC-05 | AC-05 | TS-U05 |

### B. rqt_graph

`[첨부: 로봇 1대 + 웹캠 + 관제 전체 그래프 캡처]`

