# Feature Todo — 파일·메서드 기준

각 줄 끝의 `마감 M/D` 는 스케줄러 Time Line 기준이다. 진행률은 아래 표가 체크박스를 세어 보여준다.
**이슈를 닫거나 상태를 바꿀 때는 여기 체크박스도 같이 본다** (`AGENTS.md` 「이슈 등록」 절).

<!-- STATS:BEGIN -->

**전체 27/71 완료** (████████░░░░░░░░░░░░)  ·  기준 09/06

| 파트 | 완료 | 진행 | 지난 마감 |
|---|---|---|---|
| mro_interfaces [PM 김병직] | 3/3 | `██████████` | — |
| mro_perception [인지 — 정정인, 민범진] | 1/10 | `█░░░░░░░░░` | — |
| mro_webcam [웹캠 — 한세교, 김학민] | 5/14 | `████░░░░░░` | — |
| mro_navigation [주행 — 박병후, 이시율] | 10/27 | `████░░░░░░` | — |
| mro_fleet [관제 — 김민준] | 6/11 | `█████░░░░░` | — |
| mro_bringup [PM 김병직] | 2/6 | `███░░░░░░░` | — |

**오늘 마감 3건**

- mro_perception — 인형 2종 제작 → 데이터 수집·라벨링 (상하의 전체, 9/6 결정)
- mro_webcam — 새 아레나 호모그래피 cam1/cam2 캘리브레이션 (I-002) — 맵은 9/5 에 나왔다
- mro_navigation — TB4 2대 동시 운용 검증

> 이 표는 `python3 tools/todo_stats.py` 가 체크박스를 세어 다시 쓴다. 손으로 고치지 않는다.

<!-- STATS:END -->

## mro_interfaces [PM 김병직]
- [x] msg/srv 8종 정의 · 마감 9/6
- [x] **계약 v1.3 확정** — `docs/interfaces.md`. 구역 ID `P1`·`DOCK`, 확정 값 8건, 파라미터 단일 출처, 주기·시간 상수 규칙, 서술자 추출 영역 · 마감 9/6
- [x] `WorkerDescriptor.color_hist` — H16×S8=128, `reid.hsv_hist()` 공유 호출로 확정 · 마감 9/6

## mro_perception [인지 — 정정인, 민범진]
- [ ] 인형 2종 제작 → 데이터 수집·라벨링 (상하의 전체, 9/6 결정) · 마감 9/6
- [ ] `models/worker.pt` 학습·검증, 색 편향 점검 (I-003) · 마감 9/7
- [ ] `amr_detection_node.py`: `DETECT_CLASS` → worker_a/b, `/webcam1/worker_descriptor` 구독, `core/reid.gate` 적용 → `target/paired`, `target/worker_id` 발행 · 마감 9/9
- [ ] `amr_detection_node.py`: **`target/distance` 발행 (FR-29)** — 추종거리 판정의 기준값. kpi_logger 는 이미 구독 중이라 이것만 붙으면 AC-05 가 측정된다 · 마감 9/8
- [ ] **TS-U05 줄자 대조** — `target/distance` 의 정확도를 0.8~2 m 에서 사전 검증 (FR-34: 채점 수단의 오차 < 기준의 1/3). 이게 없으면 AC-05 를 그 값으로 채점할 근거가 없다 · 마감 9/8
- [ ] `amr_detection_node.py`: 모듈 상수 → ROS2 파라미터 (`config/perception.yaml`) · 마감 9/8
- [x] `aruco_detector_node.py`: 검출·거리 초안 · 마감 9/7
- [ ] `aruco_detector_node.py`: solvePnP 자세, 마커 ID 필터 · 마감 9/8
- [ ] `core/reid.py` 단위 테스트 (같은 인형 ≥ threshold, 다른 인형 < threshold) · 마감 9/8
- [ ] `reid_threshold` 확정 — 같은/다른 인형 유사도 분포를 그려 결정 (SOT Q-13). 추출 영역이 인형 전체라 다른 인형끼리도 0.5 안팎에서 시작한다 · 마감 9/8

## mro_webcam [웹캠 — 한세교, 김학민]
- [ ] **새 아레나 호모그래피 cam1/cam2 캘리브레이션 (I-002)** — 맵은 9/5 에 나왔다 · 마감 9/6
- [x] `webcam2.py` USB 캡처 + `calibrate_cam2_markers.py` 마커 캘리브레이션 도구 (PR #2) · 마감 9/6
- [x] `webcam_register_node.py`: YOLO track ID, 복장 클래스, `hsv_hist` 서술자 → `WorkerDescriptor` 발행 (PR #6) · 마감 9/7
- [ ] `webcam_register_node.py`: `worker.pt` 교체 후 **TS-U01** (등록 ≤ 10 s, 2명 별개 ID) · 마감 9/8
- [ ] `webcam_register_node.py`: 등록 구역 게이트 — 관제탑 위치 확정 후 · 마감 9/8
- [ ] **웹캠2 발행 주기 저하 (I-016)** — `webcam2.py` 에도 `BUFFERSIZE=1`·주기 분리·Compressed 적용 · 마감 9/7
- [ ] `webcam_hint_node.py`: 카메라·YOLO 연결, 트랙-작업자 대응, `TargetHint` 발행 · 마감 9/8
- [x] `kpi_logger_node.py`: CSV 초안 · 마감 9/8
- [x] `tools/kpi_report.py`: 추종거리·IPH·리드타임·성공률·재획득 계산 구현 (PR #8) · 마감 9/10
- [ ] **`kpi_report.py`: 배송 판정(AC-08)·리드타임을 `~/mro_fleet.db` 에서 읽기 (I-023)** — CSV 에는 T_lim 판정이 없어 실제보다 좋게 나온다. **TS-07 이 9/9 라 이것이 먼저** · 마감 9/8
- [ ] `kpi_report.py`: 추종거리를 FOLLOWING 구간으로 자르고 `|d−0.5|` p95 와 **최소 이격** 출력 (I-022, AC-05) · 마감 9/9
- [ ] `kpi_report.py`: Lost 를 `target/visible` 전이로 잡고 재획득 **성공률** 산출 — SEARCHING 은 정상 절차에도 쓰인다 (I-022, K-04) · 마감 9/9
- [ ] `kpi_report.py`: IPH 분모를 undock~도킹 완료 구간 합으로 (I-022, BRD 5.3) · 마감 9/9
- [x] `kpi_logger_node.py`: `target/distance` 구독(FR-29), 로봇 네임스페이스 파라미터화, QoS 계약 반영 (PR #8) · 마감 9/9

## mro_navigation [주행 — 박병후, 이시율]
- [x] 아레나 재구성 · 마감 9/5
- [x] SLAM 맵 재작성, `mro_bringup/maps/auto_map.*` 로 교체 · 마감 9/5
- [x] 실측 절차·양식 문서화 — `docs/measurement_guide.md` · 마감 9/5
- [x] **통신 구성** — offboard 단일 서버 PC-F `192.168.108.41` ID 1. 로봇 2대·PC 5대 접속 확인 · 마감 9/5
- [ ] 공유기 DHCP 예약 요청 — 지금은 재부팅하면 IP 가 바뀐다 · 마감 9/7
- [x] **`params/arena.yaml` 좌표 입력 (I-001)** — bays A/B/C, waiting_zone, dock_zone, `semaphore_zones` 폴리곤 실측 기입 (9/6) · 마감 9/6
- [x] **`dock_zone.pose` · `robotN.yaml` `dock_pose` 실측 기입 (I-013)** — 두 파일 값 일치 확인 (9/6) · 마감 9/6
- [ ] I-004 받침대 돌출폭 실측 — `measurement_guide.md` §2 · 마감 9/7
- [ ] TB4 2대 동시 운용 검증 · 마감 9/6
- [ ] **좌표 보정 3건 (I-001 PM 검증)** — 홀딩 포인트를 도크에서 ≥ 1.5 m 로, 베이 A 를 DOCK 폴리곤 밖으로, P1 폴리곤이 통로를 완전히 가로지르는지 확인 · 마감 9/7
- [ ] 베이 B·C waypoint 확정 실측 — 지금은 대강 값이라 ArUco 도착 판정(TS-U05·U06) 전에 필요 · 마감 9/7
- [ ] `arena.yaml` `homography_marks` 입력 — 웹캠 팀과 같은 바닥 지점을 쓴다 (I-002) · 마감 9/7
- [ ] `shuttle_mission_node.py`: `mission/assigned` 트리거, bay waypoint → ArUco 확인 → 체류 → 대기공간, zone_token, `shuttle/arrived` · 마감 9/7
- [ ] **`shuttle_mission_node.py`: `shuttle/status` 발행 (I-024)** — READY/SHUTTLE/AT_BAY/WAITING/FAILED. 지금은 발행자가 없어 배송 성공률이 구조적으로 0 % 다. **TS-07 이 9/9** · 마감 9/7
- [ ] `follow_target_node.py`: standoff 0.5, 힌트 방향 회전, STANDBY, paired 게이트, estop 존중 · 마감 9/8
- [x] `zone_semaphore_node.py`: P1·DOCK 토큰, FIFO, 타임아웃 회수 (관제 → 주행 이관) · 마감 9/8
- [x] `zone_semaphore_node.py`: `arena.yaml` 구역 폴리곤 좌표 입력 (9/6) · 마감 9/8
- [ ] `zone_semaphore_node.py`: 로봇 측 진입 판정 연결 (폴리곤 vs 로봇 위치 비교) · 마감 9/8
- [x] `safety_watchdog_node.py`: /scan → estop, cmd_vel 0 · 마감 9/8
- [ ] `safety_watchdog_node.py`: Nav2 goal cancel (액션 클라이언트), TS-14 결함 주입 검증 · 마감 9/8
- [ ] `safety_watchdog_node.py`: `HIT_FRAMES`·`CLEAR_FRAMES` 를 초 단위 파라미터로 (계약 시간 상수 규칙) · 마감 9/8
- [ ] 주석 정정 — `return_dock_node.py` 도크 간격 0.9 → 0.84, `zone_semaphore_node.py` P1 통과 방향(창고에서 나올 때·복귀) · 마감 9/7
- [x] `return_dock_node.py`: DOCK 토큰 + 재시도 2회 복귀 로직 · 마감 9/8
- [ ] `return_dock_node.py`: 홀딩 포인트·**도크 정면 접근 지점**으로 goToPose 후 dock (도크 간격 0.84 m 라 필수) · 마감 9/8
- [ ] `follow_target_node.py`: `mission/finish` 구독 → 즉시 페어링 해제·goal 취소 (≤ 1 s, FR-30) · 마감 9/8
- [ ] `inflation_radius` 0.22 실주행 확인 (I-009) — 지금 값은 계산일 뿐이다 · 마감 9/8
- [ ] 도킹 반복 시험 TS-U12 — 로봇별 10회, 오도킹 0 확인 · 마감 9/8

## mro_fleet [관제 — 김민준]
- [x] `core/db.py`, `config/schema.sql`, `fleet_manager_node.py` 기본 흐름, `control_ui_node.py` + `templates/index.html` · 마감 9/8
- [x] `control_ui`: 작업자별/전체 종료 버튼, `fleet_manager` RETURN_ALL · 마감 9/8
- [x] `fleet_manager_node.py`: yaw 계산, 미션 상태 전이 구독 (`shuttle/status` → Mission state) (PR #7) · 마감 9/8
- [x] `fleet_manager_node.py`: 배터리 25 % 미만 미션 거부·사유 기록 (FR-06) (PR #7) · 마감 9/8
- [x] `fleet_manager_node.py`: 리드타임 기록, T_lim 초과 실패 확정, `arrived_late` 별기 (FR-09) — 도착 시점 판정 + 1초 스윕 타이머 (PR #7) · 마감 9/8
- [ ] **`FleetStatus.state` 를 approach·shuttle 두 출처로 분리 (I-020)** — 지금은 한 필드를 두 노드가 덮어써 도킹 상태가 화면에 안 남는다 · 마감 9/7
- [ ] `fleet_manager_node.py`: `/fleet/intervention` 구독 → DB events 기록 (I-021, FR-28) · 마감 9/8
- [ ] `fleet_manager_node.py`: `WAITING` 수신 시 `amcl_pose` 가 `waiting_zone` 근처인지 확인, 벗어나면 `FAIL_WRONG_POSE` (계약 9.2) — 대기공간은 카메라·마커가 없어 로봇 자기 보고가 유일한 근거다 · 마감 9/8
- [ ] `fleet_manager_node.py`: 작업자-로봇 배정을 worker_id 정렬 순서로 (FR-03 '동시 검출 시 순서 재현') · 마감 9/8
- [ ] `control_ui`: 페어링 폴백(명시 페어링) 버튼 (FR-13) · 마감 9/8
- [x] `fleet_manager_node.py`: 주석의 옛 구역 토큰 이름 정리 (I-019) (PR #7) · 마감 9/9

## mro_bringup [PM 김병직]
- [x] robot / webcam_pc / fleet_pc launch, params 골격 · 마감 9/7
- [x] 설계도 4장 — `docs/diagrams/` (STEP 1·2·3-1·3-2) · 마감 9/8
- [ ] launch 가 `webcam.yaml`·`perception.yaml` 을 실제로 넘기도록 연결 · 마감 9/7
- [ ] 노드들이 params 를 실제로 읽도록 전환한 뒤 값 확정 · 마감 9/8
- [ ] 중복 파라미터 한 곳으로 — `reid_threshold`(perception.yaml ↔ robotN.yaml), `follow_standoff_m`·`t_lim_s`(navigation.yaml ↔ robotN.yaml), `t_lim_s`·`t_ret_s`·`dock_retry`(arena.yaml ↔ robotN.yaml). 지금은 노드가 안 읽어 무해하지만 값 확정 때 한쪽만 고치는 사고가 난다 · 마감 9/8
- [ ] SDD 부록 A rqt_graph 캡처 첨부 · 마감 9/10
