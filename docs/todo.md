# Feature Todo — 파일·메서드 기준

각 줄 끝의 `마감 M/D` 는 스케줄러 Time Line 기준이다. 진행률은 아래 표가 체크박스를 세어 보여준다.
**이슈를 닫거나 상태를 바꿀 때는 여기 체크박스도 같이 본다** (`AGENTS.md` 「이슈 등록」 절).

<!-- STATS:BEGIN -->

**전체 80/143 완료** (███████████░░░░░░░░░)  ·  기준 09/08

| 파트 | 완료 | 진행 | 지난 마감 |
|---|---|---|---|
| mro_interfaces [PM 김병직] | 6/6 | `██████████` | — |
| mro_perception [인지 — 정정인, 민범진] | 8/23 | `███░░░░░░░` | — |
| mro_webcam [웹캠 — 한세교, 김학민] | 21/28 | `████████░░` | — |
| mro_navigation [주행 — 박병후, 이시율] | 28/59 | `█████░░░░░` | **1** |
| mro_fleet [관제 — 김민준] | 10/16 | `██████░░░░` | — |
| mro_bringup [PM 김병직] | 7/11 | `██████░░░░` | — |

**마감이 지난 항목 1건**

- `9/7` mro_navigation — 측위 검증 마커 설치·실측 — ArUco ID 4 거치대(웹캠1 박스 북쪽 ≈ 0.4 m 앞 거치대, 북향, 10 cm, 높이 …

**오늘 마감 45건**

- mro_perception — TS-U04 측면·후면 — 인형을 OAK-D 앞 1 m 에서 0°/90°/180°/270° 로 돌려 검출·outfit_class…
- mro_perception — amr_detection_node.py: target/distance 발행 (FR-29) — 추종거리 판정의 기준값. kpi_l…
- mro_perception — TS-U05 줄자 대조 — target/distance 의 정확도를 0.8~2 m 에서 사전 검증 (FR-34: 채점 수단의 오…
- mro_perception — amr_detection_node.py: 모듈 상수 → ROS2 파라미터 (config/perception.yaml)
- mro_perception — core/reid.py 단위 테스트 (같은 인형 ≥ threshold, 다른 인형 < threshold)
- mro_perception — reid_threshold 확정 — 같은/다른 인형 유사도 분포를 그려 결정 (SOT Q-13). 추출 영역이 인형 전체라 다른…
- mro_webcam — 웹캠2 캡처 fps 실측 — webcam.yaml 의 webcam_hint.fps: 30.0 은 장치 사양 가정이고 실측 기록이…
- mro_webcam — 웹캠2 호모그래피 재검증 (I-002 · TS-U02) — 재캘리 완료(PR #32: LiDAR 중심 AMCL 13점, RMSE…
- mro_webcam — webcam_hint_node.py: DEFAULT_MODEL_PATH 를 mro_perception/models/worker.…
- mro_webcam — webcam_hint_node.py: YOLO 타이머 재진입 차단 (I-027) — 개별 MutuallyExclusiveCall…
- mro_webcam — webcam2.py 는 호모그래피 검증 도구로 유지 (PR #32: 클릭·수동 픽셀 → map 변환 → Nav2 goal. 프로…
- mro_webcam — webcam_hint_node.py: worker.pt 교체 후 TS-U02 — 힌트 map 좌표 vs 실측 ≤ 0.3 m, 지…
- mro_navigation — Nav2 복구 행동에서 후진(BackUp)을 뺀다 (계약 2절 후진 금지) — 우리 코드에는 linear.x < 0 이 없지만 …
- mro_navigation — safety_watchdog_node.py: estop 중 병진 전부 차단 · 저속 회전만 통과 · CLEAR_DIST 0.45…
- mro_navigation — shuttle_mission_node.py: 초기자세를 노드 기동 시로 (I-035) — 미션을 받은 뒤 보내면 Nav2 가 이…
- mro_navigation — test_follow_target_fsm.py 신규 (PR #39 후속) — 상태기계 검사 18개가 저장소에 없어 재현·회귀 검…
- mro_navigation — arena.yaml loc_check_marker.xy 실측 — 지금 [0.0, 0.0] 은 맵 원점이라 관제가 그대로 읽으면 …
- mro_navigation — inflation_radius 0.05 → 0.25 적용 (I-009) — 주행 합의값. 승인 후 PC-R3·PC-R4 의 na…
- mro_navigation — 베이 A·B 정차점 서쪽으로 ≥ 0.15 m 이동 또는 DOCK 폴리곤 확장 (I-025) — 접근 지점과 0.31/0.38 m…
- mro_navigation — estop 중 Nav2 복구 행동이 막히는 문제 (I-036) — 벽 앞에서 후진·회전까지 0 으로 덮여 95 초를 못 빠져나왔…
- mro_navigation — 도착 오차 수치 신뢰성 확인 (I-037) — 노드 TF 0.84 m vs 같은 순간 amcl_pose 0.18 m. 3회차에 …
- mro_navigation — P1 통과 실주행 확인 (I-009) — 여유 0.175 m < 실측 측위 오차 0.32 m 라 통과가 운에 좌우된다. infl…
- mro_navigation — Nav2 초기자세 선후 문제 해결 (I-035) — 초기자세 없이 뜬 Nav2 가 global_costmap 활성화에 실패하고 …
- mro_navigation — return_dock_node.py: 복귀 시 P1 토큰 획득·반납 — 지금은 DOCK 토큰만 있다 (계약 3절: S10 도 P…
- mro_navigation — follow_target_node.py: standoff 0.5, 힌트 방향 회전, STANDBY, paired 게이트, est…
- mro_navigation — AMCL update_min_d 0.05 실주행 확인 (I-029) — 9/7 저녁 적용(0.25→0.05). 도착 판정이 안정…
- mro_navigation — PC-R4 에 localization.yaml update_min_d: 0.05 복제 (I-029) — turtlebot4_ws…
- mro_navigation — 추종거리 분포 실측 (I-029) — xy_goal_tolerance 0.25 고정이라 정지 시 오차 25 cm. TS-U07 …
- mro_navigation — follow_target_node.py: 힌트 구독을 /webcam2/target_hint(TargetHint, sensor_d…
- mro_navigation — shuttle_mission_node.py: P1 대기 중 포기해도 반납을 부를 것 (I-033) — 거절당하면 세마포어 대기열…
- mro_navigation — shuttle_mission_node.py: 성공 직후 READY 를 내지 말 것 (I-034) — kpi_report 의 DO…
- mro_navigation — follow_target_node.py: 시작 신호 구독을 옛 goto_goal/arrived → shuttle/arrived …
- mro_navigation — 셔틀 실주행 1단계 완주 — 판정 7항목 (I-024 '확인하지 못한 것' ①)
- mro_navigation — safety_watchdog_node.py: Nav2 goal cancel (액션 클라이언트), TS-14 결함 주입 검증
- mro_navigation — safety_watchdog_node.py: HIT_FRAMES·CLEAR_FRAMES 를 초 단위 파라미터로 (계약 시간 상수…
- mro_navigation — return_dock_node.py: 홀딩 포인트·도크 정면 접근 지점으로 goToPose 후 dock (도크 간격 0.84 m…
- mro_navigation — follow_target_node.py: mission/finish 구독 → 즉시 페어링 해제·goal 취소 (≤ 1 s, FR…
- mro_navigation — inflation_radius 0.25 실주행 확인 (I-009) — P1 통과 · P3 받침대 안 긁는지. 두 PC 값 일치 …
- mro_fleet — fleet_manager_node.py: /fleet/zone_event 구독 → DB events kind ZONE 기록 (I…
- mro_fleet — fleet_manager_node.py: WAITING 수신 시 amcl_pose 가 waiting_zone 근처인지 확인, 벗…
- mro_fleet — fleet_manager_node.py: aruco/id·aruco/distance 구독 → ID 4 수신 시 |amcl_pos…
- mro_fleet — control_ui: 페어링 폴백(명시 페어링) 버튼 (FR-13)
- mro_bringup — v_max 0.3 m/s 와 Nav2 max_vel_x 0.26 충돌 정리 — 계약이 0.3 이라고 적었으니 Nav2 쪽을 맞춘…
- mro_bringup — 노드들이 params 를 실제로 읽도록 전환한 뒤 값 확정
- mro_bringup — 중복 파라미터 한 곳으로 — reid_threshold(perception.yaml ↔ robotN.yaml), follow_s…

> 이 표는 `python3 tools/todo_stats.py` 가 체크박스를 세어 다시 쓴다. 손으로 고치지 않는다.

<!-- STATS:END -->

## mro_interfaces [PM 김병직]
- [x] **계약 v1.13** — `aruco/pose` 비준(회전은 정렬에 쓰지 않는다) · ArUco 발행 한계 2.0 / 수용 조건 1.5 분리 · `cmd_vel` 에 `follow_target` 추가와 estop 전진만 차단 (I-036) · 마감 9/8
- [x] **SDD v0.13** — FR-13 확정(2.4·E-8·10장·부록 B), 셔틀 노드 개편과 I-033·I-034, K-06 완결·I-030 종결 반영. 노션 🫠 SDD 하위 페이지 생성 완료 · 마감 9/7
- [x] **계약 v1.12 — 명시 페어링 폴백(FR-13) 확정** (관제 8개 질문 답변) — `target/pair_state` 토픽, `srv/ManualPair`, 개입 종류 `MANUAL_PAIR` 추가. 규칙은 계약 10절 · 마감 9/7
- [x] msg/srv 8종 정의 · 마감 9/6
- [x] **계약 v1.3 확정** — `docs/interfaces.md`. 구역 ID `P1`·`DOCK`, 확정 값 8건, 파라미터 단일 출처, 주기·시간 상수 규칙, 서술자 추출 영역 · 마감 9/6
- [x] `WorkerDescriptor.color_hist` — H16×S8=128, `reid.hsv_hist()` 공유 호출로 확정 · 마감 9/6

## mro_perception [인지 — 정정인, 민범진]
- [x] **TS-U05 독립 점 1개 (I-031 마무리)** — 완료 9/8 (PR #43). 배율 모델이 1.55 m 에서 3.0 cm 틀려 **역깊이(시차 편향) 보정**으로 교체. 독립점 1.19 m 에서 −0.5 cm. PM 재적합 확인 — 배율은 어떻게 펴도 최선이 1.9~2.1 cm 라 모델 꼴 문제였다. **robot4 독립점은 카메라 올라온 뒤**
- [x] `core/aruco.py` ROS 비의존 — 완료 9/8 (PR #43). `depth_distance` 의 ROS import 를 뷰어 노드 전용 try/except 로. PM 확인: `AMENT_PREFIX_PATH` 빈 셸에서 `rclpy` 미적재 · 단위 시험 18/18
- [ ] **robot3 depth 가 RGB 보다 +83 ms 늦다 (I-038)** — ArUco 프레임 73 % 가 depth 대신 pnp 대체로 떨어진다. 값은 같은 자라 지금 오차는 없지만, 기동 직후 '못 봤다' 구간과 오래된 depth/pnp 비가 남는다. 큐 방식 짝짓기 · 마감 9/9
- [ ] **`perception.yaml` 에 k 검증 범위를 적는다 (PR #43 후속)** — k 를 편 범위는 보정값 0.843~1.565 m 인데 발행 한계는 2.0 m 다 (보정량 −9.2 cm, 미검증). 계약 6절 수용 1.5 m 는 검증 범위 안이라는 것까지 한 줄 · 마감 9/9
- [ ] **`calibrated_depth` 가 `depth_min` 게이트에 의존한다 (PR #43 후속)** — `z_raw` 로 나누므로 게이트가 없으면 0 원값에서 죽는다. docstring 에 명시 · 마감 9/9
- [ ] **robot4 TS-U05 독립 점 1개 (I-031)** — k 0.0076 은 2점 적합이라 자기 검증이다. 카메라 올라오면 한 점. 급하지 않다 (0.58 m 정차는 pnp 경로, k 영향 0.4 cm) · 마감 9/10
- [ ] `mro_perception` flake8 정리 — main 55건에서 PR #40 으로 80건. 대부분 D400·D403 docstring 형식과 99자 초과라 한 번에 정리된다. **아무도 이 시험을 안 돌리고 있다는 뜻** · 마감 9/9
- [ ] **`amr_detection_node.py`: 명시 페어링 폴백 서버 (FR-13, 계약 10절)** — ① `target/pair_state` 발행 (`PAIRED`/`GATE_FAIL`/`NO_TARGET`) ② 게이트 미달 30 s 계측 — **대상이 보이는데 못 넘는 경우만 센다**(`NO_TARGET` 은 세지 않음) ③ `/robotN/manual_pair` 서비스 서버 — 거부 사유 6종(10.5 표) ④ `mission/finish` 구독해 명시 페어링 해제 ⑤ 확정 시 `target/paired`·`target/worker_id` 발행은 **여기서만** · 마감 9/9
- [x] 인형 2종 제작 (상하의 전체, 9/6 결정) · 마감 9/6
- [x] 데이터 수집·라벨링 (9/6 완료) · 마감 9/6
- [x] `models/worker.pt` 학습·검증, 색 편향 점검 (I-003) — PR #19, yolo26s@640 · val mAP50-95 0.816 · 밝기 차 0.6 % (PM 체크포인트 대조 일치) · 마감 9/7
- [x] **배포 PC 에서 `tools/bench_yolo.py` 로 `worker.pt` 10 Hz 확인** (PR #22 최저 사양 RTX 3060 track max 42.7 ms · PR #26 PC-W1 max 40.5 ms — nano 불필요, 실부하는 통합 9/9) — max 가 100 ms 넘으면 nano(`yolo26n`@640) 로 교체. PM 노트북 CPU 는 track 9 ms (PR #19) · 마감 9/8
- [ ] **TS-U04 측면·후면** — 인형을 OAK-D 앞 1 m 에서 0°/90°/180°/270° 로 돌려 검출·`outfit_class` 유지 확인 (AC-10 근거, SDD 부록 B). 판정 기준은 SDD v0.10 · 마감 9/8
- [ ] `amr_detection_node.py`: `MODEL_PATH` → `worker.pt`, `DETECT_CLASS` → worker_a/b (이름으로 거른다, 인덱스 아님), **`ONLY_OUR_CAR` 흰차 필터 해제**, `/webcam1/worker_descriptor` 구독, `core/reid.gate` 적용 → `target/paired`, `target/worker_id` 발행 · 마감 9/9
- [ ] `amr_detection_node.py`: RGB·depth 구독 QoS → `qos_profile_sensor_data` (계약 8절 — 발행자가 BEST_EFFORT 면 영상이 한 장도 안 온다) · `camera_info` K 를 `core/camera.corrected_K` 로 보정 (`target/map_point` 좌우 1.76 배, I-031) — PR #30 후속, 교체 PR 에 같이 · 마감 9/9
- [ ] `amr_detection_node.py`: **`target/distance` 발행 (FR-29)** — 추종거리 판정의 기준값. kpi_logger 는 이미 구독 중이라 이것만 붙으면 AC-05 가 측정된다 · 마감 9/8
- [ ] **TS-U05 줄자 대조** — `target/distance` 의 정확도를 0.8~2 m 에서 사전 검증 (FR-34: 채점 수단의 오차 < 기준의 1/3). 이게 없으면 AC-05 를 그 값으로 채점할 근거가 없다. **같은 시험에서 `aruco/distance` 도 줄자와 대조해 `camera_fx_scale` 1.7778 을 확정한다 (I-031)** · 마감 9/8
- [ ] 시험 절차 — 대기공간 로봇이 서향이므로 S5 페어링 때 **RC카를 로봇 정면(서쪽)으로 돌아 들어오게** 조종 (FR-14 정면 1 m). 작업자는 P2(남쪽)로 온다 · 마감 9/9
- [ ] `amr_detection_node.py`: 모듈 상수 → ROS2 파라미터 (`config/perception.yaml`) · 마감 9/8
- [x] `aruco_detector_node.py`: 검출·거리 초안 · 마감 9/7
- [x] `aruco_detector_node.py`: solvePnP 자세, 마커 ID 필터 — PR #30: solvePnP(IPPE_SQUARE) · `marker_ids` 허용 목록 · 거리 게이트 1.5 m · `camera_info` fx 16/9 보정 (I-031, PM 합성 재현: 보정 후 오차 ≤ 1.9 cm, 보정 없이 1.35 → 0.77 m) · 마감 9/8
- [ ] `core/reid.py` 단위 테스트 (같은 인형 ≥ threshold, 다른 인형 < threshold) · 마감 9/8
- [ ] `reid_threshold` 확정 — 같은/다른 인형 유사도 분포를 그려 결정 (SOT Q-13). 추출 영역이 인형 전체라 다른 인형끼리도 0.5 안팎에서 시작한다 · 마감 9/8

## mro_webcam [웹캠 — 한세교, 김학민]
- [ ] **웹캠2 캡처 fps 실측** — `webcam.yaml` 의 `webcam_hint.fps: 30.0` 은 장치 사양 가정이고 실측 기록이 없다. 웹캠1(Jieli)은 9/6 에 재서 25 로 확정했다. 두 캠이 같은 모델이면 30 이 아니라 25 다. `v4l2-ctl --list-formats-ext` 로 확인하고 다른 값이면 yaml 을 고친다 — 설계도는 이 파일을 읽어 그리므로 다시 생성만 하면 따라온다. 계약 4절이 '장치 상한 그대로' 라 동작에는 영향이 없다 (캡처 ≥ 발행 10 Hz) · 마감 9/8
- [x] **새 아레나 호모그래피 (I-002)** — cam2 완료 (PR #9, AMCL 기준점 13개, RMSE 2.8 cm). **cam1 은 하지 않는다 (9/6)** — 웹캠1 은 등록만, map 좌표 없음 · 마감 9/6
- [ ] **웹캠2 호모그래피 재검증 (I-002 · TS-U02)** — 재캘리 완료(PR #32: LiDAR 중심 AMCL 13점, RMSE 2.8 cm, PM LOO 평균 4.0 cm·max 8.1 cm). 검증은 **held-out 3점 — 로봇을 새 지점에 세우고 `amcl_pose` vs 픽셀 변환 대조 (≤ 0.3 m)**. **Nav2 도착 위치로는 판정하지 않는다** — `xy_goal_tolerance` 0.25(I-029) 안의 15 cm 는 호모그래피 오차의 근거가 못 된다 (PM 9/7) · 마감 9/8
- [x] `webcam2.py` USB 캡처 + `calibrate_cam2_markers.py` 마커 캘리브레이션 도구 (PR #2) · 마감 9/6
- [x] `webcam_register_node.py`: YOLO track ID, 복장 클래스, `hsv_hist` 서술자 → `WorkerDescriptor` 발행 (PR #6) · 마감 9/7
- [x] `webcam_register_node.py`: `worker.pt` 교체 후 **TS-U01** (등록 ≤ 10 s, 2명 별개 ID) — PR #26 실기동: 등록 0.90 s · W1/W2 별개 · 재등록 15/15 (체류 시간은 Q-26 todo 별도) · 마감 9/8
- [x] `webcam_register_node.py`: `mean_hist()` 스케일 정정 — 평균 뒤 **L2 정규화**로 바꿨다(`hsv_hist` 가 `cv2.normalize` 기본값 NORM_L2 라 L2=1 이다). 종전에는 합을 128 로 맞춰 발행 `color_hist` 의 L2 가 **31.6** 이었다. 유사도는 Bhattacharyya 척도 불변이라 영향 없음을 실측 확인(차이 5e-09) — 오늘 낸 측정값은 그대로 유효하다 (PR #26 PM 노트) · 마감 9/8
- [ ] `webcam_hint_node.py`: `DEFAULT_MODEL_PATH` 를 `mro_perception/models/worker.pt` 로, `mro_webcam/models/best.pt` 사본 삭제 (단일 출처, PR #26 노트) · 마감 9/8
- [x] `webcam_register_node.py`: 헤더 주석 '관제센터 시야' → 출입구, `등록 구역 게이트` TODO 삭제 — 시야 전체가 등록 구역이라 게이트 불필요 (PR #16, Q-26, PR #20) · 마감 9/8
- [x] `webcam_register_node.py`: `webcam.yaml` 파라미터화 — `webcam_index`·`conf` (PR #26) (9/7 내장 카메라가 잡히고 conf 를 못 낮춰 두 번 막힘, PR #20 노트). yaml 의 `model_path: ""` 는 launch 가 덮으므로 그대로 두되 주석 정정 · 마감 9/8
- [x] **웹캠1 출입구 체류 시간 실측 (Q-26 해소)** — 실측 결과 문제 없다. 등록 없는 상태에서 시작해 **첫 통과에 두 명 다 등록**(0.90 / 0.91 s). 체류 구간 **18.99 · 16.30 s** 로 필요한 **1.3 s** 의 10배 이상 여유. 재등록 3/3(sim 0.71·0.75·0.77) · 마감 9/8
- [x] **웹캠2 발행 주기 저하 (I-016)** — 캡처 스레드·BUFFERSIZE·주기 분리·CompressedImage 를 `webcam_hint_node.py` 에 병합 (PR #14, 9/7 머지) · 마감 9/7
- [ ] `webcam_hint_node.py`: **YOLO 타이머 재진입 차단 (I-027)** — 개별 `MutuallyExclusiveCallbackGroup`, `read()` 실패 대기, `stop → join → release` 종료 순서 코드 완료 (PR #21, PM 확정 로직 단위 검사 통과); **실장비 검증** 후 닫는다 · 마감 9/8
- [x] `webcam_hint_node.py`: `detection_confirm_count`(3 프레임) → **초 단위** `detection_confirm_s`(0.3) 로 — 계약 「시간 상수 표기」 규칙(프레임 수 금지, `webcam.yaml` 헤더). `process_hz` 를 바꾸면 의미가 조용히 달라진다 (PR #21 PM 노트) — PR #32 (`detection_confirm_s` 0.3 s, `ceil(초 × process_hz)`) · 마감 9/8
- [ ] `webcam2.py` 는 **호모그래피 검증 도구로 유지** (PR #32: 클릭·수동 픽셀 → map 변환 → Nav2 goal. 프로덕션 launch 는 `webcam_hint_node` 만 띄운다) · `webcam2_subscriber.py` 는 삭제 (PM 결정 9/7, PR #33 질문 답) · 마감 9/8
- [x] `webcam_hint_node.py`: 카메라·YOLO 연결, 트랙-작업자 대응, `TargetHint` 발행 (PR #14) · 마감 9/8
- [ ] `webcam_hint_node.py`: `worker.pt` 교체 후 **TS-U02** — 힌트 map 좌표 vs 실측 ≤ 0.3 m, 지연 ≤ 0.2 s (손 흔들기 → 화면) · 마감 9/8
- [x] `kpi_logger_node.py`: CSV 초안 · 마감 9/8
- [x] `tools/kpi_report.py`: 추종거리·IPH·리드타임·성공률·재획득 계산 구현 (PR #8) · 마감 9/10
- [x] **`kpi_report.py`: 배송 판정(AC-08)·리드타임을 `~/mro_fleet.db` 에서 읽기 (I-023)** (PR #15, PM 합성 DB 대조 일치) · 마감 9/8
- [x] `kpi_report.py`: 추종거리를 FOLLOWING 구간으로 자르고 `|d−0.5|` p95 와 **최소 이격** 출력 (I-022, AC-05, PR #15) · 마감 9/9
- [x] `kpi_report.py`: Lost 를 `target/visible` 전이로 잡고 재획득 **성공률** 산출 — SEARCHING 은 정상 절차에도 쓰인다 (I-022, K-04, PR #15) · 마감 9/9
- [x] `kpi_report.py`: **K-04 이중 계상 수정 (I-028)** — 포기/성공/미복귀를 한 함수에서 판정 (PR #17, PM 합성 로그 재대조 일치) · 마감 9/9
- [x] `kpi_report.py`: `--db` 파일은 있는데 `missions` 테이블이 없으면 `OperationalError` 로 통째로 죽는다 — 잡아서 안내문으로 (관제 DB 초기화 전 TS-07 대비) (PR #20, PM 빈 DB 로 exit 0·K-04 계속 출력 확인) · 마감 9/8
- [x] `kpi_report.py`: IPH 분모를 undock~도킹 완료 구간 합으로 (I-022, BRD 5.3, PR #15) · 마감 9/9
- [x] `kpi_logger_node.py`: `target/distance` 구독(FR-29), 로봇 네임스페이스 파라미터화, QoS 계약 반영 (PR #8) · 마감 9/9
- [x] `kpi_logger_node.py`: `/fleet/zone_event` 구독 → CSV kind ZONE · `kpi_report.py` **K-06 교착 자동 해결률** 산출 (I-030, 계약 9절: REJECT→GRANT 60 s 이내, RECLAIM 은 미해결) — PR #33 (`zone_cb` CSV source=zone · `zone_resolution()`) + **PR #34 로 RECLAIM 을 구역 단위 판정으로 보정, I-032 해결** (PM 재대조 8 시나리오 일치 · 무관 로그 400건 차등 퍼즈 불일치 0) · 마감 9/9
- [ ] `kpi_report.py` K-06 마무리 셋 (PR #34 후속, 막지 않음) — ① `zone_resolution` docstring 의 `[REJECT, GRANT)` 를 양끝 포함으로 정정(안쪽 주석과 어긋남) ② `reclaimed_between(...): pass` 죽은 코드 삭제 ③ **`강제 회수` 라벨을 `강제 회수(구역)` 로** — 이제 구역 전체 RECLAIM 수라 대기자 없이 보유만 초과해도 세어져 `대기 1 · 자동 해결 1 · 강제 회수 1 → 100.0%` 로 읽힌다 (I-032) · 마감 9/9 TS-08 전

## mro_navigation [주행 — 박병후, 이시율]
- [ ] **P1 통과 — 측위가 유일한 경로다 (I-009 PM 판단 9/8, 정정)** — 통과 확실에 필요한 통로는 0.99 m 인데 교행 불가를 유지하려면 0.78 m 미만이어야 한다. **폭으로는 못 푼다.** 인플레이션을 최대로 낮춰도 12 cm(한쪽 6 cm)뿐이라 측위 오차 32 cm 앞에서 의미가 작다. **할 일은 I-029(`update_min_d` 0.05 실주행 확인)와 I-037(판정을 `amcl_pose` 로) 둘.** 목표는 오차 0.32 → 0.22 m 미만 · 마감 9/9
- [ ] (곁가지, 공짜면 한다) 받침대가 이미 통로 바깥인지 실측 확인 → 맞으면 `inflation_radius` 0.25 → 0.20 한 줄 변경. **12 cm 를 공짜로 얻지만 P1 을 풀지는 못한다** · 마감 9/9
- [ ] `Smac`·`MPPI` 교체와 지도 덧칠은 **하지 않는다** (9/8 PM) — 원인이 플래너도 지도도 아니라 측위다. Phase 2 후보로만 적어둔다 · 마감 —
- [ ] **Nav2 복구 행동에서 후진(`BackUp`)을 뺀다 (계약 2절 후진 금지)** — 우리 코드에는 `linear.x < 0` 이 없지만 **Nav2 기본 BT 의 복구가 후진을 낸다.** 로봇 PC 의 `turtlebot4_navigation/config/nav2.yaml` 과 BT xml 소관이라 저장소 밖이다 — `nav2.yaml`·`localization.yaml` 과 같은 종류의 문제로, PC-R3·R4 둘 다 손으로 맞추고 무엇을 바꿨는지 적어둔다. 제자리 회전(`Spin`)과 `Wait` 만 남긴다 · 마감 9/8
- [ ] **`safety_watchdog_node.py`: estop 중 병진 전부 차단 · 저속 회전만 통과 · `CLEAR_DIST` 0.45 → 0.38 (I-036, 계약 v1.13)** — **후진은 쓰지 않는다**(계약 2절 확정: 범퍼·클리프가 전방에만 있다). 각속도 상한 0.4 rad/s — 제자리 회전이 쓸고 가는 폭은 1.4 cm 뿐이고 유일한 탈출 수단이다. `CLEAR_DIST` 0.45 는 **P1 반폭 0.395 m 보다 커서 통로 안에서는 어느 방향으로 돌아도 해제가 안 된다.** Nav2 goal cancel 도 함께. **변경 뒤 TS-04·TS-14 재실행** — 해제가 너무 일러 재진입이 반복되지 않는지 특히 본다 · 마감 9/8
- [ ] **`shuttle_mission_node.py`: 초기자세를 노드 기동 시로 (I-035)** — 미션을 받은 뒤 보내면 Nav2 가 이미 활성화에 실패한 뒤다. `dock_pose`·`getDockedStatus()` 는 기동 시에도 안다. Nav2 가 정해진 횟수 안에 `active` 가 안 되면 **T_lim 을 태우지 말고 즉시 FAILED + 사유** · 마감 9/8
- [ ] **`test_follow_target_fsm.py` 신규 (PR #39 후속)** — 상태기계 검사 18개가 저장소에 없어 재현·회귀 검출이 안 된다. 셔틀의 `test_shuttle_mission_fsm.py` 와 같은 방식으로 · 마감 9/8
- [ ] **P1 통과 10회 연속 성공 (4단계 게이트, I-009)** — 측위 오차 0.32 m > 통과 여유 0.175 m 인 동안 2대 교행을 잡지 않는다. I-029·I-037 이 선행 · 마감 9/9
- [ ] `arena.yaml` `loc_check_marker.xy` 실측 — 지금 `[0.0, 0.0]` 은 맵 원점이라 관제가 그대로 읽으면 정상인데도 측위 의심으로 기록한다 (PR #39 지적) · 마감 9/8
- [x] 아레나 재구성 · 마감 9/5
- [x] SLAM 맵 재작성, `mro_bringup/maps/auto_map.*` 로 교체 · 마감 9/5
- [x] 실측 절차·양식 문서화 — `docs/measurement_guide.md` · 마감 9/5
- [x] **통신 구성** — offboard 단일 서버 PC-F `192.168.108.41` ID 1. 로봇 2대·PC 5대 접속 확인 · 마감 9/5
- [x] 공유기 DHCP 예약 요청 — **불필요로 종결 (9/7)**: PC-F 로 지정한 노트북은 학원 지원 장비라 재부팅해도 IP(`.41`)가 유지된다고 학원이 확답. 시연 전 `ip a` 로 `.41` 한 번만 확인 · 마감 9/7
- [x] **`params/arena.yaml` 좌표 입력 (I-001)** — bays A/B/C, waiting_zone, dock_zone, `semaphore_zones` 폴리곤 실측 기입 (9/6) · 마감 9/6
- [x] **`dock_zone.pose` · `robotN.yaml` `dock_pose` 실측 기입 (I-013)** — 두 파일 값 일치 확인 (9/6) · 마감 9/6
- [x] I-004 받침대 돌출폭 실측 (9/6) — P1 없음 · P2 양쪽 3 cm(폭 0.86 정정) · P3 입구 5 cm·안쪽 6.5 cm · 마감 9/7
- [ ] **`inflation_radius` 0.05 → 0.25 적용 (I-009)** — 주행 합의값. 승인 후 PC-R3·PC-R4 의 `nav2.yaml` 양쪽에 같은 값으로 · 마감 9/8
- [ ] **측위 검증 마커 설치·실측** — ArUco ID 4 거치대(웹캠1 박스 북쪽 ≈ 0.4 m 앞 거치대, 북향, 10 cm, 높이 23.5 cm), map 좌표를 `arena.yaml` `loc_check_marker.xy` 에 (계약 6절·9.2 ①) · 마감 9/7
- [x] **I-013 검증 (9/7)** — ArUco 미설치라 도킹 상태 `amcl_pose` 비교로 대체. 밀림 방향이 둘 다 −x 로 같고 차이 5.1 cm, 도크 간격 측정 0.803 대 줄자 0.840 m → **map 프레임 상호 일치 확인**. 개별 20 cm 치우침은 `xy_goal_tolerance` 0.25 에서 오는 것 · 마감 9/7
- [x] TB4 2대 동시 운용 검증 (9/6) — 네임스페이스 격리 · `odom` 20 Hz · OAK-D 동시 스트리밍 · RPi CPU 21~31 % · **Nav2 2벌 동시 `active`** 확인 (I-011) · 마감 9/6
- [x] **좌표 보정 3건 (I-001 PM 검증)** — holding ≥1.5m(1.52m), bays 전부 DOCK 폴리곤 밖, P1 폴리곤이 통로를 완전히 가로지르도록 재실측 (9/6) · 마감 9/7
- [x] 베이 B·C waypoint 확정 실측 (9/6, x=-0.78 로 통일) · 마감 9/7
- [x] `arena.yaml` `homography_marks` — cam2 기준점은 `mro_webcam/config/WebCam2/marker_coordinates.json` 이 원본(PR #9), cam1 은 불필요(9/6). arena.yaml 에는 복사하지 않는다 · 마감 9/7
- [x] `shuttle_mission_node.py`: `mission/assigned` 트리거, bay waypoint → ArUco 확인 → 체류 → 대기공간, zone_token, `shuttle/arrived` — `aruco/id` 는 **1.5 m 안에서만** 나온다 (PR #30 거리 게이트) · 마감 9/7
- [ ] **베이 A·B 정차점 서쪽으로 ≥ 0.15 m 이동 또는 DOCK 폴리곤 확장 (I-025)** — 접근 지점과 0.31/0.38 m 라 몸통이 겹친다. TS-07 전 · 마감 9/8
- [ ] **estop 중 Nav2 복구 행동이 막히는 문제 (I-036)** — 벽 앞에서 후진·회전까지 0 으로 덮여 95 초를 못 빠져나왔다. 후보 ①전진 성분만 차단 ②N초 후 워치독이 후진 ③goal cancel — **PM·안전 검토 필요** · 마감 9/8
- [ ] **도착 오차 수치 신뢰성 확인 (I-037)** — 노드 TF 0.84 m vs 같은 순간 `amcl_pose` 0.18 m. 3회차에 `tf2_echo` 기록 + **베이에서 줄자 실측**으로 가른다 · 마감 9/8
- [ ] **P1 통과 실주행 확인 (I-009)** — 여유 0.175 m < 실측 측위 오차 0.32 m 라 통과가 운에 좌우된다. `inflation_radius` 0.15 실험 후 결정. **PC-R3 는 0.25 적용 확인됨, PC-R4 미확인** · 마감 9/8
- [ ] **Nav2 초기자세 선후 문제 해결 (I-035)** — 초기자세 없이 뜬 Nav2 가 `global_costmap` 활성화에 실패하고 재시도하지 않아 셔틀이 T_lim 을 날린다. 조치 후보 ①절차 ②노드가 RESUME ③`t0` 이동 — **PM 판단 필요** · 마감 9/8
- [ ] `return_dock_node.py`: 복귀 시 **P1 토큰 획득·반납** — 지금은 DOCK 토큰만 있다 (계약 3절: S10 도 P1 통과) · 마감 9/8
- [x] **`shuttle_mission_node.py`: `shuttle/status` 발행 (I-024)** — READY/SHUTTLE/AT_BAY/WAITING/FAILED. 지금은 발행자가 없어 배송 성공률이 구조적으로 0 % 다. **`FAILED` 는 포기할 때만** — 재시도 중이면 SHUTTLE 유지 (관제가 FAILED 를 미션 실패로 기록한다, I-026). **TS-07 이 9/9** · 마감 9/7
- [ ] `follow_target_node.py`: standoff 0.5, 힌트 방향 회전, STANDBY, paired 게이트, estop 존중 · 마감 9/8
- [ ] **AMCL `update_min_d` 0.05 실주행 확인 (I-029)** — 9/7 저녁 적용(0.25→0.05). 도착 판정이 안정되는지, 리샘플링이 잦아져 pose 가 더 떨리지는 않는지. 떨리면 `resample_interval` 2로 · 마감 9/8
- [ ] **PC-R4 에 `localization.yaml` `update_min_d: 0.05` 복제 (I-029)** — `turtlebot4_ws` 는 git 밖이라 PC 마다 손으로. I-009 의 `nav2.yaml` 과 같이 확인 · 마감 9/8
- [ ] **추종거리 분포 실측 (I-029)** — `xy_goal_tolerance` 0.25 고정이라 정지 시 오차 25 cm. TS-U07 로 `target/distance` p95·min 을 재고 미달이면 `cmd_vel` 미세 보정 구현 (PM: ③→② 순서, 기준 완화는 마지막) · 마감 9/8
- [ ] `follow_target_node.py`: 힌트 구독을 `/webcam2/target_hint`(**TargetHint, sensor_data QoS**) 로 — 지금은 옛 `webcam/car_map_point` PointStamped RELIABLE 이라 BEST_EFFORT 발행자(PR #14)의 힌트를 못 받는다 · 마감 9/8
- [ ] **`shuttle_mission_node.py`: P1 대기 중 포기해도 반납을 부를 것 (I-033)** — 거절당하면 세마포어 대기열에 들어가는데 `token_held` 가 False 라 `release_token` 이 그냥 돌아온다. T_lim 으로 끊기면 유령 대기열이 남아 **P1 이 영구히 잠기고 watchdog 도 못 푼다**. `token_requested` 플래그 추가. 거절만 반복하다 T_lim 으로 끝나는 단위 시험도 함께 · 마감 9/8
- [ ] **`shuttle_mission_node.py`: 성공 직후 `READY` 를 내지 말 것 (I-034)** — `kpi_report` 의 `DOCKED_STATES` 에 `READY` 가 있어 AC-13 운용시간이 대기공간에서 끊긴다. 한 사이클 재현에서 분모 390 s → 150 s, IPH 27.7 → 72.0 회/h (K-07 목표 ≤ 1.0). 성공 경로는 `WAITING` 유지, 포기 경로의 `FAILED`→`READY` 는 그대로 · 마감 9/8
- [ ] `follow_target_node.py`: 시작 신호 구독을 옛 `goto_goal/arrived` → **`shuttle/arrived`** 로 (계약 4절, 옛 토픽은 '쓰지 않는 옛 토픽' 목록). `shuttle_mission` 이 계약 이름으로 발행하도록 바뀌면 이걸 안 고치는 순간 S5 합류가 시작되지 않는다 · 마감 9/8
- [x] **셔틀 실주행 시험 절차 문서화** — `docs/shuttle_test_guide.md`. 단계별 설계, 1단계 절차, 로그 해석·증상별 판별. `fleet_manager`·마커 없이 주행 골격만 떼어 확인하는 방법과 **그래서 무엇이 확인되지 않는지**를 같이 적었다 · 마감 9/8
- [x] **셔틀 시험 환경 스크립트** — `source tools/shuttle_env.sh [build|nav2]`. source 4종 + 빌드 + T3 전용 `ROS_SUPER_CLIENT=False` 를 한 줄로. 9/7 실패 1번(디스커버리 설정 누락)이 여기서 사라진다 · 마감 9/8
- [x] **셔틀 시험 사전 점검 스크립트** — `tools/shuttle_preflight.sh [pre|nav|mission]`. `is_docked` 실측·중복 노드·**네임스페이스 없는 스택·유령 노드**·`bt_navigator active`·토큰 서비스·마커 주기·미션 구독을 한 번에 본다. 9/7 실패 세 번이 전부 '환경을 확인하지 않아서' 였다 · 마감 9/8
- [x] **셔틀 1단계 시험 리그 launch** — `mro_navigation/launch/shuttle_step1_rig.launch.py`. 옛 T4·T5·T6(세마포어·안전감시·마커주입)을 한 터미널로. 셔틀·Nav2 는 로그 관찰 대상이라 묶지 않는다 · 마감 9/8
- [ ] **셔틀 실주행 1단계 완주** — 판정 7항목 (I-024 '확인하지 못한 것' ①) · 마감 9/8
- [x] `zone_semaphore_node.py`: P1·DOCK 토큰, FIFO, 타임아웃 회수 (관제 → 주행 이관) · 마감 9/8
- [x] `zone_semaphore_node.py`: `arena.yaml` 구역 폴리곤 좌표 입력 (9/6) · 마감 9/8
- [x] `zone_semaphore_node.py`: 진입 판정은 **순서 기반(방법 B)으로 확정** — 폴리곤 계산 안 함, 폴리곤은 로그 검산용 (PM 결정 9/7) · 마감 9/8
- [x] `zone_semaphore_node.py`: 반납 시 대기열 누락으로 구역이 영구 차단되는 버그 수정 (9/7) · 마감 9/8
- [x] **`zone_semaphore_node.py`: `/fleet/zone_event` 발행 (I-030, 계약 v1.11)** — `"<zone> <robot> <REJECT|GRANT|RELEASE|RECLAIM>"`. REJECT 는 대기 시작 1건만, GRANT 는 새 획득만 · 마감 9/8
- [x] `safety_watchdog_node.py`: /scan → estop, cmd_vel 0 · 마감 9/8
- [ ] `safety_watchdog_node.py`: Nav2 goal cancel (액션 클라이언트), TS-14 결함 주입 검증 · 마감 9/8
- [ ] `safety_watchdog_node.py`: `HIT_FRAMES`·`CLEAR_FRAMES` 를 초 단위 파라미터로 (계약 시간 상수 규칙) · 마감 9/8
- [x] 주석 정정 (9/7) — `return_dock_node.py` 도크 간격 0.84·IR 차폐 폐지 명시, `zone_semaphore_node.py` P1 통과 방향(창고에서 나올 때·복귀)·P3 폭 0.93 · 마감 9/7
- [x] `return_dock_node.py`: DOCK 토큰 + 재시도 2회 복귀 로직 · 마감 9/8
- [ ] `return_dock_node.py`: 홀딩 포인트·**도크 정면 접근 지점**으로 goToPose 후 dock (도크 간격 0.84 m 라 필수) · 마감 9/8
- [ ] `follow_target_node.py`: `mission/finish` 구독 → 즉시 페어링 해제·goal 취소 (≤ 1 s, FR-30) · 마감 9/8
- [ ] `inflation_radius` **0.25** 실주행 확인 (I-009) — P1 통과 · P3 받침대 안 긁는지. 두 PC 값 일치 확인 · 마감 9/8
- [x] 도킹 반복 시험 TS-U12 (9/6) — 로봇별 10회, 차폐판 없이 오도킹 0 확인 (I-006 종결) · 마감 9/8

## mro_fleet [관제 — 김민준]
- [ ] **FR-35 는 `loc_check_marker.xy` 실측 뒤에 켠다** — `[0.0, 0.0]` 인 채로 읽으면 측위가 정상인데도 `localization_suspect` 로 기록된다. 그리고 **`aruco/distance`(카메라 기준)와 `|amcl_pose − xy|`(base_link 기준)의 6 cm 오프셋을 빼고 비교**한다 (TF 실측, `tolerance_m` 0.10 의 60 %) · 마감 9/9
- [ ] **`control_ui`: 명시 페어링 폴백 버튼 (FR-13, 계약 10절)** — `target/pair_state = GATE_FAIL` 이면 버튼 표시, 운영자가 로봇·작업자 선택 → `fleet_manager` 가 `/robotN/manual_pair` 호출, 응답의 `reason` 을 화면에 그대로 보여준다. 성공 시 `/fleet/intervention` 에 **`kind = MANUAL_PAIR`** 발행 — IPH 에 들어간다(10.7) · 마감 9/9
- [x] `core/db.py`, `config/schema.sql`, `fleet_manager_node.py` 기본 흐름, `control_ui_node.py` + `templates/index.html` · 마감 9/8
- [x] `control_ui`: 작업자별/전체 종료 버튼, `fleet_manager` RETURN_ALL · 마감 9/8
- [x] `fleet_manager_node.py`: yaw 계산, 미션 상태 전이 구독 (`shuttle/status` → Mission state) (PR #7) · 마감 9/8
- [x] `fleet_manager_node.py`: 배터리 25 % 미만 미션 거부·사유 기록 (FR-06) (PR #7) · 마감 9/8
- [x] `fleet_manager_node.py`: 리드타임 기록, T_lim 초과 실패 확정, `arrived_late` 별기 (FR-09) — 도착 시점 판정 + 1초 스윕 타이머 (PR #7) · 마감 9/8
- [x] **`FleetStatus.state` 를 approach·shuttle 두 출처로 분리 (I-020)** (PR #12) · 마감 9/7
- [x] **일시적 `FAILED` 가 미션을 굳히지 않게 (I-026)** — approach FAILED 는 미션 전이에서 빼고, FAILED 를 터미널에서 뺀다 (DONE 만 터미널) (PR #18, 9/7) · 마감 9/8
- [x] `fleet_manager_node.py`: `/fleet/intervention` 구독 → DB events 기록 (I-021, FR-28) (PR #23, PM 스텁 18/18) · 마감 9/8
- [ ] `fleet_manager_node.py`: `/fleet/zone_event` 구독 → DB `events` kind `ZONE` 기록 (I-030, 계약 v1.11 §7) · 마감 9/8
- [ ] `fleet_manager_node.py`: `WAITING` 수신 시 `amcl_pose` 가 `waiting_zone` 근처인지 확인, 벗어나면 `FAIL_WRONG_POSE` (계약 9.2 ②) · 마감 9/8
- [ ] `fleet_manager_node.py`: `aruco/id`·`aruco/distance` 구독 → ID 4 수신 시 `|amcl_pose − loc_check_marker.xy|` 와 대조, ≤ 10 cm 면 `localization_verified` 이벤트, 아니면 `localization_suspect` (계약 9.2 ①). **기준점 주의:** `aruco/distance` 는 카메라→마커, `\|amcl_pose − xy\|` 는 base_link→마커 — `tf2_echo base_link oakd_rgb_camera_optical_frame` 으로 오프셋을 재서 뺀다 (I-031) · 마감 9/8
- [x] `fleet_manager_node.py`: 작업자-로봇 배정을 worker_id 정렬 순서로 (FR-03 '동시 검출 시 순서 재현') — 1 s 배치 창 + `worker_id` 정렬, W1→robot3·W2→robot4 로 순서 무관 재현 (PR #28, PM 스텁 24/24) · 마감 9/8
- [ ] `control_ui`: 페어링 폴백(명시 페어링) 버튼 (FR-13) · 마감 9/8
- [x] `fleet_manager_node.py`: 주석의 옛 구역 토큰 이름 정리 (I-019) (PR #7) · 마감 9/9

## mro_bringup [PM 김병직]
- [ ] `v_max` 0.3 m/s 와 Nav2 `max_vel_x` 0.26 충돌 정리 — 계약이 0.3 이라고 적었으니 Nav2 쪽을 맞춘다 (PR #39 TODO ②) · 마감 9/8
- [x] **셔틀 파라미터 7개를 `robotN.yaml` 로, `arena_yaml` 경로를 `robot.launch.py` 로** (PR #36 주행 요청, 계약 5절 단일 출처) — `bay_advance_m`·`aruco_max_dist_m`·`aruco_valid_s`·`aruco_confirm_s`·`aruco_retry`·`token_retry_s`·`nav2_poll_s`. `arena.yaml` 은 데이터 yaml 이라 파라미터 파일로 못 넘겨 경로만 준다. 노드의 런타임 조회 대체값은 단독 실행용으로 그대로 둔다 · 마감 9/8
- [x] robot / webcam_pc / fleet_pc launch, params 골격 · 마감 9/7
- [x] 설계도 4장 — `docs/diagrams/` (STEP 1·2·3-1·3-2) · 마감 9/8
- [x] launch 가 `webcam.yaml` 을 넘기도록 연결 — webcam1/webcam2 launch (9/7) · 마감 9/7
- [x] webcam1 launch 에서 안 쓰는 `homography_json` 제거 (계약 v1.8, PR #26 노트) · 마감 9/7
- [x] 계약 v1.11 — 세마포어 이벤트 토픽 `/fleet/zone_event`·DB kind ZONE·K-06 산출 규칙 (I-030 PM 결정) · 마감 9/7
- [x] robot.launch 가 `perception.yaml` 을 넘기도록 연결 — 9/7, `aruco_detector` 가 PR #30 부터 파라미터를 읽어서 앞당김 (amr_detection·aruco 두 노드에만, robotN.yaml 뒤) · 마감 9/8
- [ ] 노드들이 params 를 실제로 읽도록 전환한 뒤 값 확정 · 마감 9/8
- [ ] 중복 파라미터 한 곳으로 — `reid_threshold`(perception.yaml ↔ robotN.yaml), `follow_standoff_m`·`t_lim_s`(navigation.yaml ↔ robotN.yaml), `t_lim_s`·`t_ret_s`·`dock_retry`(arena.yaml ↔ robotN.yaml). 지금은 노드가 안 읽어 무해하지만 값 확정 때 한쪽만 고치는 사고가 난다 · 마감 9/8
- [ ] SDD 부록 A rqt_graph 캡처 첨부 · 마감 9/10
