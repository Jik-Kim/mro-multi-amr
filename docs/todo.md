# Feature Todo — 파일·메서드 기준

각 줄 끝의 `마감 M/D` 는 스케줄러 Time Line 기준이다. 진행률은 아래 표가 체크박스를 세어 보여준다.
**이슈를 닫거나 상태를 바꿀 때는 여기 체크박스도 같이 본다** (`AGENTS.md` 「이슈 등록」 절).

<!-- STATS:BEGIN -->

**전체 56/96 완료** (████████████░░░░░░░░)  ·  기준 09/07

| 파트 | 완료 | 진행 | 지난 마감 |
|---|---|---|---|
| mro_interfaces [PM 김병직] | 3/3 | `██████████` | — |
| mro_perception [인지 — 정정인, 민범진] | 5/14 | `████░░░░░░` | — |
| mro_webcam [웹캠 — 한세교, 김학민] | 17/24 | `███████░░░` | — |
| mro_navigation [주행 — 박병후, 이시율] | 18/34 | `█████░░░░░` | — |
| mro_fleet [관제 — 김민준] | 9/13 | `███████░░░` | — |
| mro_bringup [PM 김병직] | 4/8 | `█████░░░░░` | — |

**오늘 마감 4건**

- mro_navigation — 공유기 DHCP 예약 요청 — 지금은 재부팅하면 IP 가 바뀐다
- mro_navigation — 측위 검증 마커 설치·실측 — ArUco ID 4 거치대(웹캠1 박스 북쪽 ≈ 0.4 m 앞 거치대, 북향, 10 cm, 높이 …
- mro_navigation — shuttle_mission_node.py: mission/assigned 트리거, bay waypoint → ArUco 확인 …
- mro_navigation — shuttle_mission_node.py: shuttle/status 발행 (I-024) — READY/SHUTTLE/AT_B…

> 이 표는 `python3 tools/todo_stats.py` 가 체크박스를 세어 다시 쓴다. 손으로 고치지 않는다.

<!-- STATS:END -->

## mro_interfaces [PM 김병직]
- [x] msg/srv 8종 정의 · 마감 9/6
- [x] **계약 v1.3 확정** — `docs/interfaces.md`. 구역 ID `P1`·`DOCK`, 확정 값 8건, 파라미터 단일 출처, 주기·시간 상수 규칙, 서술자 추출 영역 · 마감 9/6
- [x] `WorkerDescriptor.color_hist` — H16×S8=128, `reid.hsv_hist()` 공유 호출로 확정 · 마감 9/6

## mro_perception [인지 — 정정인, 민범진]
- [x] 인형 2종 제작 (상하의 전체, 9/6 결정) · 마감 9/6
- [x] 데이터 수집·라벨링 (9/6 완료) · 마감 9/6
- [x] `models/worker.pt` 학습·검증, 색 편향 점검 (I-003) — PR #19, yolo26s@640 · val mAP50-95 0.816 · 밝기 차 0.6 % (PM 체크포인트 대조 일치) · 마감 9/7
- [x] **배포 PC 에서 `tools/bench_yolo.py` 로 `worker.pt` 10 Hz 확인** (PR #22 최저 사양 RTX 3060 track max 42.7 ms · PR #26 PC-W1 max 40.5 ms — nano 불필요, 실부하는 통합 9/9) — max 가 100 ms 넘으면 nano(`yolo26n`@640) 로 교체. PM 노트북 CPU 는 track 9 ms (PR #19) · 마감 9/8
- [ ] **TS-U04 측면·후면** — 인형을 OAK-D 앞 1 m 에서 0°/90°/180°/270° 로 돌려 검출·`outfit_class` 유지 확인 (AC-10 근거, SDD 부록 B). 판정 기준은 SDD v0.10 · 마감 9/8
- [ ] `amr_detection_node.py`: `MODEL_PATH` → `worker.pt`, `DETECT_CLASS` → worker_a/b (이름으로 거른다, 인덱스 아님), **`ONLY_OUR_CAR` 흰차 필터 해제**, `/webcam1/worker_descriptor` 구독, `core/reid.gate` 적용 → `target/paired`, `target/worker_id` 발행 · 마감 9/9
- [ ] `amr_detection_node.py`: **`target/distance` 발행 (FR-29)** — 추종거리 판정의 기준값. kpi_logger 는 이미 구독 중이라 이것만 붙으면 AC-05 가 측정된다 · 마감 9/8
- [ ] **TS-U05 줄자 대조** — `target/distance` 의 정확도를 0.8~2 m 에서 사전 검증 (FR-34: 채점 수단의 오차 < 기준의 1/3). 이게 없으면 AC-05 를 그 값으로 채점할 근거가 없다 · 마감 9/8
- [ ] 시험 절차 — 대기공간 로봇이 서향이므로 S5 페어링 때 **RC카를 로봇 정면(서쪽)으로 돌아 들어오게** 조종 (FR-14 정면 1 m). 작업자는 P2(남쪽)로 온다 · 마감 9/9
- [ ] `amr_detection_node.py`: 모듈 상수 → ROS2 파라미터 (`config/perception.yaml`) · 마감 9/8
- [x] `aruco_detector_node.py`: 검출·거리 초안 · 마감 9/7
- [ ] `aruco_detector_node.py`: solvePnP 자세, 마커 ID 필터 · 마감 9/8
- [ ] `core/reid.py` 단위 테스트 (같은 인형 ≥ threshold, 다른 인형 < threshold) · 마감 9/8
- [ ] `reid_threshold` 확정 — 같은/다른 인형 유사도 분포를 그려 결정 (SOT Q-13). 추출 영역이 인형 전체라 다른 인형끼리도 0.5 안팎에서 시작한다 · 마감 9/8

## mro_webcam [웹캠 — 한세교, 김학민]
- [x] **새 아레나 호모그래피 (I-002)** — cam2 완료 (PR #9, AMCL 기준점 13개, RMSE 2.8 cm). **cam1 은 하지 않는다 (9/6)** — 웹캠1 은 등록만, map 좌표 없음 · 마감 9/6
- [x] `webcam2.py` USB 캡처 + `calibrate_cam2_markers.py` 마커 캘리브레이션 도구 (PR #2) · 마감 9/6
- [x] `webcam_register_node.py`: YOLO track ID, 복장 클래스, `hsv_hist` 서술자 → `WorkerDescriptor` 발행 (PR #6) · 마감 9/7
- [x] `webcam_register_node.py`: `worker.pt` 교체 후 **TS-U01** (등록 ≤ 10 s, 2명 별개 ID) — PR #26 실기동: 등록 0.90 s · W1/W2 별개 · 재등록 15/15 (체류 시간은 Q-26 todo 별도) · 마감 9/8
- [ ] `webcam_register_node.py`: `mean_hist()` 스케일 정정 — `hsv_hist` 는 L2 정규화(합 ≠ 128)라 "합을 원래대로" 주석이 틀렸다. 유사도는 Bhattacharyya 라 영향 없음(PM 수치 확인)이지만 발행되는 `color_hist` 척도가 AMR 과 달라지므로 평균 뒤 L2 정규화로 (PR #26 PM 노트) · 마감 9/8
- [ ] `webcam_hint_node.py`: `DEFAULT_MODEL_PATH` 를 `mro_perception/models/worker.pt` 로, `mro_webcam/models/best.pt` 사본 삭제 (단일 출처, PR #26 노트) · 마감 9/8
- [x] `webcam_register_node.py`: 헤더 주석 '관제센터 시야' → 출입구, `등록 구역 게이트` TODO 삭제 — 시야 전체가 등록 구역이라 게이트 불필요 (PR #16, Q-26, PR #20) · 마감 9/8
- [x] `webcam_register_node.py`: `webcam.yaml` 파라미터화 — `webcam_index`·`conf` (PR #26) (9/7 내장 카메라가 잡히고 conf 를 못 낮춰 두 번 막힘, PR #20 노트). yaml 의 `model_path: ""` 는 launch 가 덮으므로 그대로 두되 주석 정정 · 마감 9/8
- [ ] **웹캠1 출입구 체류 시간 실측** — 출발선이 화면 밖이라 작업자가 화면을 **가로지른다**. 등록에 `register_s` 1.0 + 검출 0.3 = **1.3 s** 이상 머물러야 하는데 9/7 로그 9개 구간 중 **3개가 미달**(0.60·0.80·1.11 s). 시연 통과 속도 확정 후 재측정, 미달이면 `register_s` 하향 (Q-26) · 마감 9/8
- [x] **웹캠2 발행 주기 저하 (I-016)** — 캡처 스레드·BUFFERSIZE·주기 분리·CompressedImage 를 `webcam_hint_node.py` 에 병합 (PR #14, 9/7 머지) · 마감 9/7
- [ ] `webcam_hint_node.py`: **YOLO 타이머 재진입 차단 (I-027)** — 개별 `MutuallyExclusiveCallbackGroup`, `read()` 실패 대기, `stop → join → release` 종료 순서 코드 완료 (PR #21, PM 확정 로직 단위 검사 통과); **실장비 검증** 후 닫는다 · 마감 9/8
- [ ] `webcam_hint_node.py`: `detection_confirm_count`(3 프레임) → **초 단위** `detection_confirm_s`(0.3) 로 — 계약 「시간 상수 표기」 규칙(프레임 수 금지, `webcam.yaml` 헤더). `process_hz` 를 바꾸면 의미가 조용히 달라진다 (PR #21 PM 노트) · 마감 9/8
- [ ] `webcam2.py`·`webcam2_subscriber.py` 정리 — 캡처가 힌트 노드로 옮겨져 같은 `/dev/video` 를 두 번 여는 잔재. 삭제하거나 subscriber 를 `CompressedImage` 뷰어로 바꾸고 `setup.py` 진입점 정리 · 마감 9/8
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

## mro_navigation [주행 — 박병후, 이시율]
- [x] 아레나 재구성 · 마감 9/5
- [x] SLAM 맵 재작성, `mro_bringup/maps/auto_map.*` 로 교체 · 마감 9/5
- [x] 실측 절차·양식 문서화 — `docs/measurement_guide.md` · 마감 9/5
- [x] **통신 구성** — offboard 단일 서버 PC-F `192.168.108.41` ID 1. 로봇 2대·PC 5대 접속 확인 · 마감 9/5
- [ ] 공유기 DHCP 예약 요청 — 지금은 재부팅하면 IP 가 바뀐다 · 마감 9/7
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
- [ ] `shuttle_mission_node.py`: `mission/assigned` 트리거, bay waypoint → ArUco 확인 → 체류 → 대기공간, zone_token, `shuttle/arrived` · 마감 9/7
- [ ] **베이 A·B 정차점 서쪽으로 ≥ 0.15 m 이동 또는 DOCK 폴리곤 확장 (I-025)** — 접근 지점과 0.31/0.38 m 라 몸통이 겹친다. TS-07 전 · 마감 9/8
- [ ] `return_dock_node.py`: 복귀 시 **P1 토큰 획득·반납** — 지금은 DOCK 토큰만 있다 (계약 3절: S10 도 P1 통과) · 마감 9/8
- [ ] **`shuttle_mission_node.py`: `shuttle/status` 발행 (I-024)** — READY/SHUTTLE/AT_BAY/WAITING/FAILED. 지금은 발행자가 없어 배송 성공률이 구조적으로 0 % 다. **`FAILED` 는 포기할 때만** — 재시도 중이면 SHUTTLE 유지 (관제가 FAILED 를 미션 실패로 기록한다, I-026). **TS-07 이 9/9** · 마감 9/7
- [ ] `follow_target_node.py`: standoff 0.5, 힌트 방향 회전, STANDBY, paired 게이트, estop 존중 · 마감 9/8
- [ ] **추종거리 분포 실측 (I-029)** — `xy_goal_tolerance` 0.25 고정이라 정지 시 오차 25 cm. TS-U07 로 `target/distance` p95·min 을 재고 미달이면 `cmd_vel` 미세 보정 구현 (PM: ③→② 순서, 기준 완화는 마지막) · 마감 9/8
- [ ] `follow_target_node.py`: 힌트 구독을 `/webcam2/target_hint`(**TargetHint, sensor_data QoS**) 로 — 지금은 옛 `webcam/car_map_point` PointStamped RELIABLE 이라 BEST_EFFORT 발행자(PR #14)의 힌트를 못 받는다 · 마감 9/8
- [x] `zone_semaphore_node.py`: P1·DOCK 토큰, FIFO, 타임아웃 회수 (관제 → 주행 이관) · 마감 9/8
- [x] `zone_semaphore_node.py`: `arena.yaml` 구역 폴리곤 좌표 입력 (9/6) · 마감 9/8
- [ ] `zone_semaphore_node.py`: 로봇 측 진입 판정 연결 (폴리곤 vs 로봇 위치 비교) · 마감 9/8
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
- [x] `core/db.py`, `config/schema.sql`, `fleet_manager_node.py` 기본 흐름, `control_ui_node.py` + `templates/index.html` · 마감 9/8
- [x] `control_ui`: 작업자별/전체 종료 버튼, `fleet_manager` RETURN_ALL · 마감 9/8
- [x] `fleet_manager_node.py`: yaw 계산, 미션 상태 전이 구독 (`shuttle/status` → Mission state) (PR #7) · 마감 9/8
- [x] `fleet_manager_node.py`: 배터리 25 % 미만 미션 거부·사유 기록 (FR-06) (PR #7) · 마감 9/8
- [x] `fleet_manager_node.py`: 리드타임 기록, T_lim 초과 실패 확정, `arrived_late` 별기 (FR-09) — 도착 시점 판정 + 1초 스윕 타이머 (PR #7) · 마감 9/8
- [x] **`FleetStatus.state` 를 approach·shuttle 두 출처로 분리 (I-020)** (PR #12) · 마감 9/7
- [x] **일시적 `FAILED` 가 미션을 굳히지 않게 (I-026)** — approach FAILED 는 미션 전이에서 빼고, FAILED 를 터미널에서 뺀다 (DONE 만 터미널) (PR #18, 9/7) · 마감 9/8
- [x] `fleet_manager_node.py`: `/fleet/intervention` 구독 → DB events 기록 (I-021, FR-28) (PR #23, PM 스텁 18/18) · 마감 9/8
- [ ] `fleet_manager_node.py`: `WAITING` 수신 시 `amcl_pose` 가 `waiting_zone` 근처인지 확인, 벗어나면 `FAIL_WRONG_POSE` (계약 9.2 ②) · 마감 9/8
- [ ] `fleet_manager_node.py`: `aruco/id`·`aruco/distance` 구독 → ID 4 수신 시 `|amcl_pose − loc_check_marker.xy|` 와 대조, ≤ 10 cm 면 `localization_verified` 이벤트, 아니면 `localization_suspect` (계약 9.2 ①) · 마감 9/8
- [ ] `fleet_manager_node.py`: 작업자-로봇 배정을 worker_id 정렬 순서로 (FR-03 '동시 검출 시 순서 재현') · 마감 9/8
- [ ] `control_ui`: 페어링 폴백(명시 페어링) 버튼 (FR-13) · 마감 9/8
- [x] `fleet_manager_node.py`: 주석의 옛 구역 토큰 이름 정리 (I-019) (PR #7) · 마감 9/9

## mro_bringup [PM 김병직]
- [x] robot / webcam_pc / fleet_pc launch, params 골격 · 마감 9/7
- [x] 설계도 4장 — `docs/diagrams/` (STEP 1·2·3-1·3-2) · 마감 9/8
- [x] launch 가 `webcam.yaml` 을 넘기도록 연결 — webcam1/webcam2 launch (9/7) · 마감 9/7
- [x] webcam1 launch 에서 안 쓰는 `homography_json` 제거 (계약 v1.8, PR #26 노트) · 마감 9/7
- [ ] robot.launch 가 `perception.yaml` 을 넘기도록 연결 — 인지 노드가 파라미터를 읽기 시작할 때 같이 · 마감 9/8
- [ ] 노드들이 params 를 실제로 읽도록 전환한 뒤 값 확정 · 마감 9/8
- [ ] 중복 파라미터 한 곳으로 — `reid_threshold`(perception.yaml ↔ robotN.yaml), `follow_standoff_m`·`t_lim_s`(navigation.yaml ↔ robotN.yaml), `t_lim_s`·`t_ret_s`·`dock_retry`(arena.yaml ↔ robotN.yaml). 지금은 노드가 안 읽어 무해하지만 값 확정 때 한쪽만 고치는 사고가 난다 · 마감 9/8
- [ ] SDD 부록 A rqt_graph 캡처 첨부 · 마감 9/10
