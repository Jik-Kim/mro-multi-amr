# Feature Todo — 파일·메서드 기준 (스케줄러 Time Line 과 대응)

## mro_interfaces [PM 김병직]
- [x] msg/srv 8종 정의
- [x] **계약 v1.0 확정 (9/5)** — `docs/interfaces.md`. 구역 ID 를 실제 아레나에 맞게 `P1`·`DOCK` 로 정정, 확정 값 6건 명시, 파라미터 단일 출처 지정
- [x] `WorkerDescriptor.color_hist` — H16×S8=128, `reid.hsv_hist()` 공유 호출로 확정

## mro_perception [인지 — 정정인, 민범진]
- [ ] 인형 2종 제작(9/5) → 데이터 수집·라벨링(9/6) → `models/worker.pt` 학습 (9/6 밤 시작, 9/7 완료)
- [ ] `amr_detection_node.py`: `DETECT_CLASS` → worker_a/b, `/webcam1/worker_descriptor` 구독, `core/reid.gate` 적용 → `target/paired`, `target/worker_id` 발행
- [ ] `amr_detection_node.py`: 모듈 상수 → ROS2 파라미터 (`config/perception.yaml`)
- [x] `aruco_detector_node.py`: 검출·거리 초안
- [ ] `aruco_detector_node.py`: solvePnP 자세, 마커 ID 필터
- [ ] `core/reid.py` 단위 테스트 (같은 인형 ≥ threshold, 다른 인형 < threshold)

## mro_webcam [웹캠 — 한세교, 김학민]
- [ ] 새 아레나 호모그래피 cam1/cam2 캘리브레이션 (9/5, 맵 완성 직후)
- [ ] `webcam_register_node.py`: YOLO track ID, 복장 클래스, `hsv_hist` 서술자 → `WorkerDescriptor` 발행 (≤ 10 s, 2명 별개)
- [ ] `webcam_hint_node.py`: 카메라·YOLO 연결, 트랙-작업자 대응, `TargetHint` 발행
- [x] `kpi_logger_node.py`: CSV 초안
- [ ] `tools/kpi_report.py`: p95 추종거리, IPH, 리드타임

## mro_navigation [주행 — 박병후, 이시율]
- [x] 아레나 재구성 (Z-01~Z-09 배치) (9/5)
- [x] SLAM 맵 재작성 (9/5)
- [x] 새 맵 파일을 `mro_bringup/maps/auto_map.pgm`·`.yaml` 로 교체 (9/5, `arena0905.*` 원본 반영)
- [ ] `params/arena.yaml` 좌표 입력 — RViz Publish Point 로 bays A/B/C, waiting_zone, semaphore_zones(P1/DOCK) 폴리곤, homography_marks (I-001)
- [ ] **`dock_zone.pose` · `robotN.yaml` `dock_pose` 실측 기입** — 맵 좌표계 통일의 전제 (I-013)
- [x] **통신 구성 완료 (9/5)** — offboard 단일 서버 PC-F `192.168.108.41` ID 1. 로봇 2대·PC 5대 전부 접속 확인. 남은 것: 공유기 예약 요청
- [ ] TB4 2대 동시 운용 검증 (9/6) → SOT Q-18
- [ ] `shuttle_mission_node.py`: `mission/assigned` 트리거, bay waypoint → ArUco 확인 → 체류 → Z-05, zone_token, `shuttle/arrived`
- [ ] `follow_target_node.py`: standoff 0.5, 힌트 방향 회전, STANDBY, paired 게이트, estop 존중
- [x] `zone_semaphore_node.py`: Z-05/08/09/03 토큰, FIFO, 타임아웃 회수 (**관제 → 주행 이관 9/5**)
- [ ] `zone_semaphore_node.py`: `arena.yaml` 구역 폴리곤 좌표 입력 + 로봇 측 진입 판정 연결
- [x] `safety_watchdog_node.py`: /scan → estop, cmd_vel 0
- [ ] `safety_watchdog_node.py`: Nav2 goal cancel (액션 클라이언트), TS-14 결함 주입 검증
- [x] `return_dock_node.py`: Z-03 도크 토큰 + 재시도 2회 복귀 로직
- [ ] `return_dock_node.py`: arena.yaml 의 홀딩 포인트·**도크 정면 접근 지점**으로 goToPose 후 dock (도크 간격 0.9 m 라 필수)
- [ ] 도킹 반복 시험 TS-U12 (9/5): 로봇별 10회, 오도킹 0 확인. 실패 시 도크 간격 확대 또는 IR 차폐 추가
- [ ] `follow_target_node.py`: `mission/finish` 구독 → 즉시 페어링 해제·goal 취소 (≤ 1 s, FR-30)

## mro_fleet [관제 — 김민준]
- [x] `core/db.py`, `config/schema.sql`, `fleet_manager_node.py` 기본 흐름, `control_ui_node.py` + `templates/index.html`
- [ ] `fleet_manager_node.py`: yaw 계산, 미션 상태 전이 구독(shuttle/status → Mission state)
- [x] `control_ui`: 작업자별/전체 종료 버튼, `fleet_manager` RETURN_ALL
- [ ] `control_ui`: 페어링 폴백(명시 페어링) 버튼

## mro_bringup [PM 김병직]
- [x] robot / webcam_pc / fleet_pc launch, params 골격
- [ ] 노드들이 params 를 실제로 읽도록 전환한 뒤 값 확정
