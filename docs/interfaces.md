# Interfaces

정의 원본은 `ros2_ws/src/mro_interfaces`. 이 문서는 의도·규칙·미정 사항을 설명한다.

## 메시지·서비스 (mro_interfaces)

| 타입 | 용도 | 비고 |
|---|---|---|
| `msg/WorkerDescriptor` | 웹캠1 등록 결과 (worker_id, outfit_class, color_hist, robot_ns) | `color_hist` 는 `mro_perception.core.reid.hsv_hist` 와 동일 bin (16×8=128) **[합의 필요]** |
| `msg/Mission` | 미션 상태 (CREATED…FAILED 상수) | 상태 전이는 fleet_manager 가 DB 에 기록 |
| `msg/FleetStatus` | 로봇별 집계 상태, 1 Hz | BR-T08 ≤ 2 s |
| `msg/TargetHint` | 웹캠2 → 대상 map 좌표 | 안전 판단 금지 |
| `msg/Intervention` | 개입 이벤트 | `SAFETY_POLICY` 는 개입으로 계상하지 않음 (BRD 5.3) |
| `srv/RequestMission` | UI → fleet | 접수 ≤ 2 s |
| `srv/ZoneToken` | 로봇 → zone_semaphore | acquire/release, FIFO |
| `srv/OperatorCommand` | UI → fleet | **RETURN 만** (작업 종료→복귀). 정지·재개는 9/4 결정으로 제외 |

## 토픽 계약

전역(관제·웹캠)은 절대 이름, 로봇 노드는 **상대 이름**(launch 가 `namespace:=/robotN` 을 붙인다).

| 토픽 | 송신 | 수신 | 타입 | 비고 |
|---|---|---|---|---|
| `/webcam1/worker_descriptor` | webcam_register | fleet_manager, amr_detection, webcam_hint | WorkerDescriptor | S1 |
| `/webcam2/target_hint` | webcam_hint | follow_target, kpi_logger | TargetHint | E-1 |
| `/fleet/mission` | fleet_manager | kpi_logger, UI | Mission | 전이마다 |
| `/fleet/status` | fleet_manager | control_ui, kpi_logger | FleetStatus | 1 Hz |
| `/fleet/intervention` | control_ui | kpi_logger, fleet_manager | Intervention | 운영자 버튼 |
| `mission/assigned` | fleet_manager | shuttle_mission | Mission | S3→S4 트리거 |
| `mission/finish` | fleet_manager | return_dock, follow_target | Bool | S9 |
| `shuttle/status` | shuttle_mission, return_dock | fleet_manager, kpi_logger | String | READY/SHUTTLE/AT_BAY/WAITING/DOCKING/DOCKED |
| `shuttle/arrived` | shuttle_mission | follow_target | Bool | Z-05 도착 (구 goto_goal/arrived) |
| `target/map_point`, `target/visible`, `target/image` | amr_detection | follow_target | PointStamped, Bool, Image | Kn1_mini 계약 유지 |
| `target/paired`, `target/worker_id` | amr_detection | follow_target, fleet_manager | Bool, String | 재식별 게이트 통과 |
| `aruco/id`, `aruco/distance` | aruco_detector | shuttle_mission | Int32, Float32 | -1 = 없음 |
| `approach/status` | follow_target | fleet_manager, kpi_logger | String | IDLE/SEARCHING/APPROACH_1/2/ARRIVED/FOLLOWING/**STANDBY**/FAILED |
| `safety/estop` | safety_watchdog | 모든 로봇 노드, fleet_manager | Bool | True 동안 주행 명령 금지 |
| `cmd_vel` | safety_watchdog(0), Nav2 | Create3 | TwistStamped | estop 중 20 Hz 0 발행 |

TB4 기본 토픽: `oakd/rgb/image_raw/compressed`, `oakd/stereo/image_raw/compressedDepth`, `oakd/rgb/camera_info`, `scan`, `battery_state`, `amcl_pose`.

## ArUco ID 체계

| ID | 의미 |
|---|---|
| 1 / 2 / 3 | 베이 A / B / C (10 cm, 베이 안쪽 벽, 높이 ≈ 20 cm) |
| 10~19 | KOZ 바닥 마커 |

## QoS

- 명령·상태·미션·서비스: RELIABLE. 카메라·scan·힌트: BEST_EFFORT(sensor_data).

## 미정

- `color_hist` bin 수, `reid_threshold` (Q-13), FleetStatus yaw 계산.
