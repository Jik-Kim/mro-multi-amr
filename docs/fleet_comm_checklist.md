# 관제 ROS 2 통신 점검표

`/{ns}`는 `/robot3` 또는 `/robot8`이다. 코드·테스트와 실제 ROS 2 통신 검증을 구분한다.

## 현재 역할

| 구성요소 | 역할 |
|---|---|
| `RobotStatus` | 로봇 한 대의 상태 구독·보관, `/fleet/status` 1 Hz 발행 |
| `RobotStateManager` | 로봇 객체·상태 변경·가용성·미션 상태 관리 |
| `FleetUICommands` | UI의 Fleet 서비스 요청과 개입 이벤트 발행 |
| `FleetManager` | 미션 처리, DB 기록, Fleet 서비스·토픽 처리 |
| `MonitorWebApp` | 상태 화면과 HTTP 요청 처리 |

## 토픽

| 토픽 | 발행자 | 구독자 | 관제 역할 | 처리 함수 | 구현 | 작업 상황 |
|---|---|---|---|---|---|---|
| `/webcam1/worker_descriptor` | `webcam_register` | `FleetManager`, `amr_detection`, `webcam_hint` | 작업자 등록·DB 저장 | `FleetManager._register_worker` | 구현 | 코드·테스트 [x] / 실통신 [ ] |
| `/fleet/status` | 로봇별 `RobotStatus` | `FleetManager`, `KpiLogger` | AMR 상태 1 Hz 전달 | `RobotStatus._publish_status`, `RobotStateManager.update_fleet_status`, `FleetManager._receive_fleet_status` | 구현 | 코드·테스트 [x] / 실통신 [ ] |
| `/fleet/intervention` | `FleetUICommands` | `FleetManager`, `KpiLogger` | 운영자 개입 DB·CSV 기록 | `FleetUICommands.publish_intervention`, `FleetManager._record_intervention_event` | 구현 | 코드·테스트 [x] / 실통신 [ ] |
| `/fleet/zone_event` | `zone_semaphore` | `FleetManager`, `KpiLogger` | P1·DOCK 이벤트 기록 | `FleetManager._record_zone_event` | 구현 | 코드·테스트 [x] / 실통신 [ ] |
| `/fleet/mission` | `FleetManager` | `KpiLogger` | 미션 생성·전이 전달 | `FleetManager._publish_mission_update` | 구현 | 코드·테스트 [x] / 실통신 [ ] |
| `/{ns}/mission/assigned` | `FleetManager` | `shuttle_mission`, `amr_detection`, `RobotStatus` | 작업자·베이·미션 전달 | `pub_assigned`, `RobotStatus._receive_mission` | 구현 | 코드·테스트 [x] / 실통신 [ ] |
| `/{ns}/mission/finish` | `FleetManager` | `return_dock`, `follow_target`, `amr_detection` | 작업 즉시 종료·복귀 | `pub_finish`, `FleetManager.return_immediately` | 구현 | 코드·테스트 [x] / 실통신 [ ] |
| `/{ns}/approach/status` | `follow_target` | `RobotStatus`, `KpiLogger` | 접근·추종 상태와 미션 전이 | `RobotStatus._receive_approach_status`, `RobotStateManager.update_source_state` | 구현 | 코드·테스트 [x] / 실통신 [ ] |
| `/{ns}/shuttle/status` | `shuttle_mission`, `return_dock` | `RobotStatus`, `KpiLogger` | 셔틀·복귀·도킹 상태와 미션 전이 | `RobotStatus._receive_shuttle_status`, `RobotStateManager.update_source_state` | 구현 | 코드·테스트 [x] / 실통신 [ ] |
| `/{ns}/battery_state` | AMR/Create 3 | `RobotStatus` | 배터리 보관·배정 판단 | `RobotStatus._receive_battery_status`, `RobotStateManager.get_available_robots` | 구현 | 코드·테스트 [x] / 실통신 [ ] |
| `/{ns}/amcl_pose` | AMCL | `RobotStatus` | map 좌표·yaw 보관 | `RobotStatus._receive_pose` | 구현 | 코드·테스트 [ ] / 실통신 [ ] |
| `/{ns}/safety/estop` | `safety_watchdog` | `RobotStatus`, `KpiLogger`, 주행 노드 | E-stop 기록·배정 제외 | `RobotStatus._receive_estop` | 구현 | 코드·테스트 [x] / 실통신 [ ] |
| `/{ns}/target/paired` | `amr_detection` | `follow_target`, `RobotStatus` | 페어링 해제 처리 | `RobotStatus._receive_paired` | 구현 | 코드·테스트 [ ] / 실통신 [ ] |
| `/{ns}/target/worker_id` | `amr_detection` | `follow_target`, `RobotStatus` | 페어링 작업자 보관 | `RobotStatus._receive_worker_id` | 구현 | 코드·테스트 [ ] / 실통신 [ ] |
| `/{ns}/target/pair_state` | `amr_detection` | `RobotStatus`, `KpiLogger` | 페어링 판정 상태 보관 | `RobotStatus._receive_pair_state` | 구현 | 코드·테스트 [x] / 실통신 [ ] |
| `/{ns}/aruco/id`, `/{ns}/aruco/distance` | `aruco_detector` | `shuttle_mission` | 관제 직접 처리 없음 | — | 관제 제외 | 확인 대상 아님 |

## 서비스

| 서비스 | 요청자 | 서버 | 기능 | 처리 함수 | 구현 | 작업 상황 |
|---|---|---|---|---|---|---|
| `/fleet/request_mission` | `FleetUICommands` | `FleetManager` | 가용 AMR 선택·미션 생성 | `FleetUICommands.request_mission`, `FleetManager._handle_mission_request` | 구현 | 코드·테스트 [x] / 실통신 [ ] |
| `/fleet/operator_command` | `FleetUICommands` | `FleetManager` | `RETURN`·`RETURN_ALL` 즉시 복귀 | `FleetUICommands.return_immediately`, `FleetManager.return_immediately` | 구현 | 코드·테스트 [x] / 실통신 [ ] |
| `/fleet/manual_pair` | `FleetUICommands` | `FleetManager` | 선택 로봇의 `/manual_pair`로 중계 | `FleetUICommands.request_manual_pair`, `FleetManager._forward_manual_pair_request` | 구현 | 코드·테스트 [x] / 실통신 [ ] |

## 점검 기준

1. `ros2 topic info` 또는 `ros2 service info`에 예상 연결이 보인다.
2. 메시지 또는 서비스 응답이 예상 형식으로 도착한다.
3. `RobotStateManager` 상태, DB 기록, 후속 발행 결과가 예상과 같다.
4. `/robot3`과 `/robot8` 상태가 서로 섞이지 않는다.
