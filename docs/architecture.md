# Architecture

## 배치 (PC 5대 + 로봇 2대, 모두 로봇 Discovery Server 경유)

PC-W1(웹캠1) · PC-W2(웹캠2) · PC-F(관제) · PC-R3 · PC-R4. 웹캠 PC 분리는 9/5 결정.

```text
[PC-W1] webcam_register(웹캠1) ─ /webcam1/worker_descriptor ─▶ [PC-F] fleet_manager ─ /robotN/mission/assigned ─▶ [PC-R3 / PC-R4]
[PC-W2] webcam_hint(웹캠2)     ─ /webcam2/target_hint ─────────────────────────────────────────────────────▶ follow_target
                                        [PC-F] kpi_logger ◀─ 상태·미션·개입 토픽
                                        [PC-F] zone_semaphore ◀─ /fleet/zone_token ─ shuttle_mission, return_dock
                                        [PC-F] control_ui(:5000) ─ /fleet/request_mission, /fleet/operator_command
```

## 로봇 1대 내부 (namespace /robotN)

```text
oakd rgb+depth ─▶ amr_detection ─ target/map_point, target/visible, target/paired ─▶ follow_target ─ goToPose / cmd_vel ─▶ Nav2 ─▶ Create3
               └▶ aruco_detector ─ aruco/id, aruco/distance ─▶ shuttle_mission ─ goToPose, undock ─▶ Nav2
/scan ─▶ safety_watchdog ─ safety/estop, cmd_vel(0) ──────────────────────────────────────────────▶ (모든 노드가 estop 을 존중)
mission/assigned ─▶ shuttle_mission ─ shuttle/arrived ─▶ follow_target           mission/finish ─▶ return_dock ─ dock
```

## 시나리오 ↔ 노드

| 단계 | 노드 | 트리거 → 결과 |
|---|---|---|
| S1 등록 | webcam_register → fleet_manager | 서술자 발행 → 작업자-로봇 배정 |
| S2 준비 | fleet_manager → shuttle_mission | (등록 완료) → undock, READY |
| S3 요청 | control_ui → fleet_manager | RequestMission → Mission(ASSIGNED) |
| S4 셔틀 | shuttle_mission (+zone_semaphore, aruco_detector) | 베이 waypoint → ArUco 확인 → 체류 → Z-05 → shuttle/arrived |
| S5 합류 | amr_detection(reid) → follow_target | target/paired → APPROACH |
| S6~S8 추종 | follow_target (+webcam_hint) | FOLLOWING / Lost → 힌트 방향 탐색 / STANDBY |
| E-4 정지 | safety_watchdog | /scan → estop, cmd_vel 0 |
| S9 종료 발행 | control_ui → fleet_manager | RETURN / RETURN_ALL → mission/finish → follow_target 즉시 추종 중단 |
| S10 순차 복귀 | return_dock (+zone_semaphore) | Z-09·Z-05·Z-08 통과 → **Z-03 도크 토큰 1대씩** → dock (실패 시 2회 재시도) → 토큰 반납 |

## 계층 원칙

- `core/` 는 ROS·장치 비의존 순수 함수(재식별, 거리 보정, 호모그래피, standoff 계산, DB). `nodes/` 가 메시지 변환과 호출 순서만 담당.
- 안전은 온보드(safety_watchdog + Nav2 costmap). 웹캠·관제·재식별은 "더 잘 하게" 할 뿐 "안 부딪히게" 하는 데 관여하지 않는다.
