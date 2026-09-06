# Interfaces — 계약 v1.0 (2026-09-05 확정)

정의 원본은 `ros2_ws/src/mro_interfaces`. 이 문서는 의도·규칙·확정 값을 설명한다.

> **이 문서는 9/7 병렬 구현의 전제다.** 여기 적힌 이름·타입·값을 기준으로 각 파트가 독립적으로 만들고, 통합 때 그대로 붙는다.
> **변경 절차:** 계약을 바꿔야 하면 **먼저 PM에게 알리고**, `mro_interfaces` 와 이 문서를 **같은 커밋에서** 고친다. 혼자 코드만 바꾸면 통합일에 드러난다.

---

## 1. 메시지·서비스 (mro_interfaces)

| 타입 | 용도 | 비고 |
|---|---|---|
| `msg/WorkerDescriptor` | 웹캠1 등록 결과 (worker_id, outfit_class, color_hist, robot_ns) | 아래 2절 |
| `msg/Mission` | 미션 상태 (CREATED…FAILED 상수) | 상태 전이는 fleet_manager 가 DB 에 기록 |
| `msg/FleetStatus` | 로봇별 집계 상태, 1 Hz | BR-T08 ≤ 2 s. `yaw` 는 아래 2절 |
| `msg/TargetHint` | 웹캠2 → 대상 map 좌표 | **안전 판단 금지.** 재탐지 회전 방향 결정에만 |
| `msg/Intervention` | 개입 이벤트 | `SAFETY_POLICY` 는 개입으로 계상하지 않음 (BRD 5.3) |
| `srv/RequestMission` | UI → fleet | 접수 ≤ 2 s |
| `srv/ZoneToken` | 로봇 → zone_semaphore | acquire/release, FIFO. 구역은 아래 3절 |
| `srv/OperatorCommand` | UI → fleet | **RETURN**(해당 로봇) / **RETURN_ALL**(전체) 만. 정지·재개는 9/4 결정으로 제외 |

## 2. 확정된 값 — 더 논의하지 않는다

| 항목 | 확정 | 근거 |
|---|---|---|
| **`color_hist` 형식** | **HSV 2D 히스토그램, H 16 bin × S 8 bin = 길이 128, `cv2.normalize` 정규화.** 웹캠1(`webcam_register`)과 AMR(`amr_detection`)이 **반드시 같은 함수 `mro_perception.core.reid.hsv_hist()` 를 호출한다** | 숫자만 맞추면 구현이 갈라진다. 함수를 공유해야 서술자가 실제로 비교 가능하다 |
| **`color_hist` 추출 영역** | **벨트 위 상의 영역만.** bbox 전체나 하의를 포함하지 않는다 | 두 인형의 하의가 동일색(`#2B2F36`)이라 섞이면 히스토그램이 가까워져 변별력이 떨어진다 |
| **bbox 범위** | **인형만. RC카 차체는 제외**한다 | 차체를 넣으면 모델이 차체를 학습해 상의 교체 검증(AC-10)이 무의미해진다 |
| **`FleetStatus.yaw`** | `amcl_pose` 의 쿼터니언에서 `yaw = atan2(2(wz+xy), 1−2(y²+z²))`, 단위 **rad**, 범위 (−π, π] | 표시·기록용. 제어에 쓰지 않는다 |
| **좌표계** | 모든 map 좌표는 **단일 맵 원점**(SLAM 작성 시점) 기준. 로봇은 기동 시 자신의 `dock_pose` 로 초기화한다 | 각자 (0,0) 으로 초기화하면 두 map 프레임이 어긋난다 (I-013) |
| **시각** | 모든 로그·CSV 는 **ROS 시각**(`get_clock().now()`). 벽시계 시각을 섞지 않는다 | 후처리에서 로봇·웹캠·관제 로그를 합쳐야 한다 |

**남은 1건 — `reid_threshold`**
값은 **파라미터**(`robotN.yaml`, 잠정 `0.6`)이므로 **구현은 이 값 없이도 진행할 수 있다.** 확정 절차만 정해둔다.

> 수집한 데이터셋에서 **같은 인형끼리의 유사도 분포**와 **다른 인형끼리의 유사도 분포**를 각각 구하고, **다른 인형 분포의 최댓값보다 위**에 임계를 둔다(오페어링 0건이 절대 기준이므로). 두 분포가 겹치면 서술자 자체를 손봐야 한다는 신호다. — 담당 정정인·민범진, 9/8 학습 완료 직후

---

## 3. 구역 토큰 (ZoneToken)

**토큰은 로봇끼리만 경합하는 곳에만 둔다.**

| 구역 ID | 위치 | 폭 | 쓰이는 곳 |
|---|---|---|---|
| `P1` | 도크 ↔ 창고 병목 | 0.79 m (실측) | S4 창고 갈 때 · S10 복귀할 때 두 번 통과 → **E-3 교착** |
| `DOCK` | 도크 구역 | — | 복귀 시 한 번에 한 대만 진입 → **E-7 순차 도킹** |

**추종 경로(중앙 복도 → 작업공간, 0.93 m)에는 토큰을 두지 않는다.** 토큰을 기다리며 멈추면 작업자를 놓치기 때문이다. 두 쌍의 출발을 엇갈리게 해서 회피한다.

**규칙**
- 획득: `acquire=true`. 비어 있고 대기열 선두면 `granted=true`, 아니면 `false` + `holder` 반환. 거절되면 **2초 후 재요청**한다.
- 반납: `acquire=false`. **성공·실패와 무관하게 반드시 반납한다** (도킹 실패 시에도).
- 강제 회수: 보유 60초 초과 시 세마포어가 회수하고 이벤트를 남긴다.
- 폴리곤 진입 **전에** 획득하고, 완전히 빠져나온 **후에** 반납한다.

---

## 4. 토픽 계약

전역(관제·웹캠)은 절대 이름, 로봇 노드는 **상대 이름**(launch 가 `namespace:=/robotN` 을 붙인다).

| 토픽 | 송신 | 수신 | 타입 | 비고 |
|---|---|---|---|---|
| `/webcam1/worker_descriptor` | webcam_register | fleet_manager, amr_detection, webcam_hint | WorkerDescriptor | S1 등록 ≤ 10 s |
| `/webcam2/target_hint` | webcam_hint | follow_target, kpi_logger | TargetHint | E-1. 유효 2 s |
| `/fleet/mission` | fleet_manager | kpi_logger, UI | Mission | 상태 전이마다 |
| `/fleet/status` | fleet_manager | control_ui, kpi_logger | FleetStatus | 1 Hz |
| `/fleet/intervention` | control_ui | kpi_logger, fleet_manager | Intervention | 운영자 버튼 |
| `mission/assigned` | fleet_manager | shuttle_mission | Mission | **S4 트리거.** 옛 `webcam/detected` 는 쓰지 않는다 |
| `mission/finish` | fleet_manager | return_dock, **follow_target** | Bool | S9. follow_target 은 **즉시 페어링 해제·추종 중단**(≤ 1 s), return_dock 은 복귀 시작 |
| `shuttle/status` | shuttle_mission, return_dock | fleet_manager, kpi_logger | String | READY / SHUTTLE / AT_BAY / WAITING / RETURNING / DOCK_WAIT / DOCKING / DOCKED / FAILED |
| `shuttle/arrived` | shuttle_mission | follow_target | Bool | 대기공간 도착 (구 `goto_goal/arrived`) |
| `target/map_point`, `target/visible`, `target/image` | amr_detection | follow_target | PointStamped, Bool, Image | Kn1_mini 계약 유지 |
| `target/distance` | amr_detection | kpi_logger | Float32 | **추종거리 판정의 기준값**(FR-29). 검출 노드가 산출한 대상 거리, m |
| `target/paired`, `target/worker_id` | amr_detection | follow_target, fleet_manager | Bool, String | 재식별 게이트 통과 여부 |
| `aruco/id`, `aruco/distance` | aruco_detector | shuttle_mission | Int32, Float32 | `id = -1` 이면 없음 |
| `approach/status` | follow_target | fleet_manager, kpi_logger | String | IDLE / SEARCHING / APPROACH_1 / APPROACH_2 / ARRIVED / FOLLOWING / STANDBY / FAILED |
| `safety/estop` | safety_watchdog | 모든 로봇 노드, fleet_manager | Bool | True 동안 **주행 명령 금지** |
| `cmd_vel` | safety_watchdog(0), Nav2 | Create3 | TwistStamped | estop 중 20 Hz 로 0 발행 |

**쓰지 않는 옛 토픽** (Kn1_mini 이관 잔재): `webcam/detected`, `webcam/car_map_point`, `goto_goal/arrived`. 이관 노드에 남아 있으면 지운다.

**TB4 기본 토픽**: `oakd/rgb/image_raw/compressed`, `oakd/stereo/image_raw/compressedDepth`, `oakd/rgb/camera_info`, `scan`, `battery_state`, `amcl_pose`.

---

## 5. 파라미터 계약

값은 `mro_bringup/params/` 가 **단일 출처**다. 노드는 상수로 하드코딩하지 않고 파라미터로 읽는다.

| 파일 | 담는 것 | 담당 |
|---|---|---|
| `arena.yaml` | 베이 waypoint·ArUco ID, 대기공간, `dock_zone`(pose·approach·holding), `passages`, `semaphore_zones` 폴리곤, `homography_marks`, `t_lim_s`·`t_ret_s`·`dock_retry` | 박병후·이시율 |
| `robot3.yaml` / `robot4.yaml` | `dock_pose`, `follow_standoff_m` 0.5, `min_separation_m` 0.3, `v_max` 0.3, `reid_threshold`, `bay_dwell_s` 5, `t_lim_s` 120, `t_ret_s` 180, `dock_retry` 2 | 박병후·이시율 |
| `mro_perception/config/perception.yaml` | `detect_conf`, `max_core_gap_mm`, `sync_slop_s`, `marker_size_m` | 정정인·민범진 |
| `mro_webcam/config/homography_cam1.json` | 웹캠1 호모그래피 행렬 | 한세교·김학민 |
| `mro_webcam/config/WebCam2/homography_cam2.json` | 웹캠2 호모그래피 행렬·대응점·재투영 오차 | 한세교·김학민 |

**`dock_pose` 는 `arena.yaml` 의 `dock_zone.pose.<robot>` 과 같은 값이어야 한다.** 두 곳이 어긋나면 맵 좌표계가 깨진다.

---

## 6. ArUco ID 체계

| ID | 의미 |
|---|---|
| 1 / 2 / 3 | 베이 A / B / C. 10 cm, **선반 뒤 벽면 높이 20 cm**(카메라 광축). 바닥 부착 금지 — 가까워지면 시야 아래로 사라진다. 무광 인쇄, 검은 테두리 바깥에 흰 여백 한 칸 이상 |
| 10~19 | *(예약)* KOZ 바닥 마커 — Phase 2. Phase 1 에서는 쓰지 않는다 |

검출 조건: 거리 ≤ 1.5 m, 정면 ±30°.

---

## 7. DB 스키마

`mro_fleet/config/schema.sql` 이 기준. 테이블 셋 — `workers`(작업자·복장·배정 로봇), `missions`(요청→도착→종료, 리드타임 판정), `events`(상태 전이·개입·estop, IPH 산출 원천).

**판정 규칙**
- `missions.result` 는 **T_lim 기준으로 확정**한다. T_lim 초과 후 도착하면 실패로 두고 도착 사실은 `arrived_late` 로 따로 남긴다 — 이중 계상하지 않는다.
- 개입은 `events.kind='INTERVENTION'`. **안전 정책 기인 정지(E-4·E-6) 후의 자동 재개는 개입이 아니다.**

---

## 8. QoS

- 명령·상태·미션·서비스: **RELIABLE**
- 카메라·`scan`·힌트: **BEST_EFFORT**, KEEP_LAST, depth 1 (sensor_data)
