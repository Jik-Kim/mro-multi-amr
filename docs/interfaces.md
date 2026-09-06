# Interfaces — 계약 v1.6 (2026-09-06)

> v1.0(9/5) 에 **주기·시간 상수 규칙**을 더하고(v1.1), **캡처 fps 를 장치 상한으로 완화**하고(v1.2), **`color_hist` 추출 영역을 인형 전체로 재확정**했고(v1.3), **P1 통과 방향을 9/6 배치도로 정정**했고(v1.4), **KPI 산출 계약(9절)을 신설**하고(v1.5), **판정 주체와 그 한계를 9.2 로 분리**했다(v1.6).
> 토픽·메시지·서비스 정의는 v1.0 그대로이므로 구현에 영향 없다.

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
| **`color_hist` 추출 영역** | **인형 머리부터 바지 밑단(RC카 지붕)까지.** 상의만 잘라내지 않는다 (9/6 재확정) | 상의만 쓰면 웹캠1 거리에서 픽셀이 부족해 히스토그램이 뭉개진다 (I-015). 두 인형의 하의·머리는 **동일**하므로 어느 쪽과 비교하든 같은 값을 더하는 상수로 작용한다 — 넣는다고 변별 신호가 사라지지 않고 **유사도 범위가 위로 압축될 뿐**이다. 상의 두 색은 H 빈이 완전히 갈리므로 판정은 여전히 상의가 가른다. **대신 다른 인형끼리의 유사도가 0.5 안팎에서 시작하므로 임계가 좁은 구간에 놓인다** → 아래 `reid_threshold` 의 분포 확인 절차를 눈대중으로 대신하지 않는다 |
| **bbox 범위** | **인형만. RC카 차체는 제외**한다 | 차체를 넣으면 모델이 차체를 학습해 상의 교체 검증(AC-10)이 무의미해진다 |
| **`FleetStatus.yaw`** | `amcl_pose` 의 쿼터니언에서 `yaw = atan2(2(wz+xy), 1−2(y²+z²))`, 단위 **rad**, 범위 (−π, π] | 표시·기록용. 제어에 쓰지 않는다 |
| **좌표계** | 모든 map 좌표는 **단일 맵 원점**(SLAM 작성 시점) 기준. 로봇은 기동 시 자신의 `dock_pose` 로 초기화한다 | 각자 (0,0) 으로 초기화하면 두 map 프레임이 어긋난다 (I-013) |
| **시각** | 모든 로그·CSV 는 **ROS 시각**(`get_clock().now()`). 벽시계 시각을 섞지 않는다 | 후처리에서 로봇·웹캠·관제 로그를 합쳐야 한다 |
| **주기** | 웹캠은 **캡처는 장치 상한 그대로(25~30 fps) · 추론 발행 10 Hz**, AMR OAK-D 는 10 Hz. 세 카메라의 주기를 통일하지 않는다 | 지켜야 하는 것은 특정 숫자가 아니라 **캡처 ≥ 발행** 이다. 장치가 30 을 못 내면 그 상한을 쓴다 (웹캠1 Jieli = 25 fps, 9/6 확인). 하는 일의 시간 상수가 달라 주기를 맞출 이유가 없고, 맞춰야 하는 것은 주기가 아니라 위의 **시각**이다 (SDD 2.3 주기 정책) |
| **시간 상수 표기** | 히스테리시스·유효기간 같은 값은 **프레임 수가 아니라 초로** 적고 파라미터로 노출한다 | 프레임 수로 적으면 추론 주기를 바꿨을 때 의미가 조용히 달라진다 |

**남은 1건 — `reid_threshold`**
값은 **파라미터**(`robotN.yaml`, 잠정 `0.6`)이므로 **구현은 이 값 없이도 진행할 수 있다.** 확정 절차만 정해둔다.

> 수집한 데이터셋에서 **같은 인형끼리의 유사도 분포**와 **다른 인형끼리의 유사도 분포**를 각각 구하고, **다른 인형 분포의 최댓값보다 위**에 임계를 둔다(오페어링 0건이 절대 기준이므로). 두 분포가 겹치면 서술자 자체를 손봐야 한다는 신호다. — 담당 정정인·민범진, 9/8 학습 완료 직후
>
> **추출 영역이 인형 전체로 정해지면서(9/6) 이 절차가 더 중요해졌다.** 하의·머리가 공통이라 다른 인형끼리도 0.5 안팎에서 시작한다. 0.6 이나 0.7 같은 어림값을 그대로 쓰면 두 인형이 같은 사람으로 판정될 수 있다. **분포를 실제로 그려보고 정한다.**
>
> 웹캠1 의 재등록 판정(끊긴 트랙이 기존 등록자인지)은 **다른 결정**이므로 값을 따로 둔다 (`webcam.yaml`).

---

## 3. 구역 토큰 (ZoneToken)

**토큰은 로봇끼리만 경합하는 곳에만 둔다.**

| 구역 ID | 위치 | 폭 | 쓰이는 곳 |
|---|---|---|---|
| `P1` | 로봇 구역(도크·창고) ↔ 중앙 복도 병목 | 0.79 m (실측) | 도크 → 창고는 벽 없이 이어져 지나지 않는다. **S4 창고에서 대기공간으로 나올 때 · S10 복귀할 때** 두 번 통과 → **E-3 교착** |
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
| `/webcam2/target_hint` | webcam_hint | follow_target, kpi_logger | TargetHint | E-1. **10 Hz** 발행, 유효 2 s |
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
| `mro_webcam/config/WebCam1/homography_cam1.json` | 웹캠1 호모그래피 행렬 | 한세교·김학민 |
| `mro_webcam/config/WebCam2/homography_cam2.json` | 웹캠2 호모그래피 행렬·대응점·재투영 오차 | 한세교·김학민 |
| `mro_webcam/config/webcam.yaml` | `process_hz` 10, `capture_fps` **장치 상한**(웹캠1 = 25), `on_time_s` 0.3, `off_time_s` 0.5, `register_s` 1.0, `hint_valid_s` 2.0 | 한세교·김학민 |

**`dock_pose` 는 `arena.yaml` 의 `dock_zone.pose.<robot>` 과 같은 값이어야 한다.** 두 곳이 어긋나면 맵 좌표계가 깨진다.

---

## 6. ArUco ID 체계

| ID | 의미 |
|---|---|
| 1 / 2 / 3 | 베이 A / B / C. 10 cm, **선반 뒤 벽면 높이 20 cm.** 실측 광축은 23.5 cm 라 마커는 그보다 3.5 cm 아래에 오지만 화각 안이다. 올리면 선반(25 cm)에 가린다. 바닥 부착 금지 — 가까워지면 시야 아래로 사라진다. 무광 인쇄, 검은 테두리 바깥에 흰 여백 한 칸 이상 |
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

---

## 9. KPI 산출 계약

**지표 하나는 셋이 다 정해져야 숫자가 된다** — ① 무엇을 1건으로 세나 ② 언제부터 언제까지 재나 ③ 어느 기록에서 읽나.
하나만 어긋나도 같은 시험에서 두 사람이 다른 숫자를 낸다. 실제로 9/6 에 리포트 50 % 와 DB 참값 0 % 가 갈렸다 (I-023).

| 지표 | 1건의 단위 | 시작 → 끝 | 읽는 곳 |
|---|---|---|---|
| **AC-08 · K-05** 배송 성공률 | 미션 1건 | 요청 접수 → **대기공간 도착** (≤ `t_lim_s` 120 s) | DB `missions.result` |
| **K-05** 리드타임 | 미션 1건 | 위와 같은 구간 | DB `waiting_at − created_at` |
| **AC-05** 추종거리 · 최소 이격 | 샘플 1개 | **`approach/status == FOLLOWING` 인 동안만** | CSV `target/distance` |
| **K-04** 재획득 | Lost 1건 | `target/visible` 1→0 (1.0 s 이상 지속) → 0→1 | CSV `target/visible` |
| **AC-13 · K-07** IPH | 개입 1건 ÷ 시간 | 분모 = undock → 도킹 완료 **구간의 합** | 분자 DB `events` · 분모 CSV `shuttle/status` |
| **AC-03 · K-01** 추종 성공률 | 유효 미션 1건 | 페어링 확정(S5) → 종료 발행(S9) | CSV 시계열 + **웹캠2 영상 판독** |
| **AC-04 · K-03** ID Swap | 교차 1건 | 교차 이벤트 단위 | **웹캠2 영상 판독** |

**"성공률" 은 두 개다.** `AC-08 배송`(제때 닿았나)과 `AC-03 추종`(엉뚱한 사람을 안 따라갔나)은 재는 구간도 판정 근거도 다르다. 표와 발표에서 반드시 이름을 붙여 부른다.

**계산식 — 어림하지 않는다**

- **배송 성공률** = `result='SUCCESS'` / 전체 미션. **`result IS NULL` 도 분모에 남긴다** — 판정이 없다고 빼면 못 간 미션일수록 성공률이 올라간다. `Mission.DONE` 도달 여부로 세지 않는다 (T_lim 을 넘겨 도착해도 결국 DONE 이 된다).
- **추종거리** = `|distance − follow_standoff_m|` 의 **p95 ≤ 0.2 m** **AND** `min(distance)` **≥ 0.3 m**. 거리 원값의 p95 가 아니고, 접근 단계(standoff 1.0·0.3 m)와 STANDBY 구간을 포함하지 않는다.
- **IPH** = 개입 건수 / 운용 시간(h). `SAFETY_POLICY` 는 개입이 아니다 (BRD 5.3). **로그 전체 구간을 분모로 쓰지 않는다** — 미션 사이 대기가 들어가면 IPH 가 실제보다 작게 나온다.
- **재획득** = Lost 후 게이트를 다시 통과한 건이 성공. **STANDBY·FAILED 로 끝난 건은 실패로 따로 센다.** 느린 성공으로 섞으면 K-04 의 성공률이 사라진다. `approach/status == 'SEARCHING'` 은 Lost 신호가 아니다 — 사이클 시작과 2단계 접근에도 쓰인다.

### 9.1 누가 무엇을 맡나

한 지표에 세 사람이 걸린다. **값을 만드는 쪽 · 기록하는 쪽 · 채점하는 쪽을 나눈다** — 자기 성적을 자기가 매기지 않게 하기 위해서다.

| 역할 | 하는 일 | 담당 |
|---|---|---|
| 값을 만든다 | 원천값을 토픽으로 발행 (`target/distance`, `target/visible`, `shuttle/status` …) | **그 노드의 담당** |
| 자를 검증한다 | 그 값 자체가 맞는지 사전 확인 (TS-U05 줄자 대조, FR-34) | **그 노드의 담당** |
| 기록한다 | `kpi_logger` CSV 한 파일. **판정하지 않고** 시각과 값만 남긴다 | **웹캠 한세교·김학민** |
| **집계·보고한다** | `tools/kpi_report.py`. 구간을 자르고 집계해 AC 와 대조한다. **판정하지 않는다** | **웹캠 한세교·김학민** |
| **미션을 판정한다** | `fleet_manager` → DB `missions.result` · `events`. 배송 성공·실패는 여기서 확정된다 | **관제 김민준** |

**AC-05 가 예다.** `target/distance` 발행과 정확도 검증(TS-U05)은 **인지**, CSV 기록과 p95·최솟값 산출은 **웹캠**, 채점당하는 것은 **주행**의 추종 제어다. 셋이 다른 사람인 것이 요점이다 — AC-05 는 인지의 성적이 아니라 주행의 성적이고, 그래서 값을 만드는 쪽도 채점당하는 쪽도 아닌 제3자가 집계한다.

**관제의 DB 판정과 웹캠의 CSV 는 서로 독립이어야 한다.** 두 기록이 어긋나는 것 자체가 신호다. 한 사람이 둘 다 만들면 그 신호가 사라진다.

**소유와 실행 위치는 별개다.** `kpi_logger` 는 `mro_webcam` 패키지지만 **PC-F 에서 실행한다**(`fleet_pc.launch.py`) — fleet 토픽을 가장 많이 구독하기 때문이다. `zone_semaphore` 가 `mro_fleet` 패키지지만 주행 소관인 것과 같은 종류의 예외다 (`docs/responsibilities.md`).

### 9.2 배송 판정(AC-08)의 사슬과 그 한계

**웹캠은 AC-08 을 보지 않는다.** 웹캠1 은 관제탑 구역, 웹캠2 는 작업공간과 P3 개구부만 본다 — **대기공간(Z-05)은 두 카메라 모두 시야 밖이다** (I-010, 9/5 확정). 판정은 로봇과 관제 사이에서 끝난다.

| 단계 | 노드 | 하는 일 |
|---|---|---|
| 도착 | `shuttle_mission` **[주행]** | Nav2 로 `waiting_zone` waypoint 도달 → `shuttle/status = WAITING` 발행 |
| 판정 | `fleet_manager` **[관제]** | 그 시각을 `waiting_at` 에 저장, `t_lim_s` 와 비교 → `missions.result` 확정 |
| 보고 | `kpi_report.py` **[웹캠]** | DB 를 읽어 표로 출력. 판정에 관여하지 않는다 |

BRD 5.2 도 K-05 의 측정 출처를 **「미션 로그」** 로 적는다. 웹캠2 가 지상 진실이 되는 것은 K-01·K-03(오추종·ID Swap)과 접촉 판정뿐이다 (BRD 4.1).

**한계 — 대기공간 도착에는 독립 검증 수단이 없다.**

베이 도착은 ArUco 로 독립 확인한다 — 로봇이 도착했다고 보고해도 마커 ID 가 다르면 오도착이다 (FR-08, 오도착 0건). **대기공간에는 마커가 없고 카메라도 닿지 않으므로 로봇의 자기 보고가 유일한 근거다.**

`amcl_pose` 로 대조해도 이 구멍은 메워지지 않는다. 로봇의 믿음을 같은 로봇의 믿음과 비교하는 것이라 **AMCL 이 발산하면 둘 다 함께 틀린다.** 잡히는 것은 `arena.yaml` 의 좌표 오타와 잘못된 상태 전이뿐이며, **그 둘은 이 프로젝트에서 실제로 났다** (I-001 — 베이 A waypoint 가 DOCK 폴리곤 안에 있었다). TS-07 배송 20회는 사람이 매회 지켜보지 않으므로 그만큼은 걸러둔다.

- 보강: `fleet_manager` 가 `WAITING` 을 받을 때 그 시점 `amcl_pose` 가 `arena.yaml` 의 `waiting_zone` 근처인지 확인하고, 벗어나면 `result='FAIL_WRONG_POSE'` 로 남긴다 (이미 `amcl_pose` 를 구독하고 있다).
- **AMCL 발산은 이것으로 잡히지 않는다.** 결과를 보고할 때 이 한계를 함께 적는다 (BRD 3.4 와 같은 성격).
