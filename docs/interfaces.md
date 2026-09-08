# Interfaces — 계약 v1.16 (2026-09-08)

> v1.0(9/5) 에 **주기·시간 상수 규칙**을 더하고(v1.1), **캡처 fps 를 장치 상한으로 완화**하고(v1.2), **`color_hist` 추출 영역을 인형 전체로 재확정**했고(v1.3), **P1 통과 방향을 9/6 배치도로 정정**했고(v1.4), **KPI 산출 계약(9절)을 신설**하고(v1.5), **판정 주체와 그 한계를 9.2 로 분리**했고(v1.6), **대기공간 측위 검증 마커(ArUco ID 4)를 추가**했고(v1.7), **웹캠1 호모그래피를 뺐고**(v1.8), **웹캠2 압축 영상 토픽을 추가했고**(v1.9, PR #14), **웹캠1 시야를 관제탑 구역 → 출입구로 좁혔다**(v1.10, PR #16 · Q-26).
> **v1.11 (9/7 저녁): 세마포어 이벤트 토픽 `/fleet/zone_event` 신설** — 대기·획득·반납·강제 회수를 기록해 K-06(교착 자동 해결률)을 센다 (I-030). `mro_interfaces` 는 바꾸지 않고 `std_msgs/String` 을 쓴다.
> **v1.12 (9/7 저녁): 명시 페어링 폴백(FR-13) 확정** — 재식별 게이트 미달이 30 s 지속될 때 운영자가 로봇과 작업자를 직접 짝지어 주는 경로다. 상태 토픽 `target/pair_state`, 서비스 `srv/ManualPair`, 개입 종류 `MANUAL_PAIR` 셋이 새로 생겼다. 규칙은 아래 **10절**. 관제가 올린 8개 질문(9/7)에 대한 답을 그대로 계약으로 굳힌 것이다.
> **v1.13 (9/8 새벽): 실주행이 드러낸 것 셋을 계약에 반영.** ① `aruco/pose` 비준 (PR #40) — 선택 발행, **회전(yaw)은 정렬에 쓰지 않는다.** ② ArUco 는 **발행 한계(2.0 m)와 수용 조건(1.5 m)을 나눈다** — 경계값 마커가 조용히 사라지지 않게 하되 판정은 소비자가 건다. ③ `cmd_vel` 발행자에 `follow_target`(제자리 회전) 추가, **estop 중 병진은 전부 막고 저속 제자리 회전만 통과** (I-036). **후진은 쓰지 않는다** — 2절 확정. 해제 거리 `CLEAR_DIST` 0.45 → **0.38 m** 로 낮춘다(P1 반폭 0.395 m 보다 작아야 통로 안에서 회전으로 풀린다). estop 이 복구까지 막아 로봇이 벽 앞에서 95 초를 쓴 실주행 기록이 근거다.
> **v1.14 (9/8 오후, PR #45): `amr_detection` 이 실제 노드가 되면서 드러난 셋.** ① `target/image`(raw `Image`) → **`target/image/compressed`**(CompressedImage) — raw 는 **구독자가 없는데** 대역폭만 먹었다. 계약이 `follow_target` 을 수신자로 적어둔 것이 틀렸다. 사람이 보는 용도이고 안전 판단·KPI 에 쓰지 않는다. ② **`target/visible` 의 의미를 바꾼다** — 「작업자가 보이나」에서 **「내 담당이 보이나」** 로. K-04 Lost 의 원천이라 그냥 둘 수 없다: 재획득은 *대상* 재획득이지 아무나 재획득이 아니다. ③ `mission/assigned` 수신자에 `amr_detection` 추가 — 이 로봇이 누구를 맡았는지는 여기서만 안다 (`WorkerDescriptor.robot_ns` 는 채워지지 않는다).
>
> **v1.15 (9/8): FR-13 관제 내부 전달 경로를 명시.** 기존 `srv/ManualPair` 형식은 바꾸지 않고 `control_ui` 가 `/fleet/manual_pair` 로 `fleet_manager` 에 요청하면, `fleet_manager` 가 선택된 `/robotN/manual_pair` 로 전달한다. 최종 판정과 확정 발행은 계속 `amr_detection` 하나가 맡는다.
> **v1.16 (9/8, PR #49): `approach/status` 의 접근 두 값에 뜻을 붙인다.** 값 목록은 그대로다. `APPROACH_1` = 중간 목표 · `APPROACH_2` = 최종 목표이며, **기본값에서는 후자가 발행되지 않는다** — 값이 안 보이는 것이 고장이 아님을 적어 두지 않으면 나중에 그것을 찾느라 시간을 쓴다.
>
> 토픽·메시지·서비스 정의는 v1.0 을 기준으로 v1.11 이 토픽 하나, v1.12 가 토픽 하나·서비스 하나·개입 종류 하나, v1.13 이 토픽 하나를 더했고, v1.14 는 하나를 갈고 하나의 뜻을 바꿨으며, v1.15 는 기존 서비스 형식의 관제 내부 경로 하나를 명시했다.

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
| `msg/Intervention` | 개입 이벤트 | `SAFETY_POLICY` 는 개입으로 계상하지 않음 (BRD 5.3). **`MANUAL_PAIR` 는 계상한다** (v1.12, 10절) |
| `srv/RequestMission` | UI → fleet | 접수 ≤ 2 s |
| `srv/ZoneToken` | 로봇 → zone_semaphore | acquire/release, FIFO. 구역은 아래 3절 |
| `srv/OperatorCommand` | UI → fleet | **RETURN**(해당 로봇) / **RETURN_ALL**(전체) 만. 정지·재개는 9/4 결정으로 제외 |
| `srv/ManualPair` | control_ui → fleet → amr_detection | **v1.12 형식, v1.15 경로 명시.** 관제 입구는 `/fleet/manual_pair`, 로봇별 최종 판정 서버는 `/robotN/manual_pair`. 10절 |

## 2. 확정된 값 — 더 논의하지 않는다

| 항목 | 확정 | 근거 |
|---|---|---|
| **`color_hist` 형식** | **HSV 2D 히스토그램, H 16 bin × S 8 bin = 길이 128, `cv2.normalize` 정규화.** 웹캠1(`webcam_register`)과 AMR(`amr_detection`)이 **반드시 같은 함수 `mro_perception.core.reid.hsv_hist()` 를 호출한다** | 숫자만 맞추면 구현이 갈라진다. 함수를 공유해야 서술자가 실제로 비교 가능하다 |
| **`color_hist` 추출 영역** | **인형 머리부터 바지 밑단(RC카 지붕)까지.** 상의만 잘라내지 않는다 (9/6 재확정) | 상의만 쓰면 웹캠1 거리에서 픽셀이 부족해 히스토그램이 뭉개진다 (I-015). 두 인형의 하의·머리는 **동일**하므로 어느 쪽과 비교하든 같은 값을 더하는 상수로 작용한다 — 넣는다고 변별 신호가 사라지지 않고 **유사도 범위가 위로 압축될 뿐**이다. 상의 두 색은 H 빈이 완전히 갈리므로 판정은 여전히 상의가 가른다. **대신 다른 인형끼리의 유사도가 0.5 안팎에서 시작하므로 임계가 좁은 구간에 놓인다** → 아래 `reid_threshold` 의 분포 확인 절차를 눈대중으로 대신하지 않는다 |
| **bbox 범위** | **인형만. RC카 차체는 제외**한다 | 차체를 넣으면 모델이 차체를 학습해 상의 교체 검증(AC-10)이 무의미해진다 |
| **`FleetStatus.yaw`** | `amcl_pose` 의 쿼터니언에서 `yaw = atan2(2(wz+xy), 1−2(y²+z²))`, 단위 **rad**, 범위 (−π, π] | 표시·기록용. 제어에 쓰지 않는다 |
| **좌표계** | 모든 map 좌표는 **단일 맵 원점**(SLAM 작성 시점) 기준. 로봇은 기동 시 자신의 `dock_pose` 로 초기화한다 | 각자 (0,0) 으로 초기화하면 두 map 프레임이 어긋난다 (I-013) |
| **후진 금지** | **TurtleBot4 는 후진시키지 않는다.** 어느 노드도 `linear.x < 0` 을 내지 않고, Nav2 의 후진 복구(`backup`)에도 기대지 않는다. 뒤로 빠져야 하는 상황은 **제자리 회전 뒤 전진**으로 푼다 | Create3 의 **범퍼와 클리프 센서가 전방에만 있다.** 후진은 접촉·낙하 감지가 없는 무센서 주행이고, `safety_watchdog` 도 전방 ±35° 만 본다. 무사고 0건(NF-06)이 절대 기준인 프로젝트에서 감지 없는 방향으로 움직이지 않는다 (강사 지침, 9/8) |
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
- 강제 회수: 보유 60초 초과 시 세마포어가 회수하고 `/fleet/zone_event` 에 `RECLAIM` 을 남긴다.
- **이벤트 (v1.11, I-030):** 세마포어는 `/fleet/zone_event`(String) 로 `"<zone_id> <robot_ns> <REJECT|GRANT|RELEASE|RECLAIM>"` 한 줄을 발행한다. `REJECT` 는 **대기가 시작될 때 1건**만(2초 재요청의 연속 거절은 발행하지 않음), `GRANT` 는 새로 획득할 때만(보유자 재요청 제외). `fleet_manager` 가 DB `events`(kind `ZONE`) 에, `kpi_logger` 가 CSV 에 남긴다. K-06 원천이며 `Intervention` 에 섞지 않는다.
- 폴리곤 진입 **전에** 획득하고, 완전히 빠져나온 **후에** 반납한다.

---

## 4. 토픽 계약

전역(관제·웹캠)은 절대 이름, 로봇 노드는 **상대 이름**(launch 가 `namespace:=/robotN` 을 붙인다).

| 토픽 | 송신 | 수신 | 타입 | 비고 |
|---|---|---|---|---|
| `/webcam1/worker_descriptor` | webcam_register | fleet_manager, amr_detection, webcam_hint | WorkerDescriptor | S1 등록 ≤ 10 s |
| `/webcam2/target_hint` | webcam_hint | follow_target, kpi_logger | TargetHint | E-1. **10 Hz** 발행, 유효 2 s. **BEST_EFFORT** — 구독자도 sensor_data QoS 로 잡아야 받는다 |
| `/webcam2/image_raw/compressed` | webcam_hint | (뷰어·영상 판독 기록) | CompressedImage | **jpeg 80, 10 Hz** (`image_publish_hz`). raw `Image` 는 발행하지 않는다 — 다른 PC 에서 구독하면 Wi-Fi 를 막는다 (NR-03). 안전 판단 금지, 세션 판정 후 폐기 (R-10). PR #14 |
| `/fleet/mission` | fleet_manager | kpi_logger, UI | Mission | 상태 전이마다 |
| `/fleet/status` | fleet_manager | control_ui, kpi_logger | FleetStatus | 1 Hz |
| `/fleet/intervention` | control_ui | kpi_logger, fleet_manager | Intervention | 운영자 버튼 |
| `/fleet/zone_event` | zone_semaphore | kpi_logger, fleet_manager | String | **v1.11 (I-030).** `"<zone_id> <robot_ns> <REJECT\|GRANT\|RELEASE\|RECLAIM>"`. K-06 원천 — 3절 이벤트 규칙 |
| `mission/assigned` | fleet_manager | shuttle_mission, **amr_detection** | Mission | **S4 트리거.** 옛 `webcam/detected` 는 쓰지 않는다. **v1.14 — `amr_detection` 도 받는다** (PR #45): 이 로봇이 누구를 맡았는지는 여기서만 안다. `WorkerDescriptor.robot_ns` 로는 알 수 없다 — `fleet_manager` 가 그 칸을 채우지 않는다 |
| `mission/finish` | fleet_manager | return_dock, **follow_target**, amr_detection | Bool | S9. follow_target 은 **즉시 페어링 해제·추종 중단**(≤ 1 s), return_dock 은 복귀 시작. **amr_detection 은 명시 페어링을 해제한다** (v1.12, 10절) |
| `shuttle/status` | shuttle_mission, return_dock | fleet_manager, kpi_logger | String | READY / SHUTTLE / AT_BAY / WAITING / RETURNING / DOCK_WAIT / DOCKING / DOCKED / FAILED |
| `shuttle/arrived` | shuttle_mission | follow_target | Bool | 대기공간 도착 (구 `goto_goal/arrived`) |
| `target/map_point` | amr_detection | follow_target | PointStamped | 거리 품질 게이트를 통과한 검출만. **`paired` 가 아닐 때도 발행된다** — 쓸지 말지는 `target/paired` 로 소비자가 정한다 |
| `target/visible` | amr_detection | kpi_logger, follow_target | Bool | **v1.14 (PR #45) — 의미가 바뀌었다.** 예전 「작업자 bbox 가 보이나」에서 **「재식별 게이트가 고른 *내 담당* 이 보이나」** 로. K-04 Lost 의 원천이므로 중요하다 — 재획득은 *대상* 재획득이지 아무나 재획득이 아니다. 남의 옷만 보이면 `False` 다. **단, 서술자·미션이 아직 없으면**(S1~S3 전·단독 시험) 확신도 1등을 대상으로 삼으므로 그때는 옛 의미다 |
| `target/image/compressed` | amr_detection | **없음 (사람이 보는 용도)** | CompressedImage | **v1.14 (PR #45).** 옛 `target/image`(raw `Image`)를 대체한다 — raw 는 구독자가 없는데 대역폭만 먹었다. `jpeg_quality` 80. 오버레이(bbox·거리·sim·paired)가 그려진 모니터링 영상이다. **안전 판단·KPI 에 쓰지 않는다** |
| `target/distance` | amr_detection | kpi_logger | Float32 | **추종거리 판정의 기준값**(FR-29). 검출 노드가 산출한 대상 거리, m |
| `target/paired`, `target/worker_id` | amr_detection | follow_target, fleet_manager | Bool, String | 재식별 게이트 통과 여부. **발행자는 amr_detection 하나뿐이다** — 자동이든 명시 페어링이든 (10절) |
| `target/pair_state` | amr_detection | control_ui, fleet_manager, kpi_logger | String | **v1.12 (FR-13).** `PAIRED` / `GATE_FAIL` / `NO_TARGET`. `GATE_FAIL` 은 **대상은 보이는데 게이트를 못 넘는 상태가 30 s 지속됐을 때만** 낸다 — 10절 |
| `aruco/id`, `aruco/distance` | aruco_detector | shuttle_mission, fleet_manager | Int32, Float32 | `id = -1` 이면 없음. **거리는 depth 원값에 로봇별 시차 편향 보정(역깊이 k, 9/8)**(0.8 m 이상), 그 아래는 solvePnP × 최근 depth/pnp 비 — 초점거리 결함(I-031)과 무관하다 (PR #40). **발행 한계는 `max_distance_m` 2.0 m 이고 6절의 수용 조건 1.5 m 는 소비자가 건다** |
| `aruco/pose` | aruco_detector | shuttle_mission (베이 정렬) | PoseStamped | **v1.13 비준. 선택 발행** (`publish_pose`, 기본 켬). 마커 중심의 위치·자세, `frame_id` = 카메라 광학 프레임(x 우·y 하·z 앞). `id` 와 같은 프레임에서만 발행. 좌우 치우침 = x, 방위 = atan2(x, z). **회전(yaw)은 정렬에 쓰지 않는다** — 정사각 마커의 두 해 문제로 PM 합성 검증에서 −30° 가 1.0 m 에서 33°, 2.0 m 에서 20° 로 나왔다. 위치는 `pnp × depth/pnp 비` 라 `aruco/distance` 와 몇 cm 다를 수 있다 |
| `approach/status` | follow_target | fleet_manager, kpi_logger | String | IDLE / SEARCHING / APPROACH_1 / APPROACH_2 / ARRIVED / FOLLOWING / STANDBY / FAILED. **v1.16 — 접근 두 값의 뜻 (PR #49):** `APPROACH_1` = **중간 목표**, `APPROACH_2` = **최종 목표**. 접근은 단계를 세지 않고 남은 거리의 절반씩 좁힌다(`max(follow_standoff_m, gap/2)`) — 회차 수가 정해져 있지 않으므로 이 두 값은 *몇 번째*가 아니라 *어떤 목표*를 뜻한다. **기본값에서는 `APPROACH_2` 가 발행되지 않는다**: 목표가 최종이 되는 조건 `gap ≤ 2×follow_standoff_m`(1.0)과 접근을 쓰는 조건 `gap > approach_skip_m`(1.0)이 겹치지 않는다. 마지막 0.5 m 는 `FOLLOWING` 이 맡고 AC-05 채점 구간도 `FOLLOWING` 뿐이라 기능상 빠지는 것은 없다. **없다고 고장이 아니다** |
| `safety/estop` | safety_watchdog | 모든 로봇 노드, fleet_manager | Bool | True 동안 **주행 명령 금지** |
| `cmd_vel` | safety_watchdog, Nav2, **follow_target**(제자리 회전 전용) | Create3 | TwistStamped | **v1.13.** estop 중 워치독이 20 Hz 로 덮어쓴다. **병진은 전부 0** — 전진은 장애물 쪽이라 막고, **후진은 애초에 쓰지 않는다**(2절 확정: 범퍼·클리프가 전방에만 있다). **각속도만 0.4 rad/s 상한으로 통과시킨다** — 제자리 회전은 footprint 내접 0.175 / 외접 0.189 라 쓸고 가는 폭이 1.4 cm 뿐이고, 이것이 유일한 탈출 수단이다. 해제 거리 **`CLEAR_DIST` 0.38 m**(0.45 에서 내림) — 0.45 는 P1 반폭 0.395 m 보다 커서 통로 안에서는 어느 방향으로 돌아도 해제 조건이 성립하지 않는다 (I-036) |

**쓰지 않는 옛 토픽** (Kn1_mini 이관 잔재): `webcam/detected`, `webcam/car_map_point`, `goto_goal/arrived`. 이관 노드에 남아 있으면 지운다.

**TB4 기본 토픽**: `oakd/rgb/image_raw/compressed`, `oakd/stereo/image_raw/compressedDepth`, `oakd/rgb/camera_info`, `scan`, `battery_state`, `amcl_pose`.

---

## 5. 파라미터 계약

값은 `mro_bringup/params/` 가 **단일 출처**다. 노드는 상수로 하드코딩하지 않고 파라미터로 읽는다.

| 파일 | 담는 것 | 담당 |
|---|---|---|
| `arena.yaml` | 베이 waypoint·ArUco ID, 대기공간, `dock_zone`(pose·approach·holding), `passages`, `semaphore_zones` 폴리곤, `homography_marks`, `t_lim_s`·`t_ret_s`·`dock_retry` | 박병후·이시율 |
| `common.yaml` `9/8 신설` | 전 로봇 공통값 — `follow_standoff_m` 0.5, `min_separation_m` 0.3, `v_max` 0.3, `bay_dwell_s` 5, `t_lim_s` 120, `t_ret_s` 180, `dock_retry` 2, 셔틀 상수 | 박병후·이시율 |
| `robot3.yaml` / `robot8.yaml` | **로봇마다 다른 값만** — `robot_ns`, `dock_pose`, `depth_k`. 로봇 추가 = 이 파일 하나 복사 | 박병후·이시율 |
| `fleet.yaml` `9/8 신설` | 관제 — `robot_namespaces`(로봇 명단의 단일 출처), `battery_min`, `t_lim_s`, `worker_assignment_window_s` | 김민준 |
| `mro_perception/config/perception.yaml` | `detect_conf`, `max_core_gap_mm`, `sync_slop_s`, `marker_size_m` | 정정인·민범진 |
| `mro_webcam/config/WebCam1/homography_cam1.json` | **쓰지 않는다 (9/6 결정)** — 웹캠1 은 작업자 출발(등록)만 인식하고 map 좌표를 내지 않는다. 파일은 Kn1_mini 잔재이며 `webcam_register` 가 좌표를 쓰면 안 된다 | 한세교·김학민 |
| `mro_webcam/config/WebCam2/homography_cam2.json` | 웹캠2 호모그래피 행렬·대응점·재투영 오차. 기준점 원본은 같은 폴더 `marker_coordinates.json` (로봇 AMCL 로 실측, PR #9 — RMSE 2.8 cm) | 한세교·김학민 |
| `mro_webcam/config/webcam.yaml` | `process_hz` 10, `capture_fps` **장치 상한**(웹캠1 = 25), `on_time_s` 0.3, `off_time_s` 0.5, `register_s` 1.0, `hint_valid_s` 2.0, 웹캠2 `image_publish_hz` 10 · `jpeg_quality` 80 · `buffer_size` 5(캡처 스레드가 큐를 계속 비우므로 1 이 아니어도 지연이 쌓이지 않는다) | 한세교·김학민 |

**`dock_pose` 는 `arena.yaml` 의 `dock_zone.pose.<robot>` 과 같은 값이어야 한다.** 두 곳이 어긋나면 맵 좌표계가 깨진다.

---

## 6. ArUco ID 체계

| ID | 의미 |
|---|---|
| 1 / 2 / 3 | 베이 A / B / C. 10 cm, **선반 뒤 벽면 높이 20 cm.** 실측 광축은 23.5 cm 라 마커는 그보다 3.5 cm 아래에 오지만 화각 안이다. 올리면 선반(25 cm)에 가린다. 바닥 부착 금지 — 가까워지면 시야 아래로 사라진다. 무광 인쇄, 검은 테두리 바깥에 흰 여백 한 칸 이상 |
| 10~19 | *(예약)* KOZ 바닥 마커 — Phase 2. Phase 1 에서는 쓰지 않는다 |
| **4** | **측위 검증 마커** (9/6 신설) — 웹캠1 박스 북쪽 ≈ 0.4 m 앞 거치대, 북향, 10 cm, 높이 23.5 cm. 베이 마커와 달리 **AMCL 이 맞는지 확인하는 용도**다. 로봇이 P1 을 나와 남쪽으로 대기공간에 접근하는 구간에서 정면에 들어온다 (턴 시점 ≈ 1.36 m · robot3 정차 1.08 m · robot8 정차 0.58 m, 9/6 실측). map 좌표는 `arena.yaml` `loc_check_marker` (주행 실측 9/7). 규칙은 9.2 |

**검출 조건 — 발행과 수용을 나눈다 (v1.13).**

| | 값 | 누가 |
|---|---|---|
| 발행 한계 | `max_distance_m` **2.0 m** · 넘으면 `id = -1` | `aruco_detector` |
| **수용 조건** | **거리 ≤ 1.5 m, 정면 ±30°** | **소비자** — 주행 `aruco_max_dist_m` 1.5, 관제 FR-35 도 같은 게이트를 걸어야 한다 |

발행을 1.5 로 자르면 **정확히 1.5 m 에 둔 마커가 1.52 로 걸러진다**(I-031 PM 재현). 그래서 발행은 느슨하게 두고 판정을 소비자에 맡긴다 — **계약만 보고 짜면 1.8 m 값을 받게 되므로 이 표를 반드시 볼 것.**

**정면 ±30° 는 기계로 검사하지 않는다.** `aruco/pose` 의 기운각은 거리가 멀수록 두 해 문제로 흔들려(PM 합성 검증: 1.0 m 에서 −30° 가 33°, 2.0 m 에서 20°) 게이트로 쓸 수 없다. **시험 절차에서 사람이 지킨다** — 마커 정면 접근을 회차 조건에 적는다.

---

## 7. DB 스키마

`mro_fleet/config/schema.sql` 이 기준. 테이블 셋 — `workers`(작업자·복장·배정 로봇), `missions`(요청→도착→종료, 리드타임 판정), `events`(상태 전이·개입·estop, IPH 산출 원천).

**판정 규칙**
- `missions.result` 는 **T_lim 기준으로 확정**한다. T_lim 초과 후 도착하면 실패로 두고 도착 사실은 `arrived_late` 로 따로 남긴다 — 이중 계상하지 않는다.
- 개입은 `events.kind='INTERVENTION'`. **안전 정책 기인 정지(E-4·E-6) 후의 자동 재개는 개입이 아니다.** **명시 페어링 폴백(`MANUAL_PAIR`)은 개입이다** — 사람이 판단을 대신했기 때문이다 (v1.12, 10절).
- 구역 토큰 이벤트는 `events.kind='ZONE'`, `value`=`REJECT|GRANT|RELEASE|RECLAIM`, `note`=zone_id. `fleet_manager` 가 `/fleet/zone_event` 를 받아 그대로 적는다 (v1.11). 개입이 아니므로 IPH 분자에 넣지 않는다.

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
| **FR-13** 폴백 발동률 | 폴백 1건 (`MANUAL_PAIR`) | 발동 건수 ÷ 페어링 시도 건수 · **≤ 20 %** | DB `events` kind `INTERVENTION` value `MANUAL_PAIR` |
| **AC-09 · K-06** 교착 자동 해결 | 대기 1건 (`REJECT` 로 시작) | `REJECT` → 같은 로봇·구역의 `GRANT` | DB `events` kind `ZONE` (CSV 동일) |
| **AC-03 · K-01** 추종 성공률 | 유효 미션 1건 | 페어링 확정(S5) → 종료 발행(S9) | CSV 시계열 + **웹캠2 영상 판독** |
| **AC-04 · K-03** ID Swap | 교차 1건 | 교차 이벤트 단위 | **웹캠2 영상 판독** |

**"성공률" 은 두 개다.** `AC-08 배송`(제때 닿았나)과 `AC-03 추종`(엉뚱한 사람을 안 따라갔나)은 재는 구간도 판정 근거도 다르다. 표와 발표에서 반드시 이름을 붙여 부른다.

**계산식 — 어림하지 않는다**

- **배송 성공률** = `result='SUCCESS'` / 전체 미션. **`result IS NULL` 도 분모에 남긴다** — 판정이 없다고 빼면 못 간 미션일수록 성공률이 올라간다. `Mission.DONE` 도달 여부로 세지 않는다 (T_lim 을 넘겨 도착해도 결국 DONE 이 된다).
- **추종거리** = `|distance − follow_standoff_m|` 의 **p95 ≤ 0.2 m** **AND** `min(distance)` **≥ 0.3 m**. 거리 원값의 p95 가 아니고, 접근 단계(standoff 1.0·0.3 m)와 STANDBY 구간을 포함하지 않는다.
- **교착 자동 해결률** = 60 s 안에 `GRANT` 로 끝난 대기 / 전체 대기. 그 사이 같은 구역에 `RECLAIM` 이 있었거나 60 s 를 넘긴 건은 미해결로 센다(사람 개입 없이 60 s 이내 재개 — BRD 5.3). 같은 로봇·구역의 연속 `REJECT` 는 한 건이다.
- **IPH** = 개입 건수 / 운용 시간(h). `SAFETY_POLICY` 는 개입이 아니다 (BRD 5.3). **`MANUAL_PAIR`(FR-13 폴백)는 개입이다** — 10.7. **로그 전체 구간을 분모로 쓰지 않는다** — 미션 사이 대기가 들어가면 IPH 가 실제보다 작게 나온다.
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

**웹캠은 AC-08 을 보지 않는다.** 웹캠1 은 **출입구만**(9/7, 관제탑 구역까지 보지 않는다 — 등록은 작업자가 출입구를 지나는 순간), 웹캠2 는 작업공간과 P3 개구부만 본다 — **대기공간(Z-05)은 두 카메라 모두 시야 밖이다** (I-010, 9/5 확정). 판정은 로봇과 관제 사이에서 끝난다.

| 단계 | 노드 | 하는 일 |
|---|---|---|
| 도착 | `shuttle_mission` **[주행]** | Nav2 로 `waiting_zone` waypoint 도달 → `shuttle/status = WAITING` 발행 |
| 판정 | `fleet_manager` **[관제]** | 그 시각을 `waiting_at` 에 저장, `t_lim_s` 와 비교 → `missions.result` 확정 |
| 보고 | `kpi_report.py` **[웹캠]** | DB 를 읽어 표로 출력. 판정에 관여하지 않는다 |

BRD 5.2 도 K-05 의 측정 출처를 **「미션 로그」** 로 적는다. 웹캠2 가 지상 진실이 되는 것은 K-01·K-03(오추종·ID Swap)과 접촉 판정뿐이다 (BRD 4.1).

**원래 한계 — 대기공간 도착에는 독립 검증 수단이 없었다.**

베이 도착은 ArUco 로 독립 확인한다 — 로봇이 도착했다고 보고해도 마커 ID 가 다르면 오도착이다 (FR-08, 오도착 0건). **대기공간에는 카메라가 닿지 않으므로 로봇의 자기 보고가 유일한 근거였다.**

**두 가지 검사로 메운다 (9/6 결정).** 둘 다 `fleet_manager` 가 한다 — 이미 `amcl_pose` 를 구독하고 있고 `arena.yaml` 을 읽으면 되므로 **주행 코드 변경도 계약(토픽) 변경도 없다.**

| 검사 | 무엇을 잡나 | 방법 |
|---|---|---|
| **① 측위 검증** | **AMCL 발산** | 로봇이 P1 을 나와 남쪽으로 접근하는 동안 **ArUco ID 4**(6절)가 정면에 들어온다. 그 순간의 `aruco/distance`(카메라 실측)와 `\|amcl_pose − 마커 map 좌표\|`(AMCL 의 믿음)를 비교한다. **차이 ≤ 10 cm 면 측위 정상** → `localization_verified`, 30 cm 이상이면 AMCL 이 밀린 것 → `localization_suspect` 이벤트 |
| **② 좌표 검증** | `arena.yaml` 오타 · 잘못된 전이 | `WAITING` 수신 시 `amcl_pose` 가 `waiting_zone` 근처인지 확인, 벗어나면 `result='FAIL_WRONG_POSE'` |

①은 마커가 보이는 **아무 지점**에서나 유효하다 — 정차 위치일 필요가 없다. 접근 중 0.5~1 m 앞에서 측위가 맞았으면 그 뒤 몇십 cm 동안 AMCL 이 갑자기 틀어지진 않는다. `amcl_pose` 만으로는 로봇의 믿음을 같은 로봇의 믿음과 비교하는 것이라 발산을 못 잡는데, 마커는 카메라가 실제 세계의 인쇄물을 보는 것이라 측위와 독립이다. ①이 "측위가 맞다", ②가 "목표 좌표로 갔다" 를 각각 보장하므로 둘이 합쳐지면 "실제로 그 자리에 있다" 가 된다.

- **기록 방식이다.** 마커를 못 읽어도 `WAITING` 은 그대로 나가고 미션은 진행한다. 보고할 때 "배송 20회 중 성공 19, **그중 측위 확인 19**" 로 적는다. 못 읽은 회차는 조사 대상이다.
- **정차 자세는 서향(작업공간 방향)을 유지한다** (9/6, 시연 고려). 그래서 ①은 정차 후가 아니라 **남향 접근 구간**에서 일어난다. 작업자는 P2(남쪽)로 오므로 **S5 페어링 때 RC카를 로봇 정면(서쪽)으로 돌아 들어오게** 조종한다 — 시험 절차에 적는다.
- 위치의 근거 — 정차 정면(서쪽 1 m)은 추종 경로라 안 되고, 로봇 뒤쪽 벽은 180° 돌아 봐야 해서 T_lim 안에서 3~5 s 를 먹는다. 웹캠1 박스 북면은 robot3 이 1.5 m 로 한계선이라 **0.4 m 앞으로 당겼다** (그 사이는 평지, 작업자 동선에서 0.6 m 서쪽).

---

## 10. 명시 페어링 폴백 (FR-13) — v1.12

로봇이 재식별 게이트를 못 넘으면 따라가지 않고 그 자리에 선다(FR-12). 멈추기만 하면 미션이 안 나가므로,
**미달이 30 s 지속되면 운영자가 로봇과 작업자를 직접 짝지어 주는 길**을 연다. 관제가 9/7 에 올린 8개 질문에
대한 PM 결정을 그대로 옮긴 것이다.

**흐름** — `docs/diagrams/fr13_manual_pairing.svg` 와 같다.

```
게이트 미달 30 s → target/pair_state = GATE_FAIL → control_ui 에 버튼
  → 운영자가 robot_ns + worker_id 선택 → fleet_manager 가 /robotN/manual_pair 호출
  → amr_detection 이 확정 → target/paired = True · target/worker_id
  → follow_target 추종 시작 + Intervention(MANUAL_PAIR) 1건 기록
```

**10.1 30 s 는 `amr_detection` 이 잰다.** 관제가 재면 "사람이 없어서 안 잡히는 것" 과 "사람은 보이는데
못 알아보는 것" 을 구분할 수 없어, 아무도 없을 때도 폴백 버튼을 띄운다. 그 둘을 가릴 수 있는 곳은 게이트
판정을 하는 노드뿐이다. 그래서 `target/pair_state` 의 값이 셋이다 — `NO_TARGET` 일 때는 30 s 를 세지 않는다.

**10.2 알림은 `target/pair_state`(String) 로 한다.** `approach/status` 로 대신하지 않는다. 그것은
`follow_target` 의 주행 상태라 소유자가 다르고, 게이트 판정 결과도 30 s 조건도 담고 있지 않다.

**10.3 요청은 서비스다.** `srv/ManualPair` — 요청 `robot_ns`, `worker_id` / 응답 `accepted`, `reason`.
토픽이면 눌렀는데 됐는지 알 수 없다. **두 필드 모두 필수다** — 어느 로봇이 누구를 따라갈지가 곧 페어링이라
하나만으로는 성립하지 않는다.

관제 내부 경로는 `control_ui → /fleet/manual_pair → fleet_manager` 다. `fleet_manager` 는 요청의
`robot_ns` 를 확인한 뒤 같은 `srv/ManualPair` 형식으로 해당 `/robotN/manual_pair` 를 호출하고,
`amr_detection` 의 `accepted` 와 `reason` 을 `control_ui` 에 그대로 돌려준다. 서비스가 준비되지
않았거나 응답 시간이 지나면 관제는 성공으로 기록하지 않는다.

**10.4 확정은 `amr_detection` 이 한다.** `target/paired` 와 `target/worker_id` 의 **발행자는 계약상
amr_detection 하나뿐이며, 명시 페어링에서도 바뀌지 않는다.** 발행자가 둘이 되면 로그만 보고 어느 값이
이겼는지 가릴 수 없다. `control_ui` 는 버튼과 표시, `fleet_manager` 는 전달만 맡는다.

**10.5 수동이 자동을 덮어쓴다. 단 이미 페어링됐으면 거부한다.**

| 그때 상태 | 처리 | `reason` |
|---|---|---|
| 미페어링 (`GATE_FAIL`) | 수동 요청을 받아 확정한다 — 폴백의 목적이다 | — |
| 이미 `PAIRED` | **거부** — 추종 중 대상 교체는 오페어링 0건(AC-04)을 흔든다. 바꾸려면 `mission/finish` 로 먼저 푼다 | `ALREADY_PAIRED` |
| 대상이 안 보임 (`NO_TARGET`) | 거부 | `NO_TARGET` |
| 대상이 1 m 밖 | 거부 — **FR-14 의 판정 조건을 명시 페어링도 지킨다.** 멀면 좌표 오차가 커져 수동이든 자동이든 결과가 같다 | `TOO_FAR` |
| `robot_ns` 가 서버 자신과 다름 | 거부 | `WRONG_ROBOT` |
| 요청 전에 게이트가 스스로 통과함 | 거부 (이미 자동으로 짝지어졌다) | `GATE_OK` |

**10.6 유효 범위는 현재 미션뿐이다.** `mission/finish` 를 받으면 `amr_detection` 이 명시 페어링을 해제한다
(`follow_target` 의 즉시 해제와 같은 신호다). 새 미션은 자동 게이트부터 다시 시작한다. 도킹·미션 종료 뒤에
수동 지정이 남아 있으면 다음 미션에서 엉뚱한 사람을 따라간다.

**10.7 폴백 발동은 개입 1건이다.** `control_ui` 가 `/fleet/intervention` 에 `kind = MANUAL_PAIR` 로
발행한다. **IPH(AC-13) 분자에 넣는다** — 개입에서 빼는 것은 `SAFETY_POLICY` 하나뿐이다.
이유가 단순한 기록이 아니다. FR-13 의 판정 기준이 **오페어링 0건 AND 폴백 발동률 ≤ 20 %** 인데,
상한이 없으면 게이트를 극단적으로 빡빡하게 잡고 매번 사람에게 넘기는 '판단하지 않는 시스템' 이 만점을
받는다. 개입으로 세야 그 상한이 실제로 작동하고, 발동률과 IPH 가 같은 원천(`events`)을 쓴다.

**10.8 적용 구간은 최초 페어링(S5)과 재탐색(E-1) 둘 다다.** 근거인 BR-T16 이 "추종 개시·재개는 게이트를
통과한 경우에만" 으로 개시와 재개를 함께 적었다. 다만 재탐색 때는 이미 `worker_id` 가 정해져 있으므로,
수동 요청의 뜻은 "대상을 바꾼다" 가 아니라 **"지금 보이는 이 사람이 그 작업자다" 를 확정하는 것**이다.
그래서 재탐색 중의 요청은 `worker_id` 가 진행 중인 미션의 것과 같아야 하고, 다르면 `WRONG_ROBOT` 과
같은 급으로 거부한다.

**담당** — `target/pair_state` 발행과 30 s 계측, `manual_pair` 서버, 확정 발행은 **인지(정정인·민범진)**.
버튼·표시와 `MANUAL_PAIR` 개입 발행은 **관제(김민준)**. 서비스 호출 전달은 `fleet_manager`(관제).
`srv/ManualPair` 정의와 이 절은 **PM**.
