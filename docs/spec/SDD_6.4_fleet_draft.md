# SDD 6.4 관제 (Fleet) — 초안 v0.1 (2026-09-09)

> **이 문서의 성격.** SDD v0.14 의 6.4 절이 `[작성 필요]` 로 비어 있어, **현재 `fleet_manager_node.py`
> 코드에서 실제 동작을 역추출해 적은 것**이다. 설계를 새로 정한 것이 아니라 **지금 코드가 하고 있는 일**의
> 기술이다. 담당(김민준)이 읽고 ① 의도와 다른 곳 ② 계약과 어긋난 곳을 표시하면, 그 차이가 곧 할 일 목록이 된다.
>
> 근거: BRD v3.5 4장·6.3 · SDD v0.14 2.4(FR)·4.2·5.3 · `docs/interfaces.md` 계약 v1.17 ·
> `mro_fleet/` 코드 (2026-09-09 `8094394` 기준).

---

## 6.4.1 요구사항 (SDD 2.4 인용)

| FR | 요구 | 이 절에서 다루는 곳 |
|---|---|---|
| FR-03 | 등록 순서대로 로봇 배정, DB `workers` 기록. 동시 검출 시 결정론적 순서 | 6.4.4 |
| FR-05 | UI 에서 작업자·베이 입력 → `RequestMission` → 접수 표시 ≤ 2 s | 6.4.5 |
| FR-06 | 배터리 임계 미만이면 미션 거부, 사유 기록 | 6.4.5 |
| FR-26 | 로봇별 상태를 `FleetStatus` 로 1 Hz 발행 (**뷰어**) | 6.4.7 |
| FR-27 | `RETURN` / `RETURN_ALL` 제공. **STOP·RESUME 은 거부** | 6.4.8 |
| FR-28 | 상태 전이·개입·실패를 DB `events` 에 ROS 시각으로 기록 | 6.4.9 |
| FR-33 | 종료 발행 → 도킹 완료 시간·재시도 기록 | 6.4.9 |
| FR-35 | ArUco ID 4 로 대기공간 도착의 측위 검증 | 6.4.10 **미구현** |
| FR-13 | 게이트 미달 30 s 지속 시 명시 페어링 폴백 | 6.4.6 |

**역할 경계 (BRD 4장).** 관제는 **단순 뷰어**다. 값을 만들지 않고 모은다.
경로 원격 조종·운영자 정지/재개는 Phase 1 범위 밖이며, **관제 PC 가 죽어도 로봇의 정지 동작은 유지된다**
(안전 계층 원칙). 관제는 안전 경로에 없다.

---

## 6.4.2 아키텍처 (모듈)

| 모듈 | 파일 | 역할 |
|---|---|---|
| `fleet_manager` | `nodes/fleet_manager_node.py` | 배정 · 미션 상태 전이 · 집계 · DB 기록 |
| `control_ui` | `nodes/control_ui_node.py` | Flask 대시보드 (뷰어 + 버튼) |
| DB | `core/db.py` · `config/schema.sql` | SQLite `~/mro_fleet.db` |

**`zone_semaphore` 는 이 절이 아니다.** 패키지는 `mro_fleet` 안에 있지만 담당은 주행이고 설계는 6.3 이다
(9/5 이관). 관제는 그 결과인 `/fleet/zone_event` 를 **받아 DB 에 적기만** 한다.

---

## 6.4.3 데이터 흐름 (입력 · 처리 · 출력)

**입력**

| 토픽·서비스 | 타입 | 쓰임 |
|---|---|---|
| `/webcam1/worker_descriptor` | WorkerDescriptor | 작업자 등록 → 로봇 배정 |
| `robotN/shuttle/status` | String | 셔틀 단계 미션 전이 · 화면 상태 |
| `robotN/approach/status` | String | 추종 단계 미션 전이 · 화면 상태 |
| `robotN/target/paired` | Bool | `PAIRED` 전이 |
| `robotN/target/worker_id` | String | 페어링된 작업자 이름 |
| `robotN/target/pair_state` | String | `PAIRED`/`GATE_FAIL`/`NO_TARGET` 보관 |
| `robotN/battery_state` | BatteryState | FR-06 배정 거부 판정 |
| `robotN/amcl_pose` | PoseWithCovarianceStamped | `FleetStatus` 의 x·y·yaw |
| `robotN/safety/estop` | Bool | 화면 표시 · `ESTOP` 이벤트 |
| `/fleet/intervention` | Intervention | `INTERVENTION` 이벤트 |
| `/fleet/zone_event` | String | `ZONE` 이벤트 (K-06 원천) |
| `/fleet/request_mission` | srv RequestMission | 미션 생성 |
| `/fleet/operator_command` | srv OperatorCommand | RETURN / RETURN_ALL |
| `/fleet/manual_pair` | srv ManualPair | FR-13 폴백 입구 |

**출력**

| 토픽·서비스 | 타입 | 시점 |
|---|---|---|
| `/fleet/mission` | Mission | 미션 상태가 바뀔 때마다 |
| `/fleet/status` | FleetStatus | **1 Hz 타이머** |
| `robotN/mission/assigned` | Mission | `RequestMission` 수락 시 1회 |
| `robotN/mission/finish` | Bool | `RETURN`/`RETURN_ALL` 시 |
| `robotN/manual_pair` | srv ManualPair | 폴백 확정 요청 |

---

## 6.4.4 작업자 배정 (FR-03)

> **[낡음 — 9/10 PR #73 으로 뒤집혔다]** 아래 절차는 `WorkerDescriptor` 를 받는 순간 배정하던 옛 구조다. 지금은 **등록과 배정이 분리**되어, 수신 시에는 작업자만 등록하고 **`RequestMission` 을 수락할 때** 미션·E-stop 이 없고 배터리 기준을 넘는 AMR 을 배정한다 (계약 v1.19). `worker_assignment_window_s` 파라미터도 코드·`fleet.yaml` 에서 사라졌다. **이 절을 보고 구현하지 말 것.** 이 문서는 PR #70 에서 당시 코드를 역추출한 초안이고, 다시 쓰는 것은 관제 몫이다.

1. `/webcam1/worker_descriptor` 수신. 이미 배정된 `worker_id` 면 무시한다.
2. 배정 가능한 로봇이 없으면 경고만 남기고 끝낸다.
3. **1 초 배치 창**(`worker_assignment_window_s`)에 모았다가 한 번에 확정한다.
   대기 인원이 로봇 수 이상이 되면 창을 기다리지 않고 즉시 확정한다.
4. 확정은 **`worker_id` 오름차순**으로, 비어 있는 로봇을 목록 순서대로 준다.
   → *"동시 검출 시 순서 재현"*(FR-03)을 이 정렬이 보장한다. 도착 순서에 의존하지 않는다.
5. DB `workers` 에 `upsert_worker(worker_id, outfit_class, registered_at, robot_ns)`.

로봇 명단은 `robot_namespaces` 파라미터(`params/fleet.yaml`)다. **코드는 대수를 가정하지 않는다** —
배정·집계·`RETURN_ALL` 이 모두 이 목록을 순회한다.

---

## 6.4.5 미션 생성과 거부 (FR-05 · FR-06)

`RequestMission(worker_id, bay)` 처리 순서:

1. 그 작업자가 배치 창에 대기 중이면 먼저 확정한다(배정 없이 미션이 생기는 것을 막는다).
2. 배정된 로봇이 없으면 `accepted=False`, `"미등록 작업자"`.
3. **배터리 < `battery_min`(0.25) 이면 거부** — `accepted=False`,
   events 에 `ASSIGN_REFUSED_BATTERY`(value=배터리) 기록. (FR-06)
4. 통과하면 `create_mission` → **`Mission.ASSIGNED` 로 즉시 set_state** →
   `/fleet/mission` 과 `robotN/mission/assigned` 발행.
5. 로봇의 상태 캐시를 초기화한다 — `approach_state`·`shuttle_state` = `UNKNOWN`,
   `status_owner` = None. **이전 미션의 잔상이 새 미션에 새지 않게 하는 장치다.**

> `Mission.CREATED`(0) 는 **쓰지 않는다.** 생성 즉시 `ASSIGNED` 로 간다.
> (`core/db.py:41` 주석이 "CREATED 상태의 미션을 만들고" 라고 되어 있으나 코드와 다르다 — 6.4.11 ②)

---

## 6.4.6 미션 상태 전이 — **이 절이 핵심이다**

미션 상태는 **9개 중 8개**를 쓴다(`CREATED` 제외). 전이를 일으키는 입력은 셋뿐이다.

### 전이표

| 입력 | 값 | → 미션 상태 | 함께 하는 일 |
|---|---|---|---|
| `RequestMission` | 수락 | `ASSIGNED` | DB `create_mission`+`set_state` · `mission/assigned` 발행 |
| `shuttle/status` | `SHUTTLE` | `SHUTTLE` | DB `set_state` |
| `shuttle/status` | `AT_BAY` | `AT_BAY` | DB `set_state` |
| `shuttle/status` | `WAITING` | `WAITING` | DB **`mark_waiting`** (리드타임·T_lim 판정). 늦었으면 `arrived_late` 이벤트 |
| `target/paired` | `True` | `PAIRED` | `paired_worker` 갱신 · `PAIRED_WORKER` 이벤트 |
| `approach/status` | `FOLLOWING` | `FOLLOWING` | DB `set_state` |
| `approach/status` | `STANDBY` | `STANDBY` | DB `set_state` |
| `shuttle/status` | `DOCKED` | **`DONE`** (종결) | DB `set_state` |
| `shuttle/status` | `FAILED` | **`FAILED`** (종결) | DB `set_state(result='FAIL_ROBOT')` |

**전이를 일으키지 않는 입력** — 화면 표시와 이벤트 기록만 한다:
`READY` · `RETURNING` · `DOCK_WAIT` · `DOCKING` (셔틀) ·
`IDLE` · `SEARCHING` · `APPROACH_1` (접근) ·
`RETURN`/`RETURN_ALL` 명령 · T_lim 초과.

> **접근 상태는 8개 중 5개만 실제로 온다** (계약 v1.16·v1.17). `APPROACH_2`·`ARRIVED` 는 기본
> 파라미터에서 조건이 겹치지 않아 발행되지 않고, `FAILED` 는 `follow_target` 이 내지 않는다
> (estop·`mission/finish` 로 멈추는 것은 정상 종료이며 `IDLE` 로 간다, I-026).
> **셋 다 '안 나오는 것이 고장이 아니다'.** 이 값을 기다리는 분기를 관제에 두면 영원히 안 온다.
> 값 목록에서 뺀 것은 아니다 — `approach_skip_m` 한 줄이면 되살아난다.

### 전이 가드 — 되돌림 방지

두 출처(셔틀·접근)가 동시에 상태를 보내므로, 늦게 온 옛 상태가 단계를 되돌리지 않게 막는다.

1. 현재가 **`DONE`** 이면 어떤 전이도 없다.
2. **접근발 전이**는 현재가 `WAITING`·`PAIRED`·`FOLLOWING`·`STANDBY` 일 때만 허용.
   또 **복귀 우선권**(셔틀이 `RETURNING`/`DOCK_WAIT`/`DOCKING`/`DOCKED`/`FAILED`)이면 거부.
   → 복귀 시작 후 늦게 도착한 `FOLLOWING` 이 미션을 되살리지 못한다.
3. **셔틀발 전이** 중 `SHUTTLE`·`AT_BAY`·`WAITING` 은, 현재가 이미 접근 단계
   (`PAIRED`/`FOLLOWING`/`STANDBY`)면 거부.

### 화면에 무엇을 보일지 (`status_owner`)

`FleetStatus.state` 는 미션 상태가 아니라 **두 상태 출처 중 하나의 원문**이다. 고르는 규칙:

- 셔틀이 복귀 상태이거나 `FAILED` → `shuttle` (미션이 `DONE` 이 아닌 한)
- 셔틀이 출고 상태(`READY`/`SHUTTLE`/`AT_BAY`/`WAITING`)이고 미션이 접근 단계가 아니면 → `shuttle`
- 그 밖에, 복귀 우선권이 없고 미션이 `DONE` 이 아니며 접근 상태가 활성이면 → `approach`
- 아무 출처도 안 정해졌으면 `approach_state`, 그것도 없으면 `shuttle_state`

---

## 6.4.7 상태 집계와 발행 (FR-26)

**1 Hz 타이머**가 로봇마다 `FleetStatus` 를 하나씩 낸다:
`state`(6.4.6 규칙) · `battery` · `x`·`y`·`yaw`(amcl_pose) · `mission_id` · `paired_worker` · `estop`.

BR-T08 의 "갱신 ≤ 2 s" 를 1 Hz 로 만족시킨다. **집계만 하고 판단하지 않는다.**

---

## 6.4.8 운영자 명령 (FR-27)

- 허용은 **`RETURN`**(해당 로봇) 과 **`RETURN_ALL`**(전체) 뿐. 그 밖은 `ok=False` 로 **거부**한다
  (STOP·RESUME 은 9/4 결정으로 범위 밖).
- `RETURN` 은 알 수 없는 `robot_ns` 면 거부.
- 처리: 대상마다 events 에 `OPERATOR` 기록 → `robotN/mission/finish` 에 `True` 발행.
- **미션 상태는 바꾸지 않는다.** `DONE` 은 로봇이 실제로 도킹해 `shuttle/status = DOCKED` 를 낼 때만 온다.
  → 종료 버튼은 "복귀 지시" 이지 "미션 종결" 이 아니다.

---

## 6.4.9 DB 반영 시점 (FR-28 · FR-33)

**`workers`** — 배정 확정 시 1회.

**`missions`** — `create_mission`(ASSIGNED) → 전이마다 `set_state` →
`WAITING` 에서 `mark_waiting`(리드타임·`result` 확정) → `FAILED` 는 `result='FAIL_ROBOT'`.

**T_lim 처리 (계약 7절).** 1 Hz 타이머가 `expire_overdue` 를 돌려 `created_at + t_lim < now` 이고
`result` 가 비어 있는 미션을 **`result='FAIL_TLIM'` 으로 한 번만** 확정하고 `TLIM_EXCEEDED` 이벤트를 남긴다.
**미션 `state` 는 바꾸지 않고 운행도 멈추지 않는다** — 초과 후 도착하면 `arrived_late` 이벤트로 따로 남긴다.
계약이 요구한 "이중 계상하지 않는다" 가 이 구조로 지켜진다.

**`events`** — `kind` 별 기록 시점:

| kind | 언제 | value / note |
|---|---|---|
| `STATE` | 셔틀·접근 상태 문자열이 **바뀔 때마다** | value=상태, note=출처(`shuttle`/`approach`) |
| `ESTOP` | estop 값 변화 | value=bool |
| `PAIRED_WORKER` | 페어링 작업자 변화 | value=worker_id |
| `INTERVENTION` | `/fleet/intervention` 수신 | **메시지의 원 시각**을 쓴다(유효하지 않으면 현재 시각) |
| `ZONE` | `/fleet/zone_event` 수신 | value=REJECT/GRANT/RELEASE/RECLAIM, note=zone_id |
| `ASSIGN_REFUSED_BATTERY` | FR-06 거부 | value=배터리 |
| `TLIM_EXCEEDED` | T_lim 초과 확정 | value=mission_id |
| `OPERATOR` | RETURN/RETURN_ALL | value=명령 |
| `arrived_late` | T_lim 후 도착 | value=리드타임(s) |

`/fleet/zone_event` 는 형식(3토막)·구역 ID·로봇 이름·동작 값을 **모두 검증**하고, 어긋나면 경고만 남기고
버린다. **개입으로 계상하지 않는다** — 섞으면 IPH 분자가 부풀어 AC-13 이 실제보다 나빠 보인다 (계약 v1.11).

---

## 6.4.10 명시 페어링 폴백 (FR-13) · 측위 검증 (FR-35)

**FR-13** — 30 s 는 관제가 세지 않는다. `amr_detection` 이 재서 `target/pair_state = GATE_FAIL` 을 낸다
(계약 10.1: 관제는 "사람이 없는 것"과 "못 알아보는 것"을 구분할 수 없다).
`control_ui` 가 `robotN/target/pair_state` 를 **직접 구독**해 버튼을 띄우고,
`/fleet/manual_pair` → `fleet_manager` → `robotN/manual_pair` 로 전달한다. 폴백 1건은 개입으로 계상한다.

**FR-35 — 미구현.** `fleet_manager` 는 `aruco/id`·`aruco/distance` 를 구독하지 않는다.
**의도된 대기 상태다**: `arena.yaml` 의 `loc_check_marker.xy` 가 아직 `[0.0, 0.0]`(맵 원점)이라,
지금 켜면 측위가 정상인데도 전부 `localization_suspect` 로 찍힌다.
주행의 실측이 들어온 뒤 켠다 (todo `mro_fleet` 절).

---

## 6.4.11 지금 코드와 계약이 어긋나는 곳

| # | 내용 | 판단 |
|---|---|---|
| ① | `fleet_manager` 가 `target/pair_state` 를 구독해 보관하지만 **아무 데서도 쓰지 않는다.** `FleetStatus` 에 그 필드가 없어 화면에도 안 나간다 | **죽은 구독.** `FleetStatus` 에 필드를 넣든지, 구독을 지우든지 택일 |
| ② | `core/db.py:41` 주석이 "CREATED 상태의 미션을 만들고" 라고 하지만 실제로는 즉시 `ASSIGNED` | 주석을 고친다 (동작은 정상) |
| ③ | `approach/status` 8개 값 중 **실제로 오는 것은 5개**다 — `IDLE`·`SEARCHING`·`APPROACH_1`·`FOLLOWING`·`STANDBY`. `FAILED` 는 `follow_target` 이 내지 않고(estop·finish 는 실패가 아니다, I-026), `APPROACH_2`·`ARRIVED` 는 기본 파라미터에서 조건이 겹치지 않아 안 나온다 | **어긋남이 아니다.** 계약 v1.16·v1.17 이 이미 명시했고 **값 목록 8개는 일부러 그대로 둔다** — `approach_skip_m` 한 줄이면 되살아나므로. 관제가 할 일은 **이 세 값을 기다리는 분기를 두지 않는 것** |
| ④ | FR-35 미구현 | 주행 실측 대기 — 순서상 정상 |
| ⑤ | 대시보드 JS 의 `uiState.robots` 가 `{robot3, robot8}` 고정 사전이라 `robot_namespaces` 를 바꾸면 화면만 어긋난다 | todo 에 등록됨 (PR #51 후속, 마감 9/10) |

---

## 6.4.12 파라미터

`mro_bringup/params/fleet.yaml` (관제 전용) + `common.yaml`(로봇과 공유하는 계약값):

| 이름 | 값 | 출처 | 뜻 |
|---|---|---|---|
| `robot_namespaces` | `[/robot3, /robot8]` | fleet.yaml | **로봇 명단의 단일 출처** |
| `battery_min` | 0.25 | fleet.yaml | 이 미만이면 배정 거부 (FR-06) |
| `worker_assignment_window_s` | 1.0 | fleet.yaml | 동시 등록으로 볼 시간창 |
| `t_lim_s` | 120.0 | **common.yaml** | 미션 제한시간. **로봇과 같은 값이어야 한다** — 갈라지면 로봇은 계속 가는데 관제만 `TLIM_EXCEEDED` 를 찍는다 |
| `db_path` | `~/mro_fleet.db` | 코드 기본값 | 런치가 주는 경로 |

---

## 6.4.13 실패 시 동작

| 상황 | 동작 |
|---|---|
| 로봇 상태 토픽이 끊김 | 마지막 값을 유지한다. 별도 타임아웃 없음 `[검토 필요]` |
| `zone_event` 형식 오류 | 경고 로그 후 버린다. DB 에 적지 않는다 |
| `pair_state` 값이 셋 중 하나가 아님 | 경고 로그 후 무시 |
| 미등록 작업자로 미션 요청 | `accepted=False`, DB 기록 없음 |
| DB 쓰기 실패 | `[검토 필요]` — 현재 예외 처리 없음 |
