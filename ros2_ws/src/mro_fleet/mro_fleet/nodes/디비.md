# 관제 DB 저장 항목

## 개요

`FleetManager`는 SQLite DB에 작업자, 미션, 이벤트를 저장한다. 기본 DB 파일은 `~/mro_fleet.db`이며 ROS 2 파라미터 `db_path`로 변경할 수 있다.

- 스키마: `ros2_ws/src/mro_fleet/config/schema.sql`
- DB 접근 코드: `ros2_ws/src/mro_fleet/mro_fleet/core/db.py`
- 실제 기록 주체: `ros2_ws/src/mro_fleet/mro_fleet/nodes/fleet_manager_node.py`

DB에는 ROS 메시지 전체를 그대로 저장하지 않는다. `FleetManager`가 필요한 필드만 테이블에 저장하거나, 상태 변경을 이벤트 한 건으로 기록한다.

## workers 테이블

웹캠1에서 등록된 작업자 정보를 저장한다. AMR은 미션 요청 시 배정하므로 이 테이블에 저장하지 않는다.

| 컬럼 | 저장 내용 | 원천 |
|---|---|---|
| `worker_id` | 작업자 ID, 예: `W1`, `W2` | `/webcam1/worker_descriptor.worker_id` |
| `outfit_class` | 복장 클래스, 예: `worker_a`, `worker_b` | `WorkerDescriptor.outfit_class` |
| `registered_at` | 등록 시각, ROS 시각 기준 초 | FleetManager 수신 시각 |

`color_hist`는 `WorkerDescriptor`에 있지만 현재 DB에는 저장하지 않는다.

## missions 테이블

자재 요청 한 건의 생성부터 종료까지를 저장한다.

| 컬럼 | 저장 내용 | 저장 시점 |
|---|---|---|
| `mission_id` | `M<ROS시각>-<16진수 4자리>` 형식의 미션 ID | 자재 요청 수락 시 |
| `worker_id` | 미션을 요청한 작업자 | 자재 요청 수락 시 |
| `robot_ns` | 미션에 배정된 AMR | 자재 요청 수락 시 |
| `bay` | 목표 창고 `A`, `B`, `C` | 자재 요청 수락 시 |
| `state` | `Mission.msg` 상태 정수 | 생성 및 상태 전이 시 |
| `created_at` | 미션 생성 ROS 시각, 초 | 자재 요청 수락 시 |
| `waiting_at` | 대기 공간에 처음 도착한 ROS 시각 | `WAITING` 첫 전이 시 |
| `finished_at` | `DONE` 또는 `FAILED`가 처음 확정된 ROS 시각 | 종료 상태 전이 시 |
| `result` | 미션 판정 결과 | 도착, 시간 초과 또는 로봇 실패 시 |
| `note` | 미션 부가 설명 | `set_state()` 호출 시 값이 있을 때 |

### state 값

| 값 | 이름 | 의미 |
|---:|---|---|
| 0 | `CREATED` | 미션 생성 |
| 1 | `ASSIGNED` | AMR 배정 |
| 2 | `SHUTTLE` | 창고로 이동 중 |
| 3 | `AT_BAY` | 창고 도착·체류 중 |
| 4 | `WAITING` | 작업자 대기 공간 도착 |
| 5 | `PAIRED` | 작업자 재식별 완료 |
| 6 | `FOLLOWING` | 작업자 추종 중 |
| 7 | `STANDBY` | 작업자 Lost 후 대기 |
| 8 | `DONE` | 미션 완료 |
| 9 | `FAILED` | 미션 실패 |

### result 값

| 값 | 현재 기록 조건 |
|---|---|
| `SUCCESS` | `WAITING` 첫 도착이 `t_lim_s` 이내 |
| `FAIL_TLIM` | 첫 도착이 `t_lim_s`를 넘겼거나 미도착 상태로 제한 시간 초과 |
| `FAIL_ROBOT` | 로봇 상태가 미션 `FAILED`로 전이 |

스키마 주석의 `FAIL_WRONG_BAY`, `FAIL_INTERVENTION` 등은 예시이며, 현재 `FleetManager` 코드에서는 기록하지 않는다.

## events 테이블

상태 변경·운영자 개입·안전·구역 이벤트를 시각 순서로 쌓는다.

| 컬럼 | 저장 내용 |
|---|---|
| `id` | SQLite 자동 증가 ID |
| `t` | 이벤트 ROS 시각, 초 |
| `robot_ns` | 관련 AMR namespace |
| `kind` | 이벤트 종류 |
| `value` | 이벤트의 주요 값 |
| `note` | 출처·구역·설명 등 부가 정보 |

### 현재 기록하는 이벤트

| kind | value | note | 발생 조건 |
|---|---|---|---|
| `STATE` | 변경된 상태 문자열 | `approach` 또는 `shuttle` | `/fleet/status` 안의 원본 상태가 변경 |
| `ESTOP` | `True` / `False` | 빈 문자열 | E-stop 값이 변경 |
| `PAIRED_WORKER` | 작업자 ID 또는 빈 문자열 | 빈 문자열 | 페어링 작업자가 변경 |
| `INTERVENTION` | `Intervention.kind` | `Intervention.note` | `/fleet/intervention` 수신 |
| `ZONE` | `REJECT`, `GRANT`, `RELEASE`, `RECLAIM` | 구역 ID `P1` 또는 `DOCK` | `/fleet/zone_event` 수신 |
| `arrived_late` | 리드타임, 초 | 빈 문자열 | `WAITING` 첫 도착이 `t_lim_s` 초과 |
| `TLIM_EXCEEDED` | 미션 ID | 빈 문자열 | 미도착 상태에서 `t_lim_s` 초과 |
| `ASSIGN_REFUSED_BATTERY` | 배터리 잔량 | 빈 문자열 | 배터리 임계 미만으로 미션 배정 거부 |
| `OPERATOR` | `RETURN` 또는 `RETURN_ALL` | 빈 문자열 | 운영자가 작업 종료·복귀 명령 |

`schema.sql`에는 `LOST`, `REACQUIRED` 등이 예시로 적혀 있지만, 현재 `FleetManager`는 해당 `kind`를 DB에 기록하지 않는다.

## `/fleet/status`에서 DB로 들어가는 값

`FleetStatus` 전체가 하나의 DB 행으로 저장되지는 않는다.

| FleetStatus 필드 | DB 저장 | 사용 방식 |
|---|---|---|
| `robot_ns` | 간접 저장 | 이벤트와 미션의 로봇 식별자 |
| `approach_state` | 변경 시 저장 | `events.kind=STATE`, `note=approach`; 미션 전이 판정 |
| `shuttle_state` | 변경 시 저장 | `events.kind=STATE`, `note=shuttle`; 미션 전이 판정 |
| `estop` | 변경 시 저장 | `events.kind=ESTOP` |
| `paired_worker` | 변경 시 저장 | `events.kind=PAIRED_WORKER`; `PAIRED` 미션 전이 판정 |
| `battery` | 조건부 저장 | 평소에는 메모리만 갱신. 잔량 부족으로 배정을 거부할 때만 `ASSIGN_REFUSED_BATTERY` 이벤트로 저장 |
| `state` | 미저장 | FleetManager가 DB 기록에 직접 사용하지 않음 |
| `pair_state` | 미저장 | FleetManager 메모리에만 보관 |
| `x`, `y`, `yaw` | 미저장 | FleetManager 메모리에만 보관 |
| `mission_id` | 미저장 | FleetManager는 자신이 생성한 미션 ID를 별도로 관리 |
| `header` | 미저장 | 수신 처리에서 사용하지 않음 |

## DB가 아닌 메모리에만 보관하는 값

`FleetManager.robot[robot_ns]`에는 다음 값이 실행 중에만 존재한다.

- 접근·셔틀 최신 상태와 대표 상태 출처
- 배터리 잔량
- `x`, `y`, `yaw`
- `pair_state`
- 현재 미션 ID와 미션 상태
- 페어링된 작업자
- E-stop 여부

프로세스가 종료되면 이 메모리 값은 사라지며, 필요한 이력만 위의 `missions`·`events` 테이블에 남는다.
