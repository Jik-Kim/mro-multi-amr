# mro_navigation — AMR 주행 노드

`mro_navigation`은 로봇 한 대의 배송(S4), 작업자 추종(S5~S8), 독립 안전 정지, 복귀·도킹(S9~S10)을 담당한다. 모든 노드는 `/robot3`, `/robot8`처럼 **반드시 로봇 네임스페이스 안에서** 실행한다. 네임스페이스가 없으면 다른 로봇의 Nav2·토큰을 건드릴 수 있으므로 노드가 기동을 거부한다.

전체 계약의 기준은 [docs/interfaces.md](../../../docs/interfaces.md), 운용 순서는 [docs/setup.md](../../../docs/setup.md), 확정된 설계 판단은 [docs/SOT.md](../../../docs/SOT.md)다. `zone_semaphore_node.py`는 패키지는 `mro_fleet`에 있지만, 저장소 규칙상 주행 담당이 관리한다.

## 전체 흐름

```text
mission/assigned
  → shuttle_mission: 도크 초기 자세 → 언독 → 베이/ArUco 확인 → P1 → 대기공간
  → shuttle/arrived
  → follow_target: 재식별 게이트 → 접근/추종/재탐색/Standby

mission/finish
  ├→ follow_target: 즉시 목표 취소·IDLE
  └→ return_dock: DOCK 토큰 → P1 토큰 → 접근 지점 → 도킹

/scan → safety_watchdog → safety/estop, cmd_vel
zone_semaphore(P1, DOCK) → FIFO 상호배제·강제 회수
```

`navigator.goToPose()`, `dock()`, `undock()` 같은 navigator 호출은 ROS 콜백에서 하지 않는다. 콜백은 수신값 또는 요청 플래그만 저장하고, 각 노드의 메인 루프/상태기계가 호출한다. 그래야 executor가 이미 spin 중인 오류와 토픽 수신 정지를 피할 수 있다.

## 노드별 읽는 순서

| 노드 | 역할 | 먼저 볼 함수 | 리뷰 핵심 |
|---|---|---|---|
| `shuttle_mission_node.py` | S4 배송 | `run_mission()` | 기동 초기 자세, Nav2 성공 뒤 신선한 TF 도착 검증, ArUco 수용 조건, P1 반납이 모든 종료 경로에 있는지 |
| `follow_target_node.py` | S5~S8 접근·추종 | `step()` → `step_searching()` → `step_following()` | `target/paired`가 이동의 게이트인지, Lost/Standby/finish/estop에서 goal을 취소하는지 |
| `safety_watchdog_node.py` | LiDAR 독립 정지 | `scan_cb()` → `tick()` | 전방 부채꼴·히스테리시스, estop 때 병진은 항상 0인지, 각속도만 제한해 통과시키는지 |
| `return_dock_node.py` | S9~S10 복귀·순차 도킹 | `run_return()` | DOCK→P1 토큰 순서, 거절 시 홀딩, 재시도 중 `DOCKING` 유지, `finally` 반납 |
| `mro_fleet/nodes/zone_semaphore_node.py` | P1·DOCK FIFO 잠금 | `cb()` → `watchdog()` | 새치기 방지, 중복 이벤트 억제, 대기자 반납 제거, 60초 강제 회수 |

## 안전·계약 불변식

- 후진 금지: 어떤 노드도 `linear.x < 0`을 내지 않는다. 도킹 실패나 좁은 공간 탈출은 제자리 회전 뒤 전진으로 처리한다.
- 안전 독립성: `safety_watchdog`은 `/scan`만으로 성립해야 하며 웹캠, 재식별, Wi-Fi, 관제가 안전 판단에 들어오면 안 된다.
- 정지 우선: `mission/finish`와 `safety/estop`은 새 주행 명령보다 우선한다. `finish`는 정상 종료이므로 `FAILED`가 아니라 각 노드의 정상 상태로 돌아간다.
- 토큰 반납: 획득에 실패해도 세마포어 대기열에 등록될 수 있으므로, 요청한 토큰은 `finally`를 포함한 모든 종료 경로에서 반납한다.
- KPI 상태: `shuttle/status`, `approach/status`, `/fleet/zone_event`는 관제와 KPI 집계의 입력이다. 값·발행 시점·중복 여부를 임의로 바꾸지 않는다.
- 시간·좌표: 교차 PC 로그에는 ROS 시간을 사용하고, map 좌표는 단일 map 원점 기준이다.

## 파라미터와 실행 위치

실행값의 단일 출처는 `mro_bringup/params/common.yaml`, `robot3.yaml`, `robot8.yaml`, `arena.yaml`이다. 이 패키지 코드의 `declare_parameter()` 기본값은 런치가 값을 넘기지 못할 때의 대체값이지 확정값이 아니다.

| 위치 | 실행 노드 |
|---|---|
| 로봇 제어 PC (`/robot3`, `/robot8`) | `safety_watchdog`, `shuttle_mission`, `follow_target`, `return_dock` |
| 관제 PC-F | `zone_semaphore` |

보통 로봇 노드는 `ros2 launch mro_bringup robot.launch.py namespace:=/robot3`으로, 세마포어는 관제 PC의 `fleet_pc.launch.py`로 실행한다. 정확한 환경 변수와 기동 순서는 `docs/setup.md`를 따른다.

## 코드리뷰 전 최소 확인

1. 각 노드 파일의 모듈 주석을 먼저 읽고 위 표의 시작 함수를 따라간다.
2. 입력 토픽·출력 토픽·서비스 이름이 `docs/interfaces.md`와 일치하는지 확인한다.
3. Nav2 호출이 콜백 안에 없는지, 예외·종료·estop 경로에서 `cancelTask()` 또는 토큰 반납이 되는지 확인한다.
4. 실제 로봇에서 확인하지 않은 것은 단위 테스트 통과로 대체하지 않는다. 특히 P1 통과, 추종거리, estop 결함 주입, 도킹 반복은 실주행 항목이다.

## 단위 검증

워크스페이스를 빌드한 환경에서 다음처럼 실행한다. 이 PC에서는 pytest 플러그인 충돌을 피하기 위해 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`을 사용한다.

셔틀 3종(`test_shuttle_mission_fsm.py`, `test_shuttle_arrival.py`, `test_shuttle_tf_updates.py`)은 **직접 pytest로 부르면 실패한다.** 실물이 붙은 환경에서 도는 것을 막으려고 스스로 거부하므로, 저장소 실행기를 쓴다.

```bash
# 셔틀 3종 — 저장소 실행기 (직접 pytest 호출은 거부된다)
bash tools/test_shuttle_offline.sh

# 나머지
cd ros2_ws/src/mro_navigation
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_follow_target_fsm.py -v
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_return_dock_fsm.py -v
```

이 검증은 상태 전이·토큰·TF 처리의 회귀를 잡는다. 센서 값의 정확도, 실제 제동 거리, P1 교행, 도킹 성공률은 실물 시험 절차로 별도 확인해야 한다.

## 코드리뷰 진행 자료 (강사 안내 반영)

코드리뷰에서는 PPT를 새로 만들지 않는다. 한 대의 PC에서 **설계도 → 실제 코드 → 기능 테스트 결과** 순서로 보여 준다. 코드를 줄마다 읽기보다, 아래 표의 “중요 구현”만 실제 파일에서 열어 설계 블록과 연결해 설명한다.

| 설계·시나리오 | 실제 코드 | 중요한 구현 | 기능 검증 자료 |
|---|---|---|---|
| S4 셔틀 | `shuttle_mission_node.py` | 초기 자세 → Nav2 준비 → ArUco 확인·접근 → P1 토큰 → WAITING | `test_shuttle_mission_fsm.py`, `test_shuttle_arrival.py`, [셔틀 실주행 절차](../../../docs/shuttle_test_guide.md) |
| S5~S8 추종 | `follow_target_node.py` | paired 게이트, 점진 접근, Lost·힌트 탐색·Standby, 종료 취소 | `test_follow_target_fsm.py` |
| E-4 안전 정지 | `safety_watchdog_node.py` | `/scan` 부채꼴, 정지/해제 히스테리시스, 병진 0·각속도 제한 | TS-04·TS-14 실물 결함 주입 로그·영상 |
| S9~S10 복귀 | `return_dock_node.py` | DOCK→P1 토큰, 홀딩, 접근 지점, 도킹 재시도·반납 | `test_return_dock_fsm.py`, TS-U12·TS-20 실물 자료 |
| E-3/E-7 상호배제 | `mro_fleet/nodes/zone_semaphore_node.py` | FIFO, 중복 이벤트 억제, 대기열 제거, 60초 강제 회수 | 2대 토큰 요청 로그와 `/fleet/zone_event` |

### 발표 직전 체크

- HDMI 연결과 ROS 환경을 미리 확인하고, 한 대의 PC만 사용한다.
- `mro_navigation` 디렉터리와 `mro_fleet/.../zone_semaphore_node.py`를 파일 탐색기에 열어 둔다.
- [시스템 아키텍처](../../../docs/architecture.md)와 [주행 코드리뷰 flow diagram](../../../docs/diagrams/navigation_review_flow.drawio)을 먼저 보여 준다. 한 페이지에서 시스템 → 패키지 → 노드 → 기능 순서로 좁혀 들어가며, 설계 블록을 실제 코드의 대표 함수와 연결한다.
- 실제 솔루션 노드를 보인다. 테스트 코드는 “무엇을 검증했는지”를 보이는 증거이지 코드 설명의 대상은 아니다.
- 각 기능별로 테스트 코드, 통과 출력, 터미널 로그, 스크린샷 또는 실물 영상을 한 묶음으로 준비한다. 타 파트 토픽이 아직 없으면 더미 발행 또는 rosbag으로 주행 노드의 정상 동작만 입증한다.
- 안전 정책·ArUco 접근·P1 토큰처럼 문제를 해결한 변경은 변경 전 증상과 변경 후 결과를 나란히 준비한다.
- 메인 발표자 한 명이 전체 흐름을 설명하고, 실제 구현 담당자는 질의응답을 맡는다. 주행 담당 개발자는 모두 참석한다.

### 30분 권장 배분

1. 5분 — 주행 노드 5개, 실행 PC, 전체 시나리오·인터페이스를 설계도로 설명
2. 15분 — 위 표 순서대로 중요한 상태 전이·안전·토큰 로직을 실제 코드에서 설명
3. 7분 — 단위 테스트와 실물 시험 로그·영상으로 기능별 검증 결과를 제시
4. 3분 — 미검증 항목과 다음 실험을 투명하게 공유하고 질의응답
