# I-049 ESTOP 복구 구현과 실주행 확인

코드·오프라인 검증 단계다. 실물 탈출 성공은 아직 확인하지 않았다. 기존 `interfaces.md` v1.17의 `cmd_vel` 다중 발행 규칙과 충돌하므로 아래 내부 토픽 변경은 PM 공유·계약 반영 대상이다. 기존 계약을 확정된 것처럼 바꾸지 않는다.

## 동작

ESTOP → 외부 속도 차단·병진 0 → 모든 주행 액션 종료와 odom 정지 확인 → watchdog SAFE_ROTATE → clear 조건 충족 시 각속도 0 → odom 정지 확인 후 ESTOP 해제. 한도 도달 시에는 정지 대기한다.

- 초기 시험값: `safe_rotate_rate=0.25 rad/s`, `safe_rotate_timeout_s=5.0`, `safe_rotate_max_angle_deg=75.0`. 설정은 `mro_bringup/params/common.yaml`.
- 전방 ±35°가 `0.38 m 초과`인 스캔 5개 연속이면 조기 해제한다. 5초를 채우지 않는다.
- 시간·odom 누적 절대 회전각 중 먼저 한도에 도달하면 STUCK. 센서 결측, 회전 영역 여유 부족, 정지 확인 소실도 정지한다. 자동 회전 재시도는 없다. 이후 장애물이 제거되어 clear 조건을 충족하면 해제할 수 있다.
- 최초 기동 시 장애물이 있으면 자동으로 회전하지 않는다.
- 전체 LiDAR 회전 영역 여유 0.21 m는 footprint 외접 0.189 m 기반 시험값이다. **LiDAR 원점과 실제 외형을 포함한 여유를 실물에서 검증해야 한다.** 전체 스캔 결측 시 회전을 거부하므로 실제 bag에서 가용성도 확인한다.

## 미션과 identity

- `cancelTask()`는 해당 navigator의 Nav2 액션만 취소한다. 정지 확인 후에만 회전을 허용한다. 도킹·언독은 별도 액션 핸들로 취소한다.
- 사람 추종: ESTOP 직전 worker_id를 보존한다. 빈 ID·다른 작업자 ID로 담당을 바꾸지 않는다. clear 이후 같은 ID·paired를 다시 확인하고, clear 이후 타임스탬프를 가진 신선한 map 좌표로 목표를 새로 계산한다. 기존 좌표·평활 추정은 goal 생성 경로에서 폐기한다. 마지막 map 위치는 탐색 참고용으로만 보존하며 회전 이후 현재 로봇 자세에서 방위를 다시 계산한다. 담당 ID를 모르면 자동 재선택 없이 STANDBY다.
- 인지의 target assignment나 mission/assigned를 수정·재발행하지 않는다. `mission/finish`는 명시적 세션 종료로 그대로 처리한다.
- 기존 PointStamped에는 worker_id가 없다. 같은 담당의 좌표라는 판단은 기존 계약의 `target/worker_id`·`target/paired`와 해당 로봇의 인지 발행자를 전제로 한다. 개별 좌표와 ID의 원자적 결합은 별도 인터페이스 변경이 필요하다. 인지의 to_map은 최신 TF 재시도 시 시각이 달라질 수 있어 촬영 시각 자체의 엄격한 보존도 인지 담당 확인 대상이다.
- 고정 목적지: clear 후 같은 goal을 다시 보낸다. 상대 이동·언독이 중단되면 남은 거리를 추측해 반복하지 않고 실패로 보고한다. 복귀 도킹 중단은 기존 접근 지점과 제한된 재시도를 사용한다.
- STUCK이면 추종은 기존 identity를 유지한 STANDBY를 발행한다. 기존 safety/estop 관제 표시도 유지한다. SAFE_ROTATE/STUCK 상세는 아래 safety/state에서 조회한다. 관제 UI의 상세 상태 표시는 관제 담당 후속 작업이다.
- I-050과 웹캠 hint 회전 속도·각도 계산·stop_on_sight는 변경하지 않았다. I-051 fallback overshoot 수정은 포함하지 않는다.

## PM 공유 대상 토픽 변경안

모두 로봇 namespace 아래 상대 이름이다. 기존 mro_interfaces 메시지 정의는 바꾸지 않는다.

| 토픽 | 타입 | 발행 → 구독 |
|---|---|---|
| cmd_vel_navigation | geometry_msgs/TwistStamped | Nav2 collision_monitor → watchdog |
| cmd_vel_search | geometry_msgs/TwistStamped | follow_target → watchdog |
| cmd_vel | geometry_msgs/TwistStamped | **watchdog 한 개만** → Create3 |
| safety/paused/follow_target | std_msgs/Header | follow_target → watchdog |
| safety/paused/shuttle_mission | std_msgs/Header | shuttle_mission → watchdog |
| safety/paused/return_dock | std_msgs/Header | return_dock → watchdog |
| safety/state | std_msgs/String | watchdog → follow_target, 진단·bag |

정지 확인 Header는 액션 종료 확인 중에만 발행한다. stamp가 현재 ESTOP 진입보다 오래되면 거부하며 수신 유효시간도 검사한다. safety/state 값은 STOPPING / SAFE_ROTATE / CLEAR / STUCK. 기존 safety/estop Bool은 CLEAR에서만 false다. clear 이전 생성된 속도는 재사용하지 않는다.

## 실행 전 구성

기존 Nav2와 새 Nav2를 동시에 띄우지 않는다. localization은 기존 절차를 유지한다.

```bash
ros2 launch mro_navigation safety_nav2.launch.py namespace:=/robot3
ros2 launch mro_bringup robot.launch.py namespace:=/robot3
```

새 Nav2 런치는 설치된 TB4 nav2.yaml을 복사해 collision_monitor의 출력 토픽만 바꾼다. 원본 파일은 수정하지 않는다. 다른 nav2.yaml을 쓰면 `params_file:=/절대/경로/nav2.yaml`로 전달한다.

추종 단독 시험은 robot.launch 대신 `follow_rig.launch.py watchdog:=true paired:=real`을 사용한다. 셔틀 리그와 함께 쓰면 watchdog은 한 곳에서만 띄우고 셔틀 리그에 `safety_participants:=shuttle_mission,follow_target`을 지정한다. 복귀 노드도 띄우면 목록에 `return_dock`을 추가한다. 전체 실행의 기본 목록은 세 노드 모두다. 누락된 참가자는 정지 상태를 유지하게 한다.

## 실물 확인 항목

1. `ros2 topic info /robot3/cmd_vel -v`로 최종 발행자가 watchdog **1개**인지 확인한다.
2. 추종 중 실제 worker_id가 비어 있지 않은지 확인한다. 게이트만 수동 주입하고 ID가 없으면 ESTOP 뒤 자동 재선택하지 않는다.
3. bag에 scan, odom, safety/estop, safety/state, safety/paused/*, cmd_vel*, target/worker_id, target/paired, target/map_point, approach/status를 기록한다.
4. 탈출 성공의 조기 정지, 막힌 경우 5초/75° 한도 정지, 같은 작업자의 최신 goal 재전송을 확인한다. 속도는 그대로 두고 bag 근거로 한도 조정한다.
5. 회전 한도는 20 Hz 제어·odom 관측 기준이다. 실제 제동 지연과 각도 오차까지 하드 한도로 보장한 것은 아니다.

## 정지 판정 보완 (2026-09-09)

액션 종료 ACK만으로 회전하거나 ESTOP을 해제하지 않는다. 신선한 odom의 병진 속력 ≤ 0.01 m/s, 각속력 ≤ 0.02 rad/s가 0.3초 이상 유지되어야 한다. 세 값은 common.yaml의 초기 시험값이며 실물 odom 노이즈·제동 지연 검증이 남았다. CLEAR 5-frame 조건은 즉시 0 출력으로 제동을 시작하는 조건이고, 이후 새 정지 시료가 쌓여야 ESTOP이 해제된다. 제동 중 장애물이 다시 나타나거나 ACK가 소실되면 STUCK으로 정지한다. 새 외부 상태값은 추가하지 않았다.

추가 검증: ROS 없는 core/실제 추종 복구 함수 테스트 19개 통과. 이번 보완의 ROS 노드 통합 및 실물 검증은 별도 필요하다.
