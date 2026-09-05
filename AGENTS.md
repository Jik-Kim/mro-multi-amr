# AGENTS.md

이 파일은 저장소 전체에 적용되는 공통 작업 규칙이다. 하위 패키지의 `AGENTS.md` 에는 해당 영역 규칙만 둔다.

## 작업 시작

- 작업 전에 `docs/SOT.md` 와 `git status` 를 확인한다. 통신 계약은 `docs/interfaces.md`, 실행 절차는 `docs/setup.md`.
- 요구사항의 근거는 BRD v3.0 (3장 시나리오 S1~S10 / E-1~E-7, 9장 AC, 10장 TS). 문서와 코드가 충돌하면 SOT 를 우선하고 충돌 사실을 알린다.
- 다른 담당의 패키지를 수정하지 않는다. 필요하면 `docs/issues.md` 에 이슈로 남기고 담당에게 알린다.

## 작업 범위

- 요청받은 범위에서 가장 작은 변경으로 목적을 달성한다.
- 토픽·메시지·서비스 계약(`mro_interfaces`, `docs/interfaces.md`)을 바꿀 때는 먼저 PM 에게 알리고 문서와 같이 고친다.
- 확정되지 않은 파라미터(BRD `[팀 확정 필요]`)를 임의로 결정하지 않는다. 값은 `mro_bringup/params/*.yaml` 에 두고 코드에 하드코딩하지 않는다.
- 구현하지 않은 영역은 한글 `TODO([담당])` 로 남긴다.
- 요청하지 않은 `git push`, PR 생성은 하지 않는다.

## 프로젝트 구조

- `mro_interfaces` 는 rosidl 전용 `ament_cmake` 패키지. 알고리즘·노드·장치 코드를 두지 않는다.
- 앱 패키지는 담당 단위 `ament_python` (`mro_perception`, `mro_webcam`, `mro_navigation`, `mro_fleet`). 각 패키지 안은 `nodes/`(rclpy 통신·호출 순서) 와 `core/`(ROS 비의존 알고리즘) 로 나눈다.
- 노드 간 데이터는 직접 호출이 아니라 합의된 ROS2 인터페이스로만 전달한다. PC 사이를 건너는 것은 토픽·서비스뿐이다.
- **안전 계층 원칙:** 정지·회피는 `safety_watchdog` + Nav2 costmap(온보드 LiDAR·범퍼)만으로 성립한다. 재식별·웹캠·Wi-Fi·관제 어느 것이 죽어도 정지 동작은 유지되어야 한다. 웹캠 정보를 안전 판단에 쓰지 않는다.
- 대용량 이미지·영상·데이터셋·rosbag 은 커밋하지 않는다 (Google Drive). 모델 `.pt`, 맵, 호모그래피 json 은 커밋한다.

## 코드 규칙

- navigator 호출(dock/undock/goToPose)은 **콜백 밖 메인 루프**에서 한다 (콜백 안에서 부르면 `Executor is already spinning`).
- 함수 하나는 하나의 책임. 변경 가능한 실행 값은 ROS2 파라미터로. 주석·TODO 는 한국어.
- 카메라 토픽 구독자는 로봇당 검출 노드 1개만 (RPi Discovery 부하). 모니터링은 `target/image` 재발행을 본다.

## 문서 동기화

- 패키지 구조·기술 방향·확정 파라미터가 바뀌면 `docs/SOT.md`. 계약이 바뀌면 `docs/interfaces.md` 와 `mro_interfaces` 를 함께.
- 회의록 원문은 Notion, 확정 결정만 SOT 에. 일정은 2PM_SCHEDULER(Google Sheets), 이슈는 `docs/issues.md`.

## 검증과 완료 보고

- 변경 범위에 맞는 가장 작은 검증부터: `python3 -m py_compile`, 관련 단위 테스트, 패키지 구조 변경 시 `colcon build --symlink-install`.
- 실물 검증은 `docs/setup.md` 절차로. 검증하지 못한 항목은 성공한 것처럼 쓰지 않는다.
- 완료 시 변경 내용, 검증 결과, 남은 TODO 를 간결하게 보고한다.
