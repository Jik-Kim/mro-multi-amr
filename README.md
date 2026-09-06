# mro-multi-amr — MRO Multi-AMR 정비 지원 시스템 (Phase 1 실증)

정비사(스케일 인형+RC카)가 관제센터에서 자재를 요청하면 TurtleBot4가 창고(A/B/C)에서 가져와 대기공간에서 만나고,
작업공간까지 따라다니며, 두 대가 사람 개입 없이 공존한다. 요구사항: **BRD v3.1** (3장 시나리오 S1~S10, E-1~E-7).

## 기준 문서

- [PROJECT.md](PROJECT.md) 개요·역할 · [AGENTS.md](AGENTS.md) 작업 규칙
- [docs/SOT.md](docs/SOT.md) 확정 결정 · [docs/architecture.md](docs/architecture.md) 데이터 흐름 · [docs/interfaces.md](docs/interfaces.md) 토픽·서비스 계약
- [docs/setup.md](docs/setup.md) 환경·실행 절차 · [docs/todo.md](docs/todo.md) 파일 단위 작업 목록 · [docs/issues.md](docs/issues.md) 개발 이슈 리스트

## 구조

```text
ros2_ws/src/
├── mro_interfaces/    # msg/srv 계약 (ament_cmake + rosidl)             [PM]
├── mro_perception/    # OAK-D 검출·거리 보정·재식별 게이트, ArUco          [인지]
├── mro_webcam/        # 웹캠1 등록, 웹캠2 힌트, 호모그래피, KPI 로거       [웹캠]
├── mro_navigation/    # 셔틀, 추종·재탐지·Standby, safety_watchdog, 복귀  [주행]
├── mro_fleet/         # DB, fleet_manager, 구역 세마포어, 관제 UI          [관제]
└── mro_bringup/       # launch(PC 역할별), params(로봇별·아레나), maps    [PM]
tools/                 # Nav2 복구 스크립트, KPI 리포트
docs/
```

각 앱 패키지는 `nodes/`(rclpy) 와 `core/`(ROS 비의존) 로 나뉜다. 미니프로젝트(Kn1_mini)에서 이관한 파일은 파일 상단 `[이관]` 헤더에 변경점과 TODO 가 있다.

## 빌드·실행 (요약 — 상세는 docs/setup.md)

```bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
```

| PC | 실행 |
|---|---|
| 로봇 제어 PC (로봇당 1대) | `localization` → `nav2`(`ROS_SUPER_CLIENT=False`) 별도 터미널 → `ros2 launch mro_bringup robot.launch.py namespace:=/robot3` (두 번째 로봇은 `/robot4`) |

모든 PC·로봇은 `ROS_DOMAIN_ID=3`, 로봇 Discovery Server 경유.
| 웹캠 PC | `ros2 launch mro_bringup webcam_pc.launch.py` |
| 관제 PC | `ros2 launch mro_bringup fleet_pc.launch.py` → http://localhost:5000 |

## 상태

골격 단계. 이관 노드는 Kn1_mini 동작 그대로이며 시나리오용 변경은 각 파일 `TODO([담당])` 참고. 신규 노드 중 `safety_watchdog`·`zone_semaphore`·`aruco_detector`·`kpi_logger`·`fleet_manager`(기본 흐름)·`control_ui` 는 동작 가능한 초안, `webcam_hint` 는 뼈대.
