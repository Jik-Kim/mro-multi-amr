# PROJECT

## 한 줄 정의

정비사(스케일 인형+RC카)가 관제센터에서 자재를 요청하면 AMR(TurtleBot4 ×2)이 창고에서 가져와 합류 지점에서 만나고,
작업 위치까지 따라다니며, 두 대가 사람 개입 없이 공존한다. — **BRD v3.0 Phase 1 (S1~S9)**

## 기술 방향

- Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, `rclpy`, `rosidl`
- TurtleBot4 Standard ×2 (RPLIDAR, OAK-D Pro, Create3 도킹) — 네임스페이스 `/robot3`, `/robot4`, `ROS_DOMAIN_ID=3` 통일
- Nav2 + AMCL (turtlebot4_navigation), YOLO11n (ultralytics), OpenCV ArUco, SQLite + Flask
- 고정 웹캠 ×2 (PoC 계측·보조 인프라 — 로봇 안전 동작은 웹캠 없이 성립)

## 역할

| 담당 | 패키지 | 영역 |
|---|---|---|
| 김병직 (PM) | `docs/`, `mro_interfaces`, `mro_bringup` | 계약·통합·리뷰·일정 |
| [인지] | `mro_perception` | 인형 데이터·YOLO, OAK-D 검출·재식별 게이트, ArUco |
| [웹캠] | `mro_webcam` | 웹캠1 등록, 웹캠2 힌트, 호모그래피, KPI 로거 |
| [주행] | `mro_navigation` | 셔틀, 추종·재탐지·Standby, safety_watchdog, 복귀 |
| [관제] | `mro_fleet` | DB, fleet_manager, 구역 세마포어, 관제 UI |

세부 결정과 미결 사항은 `docs/SOT.md`, 요구사항 원본은 BRD v3.0 (Kn1_proj/MRO_Multi-AMR_BRD_v3.0.md).
