# 영역별 책임

| 패키지 | 담당 | 핵심 파일 | 한 줄 책임 |
|---|---|---|---|
| `mro_interfaces` | PM 김병직 | `msg/*.msg`, `srv/*.srv` | 노드 간 계약. 바꾸면 `docs/interfaces.md` 동시 갱신 |
| `mro_perception` | 정정인, 민범진 [인지] | `nodes/amr_detection_node.py`(이관), `nodes/aruco_detector_node.py`, `core/reid.py`, `core/depth_distance.py`(이관), `core/target_utils.py`(이관), `models/` | OAK-D 로 작업자 검출·거리·map 좌표, 재식별 게이트, ArUco |
| `mro_webcam` | 한세교, 김학민 [웹캠] | `nodes/webcam_register_node.py`(이관), `nodes/webcam_hint_node.py`(웹캠2 캡처 포함 — `webcam2.py`·`webcam2_subscriber.py` 는 PR #14 이후 잔재, 정리 예정), `nodes/calibrate_cam2_markers.py`, `nodes/kpi_logger_node.py`, `core/homography.py`(이관), `config/WebCam1/`, `config/WebCam2/`, `config/webcam.yaml` | USB 영상·등록·힌트·계측. 안전에 관여하지 않음 |
| `mro_navigation` | 박병후, 이시율 [주행] | `nodes/shuttle_mission_node.py`(이관), `nodes/follow_target_node.py`(이관), `nodes/safety_watchdog_node.py`, `nodes/return_dock_node.py` | 셔틀·추종·정지·복귀 |
| `mro_fleet/nodes/zone_semaphore_node.py` **(예외)** | 박병후, 이시율 [주행] | 이 파일 1개만 | **구역 토큰(P1·DOCK).** 패키지는 `mro_fleet` 이지만 구역 좌표·통행 규칙이 주행 소관이라 소유권만 주행에 둔다. 관제는 이 파일을 수정하지 않는다 |
| `mro_fleet` | 김민준 [관제] | `nodes/fleet_manager_node.py`, `nodes/robot_control_node.py`, `nodes/MonitorWebApp.py`, `core/db.py`, `config/schema.sql`, `templates/index.html` | 미션·상태·UI·DB (**세마포어 제외** — 위 예외 행). `fleet_manager_node` 프로세스가 ROS UI 통신과 웹을 함께 실행 |
| `mro_bringup` | PM 김병직 | `launch/*.launch.py`, `params/*.yaml`, `maps/` | PC 역할별 실행, 로봇별 파라미터, 아레나 좌표 |

`core/` 는 ROS 비의존이므로 단위 테스트 대상이다. `nodes/` 는 실물·토픽 테스트 대상이다.
