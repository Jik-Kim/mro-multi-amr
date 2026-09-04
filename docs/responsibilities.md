# 영역별 책임

| 패키지 | 담당 | 핵심 파일 | 한 줄 책임 |
|---|---|---|---|
| `mro_interfaces` | PM | `msg/*.msg`, `srv/*.srv` | 노드 간 계약. 바꾸면 `docs/interfaces.md` 동시 갱신 |
| `mro_perception` | [인지] | `nodes/amr_detection_node.py`(이관), `nodes/aruco_detector_node.py`, `core/reid.py`, `core/depth_distance.py`(이관), `core/target_utils.py`(이관), `models/` | OAK-D 로 작업자 검출·거리·map 좌표, 재식별 게이트, ArUco |
| `mro_webcam` | [웹캠] | `nodes/webcam_register_node.py`(이관), `nodes/webcam_hint_node.py`, `nodes/kpi_logger_node.py`, `core/homography.py`(이관), `config/*.json` | 등록·힌트·계측. 안전에 관여하지 않음 |
| `mro_navigation` | [주행] | `nodes/shuttle_mission_node.py`(이관), `nodes/follow_target_node.py`(이관), `nodes/safety_watchdog_node.py`, `nodes/return_dock_node.py` | 셔틀·추종·정지·복귀 |
| `mro_fleet` | [관제] | `nodes/fleet_manager_node.py`, `nodes/zone_semaphore_node.py`, `nodes/control_ui_node.py`, `core/db.py`, `config/schema.sql`, `templates/index.html` | 미션·상태·세마포어·UI·DB |
| `mro_bringup` | PM | `launch/*.launch.py`, `params/*.yaml`, `maps/` | PC 역할별 실행, 로봇별 파라미터, 아레나 좌표 |

`core/` 는 ROS 비의존이므로 단위 테스트 대상이다. `nodes/` 는 실물·토픽 테스트 대상이다.
