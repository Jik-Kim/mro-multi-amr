# Feature Todo — 파일·메서드 기준

각 줄 끝의 `마감 M/D` 는 스케줄러 Time Line 기준이다. 진행률은 아래 표가 체크박스를 세어 보여준다.
**이슈를 닫거나 상태를 바꿀 때는 여기 체크박스도 같이 본다** (`AGENTS.md` 「이슈 등록」 절).

<!-- STATS:BEGIN -->

**전체 124/194 완료** (█████████████░░░░░░░)  ·  기준 09/10

| 파트 | 완료 | 진행 | 지난 마감 |
|---|---|---|---|
| mro_interfaces [PM 김병직] | 7/9 | `████████░░` | — |
| mro_perception [인지 — 정정인, 민범진] | 22/28 | `████████░░` | **5** |
| mro_webcam [웹캠 — 한세교, 김학민] | 24/34 | `███████░░░` | **6** |
| mro_navigation [주행 — 박병후, 이시율] | 46/88 | `█████░░░░░` | **25** |
| mro_fleet [관제 — 김민준] | 15/22 | `███████░░░` | **4** |
| mro_bringup [PM 김병직] | 10/13 | `████████░░` | **1** |

**마감이 지난 항목 41건**

- `9/9` mro_perception — mro_perception flake8 정리 — main 55건 → PR #40 으로 80건 → 9/9 실측 156건(99자 기…
- `9/8` mro_perception — TS-U04 측면·후면 — 인형을 OAK-D 앞 1 m 에서 0°/90°/180°/270° 로 돌려 검출·outfit_class…
- `9/8` mro_perception — TS-U05 줄자 대조 — target/distance 의 정확도를 0.8~2 m 에서 사전 검증 (FR-34: 채점 수단의 오…
- `9/9` mro_perception — 시험 절차 — 대기공간 로봇이 서향이므로 S5 페어링 때 RC카를 로봇 정면(서쪽)으로 돌아 들어오게 조종 (FR-14 정면 1…
- `9/8` mro_perception — reid_threshold 확정 — 같은/다른 인형 유사도 분포를 그려 결정 (SOT Q-13). 추출 영역이 인형 전체라 다른…
- `9/9` mro_webcam — /webcam2/image_raw/compressed 를 오버레이로 통일 (I-042) — 지금은 오버레이와 원본이 섞여 나가고…
- `9/9` mro_webcam — test_subscription 을 tools/ 로 (PR #48 후속) — setup.py 콘솔 진입점으로 등록돼 ros2 r…
- `9/8` mro_webcam — 웹캠2 캡처 fps 실측 — webcam.yaml 의 webcam_hint.fps: 30.0 은 장치 사양 가정이고 실측 기록이…
- `9/8` mro_webcam — 웹캠2 호모그래피 재검증 (I-002 · TS-U02) — 재캘리 완료(PR #32: LiDAR 중심 AMCL 13점, RMSE…
- `9/8` mro_webcam — webcam_hint_node.py: YOLO 타이머 재진입 차단 (I-027) — 개별 MutuallyExclusiveCall…
- `9/8` mro_webcam — webcam_hint_node.py: worker.pt 교체 후 TS-U02 — 9/8 통신 구간 부분 통과: 압축 영상 9.9…
- `9/9` mro_navigation — TS-04·TS-14 재실행 (I-036) — clear_dist 를 0.45 → 0.38 로 낮췄으므로 기존 통과 기록은 무효…
- `9/9` mro_navigation — 베이 접근 잔차 규명 (I-044, 상) — 여유 0.095 m 가 관측 잔차 0.275 m 보다 작다. d3e0568 로그로 …
- `9/9` mro_navigation — bay_align_min_range_m 을 0.80 으로 (PR #52 후속) — 지금 0.45(노드 기본값 0.35)라 자기가…
- `9/9` mro_navigation — 토큰 서비스 미기동 시 진행하는 것 (I-041, 상) — shuttle_mission·return_dock 둘 다 wait_f…
- `9/9` mro_navigation — I-033 나머지 절반 — zone_semaphore_node 가 반납 요청으로 대기열에서 빼는가 — 클라이언트는 PR #46 …
- `9/9` mro_navigation — 접근 수렴 실주행 확인 (PR #49) — 3 m 넘는 거리에서 시작해 회차마다 gap 이 실제로 반씩 주는지. 멀리서 잰 좌표…
- `9/9` mro_navigation — P1 통과 — 측위가 유일한 경로다 (I-009 PM 판단 9/8, 정정) — 통과 확실에 필요한 통로는 0.99 m 인데 교행…
- `9/9` mro_navigation — (곁가지, 공짜면 한다) 받침대가 이미 통로 바깥인지 실측 확인 → 맞으면 inflation_radius 0.25 → 0.20 …
- `9/8` mro_navigation — Nav2 복구 행동에서 후진(BackUp)을 뺀다 (계약 2절 후진 금지) — 우리 코드에는 linear.x < 0 이 없지만 …
- `9/9` mro_navigation — I-035 실주행 확인 — 새 기동 순서(T1 → 리그 → T7 → T3)로 global_costmap 이 곧바로 active …
- `9/9` mro_navigation — P1 통과 10회 연속 성공 (4단계 게이트, I-009) — 측위 오차 0.32 m > 통과 여유 0.175 m 인 동안 2대…
- `9/8` mro_navigation — arena.yaml loc_check_marker.xy 실측 — 지금 [0.0, 0.0] 은 맵 원점이라 관제가 그대로 읽으면 …
- `9/8` mro_navigation — inflation_radius 0.05 → 0.25 적용 (I-009) — 주행 합의값. 승인 후 PC-R3·PC-R8 의 na…
- `9/7` mro_navigation — 측위 검증 마커 설치·실측 — ArUco ID 4 거치대(웹캠1 박스 북쪽 ≈ 0.4 m 앞 거치대, 북향, 10 cm, 높이 …
- `9/8` mro_navigation — 베이 A·B 정차점 서쪽으로 ≥ 0.15 m 이동 또는 DOCK 폴리곤 확장 (I-025) — 접근 지점과 0.31/0.38 m…
- `9/8` mro_navigation — 도착 오차 수치 신뢰성 확인 (I-037) — 노드 TF 0.84 m vs 같은 순간 amcl_pose 0.18 m. 3회차에 …
- `9/8` mro_navigation — P1 통과 실주행 확인 (I-009) — 여유 0.175 m < 실측 측위 오차 0.32 m 라 통과가 운에 좌우된다. infl…
- `9/8` mro_navigation — AMCL update_min_d 0.05 실주행 확인 (I-029) — 9/7 저녁 적용(0.25→0.05). 도착 판정이 안정…
- `9/8` mro_navigation — PC-R8 에 localization.yaml update_min_d: 0.05 복제 (I-029) — turtlebot4_ws…
- `9/8` mro_navigation — 추종거리 분포 실측 (I-029) — xy_goal_tolerance 0.25 고정이라 정지 시 오차 25 cm. TS-U07 …
- `9/8` mro_navigation — shuttle_mission_node.py: 성공 직후 READY 를 내지 말 것 (I-034) — kpi_report 의 DO…
- `9/8` mro_navigation — 셔틀 실주행 1단계 완주 — 판정 7항목 (I-024 '확인하지 못한 것' ①)
- `9/8` mro_navigation — safety_watchdog_node.py: Nav2 goal cancel (액션 클라이언트), TS-14 결함 주입 검증
- `9/8` mro_navigation — safety_watchdog_node.py: HIT_FRAMES·CLEAR_FRAMES 를 초 단위 파라미터로 (계약 시간 상수…
- `9/8` mro_navigation — inflation_radius 0.25 실주행 확인 (I-009) — P1 통과 · P3 받침대 안 긁는지. 두 PC 값 일치 …
- `9/9` mro_fleet — FR-13 대시보드 실제 조작 확인 (PR #47·#51) — 브라우저에서 ① GATE_FAIL 발행 시 버튼이 뜨는가 ② 누르…
- `9/9` mro_fleet — FR-35 는 loc_check_marker.xy 실측 뒤에 켠다 — [0.0, 0.0] 인 채로 읽으면 측위가 정상인데도 lo…
- `9/8` mro_fleet — fleet_manager_node.py: WAITING 수신 시 amcl_pose 가 waiting_zone 근처인지 확인, 벗…
- `9/8` mro_fleet — fleet_manager_node.py: aruco/id·aruco/distance 구독 → ID 4 수신 시 |amcl_pos…
- `9/8` mro_bringup — v_max 0.3 m/s 와 Nav2 max_vel_x 0.26 충돌 정리 — 계약이 0.3 이라고 적었으니 Nav2 쪽을 맞춘…

**오늘 마감 18건**

- mro_interfaces — 계약 v1.22 — 워치독 출력 권한 분리 (I-049, 주행 PR 뒤) — 신규 토픽 safety/state(STOPPING/…
- mro_interfaces — docs/setup.md:92 의 nav2 런치 줄 (I-049 후속) — turtlebot4_navigation/nav2.la…
- mro_perception — FR-13 실기 확인 (인지↔관제 합동) — 인형이 보이는 상태로 GATE_FAIL 30 s → 관제 버튼 → 수락까지 한 번에…
- mro_webcam — webcam_hint 의 target_classes 를 yaml 로 (PR #62 후속) — webcam_hint 의 코드 전용…
- mro_webcam — webcam.yaml 의 show_window 줄 삭제 (PR #65 후속) — launch 가 yaml 뒤에 딕셔너리로 넘겨 …
- mro_webcam — mro_webcam flake8 — 9/9 실측 365건 (등록 시 345건) — 이번 PR 탓은 아니지만 이 정도면 아무도 이…
- mro_navigation — T_lim 120 s 실측 확정 (BRD Q-14) — 오늘 오전 중 — 새 축소 아레나 동선에서 요청 접수 → 대기공간(Z-0…
- mro_navigation — 웹캠2 힌트 발행이 48 s 끊긴 사유 (I-050 곁가지) — follow_0909_1609 에서 10 Hz 로 오다 48.0…
- mro_navigation — bag 기록 목록에 target/worker_id 를 넣는다 — hint_bearing() 이 이 값으로 남의 작업자 힌트를 거…
- mro_navigation — estop 탈출 회전을 워치독이 직접 만든다 (I-049, 상) — PM 승인 완료 9/9 — 1차 SAFE_ROTATE(병진 …
- mro_navigation — 회전 과도 9~34° 를 없앤다 (I-051) — odom 이 늦게 와서 이미 지나친 뒤에 멈춘다. 초과분은 상수가 아니라 그때…
- mro_navigation — R2 xy_goal_tolerance 0.25 → 0.10 은 이미 실패한 값이다 (PR #74 계획 재검토) — I-029 에…
- mro_navigation — 베이 마커 세 장의 흰 여백을 눈으로 확인 (I-052, 상) — TS-07 전 — 여백 없이 벽에 바로 붙으면 거리가 18~2…
- mro_navigation — safety_watchdog 에 네임스페이스 가드 (I-048) — 나머지 셋(:178·:114·:82)에는 있고 안전 노드에만…
- mro_navigation — 주행 README 의 불변식에 I-033 을 밝힌다 (PR #64 후속) — 「요청한 토큰은 모든 종료 경로에서 반납한다」가 지…
- mro_navigation — docs/diagrams/README.md 목록에 navigation_review_flow.drawio 추가 (PR #64 후속…
- mro_fleet — 대시보드 JS 의 로봇 이름 하드코딩 (PR #51 후속) — uiState.robots 가 {robot3, robot8} 고정…
- mro_bringup — SDD 부록 A rqt_graph 캡처 첨부

> 이 표는 `python3 tools/todo_stats.py` 가 체크박스를 세어 다시 쓴다. 손으로 고치지 않는다.

<!-- STATS:END -->

## mro_interfaces [PM 김병직]
- [ ] **계약 v1.22 — 워치독 출력 권한 분리 (I-049, 주행 PR 뒤)** — 신규 토픽 `safety/state`(`STOPPING/SAFE_ROTATE/CLEAR/STUCK`)·`safety/paused/<주행 노드>` 를 4절에 넣고, **`cmd_vel` 행의 발행자를 「safety_watchdog 단독」으로** 고친다 (v1.13 이 「safety_watchdog, Nav2, follow_target」로 적어 두었다). `approach/status` 8개 값은 안 건드린다. **PR 을 보고 쓴다** — 미리 쓰면 구현과 어긋난다 · 마감 9/10
- [ ] **`docs/setup.md:92` 의 nav2 런치 줄 (I-049 후속)** — `turtlebot4_navigation/nav2.launch.py` → `safety_nav2.launch.py`. 주행 PR 에 포함해 달라고 요청했으나 빠지면 머지 때 PM 이 고친다 · 마감 9/10
- [x] **계약 v1.17** — `ARRIVED` 도 발행되지 않는다(`APPROACH_2` 와 하나의 같은 사실). **값 목록은 8개 그대로**임을 못박아 관측 사실이 계약 축소로 읽히지 않게 했다 (PR #67 후속) · 마감 9/9
- [x] **계약 v1.13** — `aruco/pose` 비준(회전은 정렬에 쓰지 않는다) · ArUco 발행 한계 2.0 / 수용 조건 1.5 분리 · `cmd_vel` 에 `follow_target` 추가와 estop 전진만 차단 (I-036) · 마감 9/8
- [x] **SDD v0.13** — FR-13 확정(2.4·E-8·10장·부록 B), 셔틀 노드 개편과 I-033·I-034, K-06 완결·I-030 종결 반영. 노션 🫠 SDD 하위 페이지 생성 완료 · 마감 9/7
- [x] **계약 v1.12 — 명시 페어링 폴백(FR-13) 확정** (관제 8개 질문 답변) — `target/pair_state` 토픽, `srv/ManualPair`, 개입 종류 `MANUAL_PAIR` 추가. 규칙은 계약 10절 · 마감 9/7
- [x] msg/srv 8종 정의 · 마감 9/6
- [x] **계약 v1.3 확정** — `docs/interfaces.md`. 구역 ID `P1`·`DOCK`, 확정 값 8건, 파라미터 단일 출처, 주기·시간 상수 규칙, 서술자 추출 영역 · 마감 9/6
- [x] `WorkerDescriptor.color_hist` — H16×S8=128, `reid.hsv_hist()` 공유 호출로 확정 · 마감 9/6

## mro_perception [인지 — 정정인, 민범진]
- [x] **robot8 fx 재확인 — 마커 흰 여백부터 (I-031)** — robot8 이 ID 2·3 에서 최대 −16 % 짧게 나오고 검출도 깜빡였다. 카메라를 의심했으나 **원인은 마커였다** — 벽에 바로 붙여 흰 여백이 없으니 검은 테두리가 벽 경계선과 이어져 ⓐ 윤곽이 사각형으로 안 잡히고 ⓑ 서브픽셀 보정이 꼭짓점을 바깥으로 끌어 사각형이 커졌다(커지면 거리가 가까워진다). **재투영 오차는 0.01 px 로 깨끗해 지표로 안 걸렸다.** 흰 배경 3~5 cm 를 붙이자 검출 100 %, `implied marker` 11.6 → 9.8~10.6 cm, 줄자 0.980/1.265/1.530 m 에서 +0.5/+1.2/+3.2 cm — 운용 구간(0.9~1.5 m) 판정 기준 10 cm 의 1/3 안. **robot8 fx 정상 · 체커보드 재촬영 불필요.** 계약 6절이 요구하는 여백(한 칸 이상 ≈ 2 cm)이 이것이다 · 마감 9/10
- [x] **`camera_fx_scale` 1.7778 → 1.8336 PM 승인 · 민범진 통보** — **근거는 줄자다** (체커보드는 재현성 8 % 로 소수점을 못 정한다 — I-031). robot3 5점(0.575~1.455 m) 1.7 cm 안 · robot8 3점 3.2 cm 안. **`amr_detection` 의 `target/map_point` 좌우 성분이 +3.1 % 이동하므로** 짝(민범진)에게 먼저 알리고 PM 승인을 받는다. `target/distance`(depth)는 영향 없다 · 마감 9/10  **→ 승인 9/10 (PR #76 머지). PM 검산: PR 본문의 「좌우 성분을 3.1 % 키운다」는 부호·크기가 다르다 — `x = (u−cx)·z/fx` 라 좌우는 3.04 % **줄고**, 화면 안 최대 좌우가 `z·tan(18.7°)` 로 묶여 1.1 m 에서 최악 1.13 cm 다. 수평시야 38.5 → 37.4° 로 `search_step_deg` 20° 는 여전히 유효**
- [x] **`tools/checker_calib.py` 에 재현성 검증 추가** — RMS 만 보면 속는다. RMS 는 **오차 큰 사진을 자동으로 뺀 뒤**의 값이라 서로 말이 맞는 사진만 골라내면 좋아 보인다 (robot3 은 그 과정에서 RMS 가 0.73 → 0.31 로 좋아지는 동안 fx 가 1035 → 1096 으로 5.8 % 움직였다). 사진을 홀수/짝수로 갈라 풀고 20장씩 무작위로 12번 더 풀어 폭을 잰다 (합격선 2 % = 1.5 m 에서 3 cm). 실측 robot3 8.2 % · robot8 8.7 % — **둘 다 초과라 체커보드 값은 후보로만 두고 줄자로 확정한다.** `show` 의 결론도 그 판정을 따른다 · 마감 9/11
- ~~**robot8 TS-U05 독립 점 1개 (I-031)** — `depth_k` 0.028 은 줄자 3점을 **전부 적합에 썼다.** 유일한 검증점이 「약 1.30 m」(줄자 불확실) +3.42 cm 다. pnp 로 검증하면 자기 검증이 된다(robot8 pnp 가 −2.4~−4.3 % 치우침). **그때까지 FR-35 는 이 값을 신뢰하지 않는다** · 마감 9/9~~  **→ 취소 9/10 (`feature/aruco-pnp-only`) — `depth_k` 가 없어졌다.** 체커보드로 fx 를 실측해 depth 기준자 경로를 통째로 걷어냈다 (`marker_depth`·`radial_from_depth`·`calibrated_depth` 와 파라미터 7개 삭제). 완료가 아니라 **할 일 자체가 없어진 것**이라 체크박스를 뗀다 — 셈에서 빠진다
- [x] **`tools/aruco_offline_check.py` 상수를 robot8 값으로 (I-031)** — **값은 9/9 에 받았다 (PR #74)**: `fx = fy 566.0106 · cx 352.383 · cy 367.613` (704×704). robot3 566.654 와 **0.11 % 차**. **아직 못 닫는 이유 — 왜곡계수 `D` 8개가 안 왔다.** 도구는 `K_INFO` 와 `D` 를 같이 쓰므로 K 만 바꾸면 robot4 왜곡으로 robot8 을 렌더한다. `ros2 topic echo /robot8/oakd/rgb/camera_info --once` 의 `d` 배열만 더 받으면 끝난다. **곁가지로 확인된 것**: `cx` 는 704/2 에서 0.4 px 라 가로 16/9 crop 전제가 성립하고 `camera_fx_scale` 1.7778 을 유지해도 된다. 반면 **`cy` 367.613 은 15.6 px 아래로 치우쳤다** — 정사각 crop 이 세로로는 가운데가 아니다. 세로 오차는 기운각·법선 추정에 들어가므로 I-039(pnp 실효계수 요동)와 같이 볼 값이다 · 마감 9/11  **→ 완료 9/10 — 못 닫던 이유였던 왜곡계수 `D` 8개를 받았다** (`13.3801 −150.4605 0.0022977 0.0011334 541.2130 13.1323 −148.4299 533.8889`, rational_polynomial). K 와 D 를 한 짝으로 `CAMERAS` 에 넣고 `--robot` 으로 고르게 했다 — robot3 도 같이 넣어서 앞으로 남의 광학으로 합격을 매기는 일이 없다. 재실행: robot8 최대 1.1 cm · robot3 0.8 cm (계약 6절 조건 안). stereo 는 `frame_id` 가 rgb optical, K 동일, `D` 전부 0 — depth 를 RGB 에 정렬해 내보내므로 **정상이다**
- [x] **`show_window` 기본값을 `false` 로 (I-043)** — `robot.launch.py` 가 `perception.yaml` 을 모든 로봇에 넘긴다. SSH 로만 붙은 PC 가 하나라도 있으면 회차가 통째로 날아간다. 웹캠 쪽은 이미 `false` 다 · 마감 9/9  **→ 완료 9/10 (PR #75) — yaml 과 노드 모듈 기본값 둘 다. `ros2 run` 은 yaml 을 안 받고 그것이 곧 SSH 시험 경로라 한쪽만 고치면 안 됐다. `DISPLAY` 없는 환경에서 기본값 정상 기동·`true` 는 Aborted 재현 확인. PM 확인: 런치가 딕셔너리로 덮지 않아 이 값이 실제로 닿는다 — 웹캠 `webcam.yaml` 과 구조가 다르다**
- ~~**robot3 실기 재확인 (PR #50 후속)** — `depth_wait_s` 를 0.08 → 0.25 로 3배 올린 것이 robot3 발행 주기에 영향 없는지. robot3 은 마커가 시야에 없어 미확인이다 · 마감 9/9~~  **→ 취소 9/10 (`feature/aruco-pnp-only`) — `depth_wait_s` 가 없어졌다.** 체커보드로 fx 를 실측해 depth 기준자 경로를 통째로 걷어냈다 (`marker_depth`·`radial_from_depth`·`calibrated_depth` 와 파라미터 7개 삭제). 완료가 아니라 **할 일 자체가 없어진 것**이라 체크박스를 뗀다 — 셈에서 빠진다
- [x] **TS-U05 독립 점 1개 (I-031 마무리)** — 완료 9/8 (PR #43). 배율 모델이 1.55 m 에서 3.0 cm 틀려 **역깊이(시차 편향) 보정**으로 교체. 독립점 1.19 m 에서 −0.5 cm. PM 재적합 확인 — 배율은 어떻게 펴도 최선이 1.9~2.1 cm 라 모델 꼴 문제였다. **robot8 독립점은 카메라 올라온 뒤** (9/8 기체 교체로 2호기 보정은 원점)
- [x] `core/aruco.py` ROS 비의존 — 완료 9/8 (PR #43). `depth_distance` 의 ROS import 를 뷰어 노드 전용 try/except 로. PM 확인: `AMENT_PREFIX_PATH` 빈 셸에서 `rclpy` 미적재 · 단위 시험 18/18
- [x] **robot3 depth 가 RGB 보다 +83 ms 늦다 (I-038)** — ArUco 프레임 73 % 가 depth 대신 pnp 대체로 떨어진다. 값은 같은 자라 지금 오차는 없지만, 기동 직후 '못 봤다' 구간과 오래된 depth/pnp 비가 남는다. 큐 방식 짝짓기 · 마감 9/9  **→ 완료 9/8 (PR #50) — 큐 방식 짝짓기 · `depth_wait_s` 0.25. robot8 은 같은 원인으로 **발행 0 %** 였다**
- ~~**`perception.yaml` 에 k 검증 범위를 적는다 (PR #43 후속)** — k 를 편 범위는 보정값 0.843~1.565 m 인데 발행 한계는 2.0 m 다 (보정량 −9.2 cm, 미검증). 계약 6절 수용 1.5 m 는 검증 범위 안이라는 것까지 한 줄 · 마감 9/9~~  **→ 취소 9/10 (`feature/aruco-pnp-only`) — `depth_k` 가 없어졌다.** 체커보드로 fx 를 실측해 depth 기준자 경로를 통째로 걷어냈다 (`marker_depth`·`radial_from_depth`·`calibrated_depth` 와 파라미터 7개 삭제). 완료가 아니라 **할 일 자체가 없어진 것**이라 체크박스를 뗀다 — 셈에서 빠진다
- ~~**`calibrated_depth` 가 `depth_min` 게이트에 의존한다 (PR #43 후속)** — `z_raw` 로 나누므로 게이트가 없으면 0 원값에서 죽는다. docstring 에 명시 · 마감 9/9~~  **→ 취소 9/10 (`feature/aruco-pnp-only`) — `calibrated_depth` 함수가 없어졌다.** 체커보드로 fx 를 실측해 depth 기준자 경로를 통째로 걷어냈다 (`marker_depth`·`radial_from_depth`·`calibrated_depth` 와 파라미터 7개 삭제). 완료가 아니라 **할 일 자체가 없어진 것**이라 체크박스를 뗀다 — 셈에서 빠진다
- [x] **robot8 `depth_k` 실측 (I-031·I-040)** — 9/8 기체 교체로 **보정값이 없다**(`robot8.yaml` `depth_k: 0.0`). 구 robot4 의 0.0076 은 그 카메라 값이라 승계 불가 (robot3 은 0.024 — 3 배 차). 0.8 m 이상 줄자 2점 이상으로 `k = 평균(1/줄자 − 1/원값)`. **I-040 으로 robot3 보정도 그동안 안 먹고 있었으므로 TS-U05 재측정과 함께** · 마감 9/10  **→ 완료 9/8 (PR #50) — `0.028` (줄자 3점, 1.5 m 안 최대 1.01 cm). **독립 검증점은 아직** — I-031 참조**
- [ ] `mro_perception` flake8 정리 — main 55건 → PR #40 으로 80건 → **9/9 실측 156건(99자 기준).** 늘고 있다. 대부분 D400·D403 docstring 형식과 99자 초과라 한 번에 정리된다. **아무도 이 시험을 안 돌리고 있다는 뜻** · 마감 9/9
- [x] **`amr_detection_node.py`: 명시 페어링 폴백 서버 (FR-13, 계약 10절)** — ① `target/pair_state` 발행 (`PAIRED`/`GATE_FAIL`/`NO_TARGET`) ② 게이트 미달 30 s 계측 — **대상이 보이는데 못 넘는 경우만 센다**(`NO_TARGET` 은 세지 않음) ③ `/robotN/manual_pair` 서비스 서버 — 거부 사유 6종(10.5 표) ④ `mission/finish` 구독해 명시 페어링 해제 ⑤ 확정 시 `target/paired`·`target/worker_id` 발행은 **여기서만** · 마감 9/9  **→ 완료 9/9 (PR #53) — 단위 18건. 거부 사유는 5종**(10.5 표 5행 + 10.8 은 `WRONG_ROBOT` 에 흡수)
- [x] **`amr_detection`: `TARGET_STALE_S` 를 `target_stale_s` 파라미터로** (I-045, PR #53 리뷰) — 지금은 모듈 상수 1.0 s 라 현장에서 못 늘린다. 프레임 지연이 1 s 를 넘으면 `pair_state` 가 `NO_TARGET` 으로 떨어져 FR-13 폴백 버튼이 아예 안 뜬다. `perception.yaml`·README §3 에 올리고 주석의 파라미터 개수도 17 로 · 마감 9/10  **→ 완료 9/9 (PR #54) — 파라미터 17개를 실행 중 변경 12 / 기동 전용 5 로 갈랐다. 범위 밖은 거절**
- [ ] **FR-13 실기 확인 (인지↔관제 합동)** — 인형이 보이는 상태로 `GATE_FAIL` 30 s → 관제 버튼 → 수락까지 한 번에. 이제 서버가 있으므로 `인지 서비스 응답 없음` 이 아니라 실제 수락/거부 사유가 떠야 한다. `mro_perception/README` §5 에 절차 추가 · 마감 9/10
- [x] 인형 2종 제작 (상하의 전체, 9/6 결정) · 마감 9/6
- [x] 데이터 수집·라벨링 (9/6 완료) · 마감 9/6
- [x] `models/worker.pt` 학습·검증, 색 편향 점검 (I-003) — PR #19, yolo26s@640 · val mAP50-95 0.816 · 밝기 차 0.6 % (PM 체크포인트 대조 일치) · 마감 9/7
- [x] **배포 PC 에서 `tools/bench_yolo.py` 로 `worker.pt` 10 Hz 확인** (PR #22 최저 사양 RTX 3060 track max 42.7 ms · PR #26 PC-W1 max 40.5 ms — nano 불필요, 실부하는 통합 9/9) — max 가 100 ms 넘으면 nano(`yolo26n`@640) 로 교체. PM 노트북 CPU 는 track 9 ms (PR #19) · 마감 9/8
- [ ] **TS-U04 측면·후면** — 인형을 OAK-D 앞 1 m 에서 0°/90°/180°/270° 로 돌려 검출·`outfit_class` 유지 확인 (AC-10 근거, SDD 부록 B). 판정 기준은 SDD v0.10 · 마감 9/8
- [x] `amr_detection_node.py`: `MODEL_PATH` → `worker.pt`, `DETECT_CLASS` → worker_a/b (이름으로 거른다, 인덱스 아님), **`ONLY_OUR_CAR` 흰차 필터 해제**, `/webcam1/worker_descriptor` 구독, `core/reid.gate` 적용 → `target/paired`, `target/worker_id` 발행 · 마감 9/9  **→ 완료 9/8 (PR #45)**
- [x] `amr_detection_node.py`: RGB·depth 구독 QoS → `qos_profile_sensor_data` (계약 8절 — 발행자가 BEST_EFFORT 면 영상이 한 장도 안 온다) · `camera_info` K 를 `core/camera.corrected_K` 로 보정 (`target/map_point` 좌우 1.76 배, I-031) — PR #30 후속, 교체 PR 에 같이 · 마감 9/9  **→ 완료 9/8 (PR #45) — 렉의 주원인이었다**
- [x] `amr_detection_node.py`: **`target/distance` 발행 (FR-29)** — 추종거리 판정의 기준값. kpi_logger 는 이미 구독 중이라 이것만 붙으면 AC-05 가 측정된다 · 마감 9/8  **→ 완료 9/8 (PR #45)**
- [ ] **TS-U05 줄자 대조** — `target/distance` 의 정확도를 0.8~2 m 에서 사전 검증 (FR-34: 채점 수단의 오차 < 기준의 1/3). 이게 없으면 AC-05 를 그 값으로 채점할 근거가 없다. **같은 시험에서 `aruco/distance` 도 줄자와 대조해 `camera_fx_scale` 1.7778 을 확정한다 (I-031)** · 마감 9/8
- [ ] 시험 절차 — 대기공간 로봇이 서향이므로 S5 페어링 때 **RC카를 로봇 정면(서쪽)으로 돌아 들어오게** 조종 (FR-14 정면 1 m). 작업자는 P2(남쪽)로 온다 · 마감 9/9
- [x] `amr_detection_node.py`: 모듈 상수 → ROS2 파라미터 (`config/perception.yaml`) · 마감 9/8  **→ 완료 9/8 (PR #45) — 15개 전부**
- [x] `aruco_detector_node.py`: 검출·거리 초안 · 마감 9/7
- [x] `aruco_detector_node.py`: solvePnP 자세, 마커 ID 필터 — PR #30: solvePnP(IPPE_SQUARE) · `marker_ids` 허용 목록 · 거리 게이트 1.5 m · `camera_info` fx 16/9 보정 (I-031, PM 합성 재현: 보정 후 오차 ≤ 1.9 cm, 보정 없이 1.35 → 0.77 m) · 마감 9/8
- [x] `core/reid.py` 단위 테스트 (같은 인형 ≥ threshold, 다른 인형 < threshold) · 마감 9/8  **→ 완료 9/8 (PR #45) — 11건**
- [ ] `reid_threshold` 확정 — 같은/다른 인형 유사도 분포를 그려 결정 (SOT Q-13). 추출 영역이 인형 전체라 다른 인형끼리도 0.5 안팎에서 시작한다 · 마감 9/8

## mro_webcam [웹캠 — 한세교, 김학민]
- [ ] **`/webcam2/image_raw/compressed` 를 오버레이로 통일 (I-042)** — 지금은 오버레이와 원본이 섞여 나가고 받는 쪽이 구분할 수 없다. 원본 대체를 빼고 첫 검출 주기 전에는 발행하지 않는다 (PM 결정 9/8) · 마감 9/9
- [x] **`hint_valid_s` 죽은 설정 정리 (I-047)** — `webcam_hint_node` 가 선언·검사만 하고 안 쓴다. `webcam.yaml` 주석은 「이 시간 지난 힌트는 쓰지 않는다」고 적혀 있어 만지면 「바꿨는데 안 변한다」가 된다. **지우는 쪽을 권한다** — 유효기간은 받는 쪽이 정하고 `common.yaml` 이 단일 출처가 된다 · 마감 9/10  **→ 완료 9/9 (PM 작업) — 웹캠팀이 판단을 위임해 지웠다. `common.yaml` 하나가 단일 출처**
- [ ] **`test_subscription` 을 `tools/` 로 (PR #48 후속)** — `setup.py` 콘솔 진입점으로 등록돼 `ros2 run mro_webcam test_subscription` 으로 뜬다. 이름이 단위 시험처럼 읽히는데 진단 도구다. `tools/diag_lag.py` 가 선례 · 마감 9/9
- [ ] **`webcam_hint` 의 `target_classes` 를 yaml 로 (PR #62 후속)** — `webcam_hint` 의 코드 전용 값 중 경로가 아닌 것은 이것 하나 남았다 (`homography_json`·`model_path` 는 launch 가 주는 경로라 대상이 아니다). **웹캠1(`webcam_register`)쪽 yaml 에는 이미 있다** (`webcam.yaml:10`) — 같은 개념인데 한쪽만 코드에 있다. 기본값 `["worker_a","worker_b"]` 그대로 한 줄 · 마감 9/10
- [ ] **`webcam.yaml` 의 `show_window` 줄 삭제 (PR #65 후속)** — launch 가 yaml 뒤에 딕셔너리로 넘겨 **항상 덮으므로 그 줄은 이제 아무 일도 안 한다.** 값이 같아 지금은 무해하지만 시연하려고 `true` 로 고치면 안 바뀐다. `show_window` 는 「어떻게 접속해 띄우느냐」에 달린 값이라 launch 인자가 제 자리다 — yaml 이 갖지 않기로 한다 (오늘 셋째 사례: I-045·I-047 과 같은 유형) · 마감 9/10
- [ ] **`mro_webcam` flake8 — 9/9 실측 365건** (등록 시 345건) — 이번 PR 탓은 아니지만 이 정도면 아무도 이 시험을 안 돌리고 있다는 뜻이다. `mro_perception` 도 비슷하다 · 마감 9/10
- [ ] **웹캠2 캡처 fps 실측** — `webcam.yaml` 의 `webcam_hint.fps: 30.0` 은 장치 사양 가정이고 실측 기록이 없다. 웹캠1(Jieli)은 9/6 에 재서 25 로 확정했다. 두 캠이 같은 모델이면 30 이 아니라 25 다. `v4l2-ctl --list-formats-ext` 로 확인하고 다른 값이면 yaml 을 고친다 — 설계도는 이 파일을 읽어 그리므로 다시 생성만 하면 따라온다. 계약 4절이 '장치 상한 그대로' 라 동작에는 영향이 없다 (캡처 ≥ 발행 10 Hz) · 마감 9/8
- [x] **새 아레나 호모그래피 (I-002)** — cam2 완료 (PR #9, AMCL 기준점 13개, RMSE 2.8 cm). **cam1 은 하지 않는다 (9/6)** — 웹캠1 은 등록만, map 좌표 없음 · 마감 9/6
- [ ] **웹캠2 호모그래피 재검증 (I-002 · TS-U02)** — 재캘리 완료(PR #32: LiDAR 중심 AMCL 13점, RMSE 2.8 cm, PM LOO 평균 4.0 cm·max 8.1 cm). 검증은 **held-out 3점 — 로봇을 새 지점에 세우고 `amcl_pose` vs 픽셀 변환 대조 (≤ 0.3 m)**. **Nav2 도착 위치로는 판정하지 않는다** — `xy_goal_tolerance` 0.25(I-029) 안의 15 cm 는 호모그래피 오차의 근거가 못 된다 (PM 9/7) · 마감 9/8
- [x] `webcam2.py` USB 캡처 + `calibrate_cam2_markers.py` 마커 캘리브레이션 도구 (PR #2) · 마감 9/6
- [x] `webcam_register_node.py`: YOLO track ID, 복장 클래스, `hsv_hist` 서술자 → `WorkerDescriptor` 발행 (PR #6) · 마감 9/7
- [x] `webcam_register_node.py`: `worker.pt` 교체 후 **TS-U01** (등록 ≤ 10 s, 2명 별개 ID) — PR #26 실기동: 등록 0.90 s · W1/W2 별개 · 재등록 15/15 (체류 시간은 Q-26 todo 별도) · 마감 9/8
- [x] `webcam_register_node.py`: `mean_hist()` 스케일 정정 — 평균 뒤 **L2 정규화**로 바꿨다(`hsv_hist` 가 `cv2.normalize` 기본값 NORM_L2 라 L2=1 이다). 종전에는 합을 128 로 맞춰 발행 `color_hist` 의 L2 가 **31.6** 이었다. 유사도는 Bhattacharyya 척도 불변이라 영향 없음을 실측 확인(차이 5e-09) — 오늘 낸 측정값은 그대로 유효하다 (PR #26 PM 노트) · 마감 9/8
- [ ] `webcam_hint_node.py`: `DEFAULT_MODEL_PATH` 를 `mro_perception/models/worker.pt` 로, `mro_webcam/models/best.pt` 사본 삭제 (단일 출처, PR #26 노트) · 마감 9/8  **→ 절반 (9/9 확인): `DEFAULT_MODEL_PATH` 는 `mro_perception/models/worker.pt` 로 됐으나 `mro_webcam/models/best.pt` 사본이 아직 남아 있다**
- [x] `webcam_register_node.py`: 헤더 주석 '관제센터 시야' → 출입구, `등록 구역 게이트` TODO 삭제 — 시야 전체가 등록 구역이라 게이트 불필요 (PR #16, Q-26, PR #20) · 마감 9/8
- [x] `webcam_register_node.py`: `webcam.yaml` 파라미터화 — `webcam_index`·`conf` (PR #26) (9/7 내장 카메라가 잡히고 conf 를 못 낮춰 두 번 막힘, PR #20 노트). yaml 의 `model_path: ""` 는 launch 가 덮으므로 그대로 두되 주석 정정 · 마감 9/8
- [x] **웹캠1 출입구 체류 시간 실측 (Q-26 해소)** — 실측 결과 문제 없다. 등록 없는 상태에서 시작해 **첫 통과에 두 명 다 등록**(0.90 / 0.91 s). 체류 구간 **18.99 · 16.30 s** 로 필요한 **1.3 s** 의 10배 이상 여유. 재등록 3/3(sim 0.71·0.75·0.77) · 마감 9/8
- [x] **웹캠2 발행 주기 저하 (I-016)** — 캡처 스레드·BUFFERSIZE·주기 분리·CompressedImage 를 `webcam_hint_node.py` 에 병합 (PR #14, 9/7 머지) · 마감 9/7
- [ ] `webcam_hint_node.py`: **YOLO 타이머 재진입 차단 (I-027)** — 개별 `MutuallyExclusiveCallbackGroup`, `read()` 실패 대기, `stop → join → release` 종료 순서 코드 완료 (PR #21). **9/8 부분 확인:** YOLO·JPEG 동시 실행 중 이미지 9.9~10.1 Hz 유지, 지연 누적·2초 이상 중단 없음, 정상 종료 확인. 남은 것은 실제 연속 Detection 중 track ID 안정성과 CPU 사용률 확인 · 마감 9/8
- [x] `webcam_hint_node.py`: `detection_confirm_count`(3 프레임) → **초 단위** `detection_confirm_s`(0.3) 로 — 계약 「시간 상수 표기」 규칙(프레임 수 금지, `webcam.yaml` 헤더). `process_hz` 를 바꾸면 의미가 조용히 달라진다 (PR #21 PM 노트) — PR #32 (`detection_confirm_s` 0.3 s, `ceil(초 × process_hz)`) · 마감 9/8
- [x] `webcam2.py` 는 **호모그래피 검증 도구로 유지** (PR #32: 클릭·수동 픽셀 → map 변환 → Nav2 goal. 프로덕션 launch 는 `webcam_hint_node` 만 띄운다) · `webcam2_subscriber.py` 는 삭제 (PM 결정 9/7, PR #33 질문 답) · 마감 9/8  **→ 완료 (9/9 확인) — `webcam2_subscriber.py` 없음, `webcam2.py` 유지**
- [x] `webcam_hint_node.py`: 카메라·YOLO 연결, 트랙-작업자 대응, `TargetHint` 발행 (PR #14) · 마감 9/8
- [ ] `webcam_hint_node.py`: `worker.pt` 교체 후 **TS-U02** — **9/8 통신 구간 부분 통과:** 압축 영상 9.9~10.1 Hz·1.57~1.62 Mbps, pub→sub p95 ≤3.1 ms·max 75.8 ms·중단 0회, 가짜 `TargetHint` 20건→KPI CSV 20건(누락 0%). 남은 것은 실제 Detection→Hint→KPI, 손 흔들기→화면 ≤0.2 s, 힌트 map 좌표 vs held-out 실측 ≤0.3 m (`docs/TS-U02_20260908.md`) · 마감 9/8
- [x] `kpi_logger_node.py`: CSV 초안 · 마감 9/8
- [x] `tools/kpi_report.py`: 추종거리·IPH·리드타임·성공률·재획득 계산 구현 (PR #8) · 마감 9/10
- [x] **`kpi_report.py`: 배송 판정(AC-08)·리드타임을 `~/mro_fleet.db` 에서 읽기 (I-023)** (PR #15, PM 합성 DB 대조 일치) · 마감 9/8
- [x] `kpi_report.py`: 추종거리를 FOLLOWING 구간으로 자르고 `|d−0.5|` p95 와 **최소 이격** 출력 (I-022, AC-05, PR #15) · 마감 9/9
- [x] `kpi_report.py`: Lost 를 `target/visible` 전이로 잡고 재획득 **성공률** 산출 — SEARCHING 은 정상 절차에도 쓰인다 (I-022, K-04, PR #15) · 마감 9/9
- [x] `kpi_report.py`: **K-04 이중 계상 수정 (I-028)** — 포기/성공/미복귀를 한 함수에서 판정 (PR #17, PM 합성 로그 재대조 일치) · 마감 9/9
- [x] `kpi_report.py`: `--db` 파일은 있는데 `missions` 테이블이 없으면 `OperationalError` 로 통째로 죽는다 — 잡아서 안내문으로 (관제 DB 초기화 전 TS-07 대비) (PR #20, PM 빈 DB 로 exit 0·K-04 계속 출력 확인) · 마감 9/8
- [x] `kpi_report.py`: IPH 분모를 undock~도킹 완료 구간 합으로 (I-022, BRD 5.3, PR #15) · 마감 9/9
- [x] `kpi_logger_node.py`: `target/distance` 구독(FR-29), 로봇 네임스페이스 파라미터화, QoS 계약 반영 (PR #8) · 마감 9/9
- [x] `kpi_logger_node.py`: `/fleet/zone_event` 구독 → CSV kind ZONE · `kpi_report.py` **K-06 교착 자동 해결률** 산출 (I-030, 계약 9절: REJECT→GRANT 60 s 이내, RECLAIM 은 미해결) — PR #33 (`zone_cb` CSV source=zone · `zone_resolution()`) + **PR #34 로 RECLAIM 을 구역 단위 판정으로 보정, I-032 해결** (PM 재대조 8 시나리오 일치 · 무관 로그 400건 차등 퍼즈 불일치 0) · 마감 9/9
- [x] `kpi_report.py` K-06 마무리 셋 (PR #34 후속, 막지 않음) — ① `zone_resolution` docstring 의 `[REJECT, GRANT)` 를 양끝 포함으로 정정(안쪽 주석과 어긋남) ② `reclaimed_between(...): pass` 죽은 코드 삭제 ③ **`강제 회수` 라벨을 `강제 회수(구역)` 로** — 이제 구역 전체 RECLAIM 수라 대기자 없이 보유만 초과해도 세어져 `대기 1 · 자동 해결 1 · 강제 회수 1 → 100.0%` 로 읽힌다 (I-032) · 마감 9/9 TS-08 전  **→ 완료 (PR #42) — 9/9 확인: `[REJECT, GRANT]` 양끝 포함(:189), `reclaimed_between` 이 :234 에서 실제로 쓰임(죽은 코드 아님), 라벨 `강제 회수(구역)`(:366)**

## mro_navigation [주행 — 박병후, 이시율]
- [ ] **T_lim 120 s 실측 확정 (BRD Q-14) — 오늘 오전 중** — 새 축소 아레나 동선에서 **요청 접수 → 대기공간(Z-05) 도착** 실제 리드타임을 재고 120 s 를 유지할지 고칠지 정한다. 측정 수단은 이미 있다 — `tools/kpi_report.py --db ~/mro_fleet.db` 가 `waiting_at − created_at` 으로 K-05 평균·p95 를 낸다 (PR #15). **깨끗한 완주 회차만 센다** — I-033(P1 잠금)·I-035(Nav2 미활성)로 태운 시간은 동선 시간이 아니라 버그 시간이다. 짧게 잡으면 멀쩡히 배송한 회차가 실패로 찍혀 AC-08(≥ 95 %)이 떨어지고, 길게 잡으면 기준이 아무것도 거르지 못한다. **바꾸면 `common.yaml` 한 곳만 고친다** — 로봇(`shuttle_mission`)과 관제(`fleet_manager`)의 공유 계약값이라 갈라지면 로봇은 계속 가는데 관제만 `TLIM_EXCEEDED` 를 찍는다. **BRD v3.6 제출이 9/11 09:30 이라 오전 중 결론이 필요하다** · 마감 9/10
- [x] **PR #56 실주행 확인 — 재탐지 3.5 s · 회전 방향 잠금** — 9/9 세 회차로 확인했다. 재탐지: `SEARCHING` 316.98 s → 첫 회전 318.49 s = 1.51 s, 마지막 본 뒤로는 `research_s` 2.0 + `search_wait_s` 1.5 = **3.5 s** (`follow_0909_1451`). 방향 잠금: `+20°` **8회 연속 · 되돌아옴 0회**. **단 힌트가 있으면 잠금이 풀린다 — I-050 으로 등록했다.** **보이면 즉시 정지(`stop_on_sight`)도 확인됐다** — `follow_0909_1609` 에서 3건 발동 (`회전 중 작업자 발견 — 그 자리에서 멈춘다` 167.99·189.91 s). 그중 한 건은 회전 0.8 s 만에 끊고 1.6 s 뒤 FOLLOWING 에 들어갔다. **PR #56 의 미확인 3건이 전부 닫혔다** · 마감 9/10
- [ ] **웹캠2 힌트 발행이 48 s 끊긴 사유 (I-050 곁가지)** — `follow_0909_1609` 에서 10 Hz 로 오다 48.09 s 공백. 우리 쪽은 `hint_valid_s` 2 s 로 걸러 오동작하지 않으나 사각지대인지 노드 재시작인지 웹캠팀에 확인. 데모 때 그 구간이 길면 힌트 없는 재탐색으로 돌아간다 · 마감 9/10
- [ ] **bag 기록 목록에 `target/worker_id` 를 넣는다** — `hint_bearing()` 이 이 값으로 남의 작업자 힌트를 거르는데(`if self.worker_id and hw and hw != self.worker_id`), 비어 있으면 **조건이 통째로 건너뛰어져 모든 힌트를 받는다.** 힌트 토픽이 전역(`/webcam2/target_hint`)이라 **2대 운용에서 상대 로봇의 작업자 힌트로 회전 방향을 정할 수 있다.** 9/9 세 회차 모두 이 토픽을 기록하지 않아 실제로 채워지는지 확인이 안 됐다 · 마감 9/10
- [ ] **TS-04·TS-14 재실행 (I-036)** — `clear_dist` 를 0.45 → 0.38 로 낮췄으므로 기존 통과 기록은 무효다. **해제가 너무 일러 estop 재진입이 반복되지 않는지**를 특히 본다. **I-049 와 같이 본다** — 탈출 경로가 없는 채로 돌리면 통과 기록이 의미가 없다 · 마감 9/9
- [ ] **estop 탈출 회전을 워치독이 직접 만든다 (I-049, 상) — PM 승인 완료 9/9** — 1차 `SAFE_ROTATE`(병진 0 강제 · 각속도 ±0.2~0.3 rad/s 를 워치독이 **직접 생성** · **2~3 s 상한**), 실패하면 2차 `STUCK`/STANDBY + 관제 보고. **외부 `cmd_vel` 을 통과시키는 방식이 아니다** — 그러면 estop 을 무시한 노드가 로봇을 움직이는 구조가 된다. 상한이 없으면 정말 끼였을 때 끝없이 빙빙 돈다. **PM 조건 넷**: ① 워치독에 `linear.x ≠ 0` 경로가 하나도 없을 것 ② 회전 시간 상한 필수 ③ 회전 방향은 `/scan` 부채꼴 좌우 중 먼 쪽 ④ **데모 전 TS-04·TS-14 실물 확인** — 확인 못 하면 데모에서는 워치독을 끄는 편이 낫다. `STUCK` 새 값은 만들지 않고 `STANDBY` 를 쓴다 · 마감 9/10
- [ ] **회전 과도 9~34° 를 없앤다 (I-051)** — `odom` 이 늦게 와서 이미 지나친 뒤에 멈춘다. 초과분은 상수가 아니라 **그때의 `odom` 지연 × 각속도** 라 산포다(PM 검산: 같은 회차에서 +9.8·+27.8·+33.2·+33.8°). **③ `SEARCH_ROTATE_SPEED` 1.5 → 0.8 rad/s 가 산포 폭을 줄이는 1순위** (모듈 상수라 파라미터화가 먼저다) ② 감속 하한 0.4 를 낮춤 ① `search_lead_deg` 고정 리드는 산포를 못 지우므로 위 둘 뒤에 남는 몫으로만. **셋을 같이 넣고 한 회차로 재측정** — 따로 재면 회차가 세 번 든다. 판정은 「명령 20° 에 실제 20±5°」와 「`(n/18)` 이 실제 한 바퀴와 맞는가」 · 마감 9/10
- [ ] **`max_search_tries()` 의 `한 바퀴 18회` 를 실제와 맞춘다 (I-051 곁가지)** — `360/20` = 18 인데 실제 회전량이 32° 라 **11~12회**에 한 바퀴다. 과도를 고치면 저절로 맞으므로 I-051 뒤에 확인만 한다 · 마감 —
- [ ] ~~힌트도 첫 회차에만 방향을 정하고 잠근다 (I-050)~~ **보류 (9/9 실주행 판정).** 힌트 회전이 잠금보다 재획득이 **2.2배 빠르고**(2.81 s vs 6.21 s) 좌우 반전 → 실패 사례가 **0건**이다. 부호가 뒤집힌 원인도 힌트가 아니라 회전 과도(I-051)였다 — 힌트 좌표는 12초간 0.1 cm 밖에 안 움직였다. **로그 모양만 보고 잘 되는 동작을 바꾸지 않는다.** 다수결 방식은 사람이 정면을 가로지르는 경우에 값이 있으므로 I-050 의 재검토 조건이 성립하면 그때 넣는다 · 마감 —
- [x] **`DriveOnHeading` 오버슛 6.8 cm 보정 — TS-07 전 1순위 (I-044 후속)** — 지금 베이에 서는 자리가 `base_link` 0.835 로 **하드 바닥 0.805 위 3.0 cm** 뿐이다. 잔차 산포가 7 mm 라 estop 이 걸릴 자리는 아니지만 얇다. 잔차 −0.068 이 **계통 오차임이 두 방법으로 확인됐으므로**(줄자 −0.064 · 노드 자체 −0.071) 그만큼 더 명령하면 0.900 에 서고 설계 여유 9.5 cm 가 돌아온다. `bay_align_min_range_m` 0.45 → 0.80 도 같이 (PM 요청, 미이행) · 마감 9/10  **→ 완료 9/10 (PR #80) — 상수 0.068 + 비례 0.0 두 항을 다 뒀다. 두 표본의 명령값이 0.270·0.257 로 거의 같아 상수/비례가 안 갈렸다고 밝혔다 — 짧은 이동(0.10 m 급) 회차 하나면 갈린다. `bay_align_min_range_m` 0.35 → 0.80 도 같이 (PM 요청 미이행분)**
- [ ] **R2 `xy_goal_tolerance` 0.25 → 0.10 은 이미 실패한 값이다 (PR #74 계획 재검토)** — I-029 에 「0.25 → 0.15 → 0.1 로 낮출 때마다 목표 근처에서 좌우로 진동하며 도착 판정을 못 한다(9/7 robot3 실측)」가 있고 I-013 에도 같은 기록이 있다. **그대로 돌리면 회차 하나를 버린다.** 무엇이 달라졌는지(AMCL `update_min_d` 0.05 적용 등)를 먼저 적고, 아니면 오버슛 보정 + `bay_align_min_range_m` 으로 같은 사슬을 끊는 편이 낫다 — 「덜 간 채 완료」를 메우는 것이 `bay_align` 의 원래 일이다 · 마감 9/10
- [ ] **`bay_align_max_lat_m` 0.30 을 `d_n` 비례로 (PR #80 후속)** — 고정값인데 안전 여부는 거리에 비례한다. ①을 건너뛰면 ②가 법선 방향으로 돌아 마커가 `atan(e_lat/d_n)` 만큼 옆으로 가는데, **PR #76 이 시야를 38.5 → 37.4° 로 좁혀** 목표 0.9 m 에서 여유가 0.82 → **0.27°** 가 됐다. `cap = d_n · tan(반시야)` 면 0.9 m 에 0.305 · 0.8 m 에 0.271 이다. 0.25 로 내리는 것만으로도 충분하다. **막지는 않는다** — 놓치면 이전 `d_n` 으로 넘어가는 경로가 있다 · 마감 —
- [ ] **베이 마커 세 장의 흰 여백을 눈으로 확인 (I-052, 상) — TS-07 전** — 여백 없이 벽에 바로 붙으면 거리가 **18~25 cm** 틀리고 검출이 깜빡이는데 **재투영 오차는 0.01 px 로 깨끗해 지표로 안 걸린다**(9/9 robot8 실측). 베이 여유는 **3.0 cm** 뿐이라 어느 방향이든 무너진다. ① 베이 A·B·C + ID 4 의 여백 3~5 cm 와 평평함(그림자 선) 확인 ② `tools/aruco_live_check.py` 의 `implied marker` 가 10.0 cm 근처인지 — 벗어나면 종이 문제다. 계약 6절에 요구를 올렸다(v1.20) · 마감 9/10
- [ ] **`safety_watchdog` 에 네임스페이스 가드 (I-048)** — 나머지 셋(`:178`·`:114`·`:82`)에는 있고 **안전 노드에만 없다.** 없이 뜨면 상대 토픽이라 `/scan` 을 못 받아 오류 없이 estop 을 영영 안 건다. `return_dock_node.py:80` 의 가드를 그대로 · 마감 9/10
- [ ] **주행 README 의 불변식에 I-033 을 밝힌다 (PR #64 후속)** — 「요청한 토큰은 모든 종료 경로에서 반납한다」가 지금 `shuttle_mission` 에서 지켜지지 않는다. 코드리뷰 자료가 열린 상 이슈를 가린다. **리뷰 전에** · 마감 9/10
- [ ] **`docs/diagrams/README.md` 목록에 `navigation_review_flow.drawio` 추가 (PR #64 후속)** — 그 README 가 「손으로 고치지 않고 생성기를 고친다」라고 적혀 있어, 손으로 만든 파일이 하나 들어온 것을 밝혀두지 않으면 다음 사람이 `system_flowchart.drawio` 도 손으로 고친다 · 마감 9/10
- [x] **추종 STANDBY 판정을 SEARCHING 과 맞춘다 (I-046)** — `step_following`(:763~764)이 `target/visible` 만으로 `follow_seen` 을 되돌려, 보이는데 좌표가 없으면 `standby_s` 에 영영 도달하지 않는다. 같은 상황에서 SEARCHING 은 STANDBY 로 간다. **웹캠2 힌트를 추종에서도 쓰는 쪽이 더 나은 해법일 수 있다** — 지금 `self.hint` 는 SEARCHING 의 회전 방향으로만 쓰인다. TS-07 을 막지 않는다 · 마감 9/10  **→ 완료 (PR #59) — 좌표 전용 시계 `coord_seen` 을 따로 두고 `visible` 로 버티는 것을 `visible_wait_s`(3 s)까지만 인정한다. `follow_seen` 에 직접 한도를 걸면 되살릴 때 재는 값이 0 으로 돌아가 조건이 영원히 참이 되므로 안 된다. SEARCHING 과 규칙이 같아졌다. **웹캠2 힌트를 추종에도 쓰는 것은 별건으로 남긴다** — 그것은 재획득을 빠르게 하는 일이고, `standby_s` 에 닿게 하는 것과는 다른 문제다**
- [x] **짧은 가림으로 탐색에 빠지지 않는지 실주행 확인 (I-046 · PR #59 후속)** — `visible_wait_s` 3 s 한도를 새로 걸었으므로, 사람이 로봇 앞을 스쳐 지나갈 때 `FOLLOWING` 을 유지하는지 다음 회차에서 본다. 단위 시험으로만 확인된 변경이다 · 마감 9/10  **→ 확인됨 (9/9 `follow_0909_1451`) — 좌표가 0.8~1.9 s 끊긴 구간 **20건 전부 `FOLLOWING` 유지**, 탐색으로 빠진 건 0. 최장 끊김이 1.92 s 로 한도 3 s 를 넘지 않아 **반대쪽(3 s 초과 → STANDBY)은 실주행 미확인**이다 — TF 가 끊겨야 나오는 조건이라 재현 수단이 없다. 단위 시험으로 남긴다**
- [ ] **힌트 유효기간을 수신 시각이 아니라 `Header` 시각으로 (I-047 곁가지)** — `hint_cb` 가 `time.time()` 을 저장해 `:453` 이 그것과 비교하므로 실제 뜻이 「힌트가 2 s 지났다」가 아니라 「발행이 2 s 끊겼다」다. `TargetHint.header` 에 촬영 시각이 이미 있다. 10 Hz 에서는 차이가 거의 없어 급하지 않다 · 마감 —
- [x] **`test_no_reverse_anywhere_in_source` 조건식 괄호 (PR #56 후속)** — `not 주석 and linear or undock` 이라 `.undock(` 쪽만 주석 검사를 건너뛴다. 오탐만 나므로 후진이 새지는 않으나, 후진 금지(계약 2절)를 지키는 유일한 자동 검사라 정확히 둔다 · 마감 9/10  **→ 완료 (PR #59) — 괄호 한 쌍으로 두 조건에 주석 검사가 같이 걸린다**
- [x] **추종 재탐지 시간 5개를 `common.yaml` 로 (PR #56 후속)** — `research_s`·`settle_s`·`search_wait_s`·`visible_wait_s`·`search_step_deg`. `lost_s`·`standby_s`(SOT 계약값)도 아직 노드 기본값에만 있어 이 노드의 시간값 7개가 코드에만 있다. 리그 런치가 이미 `common.yaml` 을 넘기므로 통로는 있다 (계약 5절) · 마감 —  **→ 완료 9/9 (PR #58) — 9개를 옮겼다(`lost_s`·`standby_s`·`approach_skip_m`·`hint_valid_s` 포함). 노드 기본값은 대체값으로 남겼고 11/11 값이 PR #56 과 일치한다. 덤으로 죽은 `navigation.yaml` 삭제 — 그 파일에 PR #56 이전 값(`lost_research_s` 8.0·`search_rotate_deg` 30.0)이 남아 있어 「중복 통합」 방향으로 정리했으면 재탐지가 11 s 로 되돌아갔다**
- [ ] **베이 접근 잔차 규명 (I-044, 상)** — 여유 0.095 m 가 관측 잔차 0.275 m 보다 작다. `d3e0568` 로그로 다음 회차에 가리거나, 못 가리면 `bay_marker_range_m` 을 1.1 m 이상으로. **둘 중 하나를 하고 TS-07 에 들어간다** · 마감 9/9
- [ ] **`bay_align_min_range_m` 을 0.80 으로 (PR #52 후속)** — 지금 0.45(노드 기본값 0.35)라 자기가 잰 하드 바닥보다 낮다. 잘못된 목표를 거르는 유일한 방어인데 8회차로 배운 것을 못 막는다 · 마감 9/9
- [ ] **접근·재탐지 파라미터의 단일 출처는 `common.yaml`** (PR #52) — `bay_marker_range_m`·`bay_align_*`·`bay_exit_*`. 노드 기본값은 단독 실행용 대체값 · 마감 —
- [ ] **토큰 서비스 미기동 시 진행하는 것 (I-041, 상)** — `shuttle_mission`·`return_dock` 둘 다 `wait_for_service` 실패 시 `True` 를 돌려 **획득한 것으로 치고 간다.** 2대 운용에서 관제가 잠깐 죽으면 두 로봇이 동시에 P1(0.79 m)에 들어간다. `require_token` 파라미터로 갈라 fleet 운용은 포기하도록. **두 노드 같이** · 마감 9/9
- [ ] **I-033 나머지 절반 — `zone_semaphore_node` 가 반납 요청으로 대기열에서 빼는가** — 클라이언트는 PR #46 으로 끝났다. 세마포어가 안 빼주면 그대로 잠긴다(60 s 회수는 보유자만 대상). **이게 닫히기 전까지 S10 복귀를 2대로 돌리지 않는다** · 마감 9/9
- [ ] **접근 수렴 실주행 확인 (PR #49)** — 3 m 넘는 거리에서 시작해 회차마다 gap 이 실제로 반씩 주는지. 멀리서 잰 좌표가 0.5~0.7 m 틀린다는 것이 이 설계의 전제인데, 그 오차가 회차마다 줄어드는지가 전제의 확인이다 · 마감 9/9
- [ ] **P1 통과 — 측위가 유일한 경로다 (I-009 PM 판단 9/8, 정정)** — 통과 확실에 필요한 통로는 0.99 m 인데 교행 불가를 유지하려면 0.78 m 미만이어야 한다. **폭으로는 못 푼다.** 인플레이션을 최대로 낮춰도 12 cm(한쪽 6 cm)뿐이라 측위 오차 32 cm 앞에서 의미가 작다. **할 일은 I-029(`update_min_d` 0.05 실주행 확인)와 I-037(판정을 `amcl_pose` 로) 둘.** 목표는 오차 0.32 → 0.22 m 미만 · 마감 9/9
- [ ] (곁가지, 공짜면 한다) 받침대가 이미 통로 바깥인지 실측 확인 → 맞으면 `inflation_radius` 0.25 → 0.20 한 줄 변경. **12 cm 를 공짜로 얻지만 P1 을 풀지는 못한다** · 마감 9/9
- [ ] `Smac`·`MPPI` 교체와 지도 덧칠은 **하지 않는다** (9/8 PM) — 원인이 플래너도 지도도 아니라 측위다. Phase 2 후보로만 적어둔다 · 마감 —
- [ ] **Nav2 복구 행동에서 후진(`BackUp`)을 뺀다 (계약 2절 후진 금지)** — 우리 코드에는 `linear.x < 0` 이 없지만 **Nav2 기본 BT 의 복구가 후진을 낸다.** 로봇 PC 의 `turtlebot4_navigation/config/nav2.yaml` 과 BT xml 소관이라 저장소 밖이다 — `nav2.yaml`·`localization.yaml` 과 같은 종류의 문제로, **PC-R3·PC-R8** 둘 다 손으로 맞추고 무엇을 바꿨는지 적어둔다. 제자리 회전(`Spin`)과 `Wait` 만 남긴다. **I-049 로 저장소 안에서 막을 길이 생겼다 (9/9)** — 워치독이 유일한 `cmd_vel` 기록자가 되므로 릴레이 경로에 `linear.x = max(0.0, linear.x)` 를 넣으면 로봇 PC 설정에 의존하지 않는다. 주행에 요청해 두었다. 둘 다 하는 편이 낫다 — 클램프는 강제, BT 정리는 애초에 안 내게 하는 것 · 마감 9/8
- [x] **`safety_watchdog_node.py`: estop 중 병진 전부 차단 · 저속 회전만 통과 · `CLEAR_DIST` 0.45 → 0.38 (I-036, 계약 v1.13)** — **후진은 쓰지 않는다**(계약 2절 확정: 범퍼·클리프가 전방에만 있다). 각속도 상한 0.4 rad/s — 제자리 회전이 쓸고 가는 폭은 1.4 cm 뿐이고 유일한 탈출 수단이다. `CLEAR_DIST` 0.45 는 **P1 반폭 0.395 m 보다 커서 통로 안에서는 어느 방향으로 돌아도 해제가 안 된다.** Nav2 goal cancel 도 함께. **변경 뒤 TS-04·TS-14 재실행** — 해제가 너무 일러 재진입이 반복되지 않는지 특히 본다 · 마감 9/8  **→ 완료 9/8 (PR #52) — 파라미터화까지. **TS-04·TS-14 재실행이 남았다****
- [ ] **I-035 실주행 확인** — 새 기동 순서(T1 → 리그 → T7 → T3)로 `global_costmap` 이 곧바로 `active` 가 되는지. 9/8 에는 로봇을 쓰지 못해 단위 검증까지만 했다. **TS-07(9/9) 전에** · 마감 9/9
- [x] **신규 파라미터 2개 `robotN.yaml` 이관 요청 (I-035)** — `nav2_poll_max`(3) · `dock_status_wait_s`(5.0) 가 아직 코드 기본값뿐이다. 값의 단일 출처는 params 다 (계약 5절). **PM 요청 필요** · 마감 9/9  **→ 완료 9/8 (PM) — `robotN.yaml` 이 아니라 **`common.yaml`** 이 맞는 자리였다**
- [x] **`shuttle_mission_node.py`: 초기자세를 노드 기동 시로 (I-035)** — 미션을 받은 뒤 보내면 Nav2 가 이미 활성화에 실패한 뒤다. `dock_pose`·`getDockedStatus()` 는 기동 시에도 안다. Nav2 가 정해진 횟수 안에 `active` 가 안 되면 **T_lim 을 태우지 말고 즉시 FAILED + 사유** · 마감 9/8
- [x] **`test_follow_target_fsm.py` 신규 (PR #39 후속)** — 상태기계 검사 18개가 저장소에 없어 재현·회귀 검출이 안 된다. 셔틀의 `test_shuttle_mission_fsm.py` 와 같은 방식으로 · 마감 9/8  **→ 완료 9/8 (PR #46) — 508줄**
- [ ] **P1 통과 10회 연속 성공 (4단계 게이트, I-009)** — 측위 오차 0.32 m > 통과 여유 0.175 m 인 동안 2대 교행을 잡지 않는다. I-029·I-037 이 선행 · 마감 9/9
- [ ] `arena.yaml` `loc_check_marker.xy` 실측 — 지금 `[0.0, 0.0]` 은 맵 원점이라 관제가 그대로 읽으면 정상인데도 측위 의심으로 기록한다 (PR #39 지적) · 마감 9/8
- [x] 아레나 재구성 · 마감 9/5
- [x] SLAM 맵 재작성, `mro_bringup/maps/auto_map.*` 로 교체 · 마감 9/5
- [x] 실측 절차·양식 문서화 — `docs/measurement_guide.md` · 마감 9/5
- [x] **통신 구성** — offboard 단일 서버 PC-F `192.168.108.41` ID 1. 로봇 2대·PC 5대 접속 확인 · 마감 9/5
- [x] 공유기 DHCP 예약 요청 — **불필요로 종결 (9/7)**: PC-F 로 지정한 노트북은 학원 지원 장비라 재부팅해도 IP(`.41`)가 유지된다고 학원이 확답. 시연 전 `ip a` 로 `.41` 한 번만 확인 · 마감 9/7
- [x] **`params/arena.yaml` 좌표 입력 (I-001)** — bays A/B/C, waiting_zone, dock_zone, `semaphore_zones` 폴리곤 실측 기입 (9/6) · 마감 9/6
- [x] **`dock_zone.pose` · `robotN.yaml` `dock_pose` 실측 기입 (I-013)** — 두 파일 값 일치 확인 (9/6) · 마감 9/6
- [x] I-004 받침대 돌출폭 실측 (9/6) — P1 없음 · P2 양쪽 3 cm(폭 0.86 정정) · P3 입구 5 cm·안쪽 6.5 cm · 마감 9/7
- [ ] **`inflation_radius` 0.05 → 0.25 적용 (I-009)** — 주행 합의값. 승인 후 PC-R3·PC-R8 의 `nav2.yaml` 양쪽에 같은 값으로 · 마감 9/8
- [ ] **측위 검증 마커 설치·실측** — ArUco ID 4 거치대(웹캠1 박스 북쪽 ≈ 0.4 m 앞 거치대, 북향, 10 cm, 높이 23.5 cm), map 좌표를 `arena.yaml` `loc_check_marker.xy` 에 (계약 6절·9.2 ①) · 마감 9/7
- [x] **I-013 검증 (9/7)** — ArUco 미설치라 도킹 상태 `amcl_pose` 비교로 대체. 밀림 방향이 둘 다 −x 로 같고 차이 5.1 cm, 도크 간격 측정 0.803 대 줄자 0.840 m → **map 프레임 상호 일치 확인**. 개별 20 cm 치우침은 `xy_goal_tolerance` 0.25 에서 오는 것 · 마감 9/7
- [x] TB4 2대 동시 운용 검증 (9/6) — 네임스페이스 격리 · `odom` 20 Hz · OAK-D 동시 스트리밍 · RPi CPU 21~31 % · **Nav2 2벌 동시 `active`** 확인 (I-011) · 마감 9/6
- [x] **좌표 보정 3건 (I-001 PM 검증)** — holding ≥1.5m(1.52m), bays 전부 DOCK 폴리곤 밖, P1 폴리곤이 통로를 완전히 가로지르도록 재실측 (9/6) · 마감 9/7
- [x] 베이 B·C waypoint 확정 실측 (9/6, x=-0.78 로 통일) · 마감 9/7
- [x] `arena.yaml` `homography_marks` — cam2 기준점은 `mro_webcam/config/WebCam2/marker_coordinates.json` 이 원본(PR #9), cam1 은 불필요(9/6). arena.yaml 에는 복사하지 않는다 · 마감 9/7
- [x] `shuttle_mission_node.py`: `mission/assigned` 트리거, bay waypoint → ArUco 확인 → 체류 → 대기공간, zone_token, `shuttle/arrived` — `aruco/id` 는 **1.5 m 안에서만** 나온다 (PR #30 거리 게이트) · 마감 9/7
- [ ] **베이 A·B 정차점 서쪽으로 ≥ 0.15 m 이동 또는 DOCK 폴리곤 확장 (I-025)** — 접근 지점과 0.31/0.38 m 라 몸통이 겹친다. TS-07 전 · 마감 9/8
- [x] **estop 중 Nav2 복구 행동이 막히는 문제 (I-036)** — 벽 앞에서 후진·회전까지 0 으로 덮여 95 초를 못 빠져나왔다. 후보 ①전진 성분만 차단 ②N초 후 워치독이 후진 ③goal cancel — **PM·안전 검토 필요** · 마감 9/8  **→ 코드 조치 완료 9/8 (PR #52). 남은 것은 TS-04·TS-14 재실행과 Nav2 goal cancel**
- [ ] **도착 오차 수치 신뢰성 확인 (I-037)** — 노드 TF 0.84 m vs 같은 순간 `amcl_pose` 0.18 m. 3회차에 `tf2_echo` 기록 + **베이에서 줄자 실측**으로 가른다 · 마감 9/8
- [ ] **P1 통과 실주행 확인 (I-009)** — 여유 0.175 m < 실측 측위 오차 0.32 m 라 통과가 운에 좌우된다. `inflation_radius` 0.15 실험 후 결정. **PC-R3 는 0.25 적용 확인됨, PC-R8 미확인** · 마감 9/8
- [x] **I-035 조치 결정** — PM 이 ①(절차+코드)로 확정하고 ②(노드가 RESUME)·③(`t0` 이동)은 반려했다 (9/8). ③은 T_lim 기준점이 요청 접수 시각이라(계약 9절·FR-09) 옮기면 AC-08 이 실제보다 좋게 나온다. 구현은 위 줄 · 마감 9/8
- [x] `return_dock_node.py`: 복귀 시 **P1 토큰 획득·반납** — 지금은 DOCK 토큰만 있다 (계약 3절: S10 도 P1 통과) · 마감 9/8  **→ 완료 9/8 (PR #46)**
- [x] **`shuttle_mission_node.py`: `shuttle/status` 발행 (I-024)** — READY/SHUTTLE/AT_BAY/WAITING/FAILED. 지금은 발행자가 없어 배송 성공률이 구조적으로 0 % 다. **`FAILED` 는 포기할 때만** — 재시도 중이면 SHUTTLE 유지 (관제가 FAILED 를 미션 실패로 기록한다, I-026). **TS-07 이 9/9** · 마감 9/7
- [x] `follow_target_node.py`: standoff 0.5, 힌트 방향 회전, STANDBY, paired 게이트, estop 존중 · 마감 9/8  **→ 완료 (9/9 확인) — `follow_standoff_m`·힌트 회전·STANDBY·`paired_cb`·`estop_cb` 모두 있음**
- [ ] **AMCL `update_min_d` 0.05 실주행 확인 (I-029)** — 9/7 저녁 적용(0.25→0.05). 도착 판정이 안정되는지, 리샘플링이 잦아져 pose 가 더 떨리지는 않는지. 떨리면 `resample_interval` 2로 · 마감 9/8
- [ ] **PC-R8 에 `localization.yaml` `update_min_d: 0.05` 복제 (I-029)** — `turtlebot4_ws` 는 git 밖이라 PC 마다 손으로. I-009 의 `nav2.yaml` 과 같이 확인 · 마감 9/8
- [ ] **추종거리 분포 실측 (I-029)** — `xy_goal_tolerance` 0.25 고정이라 정지 시 오차 25 cm. TS-U07 로 `target/distance` p95·min 을 재고 미달이면 `cmd_vel` 미세 보정 구현 (PM: ③→② 순서, 기준 완화는 마지막) · 마감 9/8
- [x] `follow_target_node.py`: 힌트 구독을 `/webcam2/target_hint`(**TargetHint, sensor_data QoS**) 로 — 지금은 옛 `webcam/car_map_point` PointStamped RELIABLE 이라 BEST_EFFORT 발행자(PR #14)의 힌트를 못 받는다 · 마감 9/8  **→ 완료 (9/9 확인) — `:129` `create_subscription(TargetHint, /webcam2/target_hint, qos_profile_sensor_data)`**
- [ ] **`shuttle_mission_node.py`: P1 대기 중 포기해도 반납을 부를 것 (I-033)** — 거절당하면 세마포어 대기열에 들어가는데 `token_held` 가 False 라 `release_token` 이 그냥 돌아온다. T_lim 으로 끊기면 유령 대기열이 남아 **P1 이 영구히 잠기고 watchdog 도 못 푼다**. `token_requested` 플래그 추가. 거절만 반복하다 T_lim 으로 끝나는 단위 시험도 함께 · 마감 9/8  **→ 9/9 확인: 아직 그대로다.** `release_token`(:518)이 `if not self.token_held: return`. I-033 표의 「클라이언트 해결」은 `return_dock` 만이다 (그쪽만 `requested` 플래그가 있다)
- [ ] **`shuttle_mission_node.py`: 성공 직후 `READY` 를 내지 말 것 (I-034)** — `kpi_report` 의 `DOCKED_STATES` 에 `READY` 가 있어 AC-13 운용시간이 대기공간에서 끊긴다. 한 사이클 재현에서 분모 390 s → 150 s, IPH 27.7 → 72.0 회/h (K-07 목표 ≤ 1.0). 성공 경로는 `WAITING` 유지, 포기 경로의 `FAILED`→`READY` 는 그대로 · 마감 9/8
- [x] `follow_target_node.py`: 시작 신호 구독을 옛 `goto_goal/arrived` → **`shuttle/arrived`** 로 (계약 4절, 옛 토픽은 '쓰지 않는 옛 토픽' 목록). `shuttle_mission` 이 계약 이름으로 발행하도록 바뀌면 이걸 안 고치는 순간 S5 합류가 시작되지 않는다 · 마감 9/8  **→ 완료 (9/9 확인) — `shuttle/arrived` 구독. 옛 `goto_goal/arrived` 없음**
- [x] **셔틀 실주행 시험 절차 문서화** — `docs/shuttle_test_guide.md`. 단계별 설계, 1단계 절차, 로그 해석·증상별 판별. `fleet_manager`·마커 없이 주행 골격만 떼어 확인하는 방법과 **그래서 무엇이 확인되지 않는지**를 같이 적었다 · 마감 9/8
- [x] **셔틀 시험 환경 스크립트** — `source tools/shuttle_env.sh [build|nav2]`. source 4종 + 빌드 + T3 전용 `ROS_SUPER_CLIENT=False` 를 한 줄로. 9/7 실패 1번(디스커버리 설정 누락)이 여기서 사라진다 · 마감 9/8
- [x] **셔틀 시험 사전 점검 스크립트** — `tools/shuttle_preflight.sh [pre|nav|mission]`. `is_docked` 실측·중복 노드·**네임스페이스 없는 스택·유령 노드**·`bt_navigator active`·토큰 서비스·마커 주기·미션 구독을 한 번에 본다. 9/7 실패 세 번이 전부 '환경을 확인하지 않아서' 였다 · 마감 9/8
- [x] **셔틀 1단계 시험 리그 launch** — `mro_navigation/launch/shuttle_step1_rig.launch.py`. 옛 T4·T5·T6(세마포어·안전감시·마커주입)을 한 터미널로. 셔틀·Nav2 는 로그 관찰 대상이라 묶지 않는다 · 마감 9/8
- [ ] **셔틀 실주행 1단계 완주** — 판정 7항목 (I-024 '확인하지 못한 것' ①) · 마감 9/8
- [x] `zone_semaphore_node.py`: P1·DOCK 토큰, FIFO, 타임아웃 회수 (관제 → 주행 이관) · 마감 9/8
- [x] `zone_semaphore_node.py`: `arena.yaml` 구역 폴리곤 좌표 입력 (9/6) · 마감 9/8
- [x] `zone_semaphore_node.py`: 진입 판정은 **순서 기반(방법 B)으로 확정** — 폴리곤 계산 안 함, 폴리곤은 로그 검산용 (PM 결정 9/7) · 마감 9/8
- [x] `zone_semaphore_node.py`: 반납 시 대기열 누락으로 구역이 영구 차단되는 버그 수정 (9/7) · 마감 9/8
- [x] **`zone_semaphore_node.py`: `/fleet/zone_event` 발행 (I-030, 계약 v1.11)** — `"<zone> <robot> <REJECT|GRANT|RELEASE|RECLAIM>"`. REJECT 는 대기 시작 1건만, GRANT 는 새 획득만 · 마감 9/8
- [x] `safety_watchdog_node.py`: /scan → estop, cmd_vel 0 · 마감 9/8
- [ ] `safety_watchdog_node.py`: Nav2 goal cancel (액션 클라이언트), TS-14 결함 주입 검증 · 마감 9/8
- [ ] `safety_watchdog_node.py`: `HIT_FRAMES`·`CLEAR_FRAMES` 를 초 단위 파라미터로 (계약 시간 상수 규칙) · 마감 9/8
- [x] 주석 정정 (9/7) — `return_dock_node.py` 도크 간격 0.84·IR 차폐 폐지 명시, `zone_semaphore_node.py` P1 통과 방향(창고에서 나올 때·복귀)·P3 폭 0.93 · 마감 9/7
- [x] `return_dock_node.py`: DOCK 토큰 + 재시도 2회 복귀 로직 · 마감 9/8
- [x] `return_dock_node.py`: 홀딩 포인트·**도크 정면 접근 지점**으로 goToPose 후 dock (도크 간격 0.84 m 라 필수) · 마감 9/8  **→ 완료 9/8 (PR #46)**
- [x] `follow_target_node.py`: `mission/finish` 구독 → 즉시 페어링 해제·goal 취소 (≤ 1 s, FR-30) · 마감 9/8  **→ 완료 (9/9 확인) — `finish_cb`→`finish_req`(:208), `cancelTask()`(:249). ≤ 1 s 실측은 미확인**
- [ ] `inflation_radius` **0.25** 실주행 확인 (I-009) — P1 통과 · P3 받침대 안 긁는지. 두 PC 값 일치 확인 · 마감 9/8
- [x] 도킹 반복 시험 TS-U12 (9/6) — 로봇별 10회, 차폐판 없이 오도킹 0 확인 (I-006 종결) · 마감 9/8

## mro_fleet [관제 — 김민준]

### 시연 전 상위 결함

- [ ] **`RobotStatus` 초기값이 유효한 값으로 나간다 (I-053, 상)** — `battery` 0.0 이 `fleet_manager_node.py:381` 의 배정 게이트(`>= battery_min` 0.25)에 걸려 **토픽이 안 온 로봇이 방전으로 취급**된다. 표시가 아니라 동작 결함이다. pose 0,0 은 미측위 로봇을 맵 원점에 그리는데 **robot8 도크가 원점에서 4.1 cm** 라 자기 도크에 선 것처럼 보인다. **수신 여부 플래그로 「미수신」과 「부족」을 가르는 것만 먼저 해도 동작 결함은 사라진다** — UI 표시(`위치 미상`)는 `FleetStatus` 변경이라 계약이 같이 움직인다 · 마감 9/11 시연 전  **PR #81 이 소스별 수신 시각·발행자 수를 만들어 뒀으므로 이제 짧다 (9/10).** 같이 볼 것: `battery_state`·`target/pair_state` 는 **주기 발행**이라 발행자 수만으로는 「노드가 살아 있는데 멈춘 것」을 못 잡는다 — 그 둘에만 나이 검사를 덧댄다 (I-055 남는 구분)

### 실환경·UI 확인
- [ ] **관제 맵 도크 라벨 겹침 (PR #78 후속)** — 라벨이 원 바깥으로 나가면서 도크가 가까우면(robot3·robot8 약 8 cm) 글자가 겹친다. **좌표는 정확하다** — 보기 문제다 · 마감 —
- [ ] **대시보드 JS 의 로봇 이름 하드코딩 (PR #51 후속)** — `uiState.robots` 가 `{robot3, robot8}` 고정 사전이라 `robot_namespaces` 를 바꾸면 화면만 조용히 어긋난다 (빠진 로봇은 「수신 대기」로 남고 새 로봇은 안 보인다). `/status` 응답의 키로 카드를 만들면 된다 — 백엔드는 이미 `fleet.yaml` 명단을 쓴다. 오늘 robot4 → robot8 을 한 번 겪었다 · 마감 9/10
- [ ] **FR-13 대시보드 실제 조작 확인 (PR #47·#51)** — 브라우저에서 ① `GATE_FAIL` 발행 시 버튼이 뜨는가 ② 누르면 `인지 서비스 응답 없음` 이 뜨는가(인지 서버 미구현이라 이게 정답) ③ **그때 `MANUAL_PAIR` 개입 이벤트가 발행되지 않는가** — 실패한 시도를 개입으로 세면 AC-13 IPH 가 부풀어 실제보다 나빠 보인다 · 마감 9/9

### 후순위·외부 입력 의존

- [ ] **FR-35 는 `loc_check_marker.xy` 실측 뒤에 켠다** — `[0.0, 0.0]` 인 채로 읽으면 측위가 정상인데도 `localization_suspect` 로 기록된다. 그리고 **`aruco/distance`(카메라 기준)와 `|amcl_pose − xy|`(base_link 기준)의 6 cm 오프셋을 빼고 비교**한다 (TF 실측, `tolerance_m` 0.10 의 60 %) · 마감 9/9
- [ ] `fleet_manager_node.py`: `WAITING` 수신 시 `amcl_pose` 가 `waiting_zone` 근처인지 확인, 벗어나면 `FAIL_WRONG_POSE` (계약 9.2 ②) · 마감 9/8
- [ ] `fleet_manager_node.py`: `aruco/id`·`aruco/distance` 구독 → ID 4 수신 시 `|amcl_pose − loc_check_marker.xy|` 와 대조, ≤ 10 cm 면 `localization_verified` 이벤트, 아니면 `localization_suspect` (계약 9.2 ①). **기준점 주의:** `aruco/distance` 는 카메라→마커, `\|amcl_pose − xy\|` 는 base_link→마커 — `tf2_echo base_link oakd_rgb_camera_optical_frame` 으로 오프셋을 재서 뺀다 (I-031) · 마감 9/8

### 완료

- [x] **상세 화면 탭 선택 유지 (I-056)** — `/status` 1초 폴링은 `refreshRobotDetail()`로 내용만 갱신하고 `openRobotDetail()`의 기본 정보 탭 초기화 흐름을 다시 타지 않도록 분리. 회귀 테스트 포함 45건 통과 · 마감 9/10
- [x] **`control_ui`: 명시 페어링 폴백 버튼 (FR-13, 계약 10절)** — `target/pair_state = GATE_FAIL` 이면 버튼 표시, 운영자가 로봇·작업자 선택 → `fleet_manager` 가 `/robotN/manual_pair` 호출, 응답의 `reason` 을 화면에 그대로 보여준다. 성공 시 `/fleet/intervention` 에 **`kind = MANUAL_PAIR`** 발행 — IPH 에 들어간다(10.7) · 마감 9/9  **→ 완료 9/8 (PR #47) — `/fleet/manual_pair` 중계까지**
- [x] `core/db.py`, `config/schema.sql`, `fleet_manager_node.py` 기본 흐름, `control_ui_node.py` + `templates/index.html` · 마감 9/8  **→ 9/9 로봇별 `RobotStatus(namespace)`가 AMR 원본 상태를 전담하고 `/fleet/status`를 1 Hz 발행. `fleet_manager` 는 이 요약을 받아 미션 전이·DB만 담당한다(계약 v1.18, PM 승인). HTTP·맵은 `monitor_web_app.py`가 담당**
- [x] **작업자 등록과 AMR 배정 시점 분리** — 등록 시에는 `workers`에 작업자만 저장하고, `RequestMission` 시 가용 `RobotStatus` 상태를 기준으로 AMR을 선택해 `missions.robot_ns`에 저장(계약 v1.19) · 마감 9/10
- [x] `control_ui`: 작업자별/전체 종료 버튼, `fleet_manager` RETURN_ALL · 마감 9/8
- [x] `fleet_manager_node.py`: yaw 계산, 미션 상태 전이 구독 (`shuttle/status` → Mission state) (PR #7) · 마감 9/8
- [x] `fleet_manager_node.py`: 배터리 25 % 미만 미션 거부·사유 기록 (FR-06) (PR #7) · 마감 9/8
- [x] `fleet_manager_node.py`: 리드타임 기록, T_lim 초과 실패 확정, `arrived_late` 별기 (FR-09) — 도착 시점 판정 + 1초 스윕 타이머 (PR #7) · 마감 9/8
- [x] **`FleetStatus.state` 를 approach·shuttle 두 출처로 분리 (I-020)** (PR #12) · 마감 9/7
- [x] **일시적 `FAILED` 가 미션을 굳히지 않게 (I-026)** — approach FAILED 는 미션 전이에서 빼고, FAILED 를 터미널에서 뺀다 (DONE 만 터미널) (PR #18, 9/7) · 마감 9/8
- [x] `fleet_manager_node.py`: `/fleet/intervention` 구독 → DB events 기록 (I-021, FR-28) (PR #23, PM 스텁 18/18) · 마감 9/8
- [x] `fleet_manager_node.py`: `/fleet/zone_event` 구독 → DB `events` kind `ZONE` 기록 (I-030, 계약 v1.11 §7) · 마감 9/8  **→ 완료 (9/9 확인) — `_zone_event_cb`(:275)가 형식·구역·로봇을 검사하고 SQLite ZONE 이벤트로 기록**
- [x] `fleet_manager_node.py`: 작업자-로봇 배정을 worker_id 정렬 순서로 (FR-03 '동시 검출 시 순서 재현') — 1 s 배치 창 + `worker_id` 정렬, W1→robot3·W2→robot8 로 순서 무관 재현 (PR #28, PM 스텁 24/24) · 마감 9/8
- [x] `control_ui`: 페어링 폴백(명시 페어링) 버튼 (FR-13) · 마감 9/8  **→ 완료 9/8 (PR #47)**
- [x] `fleet_manager_node.py`: 주석의 옛 구역 토큰 이름 정리 (I-019) (PR #7) · 마감 9/9

## mro_bringup [PM 김병직]
- [ ] `v_max` 0.3 m/s 와 Nav2 `max_vel_x` 0.26 충돌 정리 — 계약이 0.3 이라고 적었으니 Nav2 쪽을 맞춘다 (PR #39 TODO ②) · 마감 9/8
- [x] **셔틀 파라미터 7개를 `robotN.yaml` 로, `arena_yaml` 경로를 `robot.launch.py` 로** (PR #36 주행 요청, 계약 5절 단일 출처) — ~~`bay_advance_m`~~(PR #52 로 삭제 — 마커 기준 접근이 대신한다)·`aruco_max_dist_m`·`aruco_valid_s`·`aruco_confirm_s`·`aruco_retry`·`token_retry_s`·`nav2_poll_s`. `arena.yaml` 은 데이터 yaml 이라 파라미터 파일로 못 넘겨 경로만 준다. 노드의 런타임 조회 대체값은 단독 실행용으로 그대로 둔다 · 마감 9/8
- [x] robot / webcam_pc / fleet_pc launch, params 골격 · 마감 9/7
- [x] 설계도 4장 — `docs/diagrams/` (STEP 1·2·3-1·3-2) · 마감 9/8
- [x] launch 가 `webcam.yaml` 을 넘기도록 연결 — webcam1/webcam2 launch (9/7) · 마감 9/7
- [x] webcam1 launch 에서 안 쓰는 `homography_json` 제거 (계약 v1.8, PR #26 노트) · 마감 9/7
- [x] 계약 v1.11 — 세마포어 이벤트 토픽 `/fleet/zone_event`·DB kind ZONE·K-06 산출 규칙 (I-030 PM 결정) · 마감 9/7
- [x] robot.launch 가 `perception.yaml` 을 넘기도록 연결 — 9/7, `aruco_detector` 가 PR #30 부터 파라미터를 읽어서 앞당김 (amr_detection·aruco 두 노드에만, robotN.yaml 뒤) · 마감 9/8
- [ ] 노드들이 params 를 실제로 읽도록 전환한 뒤 값 확정 · 마감 9/8 — **주행·인지·관제는 완료 (9/9).** `common.yaml` 51개·`perception.yaml` 27개·`fleet.yaml` 4개가 전부 소비되고, `follow_target` 의 코드 전용 값 9개도 `common.yaml` 로 옮겼다(PR #56 값 그대로). 남은 것 둘: ① `v_max` — 읽는 노드가 없다(아래 Nav2 항목과 같은 건) ② **웹캠 16개** `webcam2.py` 11 · `webcam_hint_node.py` 5 가 아직 코드에만 있다 [웹캠 한세교·김학민]. `db_path`·`out_dir`·`arena_yaml`·`map_yaml_path` 는 런치가 주는 경로라 yaml 대상이 아니다
- [x] ~~`reid_threshold`(perception.yaml ↔ robotN.yaml)~~ 해결 9/8 — robotN.yaml 에서 빼고 perception.yaml 단독 (계약 5절)
- [x] ~~`navigation.yaml` ↔ robotN.yaml~~ **해결 9/9 — 파일을 지웠다.** 어떤 launch 도 안 넘기는 죽은 파일이었는데, `lost_research_s` 8.0·`search_rotate_deg` 30.0 이 남아 있었다. 이건 **PR #56 이전 값**이라 (코드는 `research_s` 2.0·`search_step_deg` 20.0) 이름만 다른 같은 개념이었다. **합치는 방향으로 정리했다면 재탐지가 3.5 s → 11 s 로 되돌아갔다** · 마감 9/8
- [x] ~~`t_lim_s`·`t_ret_s`·`dock_retry`(arena.yaml ↔ common.yaml)~~ **해결 9/9.** arena.yaml 하단 4개는 어떤 코드도 안 읽어(이 파일에서 읽히는 것은 `bays`·`waiting_zone`·`dock_zone` 뿐) 삭제했다. `t_lim_s` 는 **PM 결정으로 common 통일** — 로봇(`shuttle_mission`)과 관제(`fleet_manager`)가 같은 값을 봐야 미션 실패 시점이 안 어긋나므로 fleet.yaml 에서 빼고 `fleet_pc.launch.py` 가 common.yaml 을 함께 넘기게 했다. 양쪽 120.0 확인 · 마감 9/8
- [ ] SDD 부록 A rqt_graph 캡처 첨부 · 마감 9/10
