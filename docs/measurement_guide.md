# 실측 절차 — arena.yaml 좌표 입력 + I-004/I-005/I-006

담당: 주행 [박병후, 이시율]. 대상 맵: `mro_bringup/maps/auto_map.yaml`(9/5 재작성본, 커밋 `b1fb6f1`).
값을 채운 뒤에는 이 문서가 아니라 실제 대상 파일(`arena.yaml`, `robot3.yaml`, `robot8.yaml`)에 반영하고,
`9/8` 로봇 2호기가 robot4 → **robot8** 로 교체됐다. 아래 `robot8` 항목의 좌표는 같은 도크·같은 자리라
구 robot4 실측값을 **그대로 쓴다 (9/9 PM 판단)** — 교체한 것은 기체뿐이고 도크는 옮기지 않았으므로
기체 개체차는 도킹 후 amcl_pose 편차 20 cm 에 묻힌다 (I-013). robot8 이 기동부터 측위가 어긋나면
그때 이 값을 첫 번째로 의심한다.
`docs/issues.md`·`docs/todo.md` 상태도 같이 갱신한다.

## 0. 공통 준비

1. 맵을 새 `auto_map.yaml` 로 띄운다:
   ```bash
   ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot3 \
     map:=$(ros2 pkg prefix mro_bringup)/share/mro_bringup/maps/auto_map.yaml
   ```
2. RViz에서 **2D Pose Estimate 는 클릭 금지**(`docs/setup.md` 주의사항 — 도크 기준 초기자세는 노드가 잡는다).
3. 좌표는 `Publish Point` 툴로 찍고, 터미널에서 값을 읽는다:
   ```bash
   ros2 topic echo /clicked_point
   ```
   찍을 때마다 `point.x`, `point.y` (map 프레임 기준, m 단위)가 출력된다. `z`는 무시.
4. **yaw 가 필요한 지점**(waypoint, dock pose, approach)은 점을 2개 찍어 방향을 구한다:
   - 1점: 로봇이 설 위치, 2점: 로봇이 바라볼 방향의 앞쪽 점
   - `yaw = atan2(y2 - y1, x2 - x1)` (라디안). 파이썬으로 바로 계산:
     ```bash
     python3 -c "import math; print(math.atan2(y2-y1, x2-x1))"
     ```
5. 값을 찍을 때마다 아래 표에 바로 옮겨 적고, 두 번째 사람이 교차 확인한다(오타 방지 — 좌표 하나 틀리면 세마포어·waypoint가 전부 어긋난다, I-013 참고).

## 1. arena.yaml 좌표 입력 체크리스트

| 필드 | 찍을 점 | 개수 | 완료 |
|---|---|---|---|
| `bays.A.waypoint` | 베이 A 정차 위치 + 진행방향 앞점 | 2 | [ ] |
| `bays.B.waypoint` | 베이 B 정차 위치 + 진행방향 앞점 | 2 | [ ] |
| `bays.C.waypoint` | 베이 C 정차 위치 + 진행방향 앞점 | 2 | [ ] |
| `waiting_zone.robot3` | Z-05 합류지점, robot3 정차 위치(+앞점) | 2 | [ ] |
| `waiting_zone.robot8` | Z-05 합류지점, robot8 정차 위치(+앞점, robot3와 살짝 오프셋) | 2 | [ ] |
| `dock_zone.pose.robot3` | robot3 도킹 완료 상태 pose(+앞점) — **도크 실물에 로봇을 실제로 도킹시킨 뒤 amcl pose 읽는 것을 권장** | 2 | [ ] |
| `dock_zone.pose.robot8` | robot8 도킹 완료 상태 pose(+앞점) | 2 | [ ] |
| `dock_zone.approach.robot3` | robot3 도크 정면 접근 지점(도크 앞 ~0.5m, dock() 호출 지점) | 2 | [ ] |
| `dock_zone.approach.robot8` | robot8 도크 정면 접근 지점 | 2 | [ ] |
| `dock_zone.holding` | 후행 로봇 대기 위치 — 도크에서 ≥1.5m, 선행 로봇 도킹 경로 밖 | 2 | [ ] |
| `semaphore_zones.P1.polygon` | 도크↔창고 병목(0.79m) 구간을 감싸는 사각형 4꼭짓점 | 4 | [ ] |
| `semaphore_zones.DOCK.polygon` | 도크 구역 전체를 감싸는 사각형 4꼭짓점 | 4 | [ ] |
| `homography_marks.cam1` | 십자 테이프 기준점들 (`docs/interfaces.md` 형식대로 `[u_desc, x, y]`) | 웹캠팀 협의 | [ ] |
| `homography_marks.cam2` | 위와 동일 | 웹캠팀 협의 | [ ] |

`dock_zone.pose` 를 넣은 뒤에는 **같은 값을 `robot3.yaml`/`robot8.yaml` 의 `dock_pose` 에도 반영**한다 (I-013 — 두 값이 다르면 map 원점이 어긋난다).
`9/8` `robotN.yaml` 은 **로봇마다 다른 값만** 담는다 (`robot_ns`·`dock_pose`·`depth_k`). 공통값은 `common.yaml` 이므로 그쪽은 건드리지 않는다.
`semaphore_zones` 의 ID(`P1`, `DOCK`)는 `zone_semaphore_node.py` 의 `ZONES` 상수와 철자까지 동일해야 한다.

## 2. I-004 — 벽 받침대 vs 통로 실폭

| 통로 | 벽면 기준 실측 | 받침대 돌출폭 | 받침대 포함 실통과폭 | 확인 |
|---|---|---|---|---|
| P1 (로봇 구역 ↔ 중앙 복도, 병목) | 0.79 m | **없음** | **0.79 m** | [x] |
| P2 (관제탑→대기공간) | **0.86 m** (9/6 재실측, 9/5 의 0.88 정정) | **양쪽 각 3 cm** | **0.80 m** | [x] |
| P3 (중앙복도→작업공간) | 0.93 m | **입구 5 cm · 안쪽 6.5 cm** | **0.865 m** (6.5 기준) | [x] |

실측: 주행 [박병후, 이시율] (9/6). 받침대가 통로 바깥으로 돌아가 있으면 "돌출폭 0"으로 기록한다.

**받침대는 LiDAR 스캔 높이 18.5 cm 보다 낮아 지도·costmap 에 잡히지 않는다** — Nav2 는 통로가 벽면 기준 폭인 줄 알고 벽에 붙어 계획한다.
덮는 방법은 `inflation_radius` 를 올리는 것뿐이고, 로봇 표면과 벽면 사이 여유는 `r − 0.175`(footprint 내접 반지름) 이다.
최대 돌출 6.5 cm 를 덮으려면 **r ≥ 0.24**, 여유를 두면 0.25. **가장 좁은 P1 에 받침대가 없어** r 을 올려도 병목 통과에는 부담이 없다 (r=0.25 에서 P1 하드 자유폭 0.44 m). 결정과 적용은 I-009.

## 3. I-005 — 벽 높이 / OAK-D 높이 / 작업자 전고

| 항목 | 값 | 확인 |
|---|---|---|
| 벽 상단 높이 | **41 cm** (9/6 실측, 주행) | [x] |
| OAK-D 렌즈 높이(로봇 장착 기준) | **23.5 cm** (9/6 실측) | [x] |
| LiDAR 스캔 높이 | **18.5 cm** (9/6 실측) | [x] |
| 작업자 전고 worker_a (RC카+인형, 바닥 기준) | **26.5 cm** (9/6 실측) | [x] |
| 작업자 전고 worker_b (RC카+인형, 바닥 기준) | **28.5 cm** (9/6 실측) | [x] |

실측: 인지 [정정인, 민범진] (9/6). 인형은 RC카에 얹어 움직이므로 **작업자 높이는 RC카 포함 바닥 기준 전고**로 잰다.

성립 조건: 가림원이 **카메라(OAK-D 렌즈)보다 높아야** Z-06·Z-09 에서 가림(Target Lost)이 발생한다 (E-1·TS-05 전제).
I-007 로 가림원이 벽 → **교차하는 타 작업자(RC카+인형)** 로 바뀌었으므로 조건은 `작업자 전고 > OAK-D 렌즈 > LiDAR` 이고,
**`26.5 / 28.5 > 23.5 > 18.5` 로 충족된다 (9/6 확인).**

다만 렌즈 대비 여유가 worker_a **+3.0 cm** · worker_b **+5.0 cm** 뿐이라 높이만으로 가림이 보장되지는 않는다.
가리는 작업자가 **대상 작업자보다 로봇에 더 가까이** 지나가야 시야각을 크게 먹어 대상 bbox 를 덮는다.
같은 거리에서 스쳐 가면 대상 상단이 남아 검출이 끊기지 않고, Lost 1.0 s 가 차지 않아 E-1 이 재현되지 않는다.
→ E-1 시험은 여유가 큰 **worker_b 를 가림원**으로 쓴다.

## 4. I-006 — 도크 실물 배치 확인

- [ ] 도크 2대가 사진/실물로 확인됨 (도킹점 기준 0.84 m 확정 9/6, SOT 확인)
- [x] `dock_zone.divider` — **두지 않기로 (9/6)**. TS-U12 로봇별 10회, 차폐 없이 오도킹 0 → `divider: false`
- [ ] robot3/robot8 각각의 접근 지점(`dock_zone.approach`)에서 도킹 호출 시 옆 도크로 안 가는지 1회 시험
- 이후 **TS-U12** (로봇별 10회 도킹, 오도킹 0) 로 넘어간다

## 5. 값 입력 후

1. `params/arena.yaml`, `params/robot3.yaml`, `params/robot8.yaml` 수정 (로봇을 새로 들일 때는 `params/robotN.yaml` 생성 + `params/fleet.yaml` 의 `robot_namespaces` 에 추가)
2. `docs/issues.md` — I-001(진행중→해결), I-004/I-005/I-006(열림→진행중 또는 해결), I-013(진행중→해결 후보) 갱신
3. `docs/todo.md` — 해당 체크박스 갱신
4. 웹캠팀에 알림 — `homography_marks` 를 채우면 웹캠1/2 재캘리브레이션(I-002)의 전제가 만족된다
5. 커밋 전 `git pull` 로 다른 세션 변경분과 충돌 없는지 확인 후 커밋
