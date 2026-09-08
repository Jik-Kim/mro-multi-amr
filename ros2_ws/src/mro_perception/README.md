# mro_perception — 사용법

담당 인지 정정인·민범진. 노드 2개(`amr_detection`, `aruco_detector`)와 순수 계산 모듈 `core/`, 모델 `models/`.
이 문서는 **켜는 법·보는 법·재는 법**만 다룬다. 설계·원리는 각 파일 머리의 docstring, 모델은 `models/README.md`.

## 1. 켜기

**정상 운용** — 로봇 제어 PC 에서 `docs/setup.md` 의 T1~T4 순서대로. T4 가 이 패키지 노드를 띄운다.

```bash
ros2 launch mro_bringup robot.launch.py namespace:=/robot3      # robot4 는 /robot4
```

launch 가 붙여주는 것: `namespace`, `/tf → tf` 리맵, `params/robot3.yaml`. **`config/perception.yaml` 은 아직 안 넘긴다**
(PM 9/8 항목) — 그 전까지 아래 파라미터는 노드 기본값이 쓰이고, 바꾸려면 `robot3.yaml` 에 키를 넣거나 `-p` 로 준다.

**단독 시험** (웹캠·관제·Nav2 없이 카메라만) — TS-U03·U04·U05 는 이걸로 한다.

```bash
ros2 run mro_perception amr_detection_node --ros-args -r __ns:=/robot3 -r /tf:=tf -r /tf_static:=tf_static \
    -p show_window:=false
```

- **SSH 로 띄우면 `show_window:=false` 필수.** 창을 못 만들면 노드가 죽는다. 모니터가 있으면 빼도 된다
- 서술자·미션이 없으면 노드는 "확신도 1등 작업자" 를 대상으로 잡고 `paired=False` 를 낸다 — 단독 시험용 동작
- TF(localization)가 없어도 `target/visible`·`target/distance`·`target/image` 는 나간다. `target/map_point` 만 안 나간다

## 2. 보기

```bash
ros2 topic echo /robot3/target/visible      # 매 프레임 True/False
ros2 topic echo /robot3/target/distance     # 대상까지 직선거리 m — 거리 품질 게이트 통과분만
ros2 topic echo /robot3/target/paired       # 재식별 게이트 통과 여부 (바뀔 때 + 1 Hz)
ros2 topic echo /robot3/target/worker_id    # 짝이 된 W1/W2. 미확정이면 ''
ros2 run rqt_image_view rqt_image_view /robot3/target/image/compressed   # 오버레이 JPEG (창 대신 이걸로 본다)
```

오버레이 읽는 법 — 좌 RGB, 우 depth 컬러맵, 위 검은 띠가 상태.

| 띠 문구 | 뜻 |
|---|---|
| `waiting for camera_info` | K 를 아직 못 받음. `oakd/stereo/camera_info` 확인 |
| `detecting - no target (no descriptor)` | 작업자 안 보임. 괄호는 게이트 사유 |
| `worker_a 1.21 m (no TF) sim=0.00 unpaired` | 검출·거리 OK, map 변환만 못 함 (단독 시험이면 정상) |
| `worker_a 1.21 m -> map (2.10, 0.80) sim=0.83 PAIRED` | 전부 정상 |
| `core gap 210 mm > 150 - rejected` | 거리 품질 게이트 탈락 (가림·경계 걸침). 이 프레임은 distance·map_point 안 나감 |

굵은 초록 네모가 대상, 얇은 회색 네모가 그 밖의 작업자 후보. 로그에도 매 프레임 `worker_a conf=0.87 sim=0.83 PAIRED | n=.. p20=.. med=..` 가 찍힌다.

## 3. 파라미터 (`config/perception.yaml` · `params/robotN.yaml`)

| 키 | 기본 | 뜻 | 언제 만지나 |
|---|---|---|---|
| `detect_conf` | 0.5 | YOLO 확신도 하한 | 오탐이 보이면 0.6 (오탐=오추종, AC-04) |
| `reid_threshold` | 0.6 | 색 지문 유사도 임계 (Q-13) | **TS-U04 분포로 확정** — 아래 5절 |
| `descriptor_frames` | 10 | 지문 평균 창 | 웹캠1 과 같게 유지 |
| `pair_on_s` / `pair_off_s` | 1.0 / 3.0 | paired 켜짐/꺼짐 연속 시간 | 깜빡이면 on 을, 가림에 풀리면 off 를 올림 |
| `publish_hz` | 1.0 | paired·worker_id 재발행 | — |
| `max_core_gap_mm` | 150 | 거리 품질 게이트 | 거리가 자꾸 `rejected` 면 200 |
| `bbox_inner` | 0.5 | 거리 측정 bbox 중심부 비율 | — |
| `sync_slop_s` | 0.1 | RGB·depth 시각 차 허용 | 동기화 쌍이 안 나오면 0.2 |
| `show_window` | true | 미리보기 창 | SSH 면 false |

실행 중 바꾸기: `ros2 param set /robot3/amr_detection detect_conf 0.6` (다음 프레임부터는 안 먹는 값도 있다 — `sync_slop_s`·`show_window` 는 재시작).

## 4. 테스트

```bash
cd ros2_ws/src/mro_perception
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_reid.py -q     # 11개. 게이트 논리, ROS 불필요
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 은 설치된 pytest 와 `launch_testing` 플러그인이 안 맞아서 필요하다 (환경 문제). `colcon test` 는 그대로 된다.

## 5. 시험 절차 (9/8)

**TS-U03 검출** — 인형 하나를 OAK-D 앞 1 m 정면. 띠에 `worker_a`/`worker_b` 가 옷대로 찍히고 `visible` 이 True 로 유지되면 통과.
상대 로봇을 화면에 넣어도 초록 네모가 안 붙어야 한다(`amr` 은 후보 제외).

**TS-U05 줄자 대조** (FR-34) — `target/distance` 를 0.8 · 1.0 · 1.5 · 2.0 m 에서 줄자와 비교.
- 줄자는 **렌즈에서 인형 몸통 중심까지 곧장** — `distance` 가 직선거리라 인형이 화면 어디 있어도 된다
- 각 지점 30 초 `ros2 topic echo` 로 받아 중앙값을 쓴다. 차이 ≤ 7 cm(기준 0.2 m 의 1/3) 면 통과
- 결과를 **주행·웹캠에 공유** — I-029(Nav2 도착 오차)·I-002(호모그래피 오차)에서 측정 오차를 분리하는 근거가 된다

**TS-U04 측면·후면** (AC-10 근거) — 인형을 1 m 에서 0° / 90° / 180° / 270° 로 돌리며 각 10 초. 띠의 옷 이름이 네 방향 모두 유지돼야 한다.
이때 **프레임을 저장**한다 — Q-13 에 쓴다.

```bash
ros2 bag record -o tsu04_worker_a /robot3/oakd/rgb/image_raw/compressed     # worker_b 도 따로
```

**Q-13 임계값** — 저장한 프레임을 YOLO 형식(`images/`·`labels/*.txt`, `data.yaml`)으로 만든 뒤

```bash
python3 tools/reid_threshold.py --dataset <그 폴더> --splits train --current 0.6
```

같은 인형 쌍 / 다른 인형 쌍 유사도 분포가 나온다. **다른 인형 분포의 최댓값 위, 같은 인형 분포의 대부분 아래**에 선을 긋고
`reid_threshold` 에 넣는다. 웹캠1 의 `rematch_threshold`(0.45, 웹캠 시점)와는 **다른 결정**이다 — AMR 은 낮고 가깝고 측면·후면을 본다.

## 6. 알아둘 것

- 카메라 구독은 **최신 한 장만**(BEST_EFFORT·depth 1, 동기화 큐 3). 처리가 밀려도 묵은 프레임을 재생하지 않고 건너뛴다 —
  `[diag]` 의 `age` 가 항상 한 프레임 간격 근처여야 정상. 커지면 입력 쪽(RPi·Wi-Fi·시계) 문제다 (9/8 A-1)
- 검출은 시작 신호 없이 항상 켜진다 (계약 v1.10). 셔틀 중 작업자를 스치면 `visible` 이 토글되고, `kpi_report.py` 의 K-04 가
  FOLLOWING 구간으로 안 걸러 Lost 로 셀 수 있다 — 웹캠팀 이슈
- `follow_target` 이 `target/paired` 를 보고 추종을 시작하는 연동은 주행 TODO ④. 그 전까지 `paired` 는 발행만 된다
- `/webcam1/worker_descriptor` 는 절대 이름 — 네임스페이스 리맵에 걸리지 않는다. "내 담당" 은 `mission/assigned` 의 `worker_id` 로 안다

## 7. 렉 진단 — 추측 말고 숫자로

`diag_log_s`(기본 1 s) 마다 이런 줄이 찍힌다.

```
[diag 1.0s] loop 48/s | cb rgb 9.2 depth 9.1 tf 25/s | synced 9.0/s | processed 9.0/s | age avg 140 max 210 ms | proc avg 31 max 58 ms
```

| 항목 | 뜻 |
|---|---|
| `loop` | 메인 루프 회전 수. spin_once(콜백 1개) + process() 한 쌍이 1회 |
| `cb rgb/depth/tf` | 실제로 **실행된** 콜백 수. 네트워크로 온 수가 아니다 — 그건 `ros2 topic hz` 로 따로 잰다 |
| `synced` | RGB·depth 가 짝지어진 수 |
| `processed` | YOLO 까지 돈 프레임 수 |
| `age` | 처리한 프레임이 **카메라에서 찍힌 뒤 지금까지** 걸린 시간. 이게 곧 "렉" |
| `proc` | process() 한 번의 시간 (YOLO·거리·시각화 포함) |

**한 번에 재는 도구** — 노드가 도는 PC 에서, 노드는 그대로 둔 채:

```bash
python3 tools/diag_lag.py --ns /robot3 --seconds 10     # 입력·tf·처리·출력 Hz/대역폭/지연 + [diag] 로그 + 판정
```

파일을 쓰지 않고 파라미터를 바꾸지 않는다. 손으로 재려면:

```bash
ros2 topic hz    /robot3/oakd/rgb/image_raw/compressed     # 입력이 몇 Hz 로 도착하나
ros2 topic delay /robot3/oakd/rgb/image_raw/compressed     # 입력이 찍힌 뒤 도착까지 (RPi→Wi-Fi 지연)
ros2 topic hz    /robot3/tf                                # tf 가 초당 몇 개 오나
ros2 topic bw    /robot3/target/image/compressed           # 모니터 영상 대역폭 (JPEG ≈ 6 MB/s. raw 였으면 27)
nvidia-smi                                                 # YOLO 가 GPU 를 쓰나 (python 프로세스가 보여야)
```

읽는 법 — 렉의 자리는 넷 중 하나다.

| 증상 | 자리 | 확인 |
|---|---|---|
| `proc avg` ≫ 100 ms | **계산이 느림** — GPU 미사용, 또는 SSH -X 로 창 띄움 | `nvidia-smi`, `show_window:=false` 로 다시 |
| `cb rgb` ≪ `topic hz` 입력 | **콜백이 굶음** — 노드가 메시지를 못 꺼냄 | `tf` 콜백 수가 높으면 TF 리스너를 별도 스레드로(`spin_thread=True`) |
| `age` 큰데 `topic delay` 도 큼 | **입력이 늦게 옴** — RPi·Wi-Fi | 로봇 Wi-Fi, RPi 부하. 단 `/scan` 지연도 같은 값이면 **시계 어긋남**(지연 아님) |
| `proc max` 가 **1000·2000 ms** | **TF 대기 블로킹** — `to_map` 이 실행기를 세움 | 9/8 수정(`timeout=0`)이 든 빌드인지. 탐지될 때만 렉이 나는 게 지문 |
| 노드 `age` 는 작은데 rqt 만 늦음 | **출력 대역폭** — JPEG 로 ≈ 6 MB/s (raw 였으면 27, 9/8 압축으로 전환) | rqt 를 다른 PC 에서 보면 그래도 Wi-Fi 부담. `bw` 확인, rqt 끄고 `age` 재확인 |

한 번에 하나씩 끄면서 본다: ① rqt 끄기 → ② `show_window:=false` → ③ 그래도 `age` 크면 `cb` 와 `topic hz` 비교.
