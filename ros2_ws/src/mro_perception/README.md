# mro_perception — 사용법

담당 인지 정정인·민범진. 노드 2개(`amr_detection`, `aruco_detector`)와 순수 계산 모듈 `core/`, 모델 `models/`.
이 문서는 **켜는 법·보는 법·재는 법**만 다룬다. 설계·원리는 각 파일 머리의 docstring, 모델은 `models/README.md`.

## 1. 켜기

**정상 운용** — 로봇 제어 PC 에서 `docs/setup.md` 의 T1~T4 순서대로. T4 가 이 패키지 노드를 띄운다.

```bash
ros2 launch mro_bringup robot.launch.py namespace:=/robot3      # 두 번째 로봇은 /robot8 (9/8 robot4 → robot8 교체)
```

launch 가 붙여주는 것: `namespace`, `/tf → tf` 리맵, `params/robotN.yaml`, 그 뒤에 `config/perception.yaml` (9/7 연결됨).
같은 키가 둘 다에 있으면 뒤에 오는 `perception.yaml` 이 이긴다. launch 로 띄울 때는 `-p` 를 못 주니 값은 이 파일에서 바꾼다.

**단독 시험** (웹캠·관제·Nav2 없이 카메라만) — TS-U03·U04·U05 는 이걸로 한다.

```bash
ros2 run mro_perception amr_detection_node --ros-args -r __ns:=/robot3 -r /tf:=tf -r /tf_static:=tf_static
```

- **미리보기 창은 기본으로 꺼져 있다** (I-043). 모니터 앞이라 보고 싶으면 `-p show_window:=true` 를 붙인다.
  화면 없는 PC(SSH)에서 켜면 창을 못 만들어 노드가 죽으므로 기본을 끔으로 둔 것이다. 창 없이 보려면 2절의
  `rqt_image_view` 를 쓴다 — 어느 PC 에서든 되고 SSH 에서도 된다
- 서술자·미션이 없으면 노드는 "확신도 1등 작업자" 를 대상으로 잡고 `paired=False` 를 낸다 — 단독 시험용 동작
- TF(localization)가 없어도 `target/visible`·`target/distance`·`target/image` 는 나간다. `target/map_point` 만 안 나간다

## 2. 보기

```bash
ros2 topic echo /robot3/target/visible      # 매 프레임 True/False
ros2 topic echo /robot3/target/distance     # 대상까지 직선거리 m — 거리 품질 게이트 통과분만
ros2 topic echo /robot3/target/paired       # 재식별 게이트 통과 여부 (바뀔 때 + 1 Hz)
ros2 topic echo /robot3/target/worker_id    # 짝이 된 W1/W2. 미확정이면 ''
ros2 topic echo /robot3/target/pair_state   # PAIRED / GATE_FAIL(보이는데 30 s 못 넘음 → 관제 버튼) / NO_TARGET
# 명시 페어링(FR-13)을 손으로 흉내: 관제 UI 대신 서비스를 직접 부른다. 수락이면 accepted=true, 거부면 reason 에 사유
ros2 service call /robot3/manual_pair mro_interfaces/srv/ManualPair "{robot_ns: '/robot3', worker_id: 'W1'}"
ros2 topic pub -1 /robot3/mission/finish std_msgs/msg/Bool "{data: true}"   # 페어링(자동·수동) 해제
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
| `reid_threshold` | 0.34 | 색 지문 유사도 임계 (Q-13) | **9/10 실측 확정** — 근거·한계는 아래 5절 |
| `descriptor_frames` | 10 | 지문 평균 창 | 웹캠1 과 같게 유지 |
| `pair_on_s` / `pair_off_s` | 1.0 / 3.0 | paired 켜짐/꺼짐 연속 시간 | 깜빡이면 on 을, 가림에 풀리면 off 를 올림 |
| `publish_hz` | 1.0 | paired·worker_id 재발행 | — |
| `max_core_gap_mm` | 150 | 거리 품질 게이트 | 거리가 자꾸 `rejected` 면 200 |
| `bbox_inner` | 0.5 | 거리 측정 bbox 중심부 비율 | — |
| `sync_slop_s` | 0.1 | RGB·depth 시각 차 허용 | 동기화 쌍이 안 나오면 0.2 |
| `camera_fx_scale` | 1.7778 | `camera_info` 초점거리 보정 (I-031). `map_point` 좌우에 영향 | 로봇이 맞는 `camera_info` 를 내면 1.0 |
| `show_window` | **false** | 미리보기 창 (기동 때만) | 모니터 앞이면 true. SSH 면 그대로 둔다 (I-043) |
| `jpeg_quality` | 80 | `target/image/compressed` 품질 | 웹캠 노드와 같은 값 유지 |
| `diag_log_s` | 1.0 | `[diag]` 계측 로그 주기 | 0 이면 끔. 렉 의심될 때 1 |
| `tf_wait_s` | 0.0 | `to_map` 이 TF 를 기다리는 시간 | **항상 0.** 1.0 은 9/8 렉 재현용 (7절) |
| `gate_fail_s` | 30.0 | 보이는데 게이트 미달이 이 시간 이어지면 `pair_state=GATE_FAIL` (FR-13) | 계약 10.1 의 30 s. 시연 때만 5 |
| `manual_pair_max_m` | 1.0 | 명시 페어링을 받는 최대 거리 (FR-14) | — |
| `target_stale_s` | 1.0 | 마지막으로 본 후보·거리를 "지금" 으로 치는 시간 | **렉으로 프레임이 늦으면 2~3.** 안 늘리면 폴백이 닫힌다 (I-045) |

실행 중 바꾸기: `ros2 param set /robot3/amr_detection detect_conf 0.6` — 다음 프레임부터 먹는다.
범위 밖이면 **거절**되고 (`Set parameter failed`) 노드는 옛 값으로 계속 돈다. 파라미터 서버와 노드가 다른 값을
갖는 일이 없게 한 것이다. `sync_slop_s`·`show_window`·`publish_hz`·`descriptor_frames`·`camera_fx_scale` 은
값이 기동 때 구조(동기화기·창·타이머·버퍼·보정 K)를 만들어서 **재시작해야 한다** — 바꾸려 하면 그 이유로 거절된다.

## 4. 테스트

```bash
cd ros2_ws/src/mro_perception
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_reid.py -q     # 11개. 게이트 논리, ROS 불필요
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_manual_pair.py -q   # 18개. FR-13 pair_state·수동 요청 판정
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

**확정 결과 (9/10) — `reid_threshold` = 0.34**

게이트가 실제로 하는 비교는 **웹캠1 등록 서술자 ↔ 로봇이 본 crop** 이다 (`core/reid.py` 의 `update()`: 복장 클래스로 먼저
거르고, 통과한 후보의 10프레임 평균을 등록 서술자와 비교). 그래서 **로봇 crop 끼리 재면 안 된다** — 시점이 다른 두 카메라를
건너는 비교라 값이 완전히 달라진다. 실제로 로봇끼리 재면 두 분포가 0.135/0.546 으로 시원하게 갈라지지만, 시점을 건너면
아래처럼 좁혀진다.

실측 — 웹캠1 실기동 화면(9/10)에서 등록 서술자를, AMR Detection 화면녹화(9/8)에서 로봇 관측을 뽑아 양쪽 다 10프레임 평균:

| | n | min | p5 | p50 | max |
|---|---|---|---|---|---|
| 같은 인형 (웹캠1 b ↔ 로봇 b) | 1080 | **0.348** | 0.360 | 0.401 | 0.470 |
| 다른 인형 (웹캠1 a ↔ 로봇 b) | 600 | 0.248 | 0.273 | 0.319 | **0.361** |

| 임계 | 페어링 실패 | 영영 실패하는 등록 | (오분류 시) 오붙음 |
|---|---|---|---|
| 0.30 | 0.0 % | 0/36 | 81.3 % |
| **0.34** | **0.0 %** | **0/36** | 17.8 % |
| 0.40 | 48.6 % | 1/36 | 0.0 % |
| 0.60 (옛 값) | **100 %** | **36/36** | 0.0 % |

**옛 값 0.6 은 같은 인형 최댓값 0.470 보다 높아 한 번도 넘을 수 없었다.** 자동 페어링(FR-12)이 구조적으로 안 걸리고
`gate_fail_s` 30 초 뒤 `GATE_FAIL` 로 떨어져 매번 관제 버튼(FR-13)을 눌러야 하는 상태였다.

두 분포가 **0.348 / 0.361 로 겹친다.** 계약 2절의 "다른 인형 분포 최댓값 위" 를 문자 그대로는 만족시킬 수 없다는 뜻이고,
원인도 계약이 이미 적어 둔 그대로다 — 서술자가 인형 전체를 쓰는데 하의·머리가 두 인형에서 동일하다. 겹치는 구간에서는
**놓침 0 % 를 우선**했다. `update()` 가 복장 클래스를 먼저 거르므로 다른 인형 쌍은 YOLO 오분류가 있어야만 유사도까지 온다.

**근거의 한계 (남은 확인 절차)**
- 양쪽 다 화면녹화(압축·리스케일)본이고 **웹캠1 은 9/10 · 로봇은 9/8 로 날짜와 조명이 다르다.** 절댓값이 원본보다 눌렸을 수 있다
- **로봇 시점 worker_a 가 0 장** — 같은 인형 분포는 worker_b 로만 쟀다
- **TS-U04 측면·후면이 빠졌다.** 같은 인형 유사도가 가장 떨어지는 구간이라 0.348 이 낙관적일 수 있다
- → 위 세 가지는 TS-U04 를 찍을 때 함께 확인한다. 그때 `bag → 데이터셋` 변환은 인형별로 따로 녹화해 bag 이름을 정답 라벨로 쓴다

## 6. 알아둘 것

- 카메라 구독은 **최신 한 장만**(BEST_EFFORT·depth 1). 처리가 밀려도 묵은 프레임을 재생하지 않고 건너뛴다 —
  `[diag]` 의 `age` 가 항상 한 프레임 간격 근처여야 정상. 커지면 입력 쪽(RPi·Wi-Fi) 문제다 (9/8 A-1).
  동기화 큐(`SYNC_QUEUE` 20)는 rgb·depth **도착 시차** 허용치라 줄이면 안 된다 — 3 으로 뒀다가 짝이 안 맞아 처리 0 이 났다
- 검출은 시작 신호 없이 항상 켜진다 (계약 v1.10). 셔틀 중 작업자를 스치면 `visible` 이 토글되고, `kpi_report.py` 의 K-04 가
  FOLLOWING 구간으로 안 걸러 Lost 로 셀 수 있다 — 웹캠팀 이슈
- `follow_target` 이 `target/paired` 를 보고 추종을 시작하는 연동은 주행 TODO ④. 그 전까지 `paired` 는 발행만 된다
- `/webcam1/worker_descriptor` 는 절대 이름 — 네임스페이스 리맵에 걸리지 않는다. "내 담당" 은 `mission/assigned` 의 `worker_id` 로 안다
- **명시 페어링 폴백 (FR-13, 계약 10절)** — 작업자가 보이는데 게이트를 `gate_fail_s`(30 s) 넘게 못 넘으면 `target/pair_state=GATE_FAIL`
  을 내고, 관제 UI 버튼 → fleet_manager → `/robotN/manual_pair` 요청을 이 노드가 받아 `core/reid.manual_pair_decision` 으로
  판정한다 (남의 로봇·이미 짝·안 보임·1 m 밖·그새 스스로 통과 → 거부 + 사유). 수락하면 `paired=True`·`worker_id` 를 여기서 내고
  자동 게이트가 뒤집지 못하게 잠근다. `mission/finish` 가 오면 잠금과 게이트를 함께 푼다. **30 s 를 채우기 전 구간의 값은 계약에
  없어 `NO_TARGET` 을 유지한다 — PM 확인 항목.** 테스트는 `test/test_manual_pair.py` 18개

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
| `proc avg` ≫ 100 ms | **계산이 느림** — GPU 미사용, 또는 SSH -X 로 창 띄움 | `nvidia-smi` 확인, 창을 켰다면 끄고 다시 |
| `cb rgb` ≪ `topic hz` 입력 | **콜백이 굶음** — 노드가 메시지를 못 꺼냄 | `tf` 콜백 수가 높으면 TF 리스너를 별도 스레드로(`spin_thread=True`) |
| `age` 큰데 `topic delay` 도 큼 | **입력이 늦게 옴** — RPi·Wi-Fi | 로봇 Wi-Fi, RPi 부하. 단 `/scan` 지연도 같은 값이면 **시계 어긋남**(지연 아님) |
| `proc max` 가 **1000·2000 ms** | **TF 대기 블로킹** — `to_map` 이 실행기를 세움 | 9/8 수정(`timeout=0`)이 든 빌드인지. 탐지될 때만 렉이 나는 게 지문 |
| `cb rgb/depth` 는 오는데 `synced 0` | **동기화 실패** — depth 가 rgb 보다 늦게 와 큐를 넘음 | 도구 표의 depth 지연 ≫ rgb 지연이면 depth 스트림 과부하. 아래 줄 |
| depth 지연만 크다, 도구가 `구독자 2 이상` | **중복 노드** — 탐지 노드가 여러 개 떠서 RPi 가 스트림을 N 번 전송 | `pkill -f amr_detection_node` 후 하나만. GPU 에 python 이 여럿이면 의심 |
| 노드 `age` 는 작은데 rqt 만 늦음 | **출력 대역폭** — JPEG 로 ≈ 6 MB/s (raw 였으면 27, 9/8 압축으로 전환) | rqt 를 다른 PC 에서 보면 그래도 Wi-Fi 부담. `bw` 확인, rqt 끄고 `age` 재확인 |

한 번에 하나씩 끄면서 본다: ① rqt 끄기 → ② 미리보기 창 끄기(기본이 꺼짐) → ③ 그래도 `age` 크면 `cb` 와 `topic hz` 비교.

### 렉 재현 (전/후 비교·시연용)

두 원인 모두 코드 수정 없이 스위치로 재현된다. 노드는 인형이 **보이는** 상태여야 한다(탐지가 없으면 `to_map` 이 안 불린다).

| 원인 | 재현 | 되돌리기 | `[diag]` 에서 보이는 것 |
|---|---|---|---|
| ① TF 대기 블로킹 | 노드 실행에 `-p tf_wait_s:=1.0` (localization 없이) | 플래그 빼기 (기본 0) | `proc max` ≈ 2000, `processed` ≈ 3/s, `loop` 2/s ↔ 180/s 요동 |
| ② RELIABLE 구독 | PC 다른 터미널에서 `ros2 topic echo --qos-reliability reliable --qos-depth 10 --no-arr /robot3/oakd/rgb/image_raw/compressed` | 그 터미널 Ctrl+C | `age` 가 수백 ms → 1~2 s 로 올라감. `proc` 는 그대로 |

②는 옛 노드가 하던 구독을 흉내 낸 것이다 — 손실 재전송이 송신 큐를 채워 best-effort 독자까지 밀린다. 링크 상태에 따라 수십 초 걸릴 수 있다.
