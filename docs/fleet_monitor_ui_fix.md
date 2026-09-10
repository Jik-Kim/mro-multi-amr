# 관제 UI 버그 수정 — 맵 마커 앵커 · 상세 보기 탭

`fix/fleet-monitor-ui` 브랜치에서 고친 두 가지. 대상 파일은 `mro_fleet/templates/index.html` 하나이고
파이썬·계약(`mro_interfaces`)은 건드리지 않았다.

소유는 관제(김민준)다 — `docs/responsibilities.md`. 인지 담당이 실행 중 발견해 고친 것이라
병합 전에 관제 리뷰가 필요하다.

---

## 1. 맵의 로봇·도크 표시가 실제 좌표에서 어긋난다

### 증상

로봇이 도크에 정확히 서 있어도 로봇 원과 주황 점선 도크 원이 겹치지 않는다.

### 원인

좌표 계산(`projectWorldPoint`)은 정상이었다. CSS 앵커가 문제였다.

두 표시 모두 `transform: translate(-50%,-50%)` 로 **요소 상자의 중심**을 좌표에 놓는데,

- `.robot-marker` 는 세로 flex — 아이콘 + 이름 + `안전 정지` 뱃지를 **합친 상자**의 중심이 놓인다 → 원이 위로 뜬다
- `.dock-reference` 는 가로 flex — 화살표 + 라벨을 **합친 줄**의 중심이 놓인다 → 화살표가 왼쪽으로 밀린다

밀리는 축이 서로 달라서 두 원은 좌표가 같아도 절대 겹칠 수 없었다.

600×400 스테이지에 같은 좌표를 주고 잰 앵커 오차:

| | 수정 전 | 수정 후 |
|---|---|---|
| 로봇 아이콘 중심 | dx 0, dy **−9 px** | 0, 0 |
| 도크 화살표 중심 | dx **−28.15 px**, dy 0 | 0, 0 |

### 수정

앵커 상자를 아이콘·화살표 크기로 고정하고 라벨은 상자 밖으로 넘치게 뒀다.

- `.robot-marker` → `width:0; height:34px`
- `.dock-reference` → `width:22px; height:0`
- `.marker-icon` · `.dock-arrow` · estop 뱃지 → `flex:none`

`flex:none` 이 없으면 flex 가 아이콘을 세로로 찌그러뜨려 앵커가 다시 틀어진다.

---

## 2. 상세 보기에서 다른 탭을 누르면 기본 정보로 튕긴다

### 증상

`상세 보기 ›` 를 열고 `위치 정보` / `미션 정보` / `이벤트 이력` 탭을 눌러도 1초 안에
`기본 정보` 로 돌아간다. 사실상 나머지 탭을 볼 수 없다.

### 원인

`pollFleetStatus` → `applyStatusPayload` 가 서랍이 열려 있으면 매초 `openRobotDetail()` 을
다시 부르는데, 그 함수 끝이 `selectTab('basic')` 으로 굳어 있었다.

### 수정

선택한 탭을 `uiState.activeTab` 에 두고, **서랍을 새로 열거나 다른 로봇으로 바꿀 때만**
`basic` 으로 초기화한다. 1초 갱신 자체는 그대로 돈다.

---

## 검증

| 항목 | 방법 | 결과 |
|---|---|---|
| 앵커 오차 | 헤드리스 크롬에서 `getBoundingClientRect` 실측 | 위 표대로 0, 0 |
| 맵 캔버스 무영향 | 수정 전후 캔버스 픽셀 지문 비교 | 138×71, 화면 887×456, fnv `ae0816b4` — 동일 |
| 탭 유지 | 스텁 fetch 로 폴링 2회 통과 | 수정 전 `basic` 으로 복귀 / 수정 후 `position` 유지 |
| 실환경 | `fleet_pc.launch.py` 로 실제 기동, `http://127.0.0.1:5000` | 서빙 HTML 에 수정 반영, 도크 원과 로봇 원 포개짐 확인 |

---

## 확인 절차

```bash
cd ~/mro-multi-amr-1/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch mro_bringup fleet_pc.launch.py
```

브라우저에서 `http://127.0.0.1:5000` 을 `Ctrl+Shift+R` 로 연다.

- 맵: 도킹 상태의 로봇 원과 주황 점선 도크 원이 동심원으로 겹치는지
- 상세 보기 → `위치 정보` 탭을 누르고 3초 이상 두기

이 리빌드 이후로는 `build/mro_fleet/templates/index.html` 이 소스로 심링크되어,
HTML 만 고칠 때는 리빌드 없이 브라우저 새로고침으로 반영된다.

---

## 남은 것 (이번 범위 밖)

- **도크 라벨 겹침.** 라벨이 원 바깥으로 나가면서 도크가 가까우면(robot3·robot8 은 약 8 cm) 글자가
  겹쳐 보인다. 좌표는 정확하다. 라벨을 아래로 내리거나 도크는 툴팁만 남기는 선택지가 있다.
- **pose 미수신과 원점을 구분하지 못한다.** `robot_control_node` 는 `x`/`y`/`yaw` 를 `0.0` 으로 초기화하고
  `{ns}/amcl_pose` 콜백에서만 갱신한다. 측위가 안 뜬 로봇은 `0,0` 을 보내고 UI 는 이를 맵 원점에
  그대로 찍는다. 9/10 실행에서 robot3 는 Nav2 미기동, robot8 은 주행 스택만 뜨고 amcl 이 없어
  두 마커가 원점에 겹쳐 있었다. pose 유효 플래그를 실어 `위치 미상` 으로 표시하는 편이 맞다.
