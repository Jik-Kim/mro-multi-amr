# 설계도 (SDD 3장)

2026-09-08(화) 09:30 설계도 리뷰 제출물. 교재 STEP 1~3 순서를 따른다.

| 파일 | 대응 | 내용 |
|---|---|---|
| `step1_deployment.svg` | SDD 3.1 | PC 5대 · AMR 2대 · 네트워크 배치. Discovery Server 구성 포함 |
| `step2_nav2.svg` | SDD 3.2 | 로봇 1대 내부. 우리 노드가 Nav2 스택과 무엇을 주고받는지 |
| `step3_scenario.svg` | SDD 3.3 | **3-1** 시나리오 S1~S10 흐름과 예외 E-1 · E-2 · E-3 · E-4 · E-7 |
| `step3b_system_flow.svg` | SDD 3.3 | **3-2** 전체 기능 흐름 — PC·노드 내부 처리와 그 사이를 오가는 토픽 19개 |
| `mro_design_diagrams.drawio` | — | 위 3장을 3 페이지로 담은 draw.io 편집본 |

## 고치는 법

그림은 **손으로 그리지 않고 스크립트가 만든다.** 좌표·상자 크기·연결선이 자동으로 맞춰지고,
문서와 값이 어긋나면 한 곳만 고치면 되기 때문이다.

```bash
# 내용을 고친 뒤
python3 tools/build_diagrams.py
```

스크립트는 글자가 상자 밖으로 넘치는지, 상자가 캔버스를 벗어나는지 검사해 경고한다.

draw.io 에서 직접 손볼 일이 있으면 `mro_design_diagrams.drawio` 를 열어 편집하고,
**그 결과를 스크립트에도 반영한다.** 반영하지 않으면 다음 생성 때 되돌아간다.

## 근거

`docs/architecture.md`(노드 구성) · `docs/interfaces.md`(토픽 계약) · `docs/setup.md`(네트워크) ·
BRD v3.0 3.2(시나리오) · SDD 2.3 NR-01(통신). 이 문서들이 바뀌면 그림도 같이 고친다.
