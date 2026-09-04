# 모델

| 파일 | 클래스 | 용도 | 상태 |
|---|---|---|---|
| `my_best.pt` | car, dummy | Kn1_mini OAK-D 모델. **골격 동작 확인용**. 시나리오에서는 교체 대상 | 이관 |
| `worker.pt` | worker_a, worker_b | 인형 2종(작업별 복장) 검출 모델 | **미학습 — [인지] 담당** |

학습 재현: Kn1_mini/yolo/custom_data.yaml 방식 그대로 (`yolo detect train model=yolo11n.pt data=... epochs=100 imgsz=640`).
색 편향 주의: 두 인형의 채도·명도 통계를 맞추고 검증 세트에 조명 변화를 넣을 것 (Kn1_mini/yolo/2_4_troubleshooting.md).
