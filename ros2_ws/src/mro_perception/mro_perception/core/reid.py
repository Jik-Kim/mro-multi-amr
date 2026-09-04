"""재식별(re-ID) 게이트 — 웹캠1 서술자와 AMR 관측을 대조한다. BRD v3.0 BR-T16.

ROS 에 의존하지 않는 순수 함수만 둔다. 노드(amr_detection_node)가 호출한다.
"""
import numpy as np


def hsv_hist(bgr_crop, bins=(16, 8)):
    """bbox crop → 정규화된 H·S 히스토그램(1차원). 웹캠1과 AMR 이 같은 함수를 써야 비교가 성립한다."""
    import cv2
    hsv = cv2.cvtColor(bgr_crop, cv2.COLOR_BGR2HSV)
    h = cv2.calcHist([hsv], [0, 1], None, list(bins), [0, 180, 0, 256])
    h = cv2.normalize(h, None).flatten()
    return h.astype(np.float32)


def similarity(hist_a, hist_b):
    """0~1. 1 이 동일. (Bhattacharyya 거리의 보수)"""
    import cv2
    d = cv2.compareHist(np.asarray(hist_a, np.float32), np.asarray(hist_b, np.float32), cv2.HISTCMP_BHATTACHARYYA)
    return float(max(0.0, 1.0 - d))


def gate(outfit_observed, outfit_registered, sim, threshold):
    """추종 허용 판정. 복장 클래스가 다르면 즉시 거부, 같으면 히스토그램 유사도가 임계 이상이어야 한다.

    반환: (allowed: bool, reason: str). 임계 미달이면 추종하지 않고 대기한다 (R-05). 임계값은 Q-13 [팀 확정].
    """
    if outfit_observed != outfit_registered:
        return False, f'outfit mismatch {outfit_observed}!={outfit_registered}'
    if sim < threshold:
        return False, f'sim {sim:.2f} < {threshold:.2f}'
    return True, 'ok'
