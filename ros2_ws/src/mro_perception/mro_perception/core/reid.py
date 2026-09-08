"""재식별(re-ID) 게이트 — 웹캠1 서술자와 AMR 관측을 대조한다. BRD v3.0 BR-T16 · BR-T17.

ROS 에 의존하지 않는 순수 함수·클래스만 둔다 (단위 테스트 대상). 노드(amr_detection_node)가 호출한다.

  hsv_hist()     bbox crop → 색 지문(H16×S8=128). 웹캠1(webcam_register)과 AMR 이 반드시 이 함수를 쓴다 (계약 2절)
  mean_hist()    최근 N 프레임 지문 평균. 한 장의 잡음을 씻는다. 웹캠1 이 서술자를 만들 때 쓰는 것과 같은 식
  similarity()   지문 두 개의 닮은 정도 0~1
  gate()         한 번의 판정 — 복장 클래스가 같고 유사도가 임계 이상이면 통과
  PairingGate    프레임마다 gate() 를 부르고, 연속 시간으로 paired 를 켜고 끄는 상태기계
"""
from collections import deque

import numpy as np


def hsv_hist(bgr_crop, bins=(16, 8)):
    """bbox crop → 정규화된 H·S 히스토그램(1차원). 웹캠1과 AMR 이 같은 함수를 써야 비교가 성립한다."""
    import cv2
    hsv = cv2.cvtColor(bgr_crop, cv2.COLOR_BGR2HSV)
    h = cv2.calcHist([hsv], [0, 1], None, list(bins), [0, 180, 0, 256])
    h = cv2.normalize(h, None).flatten()
    return h.astype(np.float32)


def mean_hist(hists):
    """최근 프레임들의 히스토그램 평균. 한 장의 잡음(먼 거리·흐림)을 씻는다.

    hsv_hist() 는 cv2.normalize 기본값(NORM_L2)이라 L2 노름이 1 이다. 평균을 내면 그 성질이 깨지므로
    다시 L2 정규화해서 돌려준다 — 웹캠1 이 발행하는 color_hist 와 같은 척도여야 하기 때문 (계약 2절).
    webcam_register_node.mean_hist 와 같은 구현이다.
    """
    if len(hists) == 1:
        return hists[0]
    m = np.mean(np.stack(hists), axis=0).astype(np.float32)
    norm = float(np.linalg.norm(m))
    if norm <= 0:
        return m
    return (m / norm).astype(np.float32)


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


class PairingGate:
    """재식별 게이트 상태기계. "지금 보이는 인형 중 내 담당이 있나" 를 매 프레임 판정하고 paired 를 관리한다.

    쓰는 법
      register(outfit, hist)      웹캠1 서술자(내 담당 작업자의 복장·색 지문)를 넣는다. 갱신되면 다시 불러도 된다
      update(t, candidates)       프레임마다. t 는 초, candidates 는 [(outfit, hist, payload), ...]
                                  → (paired, chosen_payload, sim, reason)

    판정 순서 (한 프레임)
      ① 복장 클래스가 내 담당과 같은 후보만 남긴다               — gate() 1단계. 다른 옷은 아예 후보가 아니다
      ② 남은 후보 각각 단일 프레임 유사도를 재서 가장 닮은 하나를 고른다 — 두 인형이 다 보여도 내 담당을 놓치지 않게
      ③ 고른 후보의 지문을 최근 N 프레임 버퍼에 넣고, 버퍼 평균으로 유사도를 다시 잰다
                                                                — 웹캠1 이 10 프레임 평균으로 서술자를 만드니 같은 척도
      ④ gate() 로 임계 판정. 실패하면 버퍼를 비운다 — 잡음 한 장은 평균이 흡수하지만, 같은 옷을 입은 남의
        지문이 연속으로 쌓이면 내 담당이 돌아와도 평균이 한동안 남의 색이라 재통과가 늦어진다
      ⑤ 히스테리시스 — on_s 동안 연속 통과해야 paired=True, off_s 동안 연속 실패(또는 안 보임)해야 False.
        한 프레임 깜빡임으로 켜졌다 꺼졌다 하지 않게 한다. BR-T17(STANDBY 뒤 재통과)도 이 규칙으로 성립한다
    """

    def __init__(self, threshold, frames=10, on_s=1.0, off_s=3.0):
        """threshold 는 Q-13 임계, frames 는 지문 평균 창, on_s/off_s 는 켜짐·꺼짐 히스테리시스(초)."""
        self.threshold = threshold
        self.on_s = on_s
        self.off_s = off_s
        self.buf = deque(maxlen=frames)      # 고른 후보의 최근 지문들 (평균용)
        self.outfit = None                    # 내 담당의 복장 클래스
        self.hist = None                      # 내 담당의 색 지문 (웹캠1 서술자)
        self.paired = False
        self._pass_since = None               # 연속 통과 시작 시각
        self._fail_since = None               # 연속 실패 시작 시각

    def ready(self):
        """서술자가 들어와 판정할 수 있는 상태인가."""
        return self.hist is not None

    def register(self, outfit, hist):
        """내 담당 서술자 등록. 복장이 바뀌면(다른 사람이 배정됨) 버퍼·상태를 초기화한다."""
        if outfit != self.outfit:
            self.buf.clear()
            self.paired = False
            self._pass_since = self._fail_since = None
        self.outfit = outfit
        self.hist = np.asarray(hist, np.float32)

    def update(self, t, candidates):
        """프레임 하나 판정. 반환 (paired, 고른 후보의 payload 또는 None, 평균 유사도, 사유)."""
        chosen, sim, reason = None, 0.0, 'no descriptor'
        if self.ready():
            same = [c for c in candidates if c[0] == self.outfit]                      # ①
            if same:
                _, chosen = max(((similarity(c[1], self.hist), c) for c in same), key=lambda x: x[0])   # ②
                self.buf.append(chosen[1])                                              # ③
                sim = similarity(mean_hist(list(self.buf)), self.hist)
                ok, reason = gate(chosen[0], self.outfit, sim, self.threshold)          # ④
                if not ok:
                    self.buf.clear()                                                    # 남의 지문이 쌓이지 않게
            elif candidates:
                ok, reason = False, f'outfit mismatch (visible: {sorted({c[0] for c in candidates})})'
            else:
                ok, reason = False, 'no candidate'
            self._hysteresis(t, ok)                                                     # ⑤
        return self.paired, (chosen[2] if chosen else None), sim, reason

    def _hysteresis(self, t, ok):
        if ok:
            self._fail_since = None
            if self._pass_since is None:
                self._pass_since = t
            if not self.paired and t - self._pass_since >= self.on_s:
                self.paired = True
        else:
            self._pass_since = None
            if self._fail_since is None:
                self._fail_since = t
            if self.paired and t - self._fail_since >= self.off_s:
                self.paired = False
                self.buf.clear()
