"""재식별(re-ID) 게이트 — 웹캠1 서술자와 AMR 관측을 대조한다. BRD v3.0 BR-T16 · BR-T17.

ROS 에 의존하지 않는 순수 함수·클래스만 둔다 (단위 테스트 대상). 노드(amr_detection_node)가 호출한다.

  hsv_hist()     bbox crop → 색 지문(H16×S8=128). 웹캠1(webcam_register)과 AMR 이 반드시 이 함수를 쓴다 (계약 2절)
  mean_hist()    최근 N 프레임 지문 평균. 한 장의 잡음을 씻는다. 웹캠1 이 서술자를 만들 때 쓰는 것과 같은 식
  similarity()   지문 두 개의 닮은 정도 0~1
  gate()         한 번의 판정 — 복장 클래스가 같고 유사도가 임계 이상이면 통과
  PairingGate    프레임마다 gate() 를 부르고, 연속 시간으로 paired 를 켜고 끄는 상태기계
  PairStateTracker        target/pair_state 값을 정한다 — NO_TARGET / GATE_FAIL(30 s) / PAIRED (FR-13, 계약 10.1·10.2)
  manual_pair_decision()  운영자의 명시 페어링 요청을 받을지 판정 — 거부 사유 표는 계약 10.5·10.8
  norm_ns()               로봇 네임스페이스 표기 통일 ('/robot3' == 'robot3' == '/robot3/')
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
        self.last_ok = False                  # 마지막 프레임의 gate() 결과 — 명시 페어링 GATE_OK 판정에 쓴다
        self._pass_since = None               # 연속 통과 시작 시각
        self._fail_since = None               # 연속 실패 시작 시각

    def ready(self):
        """서술자가 들어와 판정할 수 있는 상태인가."""
        return self.hist is not None

    def register(self, outfit, hist):
        """내 담당 서술자 등록. 복장이 바뀌면(다른 사람이 배정됨) 버퍼·상태를 초기화한다."""
        if outfit != self.outfit:
            self.reset()                      # 다른 사람 — 버퍼·paired·last_ok·시계 전부 처음으로
        self.outfit = outfit
        self.hist = np.asarray(hist, np.float32)

    def reset(self, forget=False):
        """판정 상태를 처음으로. mission/finish(FR-30, 계약 10.6) 때 노드가 부른다.

        forget=True 면 서술자까지 지워 ready() 가 False 가 된다 — 다음 mission/assigned 가 오기 전에는 짝을 만들지
        않게. 웹캠1 서술자는 노드가 따로 캐시하므로 새 미션이 오면 register() 로 곧바로 다시 준비된다.
        """
        self.buf.clear()
        self.paired = False
        self.last_ok = False
        self._pass_since = self._fail_since = None
        if forget:
            self.outfit = self.hist = None

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
            self.last_ok = ok
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


class PairStateTracker:
    """`target/pair_state` 의 값을 정한다 (FR-13, 계약 10.1·10.2). 값은 셋 — 관제 UI 는 이 세 문자열만 받는다.

      PAIRED     짝 확정 (자동 게이트든 명시 페어링이든)
      GATE_FAIL  작업자가 보이는데 게이트를 fail_s(30 s) 넘게 못 넘고 있다 → 관제 UI 에 명시 페어링 버튼이 뜬다
      NO_TARGET  그 밖 — 후보가 안 보인다 · 게이트가 아직 준비 안 됐다(미션·서술자 전) · 미달이 fail_s 를 못 채웠다

    30 s 는 "보이는데 못 넘는" 시간만 센다. 후보가 사라지면 0 으로 되돌린다 — 사람이 없어서 못 잡는 것은 폴백
    대상이 아니라서다 (10.1: 관제가 재면 이 둘을 못 가른다). 게이트가 준비되기 전(S1~S4)도 세지 않는다 — 판정
    자체가 없으니 '미달' 이 아니다. 미달이 fail_s 를 채우기 전 구간에 쓸 값은 계약에 없어 NO_TARGET 을 유지한다 [PM 확인].
    """

    def __init__(self, fail_s=30.0):
        """fail_s: 보이는데 못 넘는 상태가 이만큼 이어지면 GATE_FAIL (계약 10.1 의 30 s)."""
        self.fail_s = fail_s
        self.state = 'NO_TARGET'
        self._fail_since = None       # 보이는데 못 넘기 시작한 시각

    def update(self, t, visible, paired, ready=True):
        """프레임 하나. t 초, visible = 작업자 후보가 하나라도 보였나, paired = 확정됐나, ready = 게이트가 판정 가능한가."""
        if paired:
            self._fail_since = None
            self.state = 'PAIRED'
        elif not (visible and ready):
            self._fail_since = None
            self.state = 'NO_TARGET'
        else:
            if self._fail_since is None:
                self._fail_since = t
            self.state = 'GATE_FAIL' if t - self._fail_since >= self.fail_s else 'NO_TARGET'
        return self.state

    def fail_elapsed(self, t):
        """보이는데 못 넘고 있는 시간(초). 세고 있지 않으면 0."""
        return 0.0 if self._fail_since is None else t - self._fail_since

    def reset(self):
        """미션 종료 등으로 처음 상태로."""
        self._fail_since = None
        self.state = 'NO_TARGET'


def norm_ns(ns):
    """'/robot3' · 'robot3' · '/robot3/' 를 같은 이름으로. 관제가 어떤 꼴로 보내도 내 이름과 비교되게 (mission_cb 도 쓴다)."""
    ns = (ns or '').strip()
    if not ns.startswith('/'):
        ns = '/' + ns
    return ns.rstrip('/') or '/'


def manual_pair_decision(req_ns, my_ns, req_worker, my_worker, paired, visible, distance_m, max_m, gate_ok):
    """명시 페어링 요청(srv/ManualPair)을 받을지. → (accepted, reason). 위에서부터 먼저 걸리는 사유 하나만 돌려준다.

    계약 10.5 표 + 10.8. reason 은 srv 정의의 다섯 문자열 그대로 — control_ui 가 화면에 띄운다.
      WRONG_ROBOT     요청의 robot_ns 가 내가 아니다. 이 로봇에 미션이 없거나(요청 대상이 아니다), worker_id 가 비었거나
                      (10.3 두 필드 필수), 진행 중 미션의 작업자와 다른 worker_id 도 같은 급(10.8) — 재탐색 중의 수동
                      요청은 "대상을 바꾼다" 가 아니라 "지금 보이는 이 사람이 그 작업자다" 를 확정하는 것이다
      ALREADY_PAIRED  이미 짝이 되어 있다. 추종 중 대상 교체는 오페어링 0건(AC-04)을 흔든다 — mission/finish 로 먼저 푼다
      NO_TARGET       최근에 본 작업자 후보가 없다
      TOO_FAR         마지막 거리가 max_m(1 m, FR-14) 밖이거나 거리를 확인할 수 없다 — 멀면 좌표 오차가 커져 수동도 결과가 같다
      GATE_OK         요청이 오는 사이 게이트가 스스로 통과했다 (히스테리시스로 paired 가 곧 켜진다) — 수동이 필요 없다
    """
    if not req_ns or norm_ns(req_ns) != norm_ns(my_ns):      # 빈 이름은 네임스페이스 없이 뜬 노드('/')와 같아져 버린다
        return False, 'WRONG_ROBOT'
    if not my_worker or not req_worker or req_worker != my_worker:
        return False, 'WRONG_ROBOT'
    if paired:
        return False, 'ALREADY_PAIRED'
    if not visible:
        return False, 'NO_TARGET'
    if distance_m is None or distance_m > max_m:
        return False, 'TOO_FAR'
    if gate_ok:
        return False, 'GATE_OK'
    return True, ''
