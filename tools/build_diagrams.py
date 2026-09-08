#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SDD 3장 설계도 생성기 (9/8 설계도 리뷰 제출용).

출력 (docs/diagrams/):
  step1_deployment.svg   STEP 1 배치 블록도
  step2_nav2.svg         STEP 2 로봇 내부 · Nav2 스택과의 교환
  step3_scenario.svg     STEP 3-1 시나리오 S1~S10 플로우차트
  step3b_system_flow.svg STEP 3-2 전체 노드·토픽 흐름
  mro_design_diagrams.drawio   위 4장을 담은 draw.io 편집본 (4 페이지)

재생성:  python3 tools/build_diagrams.py
"""
import os
import re as _re
import html as _html

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.normpath(os.path.join(HERE, '..', 'docs', 'diagrams'))
os.makedirs(OUT, exist_ok=True)

FONT = "'Noto Sans KR','Malgun Gothic','Apple SD Gothic Neo','Segoe UI',sans-serif"
DRAWIO_FONT = "Noto Sans KR"

# style key -> (fill, stroke, title color)
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def from_yaml(relpath, *keys, default=None):
    """설정 파일에서 값을 직접 읽는다 — 그림에 숫자를 손으로 적지 않기 위해서다.

    9/7 에 웹캠1 캡처 fps 를 그림에만 30 으로 적어 실제 값(25)과 어긋난 적이 있다.
    읽지 못하면 default 를 쓰고 경고를 남긴다.
    """
    try:
        import yaml as _y
        with open(os.path.join(ROOT, relpath), encoding='utf-8') as f:
            node = _y.safe_load(f)
        for k in keys:
            node = node[k]
        return node
    except Exception as e:
        print('  [경고] %s 에서 %s 를 읽지 못했다 (%s) — 기본값 %r 사용'
              % (relpath, ' / '.join(keys), e, default))
        return default


WEBCAM_YAML = 'ros2_ws/src/mro_webcam/config/webcam.yaml'
CAM1_FPS = from_yaml(WEBCAM_YAML, 'webcam_register', 'ros__parameters', 'capture_fps', default=25)
CAM2_FPS = from_yaml(WEBCAM_YAML, 'webcam_hint', 'ros__parameters', 'fps', default=30.0)
PROC_HZ = from_yaml(WEBCAM_YAML, 'webcam_register', 'ros__parameters', 'process_hz', default=10.0)


CONTRACT_MD = 'docs/interfaces.md'


def contract_topics(relpath=CONTRACT_MD):
    """계약 4절 「토픽 계약」 표를 읽어 {토픽: (송신, 수신, 타입)} 로 돌려준다.

    그림에 토픽 이름·타입·구독자를 손으로 적지 않기 위해서다. 9/7 에 웹캠 fps 를 손으로
    적었다가 실제 값과 어긋난 적이 있고(그래서 from_yaml 을 넣었다), 토픽 표는 그보다
    훨씬 자주 바뀐다. **계약이 원본이고 그림은 그 사본이다** — 어긋나면 그림이 틀린 것이다.

    읽지 못하면 빈 dict 를 주고 경고한다. 부르는 쪽이 필요한 토픽을 못 찾으면 거기서 죽는다
    (조용히 옛 값을 그리는 것보다 낫다).
    """
    try:
        with open(os.path.join(ROOT, relpath), encoding='utf-8') as f:
            lines = f.read().splitlines()
    except OSError as e:
        print('  [경고] %s 를 읽지 못했다 (%s) — 토픽 표 없이 그린다' % (relpath, e))
        return {}
    out, inside = {}, False
    for ln in lines:
        if ln.startswith('## 4. 토픽 계약'):
            inside = True
            continue
        if inside and ln.startswith('## '):
            break
        if not inside or not ln.startswith('|'):
            continue
        c = [x.strip() for x in ln.strip('|').split('|')]
        if len(c) < 4 or c[0] in ('토픽', '') or set(c[0]) <= set('-: '):
            continue
        names = [t.strip(' `*') for t in c[0].split('`,')]
        names = [t.strip(' `*') for t in names if t.strip(' `*')]
        types = [t.strip() for t in c[3].split(',')]
        for i, nm in enumerate(names):
            out[nm] = (c[1], c[2], types[i] if i < len(types) else types[-1])
    print('  계약 4절에서 토픽 %d 개를 읽었다' % len(out))
    return out


def tp(topics, name, *, short=None):
    """토픽 상자에 넣을 두 줄 — 이름과 타입. 계약에 없는 이름이면 바로 죽는다."""
    if name not in topics:
        raise KeyError('계약 4절에 없는 토픽: %r — 이름이 바뀌었거나 오타다' % name)
    return [short or name, topics[name][2]]


STYLES = {
    'pc':     ('#EEF4FD', '#3576C8', '#1B4F91'),
    'pcf':    ('#FDF3E3', '#B06810', '#8A4F0B'),
    'robot':  ('#E9F6EE', '#2E8B57', '#1E6B40'),
    'ours':   ('#EEF4FD', '#3576C8', '#1B4F91'),
    'nav2':   ('#F3EEFA', '#6B4FA8', '#4B357A'),
    'hw':     ('#F1F2F4', '#5A5F6A', '#333840'),
    'safety': ('#FCECEC', '#C0392B', '#8E2A1F'),
    'exc':    ('#FCECEC', '#C0392B', '#8E2A1F'),
    'dec':    ('#FFF8DE', '#B8912B', '#6F5610'),
    'note':   ('#F7F8FA', '#C9CFD6', '#333840'),
    'term':   ('#DFF0E6', '#2E8B57', '#1E6B40'),
    'ext':    ('#F7F8FA', '#8A8F98', '#333840'),
    'band':   ('#FAFBFC', '#D7DBE0', '#5A5F6A'),
    'pcbox':  ('#F7FAFE', '#3576C8', '#1B4F91'),   # PC 컨테이너
    'fbox':   ('#FFFBF3', '#B06810', '#8A4F0B'),   # 관제 PC 컨테이너
    'nodebox':('#FFFFFF', '#A9C3E3', '#1B4F91'),   # 노드 컨테이너
    'hwbox':  ('#FAFAFB', '#8A8F98', '#333840'),   # 하드웨어 컨테이너
    'step':   ('#EEF4FD', '#7FA6D6', '#1B4F91'),   # 흐름 단계
    'hwstep': ('#F1F2F4', '#9AA0A8', '#333840'),
    'sfstep': ('#FCECEC', '#D98A80', '#8E2A1F'),
    'n2step': ('#F3EEFA', '#A992CE', '#4B357A'),
    # PC 구분 — 채움색으로만 나눈다. 테두리는 회색으로 통일해 화살표 색과 섞이지 않게 한다
    'pc_r3':  ('#EDF3FC', '#7C8794', '#1B4F91'),   # 로봇3 제어 PC
    'pc_r4':  ('#F4EFFB', '#7C8794', '#4B357A'),   # 로봇8 제어 PC
    'pc_ctl': ('#FFF9EC', '#7C8794', '#8A4F0B'),   # 관제 PC
    'pc_cam': ('#EDF7F1', '#7C8794', '#1E6B40'),   # 웹캠 PC
    'pc_hw':  ('#F2F3F5', '#7C8794', '#333840'),   # 로봇 온보드
    'in_r3':  ('#FFFFFF', '#9FB4CE', '#1B4F91'),   # 각 PC 안의 노드 상자
    'in_r4':  ('#FFFFFF', '#B4A2D2', '#4B357A'),
    'in_ctl': ('#FFFFFF', '#D6B685', '#8A4F0B'),
    'in_cam': ('#FFFFFF', '#93C4A8', '#1E6B40'),
    'in_hw':  ('#FFFFFF', '#AAB0B8', '#333840'),
    'in_saf': ('#FDF0EF', '#D98A80', '#8E2A1F'),   # 안전 노드는 따로 표시
    'bus':    ('#F7F8FA', '#5A5F6A', '#333840'),
    'bushex': ('#EFF2F6', '#5A5F6A', '#333840'),
    'db':     ('#FFFFFF', '#8A8F98', '#333840'),
    # STEP 3-3 ROS 그래프 (rqt_graph 규약) — 노드는 타원, 토픽은 네모, 서비스는 육각, 액션은 묶음
    'tp_t':   ('#EEF4FD', '#3576C8', '#1B4F91'),   # 토픽
    'tp_hw':  ('#F4F5F7', '#8A8F98', '#333840'),   # TB4 기본 토픽 (계약 표 밖)
    'tp_saf': ('#FCECEC', '#C0392B', '#8E2A1F'),   # 안전에 걸린 토픽
    'srv':    ('#FDF3E3', '#B06810', '#8A4F0B'),   # 서비스
    'act':    ('#F6F2FC', '#6B4FA8', '#4B357A'),   # 액션 묶음
    'act_srv':('#FDF3E3', '#C99A5B', '#8A4F0B'),   # 액션 속 서비스
    'act_tp': ('#EEF4FD', '#7FA6D6', '#1B4F91'),   # 액션 속 토픽
    # STEP 0 완전 통합 상세 — 레인(실행 주체) 배경과 그 안의 상자
    'lane_nav':  ('#F4EFFB', '#7C8794', '#4B357A'),   # Nav2 · 도킹 스택 레인
    'in_nav':    ('#FFFFFF', '#B4A2D2', '#4B357A'),
    'lane_step': ('#F7F8FA', '#C9CFD6', '#333840'),   # 맨 왼쪽 단계 레인
    'rowsep':    ('#E4E8EC', '#E4E8EC', '#5A5F6A'),   # 행(단계) 경계선
    'stepcell':  ('#EFF2F6', '#9AA0A8', '#333840'),   # 단계 이름 칸
    # 시나리오 단계 — 그 단계를 수행하는 PC 를 채움색으로 나타낸다
    'st_cam': ('#EDF7F1', '#93C4A8', '#1E6B40'),
    'st_ctl': ('#FFF6E8', '#D6B685', '#8A4F0B'),
    'st_rob': ('#EDF3FC', '#9FB4CE', '#1B4F91'),
    'st_saf': ('#FDF0EF', '#D98A80', '#8E2A1F'),
}

PAD_TOP = 21     # baseline of first line
GAP_1_2 = 19     # first -> second line
LINE_H = 15
PAD_BOT = 12


def box_h(lines):
    n = len(lines)
    if n <= 1:
        return 34
    return PAD_TOP + GAP_1_2 + LINE_H * (n - 2) + PAD_BOT


def esc(s):
    return _html.escape(s, quote=True)


class Diagram:
    def __init__(self, key, w, h, title, subtitle=''):
        self.key = key
        self.w = w
        self.h = h
        self.title = title
        self.subtitle = subtitle
        self.order = []          # rendering order
        self.nodes = {}
        self.edges = []
        self.free = []           # free-floating text

    # ---------- content ----------
    def box(self, nid, x, y, w, lines, style='pc', h=None, shape='rect', align='left'):
        h = h if h is not None else box_h(lines)
        n = dict(id=nid, x=x, y=y, w=w, h=h, lines=lines, style=style,
                 shape=shape, align=align)
        self.nodes[nid] = n
        self.order.append(('node', nid))
        return n

    def band(self, nid, x, y, w, h, label='', style='band', dash=True):
        n = dict(id=nid, x=x, y=y, w=w, h=h, lines=[label] if label else [],
                 style=style, shape='band', align='left', dash=dash)
        self.nodes[nid] = n
        self.order.append(('band', nid))
        return n

    def edge(self, a, b, label='', anchors='bottom-top', dashed=False,
             color='#5A5F6A', label_dx=0, label_dy=-6, width=1.4, arrow=True,
             mid=None, via=None, label_lines=None, label_x=None,
             label_anchor='middle'):
        self.edges.append(dict(a=a, b=b, label=label, anchors=anchors,
                               dashed=dashed, color=color, ldx=label_dx,
                               ldy=label_dy, width=width, arrow=arrow,
                               mid=mid, via=list(via) if via else None,
                               lines=label_lines, lx_fix=label_x,
                               lanchor=label_anchor))

    def text(self, x, y, s, size=11, color='#5A5F6A', bold=False, anchor='start'):
        self.free.append(dict(x=x, y=y, s=s, size=size, color=color,
                              bold=bold, anchor=anchor))

    # ---------- geometry ----------
    def anchor(self, nid, side):
        n = self.nodes[nid]
        x, y, w, h = n['x'], n['y'], n['w'], n['h']
        f = 0.5
        if '@' in side:
            side, frac = side.split('@')
            f = float(frac)
        return {'top': (x + w * f, y), 'bottom': (x + w * f, y + h),
                'left': (x, y + h * f), 'right': (x + w, y + h * f)}[side]

    def path_of(self, e):
        if e.get('raw'):
            pts = [tuple(p) for p in e['via']]
            return pts, (pts[0][0] + pts[-1][0]) / 2, pts[0][1]
        s_side, t_side = e['anchors'].split('-')
        x1, y1 = self.anchor(e['a'], s_side)
        x2, y2 = self.anchor(e['b'], t_side)
        if e.get('via'):
            pts = [(x1, y1)] + [tuple(p) for p in e['via']] + [(x2, y2)]
            k = len(pts) // 2
            lx = (pts[k - 1][0] + pts[k][0]) / 2
            ly = (pts[k - 1][1] + pts[k][1]) / 2
            return pts, lx, ly
        # 'bottom@0.30' 처럼 분수가 붙어도 방향을 알아봐야 한다. 예전에는 그대로 비교해서
        # 세로 화살표가 가로 분기로 빠졌고, mid 가 y 가 아니라 x 로 쓰였다 (9/8).
        if s_side.split('@')[0] in ('bottom', 'top'):
            my = e['mid'] if e.get('mid') is not None else (y1 + y2) / 2
            if abs(x1 - x2) < 1:
                pts = [(x1, y1), (x2, y2)]
            else:
                pts = [(x1, y1), (x1, my), (x2, my), (x2, y2)]
            lx, ly = (x1 + x2) / 2, my
        else:
            mx = e['mid'] if e.get('mid') is not None else (x1 + x2) / 2
            if abs(y1 - y2) < 1:
                pts = [(x1, y1), (x2, y2)]
            else:
                pts = [(x1, y1), (mx, y1), (mx, y2), (x2, y2)]
            lx, ly = mx, (y1 + y2) / 2
        return pts, lx, ly

    def label_rect(self, e):
        """화살표 라벨 상자의 (x, y, w, h). 없으면 None."""
        if not e.get('lines'):
            return None
        _pts, lx, ly = self.path_of(e)
        ls = e['lines']
        lh = 13
        # 첫 줄은 font-weight 600 이라 같은 크기라도 넓다. 안 보면 글자가 상자 밖으로 나간다
        wmax = max(self.text_w(t, False) * (1.10 if i == 0 else 1.0)
                   for i, t in enumerate(ls))
        bw, bh = wmax + 14, lh * len(ls) + 8
        bx = lx - bw / 2 if e['lanchor'] == 'middle' else lx - 7
        if e.get('lx_fix') is not None:
            bx = e['lx_fix'] - bw / 2 if e['lanchor'] == 'middle' else e['lx_fix'] - 7
        return bx, ly - bh - 5, bw, bh

    def autosize(self, margin=28):
        b = 0
        for n in self.nodes.values():
            b = max(b, n['y'] + n['h'])
        for t in self.free:
            b = max(b, t['y'] + 6)
        self.h = int(b + margin)
        return self

    def text_w(self, s, first):
        """글자 폭 어림. 한글·기호는 전각, 나머지는 반각으로 센다."""
        cj, asc = (12.5, 7.0) if first else (11.0, 6.1)
        return sum(cj if ord(c) > 0x2E80 else asc for c in s)

    def check(self):
        warn = []
        for n in self.nodes.values():
            if n['y'] + n['h'] > self.h:
                warn.append('%s 가 캔버스 아래로 넘침' % n['id'])
            if n['x'] + n['w'] > self.w:
                warn.append('%s 가 캔버스 오른쪽으로 넘침' % n['id'])
            if n['shape'] == 'band':
                continue
            # 상자 폭 대비 글자 폭 — 마름모·육각형은 가운데가 좁아 실효 폭이 작다
            usable = n['w'] * (0.60 if n['shape'] == 'diamond' else
                               0.80 if n['shape'] in ('hex', 'stadium') else 1.0) - 22
            for i, ln in enumerate(n['lines']):
                over = self.text_w(ln, i == 0) - usable
                if over > 0:
                    warn.append('%s %d번째 줄이 %d px 넘침 — "%s"'
                                % (n['id'], i + 1, over, ln))
        # 화살표 라벨이 상자를 가리는지 — 가리면 글자가 서로 겹쳐 못 읽는다
        boxes = [n for n in self.nodes.values() if n['shape'] != 'band']
        for e in self.edges:
            r = self.label_rect(e)
            if r is None:
                continue
            lx, ly, lw, lh = r
            for n in boxes:
                if (lx < n['x'] + n['w'] and lx + lw > n['x']
                        and ly < n['y'] + n['h'] and ly + lh > n['y']):
                    warn.append('라벨 "%s" 이 상자 %s 를 가린다' % (e['lines'][0], n['id']))
                    break
        for e in self.edges:
            r = self.label_rect(e)
            if r is None:
                continue
            lx, ly, lw, _lh = r
            if lx < 0 or lx + lw > self.w:
                warn.append('라벨 "%s" 이 캔버스 밖으로 %d px 나간다'
                            % (e['lines'][0], max(-lx, lx + lw - self.w)))
        # 7. 선이 남의 상자 뒤로 지나가는지 — 그 상자에서 나오는 선처럼 보인다.
        #    9/8 에 scan 이 Nav2 상자 뒤를 지나 "Nav2 도 관련 있느냐" 는 질문이 나왔다.
        def _seg_hits(p, q, n):
            x0, x1 = sorted((p[0], q[0]))
            y0, y1 = sorted((p[1], q[1]))
            return (x0 < n['x'] + n['w'] - 1 and x1 > n['x'] + 1
                    and y0 < n['y'] + n['h'] - 1 and y1 > n['y'] + 1)

        seen = set()
        for e in self.edges:
            pts, _lx, _ly = self.path_of(e)
            for i in range(len(pts) - 1):
                for n in boxes:
                    if n['id'] in (e['a'], e['b']) or (e['a'], e['b'], n['id']) in seen:
                        continue
                    if _seg_hits(pts[i], pts[i + 1], n):
                        seen.add((e['a'], e['b'], n['id']))
                        warn.append('%s → %s 선이 상자 %s 뒤로 지나간다'
                                    % (e['a'], e['b'], n['id']))
        # 8. 화살촉이 다른 상자에 덮이는지 — 방향을 알 수 없게 된다
        for e in self.edges:
            if not e['arrow']:
                continue
            tx, ty = self.path_of(e)[0][-1]
            for n in boxes:
                if n['id'] == e['b']:
                    continue
                if (n['x'] - 2 < tx < n['x'] + n['w'] + 2
                        and n['y'] - 2 < ty < n['y'] + n['h'] + 2):
                    warn.append('%s → %s 의 화살촉이 상자 %s 에 덮인다'
                                % (e['a'], e['b'], n['id']))
                    break
        # 9. 화살촉 둘이 같은 점에 꽂히는지 — 화살표 하나로 보인다
        tips = {}
        for e in self.edges:
            if not e['arrow']:
                continue
            k = tuple(round(v) for v in self.path_of(e)[0][-1])
            tips.setdefault(k, []).append('%s→%s' % (e['a'], e['b']))
        for k, v in tips.items():
            if len(v) > 1:
                warn.append('화살촉이 같은 점 %s 에 겹친다 — %s' % (k, ' · '.join(v)))
        # 라벨이 컨테이너 경계선에 걸치는지 — 배경 상자와 반쯤 겹쳐 보기 나쁘다
        bands = [n for n in self.nodes.values() if n['shape'] == 'band']
        for e in self.edges:
            r = self.label_rect(e)
            if r is None:
                continue
            lx, ly, lw, lh = r
            for b in bands:
                if not (lx < b['x'] + b['w'] and lx + lw > b['x']):
                    continue
                for edge_y in (b['y'], b['y'] + b['h']):
                    if ly < edge_y < ly + lh:
                        warn.append('라벨 "%s" 이 컨테이너 %s 의 경계선에 걸친다'
                                    % (e['lines'][0], b['id']))
                        break
                else:
                    continue
                break
        # 라벨이 화살촉을 덮는지 — 선은 보이는데 화살표 머리만 사라진다
        heads = []
        for e in self.edges:
            if not e.get('arrow'):
                continue
            pts, _lx, _ly = self.path_of(e)
            heads.append((pts[-1], e))
        for e in self.edges:
            r = self.label_rect(e)
            if r is None:
                continue
            lx, ly, lw, lh = r
            for (hx, hy), he in heads:
                if lx - 4 < hx < lx + lw + 4 and ly - 4 < hy < ly + lh + 4:
                    warn.append('라벨 "%s" 이 화살표 "%s" 의 화살촉을 덮는다'
                                % (e['lines'][0],
                                   (he['lines'] or [he['label'] or '이름 없음'])[0]))
                    break
        # 라벨끼리 겹치는지
        rects = [(self.label_rect(e), e) for e in self.edges]
        rects = [(r, e) for r, e in rects if r]
        for i in range(len(rects)):
            for j in range(i + 1, len(rects)):
                (ax, ay, aw2, ah), ea = rects[i]
                (bx2, by2, bw2, bh2), eb = rects[j]
                if (ax < bx2 + bw2 and ax + aw2 > bx2
                        and ay < by2 + bh2 and ay + ah > by2):
                    warn.append('라벨 "%s" 과 "%s" 이 겹친다'
                                % (ea['lines'][0], eb['lines'][0]))
        for w in warn:
            print('  [경고]', w)
        return warn

    # ---------- SVG ----------
    def to_svg(self):
        o = []
        o.append('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
                 'viewBox="0 0 %d %d" font-family="%s">'
                 % (self.w, self.h, self.w, self.h, FONT))
        o.append('<defs>')
        for name, col in [('ar', '#5A5F6A'), ('arR', '#C0392B'),
                          ('arB', '#3576C8'), ('arP', '#6B4FA8'),
                          ('arO', '#B06810'), ('arG', '#2E8B57')]:
            o.append('<marker id="%s" viewBox="0 0 10 10" refX="9" refY="5" '
                     'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
                     '<path d="M0,0 L10,5 L0,10 z" fill="%s"/></marker>' % (name, col))
        o.append('</defs>')
        o.append('<rect width="100%" height="100%" fill="#FFFFFF"/>')
        # title
        o.append('<text x="24" y="34" font-size="19" font-weight="700" '
                 'fill="#1A1D22">%s</text>' % esc(self.title))
        if self.subtitle:
            o.append('<text x="24" y="55" font-size="12" fill="#5A5F6A">%s</text>'
                     % esc(self.subtitle))

        # bands first
        for kind, nid in self.order:
            if kind == 'band':
                o.append(self._svg_band(self.nodes[nid]))
        # edges under boxes
        for e in self.edges:
            o.append(self._svg_edge(e))
        for kind, nid in self.order:
            if kind == 'node':
                o.append(self._svg_box(self.nodes[nid]))
        # 화살표 라벨은 상자보다 나중에 — 먼저 그리면 상자에 덮여 글자가 가려진다
        for e in self.edges:
            o.append(self._svg_edge_label(e))
        for t in self.free:
            o.append('<text x="%g" y="%g" font-size="%g" fill="%s"%s%s>%s</text>'
                     % (t['x'], t['y'], t['size'], t['color'],
                        ' font-weight="700"' if t['bold'] else '',
                        ' text-anchor="%s"' % t['anchor'] if t['anchor'] != 'start' else '',
                        esc(t['s'])))
        o.append('</svg>')
        return '\n'.join(o)

    def _svg_band(self, n):
        fill, stroke, tc = STYLES[n['style']]
        s = ('<rect x="%g" y="%g" width="%g" height="%g" rx="6" fill="%s" '
             'stroke="%s" stroke-width="%g"%s/>'
             % (n['x'], n['y'], n['w'], n['h'], fill, stroke,
                1 if n.get('dash', True) else 1.6,
                ' stroke-dasharray="5 4"' if n.get('dash', True) else ''))
        if n['lines']:
            s += ('<text x="%g" y="%g" font-size="12" font-weight="700" fill="%s">%s</text>'
                  % (n['x'] + 12, n['y'] + 19, tc, esc(n['lines'][0])))
        return s

    def _svg_box(self, n):
        fill, stroke, tc = STYLES[n['style']]
        x, y, w, h = n['x'], n['y'], n['w'], n['h']
        out = []
        if n['shape'] == 'diamond':
            pts = '%g,%g %g,%g %g,%g %g,%g' % (x + w / 2, y, x + w, y + h / 2,
                                               x + w / 2, y + h, x, y + h / 2)
            out.append('<polygon points="%s" fill="%s" stroke="%s" stroke-width="1.6"/>'
                       % (pts, fill, stroke))
        elif n['shape'] == 'stadium':
            out.append('<rect x="%g" y="%g" width="%g" height="%g" rx="%g" fill="%s" '
                       'stroke="%s" stroke-width="1.6"/>' % (x, y, w, h, h / 2, fill, stroke))
        elif n['shape'] == 'hex':
            k = min(26, w / 4)
            pts = '%g,%g %g,%g %g,%g %g,%g %g,%g %g,%g' % (
                x + k, y, x + w - k, y, x + w, y + h / 2,
                x + w - k, y + h, x + k, y + h, x, y + h / 2)
            out.append('<polygon points="%s" fill="%s" stroke="%s" stroke-width="1.8"/>'
                       % (pts, fill, stroke))
        elif n['shape'] == 'ellipse':
            out.append('<ellipse cx="%g" cy="%g" rx="%g" ry="%g" fill="%s" '
                       'stroke="%s" stroke-width="1.8"/>'
                       % (x + w / 2, y + h / 2, w / 2, h / 2, fill, stroke))
        elif n['shape'] == 'cyl':
            r = min(13, h / 5)
            out.append('<path d="M %g %g L %g %g A %g %g 0 0 0 %g %g L %g %g '
                       'A %g %g 0 0 0 %g %g Z" fill="%s" stroke="%s" stroke-width="1.6"/>'
                       % (x, y + r, x, y + h - r, w / 2, r, x + w, y + h - r,
                          x + w, y + r, w / 2, r, x, y + r, fill, stroke))
            out.append('<path d="M %g %g A %g %g 0 0 0 %g %g A %g %g 0 0 0 %g %g" '
                       'fill="none" stroke="%s" stroke-width="1.6"/>'
                       % (x, y + r, w / 2, r, x + w, y + r, w / 2, r, x, y + r, stroke))
        else:
            out.append('<rect x="%g" y="%g" width="%g" height="%g" rx="5" fill="%s" '
                       'stroke="%s" stroke-width="1.6"/>' % (x, y, w, h, fill, stroke))
        centered = n['align'] == 'center' or n['shape'] in ('diamond', 'stadium', 'hex', 'cyl', 'ellipse')
        tx = x + w / 2 if centered else x + 12
        anc = ' text-anchor="middle"' if centered else ''
        lines = n['lines']
        if centered and len(lines) > 1:
            total = GAP_1_2 + LINE_H * (len(lines) - 2)
            base = y + h / 2 - total / 2 + 5
        elif centered:
            base = y + h / 2 + 5
        else:
            base = y + PAD_TOP
        for i, ln in enumerate(lines):
            by = base if i == 0 else base + GAP_1_2 + LINE_H * (i - 1)
            if i == 0:
                out.append('<text x="%g" y="%g" font-size="12.5" font-weight="700" '
                           'fill="%s"%s>%s</text>' % (tx, by, tc, anc, esc(ln)))
            else:
                col = '#8A2E22' if ln.startswith('!') else '#3A3E46'
                txt = ln[1:] if ln.startswith('!') else ln
                out.append('<text x="%g" y="%g" font-size="11" fill="%s"%s>%s</text>'
                           % (tx, by, col, anc, esc(txt)))
        return ''.join(out)

    def _svg_edge(self, e):
        pts, lx, ly = self.path_of(e)
        d = 'M ' + ' L '.join('%g %g' % p for p in pts)
        mk = {'#5A5F6A': 'ar', '#C0392B': 'arR', '#3576C8': 'arB',
              '#6B4FA8': 'arP', '#B06810': 'arO', '#2E8B57': 'arG'}.get(e['color'], 'ar')
        s = ('<path d="%s" fill="none" stroke="%s" stroke-width="%g"%s%s/>'
             % (d, e['color'], e['width'],
                ' stroke-dasharray="6 4"' if e['dashed'] else '',
                ' marker-end="url(#%s)"' % mk if e['arrow'] else ''))
        return s

    def _svg_edge_label(self, e):
        r = self.label_rect(e)
        if r is None:
            if e['label']:
                _pts, lx, ly = self.path_of(e)
                return ('<text x="%g" y="%g" font-size="10.5" fill="%s" text-anchor="middle">'
                        '%s</text>' % (lx + e['ldx'], ly + e['ldy'], e['color'], esc(e['label'])))
            return ''
        bx, by, bw, bh = r
        # 라벨에 배경 상자를 두지 않는다 — 상자가 화살표를 덮어 선이 끊겨 보인다 (9/8).
        # 대신 글자마다 흰 테두리(halo)를 두른다. 글자 사이로 선이 그대로 보인다.
        s = ''
        for i, t in enumerate(e['lines']):
            col = e['color'] if i == 0 else '#3A3E46'
            wt = ' font-weight="600"' if i == 0 else ''
            s += ('<text x="%g" y="%g" font-size="10.5" fill="%s"%s '
                  'stroke="#FFFFFF" stroke-width="3.2" stroke-linejoin="round" '
                  'paint-order="stroke">%s</text>'
                  % (bx + 7, by + 15 + 13 * i, col, wt, esc(t)))
            s += ('<text x="%g" y="%g" font-size="10.5" fill="%s"%s>%s</text>'
                  % (bx + 7, by + 15 + 13 * i, col, wt, esc(t)))
        return s

    # ---------- draw.io ----------
    def to_drawio_page(self, page_id):
        cells = ['<mxCell id="0"/>', '<mxCell id="1" parent="0"/>']
        uid = [0]

        def nid(prefix):
            uid[0] += 1
            return '%s_%s_%d' % (self.key, prefix, uid[0])

        idmap = {}
        cells.append(self._dio_text(nid('t'), 24, 12, self.w - 48, 30,
                                    self.title, 19, True, '#1A1D22'))  # 제목은 왼쪽 정렬이라 폭이 넉넉하면 된다
        if self.subtitle:
            cells.append(self._dio_text(nid('t'), 24, 42, self.w - 48, 20,
                                        self.subtitle, 12, False, '#5A5F6A'))
        # 밴드를 먼저 깐다. draw.io 의 z 순서는 이 문서 순서 그대로라, 삽입 순서로 내보내면
        # 자식 상자보다 나중에 추가된 안쪽 밴드가 그 상자들을 덮어 패널이 텅 빈 것처럼 보인다
        # (9/8 관제 PC). _svg() 는 밴드 → 상자로 그리므로 SVG 만 멀쩡했다. 같은 순서로 맞춘다.
        # sorted 는 안정 정렬이라 밴드끼리의 순서(바깥 먼저)와 상자끼리의 순서는 그대로다.
        for kind, k in sorted(self.order, key=lambda t: 0 if t[0] == 'band' else 1):
            n = self.nodes[k]
            fill, stroke, tc = STYLES[n['style']]
            gid = nid('n')
            idmap[k] = gid
            if n['shape'] == 'diamond':
                shape = 'rhombus;'
            elif n['shape'] == 'stadium':
                shape = 'rounded=1;arcSize=50;'
            elif n['shape'] == 'hex':
                shape = 'shape=hexagon;'
            elif n['shape'] == 'ellipse':
                shape = 'ellipse;'
            elif n['shape'] == 'cyl':
                shape = 'shape=cylinder3;boundedLbl=1;'
            else:
                shape = 'rounded=1;arcSize=6;'
            dashed = ('dashed=1;dashPattern=5 4;'
                      if kind == 'band' and n.get('dash', True) else 'dashed=0;')
            align = ('align=center;' if (n['align'] == 'center' or
                                         n['shape'] in ('diamond', 'stadium', 'hex', 'cyl', 'ellipse'))
                     else 'align=left;spacingLeft=10;')
            # draw.io 의 value 는 XML 속성이다. HTML 태그째로 넣되
            # 문자열 전체를 마지막에 한 번만 이스케이프한다 (아래 esc()).
            body = []
            for i, ln in enumerate(n['lines']):
                t = ln[1:] if ln.startswith('!') else ln
                if i == 0:
                    body.append('<font style="font-size:%gpx;line-height:%gpx" color="%s">'
                                '<b>%s</b></font>' % (12.5, GAP_1_2, tc, t))
                else:
                    body.append('<font style="font-size:%gpx;line-height:%gpx" color="%s">'
                                '%s</font>' % (11, LINE_H,
                                               '#8A2E22' if ln.startswith('!') else '#3A3E46', t))
            # 줄바꿈을 끈다 — 켜 두면 폰트가 조금만 넓어도 줄이 접혀 상자 밖으로 밀린다.
            # 폭은 SVG 에서 이미 글자에 맞춰 잡았고, check() 가 넘침을 잡아 준다.
            # 밴드(패널) 제목도 위에 붙인다. middle 로 두면 675 px 짜리 PC 패널
            # 한가운데로 내려가 안쪽 노드 상자에 완전히 가린다 — SVG 는 y+19 에
            # 그리는데 draw.io 만 가운데였다 (9/8). 둘을 같은 자리로 맞춘다.
            vtop = n['align'] != 'center' and n['shape'] in ('rect', 'band')
            style = ('%swhiteSpace=nowrap;html=1;fillColor=%s;strokeColor=%s;%s'
                     'verticalAlign=%s;spacing=0;spacingTop=%g;spacingBottom=0;'
                     'fontFamily=%s;fontSize=11;%s'
                     % (shape, fill, stroke, align,
                        'top' if vtop else 'middle',
                        # SVG 첫 줄 글자 윗변에 맞춘 값. 상자는 y+21, 밴드는 y+19 다.
                        (5 if n['shape'] == 'band' else 7) if vtop else 0,
                        DRAWIO_FONT, dashed))
            cells.append('<mxCell id="%s" value="%s" style="%s" vertex="1" parent="1">'
                         '<mxGeometry x="%g" y="%g" width="%g" height="%g" as="geometry"/>'
                         '</mxCell>' % (gid, esc('<br>'.join(body)), style,
                                        n['x'], n['y'], n['w'], n['h']))
        def side_xy(side):
            f = 0.5
            if '@' in side:
                side, frac = side.split('@')
                f = float(frac)
            return {'top': (f, 0.0), 'bottom': (f, 1.0),
                    'left': (0.0, f), 'right': (1.0, f)}[side]

        for e in self.edges:
            if e.get('raw'):
                continue
            s_side, t_side = e['anchors'].split('-')
            ex, ey = side_xy(s_side)
            nx, ny = side_xy(t_side)
            style = ('edgeStyle=orthogonalEdgeStyle;rounded=0;html=1;'
                     'exitX=%g;exitY=%g;exitDx=0;exitDy=0;entryX=%g;entryY=%g;'
                     'entryDx=0;entryDy=0;strokeColor=%s;strokeWidth=%g;dashed=%d;'
                     'fontFamily=%s;fontSize=10;fontColor=%s;endArrow=%s;'
                     % (ex, ey, nx, ny, e['color'], e['width'], 1 if e['dashed'] else 0,
                        DRAWIO_FONT, e['color'], 'classic' if e['arrow'] else 'none'))
            pts, _, _ = self.path_of(e)
            inner = pts[1:-1]
            arr = ''
            if inner:
                arr = ('<Array as="points">%s</Array>'
                       % ''.join('<mxPoint x="%g" y="%g"/>' % (px, py)
                                 for px, py in inner))
            lbl = '<br>'.join(e['lines']) if e.get('lines') else e['label']
            cells.append('<mxCell id="%s" value="%s" style="%s" edge="1" parent="1" '
                         'source="%s" target="%s"><mxGeometry relative="1" as="geometry">'
                         '%s</mxGeometry></mxCell>'
                         % (nid('e'), esc(lbl), style,
                            idmap[e['a']], idmap[e['b']], arr))
        for t in self.free:
            tw = max(60.0, self.text_w(t['s'], t['size'] >= 12) * (t['size'] / 12.5) + 14)
            if t['anchor'] == 'start':
                tx, ta = t['x'] - 7, 'left'
            elif t['anchor'] == 'end':
                tx, ta = t['x'] - tw + 7, 'right'
            else:
                tx, ta = t['x'] - tw / 2, 'center'
            cells.append(self._dio_text(nid('f'), tx, t['y'] - t['size'] - 3, tw,
                                        t['size'] + 8, t['s'], t['size'],
                                        t['bold'], t['color'], ta))
        model = ('<mxGraphModel dx="1400" dy="900" grid="1" gridSize="10" guides="1" '
                 'tooltips="1" connect="1" arrows="1" fold="1" page="1" pageScale="1" '
                 'pageWidth="%d" pageHeight="%d" math="0" shadow="0"><root>%s</root>'
                 '</mxGraphModel>' % (self.w, self.h, ''.join(cells)))
        return ('<diagram id="%s" name="%s">%s</diagram>'
                % (esc(page_id), esc(self.key), model))

    @staticmethod
    def _dio_text(gid, x, y, w, h, s, size, bold, color, align='left'):
        style = ('text;html=1;whiteSpace=nowrap;strokeColor=none;fillColor=none;align=%s;'
                 'verticalAlign=middle;spacing=0;fontFamily=%s;fontSize=%g;fontColor=%s;%s'
                 % (align, DRAWIO_FONT, size, color, 'fontStyle=1;' if bold else ''))
        return ('<mxCell id="%s" value="%s" style="%s" vertex="1" parent="1">'
                '<mxGeometry x="%g" y="%g" width="%g" height="%g" as="geometry"/>'
                '</mxCell>' % (gid, esc(s), style, x, y, w, h))


# =====================================================================
# STEP 1 — 배치 블록도
# =====================================================================
# STEP 2 — 로봇 내부 · Nav2 교환
# =====================================================================
# STEP 3 — 시나리오 플로우차트
# =====================================================================
# STEP 0 — 완전 통합 상세 (레인 = 실행 주체 · 세로 = 시나리오)
# =====================================================================
# STEP 4 — 노드 명세도 (재현 가능 수준)
# =====================================================================
# STEP 5 — 시스템 노드 그래프 (재현용)
# =====================================================================
def step5_graph_w1():
    """웹캠1 갈래 — PC 안에 노드, 노드 안에 동작 순서도.

    강사 예시(SDD_0cha 5p) 형식을 따른다.
      · PC 컨테이너 안에 노드 컨테이너, 그 안에 세로 순서도를 그린다.
      · 상자에는 **무슨 일을 하는지 우리말로** 적는다. 함수명 · 변수명은 쓰지 않는다.
      · 판단은 마름모, 시작은 타원.
      · 화살표에는 **ROS 토픽 이름과 메시지 타입**을 적는다. 이건 그대로 쓰는 것이 맞다.
      · 외부 장비 상자에만 사양을 적는다.

    받는 팀이 우리 문서를 못 본다는 전제이므로 내부 이슈 번호는 쓰지 않는다.
    """
    T = contract_topics()
    TOPIC, PLAIN = '#3576C8', '#5A5F6A'
    WD = '/webcam1/worker_descriptor'
    wd_ty = T[WD][2] if WD in T else 'WorkerDescriptor'
    MA_ty = T['mission/assigned'][2] if 'mission/assigned' in T else 'Mission'

    d = Diagram('STEP5 노드 그래프 — 웹캠1', 2860, 100,
                'STEP 5 — 웹캠1 갈래   ·   PC 안에 노드, 노드 안에 동작 순서도',
                '상자에는 무슨 일을 하는지 우리말로 적는다 (함수명 · 변수명은 쓰지 않는다). '
                '화살표에는 ROS 토픽 이름과 메시지 타입을 적는다. 사양은 외부 장비 상자에만 적는다.')

    def bw(lines, extra=46, lo=180, hi=380):
        m = max(d.text_w(t, k == 0) for k, t in enumerate(lines))
        return max(lo, min(hi, m + extra))

    def step(nid, x, y, lines, sty='step', shape='rect'):
        w = bw(lines) * (1.75 if shape == 'diamond' else 1.0)
        h = 62 if shape == 'diamond' else None
        d.box(nid, x - w / 2, y, w, lines, sty, h=h, shape=shape, align='center')
        return d.nodes[nid]

    def flow(a, b, lab=None, anc='bottom@0.5-top@0.5', col=PLAIN, **kw):
        d.edge(a, b, '', anc, color=col, width=1.6, label_lines=lab, **kw)

    # ═══════════ 외부 장비 ═══════════
    cam = ['웹캠 1        USB 카메라',
           '설치      출입구 정면 · 남쪽 외벽',
           '시야      출입구만 본다 (그 밖은 등록하지 않는다)',
           '촬영      초당 25장 — 이 장비의 상한',
           '해상도   장치 기본값을 그대로 쓴다',
           '             프로그램이 따로 지정하지 않는다',
           '연결      USB. 꽂힌 카메라를 프로그램이 자동으로 찾는다',
           '             여러 대면 나중에 꽂은 것부터 시도한다',
           '             내장 카메라가 잡히면 번호를 직접 지정한다',
           '전달      ROS 토픽이 아니다 — 프로그램이 직접 읽는다',
           '             오래된 화면이 쌓이지 않게 한 장만 들고 있는다']
    d.box('cam1', 30, 268, bw(cam, extra=30, hi=460), cam, 'hw')
    d.text(30, 250, '외부 장비   — 장비 상자에는 사양을 적는다', 12, '#333840', bold=True)

    # ═══════════ PC-W1 · webcam_register ═══════════
    CX = 720
    y = 214
    step('a0', CX, y, ['작업자 등록 시작'], 'term', 'stadium'); y += 132
    step('a1', CX, y, ['웹캠 영상 한 장 읽기'], 'step'); y += 92
    step('a2', CX, y, ['사람 검출'], 'step'); y += 100
    step('a3', CX, y, ['등록 대상 복장인가?'], 'dec', 'diamond'); y += 132
    step('a4', CX, y, ['같은 사람에게 추적 번호 유지'], 'step'); y += 92
    step('a5', CX, y, ['상의 색 지문 뽑기'], 'step'); y += 92
    step('a6', CX, y, ['최근 열 장을 평균내 지문 안정화'], 'step'); y += 100
    step('a7', CX, y, ['1초 넘게 같은 사람으로 보이는가?'], 'dec', 'diamond'); y += 132
    step('a8', CX, y, ['작업자 번호 부여  (최대 2명)'], 'step'); y += 92
    step('a9', CX, y, ['작업자 정보 내보내기', '등록 뒤에도 1초마다 다시 보낸다'], 'step')
    a9b = d.nodes['a9']['y'] + d.nodes['a9']['h']

    step('x1', CX + 510, d.nodes['a3']['y'] + 4, ['등록하지 않고 버린다'], 'exc')
    step('x2', CX + 510, d.nodes['a7']['y'] + 4, ['아직 확정하지 않는다'], 'exc')

    # 화살표마다 무엇이 넘어가는지 적는다 — 상자만 이으면 무엇이 흐르는지 알 수 없다
    for a, b, lab in [
            ('a0', 'a1', None),
            ('a1', 'a2', ['카메라 화면 한 장']),
            ('a2', 'a3', ['화면 속 사람 위치 · 복장 종류 · 확신도']),
            ('a3', 'a4', ['예 · 대상 복장인 사람만']),
            ('a4', 'a5', ['사람마다 붙은 추적 번호 · 화면 속 위치']),
            ('a5', 'a6', ['그 사람 상의의 색 분포 (숫자 128개)']),
            ('a6', 'a7', ['흔들림을 줄인 색 지문']),
            ('a7', 'a8', ['예 · 안정된 추적 번호와 지문']),
            ('a8', 'a9', ['작업자 번호 · 복장 종류 · 색 지문'])]:
        flow(a, b, lab)      # 간격을 넓혔으므로 라벨을 화살표 위에 그대로 둔다
    flow('a3', 'x1', ['아니오 · 대상 아님'], 'right@0.62-left@0.5')
    flow('a7', 'x2', ['아니오 · 아직 불안정'], 'right@0.62-left@0.5')
    # 갈라진 쪽도 반드시 어딘가로 돌아가야 한다. 어디로 가는지 라벨을 붙인다 —
    # 없으면 그 상자에서 흐름이 끊긴 것처럼 보인다.
    # 이 프로그램은 0.1초마다 한 번씩 도는 구조라, 갈라진 쪽은 모두 '다음 화면' 으로 간다.
    flow('x1', 'a1', ['이 사람은 넘기고', '다음 화면을 읽는다'], 'top@0.5-right@0.5',
         via=[(CX + 510, 300), (CX + 210, 300),
              (CX + 210, d.nodes['a1']['y'] + 22)], label_x=CX + 360)
    # 라벨 x 를 손으로 주지 않는다 — 선은 통로(CX+430)에 있는데 라벨만 딴 데 떠 있었다.
    # 기본값이 경로 중간점이라 선 위에 붙는다.
    flow('x2', 'a1', ['0.1초 뒤 다음 화면부터', '다시 본다'], 'left@0.5-right@0.8',
         via=[(CX + 370, d.nodes['x2']['y'] + 20), (CX + 370, d.nodes['a1']['y'] + 26)])

    inner = [n for k, n in d.nodes.items() if k[0] in 'ax' and n['shape'] != 'band']
    l = min(n['x'] for n in inner) - 26
    r = max(n['x'] + n['w'] for n in inner) + 26
    d.band('nd1', l, 176, r - l, a9b - 176 + 22,
           'webcam_register        작업자를 등록하는 프로그램', 'nodebox', dash=True)
    d.band('pcw1', l - 20, 138, r - l + 40, a9b - 138 + 60,
           'PC-W1   웹캠1 PC', 'pc_cam', dash=False)
    flow('cam1', 'a1', ['영상 프레임', 'ROS 토픽이 아니다 — 노드가 직접 읽는다'],
         'right@0.4-left@0.5')

    # ═══════════ 오른쪽 — 받는 PC 셋 ═══════════
    QX, QW = r + 110, 760
    def consumer(tag, top, title, node, sub, steps, band_sty, box_sty, links=None):
        yy = top + 66
        ids = []
        # 상자 폭을 글자에 맞춘다 — 고정 폭이면 짧은 문장이 빈 자리를 크게 차지한다
        ws = [bw(ln) * (1.75 if ln[0].endswith('?') else 1.0) for ln in steps]
        colw = max(ws)
        ccx = QX + 60 + colw / 2
        for k, ln in enumerate(steps):
            nid = '%s%d' % (tag, k)
            shp = 'diamond' if ln[0].endswith('?') else 'rect'
            sty = 'dec' if shp == 'diamond' else box_sty
            d.box(nid, ccx - ws[k] / 2, yy, ws[k], ln, sty,
                  h=62 if shp == 'diamond' else None, shape=shp, align='center')
            ids.append(nid)
            yy += d.nodes[nid]['h'] + 56
        for k, (a, b) in enumerate(zip(ids, ids[1:])):
            flow(a, b, links[k] if links and k < len(links) else None)
        d.band(tag + 'nd', QX + 30, top + 40, colw + 60, yy - 56 - top - 40 + 22,
               node + '        ' + sub, 'nodebox', dash=True)
        d.band(tag + 'pc', QX, top, colw + 620, yy - 56 - top + 60, title, band_sty, dash=False)
        return ids, yy - 56 + 60

    w2, y2 = consumer('b', 150, 'PC-W2   웹캠2 PC', 'webcam_hint', '재탐지 힌트를 내는 프로그램',
                      [['작업자 번호에 복장 종류를 짝지어 표에 적어 둔다'],
                       ['이 표가 있어야 화면에서 본 사람이', '누구인지 가릴 수 있다'],
                       ['이후 그 사람이 어디 있는지', '방향 힌트를 로봇에 보낸다']], 'pc_cam', 'in_cam',
                      links=[['작업자 번호 ↔ 복장 표'],
                             ['화면에서 가려낸 그 사람의 위치']])
    f2, y3 = consumer('c', y2 + 40, 'PC-F   관제 PC', 'fleet_manager', '작업자에게 로봇을 붙이는 프로그램',
                      [['이미 로봇이 붙은 작업자인가?'],
                       ['쓸 수 있는 로봇이 있는가?'],
                       ['대기 명단에 올린다  (등록된 시각과 함께)'],
                       ['명단이 로봇 수만큼 찼는가?'],
                       ['작업자 번호가 빠른 순서로', '로봇을 한 대씩 붙인다'],
                       ['누가 어느 로봇을 맡았는지 기록에 남긴다'],
                       ['배정 결과를 그 로봇에 알린다']], 'pc_ctl', 'in_ctl',
                      links=[['예 · 아직 로봇이 없는 작업자'],
                             ['예 · 비어 있는 로봇 목록'],
                             ['작업자 정보와 등록된 시각'],
                             ['예 · 한꺼번에 처리할 작업자들'],
                             ['작업자 ↔ 로봇 짝'],
                             ['누가 어느 로봇을 맡았는지']])
    r2, y4 = consumer('e', y3 + 96, 'PC-RN   로봇 제어 PC   (로봇마다 한 벌)', 'amr_detection',
                      '작업자를 알아보는 프로그램',
                      [['색 지문이 비어 있지는 않은가?'],
                       ['작업자 번호마다 복장과 색 지문을 기억해 둔다'],
                       ['내가 맡은 작업자인가?'],
                       ['그 지문을 짝짓기 기준값으로 건다'],
                       ['지문이 새로 오면 기준값도 따라 바꾼다']], 'pc_r3', 'in_r3',
                      links=[['예 · 쓸 수 있는 색 지문'],
                             ['작업자별 복장과 색 지문'],
                             ['예 · 내 담당 작업자의 지문'],
                             ['짝짓기 기준값']])

    # 판단에서 갈라지는 쪽을 적는다 — 무엇을 하고 끝나는지가 보여야 재현할 수 있다
    # 갈라지는 쪽도 끝을 밝힌다. 끝나는 것인지, 어디로 이어지는 것인지 적지 않으면
    # 흐름이 그 상자에서 끊긴 것처럼 보인다.
    for did, txt, lab, back in (
            (f2[0], ['그냥 넘어간다', '이번 소식은 여기서 끝'], ['아니오 · 이미 짝이 있는 작업자'], None),
            (f2[1], ['배정하지 않고 기록만 남긴다', '이번 소식은 여기서 끝'],
             ['아니오 · 남는 로봇이 없다'], None),
            (f2[3], ['잠깐 기다렸다 처리한다'], ['아직 · 더 올 수 있다'], f2[4]),
            (r2[0], ['버린다', '이번 소식은 여기서 끝'], ['아니오 · 지문이 비었다'], None),
            (r2[2], ['기억만 하고 끝낸다', '남의 작업자는 짝짓지 않는다'],
             ['아니오 · 내 담당이 아니다'], None)):
        n = d.nodes[did]
        eid = did + '_x'
        d.box(eid, n['x'] + n['w'] + 210, n['y'] + 4, 250, txt, 'exc', align='center')
        d.edge(did, eid, '', 'right@0.5-left@0.5', color=PLAIN, width=1.4, label_lines=lab)
        if back:                       # 끝나지 않고 이어지는 갈래는 되돌아가는 선을 그린다
            bn = d.nodes[back]
            d.edge(eid, back, '', 'bottom@0.5-right@0.5', color=PLAIN, width=1.4,
                   label_lines=['정한 시간이 지나면', '모인 작업자들로'],
                   via=[(d.nodes[eid]['x'] + 125, bn['y'] + 20)],
                   label_x=d.nodes[eid]['x'] + 250)

    # 같은 토픽이 셋으로 가는 것이지 세 개의 토픽이 아니다.
    # 한 줄기로 내보내고 갈림점에서 셋으로 나눈다.
    HX = (r + 20 + QX) / 2
    d.box('hub', HX - 9, d.nodes['a9']['y'] + d.nodes['a9']['h'] / 2 - 9, 18,
          [], 'tp_t', h=18, align='center')
    d.edge('a9', 'hub', '', 'right@0.5-left@0.5', color=TOPIC, width=2.2,
           label_lines=[WD, '토픽 · %s' % wd_ty, '1초마다 다시 보낸다'],
           label_x=r - 150)      # 통로가 좁으므로 PC-W1 오른쪽 빈 자리에 놓는다
    for tgt in (w2[0], f2[0], r2[0]):
        # 세로로 갈라지는 자리를 통로 안에 둔다 — 기본값이면 PC 칸 안으로 들어간다
        d.edge('hub', tgt, '', 'right@0.5-left@0.5', color=TOPIC, width=2.0, mid=QX - 22)
    d.edge(f2[-1], r2[0], '', 'bottom@0.5-top@0.5', color=TOPIC, width=2.0,
           label_lines=['mission/assigned', '토픽 · %s' % MA_ty])

    # ── 읽는 법 ──
    # 폭도 내용에 맞춰 자른다 — autosize 는 높이만 줄인다
    d.w = int(max(max(n['x'] + n['w'] for n in d.nodes.values()),
                  max(t['x'] + d.text_w(t['s'], t['bold']) for t in d.free)) + 30)

    ly = max(a9b, y4) + 54
    d.text(30, ly, '읽는 법', 13, '#333840', bold=True)
    d.text(30, ly + 24, '타원 = 시작   ·   네모 = 하는 일   ·   마름모 = 판단   ·   빨간 네모 = 그때의 처리', 11, '#5A5F6A')
    d.text(30, ly + 44, '상자에는 무슨 일을 하는지만 적는다. 함수 이름과 변수 이름은 쓰지 않는다.', 11, '#5A5F6A')
    d.text(30, ly + 64, '파란 화살표가 PC 를 건너는 ROS 토픽이다 — 토픽 이름과 메시지 타입을 적는다.', 11, '#5A5F6A')
    d.text(30, ly + 84, '사양은 외부 장비 상자에만 적는다. 장비는 화살표로 설명할 상대가 없기 때문이다.', 11, '#5A5F6A')
    return d.autosize()

# =====================================================================
def main():
    diagrams = [('step5_graph_w1', step5_graph_w1())]
    pages = []
    for name, d in diagrams:
        p = os.path.join(OUT, name + '.svg')
        with open(p, 'w', encoding='utf-8') as f:
            f.write(d.to_svg())
        d.check()
        print('wrote', p, '(%d×%d)' % (d.w, d.h))
        pages.append(d.to_drawio_page(name))
    dio = os.path.join(OUT, 'mro_design_diagrams.drawio')
    xml = '<mxfile host="app.diagrams.net" type="device">%s</mxfile>' % ''.join(pages)
    # draw.io 가 열 수 있는지 쓰기 전에 확인한다 — 깨진 파일을 내보내지 않는다.
    import xml.etree.ElementTree as ET
    ET.fromstring(xml)
    with open(dio, 'w', encoding='utf-8') as f:
        f.write(xml)
    print('wrote', dio, '(XML 검증 통과)')


if __name__ == '__main__':
    main()
