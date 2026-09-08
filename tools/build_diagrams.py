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
    'pc_r4':  ('#F4EFFB', '#7C8794', '#4B357A'),   # 로봇4 제어 PC
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
        s = ('<rect x="%g" y="%g" width="%g" height="%g" rx="3" fill="#FFFFFF" '
             'fill-opacity="0.97" stroke="%s" stroke-width="0.8"/>' % (bx, by, bw, bh, e['color']))
        for i, t in enumerate(e['lines']):
            s += ('<text x="%g" y="%g" font-size="10.5" fill="%s"%s>%s</text>'
                  % (bx + 7, by + 15 + 13 * i,
                     e['color'] if i == 0 else '#3A3E46',
                     ' font-weight="600"' if i == 0 else '', esc(t)))
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
        for kind, k in self.order:
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
            vtop = not (n['align'] == 'center' or n['shape'] != 'rect')
            style = ('%swhiteSpace=nowrap;html=1;fillColor=%s;strokeColor=%s;%s'
                     'verticalAlign=%s;spacing=0;spacingTop=%g;spacingBottom=0;'
                     'fontFamily=%s;fontSize=11;%s'
                     % (shape, fill, stroke, align,
                        'top' if vtop else 'middle',
                        7 if vtop else 0,   # SVG 첫 줄 글자 윗변(y+12)에 맞춘 값
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
def step1():
    d = Diagram('STEP1 배치도', 1280, 790,
                'STEP 1 — 시스템 배치도 (PC 5대 · AMR 2대 · 네트워크)',
                'MRO Multi-AMR Phase 1 · 2026-09-05 구성 확정 및 검증 완료 · SDD 3.1')

    d.band('wifi', 24, 70, 1232, 34,
           '무선망 SSID turtle08   ·   ROS_DOMAIN_ID = 3   ·   RMW = rmw_fastrtps_cpp   '
           '·   전송 = Fast DDS Discovery Server 방식 (멀티캐스트 미사용)')

    px = [24, 273, 522, 771, 1020]
    pw = 236

    d.box('w1', px[0], 126, pw, [
        'PC-W1 — 웹캠1 PC',
        'Ubuntu 24.04 · ROS 2 Jazzy',
        'pkg  mro_webcam',
        '• webcam_register',
        '   작업자 등록 · 외형 서술자 발행',
        'USB 웹캠 640×480 (출입구 시야)',
        'ROS_SUPER_CLIENT = False',
        '담당  한세교 · 김학민',
    ], 'pc')

    d.box('w2', px[1], 126, pw, [
        'PC-W2 — 웹캠2 PC',
        'Ubuntu 24.04 · ROS 2 Jazzy',
        'pkg  mro_webcam',
        '• webcam_hint',
        '   재탐지 방위 힌트 · KPI 계측',
        'USB 웹캠 640×480 (작업공간 남단)',
        'ROS_SUPER_CLIENT = False',
        '담당  한세교 · 김학민',
    ], 'pc')

    d.box('f', px[2], 126, pw, [
        'PC-F — 관제 PC',
        'Ubuntu 24.04 · Jazzy · GPU 불필요',
        'IP 192.168.108.41/24 (고정)',
        '★ Fast DDS Discovery Server',
        '    서버 ID 1 · UDP 11811',
        'pkg  mro_fleet',
        '• fleet_manager   • zone_semaphore',
        '• control_ui (Flask :5000)',
        '• kpi_logger · SQLite',
        '담당  김민준',
    ], 'pcf')

    d.box('r3', px[3], 126, pw, [
        'PC-R3 — 로봇3 제어 PC',
        'Ubuntu 24.04 · Jazzy · NVIDIA GPU',
        'Nav2 + AMCL + RViz2',
        'pkg  mro_perception',
        '• amr_detection  • aruco_detector',
        'pkg  mro_navigation',
        '• shuttle_mission  • follow_target',
        '• return_dock  • safety_watchdog',
        'namespace = /robot3',
    ], 'pc')

    d.box('r4', px[4], 126, pw, [
        'PC-R4 — 로봇4 제어 PC',
        'Ubuntu 24.04 · Jazzy · NVIDIA GPU',
        'Nav2 + AMCL + RViz2',
        'pkg  mro_perception',
        '• amr_detection  • aruco_detector',
        'pkg  mro_navigation',
        '• shuttle_mission  • follow_target',
        '• return_dock  • safety_watchdog',
        'namespace = /robot4',
    ], 'pc')

    # 디스커버리 버스
    d.box('bus', 24, 402, 1232, [
        'ROS_DISCOVERY_SERVER = ";192.168.108.41:11811"      ← 앞의 세미콜론 하나가 "서버 ID 1" 을 뜻한다. 5대 PC + 로봇 2대가 모두 같은 값을 쓴다.',
        '이 경로로 오가는 것은 참가자·토픽 탐색 정보뿐이다. 실제 토픽 데이터는 발행 노드와 구독 노드 사이를 직접(P2P) 흐르므로 PC-F 를 거치지 않는다.',
    ], 'pcf', align='left')

    for k in ('w1', 'w2', 'f', 'r3', 'r4'):
        d.edge(k, 'bus', '', 'bottom-top', color='#B06810', width=1.6, arrow=False)

    d.box('rb3', 771, 512, 236, [
        'robot3 — TurtleBot4 Standard',
        'Raspberry Pi 4B · Ubuntu 24.04',
        'turtlebot4_bringup · _node',
        'depthai_ros_driver → OAK-D Pro',
        'rplidar_ros → RPLIDAR A1',
        'iRobot Create3 (범퍼·클리프·도크)',
        'onboard Discovery ID 3 — 남겨두되',
        'PC 의 접속 목록에는 넣지 않는다',
        'Offboard IP → 192.168.108.41',
    ], 'robot')

    d.box('rb4', 1020, 512, 236, [
        'robot4 — TurtleBot4 Standard',
        'Raspberry Pi 4B · Ubuntu 24.04',
        'turtlebot4_bringup · _node',
        'depthai_ros_driver → OAK-D Pro',
        'rplidar_ros → RPLIDAR A1',
        'iRobot Create3 (범퍼·클리프·도크)',
        'onboard Discovery ID 4 — 남겨두되',
        'PC 의 접속 목록에는 넣지 않는다',
        'Offboard IP → 192.168.108.41',
    ], 'robot')

    d.edge('rb3', 'bus', '', 'top-bottom', color='#B06810', width=1.6, arrow=False)
    d.edge('rb4', 'bus', '', 'top-bottom', color='#B06810', width=1.6, arrow=False)

    d.box('legend', 24, 512, 723, [
        '읽는 법 · 확인 상태',
        '─── 주황 실선 = 디스커버리 등록 경로. 모든 참가자가 PC-F 의 서버 하나에만 등록한다.',
        '      토픽 데이터 경로는 P2P 라서 이 그림에 선으로 그리지 않는다 (STEP 2 가 그 내용이다).',
        '',
        '[검증됨] 2026-09-05 — PC 5대와 로봇 2대를 모두 붙인 뒤 각 PC 에서 /robot3 · /robot4 토픽이',
        '      동시에 보이는 것을 확인했다. 통신 구성은 이 상태로 확정한다 (SDD NR-01).',
        '',
        '이 구조를 택한 이유 — 종전에는 로봇의 라즈베리파이가 디스커버리 허브를 겸했다. PC 가 늘어날수록',
        '      RPi 가 병목이 되고, 로봇 2대면 PC 마다 서버 두 곳을 동시에 봐야 해서 설정이 갈라진다.',
        '      서버를 관제 노트북 한 대로 빼면 RPi 는 자기 센서 처리에만 쓰이고, 모든 PC 의 설정이 한 줄로 같아진다.',
        '',
        '웹캠 PC 와 nav2 터미널은 ROS_SUPER_CLIENT = False 로 둔다. 전체 토픽 목록이 필요한 도구',
        '      (ros2 topic list, RViz 탐색)를 쓸 때만 True 가 필요하고, 상시 True 는 불필요한 트래픽을 만든다.',
        '',
        '고정 IP 192.168.108.41 은 공유기 DHCP 예약이 아직 걸려 있지 않다 → 강사님께 예약 요청 필요.',
    ], 'note')

    d.text(24, d.nodes['legend']['y'] + d.nodes['legend']['h'] + 22,
           '생성: tools/build_diagrams.py   ·   원본 근거: docs/setup.md, docs/architecture.md, SDD 2.3 NR-01',
           10, '#8A8F98')
    return d.autosize()


# =====================================================================
# STEP 2 — 로봇 내부 · Nav2 교환
# =====================================================================
def step2():
    d = Diagram('STEP2 Nav2 스택', 1340, 900,
                'STEP 2 — 로봇 1대(/robotN) 내부 구성과 Nav2 스택과의 교환',
                'robot3 · robot4 는 네임스페이스만 다르고 구성은 같다 · SDD 3.2')

    AX, AW, BX, BW, CX, CW, DX, DW = 24, 300, 356, 350, 748, 340, 1112, 200

    def stack(x, w, y0, items, gap=30):
        y = y0
        for nid, lines, st in items:
            n = d.box(nid, x, y, w, lines, st)
            y = n['y'] + n['h'] + gap
        return y

    # --- A. 센서 · 외부 입력 ---
    stack(AX + 16, AW - 32, 110, [
        ('oak', ['OAK-D Pro  ·  depthai_ros_driver',
                 'oakd/rgb/image_raw/compressed',
                 'oakd/stereo/…/compressedDepth',
                 'oakd/rgb/camera_info',
                 '704×704 @10 Hz · 깊이 정렬 · subpixel'], 'hw'),
        ('lidar', ['RPLIDAR A1  ·  rplidar_ros',
                   'scan  (LaserScan, 10 Hz)'], 'hw'),
        ('create', ['iRobot Create3',
                    'odom · tf · battery_state',
                    '범퍼 · 클리프 (하드웨어 안전)'], 'hw'),
    ])
    d.box('ext', AX + 16, 416, AW - 32, [
        '외부 입력 (PC-F · 웹캠 PC 에서 옴)',
        'mission/assigned      S4 셔틀 트리거',
        'mission/finish          S9 종료 신호',
        '/webcam1/worker_descriptor',
        '/webcam2/target_hint   재탐지 힌트',
        '/fleet/zone_token   서비스 · P1 · DOCK',
    ], 'ext')

    # --- B. 우리 노드 ---
    stack(BX + 16, BW - 32, 110, [
        ('aruco', ['aruco_detector   (mro_perception)',
                   'DICT_4X4_50 · 베이 ID 1/2/3 판독',
                   'aruco/id · aruco/distance'], 'ours'),
        ('shuttle', ['shuttle_mission   (mro_navigation)',
                     'S2 undock · S4 베이 왕복 · Z-05 대기공간',
                     'shuttle/status · shuttle/arrived'], 'ours'),
        ('det', ['amr_detection   (mro_perception)',
                 'YOLO11n worker.pt + HSV 재식별 게이트',
                 'target/map_point · target/visible',
                 'target/paired · target/worker_id',
                 'target/distance  ← 추종거리 판정의 기준값'], 'ours'),
        ('follow', ['follow_target   (mro_navigation)',
                    'S5 합류 · S6~S8 추종 (간격 0.5 m)',
                    'Lost → 힌트 방향 탐색 → STANDBY',
                    'approach/status'], 'ours'),
        ('ret', ['return_dock   (mro_navigation)',
                 'S10 복귀 · DOCK 토큰 · 도킹 재시도 2회'], 'ours'),
        ('safe', ['safety_watchdog   (mro_navigation)',
                  'scan 만 구독 · 전방 ±35°',
                  '정지 0.30 m / 해제 0.45 m',
                  'safety/estop · cmd_vel = 0 @20 Hz',
                  '!YOLO · 웹캠 · Wi-Fi 에 의존하지 않는다'], 'safety'),
    ])

    # --- C. Nav2 ---
    stack(CX + 16, CW - 32, 110, [
        ('bt', ['bt_navigator',
                'action  navigate_to_pose',
                '동작 트리가 계획 · 제어 · 복구를 조립한다'], 'nav2'),
        ('planner', ['planner_server  (Smac / NavFn)',
                     '전역 경로 생성 (정적 맵 + 전역 costmap)'], 'nav2'),
        ('ctrl', ['controller_server  (RPP / DWB)',
                  '지역 경로 추종 → 속도 명령 산출'], 'nav2'),
        ('behav', ['behavior_server',
                   'spin · backup · wait  (복구 동작)'], 'nav2'),
        ('cost', ['global / local costmap',
                  '입력: 정적 맵 + scan',
                  'inflation_radius 0.25 — 받침대 덮기 (I-009)'], 'nav2'),
        ('amcl', ['amcl  +  map_server',
                  '입력: scan + odom + 정적 맵',
                  '출력: amcl_pose · map→odom 변환'], 'nav2'),
        ('dock', ['opennav_docking',
                  'action  dock / undock  (Create3 IR 도크)'], 'nav2'),
    ])

    # --- D. 구동 ---
    d.box('smooth', DX + 16, 289, DW - 32, [
        'velocity_smoother',
        'cmd_vel (TwistStamped)',
    ], 'nav2')
    d.box('wheel', DX + 16, 381, DW - 32, [
        'Create3 구동부',
        'v ≤ 0.3 m/s',
    ], 'hw')

    bot = max(d.nodes[k]['y'] + d.nodes[k]['h'] for k in
              ('ext', 'safe', 'dock', 'wheel')) + 46
    d.band('bA', AX, 74, AW, bot - 74, 'A. 센서 · 구동 (로봇 온보드) 및 외부 입력')
    d.band('bB', BX, 74, BW, bot - 74, 'B. 우리가 만드는 노드 (PC-RN 에서 실행)')
    d.band('bC', CX, 74, CW, bot - 74, 'C. Nav2 스택 (turtlebot4_navigation)')
    d.band('bD', DX, 74, DW, bot - 74, 'D. 구동 출력')
    # 밴드가 상자를 덮지 않도록 맨 뒤로
    d.order.sort(key=lambda t: 0 if t[0] == 'band' else 1)

    B, P, R, G = '#3576C8', '#6B4FA8', '#C0392B', '#5A5F6A'
    d.edge('oak', 'det', '①', 'right-left', color=B, mid=334)
    d.edge('oak', 'aruco', '②', 'right-left', color=B, mid=342)
    d.edge('lidar', 'safe', '③', 'right-left', color=R, mid=344)
    d.edge('ext', 'shuttle', '④', 'right-left', color=G, mid=326)
    d.edge('ext', 'follow', '⑤', 'right-left', color=G, mid=330)
    d.edge('ext', 'ret', '⑥', 'right-left', color=G, mid=338)
    d.edge('det', 'follow', '⑦', 'bottom-top', color=B)
    d.edge('aruco', 'shuttle', '⑧', 'bottom-top', color=B)
    d.edge('shuttle', 'bt', '⑨', 'right-left', color=P, mid=714)
    d.edge('follow', 'bt', '⑩', 'right-left', color=P, mid=722)
    d.edge('ret', 'bt', '⑪', 'right-left', color=P, mid=730)
    d.edge('shuttle', 'dock', '⑫', 'right-left', color=P, mid=738)
    d.edge('ret', 'dock', '⑬', 'right-left', color=P, mid=744)
    d.edge('amcl', 'det', '⑭', 'left-right', color=P, mid=710)
    d.edge('ctrl', 'smooth', '⑮', 'right-left', color=P, mid=1100)
    d.edge('smooth', 'wheel', '⑯', 'bottom-top', color=P)

    sy = d.nodes['safe']['y'] + d.nodes['safe']['h'] / 2
    wx = d.nodes['wheel']['x'] + d.nodes['wheel']['w'] / 2
    d.edge('safe', 'wheel', '⑰', 'right-bottom', color=R, width=2.2,
           via=[(BX + BW + 24, sy), (BX + BW + 24, bot - 26), (wx, bot - 26)])
    fy = d.nodes['follow']['y'] + d.nodes['follow']['h'] / 2
    d.edge('safe', 'follow', '⑱', 'left-left', color=R,
           via=[(BX - 8, sy), (BX - 8, fy)])

    d.box('lg', 24, bot + 16, 1292, [
        '교환 지점 — 번호별 내용',
        '①  rgb + depth + camera_info 를 message_filters 로 동기화해 사람 검출과 거리 산출에 쓴다.      ②  rgb 만 받아 ArUco 를 읽는다.',
        '③  scan 은 안전 감시가 직접 구독한다. 인지·웹캠·관제 어느 것도 이 경로에 끼지 않는다.',
        '④⑤⑥  관제·웹캠에서 오는 트리거. mission/assigned → 셔틀 시작 · /webcam2/target_hint → 재탐지 방향 · mission/finish → 추종 즉시 중단 및 복귀 시작.',
        '⑦  target/paired 가 참일 때만 추종이 시작된다 (재식별 게이트).      ⑧  베이 도착 판정의 근거.',
        '⑨⑩⑪  우리 노드가 Nav2 에 붙는 주행 인터페이스는 navigate_to_pose 액션 하나뿐이다. 경로 계획과 지역 제어는 Nav2 에 맡긴다.',
        '⑫⑬  도킹도 opennav_docking 액션에 맡긴다. 우리가 정하는 것은 언제 부를지와 실패 시 몇 번 다시 부를지뿐이다.',
        '⑭  amcl_pose 로 카메라가 본 대상을 map 프레임 좌표로 옮긴다. 두 로봇이 같은 맵 원점을 쓰기 때문에 이 값이 서로 통한다.',
        '⑮⑯  Nav2 가 낸 속도 명령이 평활화를 거쳐 Create3 로 간다.',
        '⑰  안전 감시는 정지 조건에서 cmd_vel 에 0 을 20 Hz 로 덮어써 Nav2 출력을 이긴다.      ⑱  safety/estop 을 받은 우리 노드는 새 주행 명령을 내지 않는다.',
        '',
        '설계 원칙 — 안전은 온보드(scan + costmap)에서만 성립시킨다. 웹캠 · 재식별 · 관제는 "더 잘 하게" 할 뿐 "안 부딪히게" 하는 일에는 관여하지 않는다.',
        '           그래야 Wi-Fi 가 끊기거나 웹캠이 죽어도 무사고가 유지된다. (AGENTS.md 안전 계층 원칙)',
    ], 'note')

    d.text(24, d.nodes['lg']['y'] + d.nodes['lg']['h'] + 20,
           '생성: tools/build_diagrams.py   ·   원본 근거: docs/architecture.md, docs/interfaces.md 4절, SDD 3.2',
           10, '#8A8F98')
    return d.autosize()


# =====================================================================
# STEP 3 — 시나리오 플로우차트
# =====================================================================
def step3():
    """STEP 3-1 시나리오 순서도 — 3-2 와 같은 밀도.

    상자마다 그 단계를 누가 · 어떤 토픽으로 하는지 3~5줄, 화살표마다 토픽·서비스·액션
    이름과 메시지 타입, 예외는 오른쪽에 내용까지. 흐름이 길어 두 단으로 접고 ① 로 잇는다.
    """
    W = 2360
    d = Diagram('STEP3 시나리오', W, 100,
                'STEP 3-1 — 운영 시나리오 S1 ~ S10 순서도',
                '단계 상자 = 채움색이 수행 PC · 화살표 색 = 통신 종류 · 오른쪽 = 그때의 예외 처리 · SDD 3.3 / BRD 3.2')

    TOPIC, SRV, ACT, SAFE, PLAIN = '#3576C8', '#B06810', '#6B4FA8', '#C0392B', '#5A5F6A'
    FW, CORR, EW = 400, 190, 520
    A_X = 40
    B_X = 1210
    TOP = 118
    GAP = 58                      # 단계 사이 — 두 줄짜리 화살표 라벨이 들어갈 자리

    def ylab_in(gy0, gh, nlines=1):
        bh = 13 * nlines + 8
        return gy0 + (gh - bh) / 2 + bh + 5

    def column(fx0, y, steps):
        """steps 를 위에서 아래로 놓고, 사이를 라벨 달린 화살표로 잇는다."""
        ex0 = fx0 + FW + CORR
        prev = None
        for st in steps:
            shape = st.get('shape', 'rect')
            h = st.get('h') or (76 if shape == 'diamond' else None)
            d.box(st['id'], fx0, y, FW, st['lines'], st['style'], h=h, shape=shape,
                  align='center' if shape != 'rect' else 'left')
            hh = d.nodes[st['id']]['h']
            if prev is not None:
                gy0, gh = prev[1], y - prev[1]
                d.edge(prev[0], st['id'], '', 'bottom@0.5-top@0.5',
                       color=st.get('incolor', TOPIC), width=1.8,
                       dashed=st.get('indash', False),
                       via=[(fx0 + FW / 2, ylab_in(gy0, gh, len(st['in'])) - 1),
                            (fx0 + FW / 2, ylab_in(gy0, gh, len(st['in'])) + 1)],
                       label_lines=st['in'], label_x=fx0 + FW / 2)
            if st.get('exc'):
                eid = st['id'] + '_e'
                eh = box_h(st['exc'])
                d.box(eid, ex0, y + hh / 2 - eh / 2, EW, st['exc'], 'exc')
                d.edge(st['id'], eid, '', 'right@0.5-left@0.5',
                       color=st.get('exccolor', SAFE), width=1.8,
                       label_lines=st['exclabel'], label_x=fx0 + FW + CORR / 2)
            prev = (st['id'], y + hh)
            y = y + hh + GAP
        return y - GAP

    # ═════════ 1단 · S1 ~ S4′ ═════════
    d.text(A_X, TOP - 22, '1단  —  진입 · 등록에서 대기공간 도착까지', 13, '#333840', bold=True)
    a_bot = column(A_X, TOP, [
        dict(id='s1', style='st_cam', lines=[
            'S1   진입 · 등록',
            'webcam_register (PC-W1) · 출입구 시야 · 추론 10 Hz',
            'ByteTrack ID 부여 · 0.3 s 연속으로 잡혀야 등록',
            'HSV H16×S8 = 128 · 10프레임 평균 · 재등록 임계 0.45',
            '등록 ≤ 10 초 → 로봇 배정 (트랙 ID 작은 쪽 먼저)',
        ], exc=[
            '두 작업자의 서술자가 너무 비슷할 때',
            '같은 인형끼리의 유사도와 다른 인형끼리의 유사도 분포가 겹치면',
            '서술자 자체를 손봐야 한다는 신호다. 조명을 맞추고 다시 등록한다.',
            '상의 색 두 가지는 휘도를 맞춰 인쇄해 색만으로 갈리게 한다.',
        ], exclabel=['분포가 겹친다']),
        dict(id='s2', style='st_rob', lines=[
            'S2   준비 — 도크에서 나온다',
            'shuttle_mission (PC-R3 / PC-R4)',
            'dock_pose 로 AMCL 초기 위치 설정 — I-013',
            'opennav_docking   action  undock',
            'shuttle/status = READY → SHUTTLE',
        ], **{'in': ['/webcam1/worker_descriptor',
                     'WorkerDescriptor → fleet_manager 가 로봇을 배정한다']},
            exc=[
            'I-013   두 로봇의 맵 좌표계가 어긋나는 사고',
            '각 로봇이 (0,0) 으로 초기화하면 같은 좌표가 서로 다른 장소를',
            '가리키게 되어 세마포어도 waypoint 도 전부 무의미해진다.',
            '맵 원점은 SLAM 작성 시점 하나로 고정하고, 각 로봇은',
            'arena.yaml 의 dock_zone.pose 값으로 초기화한다.',
        ], exclabel=['(0,0) 으로 초기화하면']),
        dict(id='s3', style='st_ctl', lines=[
            'S3   자재 요청',
            'control_ui (Flask :5000) 에서 작업자 · 베이 선택',
            'RequestMission (서비스) → fleet_manager',
            '배터리 25 % 미만이면 거부하고 사유를 남긴다',
            '접수 ≤ 2 초 · missions.created_at 기록 (K-05 시작점)',
        ], **{'in': ['shuttle/status = READY', 'std_msgs/String → fleet_manager · 배차 가능']}),
        dict(id='s4', style='st_rob', lines=[
            'S4   셔틀 주행 — 창고 베이로',
            'shuttle_mission → Nav2   navigate_to_pose (액션)',
            'arena.yaml 의 베이 A / B / C waypoint',
            '도크와 창고는 벽 없이 이어져 여기서는 토큰이 없다',
            'shuttle/status = SHUTTLE → AT_BAY',
        ], **{'in': ['mission/assigned', 'mro_interfaces/Mission (ASSIGNED) → 배정된 로봇']}),
        dict(id='q1', style='dec', shape='diamond', lines=[
            '베이 ArUco ID 가 일치하는가?',
            'aruco_detector · 거리 ≤ 1.5 m',
        ], **{'in': ['aruco/id · aruco/distance',
                     'Int32 · Float32 — 1.5 m 안에서만 나온다 (PR #30)']},
            exc=[
            '도착 판정 실패 — 오도착',
            '다시 접근해 읽는다. T_lim 120 초를 넘기면 미션을 FAILED 로',
            '확정하고 운영자에게 알린다. 늦게라도 도착한 사실은',
            'arrived_late 로 따로 남겨 성공으로 이중 계상하지 않는다.',
        ], exclabel=['N  ·  ID 불일치 또는 -1']),
        dict(id='q2', style='dec', shape='diamond', lines=[
            'P1 병목 토큰을 얻었는가?',
            'zone_semaphore · 한 번에 한 대',
        ], **{'in': ['/fleet/zone_token   acquire',
                     'mro_interfaces/ZoneToken (서비스) → zone_semaphore']},
            incolor=SRV, indash=True,
            exc=[
            'E-3   교착 회피',
            '거절되면 2 초 뒤 다시 요청한다 (REJECT 는 대기 시작 1건만).',
            '폴리곤에 들어가기 전에 얻고 완전히 빠져나온 뒤 반납한다.',
            '60 초를 넘겨 쥐고 있으면 세마포어가 강제 회수(RECLAIM)한다.',
            'S4 창고에서 나올 때와 S10 돌아갈 때 두 번 통과한다.',
        ], exclabel=['N  ·  granted = false', '2 s 후 재요청'], exccolor=SRV),
        dict(id='s5', style='st_rob', lines=[
            'S4′   적재 · 대기공간 이동',
            '적재 5 초 체류 → P1 통과 → 대기공간 (Z-05)',
            '도착 후 P1 토큰 반납 — 못 받았어도 반드시 반납 (I-033)',
            'shuttle/arrived (Bool) · shuttle/status = WAITING',
            'WAITING 이 AC-08 · K-05 판정 기준점 — 미션당 1회',
        ], **{'in': ['granted = true', '베이를 떠나기 전에 얻는다 (방법 B, 순서 기반)']},
            incolor=SRV, indash=True),
    ])
    d.box('cx_a', A_X + FW / 2 - 21, a_bot + 26, 42, ['①'], 'term', h=42, shape='stadium',
          align='center')
    d.edge('s5', 'cx_a', '', 'bottom@0.5-top@0.5', color=TOPIC, width=1.8)

    # ═════════ 2단 · S5 ~ 도킹 ═════════
    d.text(B_X, TOP - 22, '2단  —  합류에서 도킹 완료까지', 13, '#333840', bold=True)
    d.box('cx_b', B_X + FW / 2 - 21, TOP - 68, 42, ['①'], 'term', h=42, shape='stadium',
          align='center')
    b_bot = column(B_X, TOP, [
        dict(id='q3', style='dec', shape='diamond', lines=[
            '재식별 게이트를 통과했는가?',
            'amr_detection · 유사도 ≥ 0.6',
        ], exc=[
            'E-2   엉뚱한 사람을 따라가지 않기',
            '임계에 못 미치면 페어링하지 않고 그 자리에서 대기한다.',
            '잘못된 작업자를 따라가는 일은 0 건이 절대 기준이라,',
            '확신이 없을 때 멈추는 쪽이 항상 옳다.',
            '폴백 — 관제 UI 에서 운영자가 대상을 수동으로 지정한다.',
        ], exclabel=['N  ·  임계 미달']),
        dict(id='s6', style='st_rob', lines=[
            'S5   합류 — 배정된 작업자에게 접근',
            'follow_target → Nav2   navigate_to_pose (액션)',
            'APPROACH_1 → APPROACH_2 → ARRIVED',
            'approach/status (String) 로 관제에 보고한다',
        ], **{'in': ['target/paired · target/worker_id',
                     'Bool · String → follow_target 이 추종을 시작한다']}),
        dict(id='s7', style='st_rob', lines=[
            'S6 ~ S8   추종',
            '중앙 복도(P3 0.93 m)를 지나 작업공간까지 따라간다',
            'standoff 0.5 m 유지 · v ≤ 0.3 m/s',
            'approach/status = FOLLOWING · target/distance 발행',
            '추종 경로에는 토큰을 두지 않는다 — 기다리면 놓친다',
        ], **{'in': ['approach/status = ARRIVED', 'std_msgs/String — 합류 완료']}),
        dict(id='q4', style='dec', shape='diamond', lines=[
            '대상을 놓쳤는가?   (Target Lost)',
            'target/visible = false 가 1.0 s 이상',
        ], **{'in': ['target/distance · target/visible',
                     'Float32 · Bool — amr_detection 이 10 Hz 로 낸다']},
            exc=[
            'E-1   재탐지',
            '작업공간 안이면 웹캠2 힌트가 있다 → 그 방향으로 회전 탐색.',
            '중앙 복도는 웹캠2 시야 밖이라 힌트가 없다 → 제자리 탐색.',
            '두 조건의 재획득 시간을 비교하는 것이 시험 TS-19 다.',
            'T_lim 을 넘기면 STANDBY 로 들어가 운영자를 기다린다.',
        ], exclabel=['Y  ·  1.0 s 이상', '웹캠2 힌트로 방향']),
        dict(id='q5', style='dec', shape='diamond', lines=[
            '전방 0.30 m 안에 장애물이 있는가?',
            'safety_watchdog · scan · 전방 ±35°',
        ], **{'in': ['target/visible = true', '계속 추종한다']},
            exc=[
            'E-4   안전 정지',
            'safety/estop 을 올리고 cmd_vel 에 0 을 20 Hz 로 덮어쓴다.',
            '0.45 m 밖으로 벗어나면 스스로 재개한다.',
            '이 정지는 사람이 개입한 것이 아니므로 IPH 에 계상하지 않는다.',
            '웹캠이나 Wi-Fi 가 죽어도 이 경로는 온보드에서 살아 있다.',
        ], exclabel=['Y  ·  2프레임 연속', 'safety/estop = true'], exccolor=SAFE),
        dict(id='s8', style='st_ctl', lines=[
            'S9   작업 종료',
            '작업자가 종료 버튼 → control_ui → fleet_manager',
            'OperatorCommand   RETURN / RETURN_ALL',
            'mission/finish (Bool) — 추종 즉시 중단 (≤ 1 초)',
            '페어링 해제 · return_dock 이 복귀를 시작한다',
        ], **{'in': ['N  —  정상 주행 유지', 'estop 이 풀리면 Nav2 가 이어서 간다']},
            incolor=PLAIN),
        dict(id='q6', style='dec', shape='diamond', lines=[
            '복귀 통로와 도크 토큰을 얻었는가?',
            'P1 통과 → DOCK 은 한 번에 한 대',
        ], **{'in': ['mission/finish',
                     'std_msgs/Bool → follow_target 중단 · return_dock 시작']}),
        dict(id='s9', style='st_rob', lines=[
            'S10   도킹',
            '자기 도크 정면의 접근 지점을 거쳐 도킹한다',
            'opennav_docking   action  dock · T_ret 180 초',
            'shuttle/status = RETURNING → DOCK_WAIT → DOCKED',
        ], **{'in': ['/fleet/zone_token   acquire', 'ZoneToken (서비스) — P1 → DOCK 순서']},
            incolor=SRV, indash=True,
            exc=[
            'E-7   순차 도킹',
            '도크 두 대의 간격이 0.84 m(도킹점 기준)뿐이라 두 대가 동시에',
            '들어가면 IR 신호가 서로 간섭한다. DOCK 토큰으로 한 대씩',
            '들여보내고 나머지는 홀딩 포인트에서 기다린다.',
            '차폐판은 두지 않는다 — TS-U12 10회 오도킹 0 (9/6).',
        ], exclabel=['N  ·  다른 로봇이 보유', '홀딩 포인트 대기'], exccolor=SRV),
        dict(id='q7', style='dec', shape='diamond', lines=[
            '도킹에 성공했는가?',
            'dock_status · 재시도 2회까지',
        ], **{'in': ['action  dock', 'opennav_docking (액션) — 접근 지점에서 호출']},
            incolor=ACT,
            exc=[
            '도킹 실패',
            '최대 2회까지 다시 시도한다. 그래도 안 되면 FAILED 를 발행하고',
            '멈춘다. 성공이든 실패든 DOCK 토큰은 반드시 반납한다 —',
            '반납하지 않으면 다른 로봇이 영영 들어오지 못한다.',
        ], exclabel=['N  ·  2회 실패']),
        dict(id='end', style='term', shape='stadium', h=48, lines=[
            'DOCKED   ·   Phase 1 종료',
            '두 대 모두 도킹 완료 · KPI 로그 확정',
        ], **{'in': ['shuttle/status = DOCKED', 'Mission.DONE — 미션이 여기서 확정된다']}),
    ])

    # ═════════ 아래 설명 블록 ═════════
    def note(nid, px, pw, y0, title, lines):
        d.band(nid, px, y0, pw, 50 + len(lines) * 20 + 8, '', 'band', dash=True)
        d.text(px + 18, y0 + 26, title, 13, '#333840', bold=True)
        for i, ln in enumerate(lines):
            d.text(px + 18, y0 + 50 + i * 20, ln, 11, '#3A3E46')
        return y0 + 50 + len(lines) * 20 + 8

    ny = max(a_bot + 90, b_bot) + 54
    n1 = note('n1', A_X, FW + CORR + EW, ny, '전 구간 공통', [
        '· 모든 로그 · CSV · DB 는 ROS 시각을 쓴다 — 벽시계를 섞지 않는다 (계약 2절)',
        '· safety_watchdog 은 어느 단계에서든 항상 동작한다. estop 중에는 주행 명령을 내지 않는다',
        '· 안전 정지 뒤의 자동 재개는 사람의 개입이 아니므로 IPH(AC-13) 분자에 넣지 않는다',
        '· 토큰은 받았든 못 받았든 반드시 반납한다 — 안 하면 대기열에 남아 그 구역이 잠긴다 (I-033)',
        '· T_lim 120 초를 넘긴 미션은 폐기한다. 늦은 도착은 arrived_late 로 따로 남긴다 (계약 7절)',
        '· S1 ~ S10 은 로봇 두 대가 동시에 돈다. 두 대가 만나는 곳은 P1 과 DOCK 토큰뿐이다',
    ])
    n2 = note('n2', B_X, FW + CORR + EW, ny, '이 순서도가 정하는 KPI 기준점', [
        '· K-05 리드타임 — S3 의 created_at 에서 S4′ 의 WAITING 까지. 도킹 완료(DONE)와 다르다',
        '· AC-08 배송 성공 — S4′ 의 WAITING 이 T_lim 안이면 성공. 미션당 한 번만 센다',
        '· AC-05 추종거리 — S6~S8 의 FOLLOWING 구간만. Nav2 도착 오차 0.25 는 뺀다 (I-029)',
        '· K-04 재획득 — q4 의 Lost 1건을 성공 · 포기 · 미복귀 중 하나로만 센다 (I-028)',
        '· K-06 교착 — q2 · q6 의 REJECT → GRANT 가 60 초 안. 그 사이 RECLAIM 이면 미해결 (I-032)',
        '· AC-13 IPH — 개입 ÷ (undock ~ DOCKED). q5 의 안전 정지는 개입이 아니다',
    ])

    # ═════════ 범례 ═════════
    ly = max(n1, n2) + 44
    d.text(A_X, ly, '읽는 법', 13, '#333840', bold=True)
    seg = [('토픽  Topic', TOPIC, False, 1.8), ('서비스  Service', SRV, True, 1.8),
           ('액션  Action', ACT, False, 2.4), ('안전 경로', SAFE, False, 2.8)]
    lx = A_X
    for name, col, dash, wd in seg:
        d.edges.append(dict(a=None, b=None, label='', anchors='', dashed=dash, color=col,
                            ldx=0, ldy=0, width=wd, arrow=True, mid=None,
                            via=[(lx, ly + 22), (lx + 54, ly + 22)], lines=None,
                            lx_fix=None, lanchor='middle', raw=True))
        d.text(lx + 62, ly + 26, name, 11, '#3A3E46')
        lx += 62 + d.text_w(name, False) + 34
    for name, sty in (('웹캠 PC', 'st_cam'), ('관제 PC', 'st_ctl'),
                      ('로봇 제어 PC', 'st_rob'), ('판단', 'dec'), ('예외 처리', 'exc')):
        wd = d.text_w(name, True) + 28
        d.box('lg_' + sty + name[:2], lx, ly + 12, wd, [name], sty, h=26, align='center')
        lx += wd + 16
    d.text(A_X, ly + 58,
           '단계 상자의 채움색이 그 단계를 수행하는 PC 다. 화살표 위 상자는 첫 줄이 토픽 · 서비스 · 액션 이름, '
           '둘째 줄이 메시지 타입과 그 선이 하는 일이다.', 11, '#3A3E46')
    d.text(A_X, ly + 78,
           '오른쪽 빨간 상자는 그 판단에서 갈라지는 예외와 그때의 처리다. 판단의 나머지 한쪽(Y 또는 N)이 '
           '아래로 이어지는 정상 흐름이다.', 11, '#3A3E46')
    d.text(A_X, ly + 100,
           '생성: tools/build_diagrams.py   ·   근거: BRD v3.5 3.2 시나리오, SDD 3.3, docs/interfaces.md',
           10, '#8A8F98')
    return d.autosize()
# =====================================================================
def step3b():
    """STEP 3-2 전체 기능 흐름 — PC 패널 + 가운데 DDS 버스.

    · PC 는 채움색으로 구분한다 (테두리는 전부 회색이라 화살표 색과 섞이지 않는다)
    · 화살표 색·선모양이 통신 종류다 — 토픽 / 서비스 / 액션 / 안전
    · 모든 화살표에 토픽·서비스·액션 이름을 적는다
    · 버스를 가운데 가로로 두어 PC ↔ PC 화살표가 짧게 끝난다
    """
    W = 2620
    d = Diagram('STEP3-2 전체 흐름', W, 100,
                'STEP 3-2 — 전체 기능 흐름 (Functional Process Flow)',
                'PC = 채움색 · 화살표 색 = 통신 종류(토픽 / 서비스 / 액션 / 안전) · SDD 3.3 · 계약 v1.11')

    TOPIC, SRV, ACT, SAFE, PLAIN = '#3576C8', '#B06810', '#6B4FA8', '#C0392B', '#5A5F6A'
    # 로봇 네임스페이스는 여기 한 곳에서만 정한다. 기체가 바뀌면 이 두 줄만 고친다
    # (하드웨어 교체가 실제로 일어난다 — 9/8). 그림 곳곳에 이름을 흩어 놓지 않는다.
    NS_A, NS_B = '/robot3', '/robot4'
    L1X, L1W = 20, 860          # 로봇3 제어 PC
    CX, CW = 900, 820           # 관제 PC / 웹캠 PC
    L2X, L2W = 1740, 860        # 로봇4 제어 PC
    TOP = 96                    # 부제 아래에서 시작한다

    def ylab_in(gy0, gh, nlines=1):
        """틈(gy0 ~ gy0+gh) 한가운데에 라벨 상자가 오도록 하는 기준 y."""
        bh = 13 * nlines + 8
        return gy0 + (gh - bh) / 2 + bh + 5

    def fx(nid, ax):
        n = d.nodes[nid]
        return max(0.02, min(0.98, (ax - n['x']) / n['w']))

    def link(a, b, ax, lines, color, dashed=False, width=1.6, ylab=None, updown='down'):
        """세로 짧은 화살표 — 라벨 y 를 직접 잡는다."""
        sa = 'bottom@%.4f' % fx(a, ax) if updown == 'down' else 'top@%.4f' % fx(a, ax)
        sb = 'top@%.4f' % fx(b, ax) if updown == 'down' else 'bottom@%.4f' % fx(b, ax)
        y1 = d.anchor(a, sa)[1]
        y2 = d.anchor(b, sb)[1]
        yl = ylab if ylab is not None else (y1 + y2) / 2
        d.edge(a, b, '', '%s-%s' % (sa, sb), color=color, width=width, dashed=dashed,
               via=[(ax, yl - 1), (ax, yl + 1)], label_lines=lines, label_x=ax)

    def side(a, b, anchors, lines, color, dashed=False, width=1.6, mid=None, lx=None):
        d.edge(a, b, '', anchors, color=color, width=width, dashed=dashed, mid=mid,
               label_lines=lines, label_x=lx)

    # ══════════════ 로봇 제어 PC (좌·우 같은 구성) ══════════════
    def robot_pc(px, pw, ns, sty, insty, tag):
        bx, bw = px + 16, pw - 32
        cw = (bw - 40) / 3                 # 윗줄 3열
        aw = (bw - 150) / 2                # 아랫줄 2열 — 가운데 150 px 는 액션 라벨 자리
        bx2 = bx + aw + 150
        G1, G2, G3 = 46, 54, 46            # 줄 사이 간격 = 라벨이 들어갈 자리

        y0 = TOP + 40
        d.box(tag + 'drv', bx, y0, bw, [
            'TurtleBot4 · OAK-D Pro 드라이버',
            'oakd/rgb · stereo/depth · camera_info   704×704 crop · BEST_EFFORT',
            'scan 10 Hz · odom · tf · battery_state · dock_status',
            'cmd_vel (geometry_msgs/TwistStamped) 로 구동한다',
        ], insty)
        y1 = y0 + 82 + G1
        d.box(tag + 'aru', bx, y1, cw, [
            'aruco_detector',
            'camera_fx_scale 1.7778 보정 (I-031)',
            'solvePnP IPPE_SQUARE · 1.5 m 게이트',
            'aruco/id · aruco/distance',
        ], insty)
        d.box(tag + 'det', bx + cw + 20, y1, cw, [
            'amr_detection',
            'RGB+Depth 동기 slop 0.1 s',
            'worker.pt · 재식별 임계 0.6',
            'target/paired · map_point',
        ], insty)
        d.box(tag + 'saf', bx + 2 * cw + 40, y1, cw, [
            'safety_watchdog  (온보드)',
            'scan 만 구독 · 전방 ±35°',
            '0.30 m 2프레임 → estop',
            'cmd_vel 0 을 20 Hz 로 덮어씀',
        ], 'in_saf')
        y2 = y1 + 82 + G2
        d.box(tag + 'shu', bx, y2, aw, [
            'shuttle_mission    S4 셔틀',
            'READY → SHUTTLE → AT_BAY → WAITING',
            'ArUco 확인 → 적재 5 s → P1 토큰 → 대기공간',
            'T_lim 120 s 초과 → 미션 폐기 (계약 7절)',
            'WAITING 은 도착 확정 시 1회 — AC-08 기준점',
        ], insty)
        d.box(tag + 'nav', bx2, y2, aw, [
            'Nav2 / AMCL   turtlebot4_navigation',
            'map_server · amcl · bt_navigator',
            'planner_server · controller_server',
            'velocity_smoother → cmd_vel',
            'xy_goal_tolerance 0.25 (I-029)',
        ], insty)
        y3 = y2 + 97 + G3
        d.box(tag + 'fol', bx, y3, aw, [
            'follow_target      S5~S8 추종',
            'APPROACH_1 / 2 → ARRIVED → FOLLOWING',
            'standoff 0.5 m · 놓치면 힌트 방향 탐색',
            'T_lim 초과 → STANDBY (운영자 대기)',
        ], insty)
        d.box(tag + 'dok', bx2, y3, aw, [
            'opennav_docking',
            'action  undock  — S2 기동',
            'action  dock     — S10 복귀',
        ], insty)
        y4 = y3 + 82 + G3
        d.box(tag + 'ret', bx, y4, aw, [
            'return_dock       S10 복귀',
            'P1 → DOCK 토큰 (한 번에 한 대, E-7)',
            '도크 정면 접근 지점 경유 → dock',
            'T_ret 180 s · 재시도 2회 · 실패해도 반납',
        ], insty)
        d.box(tag + 'mot', bx2, y4, aw, [
            'Create3 구동부',
            'v ≤ 0.3 m/s · 범퍼 · 클리프 · 도크 IR',
        ], 'in_hw')
        bot = y4 + 82

        # ── 세로 화살표: 라벨 y 를 줄 사이 간격 한가운데로 못박는다 ──
        def vlink(a, b, ax, ylab, lines, color, width=1.6, dashed=False):
            sa, sb = 'bottom@%.4f' % fx(a, ax), 'top@%.4f' % fx(b, ax)
            d.edge(a, b, '', '%s-%s' % (sa, sb), color=color, width=width, dashed=dashed,
                   via=[(ax, ylab - 1), (ax, ylab + 1)], label_lines=lines, label_x=ax)

        c1, c2, c3 = bx + cw / 2, bx + cw * 1.5 + 20, bx + cw * 2.5 + 40
        m1, m2 = ylab_in(y0 + 82, G1), ylab_in(y1 + 82, G2)
        vlink(tag + 'drv', tag + 'aru', c1, m1, ['oakd/rgb · camera_info'], TOPIC)
        vlink(tag + 'drv', tag + 'det', c2, m1, ['oakd/rgb · stereo/depth'], TOPIC)
        vlink(tag + 'drv', tag + 'saf', c3, m1, ['scan   LaserScan'], SAFE, width=2.2)
        vlink(tag + 'aru', tag + 'shu', c1, m2, ['aruco/id · aruco/distance'], TOPIC)
        vlink(tag + 'det', tag + 'fol', c2, m2, ['target/paired · map_point'], TOPIC)
        vlink(tag + 'saf', tag + 'nav', c3, m2, ['safety/estop → Goal Cancel'], SAFE, width=2.2)
        vlink(tag + 'shu', tag + 'fol', bx + aw * 0.18, ylab_in(y2 + 97, G3),
              ['shuttle/arrived'], TOPIC)
        vlink(tag + 'dok', tag + 'mot', bx2 + aw * 0.52, ylab_in(y3 + 82, G3, 2),
              ['cmd_vel  TwistStamped', 'estop 중엔 safety 의 0 이 이긴다'], TOPIC)

        # ── 가로 화살표: 라벨은 가운데 150 px 통로에 들어간다 ──
        gm = bx + aw + 75
        side(tag + 'shu', tag + 'nav', 'right@0.30-left@0.30',
             ['navigate_to_pose', '(액션) 베이 · 대기공간'], ACT, width=2.0, mid=gm, lx=gm)
        side(tag + 'fol', tag + 'nav', 'right@0.35-left@0.85',
             ['navigate_to_pose', '(액션) 작업자 추종'], ACT, width=2.0, mid=gm, lx=gm)
        side(tag + 'ret', tag + 'dok', 'right@0.30-left@0.80',
             ['dock / undock'], ACT, width=2.0, mid=gm, lx=gm)

        d.band(tag + 'pc', px, TOP, pw, bot - TOP + 18, ns, sty, dash=False)
        return bot + 18

    r3_bot = robot_pc(L1X, L1W, 'PC-R3 — 로봇3 제어 PC   (namespace /robot3)', 'pc_r3', 'in_r3', 'a')
    r4_bot = robot_pc(L2X, L2W, 'PC-R4 — 로봇4 제어 PC   (namespace /robot4)', 'pc_r4', 'in_r4', 'b')

    # ══════════════ 관제 PC ══════════════
    cb, cbw = CX + 16, CW - 32
    y = TOP + 62          # 안쪽 밴드 제목 자리를 비운다
    hw = (cbw - 130) / 2
    hx2 = cb + hw + 130
    d.box('ui', cb, y, hw, [
        'control_ui     Flask :5000',
        '운영자 화면 — 2대 상태 표시 (뷰어)',
        '자재 요청 → RequestMission (서비스)',
        '종료 → OperatorCommand RETURN / RETURN_ALL',
    ], 'in_ctl')
    d.box('fm', hx2, y, hw, [
        'fleet_manager',
        '서술자 수신 → 로봇 배정 (트랙 ID 작은 쪽)',
        '배터리 25 % 미만이면 미션 거부',
        'mission/assigned · mission/finish 발행',
        'FleetStatus 1 Hz · SQLite events 기록',
    ], 'in_ctl')
    y += 97 + 22
    d.box('zs', cb, y, hw, [
        'zone_semaphore   (소유: 주행)',
        '구역은 P1 · DOCK 둘뿐 · FIFO 대기열',
        'REJECT 는 대기 시작 1건 · GRANT 는 새 획득만',
        '보유 60 s 초과 → RECLAIM (보유자 ns)',
    ], 'in_ctl')
    d.box('kpi', hx2, y, hw, [
        'kpi_logger',
        '상태 · 미션 · 개입 · 구역 이벤트 구독',
        '거르지 않고 받아적는다 — 판정은 리포트에서',
        '전부 ROS 시각 기준',
    ], 'in_ctl')
    ctl_bot = y + 82
    d.band('ctl_sys', CX + 8, TOP + 32, CW - 16, ctl_bot - TOP - 10,
           '관제 시스템 / Control Server', 'band', dash=True)

    y = ctl_bot + 96      # 안쪽 밴드 제목 자리 + 그 위 라벨 자리를 비운다
    d.box('db', hx2 + 40, y, hw - 80, [
        'SQLite   ~/mro_fleet.db',
        'workers · missions · events',
        'kind = STATE / INTERVENTION / ZONE',
    ], 'db', h=76, shape='cyl')
    d.box('rep', cb, y, hw, [
        'kpi_report.py',
        'AC-05 · AC-08 · AC-13 · K-04 · K-05 · K-06',
        'K-06: REJECT → GRANT 60 s · RECLAIM 있으면 미해결',
    ], 'in_ctl')
    obs_bot = y + 76
    d.band('ctl_obs', CX + 8, ctl_bot + 60, CW - 16, obs_bot - ctl_bot - 44,
           '관측 · 기록 / Observability', 'band', dash=True)
    # 두 안쪽 밴드 사이(ctl_sys 아래 ctl_bot+18 ~ ctl_obs 위 ctl_bot+60)에 라벨을 넣는다
    link('kpi', 'db', hx2 + hw * 0.5, ['CSV + SQLite 기록'], PLAIN,
         ylab=ctl_bot + 52)
    side('db', 'rep', 'left@0.5-right@0.5', ['events · missions 조회'], PLAIN,
         mid=cb + hw + 85, lx=cb + hw + 85)
    side('ui', 'fm', 'right@0.35-left@0.35', ['RequestMission', '(서비스)'], SRV,
         dashed=True, mid=cb + hw + 65, lx=cb + hw + 65)

    ctl_pc_bot = obs_bot + 40      # 안쪽 밴드 아래 선과 22 px 띄운다
    d.band('pc_f', CX, TOP, CW, ctl_pc_bot - TOP, 'PC-F — 관제 PC   (Discovery Server ID 1)',
           'pc_ctl', dash=False)

    # ══════════════ DDS 버스 — 토픽을 상자로 세운다 (rqt_graph 규약) ══════════════
    # 지금까지 토픽은 화살표의 라벨이었다. 그러면 한 토픽을 여럿이 구독하는 것이 안 보인다 —
    # mission/finish 는 받는 노드가 셋인데(계약 4절) 화살표 하나에 가려 있었다.
    # 강사 자료 Day5 p.72(rqt_graph)처럼 **토픽도 상자**로 세우고 노드 → 토픽 → 노드로 잇는다.
    # 로봇 토픽은 상대 이름이라 로봇마다 다른 토픽이다(/robot3/… vs /robot4/…). p.72 처럼
    # 네임스페이스를 묶음 상자로 두고 안에는 상대 이름만 적는다 — 그래야 선이 안 엉킨다.
    T = contract_topics()
    BY = max(r3_bot, r4_bot, ctl_pc_bot) + 300
    BW, BH = W - 2 * L1X, 196
    d.band('bus', L1X, BY, BW, BH, '', 'bus', dash=False)
    d.text(L1X + 18, BY + 24, 'ROS 2 / DDS  —  토픽 · 서비스', 12.5, '#333840', bold=True)
    d.text(L1X + 18, BY + 42, 'Discovery Server 192.168.108.41 · 중앙 브로커 없음 (NR-06)',
           10, '#5A5F6A')
    d.text(W - L1X - 18, BY + 24, '명령 · 상태 = RELIABLE', 12.5, '#333840', bold=True, anchor='end')
    d.text(W - L1X - 18, BY + 42, '카메라 · 힌트 = BEST_EFFORT (구독자도 sensor_data QoS)',
           10, '#8E2A1F', anchor='end')

    TBH, TG = 54, 10
    TBW_R, TBW_C = 146, 186        # 전역 이름 쪽이 길다 (/webcam1/worker_descriptor)
    GW_R = 5 * TBW_R + 4 * TG + 24
    GW_C = 5 * TBW_C + 4 * TG + 24
    GY = BY + 58
    G1X, G3X = L1X + 26, L1X + BW - 26 - GW_R
    G2X = L1X + (BW - GW_C) / 2
    bxc = {}

    def group(gid, gx, gw, tbw, label, items, ns_style='band'):
        # id 는 반드시 직접 준다. 예전에 label 앞글자로 만들었더니 두 로봇 묶음이 같은 id 가 돼
        # 하나가 통째로 사라졌다 (9/8)
        d.band(gid, gx, GY, gw, TBH + 40, '', ns_style)
        d.text(gx + 12, GY + 17, label, 11.5, '#4B357A', bold=True)
        for i, (nid, nm, short, kind) in enumerate(items):
            x = gx + 12 + i * (tbw + TG)
            bxc[nid] = x + tbw / 2
            if kind == 'srv':
                d.box(nid, x, GY + 26, tbw, [short or nm, 'srv/ZoneToken'], 'srv',
                      h=TBH, shape='hex', align='center')
            else:
                d.box(nid, x, GY + 26, tbw, tp(T, nm, short=short),
                      'tp_saf' if kind == 'safe' else 'tp_t', h=TBH, align='center')

    # 키는 손으로 준다 — 이름 끝조각으로 만들면 shuttle/status 와 approach/status 가 겹친다
    ROBOT_TOPICS = [('ss', 'shuttle/status', None, 'tp'),
                    ('as', 'approach/status', None, 'tp'),
                    ('pr', 'target/pair_state', None, 'tp'),
                    ('ad', 'aruco/distance', None, 'tp'),
                    ('es', 'safety/estop', None, 'safe')]
    group('g_r3', G1X, GW_R, TBW_R, '%s  — 로봇마다 따로 뜬다' % NS_A,
          [('r3_' + k, n, sh, kind) for k, n, sh, kind in ROBOT_TOPICS])
    group('g_r4', G3X, GW_R, TBW_R, '%s  — 같은 이름, 다른 토픽' % NS_B,
          [('r4_' + k, n, sh, kind) for k, n, sh, kind in ROBOT_TOPICS])
    group('g_gl', G2X, GW_C, TBW_C, '전역 이름  (관제 · 웹캠 — 네임스페이스 없음)', [
        ('gt_ma', 'mission/assigned', None, 'tp'),
        ('gt_mf', 'mission/finish', None, 'tp'),
        ('gt_wd', '/webcam1/worker_descriptor', 'webcam1/worker_desc.', 'tp'),
        ('gt_th', '/webcam2/target_hint', 'webcam2/target_hint', 'tp'),
        ('gt_zt', '/fleet/zone_token', '/fleet/zone_token', 'srv')])

    d.text(L1X + 18, BY + BH - 14,
           '네모 = 토픽 · 육각 = 서비스.  노드에서 나온 화살표는 발행, 토픽에서 나온 화살표는 구독이다 — '
           '한 토픽에서 화살표가 여럿 나가면 구독자가 여럿이라는 뜻이다 (rqt_graph 규약, 강사 자료 Day5 p.72).',
           11, '#3A3E46')

    # ── 패널 ↔ 토픽 상자 ── 차선(가로 y)을 토픽마다 다르게 준다. 같으면 선이 포개진다
    LANE0, LANE_H = BY - 282, 21

    def wire(panel, tnid, lane, pf, tf=0.5, color=TOPIC, dashed=False, up=False, width=1.7):
        ly = LANE0 + lane * LANE_H
        if up:
            d.edge(tnid, panel, '', 'top@%.3f-bottom@%.3f' % (tf, pf), color=color,
                   width=width, dashed=dashed, mid=ly)
        else:
            d.edge(panel, tnid, '', 'bottom@%.3f-top@%.3f' % (pf, tf), color=color,
                   width=width, dashed=dashed, mid=ly)

    # 로봇 → 자기 네임스페이스 토픽 (발행). 짧게 내려간다
    for tag, pref, pf0 in (('a', 'r3_', 0.06), ('b', 'r4_', 0.60)):
        for i, (key, _nm, _sh, k) in enumerate(ROBOT_TOPICS):
            wire(tag + 'pc', pref + key, i, pf0 + i * 0.075, tf=0.5,
                 color=SAFE if k == 'safe' else TOPIC, width=2.1 if k == 'safe' else 1.7)
    # 토픽 → 관제 (구독). 관제는 두 로봇 것을 모두 받는다
    for i, (key, _nm, _sh, k) in enumerate(ROBOT_TOPICS):
        wire('pc_f', 'r3_' + key, 5 + i, 0.05 + i * 0.04, tf=0.30,
             color=SAFE if k == 'safe' else TOPIC, up=True, width=1.5)
        wire('pc_f', 'r4_' + key, 5 + i, 0.25 + i * 0.04, tf=0.70,
             color=SAFE if k == 'safe' else TOPIC, up=True, width=1.5)
    # 관제 → 전역 토픽 (발행) · 전역 토픽 → 두 로봇 (구독). 화살표가 둘로 갈라지는 자리다
    wire('pc_f', 'gt_ma', 10, 0.52, tf=0.5)
    wire('pc_f', 'gt_mf', 11, 0.58, tf=0.5)
    wire('pc_f', 'gt_zt', 12, 0.64, tf=0.5, color=SRV, dashed=True, width=2.0)
    for tag, tf_l, pf in (('a', 0.22, 0.86), ('b', 0.78, 0.14)):
        wire(tag + 'pc', 'gt_ma', 10, pf, tf=tf_l, up=True)
        wire(tag + 'pc', 'gt_mf', 11, pf - 0.05 if tag == 'a' else pf + 0.05, tf=tf_l, up=True)
        wire(tag + 'pc', 'gt_zt', 12, pf - 0.10 if tag == 'a' else pf + 0.10, tf=tf_l,
             color=SRV, dashed=True, up=True, width=2.0)
        wire(tag + 'pc', 'gt_wd', 13, pf - 0.15 if tag == 'a' else pf + 0.15, tf=tf_l, up=True)
        wire(tag + 'pc', 'gt_th', 13, pf - 0.20 if tag == 'a' else pf + 0.20, tf=tf_l,
             dashed=True, up=True)
    wire('pc_f', 'gt_wd', 13, 0.94, tf=0.5, up=True, width=1.5)

    # ══════════════ 웹캠 PC ══════════════
    WY = BY + BH + 190          # 버스 높이(BH)를 따라간다 — 108 로 굳어 있어 웹캠 패널이 버스에 붙었다
    wb, wbw = CX + 16, CW - 32
    d.box('w1', wb, WY + 40, (wbw - 16) / 2, [
        'webcam_register     PC-W1',
        '출입구 시야 · 캡처 %g fps (Jieli 장치 상한) → 추론 %g Hz' % (CAM1_FPS, PROC_HZ),
        'ByteTrack ID · 0.3 s 연속이어야 등록',
        'HSV H16×S8 = 128 · 10프레임 평균 · 재등록 0.45',
        '/webcam1/worker_descriptor 발행',
    ], 'in_cam')
    d.box('w2', wb + (wbw - 16) / 2 + 16, WY + 40, (wbw - 16) / 2, [
        'webcam_hint          PC-W2',
        '작업공간 + P3 입구 · 캡처 %g fps → 추론 %g Hz' % (CAM2_FPS, PROC_HZ),
        '호모그래피로 바닥 좌표 환산 (I-002 재검증 중)',
        '/webcam2/target_hint  10 Hz · 유효 2 s',
        'BEST_EFFORT — 안전 판단에는 쓰지 않는다 (NF-07)',
    ], 'in_cam')
    cam_bot = WY + 40 + 97 + 18
    d.band('pc_w', CX, WY, CW, cam_bot - WY, 'PC-W1 / PC-W2 — 웹캠 PC', 'pc_cam', dash=False)
    # 웹캠 PC 는 버스 아래에 있다 — 토픽 상자의 아랫변으로 올려 붙인다 (발행)
    d.edge('pc_w', 'gt_wd', '', 'top@0.30-bottom@0.5', color=TOPIC, width=1.7,
           mid=BY + BH + 66)
    d.edge('pc_w', 'gt_th', '', 'top@0.70-bottom@0.5', color=TOPIC, width=1.7,
           dashed=True, mid=BY + BH + 108)
    d.text(CX + CW / 2, BY + BH + 56, 'webcam_register 발행', 10.5, TOPIC, True, 'middle')
    d.text(CX + CW / 2, BY + BH + 128, 'webcam_hint 발행 · BEST_EFFORT', 10.5, TOPIC, True, 'middle')

    # ══════════════ 게이트 규칙 (좌·우) ══════════════
    def rules(nid, px, pw, y0, title, lines):
        d.band(nid, px, y0, pw, 50 + len(lines) * 20 + 8, '', 'band', dash=True)
        d.text(px + 18, y0 + 26, title, 13, '#333840', bold=True)
        for i, ln in enumerate(lines):
            d.text(px + 18, y0 + 50 + i * 20, ln, 11, '#3A3E46')
        return y0 + 50 + len(lines) * 20 + 8

    g1 = rules('gate1', L1X, L1W, WY, '주행 게이트', [
        '· P1 토큰 — 베이 출발 전 획득, 대기공간 도착 후 반납 (방법 B, 순서 기반)',
        '· 거절되면 2 s 후 재요청 · 60 s 초과 보유는 세마포어가 강제 회수 (RECLAIM)',
        '· 받았든 못 받았든 반드시 반납한다 — 안 하면 대기열에 남아 P1 이 잠긴다 (I-033)',
        '· estop true 동안 주행 명령 금지 · cmd_vel 0 이 Nav2 출력을 덮어쓴다',
        '· T_lim 120 s 초과 → 미션 폐기 · 늦은 도착은 arrived_late 로 따로 기록',
        '· 도크 간격 0.84 m — DOCK 토큰으로 한 대씩 (E-7 순차 도킹)',
    ])
    g2 = rules('gate2', L2X, L2W, WY, '판정 게이트  (KPI 기준점)', [
        '· AC-08 배송 성공 — shuttle/status = WAITING 이 도착 확정 시각. 미션당 1회',
        '· K-05 리드타임 — 요청 created_at → WAITING. DONE(도킹 완료)과 다르다',
        '· AC-05 추종거리 — FOLLOWING 구간만. Nav2 도착 오차 0.25 는 판정에서 뺀다 (I-029)',
        '· K-04 재획득 — target/visible 1→0 이 1.0 s 이상. STANDBY 면 포기로 센다',
        '· K-06 교착 — REJECT → GRANT 60 s. 그 사이 그 구역에 RECLAIM 이 있으면 미해결',
        '· AC-13 IPH — 개입 ÷ (undock ~ DOCKED). 안전 정지 뒤 자동 재개는 개입이 아니다',
    ])

    # 바깥 PC 컨테이너를 먼저 그린다 — 나중에 그리면 안쪽 밴드의 제목을 덮는다
    outer = ('apc', 'bpc', 'pc_f', 'pc_w', 'bus')
    d.order.sort(key=lambda t: 0 if (t[0] == 'band' and t[1] in outer) else 1)

    # ══════════════ 범례 ══════════════
    ly = max(cam_bot, g1, g2) + 44
    d.text(L1X + 6, ly, '읽는 법', 13, '#333840', bold=True)
    seg = [('토픽  Topic', TOPIC, False, 1.8), ('서비스  Service', SRV, True, 1.8),
           ('액션  Action', ACT, False, 2.4), ('안전 경로  (토픽이지만 따로 표시)', SAFE, False, 2.8)]
    lx = L1X + 6
    for name, col, dash, wd in seg:
        d.edges.append(dict(a=None, b=None, label='', anchors='', dashed=dash, color=col,
                            ldx=0, ldy=0, width=wd, arrow=True, mid=None,
                            via=[(lx, ly + 22), (lx + 54, ly + 22)], lines=None,
                            lx_fix=None, lanchor='middle', raw=True))
        d.text(lx + 62, ly + 26, name, 11, '#3A3E46')
        lx += 62 + d.text_w(name, False) + 34
    d.text(L1X + 6, ly + 54,
           'PC 는 상자 채움색으로 나눈다 — 파랑 로봇3 · 보라 로봇4 · 주황 관제 · 초록 웹캠 · 회색 온보드. '
           '테두리는 전부 회색이라 화살표 색과 섞이지 않는다.', 11, '#3A3E46')
    d.text(L1X + 6, ly + 74,
           '화살표 위 상자 — 첫 줄이 토픽 · 서비스 · 액션 이름, 둘째 줄이 메시지 타입과 방향이다. '
           'PC 를 넘는 것은 전부 가운데 DDS 버스를 지난다 — 직접 호출도 소켓도 없다 (NR-06).', 11, '#3A3E46')
    d.text(L1X + 6, ly + 94,
           'robot4 는 robot3 과 구성이 같다. 두 대가 만나는 곳은 zone_semaphore 의 P1 · DOCK 토큰뿐이고, '
           '그 경합이 E-3 교착과 E-7 순차 도킹이다.', 11, '#8A8F98')
    d.text(L1X + 6, ly + 116,
           '생성: tools/build_diagrams.py   ·   근거: docs/architecture.md, docs/interfaces.md, SDD 2.4 FR 표',
           10, '#8A8F98')
    return d.autosize()


# =====================================================================
def main():
    diagrams = [('step1_deployment', step1()),
                ('step2_nav2', step2()),
                ('step3_scenario', step3()),
                ('step3b_system_flow', step3b())]
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
