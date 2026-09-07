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
        s_side, t_side = e['anchors'].split('-')
        x1, y1 = self.anchor(e['a'], s_side)
        x2, y2 = self.anchor(e['b'], t_side)
        if e.get('via'):
            pts = [(x1, y1)] + [tuple(p) for p in e['via']] + [(x2, y2)]
            k = len(pts) // 2
            lx = (pts[k - 1][0] + pts[k][0]) / 2
            ly = (pts[k - 1][1] + pts[k][1]) / 2
            return pts, lx, ly
        if s_side in ('bottom', 'top'):
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

    def autosize(self, margin=28):
        b = 0
        for n in self.nodes.values():
            b = max(b, n['y'] + n['h'])
        for t in self.free:
            b = max(b, t['y'] + 6)
        self.h = int(b + margin)
        return self

    def check(self):
        warn = []
        for n in self.nodes.values():
            if n['y'] + n['h'] > self.h:
                warn.append('%s 가 캔버스 아래로 넘침' % n['id'])
            if n['x'] + n['w'] > self.w:
                warn.append('%s 가 캔버스 오른쪽으로 넘침' % n['id'])
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
        else:
            out.append('<rect x="%g" y="%g" width="%g" height="%g" rx="5" fill="%s" '
                       'stroke="%s" stroke-width="1.6"/>' % (x, y, w, h, fill, stroke))
        centered = n['align'] == 'center' or n['shape'] in ('diamond', 'stadium')
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
        if e.get('lines'):
            ls = e['lines']
            cx = e['lx_fix'] if e.get('lx_fix') is not None else lx
            lh = 13
            wmax = max(sum(11 if ord(c) > 0x2E80 else 6.1 for c in t) for t in ls)
            bw, bh = wmax + 14, lh * len(ls) + 8
            bx = cx - bw / 2 if e['lanchor'] == 'middle' else cx - 7
            by = ly - bh - 5
            s += ('<rect x="%g" y="%g" width="%g" height="%g" rx="3" fill="#FFFFFF" '
                  'fill-opacity="0.94" stroke="%s" stroke-width="0.8"/>'
                  % (bx, by, bw, bh, e['color']))
            for i, t in enumerate(ls):
                s += ('<text x="%g" y="%g" font-size="10.5" fill="%s"%s>%s</text>'
                      % (bx + 7, by + 15 + lh * i,
                         e['color'] if i == 0 else '#3A3E46',
                         ' font-weight="600"' if i == 0 else '', esc(t)))
        elif e['label']:
            s += ('<text x="%g" y="%g" font-size="10.5" fill="%s" text-anchor="middle">'
                  '%s</text>' % (lx + e['ldx'], ly + e['ldy'], e['color'], esc(e['label'])))
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
                                    self.title, 19, True, '#1A1D22'))
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
            else:
                shape = 'rounded=1;arcSize=6;'
            dashed = ('dashed=1;dashPattern=5 4;'
                      if kind == 'band' and n.get('dash', True) else 'dashed=0;')
            align = ('align=center;' if (n['align'] == 'center' or
                                         n['shape'] in ('diamond', 'stadium'))
                     else 'align=left;spacingLeft=10;')
            # draw.io 의 value 는 XML 속성이다. HTML 태그째로 넣되
            # 문자열 전체를 마지막에 한 번만 이스케이프한다 (아래 esc()).
            body = []
            for i, ln in enumerate(n['lines']):
                t = ln[1:] if ln.startswith('!') else ln
                if i == 0:
                    body.append('<b><font color="%s">%s</font></b>' % (tc, t))
                else:
                    body.append('<font style="font-size:10px">%s</font>' % t)
            style = ('%swhiteSpace=wrap;html=1;fillColor=%s;strokeColor=%s;%s'
                     'verticalAlign=%s;spacingTop=4;fontFamily=%s;fontSize=11;%s'
                     % (shape, fill, stroke, align,
                        'middle' if n['align'] == 'center' or n['shape'] != 'rect' else 'top',
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
            cells.append(self._dio_text(nid('f'), t['x'] - (0 if t['anchor'] == 'start' else 120),
                                        t['y'] - 14, 260, 20, t['s'], t['size'],
                                        t['bold'], t['color'],
                                        'center' if t['anchor'] != 'start' else 'left'))
        model = ('<mxGraphModel dx="1400" dy="900" grid="1" gridSize="10" guides="1" '
                 'tooltips="1" connect="1" arrows="1" fold="1" page="1" pageScale="1" '
                 'pageWidth="%d" pageHeight="%d" math="0" shadow="0"><root>%s</root>'
                 '</mxGraphModel>' % (self.w, self.h, ''.join(cells)))
        return ('<diagram id="%s" name="%s">%s</diagram>'
                % (esc(page_id), esc(self.key), model))

    @staticmethod
    def _dio_text(gid, x, y, w, h, s, size, bold, color, align='left'):
        style = ('text;html=1;strokeColor=none;fillColor=none;align=%s;'
                 'verticalAlign=middle;fontFamily=%s;fontSize=%g;fontColor=%s;%s'
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
                  'inflation_radius 0.25  ← 받침대 6.5 cm 덮기 (I-009)'], 'nav2'),
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
    """STEP 3-1 시나리오 순서도 — 강의안(Day5 p.73) 형식.

    상자 하나에 짧은 한 줄, 판단은 마름모 + Y/N, 예외는 오른쪽에 태그만 붙이고
    설명은 그림 아래 표로 뺀다. 흐름이 길어 두 단으로 접고 ① 로 잇는다.
    """
    d = Diagram('STEP3 시나리오', 1180, 100,
                'STEP 3-1 — 운영 시나리오 S1 ~ S10 순서도',
                '상자 = 단계 · 마름모 = 판단(Y/N) · 오른쪽 태그 = 그때의 예외 처리(아래 표) · SDD 3.3 / BRD 3.2')

    BH, DH, GAP = 30, 44, 18
    S, D, T, X = 'step', 'dec', 'term', 'exc'

    A_NODE, AX, AW, A_EXC = 196, 210, 210, 440   # 1단 — 노드 이름 오른쪽 끝 · 흐름 · 예외 태그
    B_NODE, BX, BW, B_EXC = 756, 770, 210, 1000  # 2단
    TOP = 142

    def column(x, w, y0, node_x, exc_x, items):
        """items = (id, 라벨, 모양, 아래 가지 라벨, 수행 노드, (예외태그, 가지라벨))"""
        prev, prev_lbl, y = None, '', y0
        for nid, lbl, shape, out, node, exc in items:
            h = DH if shape == 'd' else (26 if shape == 't' else BH)
            style = {'d': D, 't': T}.get(shape, S)
            sh = {'d': 'diamond', 't': 'stadium'}.get(shape, 'rect')
            d.box(nid, x, y, w, [lbl], style, h=h, shape=sh, align='center')
            if node:
                d.text(node_x, y + h / 2 + 4, node, 10.5, '#8A8F98', anchor='end')
            if exc:
                tag, elbl = exc
                d.box(nid + '_e', exc_x, y + h / 2 - 13, 150, [tag], X, h=26, align='center')
                d.edge(nid, nid + '_e', elbl, 'right-left', color='#C0392B',
                       width=1.2, label_dy=-4)
            if prev:
                d.edge(prev, nid, prev_lbl, 'bottom-top', color='#5A5F6A',
                       width=1.3, label_dx=13, label_dy=-5)
            prev, prev_lbl, y = nid, out, y + h + GAP
        return y - GAP

    # ─────────── 1단 · S1 ~ S4′ ───────────
    d.text(AX, TOP - 58, '1단  —  요청에서 대기공간까지', 12, '#5A5F6A', bold=True)
    a_bot = column(AX, AW, TOP, A_NODE, A_EXC, [
        ('s1', 'S1  진입 · 등록', 'r', '', 'webcam_register', ('서술자 유사', '')),
        ('s2', 'S2  준비 (undock)', 'r', '', 'shuttle_mission', ('I-013  맵 좌표계', '')),
        ('s3', 'S3  자재 요청', 'r', '', 'control_ui', None),
        ('s4', 'S4  베이 이동', 'r', '', 'shuttle_mission', None),
        ('q1', '베이 ArUco 일치?', 'd', 'Y', 'aruco_detector', ('오도착 → 재접근', 'N')),
        ('q2', 'P1 토큰 획득?', 'd', 'Y', 'zone_semaphore', ('E-3  교착 회피', 'N')),
        ('s5', "S4′  적재 · 대기공간", 'r', '', 'shuttle_mission', None),
    ])
    d.box('cx_a', AX + AW / 2 - 17, a_bot + 22, 34, ['①'], 'term', h=34, shape='stadium',
          align='center')
    d.edge('s5', 'cx_a', '', 'bottom-top', color='#5A5F6A', width=1.3)

    # ─────────── 2단 · 재식별 ~ 도킹 ───────────
    d.text(BX, TOP - 58, '2단  —  합류에서 도킹까지', 12, '#5A5F6A', bold=True)
    d.box('cx_b', BX + BW / 2 - 17, TOP - 44, 34, ['①'], 'term', h=32, shape='stadium',
          align='center')
    b_bot = column(BX, BW, TOP, B_NODE, B_EXC, [
        ('q3', '재식별 통과?', 'd', 'Y', 'amr_detection', ('E-2  오추종 금지', 'N')),
        ('s6', 'S5  합류 · 접근', 'r', '', 'follow_target', None),
        ('s7', 'S6~S8  추종 0.5 m', 'r', '', 'follow_target', None),
        ('q4', '대상 놓침?', 'd', 'N', 'amr_detection', ('E-1  재탐지', 'Y')),
        ('q5', '전방 0.30 m 이내?', 'd', 'N', 'safety_watchdog', ('E-4  안전 정지', 'Y')),
        ('s8', 'S9  작업 종료', 'r', '', 'control_ui', None),
        ('q6', 'P1 · DOCK 토큰?', 'd', 'Y', 'return_dock', ('E-7  순차 도킹', 'N')),
        ('s9', 'S10  도킹', 'r', '', 'return_dock', None),
        ('q7', '도킹 성공?', 'd', 'Y', 'return_dock', ('재시도 2회 → 실패', 'N')),
        ('end', 'DOCKED  ·  Phase 1 종료', 't', '', '', None),
    ])
    d.edge('cx_b', 'q3', '', 'bottom-top', color='#5A5F6A', width=1.3)

    # ─────────── 예외 표 ───────────
    ty = max(a_bot, b_bot) + 74
    d.text(AX - 186, ty - 16, '예외 처리', 12, '#5A5F6A', bold=True)
    rows = [
        ('서술자 유사', '같은 인형끼리와 다른 인형끼리의 유사도 분포가 겹치면 서술자를 손봐야 한다는 신호다. 조명을 맞추고 다시 등록한다.'),
        ('I-013  맵 좌표계', '각 로봇이 (0,0) 으로 초기화하면 같은 좌표가 다른 장소를 가리킨다. arena.yaml 의 dock_zone.pose 값으로 초기화한다.'),
        ('오도착 → 재접근', '다시 접근해 마커를 읽는다. T_lim 120 초를 넘기면 FAILED. 늦은 도착은 arrived_late 로 따로 남겨 이중 계상하지 않는다.'),
        ('E-3  교착 회피', '거절되면 2 초 뒤 다시 요청한다. 구역에 들어가기 전에 얻고 빠져나온 뒤 반납한다. 60 초를 넘겨 쥐면 강제 회수한다.'),
        ('E-2  오추종 금지', '유사도가 임계에 못 미치면 페어링하지 않고 대기한다. 확신이 없을 때 멈추는 쪽이 항상 옳다. 폴백은 운영자 수동 지정.'),
        ('E-1  재탐지', '작업공간이면 웹캠2 힌트 방향으로 회전 탐색, 중앙 복도는 시야 밖이라 제자리 탐색. T_lim 초과 시 STANDBY.'),
        ('E-4  안전 정지', 'safety/estop 을 올리고 cmd_vel 에 0 을 20 Hz 로 덮어쓴다. 0.45 m 밖이면 스스로 재개. 개입(IPH)에 계상하지 않는다.'),
        ('E-7  순차 도킹', '도크 간격이 0.84 m 뿐이라 동시에 들어가면 IR 이 간섭한다. DOCK 토큰으로 한 대씩, 나머지는 홀딩 포인트 대기.'),
        ('재시도 2회 → 실패', '최대 2회 재시도 후 FAILED 를 발행하고 멈춘다. 성공이든 실패든 DOCK 토큰은 반드시 반납한다.'),
    ]
    for i, (tag, txt) in enumerate(rows):
        yy = ty + i * 27
        d.box('tb%d' % i, AX - 186, yy, 150, [tag], X, h=23, align='center')
        d.text(AX - 24, yy + 16, txt, 11, '#3A3E46')

    d.text(AX - 186, ty + len(rows) * 27 + 26,
           '전 구간 공통 — 모든 로그와 CSV 는 ROS 시각을 쓴다. 안전 정지 뒤의 자동 재개는 개입으로 계상하지 않는다.',
           11, '#5A5F6A')
    d.text(AX - 186, ty + len(rows) * 27 + 46,
           '생성: tools/build_diagrams.py   ·   근거: BRD 3.2 시나리오, SDD 3.3, docs/interfaces.md',
           10, '#8A8F98')
    return d.autosize()
# =====================================================================
def step3b():
    """STEP 3-2 기능 흐름 — 강의안(Day5 p.73) 순서도 형식.

    규칙: 상자 하나에 짧은 동사구 한 줄, 판단은 마름모 + Y/N, 설명 문장은 그림에 넣지 않는다.
    토픽 이름과 메시지 타입만 화살표 위에 적는다.
    """
    d = Diagram('STEP3-2 전체 흐름', 1660, 100,
                'STEP 3-2 — 전체 기능 흐름 (Functional Process Flow)',
                '상자 = 처리 · 마름모 = 판단(Y/N) · 화살표 위 = 토픽 / 메시지 타입 · SDD 3.3')

    C1X, C1W = 24, 250          # 웹캠 PC
    C2X, C2W = 300, 250         # 관제 PC
    C3X, C3W = 576, 712         # 로봇 제어 PC
    N1X, N1W = 592, 320         #  (안) 인지·미션 노드
    N2X, N2W = 952, 320         #  (안) 복귀·안전·Nav2
    C4X, C4W = 1420, 200        # 로봇 하드웨어 (안전 경로 라벨 통로를 두고 띄운다)
    TOP = 330

    BH = 30                     # 처리 상자 높이
    DH = 44                     # 판단 마름모 높이
    GAP = 17                    # Y/N 글자가 들어갈 만큼 띄운다

    D, S, H, N2, SF = 'dec', 'step', 'hwstep', 'n2step', 'sfstep'
    R, B, P, G, O = '#C0392B', '#3576C8', '#6B4FA8', '#5A5F6A', '#B06810'

    def chain(bx, bw, y0, items, color=G, link=True):
        """items = (id, 라벨, 스타일, 'rect'|'dia', 아래로 나가는 가지 라벨)"""
        prev, prev_lbl, y = None, '', y0
        for nid, lbl, st, shape, out in items:
            h = DH if shape == 'dia' else BH
            n = d.box(nid, bx, y, bw, [lbl], st,
                      h=h, shape='diamond' if shape == 'dia' else 'rect', align='center')
            if prev and link:
                d.edge(prev, nid, prev_lbl, 'bottom-top', color=color,
                       width=1.2, label_dx=13, label_dy=-5)
            prev, prev_lbl = nid, out
            y = y + h + GAP
        return y - GAP

    def group(nid, x, w, y0, title, items, color=G, style='nodebox', link=True):
        bot = chain(x + 14, w - 120, y0 + 28, items, color, link)
        d.band(nid, x, y0, w, bot - y0 + 12, title, style, dash=False)
        return bot + 12

    def loop(dia, back_to, bx, bw, label='N'):
        """되돌아가는 가지 — 마름모 오른쪽으로 나가 이전 상자로 다시 들어간다."""
        lane = bx + bw + 22
        d.edge(dia, back_to, label, 'right-right', color='#8A8F98', width=1.1,
               via=[(lane, d.nodes[dia]['y'] + d.nodes[dia]['h'] / 2),
                    (lane, d.nodes[back_to]['y'] + d.nodes[back_to]['h'] / 2)],
               label_dx=13, label_dy=-3)

    def stop(nid, dia, bx, bw, label, txt='정지'):
        """빠져나가는 가지 — 마름모 오른쪽의 종단."""
        y = d.nodes[dia]['y'] + d.nodes[dia]['h'] / 2 - 13
        d.box(nid, bx + bw + 30, y, 74, [txt], 'term', h=26, shape='stadium', align='center')
        d.edge(dia, nid, label, 'right-left', color='#8A8F98', width=1.1, label_dy=-4)

    # ─────────── 열 1 · 웹캠 PC ───────────
    b1, w1 = C1X + 12, C1W - 24
    y = TOP + 34
    y = group('nb_w1', b1, w1, y, 'webcam_register  (PC-W1)', [
        ('w1a', '웹캠1 프레임 10 Hz', S, 'rect', ''),
        ('w1b', '사람?', D, 'dia', 'Y'),
        ('w1c', 'ByteTrack ID 부여', S, 'rect', ''),
        ('w1d', '0.3 s 연속?', D, 'dia', 'Y'),
        ('w1e', 'HSV 히스토그램 128', S, 'rect', ''),
        ('w1f', '서술자 발행', S, 'rect', ''),
    ]) + 20
    loop('w1b', 'w1a', b1 + 14, w1 - 120)
    stop('w1d_x', 'w1d', b1 + 14, w1 - 120, 'N', '무시')
    y = group('nb_w2', b1, w1, y, 'webcam_hint  (PC-W2)', [
        ('w2a', '웹캠2 프레임', S, 'rect', ''),
        ('w2b', '등록 트랙?', D, 'dia', 'Y'),
        ('w2c', '호모그래피 환산', S, 'rect', ''),
        ('w2d', '힌트 발행 10 Hz', S, 'rect', ''),
    ])
    loop('w2b', 'w2a', b1 + 14, w1 - 120)

    # ─────────── 열 2 · 관제 PC ───────────
    b2, w2 = C2X + 12, C2W - 24
    y = TOP + 34
    y = group('nb_ui', b2, w2, y, 'control_ui  (Flask)', [
        ('ua', '상태 표시', S, 'rect', ''),
        ('ub', '자재 요청', S, 'rect', ''),
        ('uc', '종료 명령', S, 'rect', ''),
    ]) + 18
    y = group('nb_fm', b2, w2, y, 'fleet_manager', [
        ('fa', '로봇 배정', S, 'rect', ''),
        ('fb', '배터리 ≥ 25 %?', D, 'dia', 'Y'),
        ('fc', '미션 발행', S, 'rect', ''),
        ('fd', '종료 전달', S, 'rect', ''),
        ('fe', '상태 집계 1 Hz', S, 'rect', ''),
    ]) + 18
    stop('fb_x', 'fb', b2 + 14, w2 - 120, 'N', '거부')
    y = group('nb_zs', b2, w2, y, 'zone_semaphore', [
        ('za', '토큰 요청 수신', S, 'rect', ''),
        ('zb', '구역 비었나?', D, 'dia', 'Y'),
        ('zc', 'granted 반환', S, 'rect', ''),
        ('zd', '60 s 초과 회수', S, 'rect', ''),
    ]) + 18
    loop('zb', 'za', b2 + 14, w2 - 120)
    y = group('nb_kpi', b2, w2, y, 'kpi_logger', [
        ('ka', '상태 · 개입 구독', S, 'rect', ''),
        ('kb', 'CSV · DB 기록', S, 'rect', ''),
    ])

    # ─────────── 열 3-A · 인지 · 미션 ───────────
    y = TOP + 34
    y = group('nb_det', N1X, N1W, y, 'amr_detection', [
        ('da', 'RGB + Depth 동기화', S, 'rect', ''),
        ('db', 'worker 검출?', D, 'dia', 'Y'),
        ('dc', '거리 → map 좌표', S, 'rect', ''),
        ('dd', '유사도 ≥ 임계?', D, 'dia', 'Y'),
        ('de', '대상 발행', S, 'rect', ''),
    ], color=B) + 18
    loop('db', 'da', N1X + 14, N1W - 120)
    stop('dd_x', 'dd', N1X + 14, N1W - 120, 'N', '대기')
    y = group('nb_sh', N1X, N1W, y, 'shuttle_mission', [
        ('sa', '미션 수신?', D, 'dia', 'Y'),
        ('sb', '베이 이동', S, 'rect', ''),
        ('sc', 'ArUco 일치?', D, 'dia', 'Y'),
        ('sd', '적재 체류 5 s', S, 'rect', ''),
        ('se', 'P1 토큰 획득', S, 'rect', ''),
        ('sf', '대기공간 이동', S, 'rect', ''),
        ('sg', 'P1 토큰 반납', S, 'rect', ''),
        ('sh', 'WAITING 발행', S, 'rect', ''),
    ], color=B) + 18
    loop('sc', 'sb', N1X + 14, N1W - 120)
    y = group('nb_fo', N1X, N1W, y, 'follow_target', [
        ('oa', '도착 · 페어링?', D, 'dia', 'Y'),
        ('ob', '접근', S, 'rect', ''),
        ('oc', '추종 0.5 m', S, 'rect', ''),
        ('od', '대상 놓침?', D, 'dia', 'Y'),
        ('oe', '회전 탐색', S, 'rect', ''),
        ('of', '페어링 해제', S, 'rect', ''),
    ], color=B)
    bot_a = y
    loop('od', 'oc', N1X + 14, N1W - 120, 'N')

    # ─────────── 열 3-B · 복귀 · 안전 · Nav2 ───────────
    y = TOP + 34
    y = group('nb_rd', N2X, N2W, y, 'return_dock', [
        ('ra', 'P1 · DOCK 토큰', S, 'rect', ''),
        ('rb', '접근 지점 이동', S, 'rect', ''),
        ('rc', '도킹', S, 'rect', ''),
        ('rd', '성공?', D, 'dia', 'Y'),
        ('re', '토큰 반납', S, 'rect', ''),
    ], color=B) + 18
    loop('rd', 'rc', N2X + 14, N2W - 120, 'N (2회)')
    y = group('nb_sw', N2X, N2W, y, 'safety_watchdog  (온보드)', [
        ('wa', 'scan 구독 ±35°', SF, 'rect', ''),
        ('wb', '0.30 m 이내?', D, 'dia', 'Y'),
        ('wc', 'estop = True', SF, 'rect', ''),
        ('wd', 'cmd_vel 0 @ 20 Hz', SF, 'rect', ''),
        ('we', '0.45 m 밖 → 해제', SF, 'rect', ''),
    ], color=R) + 18
    loop('wb', 'wa', N2X + 14, N2W - 120)
    y = group('nb_nav', N2X, N2W, y, 'Nav2 스택', [
        ('va', 'map_server', N2, 'rect', ''),
        ('vb', 'amcl', N2, 'rect', ''),
        ('vc', 'bt_navigator', N2, 'rect', ''),
        ('vd', 'planner_server', N2, 'rect', ''),
        ('ve', 'costmap', N2, 'rect', ''),
        ('vf', 'controller_server', N2, 'rect', ''),
        ('vg', 'velocity_smoother', N2, 'rect', ''),
    ], color=P) + 18
    bot_b = group('nb_dk', N2X, N2W, y, 'opennav_docking', [
        ('ga', 'undock', N2, 'rect', ''),
        ('gb', 'dock', N2, 'rect', ''),
    ], color=P)

    # ─────────── 열 4 · 로봇 하드웨어 ───────────
    group('nb_hw', C4X + 10, C4W - 20, TOP + 34, 'robot3  (TurtleBot4)', [
        ('ha', 'OAK-D Pro', H, 'rect', ''),
        ('hb', 'RPLIDAR A1', H, 'rect', ''),
        ('hc', 'Create3', H, 'rect', ''),
        ('hd', '구동부 v ≤ 0.3', H, 'rect', ''),
    ], color=G, style='hwbox', link=False)

    # ─────────── PC 컨테이너 ───────────
    def wrap(nid, x, w, title, style, members, pad=14):
        top = min(d.nodes[m]['y'] for m in members) - 32
        bot = max(d.nodes[m]['y'] + d.nodes[m]['h'] for m in members) + pad
        d.band(nid, x, top, w, bot - top, title, style, dash=False)
        return bot

    wrap('pc_w', C1X, C1W, 'PC-W1 / PC-W2 — 웹캠', 'pcbox', ['nb_w1', 'nb_w2'])
    wrap('pc_f', C2X, C2W, 'PC-F — 관제', 'fbox', ['nb_ui', 'nb_fm', 'nb_zs', 'nb_kpi'])
    pcr_bot = wrap('pc_r', C3X, C3W, 'PC-R3 — 로봇3 제어 PC  (namespace = /robot3)', 'pcbox',
                   ['nb_det', 'nb_sh', 'nb_fo', 'nb_rd', 'nb_sw', 'nb_nav', 'nb_dk'])
    wrap('pc_h', C4X, C4W, '로봇 온보드', 'hwbox', ['nb_hw'])
    d.order.sort(key=lambda t: (0, 0 if t[1] in ('pc_w', 'pc_f', 'pc_r', 'pc_h') else 1)
                 if t[0] == 'band' else (1, 0))

    # ═══════════ 위 통로 — 관제 · 웹캠 → 로봇 ═══════════
    def down(src, sf, dst, tf, lane, lines, lx, color):
        sx = d.anchor(src, 'top@%g' % sf)[0]
        tx = d.anchor(dst, 'top@%g' % tf)[0]
        d.edge(src, dst, '', 'top@%g-top@%g' % (sf, tf), color=color, width=1.5,
               via=[(sx, lane), (tx, lane)], label_lines=lines, label_x=lx)

    down('nb_fm', 0.50, 'nb_fo', 0.45, 100,
         ['mission/finish', 'std_msgs/Bool'], 830, O)
    down('nb_fm', 0.70, 'nb_rd', 0.45, 138,
         ['mission/finish', '복귀 시작'], 1120, O)
    down('nb_fm', 0.30, 'nb_sh', 0.55, 176,
         ['mission/assigned', 'mro_interfaces/Mission'], 700, O)
    down('nb_w2', 0.50, 'nb_fo', 0.75, 214,
         ['/webcam2/target_hint', 'mro_interfaces/TargetHint'], 480, B)
    down('nb_w1', 0.40, 'nb_det', 0.30, 252,
         ['/webcam1/worker_descriptor', 'mro_interfaces/WorkerDescriptor'], 640, B)
    down('nb_w1', 0.70, 'nb_fm', 0.15, 290,
         ['/webcam1/worker_descriptor', '배정 근거'], 250, B)

    # ═══════════ 아래 통로 — 로봇 → 관제 · 센서 ═══════════
    def up(src, sf, dst, tf, lane, lines, lx, color, width=1.5):
        sx = d.anchor(src, 'bottom@%g' % sf)[0]
        tx = d.anchor(dst, 'bottom@%g' % tf)[0]
        d.edge(src, dst, '', 'bottom@%g-bottom@%g' % (sf, tf), color=color, width=width,
               via=[(sx, lane), (tx, lane)], label_lines=lines, label_x=lx)

    b0 = pcr_bot + 40
    up('nb_sh', 0.25, 'nb_zs', 0.35, b0,
       ['/fleet/zone_token  (서비스)', 'mro_interfaces/ZoneToken — P1'], 620, B)
    up('nb_rd', 0.30, 'nb_zs', 0.60, b0 + 40,
       ['/fleet/zone_token  (서비스)', 'DOCK — 한 번에 한 대'], 780, B)
    up('nb_sh', 0.75, 'nb_fm', 0.85, b0 + 80,
       ['shuttle/status · approach/status', 'std_msgs/String'], 640, G)
    up('nb_hw', 0.30, 'nb_det', 0.70, b0 + 120,
       ['oakd/rgb · stereo · camera_info', 'sensor_msgs — BEST_EFFORT'], 1120, G)
    # 안전 경로는 로봇 온보드에서 닫힌다 — 아래 통로로 돌리지 않고 바로 잇는다
    mid_hw = (N2X + N2W + C4X) / 2
    d.edge('nb_hw', 'nb_sw', '', 'left@0.30-right@0.25', color=R, width=1.5, mid=mid_hw,
           label_lines=['scan', 'sensor_msgs/LaserScan'], label_x=mid_hw)
    d.edge('nb_sw', 'nb_hw', '', 'right@0.80-left@0.80', color=R, width=2.2, mid=mid_hw + 8,
           label_lines=['cmd_vel = 0  @ 20 Hz', 'estop 중 Nav2 출력을 덮어쓴다'], label_x=mid_hw - 40)

    # ═══════════ 열 안쪽 연결 ═══════════
    gi = (N1X + N1W + N2X) / 2
    d.edge('nb_sh', 'nb_nav', '', 'right@0.35-left@0.35', color=P, width=1.5, mid=gi)
    d.edge('nb_fo', 'nb_nav', '', 'right@0.60-left@0.60', color=P, width=1.5, mid=gi)
    d.edge('nb_rd', 'nb_dk', '', 'bottom@0.85-top@0.15', color=P, width=1.5)
    d.edge('nb_nav', 'nb_hw', '', 'right@0.85-left@0.55', color=P, width=1.6,
           mid=(N2X + N2W + C4X) / 2,
           label_lines=['cmd_vel', 'TwistStamped'], label_x=(N2X + N2W + C4X) / 2)
    d.edge('nb_det', 'nb_fo', '', 'left@0.5-left@0.30', color=B, width=1.5,
           via=[(N1X - 14, d.nodes['nb_det']['y'] + d.nodes['nb_det']['h'] * 0.5),
                (N1X - 14, d.nodes['nb_fo']['y'] + d.nodes['nb_fo']['h'] * 0.30)],
           label_lines=['target/paired'], label_x=N1X - 14)
    d.edge('nb_sh', 'nb_fo', '', 'left@0.85-left@0.60', color=B, width=1.5,
           via=[(N1X - 30, d.nodes['nb_sh']['y'] + d.nodes['nb_sh']['h'] * 0.85),
                (N1X - 30, d.nodes['nb_fo']['y'] + d.nodes['nb_fo']['h'] * 0.60)],
           label_lines=['shuttle/arrived'], label_x=N1X - 30)

    # ═══════════ 범례 ═══════════
    lg_y = b0 + 236
    d.text(C1X, lg_y, '기호', 12, '#5A5F6A', bold=True)
    d.box('lg_r', C1X, lg_y + 10, 92, ['처리'], 'step', h=26, align='center')
    d.box('lg_d', C1X + 104, lg_y + 4, 92, ['판단'], 'dec', h=38, shape='diamond', align='center')
    d.box('lg_t', C1X + 208, lg_y + 10, 92, ['종단'], 'term', h=26, shape='stadium', align='center')
    d.text(C1X + 314, lg_y + 27, 'Y / N = 판단의 두 갈래   ·   화살표 위 상자 = 토픽 이름 / 메시지 타입',
           11, '#3A3E46')
    d.text(C1X, lg_y + 60,
           '색 — 주황: 관제 명령 · 파랑: 인지 · 웹캠 데이터 · 보라: Nav2 액션 · 빨강: 안전 경로 · 회색: 상태 · 센서',
           11, '#5A5F6A')
    d.text(C1X, lg_y + 80,
           '빨간 경로(scan → safety_watchdog → cmd_vel)만 로봇 온보드에서 닫힌다. '
           'Wi-Fi 나 웹캠 PC 가 죽어도 로봇은 멈춘다 — 무사고 0 건(NF-06)을 지탱하는 구조다.',
           11, '#3A3E46')
    d.text(C1X, lg_y + 104,
           'robot4 는 생략 — 네임스페이스만 다르고 구성이 같다. 두 대가 만나는 곳은 zone_semaphore 의 P1 · DOCK 뿐이다.',
           11, '#8A8F98')
    d.text(C1X, lg_y + 126,
           '생성: tools/build_diagrams.py   ·   근거: docs/architecture.md, docs/interfaces.md 4절',
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
