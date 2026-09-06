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
        'USB 웹캠 640×480 (관제탑 전경)',
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
    d = Diagram('STEP3 시나리오', 1320, 100,
                'STEP 3 — 운영 시나리오 S1~S10 흐름과 예외 처리',
                '가운데가 정상 흐름 · 왼쪽은 수행 노드 · 오른쪽은 예외와 그때의 처리 · SDD 3.3 / BRD 3.2')

    LX, LW = 24, 320
    MX, MW = 392, 380
    RX, RW = 840, 456

    d.text(LX + 6, 92, '수행 노드', 12, '#5A5F6A', bold=True)
    d.text(MX + 6, 92, '정상 흐름  S1 → S10', 12, '#5A5F6A', bold=True)
    d.text(RX + 6, 92, '예외 · 분기와 그때의 처리', 12, '#5A5F6A', bold=True)

    state = {'y': 104}
    spine = []

    def step(nid, lines, style='pc', shape='rect', node=None,
             exc_lines=None, exc_style='exc', exc_label='아니오', gap=26):
        y = state['y']
        n = d.box(nid, MX, y, MW, lines, style, shape=shape, align='center')
        spine.append(nid)
        hs = [n['h']]
        if node:
            hs.append(d.box(nid + '_n', LX, y, LW, node, 'ext')['h'])
        if exc_lines:
            hs.append(d.box(nid + '_e', RX, y, RW, exc_lines, exc_style)['h'])
            d.edge(nid, nid + '_e', exc_label, 'right-left', color='#C0392B')
        state['y'] = y + max(hs) + gap
        return n

    step('s1', ['S1  진입 · 등록',
                '작업자가 관제 구역에 들어서면 웹캠1 이 외형을 등록하고',
                '로봇 한 대를 배정한다  ·  등록 완료 ≤ 10 초'],
         node=['webcam_register  (PC-W1)',
               '→ fleet_manager  (PC-F)',
               '/webcam1/worker_descriptor',
               'HSV 히스토그램 H16 × S8 = 128',
               '인형 전체(머리~바지 밑단)를 쓴다',
               '담당 한세교 · 김학민'],
         exc_lines=['두 작업자의 서술자가 너무 비슷할 때',
                    '같은 인형끼리의 유사도와 다른 인형끼리의 유사도 분포가 겹치면',
                    '서술자 자체를 손봐야 한다는 신호다. 조명을 맞추고 다시 등록한다.',
                    '상의 색 두 가지는 휘도를 맞춰 인쇄해 색만으로 갈리게 한다.'],
         exc_label='')

    step('s2', ['S2  준비',
                '배정된 로봇이 도크에서 빠져나와 대기 상태가 된다',
                'undock  →  shuttle/status = READY'],
         node=['fleet_manager → shuttle_mission',
               'opennav_docking  undock',
               '기동할 때 자신의 dock_pose 로',
               'AMCL 초기 위치를 설정한다'],
         exc_lines=['두 로봇의 맵 좌표계가 어긋나는 사고  (I-013)',
                    '각 로봇이 (0,0) 으로 초기화하면 같은 좌표가 서로 다른 장소를',
                    '가리키게 되어 세마포어도 waypoint 도 전부 무의미해진다.',
                    '→ 맵 원점은 SLAM 작성 시점 하나로 고정하고, 각 로봇은',
                    '   arena.yaml 의 dock_zone.pose 값으로 초기화한다.'],
         exc_label='')

    step('s3', ['S3  자재 요청',
                '관제 UI 에서 작업자와 창고 베이를 골라 요청한다',
                'RequestMission → Mission(ASSIGNED)  ·  접수 ≤ 2 초'],
         node=['control_ui  (Flask :5000)',
               '→ fleet_manager',
               'mission/assigned 발행',
               '담당 김민준'])

    step('s4', ['S4  셔틀 주행',
                '도크에서 창고 베이 A / B / C 의 고정 waypoint 로 이동한다',
                '도크와 창고는 벽 없이 이어져 여기서는 토큰이 필요 없다'],
         node=['shuttle_mission',
               'Nav2  navigate_to_pose',
               'arena.yaml 의 베이 waypoint'])

    step('d2', ['베이 ArUco ID 가 일치하는가?',
                '거리 ≤ 1.5 m · 정면 ±30°'],
         'dec', shape='diamond',
         node=['aruco_detector',
               'aruco/id · aruco/distance',
               'ID 1/2/3 = 베이 A/B/C',
               '선반 뒤 벽면, 높이 20 cm'],
         exc_lines=['도착 판정 실패',
                    '다시 접근해 읽는다. T_lim 120 초를 넘기면 미션을 FAILED 로',
                    '확정하고 운영자에게 알린다. 늦게라도 도착한 사실은',
                    'arrived_late 로 따로 남겨 성공으로 이중 계상하지 않는다.'])

    step('d1', ['P1 병목 토큰을 얻었는가?',
                '로봇 구역 → 중앙 복도 출구 0.79 m · 5 초 체류 뒤 요청 · 한 번에 한 대만'],
         'dec', shape='diamond',
         node=['zone_semaphore  (PC-F 실행)',
               'srv/ZoneToken  acquire',
               '패키지는 관제, 소유는 주행',
               '담당 박병후 · 이시율'],
         exc_lines=['E-3  교착 회피',
                    '거절되면 2 초 뒤 다시 요청한다. 폴리곤에 들어가기 전에 얻고,',
                    '완전히 빠져나온 뒤에 반납한다. 60 초를 넘겨 쥐고 있으면',
                    '세마포어가 강제로 회수하고 이벤트를 남긴다.',
                    'S4 창고에서 나올 때와 S10 돌아갈 때 두 번 통과하므로 교착을 여기서 본다.'])

    step('s4b', ['S4′  적재 · 합류 지점 이동',
                 '적재(5 초 체류)를 마친 로봇이 P1 을 지나 대기공간(Z-05)에 닿으면',
                 'P1 토큰을 반납한다  ·  shuttle/arrived 발행'],
         node=['shuttle_mission',
               'zone_semaphore  release',
               '실패해도 토큰은 반드시 반납한다'])

    step('d3', ['재식별 게이트를 통과했는가?',
                '외형 유사도 ≥ reid_threshold'],
         'dec', shape='diamond',
         node=['amr_detection  (재식별)',
               'core/reid.hsv_hist() 공유 함수',
               'target/paired · target/worker_id',
               '담당 정정인 · 민범진'],
         exc_lines=['E-2  엉뚱한 사람을 따라가지 않기',
                    '임계에 못 미치면 페어링하지 않고 그 자리에서 대기한다.',
                    '잘못된 작업자를 따라가는 일은 0 건이 절대 기준이라,',
                    '확신이 없을 때 멈추는 쪽이 항상 옳다.',
                    '폴백 — 관제 UI 에서 운영자가 대상을 수동으로 지정한다.'])

    step('s5', ['S5  합류',
                '배정된 작업자를 확인하고 접근한다',
                'APPROACH_1 → APPROACH_2 → ARRIVED'],
         node=['follow_target',
               'approach/status'])

    step('s6', ['S6 ~ S8  추종',
                '중앙 복도(P3 0.93 m)를 지나 작업공간까지 따라간다',
                '간격 0.5 m 유지  ·  approach/status = FOLLOWING'],
         node=['follow_target',
               'Nav2  navigate_to_pose',
               '추종 경로에는 토큰을 두지 않는다',
               '(기다리다 작업자를 놓치기 때문)'])

    step('d4', ['대상을 놓쳤는가?  (Target Lost)'],
         'dec', shape='diamond',
         node=['amr_detection → follow_target',
               'target/visible = false'],
         exc_label='예',
         exc_lines=['E-1  재탐지',
                    '작업공간 안이면 웹캠2 힌트가 있다 → 그 방향으로 회전 탐색.',
                    '중앙 복도는 웹캠2 시야 밖이라 힌트가 없다 → 제자리 탐색.',
                    '두 조건의 재획득 시간을 비교하는 것이 시험 TS-19 다.',
                    'T_lim 을 넘기면 STANDBY 로 들어가 운영자를 기다린다.',
                    '가림은 교차하는 다른 작업자(RC카 2호)로 만든다 —',
                    '벽으로 만들면 웹캠2 의 바닥 시야까지 함께 막힌다.'])

    step('d5', ['전방 0.30 m 안에 장애물이 있는가?'],
         'dec', shape='diamond',
         node=['safety_watchdog  (온보드)',
               'scan 만 구독 · 전방 ±35°',
               '어느 단계에서든 항상 동작한다'],
         exc_style='safety', exc_label='예',
         exc_lines=['E-4  안전 정지',
                    'safety/estop 을 올리고 cmd_vel 에 0 을 20 Hz 로 덮어쓴다.',
                    '0.45 m 밖으로 벗어나면 스스로 재개한다.',
                    '이 정지는 사람이 개입한 것이 아니므로 개입 지표(IPH)에',
                    '계상하지 않는다. 웹캠이나 Wi-Fi 가 죽어도 이 경로는 살아 있다.'])

    step('s9', ['S9  작업 종료 발행',
                '작업자가 종료 버튼을 누르면 추종을 즉시 멈춘다',
                'RETURN / RETURN_ALL → mission/finish → 페어링 해제 ≤ 1 초'],
         'pcf',
         node=['control_ui → fleet_manager',
               '→ follow_target  (즉시 중단)',
               '→ return_dock  (복귀 시작)'])

    step('d6', ['복귀 통로와 도크 토큰을 얻었는가?',
                'P1 통과 → DOCK 은 한 번에 한 대'],
         'dec', shape='diamond',
         node=['return_dock',
               'zone_semaphore  P1 · DOCK'],
         exc_lines=['E-7  순차 도킹',
                    '도크 두 대의 간격이 0.84 m(도킹점 기준) 뿐이라 두 대가 동시에 들어가면',
                    'IR 신호가 서로 간섭한다. DOCK 토큰으로 한 대씩 들여보내고',
                    '나머지는 홀딩 포인트에서 기다린다.',
                    '차폐판은 두지 않는다 — TS-U12 10회 오도킹 0 (9/6).'])

    step('s10', ['S10  도킹',
                 '자기 도크 정면의 접근 지점을 거쳐 도킹한다',
                 'opennav_docking  dock'],
         node=['return_dock',
               'T_ret 180 초 · 재시도 2회'])

    step('d7', ['도킹에 성공했는가?'],
         'dec', shape='diamond',
         node=['shuttle/status 로 보고'],
         exc_lines=['도킹 실패',
                    '최대 2회까지 다시 시도한다. 그래도 안 되면 FAILED 를 발행하고',
                    '멈춘다. 성공이든 실패든 DOCK 토큰은 반드시 반납한다 —',
                    '반납하지 않으면 다른 로봇이 영영 들어오지 못한다.'])

    step('end', ['DOCKED  ·  Phase 1 종료',
                 '두 대 모두 도킹 완료 · KPI 로그 확정'],
         'term', shape='stadium', gap=12)

    for a, b in zip(spine, spine[1:]):
        lab = ''
        if a.startswith('d'):
            lab = '아니오' if a in ('d4', 'd5') else '예'
        d.edge(a, b, lab, 'bottom-top',
               color='#2E8B57' if a.startswith('d') else '#5A5F6A')

    d.text(24, state['y'] + 6,
           '전 구간 공통 — 모든 로그와 CSV 는 ROS 시각을 쓴다.  안전 정지 뒤의 자동 재개는 개입으로 계상하지 않는다.  '
           '생성: tools/build_diagrams.py',
           10.5, '#8A8F98')
    return d.autosize()


# =====================================================================
# STEP 3-2 — 전체 노드·토픽 흐름 (교재 STEP 3 형식)
# =====================================================================
def step3b():
    d = Diagram('STEP3-2 전체 흐름', 2340, 100,
                'STEP 3-2 — 전체 기능 흐름: PC·노드 내부 처리와 그 사이를 오가는 토픽',
                '위쪽 통로 = 관제·웹캠에서 로봇으로 내려가는 명령 · 아래쪽 통로 = 로봇에서 올라오는 상태와 센서 · SDD 3.3')

    C1X, C1W = 30, 390          # 웹캠 PC
    C2X, C2W = 480, 390         # 관제 PC
    C3X, C3W = 930, 1040        # 로봇 제어 PC
    N1X, N1W = 952, 450         #  (안) 우리 노드
    N2X, N2W = 1532, 416        #  (안) Nav2 — 우리 노드와의 사이를 넓혀 액션 표기를 넣는다
    C4X, C4W = 2030, 280        # 로봇 하드웨어
    TOP = 360                   # 컨테이너 시작 y

    def flow(x, w, y0, items, gap=13, color='#5A5F6A'):
        prev, y = None, y0
        for nid, lines, st, shape in items:
            n = d.box(nid, x, y, w, lines, st, shape=shape, align='center')
            if prev:
                d.edge(prev, nid, '', 'bottom-top', color=color, width=1.2)
            prev = nid
            y = n['y'] + n['h'] + gap
        return y - gap

    def nodebox(nid, x, w, y0, title, items, gap=13, color='#5A5F6A', style='nodebox'):
        bot = flow(x + 14, w - 28, y0 + 30, items, gap, color)
        d.band(nid, x, y0, w, bot - y0 + 14, title, style, dash=False)
        return bot + 14

    D, S, H, N2, SF = 'dec', 'step', 'hwstep', 'n2step', 'sfstep'
    R, B, P, G, O = '#C0392B', '#3576C8', '#6B4FA8', '#5A5F6A', '#B06810'

    # ─────────── 열 1 · 웹캠 PC ───────────
    y = TOP + 38
    y = nodebox('nb_w1', C1X + 12, C1W - 24, y, 'webcam_register  (PC-W1)', [
        ('w1a', ['웹캠1 프레임', '캡처 30 fps → 처리 10 Hz'], S, 'rect'),
        ('w1b', ['사람이 보이는가?'], D, 'diamond'),
        ('w1c', ['ByteTrack 으로 트랙 ID 부여'], S, 'rect'),
        ('w1d', ['0.3 s 연속으로 잡히는가?', '아니오 → 무시 (오검출 억제)'], D, 'diamond'),
        ('w1e', ['인형 전체(머리~바지 밑단)에서', 'HSV 히스토그램 H16×S8 = 128'], S, 'rect'),
        ('w1f', ['WorkerDescriptor 발행'], S, 'rect'),
    ]) + 26
    y = nodebox('nb_w2', C1X + 12, C1W - 24, y, 'webcam_hint  (PC-W2)', [
        ('w2a', ['웹캠2 프레임', '작업공간 + P3 입구 (중앙 복도는 시야 밖)'], S, 'rect'),
        ('w2b', ['등록된 트랙이 보이는가?'], D, 'diamond'),
        ('w2c', ['호모그래피로 바닥 좌표 환산'], S, 'rect'),
        ('w2d', ['TargetHint 발행', '10 Hz · 유효 2 s'], S, 'rect'),
    ]) + 26
    d.box('c1note', C1X + 12, y, C1W - 24, [
        '웹캠은 안전에 관여하지 않는다',
        '여기서 나온 좌표는 재탐지 방향과 KPI 계측에만 쓴다.',
        '정지·회피 판단에 쓰지 않는 것이 설계 원칙이다 (NF-07).',
    ], 'note')

    # ─────────── 열 2 · 관제 PC ───────────
    y = TOP + 38
    y = nodebox('nb_ui', C2X + 12, C2W - 24, y, 'control_ui  (Flask :5000)', [
        ('ua', ['운영자 화면 — 2대 상태 표시 (뷰어)'], S, 'rect'),
        ('ub', ['자재 요청 → RequestMission'], S, 'rect'),
        ('uc', ['종료 버튼 → OperatorCommand', 'RETURN / RETURN_ALL'], S, 'rect'),
    ]) + 24
    y = nodebox('nb_fm', C2X + 12, C2W - 24, y, 'fleet_manager', [
        ('fa', ['서술자 수신 → 로봇 배정', '트랙 ID 작은 쪽 먼저 (결정론적)'], S, 'rect'),
        ('fb', ['배터리 25 % 이상인가?', '아니오 → 미션 거부 · 사유 기록'], D, 'diamond'),
        ('fc', ['mission/assigned 발행'], S, 'rect'),
        ('fd', ['종료 명령 수신 시 mission/finish 발행'], S, 'rect'),
        ('fe', ['FleetStatus 1 Hz 발행'], S, 'rect'),
    ]) + 24
    y = nodebox('nb_zs', C2X + 12, C2W - 24, y, 'zone_semaphore  (소유: 주행)', [
        ('za', ['ZoneToken 요청 수신 — 구역은 P1 · DOCK 둘뿐'], S, 'rect'),
        ('zb', ['구역이 비어 있는가?', '아니오 → 대기열 등록, 2 s 후 재요청'], D, 'diamond'),
        ('zc', ['granted 반환'], S, 'rect'),
        ('zd', ['보유 60 s 초과 시 강제 회수'], S, 'rect'),
    ]) + 24
    y = nodebox('nb_kpi', C2X + 12, C2W - 24, y, 'kpi_logger', [
        ('ka', ['상태 · 미션 · 개입 · 힌트 구독'], S, 'rect'),
        ('kb', ['CSV + SQLite 기록 — 전부 ROS 시각 기준'], S, 'rect'),
    ])

    # ─────────── 열 3 · 우리 노드 ───────────
    y = TOP + 38
    y = nodebox('nb_det', N1X, N1W, y, 'amr_detection  (mro_perception)', [
        ('da', ['RGB + Depth 동기화 (slop 0.1 s)'], S, 'rect'),
        ('db', ['worker 를 검출했는가?'], D, 'diamond'),
        ('dc', ['ROI p20 역깊이 보정 → 거리', 'amcl_pose 로 map 좌표 변환'], S, 'rect'),
        ('dd', ['등록 서술자와 유사도 ≥ 임계?', '아니오 → 페어링하지 않고 대기'], D, 'diamond'),
        ('de', ['target/paired · map_point · distance 발행'], S, 'rect'),
    ]) + 22
    y = nodebox('nb_sh', N1X, N1W, y, 'shuttle_mission  (mro_navigation)', [
        ('sa', ['mission/assigned 를 받았는가?'], D, 'diamond'),
        ('sb', ['베이 waypoint 로 이동 (도크·창고 사이는 토큰 없음)'], S, 'rect'),
        ('sc', ['ArUco ID 가 요청 베이와 같은가?', '아니오 → 재접근, T_lim 초과 시 실패'], D, 'diamond'),
        ('sd', ['5 s 체류 → P1 토큰 → 대기공간 → 반납', 'shuttle/arrived 발행'], S, 'rect'),
    ], color=B) + 22
    y = nodebox('nb_fo', N1X, N1W, y, 'follow_target  (mro_navigation)', [
        ('oa', ['도착했고 페어링됐는가?'], D, 'diamond'),
        ('ob', ['APPROACH → 간격 0.5 m 유지 → FOLLOWING'], S, 'rect'),
        ('oc', ['대상을 놓쳤는가?'], D, 'diamond'),
        ('od', ['힌트 방향으로 회전 탐색', '없으면 제자리 탐색 → STANDBY'], S, 'rect'),
        ('oe', ['mission/finish 수신 시', '즉시 페어링 해제 (≤ 1 s)'], S, 'rect'),
    ], color=B) + 22
    y = nodebox('nb_rd', N1X, N1W, y, 'return_dock  (mro_navigation)', [
        ('ra', ['P1 → DOCK 토큰 (한 번에 한 대)'], S, 'rect'),
        ('rb', ['도크 정면 접근 지점 → dock'], S, 'rect'),
        ('rc', ['도킹에 성공했는가?', '아니오 → 최대 2회 재시도'], D, 'diamond'),
        ('rd', ['성공·실패와 무관하게 토큰 반납'], S, 'rect'),
    ], color=B) + 22
    bot_ours = nodebox('nb_sw', N1X, N1W, y, 'safety_watchdog  (온보드 안전)', [
        ('wa', ['scan 만 구독 · 전방 ±35°'], SF, 'rect'),
        ('wb', ['0.30 m 이내가 2프레임 연속인가?'], D, 'diamond'),
        ('wc', ['safety/estop = True', 'cmd_vel 에 0 을 20 Hz 로 덮어쓴다'], SF, 'rect'),
        ('wd', ['0.45 m 밖 5프레임 → 스스로 해제'], SF, 'rect'),
    ], color=R)

    # ─────────── 열 3 · Nav2 ───────────
    y = TOP + 38
    y = nodebox('nb_nav', N2X, N2W, y, 'Nav2 스택  (turtlebot4_navigation)', [
        ('va', ['map_server — auto_map.yaml'], N2, 'rect'),
        ('vb', ['amcl — scan + odom → amcl_pose'], N2, 'rect'),
        ('vc', ['bt_navigator — action  navigate_to_pose'], N2, 'rect'),
        ('vd', ['planner_server — 전역 경로'], N2, 'rect'),
        ('ve', ['costmap — inflation 0.25 (받침대 덮기, P1 하드 자유폭 0.44)'], N2, 'rect'),
        ('vf', ['controller_server — 지역 경로 추종'], N2, 'rect'),
        ('vg', ['velocity_smoother → cmd_vel'], N2, 'rect'),
    ], color=P) + 24
    y = nodebox('nb_dk', N2X, N2W, y, 'opennav_docking', [
        ('ga', ['action  undock  (S2 기동)'], N2, 'rect'),
        ('gb', ['action  dock  (S10 복귀)'], N2, 'rect'),
    ], color=P) + 24
    d.box('r4note', N2X, y, N2W, [
        'PC-R4 / robot4 는 그림을 생략한다',
        '네임스페이스가 /robot4 인 것 말고는 구성이 완전히 같다.',
        '두 대가 만나는 곳은 zone_semaphore 의 P1 · DOCK 토큰뿐이고,',
        '그 경합이 E-3 교착과 E-7 순차 도킹이다.',
    ], 'note')

    # ─────────── 열 4 · 로봇 하드웨어 ───────────
    nodebox('nb_hw', C4X, C4W, TOP + 38, 'robot3  (TurtleBot4)', [
        ('ha', ['OAK-D Pro', 'rgb · depth · camera_info'], H, 'rect'),
        ('hb', ['RPLIDAR A1', 'scan  10 Hz'], H, 'rect'),
        ('hc', ['Create3', 'odom · tf · battery_state', '범퍼 · 클리프'], H, 'rect'),
        ('hd', ['구동부', 'v ≤ 0.3 m/s'], H, 'rect'),
    ], color=G, style='hwbox')

    # ─────────── PC 컨테이너 ───────────
    def wrap(nid, x, w, title, style, members, pad=16):
        top = min(d.nodes[m]['y'] for m in members) - 34
        bot = max(d.nodes[m]['y'] + d.nodes[m]['h'] for m in members) + pad
        d.band(nid, x, top, w, bot - top, title, style, dash=False)
        return bot

    wrap('pc_w', C1X, C1W, 'PC-W1 / PC-W2 — 웹캠 PC 2대', 'pcbox', ['nb_w1', 'nb_w2', 'c1note'])
    wrap('pc_f', C2X, C2W, 'PC-F — 관제 PC (192.168.108.41 · Discovery Server ID 1)', 'fbox',
         ['nb_ui', 'nb_fm', 'nb_zs', 'nb_kpi'])
    pcr_bot = wrap('pc_r', C3X, C3W, 'PC-R3 — 로봇3 제어 PC  (namespace = /robot3)', 'pcbox',
                   ['nb_det', 'nb_sh', 'nb_fo', 'nb_rd', 'nb_sw', 'nb_nav', 'nb_dk', 'r4note'])
    wrap('pc_h', C4X, C4W, '로봇 온보드', 'hwbox', ['nb_hw'])
    d.order.sort(key=lambda t: (0, 0 if t[1] in ('pc_w', 'pc_f', 'pc_r', 'pc_h') else 1)
                 if t[0] == 'band' else (1, 0))

    # ═══════════ 위쪽 통로 — 관제·웹캠 → 로봇 ═══════════
    def down(src, sf, dst, tf, lane, lines, lx, color):
        sx = d.anchor(src, 'top@%g' % sf)[0]
        tx = d.anchor(dst, 'top@%g' % tf)[0]
        d.edge(src, dst, '', 'top@%g-top@%g' % (sf, tf), color=color, width=1.6,
               via=[(sx, lane), (tx, lane)], label_lines=lines, label_x=lx)

    down('nb_fm', 0.50, 'nb_fo', 0.50, 106,
         ['mission/finish', 'std_msgs/Bool — S9 종료. 즉시 추종 중단 (≤ 1 s)'], 1180, O)
    down('nb_fm', 0.72, 'nb_rd', 0.72, 148,
         ['mission/finish', '같은 신호가 복귀를 시작시킨다'], 1500, O)
    down('nb_fm', 0.28, 'nb_sh', 0.30, 190,
         ['mission/assigned', 'mro_interfaces/Mission — S4 셔틀 트리거'], 1180, O)
    down('nb_w2', 0.50, 'nb_fo', 0.80, 232,
         ['/webcam2/target_hint', 'mro_interfaces/TargetHint — 재탐지 방향. 유효 2 s'], 1050, B)
    down('nb_w1', 0.35, 'nb_det', 0.22, 274,
         ['/webcam1/worker_descriptor', 'mro_interfaces/WorkerDescriptor — 로봇이 대조할 등록 외형'], 800, B)
    down('nb_w1', 0.70, 'nb_fm', 0.14, 316,
         ['/webcam1/worker_descriptor', '관제는 이것을 보고 로봇을 배정한다'], 470, B)

    # ═══════════ 아래쪽 통로 — 로봇 → 관제 · 센서 ═══════════
    def up(src, sf, dst, tf, lane, lines, lx, color, width=1.6):
        sx = d.anchor(src, 'bottom@%g' % sf)[0]
        tx = d.anchor(dst, 'bottom@%g' % tf)[0]
        d.edge(src, dst, '', 'bottom@%g-bottom@%g' % (sf, tf), color=color, width=width,
               via=[(sx, lane), (tx, lane)], label_lines=lines, label_x=lx)

    b0 = pcr_bot + 46
    up('nb_sh', 0.30, 'nb_zs', 0.30, b0,
       ['/fleet/zone_token   (서비스)', 'mro_interfaces/ZoneToken — P1 병목. S4 창고에서 나올 때'], 1000, B)
    up('nb_rd', 0.55, 'nb_zs', 0.55, b0 + 44,
       ['/fleet/zone_token   (서비스)', 'DOCK 구역. 한 번에 한 대 → E-7 순차 도킹'], 1030, B)
    up('nb_sh', 0.80, 'nb_fm', 0.80, b0 + 88,
       ['shuttle/status · approach/status', 'std_msgs/String — 상태 보고. FleetStatus 로 집계된다'], 1130, G)
    up('nb_hw', 0.30, 'nb_det', 0.55, b0 + 132,
       ['oakd/rgb · stereo · camera_info', '압축 전송 · 로봇당 구독자 1개 (NR-03)'], 1700, G)
    up('nb_hw', 0.55, 'nb_sw', 0.80, b0 + 176,
       ['scan   sensor_msgs/LaserScan', '안전 감시가 직접 구독한다 — 인지도 웹캠도 관제도 끼지 않는다'], 1660, R)
    up('nb_sw', 0.30, 'nb_hw', 0.80, b0 + 220,
       ['cmd_vel = 0  @ 20 Hz', 'estop 동안 Nav2 출력을 덮어쓴다 — 이쪽이 이긴다'], 1700, R, 2.4)

    # ═══════════ 열 안쪽 — 우리 노드 ↔ Nav2 ↔ 하드웨어 ═══════════
    gi = (N1X + N1W + N2X) / 2
    for src, dst, f in [('nb_sh', 'nb_nav', 0.30), ('nb_fo', 'nb_nav', 0.55),
                        ('nb_rd', 'nb_nav', 0.75), ('nb_rd', 'nb_dk', 0.90)]:
        d.edge(src, dst, '', 'right@%g-left@%g' % (f, f), color=P, width=1.6, mid=gi)
    d.box('navnote', N1X + N1W + 6, TOP + 8, N2X - N1X - N1W - 12, [
        'Nav2 액션', 'navigate_to_pose', 'dock · undock',
    ], 'n2step', align='center')
    d.edge('nb_nav', 'nb_hw', '', 'right@0.9-left@0.35', color=P, width=1.8,
           mid=(N2X + N2W + C4X) / 2,
           label_lines=['cmd_vel', 'geometry_msgs/TwistStamped'],
           label_x=(N2X + N2W + C4X) / 2)
    d.edge('nb_det', 'nb_fo', '', 'left@0.5-left@0.35', color=B, width=1.6,
           via=[(N1X - 16, d.nodes['nb_det']['y'] + d.nodes['nb_det']['h'] * 0.5),
                (N1X - 16, d.nodes['nb_fo']['y'] + d.nodes['nb_fo']['h'] * 0.35)],
           label_lines=['target/paired · map_point'], label_x=N1X + 90)
    d.edge('nb_sh', 'nb_fo', '', 'left@0.8-left@0.65', color=B, width=1.6,
           via=[(N1X - 34, d.nodes['nb_sh']['y'] + d.nodes['nb_sh']['h'] * 0.8),
                (N1X - 34, d.nodes['nb_fo']['y'] + d.nodes['nb_fo']['h'] * 0.65)],
           label_lines=['shuttle/arrived'], label_x=N1X + 70)

    # ═══════════ 범례 ═══════════
    lg_y = b0 + 260
    d.box('lg2', 30, lg_y, 2280, [
        '읽는 법',
        '· 화살표 위 상자의 첫 줄이 토픽·서비스 이름, 둘째 줄이 메시지 타입과 그 선이 하는 일이다. PC 를 넘는 것은 토픽과 서비스뿐이며 직접 호출도 소켓 통신도 없다 (NR-06).',
        '· 색 — 주황: 관제가 내리는 명령 · 파랑: 인지·웹캠 데이터 · 보라: Nav2 액션 · 빨강: 안전 경로 · 회색: 상태 보고와 센서',
        '· 위쪽 통로는 관제·웹캠에서 로봇으로 내려가는 흐름, 아래쪽 통로는 로봇에서 올라오는 상태와 센서다. 선이 겹치지 않도록 각 신호에 통로를 하나씩 줬다.',
        '',
        '이 그림이 말하는 것 — 빨간 경로(scan → safety_watchdog → cmd_vel)만 로봇 온보드에서 닫힌 고리를 이룬다. 파란 웹캠 경로도 주황 관제 경로도 그 고리 밖에 있다.',
        '                     그래서 Wi-Fi 가 끊기거나 웹캠 PC 가 죽어도 로봇은 여전히 멈춘다. 이것이 무사고 0 건(NF-06)을 지탱하는 구조다.',
    ], 'note')
    d.text(30, lg_y + d.nodes['lg2']['h'] + 20,
           '생성: tools/build_diagrams.py   ·   근거: docs/architecture.md, docs/interfaces.md 4절, SDD 2.4 FR 표',
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
