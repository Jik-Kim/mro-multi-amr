# -*- coding: utf-8 -*-
"""시스템 플로우차트 생성기 — SDD 0차 5쪽 양식.
   PC 테두리(제목 띠) → 노드 테두리(제목 띠) → 세로 한 줄 순서도.
   모든 상자·색·글꼴은 여기 한 곳에서만 정한다. 담당자별로 달라질 수 없다."""
import re, html, xml.etree.ElementTree as ET

FONT, FS, LH = 'Malgun Gothic', 11, 14.6
PADX, PADY = 9, 8
W_MAIN, W_SIDE, W_IO = 400, 250, 250
GAP_X, GAP_Y = 30, 24
BAR = 24                      # 제목 띠 높이
PAD_NODE, PAD_PC = 14, 64   # PC 안쪽 여백 = 노드 옆을 지나는 세로 통로

C = {
 'do':    ('#EEF4FD', '#5B87C4'),
 'ask':   ('#FFF6E0', '#C88A00'),
 'else':  ('#FDECEC', '#C0272D'),
 'start': ('#E4ECF7', '#3B6CA8'),
 'io':    ('#E8F0FE', '#1F5FBF'),
 'dev':   ('#EFEFEF', '#777777'),
 'note':  ('#FAFAFA', '#CCCCCC'),
}
PC_BAR = {'W': '#6E5AA0', 'F': '#B03A2E', 'R': '#2C6E9B', 'T': '#7A5C1E'}
NODE_BAR = '#6F7B8A'
LINE = '#4A4A4A'
# 토픽 갈래마다 색을 나눈다 — 어느 선이 무슨 종류인지 색만 보고 갈리게
TOPIC = '#1F5FBF'    # 식별·대상 (웹캠 등록 · target/*)
CMD   = '#7A3E9D'    # 미션 명령 (mission/assigned · mission/finish)
STAT  = '#0E7C7B'    # 상태 보고 (approach/status · shuttle/status · fleet/status · zone_event)
SENS  = '#55606B'    # 장비 원본 (oakd · scan · battery · amcl)
SVC   = '#D97706'    # 서비스 (zone_token · 관제 서비스)
SAFE  = '#C0272D'    # 안전 (estop · cmd_vel)

def tw(s):
    w = 0.0
    for ch in s:
        w += 11.3 if ord(ch) > 0x2000 else (3.3 if ch == ' ' else 6.1)
    return w
def wrap(text, width):
    inner = width - PADX * 2
    out = []
    for para in text.split('\n'):
        if not para:
            out.append(''); continue
        cur = ''
        for tok in re.findall(r'\S+\s*', para):
            if not cur or tw(cur + tok) <= inner: cur += tok
            else: out.append(cur.rstrip()); cur = tok
        out.append(cur.rstrip())
    return out
def box_h(text, width, kind):
    n = max(1, len(wrap(text, width)))
    h = n * LH + PADY * 2
    if kind == 'ask': h = max(h + n * 6, 50)
    if kind == 'start': h = max(h, 34)
    return int(round(h))

class Doc:
    def __init__(self):
        self.cells = []; self.n = 0; self.by = {}
    def _id(self, p='c'):
        self.n += 1; return f'{p}{self.n}'
    def rect(self, x, y, w, h, text, kind='do', extra='', key=None, front=False):
        f, s = C.get(kind, C['do'])
        st = (('ellipse;' if kind == 'start' else '') + ('rhombus;' if kind == 'ask' else '') +
              f'rounded=0;whiteSpace=wrap;html=1;fontFamily={FONT};fontSize={FS};'
              f'fillColor={f};strokeColor={s};verticalAlign=middle;spacing=4;' + extra)
        i = self._id()
        self.cells.append(('v', i, text, st, x, y, w, h, front))
        if key: self.by[key] = (i, x, y, w, h)
        return i
    def frame(self, x, y, w, h, title, bar, key=None, sub=''):
        i = self._id('f')
        self.cells.append(('v', i, '', f'rounded=0;html=1;fillColor=none;strokeColor={bar};'
                                      f'strokeWidth=2;', x, y, w, h, True))
        t = self._id('b')
        lab = (f'<b style="color:#FFFFFF;font-size:12px">{title}</b>' +
               (f'<span style="color:#E8E8E8;font-size:10px">　{sub}</span>' if sub else ''))
        self.cells.append(('v', t, lab, f'rounded=0;html=1;fillColor={bar};strokeColor={bar};'
                                        f'fontFamily={FONT};fontSize=12;align=left;spacingLeft=8;'
                                        f'verticalAlign=middle;', x, y, w, BAR, True))
        if key: self.by[key] = (i, x, y, w, h)
        return i
    def edge(self, a, b, label='', color=LINE, dashed=False, pts=(), exit=None, entry=None, w=1.4, lx=None, loff=None):
        # 글자는 제 선 위에 흰 바탕 + 같은 색 테두리로 얹는다 — 어느 선의 글자인지 바로 보이게
        st = (f'edgeStyle=orthogonalEdgeStyle;rounded=0;html=1;strokeColor={color};strokeWidth={w};'
              f'fontFamily={FONT};fontSize=10;fontColor={color};labelBackgroundColor=#FFFFFF;'
              + (f'labelBorderColor={color};' if color != LINE else '')
              + f'verticalAlign=middle;align=center;'
              f'endArrow=block;endFill=1;endSize=6;jettySize=auto;'
              + ('dashed=1;' if dashed else ''))
        if exit: st += f'exitX={exit[0]};exitY={exit[1]};exitDx=0;exitDy=0;'
        if entry: st += f'entryX={entry[0]};entryY={entry[1]};entryDx=0;entryDy=0;'
        i = self._id('e')
        self.cells.append(('e', i, label, st, a, b, list(pts), lx, loff))
        return i
    def text(self, x, y, w, h, s, size=10, color='#444444', bold=False):
        i = self._id('t')
        st = (f'text;html=1;strokeColor=none;fillColor=none;align=left;verticalAlign=top;'
              f'whiteSpace=wrap;fontFamily={FONT};fontSize={size};fontColor={color};'
              + ('fontStyle=1;' if bold else ''))
        self.cells.append(('v', i, s, st, x, y, w, h, False))
        return i
    def write(self, path, pages):
        mx = ET.Element('mxfile', {'host': 'app.diagrams.net', 'type': 'device'})
        for name, cells in pages:
            d = ET.SubElement(mx, 'diagram', {'name': name, 'id': name})
            m = ET.SubElement(d, 'mxGraphModel', {'dx': '2000', 'dy': '1200', 'grid': '0',
                'gridSize': '10', 'guides': '1', 'tooltips': '1', 'connect': '1', 'arrows': '1',
                'fold': '1', 'page': '0', 'pageScale': '1', 'pageWidth': '1169',
                'pageHeight': '827', 'math': '0', 'shadow': '0'})
            r = ET.SubElement(m, 'root')
            ET.SubElement(r, 'mxCell', {'id': '0'}); ET.SubElement(r, 'mxCell', {'id': '1', 'parent': '0'})
            for c in sorted(cells, key=lambda c: 0 if (c[0] == 'v' and c[8]) else 1):
                if c[0] == 'v':
                    _, i, val, st, x, y, w, h, _f = c
                    e = ET.SubElement(r, 'mxCell', {'id': i, 'value': val, 'style': st,
                                                    'vertex': '1', 'parent': '1'})
                    ET.SubElement(e, 'mxGeometry', {'x': str(round(x)), 'y': str(round(y)),
                                                    'width': str(round(w)), 'height': str(round(h)),
                                                    'as': 'geometry'})
                else:
                    _, i, val, st, a, b, pts, lx, loff = c
                    e = ET.SubElement(r, 'mxCell', {'id': i, 'value': val, 'style': st,
                                                    'edge': '1', 'parent': '1', 'source': a, 'target': b})
                    ga = {'relative': '1', 'as': 'geometry'}
                    if lx is not None: ga['x'] = str(lx)
                    g = ET.SubElement(e, 'mxGeometry', ga)
                    if loff: ET.SubElement(g, 'mxPoint', {'as': 'offset',
                                                          'x': str(round(loff[0])), 'y': str(round(loff[1]))})
                    if pts:
                        arr = ET.SubElement(g, 'Array', {'as': 'points'})
                        for px, py in pts:
                            ET.SubElement(arr, 'mxPoint', {'x': str(round(px)), 'y': str(round(py))})
        ET.ElementTree(mx).write(path, encoding='utf-8', xml_declaration=True)
