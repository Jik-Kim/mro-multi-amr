# -*- coding: utf-8 -*-
"""선이 남의 상자를 뚫고 지나가는지 검사한다. 하나라도 남으면 내보내지 않는다."""
import xml.etree.ElementTree as ET, re, html, sys, os
HERE = os.path.dirname(os.path.abspath(__file__))
src = sys.argv[1] if len(sys.argv) > 1 else os.path.normpath(
    os.path.join(HERE, '..', '..', 'docs', 'diagrams', 'system_flowchart.drawio'))
r = ET.parse(src).getroot()[0].find('.//root')
C = {c.get('id'): c for c in r}
def g(i, k):
    m = re.findall(rf'{k}=([^;]+)', C[i].get('style') or ''); return m[-1] if m else None
def bx(i):
    gg = C[i].find('mxGeometry'); return [float(gg.get(k) or 0) for k in ('x', 'y', 'width', 'height')]
V = {i: bx(i) for i, c in C.items() if c.get('vertex') == '1' and c.find('mxGeometry') is not None}
def txt(i):
    v = html.unescape(C[i].get('value') or ''); v = re.sub(r'<[^>]+>', ' ', v)
    return re.sub(r'\s+', ' ', v).strip()
BIG = {i for i, (x, y, w, h) in V.items() if w * h > 260000}    # PC·노드 테두리는 지나가도 된다
BAR = {i for i in V if 'fillColor=#6F7B8A' in (C[i].get('style') or '') or
       any(c in (C[i].get('style') or '') for c in ('#6E5AA0', '#B03A2E', '#2C6E9B', '#7A5C1E'))}
SKIP = (BIG | BAR | {i for i in V if 'text;' in (C[i].get('style') or '')}
        | {i for i in V if (g(i, 'fillColor') or 'none') == 'none'})   # 테두리는 통과해도 된다
def anc(i, st, kind, other):
    x, y, w, h = V[i]
    m = re.search(rf'{kind}X=([\d.]+);{kind}Y=([\d.]+)', st)
    if m: return (x + w * float(m.group(1)), y + h * float(m.group(2)))
    ox, oy, ow, oh = V[other]
    cx, cy = x + w / 2, y + h / 2; dx, dy = ox + ow / 2 - cx, oy + oh / 2 - cy
    if abs(dx) * h > abs(dy) * w: return (x + (w if dx > 0 else 0), cy)
    return (cx, y + (h if dy > 0 else 0))
def hit(p, q, R):
    x, y, w, h = R; x2, y2 = x + w, y + h
    if abs(p[1] - q[1]) < 1:
        return (y + 4 < p[1] < y2 - 4) and min(p[0], q[0]) + 4 < x2 and max(p[0], q[0]) - 4 > x
    if abs(p[0] - q[0]) < 1:
        return (x + 4 < p[0] < x2 - 4) and min(p[1], q[1]) + 4 < y2 and max(p[1], q[1]) - 4 > y
    return False
bad = 0
for i, c in C.items():
    if c.get('edge') != '1': continue
    a, b = c.get('source'), c.get('target')
    if a not in V or b not in V: continue
    st = c.get('style') or ''
    pts = [anc(a, st, 'exit', b)]
    arr = c.find('mxGeometry').find('Array')
    if arr is not None: pts += [(float(p.get('x')), float(p.get('y'))) for p in arr]
    pts.append(anc(b, st, 'entry', a))
    path = [pts[0]]
    for p in pts[1:]:
        q = path[-1]
        if abs(p[0] - q[0]) > 1 and abs(p[1] - q[1]) > 1: path.append((p[0], q[1]))
        path.append(p)
    hits = []
    for k in range(len(path) - 1):
        for vi, R in V.items():
            if vi in (a, b) or vi in SKIP: continue
            if hit(path[k], path[k + 1], R): hits.append(vi)
    if hits:
        bad += 1
        print(f"  {i:>6} [{txt(i)[:40]:<40}] → {len(set(hits))}개: "
              + ', '.join((txt(h)[:22] or h) for h in list(dict.fromkeys(hits))[:3]))
print(f"\n상자를 가로지르는 선: {bad}건 / 전체 {sum(1 for c in C.values() if c.get('edge')=='1')}")
