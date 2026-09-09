# -*- coding: utf-8 -*-
"""내가 만든 drawio 를 그대로 SVG 로 그려 눈으로 확인한다 (양식이 내 것뿐이라 가능)."""
import xml.etree.ElementTree as ET, re, html, sys, os
from engine import wrap, tw as wrapw, FS, LH, PADX, PADY

HERE = os.path.dirname(os.path.abspath(__file__))
DEF = os.path.normpath(os.path.join(HERE, '..', '..', 'docs', 'diagrams', 'system_flowchart.drawio'))
src = sys.argv[1] if len(sys.argv) > 1 else DEF
out = sys.argv[2] if len(sys.argv) > 2 else DEF.replace('.drawio', '_preview.svg')
r = ET.parse(src).getroot()[0].find('.//root')
C = {c.get('id'): c for c in r}
def g(i, k):
    m = re.findall(rf'{k}=([^;]+)', C[i].get('style') or ''); return m[-1] if m else None
def bx(i):
    gg = C[i].find('mxGeometry')
    return [float(gg.get(k) or 0) for k in ('x', 'y', 'width', 'height')]
V = {i: bx(i) for i, c in C.items() if c.get('vertex') == '1' and c.find('mxGeometry') is not None}
X1 = max(b[0] + b[2] for b in V.values()) + 40
Y1 = max(b[1] + b[3] for b in V.values()) + 40
o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{X1:.0f}" height="{Y1:.0f}" viewBox="0 0 {X1:.0f} {Y1:.0f}">',
     f'<rect width="{X1:.0f}" height="{Y1:.0f}" fill="#ffffff"/>',
     '<style>text{font-family:"Malgun Gothic","Noto Sans CJK KR",sans-serif}</style>',
     '<defs>' + ''.join(
        f'<marker id="m{k}" markerWidth="9" markerHeight="9" refX="8" refY="3" orient="auto">'
        f'<path d="M0,0 L9,3 L0,6 z" fill="{c}"/></marker>'
        for k, c in (('0', '#4A4A4A'), ('1', '#1F5FBF'), ('2', '#C0272D'), ('3', '#D97706'))) + '</defs>']
MK = {'#4A4A4A': 'm0', '#1F5FBF': 'm1', '#C0272D': 'm2', '#D97706': 'm3'}
def esc(s): return html.escape(s)
def txt_of(i):
    v = html.unescape(C[i].get('value') or '')
    v = re.sub(r'<br\s*/?>', '\n', v, flags=re.I); v = re.sub(r'<[^>]+>', '', v)
    return html.unescape(v)
for i in sorted(V, key=lambda i: -V[i][2] * V[i][3]):
    x, y, w, h = V[i]; st = C[i].get('style') or ''
    f = g(i, 'fillColor') or 'none'; s = g(i, 'strokeColor') or 'none'
    sw = g(i, 'strokeWidth') or '1'
    if 'text;' in st: f, s = 'none', 'none'
    if 'ellipse' in st:
        o.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{h/2}" fill="{f}" stroke="{s}" stroke-width="{sw}"/>')
    elif 'rhombus' in st:
        o.append(f'<polygon points="{x+w/2},{y} {x+w},{y+h/2} {x+w/2},{y+h} {x},{y+h/2}" fill="{f}" stroke="{s}" stroke-width="{sw}"/>')
    elif f != 'none' or s != 'none':
        o.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{f}" stroke="{s}" stroke-width="{sw}"/>')
    t = txt_of(i)
    if not t: continue
    fs = float(g(i, 'fontSize') or FS)
    fc = g(i, 'fontColor') or ('#FFFFFF' if 'fillColor=#6F7B8A' in st or (f and f in ('#6E5AA0','#B03A2E','#2C6E9B','#7A5C1E','#6F7B8A')) else '#111111')
    bold = 'fontStyle=1' in st or '<b' in (C[i].get('value') or '')
    align = 'start' if ('align=left' in st or 'text;' in st) else 'middle'
    lines = wrap(t, w if align == 'middle' else w + 999)
    lines = [l for l in lines]
    lh = fs * 1.33
    if align == 'middle':
        ty = y + h / 2 - (len(lines) - 1) * lh / 2 + fs * 0.36
        tx = x + w / 2
    else:
        ty = y + (h / 2 - (len(lines) - 1) * lh / 2 + fs * 0.36 if 'verticalAlign=middle' in st else fs * 1.1)
        tx = x + (8 if 'text;' not in st else 0)
    for k, ln in enumerate(lines):
        if not ln.strip(): continue
        o.append(f'<text x="{tx:.1f}" y="{ty + k*lh:.1f}" font-size="{fs}" fill="{fc}" '
                 f'text-anchor="{align}"{" font-weight=\'700\'" if bold else ""}>{esc(ln)}</text>')
def anc(i, st, kind, other):
    x, y, w, h = V[i]
    m = re.search(rf'{kind}X=([\d.]+);{kind}Y=([\d.]+)', st)
    if m: return (x + w * float(m.group(1)), y + h * float(m.group(2)))
    ox, oy, ow, oh = V[other]
    cx, cy = x + w / 2, y + h / 2; dx, dy = ox + ow / 2 - cx, oy + oh / 2 - cy
    if abs(dx) * h > abs(dy) * w: return (x + (w if dx > 0 else 0), cy)
    return (cx, y + (h if dy > 0 else 0))
for i, c in C.items():
    if c.get('edge') != '1': continue
    a, b = c.get('source'), c.get('target')
    if a not in V or b not in V: continue
    st = c.get('style') or ''; col = g(i, 'strokeColor') or '#4A4A4A'
    pts = [anc(a, st, 'exit', b)]
    arr = c.find('mxGeometry').find('Array')
    if arr is not None: pts += [(float(p.get('x')), float(p.get('y'))) for p in arr]
    pts.append(anc(b, st, 'entry', a))
    path = [pts[0]]
    for p in pts[1:]:
        q = path[-1]
        if abs(p[0]-q[0]) > 1 and abs(p[1]-q[1]) > 1: path.append((p[0], q[1]) if abs(p[0]-q[0]) < abs(p[1]-q[1]) else (q[0], p[1]))
        path.append(p)
    dd = ' '.join(('M' if k == 0 else 'L') + f'{p[0]:.1f},{p[1]:.1f}' for k, p in enumerate(path))
    dash = ' stroke-dasharray="6 4"' if 'dashed=1' in st else ''
    o.append(f'<path d="{dd}" fill="none" stroke="{col}" stroke-width="{g(i,"strokeWidth") or 1.4}"{dash} marker-end="url(#{MK.get(col,"m0")})"/>')
    lab = txt_of(i)
    if lab:
        gx = float(c.find('mxGeometry').get('x') or 0)
        segs = [(path[i], path[i+1]) for i in range(len(path)-1)]
        tot = sum(abs(b_[0]-a_[0]) + abs(b_[1]-a_[1]) for a_, b_ in segs) or 1
        want = tot * ((gx + 1) / 2)
        acc = 0; mid = path[-1]
        for a_, b_ in segs:
            L = abs(b_[0]-a_[0]) + abs(b_[1]-a_[1])
            if acc + L >= want:
                f = (want - acc) / L if L else 0
                mid = (a_[0] + (b_[0]-a_[0])*f, a_[1] + (b_[1]-a_[1])*f); break
            acc += L
        off = c.find('mxGeometry').find("mxPoint[@as='offset']")
        if off is not None: mid = (mid[0] + float(off.get('x') or 0), mid[1] + float(off.get('y') or 0))
        if col == '#4A4A4A':                      # 노드 안 라벨은 테두리 없이
            o.append(f'<text x="{mid[0]:.1f}" y="{mid[1]-4:.1f}" font-size="10" fill="{col}" '
                     f'text-anchor="middle" paint-order="stroke" stroke="#fff" stroke-width="3">'
                     f'{esc(lab.splitlines()[0])}</text>')
            continue
        ls = [l for l in lab.split('\n') if l.strip()]
        lw = max(wrapw(l) for l in ls) + 14; lh = len(ls) * 13 + 8
        o.append(f'<rect x="{mid[0]-lw/2:.1f}" y="{mid[1]-lh/2:.1f}" width="{lw:.1f}" height="{lh:.1f}" '
                 f'fill="#ffffff" stroke="{col}" stroke-width="1" rx="2"/>')
        for kk, ln in enumerate(ls):
            o.append(f'<text x="{mid[0]:.1f}" y="{mid[1]-lh/2+13+kk*13:.1f}" font-size="10" fill="{col}" '
                     f'text-anchor="middle">{esc(ln)}</text>')
o.append('</svg>')
open(out, 'w', encoding='utf-8').write('\n'.join(o))
print(out, f'{X1:.0f} × {Y1:.0f}')
