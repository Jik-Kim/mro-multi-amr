# -*- coding: utf-8 -*-
"""마무리 — 다 그린 뒤 글자끼리 겹치면 세로로 밀어낸다. 선은 건드리지 않는다."""
import re, html

def anchor(box, st, kind, other):
    x, y, w, h = box
    m = re.search(rf'{kind}X=([\d.]+);{kind}Y=([\d.]+)', st or '')
    if m: return (x + w * float(m.group(1)), y + h * float(m.group(2)))
    ox, oy, ow, oh = other
    cx, cy = x + w / 2, y + h / 2
    dx, dy = ox + ow / 2 - cx, oy + oh / 2 - cy
    if abs(dx) * h > abs(dy) * w: return (x + (w if dx > 0 else 0), cy)
    return (cx, y + (h if dy > 0 else 0))

def run(d, BOX, gapy=54, gapx=420, rounds=90):
    items = []
    for idx, c in enumerate(d.cells):
        if c[0] != 'e': continue
        _, i, val, st, a, b, pts, lx, loff = c
        if not val or a not in BOX or b not in BOX or lx is None: continue
        full = [anchor(BOX[a], st, 'exit', BOX[b])] + [tuple(p) for p in pts] + \
               [anchor(BOX[b], st, 'entry', BOX[a])]
        segs = [(full[k], full[k + 1]) for k in range(len(full) - 1)]
        tot = sum(abs(q[0] - p[0]) + abs(q[1] - p[1]) for p, q in segs) or 1
        want = tot * ((lx + 1) / 2); acc = 0; pos = full[-1]
        for p, q in segs:
            L = abs(q[0] - p[0]) + abs(q[1] - p[1])
            if acc + L >= want:
                f = (want - acc) / L if L else 0
                pos = (p[0] + (q[0] - p[0]) * f, p[1] + (q[1] - p[1]) * f); break
            acc += L
        ox, oy = loff if loff else (0, 0)
        lines = len(re.sub(r'<[^>]+>', '', html.unescape(val)).split('\n'))
        items.append([idx, pos[0] + ox, pos[1] + oy, max(1, lines), ox, oy])
    moved = 0
    for _ in range(rounds):
        items.sort(key=lambda t: (round(t[1] / gapx), t[2]))
        hit = False
        for i in range(len(items)):
            for j in range(i + 1, len(items)):
                A, B = items[i], items[j]
                need = gapy + (A[3] + B[3] - 2) * 9
                if abs(A[1] - B[1]) < gapx and abs(A[2] - B[2]) < need:
                    push = (need - abs(A[2] - B[2])) / 2 + 2
                    if A[2] <= B[2]: A[2] -= push; A[5] -= push; B[2] += push; B[5] += push
                    else:            A[2] += push; A[5] += push; B[2] -= push; B[5] -= push
                    hit = True; moved += 1
        if not hit: break
    for idx, _x, _y, _n, ox, oy in items:
        c = list(d.cells[idx]); c[8] = (ox, oy); d.cells[idx] = tuple(c)
    # 남은 겹침 보고
    left = sum(1 for i in range(len(items)) for j in range(i + 1, len(items))
               if abs(items[i][1] - items[j][1]) < gapx and abs(items[i][2] - items[j][2]) < gapy)
    print(f'글자 {len(items)}개 · 밀어낸 횟수 {moved} · 남은 겹침 {left}건')
