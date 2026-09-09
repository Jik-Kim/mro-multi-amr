# -*- coding: utf-8 -*-
"""노드 순서도를 세로 한 줄로 세우고, 노드를 PC 테두리에, PC 를 캔버스에 놓는다."""
from engine import (Doc, C, PC_BAR, NODE_BAR, LINE, TOPIC, SVC, SAFE,
                    W_MAIN, W_SIDE, GAP_X, GAP_Y, BAR, PAD_NODE, PAD_PC, box_h)

def chain_size(steps):
    """한 갈래의 폭·높이와 각 칸 위치를 미리 잰다."""
    has_side = any(s['k'] == 'ask' and s.get('no') for s in steps)
    w = W_MAIN + (GAP_X + W_SIDE if has_side else 0)
    y = 0.0; place = []
    for s in steps:
        h = box_h(s['t'], W_MAIN, s['k'])
        sh = box_h(s['no'], W_SIDE, 'else') if (s['k'] == 'ask' and s.get('no')) else 0
        place.append((y, h, sh))
        y += max(h, sh) + GAP_Y
    return w, max(0.0, y - GAP_Y), place

def draw_chain(d, steps, x, y):
    w, h, place = chain_size(steps)
    ids = []
    for i, (s, (dy, bh, sh)) in enumerate(zip(steps, place)):
        bid = d.rect(x, y + dy, W_MAIN, bh, s['t'], s['k'])
        sid = None
        if s['k'] == 'ask' and s.get('no'):
            sid = d.rect(x + W_MAIN + GAP_X, y + dy + (bh - sh) / 2, W_SIDE, sh, s['no'], 'else')
        ids.append((bid, sid))
    for i in range(len(steps) - 1):
        lab = steps[i].get('yes', '') if steps[i]['k'] == 'ask' else ''
        d.edge(ids[i][0], ids[i + 1][0], lab, exit=(.5, 1), entry=(.5, 0))
    for i, s in enumerate(steps):
        if ids[i][1] is None: continue
        d.edge(ids[i][0], ids[i][1], s.get('nol', '아니오'), exit=(1, .5), entry=(0, .5))
        b = s.get('back')
        if b is not None and b < len(steps):
            # 갈래에서 되돌아가는 선 — 오른쪽으로 빠져 칸 사이 빈 줄을 타고 왼쪽으로 돌아 들어간다
            xr = x + W_MAIN + GAP_X + W_SIDE + 14 + (i % 3) * 7
            xl = x - 12 - (i % 3) * 7
            yb = place[i][0] + (place[i][1] - place[i][2]) / 2 + place[i][2] / 2
            ymid = place[b][0] - GAP_Y / 2
            yt = place[b][0] + place[b][1] / 2
            d.edge(ids[i][1], ids[b][0], '', dashed=True,
                   pts=[(xr, y + yb), (xr, y + ymid), (xl, y + ymid), (xl, y + yt)],
                   exit=(1, .5), entry=(0, .5))
    return w, h, ids

def chain_rows(nd):
    """갈래가 많으면 옆으로만 늘리지 않고 줄을 나눠 담는다 — 노드가 지나치게 넓어지지 않게."""
    sizes = [chain_size(c) for c in nd['chains']]
    ncol = nd.get('chaincols', 2)
    rows, cur = [], []
    for i, s in enumerate(sizes):
        cur.append(i)
        if len(cur) == ncol: rows.append(cur); cur = []
    if cur: rows.append(cur)
    return rows, sizes

def node_size(nd):
    rows, sizes = chain_rows(nd)
    w = max(sum(sizes[i][0] for i in r) + GAP_X * 2 * (len(r) - 1) for r in rows)
    h = sum(max(sizes[i][1] for i in r) for r in rows) + GAP_Y * 2.4 * (len(rows) - 1)
    return w + PAD_NODE * 2, h + BAR + PAD_NODE * 2

def draw_node(d, nd, x, y, width=None):
    w, h = node_size(nd)
    if width: w = width          # PC 안에서 노드 폭을 하나로 맞춘다
    nd['_frame'] = d.frame(x, y, w, h, nd['name'], NODE_BAR, sub=nd.get('sub', ''))
    nd['_box'] = (x, y, w, h)
    rows, sizes = chain_rows(nd)
    first = {}; anch = {}; yy = y + BAR + PAD_NODE
    for r in rows:
        cx = x + PAD_NODE
        for ci in r:
            cw, chh, ids = draw_chain(d, nd['chains'][ci], cx, yy)
            first[ci] = ids
            for si, st in enumerate(nd['chains'][ci]):
                if st.get('a'): anch[st['a']] = ids[si][0]
            cx += cw + GAP_X * 2
        yy += max(sizes[i][1] for i in r) + GAP_Y * 2.4
    nd['_anch'] = anch
    return w, h, first

def draw_pc(d, key, title, sub, nodes, x, y, ncol, bar, cols=None, minw=None):
    """노드를 열에 나눠 담는다. cols 를 주면 그 순서대로 — 화살표가 오가는 노드끼리 붙여 놓는다."""
    if cols is not None:
        cols = [[(n, *node_size(n)) for n in col] for col in cols]
    else:
        items = [(n, *node_size(n)) for n in nodes]
        items.sort(key=lambda t: -t[2])
        cols = [[] for _ in range(max(1, min(ncol, len(items))))]
        ch = [0.0] * len(cols)
        for n, w, h in items:
            k = min(range(len(cols)), key=lambda i: ch[i])
            cols[k].append((n, w, h)); ch[k] += h + GAP_Y * 2
    xx = x + PAD_PC; W = 0.0; H = 0.0
    for col in cols:
        cw = max(w for _, w, _ in col); yy = y + BAR + PAD_PC
        for n, w, h in col:
            n['_col'] = (xx, cw)          # 열의 좌우 끝 — 화살표가 열 밖으로 빠질 때 쓴다
            draw_node(d, n, xx + (cw - w) / 2, yy); yy += h + GAP_Y * 2
        H = max(H, yy - GAP_Y * 2 + PAD_PC - y)
        xx += cw + GAP_X * 2
    W = xx - GAP_X * 2 + PAD_PC - x
    d.frame(x, y, W, H, title, bar, key=key, sub=sub)
    return W, H
