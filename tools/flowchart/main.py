# -*- coding: utf-8 -*-
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from engine import Doc, PC_BAR, LINE, TOPIC, SVC, SAFE, BAR, PAD_PC, GAP_X, GAP_Y
import render, nodes as N

d = Doc()
TOP, LEFT, GAPPC = 330, 40, 150
REG = {}
def pc(key, title, sub, ns, x, y, maxw, bar):
    """PC 한 대를 그리고 안의 상자 id 를 기억해 둔다."""
    saved = len(d.cells)
    w, h = render.draw_pc(d, key, title, sub, ns, x, y, maxw, bar)
    return w, h

# 상자 id 를 되찾기 위해 draw_node 를 감싼다
ANCH = {}
_orig = render.draw_node
def draw_node(dd, nd, x, y, width=None):
    w, h, first = _orig(dd, nd, x, y, width)
    ANCH[nd['_key']] = dict(frame=nd['_frame'], box=nd['_box'], chains=first,
                            col=nd.get('_col'), anch=nd.get('_anch', {}))
    return w, h, first
render.draw_node = draw_node
def key(nd, k):
    nd = dict(nd); nd['_key'] = k; return nd

# ── 배치 : 가운데 한 줄에 관제·웹캠·터틀봇, 좌우에 로봇 제어 PC ─────────────
#    PC 마다 노드를 한 줄로 세우면 모든 노드가 PC 양쪽 테두리에 닿는다 →
#    PC 를 건너는 화살표가 옆으로 곧장 이어진다. 가운데 것들은 두 로봇 PC 와 대칭이다.
RN = [N.AMRDET, N.ARUCO, N.FOLLOW, N.SHUTTLE, N.WATCHDOG, N.RETURN, N.NAV2]
def rpc(rp, x, y0, title, sub, barc):
    K = {n['name']: key(n, f'{rp}_{n["name"]}') for n in RN}
    # Nav2 는 우리가 만든 노드가 아니지만 이 PC 에서 같이 도니 같은 테두리 안에 둔다.
    # 주행 세 노드가 목표를 주고 안전 감시가 회전분을 빌려 쓰므로 그 사이에 놓는다.
    order = [K['amr_detection'], K['follow_target'], K['shuttle_mission'],
             K['aruco_detector'], K['return_dock'], K['Nav2  (길찾기 · 도킹)'],
             K['safety_watchdog']]
    return render.draw_pc(d, rp, title, sub, order, x, y0, 1, barc, cols=[order])

y = TOP
# 가운데 줄 — 로봇 PC 의 노드 차례(인지 → 추종 → 셔틀 → 아르코 → 복귀 → 안전)와
# 높이를 맞춰 두면 오가는 선이 짧아진다
_fm, _cui, _zone, _kpi = key(N.FLEET, 'fm'), key(N.CONTROL, 'cui'), key(N.ZONE, 'zone'), key(N.KPI, 'kpi')
for _n in (_fm, _zone): _n['chaincols'] = 2
MID = [('PC1', 'PC1 · 웹캠1 PC', 'PC-W1', [key(N.WEBCAM1, 'wreg')], PC_BAR['W'], 1),
       ('PC2', 'PC2 · 웹캠2 PC', 'PC-W2', [key(N.WEBCAM2, 'whint')], PC_BAR['W'], 1),
       ('PC3', 'PC3 · 관제 PC', 'PC-F', [_fm, _cui, _zone, _kpi], PC_BAR['F'], 1),
       ('T3', '터틀봇 · robot3', '실물 장비 — 왼쪽 PC4 가 이 로봇을 몬다',
        [key(N.robot_dev('/robot3'), 't3')], PC_BAR['T'], 1),
       ('T8', '터틀봇 · robot8', '실물 장비 — 오른쪽 PC5 가 이 로봇을 몬다',
        [key(N.robot_dev('/robot8'), 't8')], PC_BAR['T'], 1)]
MIDW = max(max(render.node_size(n)[0] for n in ns) + 128 for _k, _t, _s, ns, _b, _c in MID)

x4 = LEFT
w4, h4 = rpc('r3', x4, y, 'PC4 · AMR 제어 PC  (/robot3)', 'PC-R3', PC_BAR['R'])
RX = x4 + w4 + 680
# 가운데 열 — PC 마다 제 폭을 쓰되 열 가운데로 맞춘다 (억지로 늘리지 않는다)
yy = y
CENTER = []
for _k, _t, _s, _ns, _b, _c in MID:
    _w0 = max(render.node_size(n)[0] for n in _ns) + 128
    _x0 = RX + (MIDW - _w0) / 2
    ww, hh = render.draw_pc(d, _k, _t, _s, _ns, _x0, yy, 1, _b, cols=[_ns])
    CENTER.append((_k, _x0, yy, ww, hh)); yy += hh + 170
w3 = MIDW; h3 = yy - 170 - y
x5 = RX + w3 + 680
w5, h5 = rpc('r8', x5, y, 'PC5 · AMR 제어 PC  (/robot8)', 'PC-R8', PC_BAR['R'])
y2 = y; y3 = CENTER[3][2]
rowA_h = max(h4, h5, h3); rowA_w = x5 + w5 - LEFT; rowB_h = 0
ht3 = CENTER[3][4]; ht8 = CENTER[4][4]; w1 = CENTER[0][3]; h1 = CENTER[0][4]
w2 = CENTER[1][3]; h2 = CENTER[1][4]
W = max(rowA_w, x5 + w5 - LEFT) + LEFT
H = y3 + max(ht3, ht8) + 60
d.text(LEFT, 30, 4000, 46, 'MRO Multi-AMR — 시스템 플로우차트', 30, '#111111', True)
d.text(LEFT, 78, 5200, 90,
       'ROKEY 9기 2PM · Phase 1 · 로봇 2대(/robot3 · /robot8) · 웹캠 2대 · 관제 1대<br>'
       'PC 한 대 = 테두리 하나. 테두리 안이 그 PC 에서 도는 노드, 노드 안이 그 노드가 하는 일의 순서다.<br>'
       'PC 를 건너는 화살표에 ROS 토픽 이름 · 타입 · 값의 뜻을 적었다.', 12)
# ── 범례 ────────────────────────────────────────────────────────────────────
LX, LY = LEFT, TOP - 210
d.text(LX, LY, 300, 20, '읽는 법', 13, '#111111', True)
leg = [('start', '시작'), ('do', '하는 일'), ('ask', '판단'), ('else', '갈라진 쪽의 처리'),
       ('io', '계약된 토픽·서비스'), ('dev', '외부 장비·설정')]
lx = LX
for k, lab in leg:
    w = 26 + len(lab) * 12
    d.rect(lx, LY + 24, 22, 16, '', k)
    d.text(lx + 28, LY + 25, w, 16, lab, 11, '#333333')
    lx += w + 26
d.text(LX, LY + 50, 2500, 46,
       '<b>선 색 = 토픽 갈래</b>　'
       '<span style="color:#1F5FBF">━ 식별·대상</span> (웹캠 등록 · target/*)　'
       '<span style="color:#7A3E9D">━ 미션 명령</span> (mission/assigned · finish)　'
       '<span style="color:#0E7C7B">━ 상태 보고</span> (approach · shuttle · zone_event)<br>'
       '　　　　　　　　　　'
       '<span style="color:#55606B">━ 장비 원본</span> (oakd · scan · battery · amcl)　'
       '<span style="color:#D97706">┅ 서비스</span> (파선)　'
       '<span style="color:#C0272D">━ 안전</span> (estop · cmd_vel)　'
       '<span style="color:#4A4A4A">━ 노드 안에서 다음 칸으로 · ┅ 앞 칸으로 되돌아간다</span>', 11)
d.text(LX, LY + 100, 2500, 60,
       '<b>말풍선</b>　토픽 이름은 <b>제 선 위에</b> 흰 바탕 상자로 얹었다 — 상자를 덮고 있는 선이 그 토픽의 선이다.<br>'
       '<b>둥근 점</b>　한 토픽이 여럿에게 갈라지는 자리. <b>이름과 타입은 줄기에 한 번만</b> 적고, '
       '갈래에는 받는 노드가 그것으로 무엇을 하는지만 적었다.<br>'
       '<b>테두리</b>　바깥 = PC 한 대　·　안쪽 = 그 PC 에서 도는 노드 하나　·　'
       '<b>PC4 와 PC5 는 똑같다</b> — 노드도 토픽도 같고 이름 앞만 /robot3 · /robot8 로 갈린다', 11)

import arrows
BOX = {c[1]: (c[4], c[5], c[6], c[7]) for c in d.cells if c[0] == 'v'}
arrows.draw(d, ANCH, dict(x4=x4, x5=x5, y2=y2, y3=y3, TOP=TOP, LEFT=LEFT, RX=RX,
                          bot=y3 + max(ht3, ht8) + 90,
                          rowA_h=rowA_h, rowB_h=rowB_h, W=W, H=H,
                          pc1=CENTER[0][1:], pc2=CENTER[1][1:], pc3=CENTER[2][1:],
                          mid=(RX, RX + w3), pc4=(x4, y, w4, h4), pc5=(x5, y, w5, h5),
                          t3=CENTER[3][1:], t8=CENTER[4][1:]), BOX)
import spread
BOX2 = {c[1]: (c[4], c[5], c[6], c[7]) for c in d.cells if c[0] == 'v'}   # 갈림점까지 포함
spread.run(d, BOX2)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                   'docs', 'diagrams', 'system_flowchart.drawio')
d.write(os.path.normpath(OUT), [('시스템 플로우차트', d.cells)])
print('→', os.path.normpath(OUT))
print(f'초안 {W:.0f} × {H:.0f} · 셀 {len(d.cells)}')
print('노드 키:', sorted(ANCH))
