# -*- coding: utf-8 -*-
"""PC 를 건너는 계약 화살표. 출처는 docs/interfaces.md 4절.
   노드 옆구리 → 열 사이 세로 통로 → 행 사이 가로 통로 → 상대 노드 옆구리.
   통로는 전부 빈 자리라 어떤 상자도 지나지 않는다."""
from engine import TOPIC, CMD, STAT, SENS, SVC, SAFE

def draw(d, A, G, BOX):
    """BOX = 상자 id → (x, y, w, h). 'noderef:앵커' 로 상자에 직접 꽂는다."""
    def _sp(k):
        return k.split(':') if ':' in k else (k, None)
    def fr(k):
        n, a_ = _sp(k)
        return A[n]['anch'].get(a_, A[n]['frame']) if a_ else A[n]['frame']
    def bx(k):
        n, a_ = _sp(k)
        if a_ and a_ in A[n]['anch']:
            i = A[n]['anch'][a_]
            return list(BOX[i])
        return A[n]['box']
    def col(k):
        """그 노드가 속한 열의 좌우 끝. 열 안에서 제일 넓은 노드를 기준으로 빠져나간다."""
        c = A[_sp(k)[0]].get('col')
        if c: return c[0], c[0] + c[1]
        x, y, w, h = A[_sp(k)[0]]['box']; return x, x + w
    c1 = [G['y2'] - 268 + i * 25 for i in range(11)]
    c2 = [G['y3'] - 250 + i * 30 for i in range(5)]
    seq = [0]
    GL, GRr = G['mid']                                    # 가운데 열의 좌우 끝
    def cg(rp):
        """가운데 열 ↔ 로봇 PC 사이 통로. 가운데 열 '전체' 밖으로 빼야 안쪽 PC 를 뚫지 않는다."""
        return ((G['pc4'][0] + G['pc4'][2], GL) if rp == 'r3' else (GRr, G['pc5'][0]))

    def link(ka, kb, lab, ch, color=TOPIC, dash=False, wid=1.6):
        seq[0] += 1; off = (seq[0] % 6) * 8
        ax, ay, aw, ah = bx(ka); tx0, ty, tw, th = bx(kb)
        al, ar = col(ka); tl, tr = col(kb)
        right = (tx0 + tw / 2) > (ax + aw / 2)
        ex = (ar + 22 + off) if right else (al - 22 - off)
        tx = (tl - 22 - off) if right else (tr + 22 + off)
        d.edge(fr(ka), fr(kb), lab, color=color, dashed=dash, w=wid, lx=0.78,
               pts=[(ex, ay + ah / 2), (ex, ch), (tx, ch), (tx, ty + th / 2)],
               exit=(1, .5) if right else (0, .5), entry=(0, .5) if right else (1, .5))

    def nb(k):
        """그 앵커가 든 노드의 테두리."""
        return A[_sp(k)[0]]['box']

    def route(ka, kb, lab, vx, color=TOPIC, dash=False, lx=0.88):
        """상자 → 상자. 상자 왼쪽 빈 통로로 빠져 노드 사이 빈 줄을 타고 건너간다.
           통로·빈 줄은 어떤 상자도 없는 자리라 무엇도 뚫지 않는다."""
        ax, ay, aw, ah = bx(ka); tx0, ty, tw, th = bx(kb)
        anx, any_, anw, anh = nb(ka); bnx, bny, bnw, bnh = nb(kb)
        ax0 = ax - 30; bx0 = tx0 - 30
        # 노드 밖으로 빠지는 줄 — 상대가 아래면 아래로, 위면 위로
        aout = (any_ + anh + 16) if (bny + bnh / 2) > (any_ + anh / 2) else (any_ - 16)
        bout = (bny - 16) if (bny + bnh / 2) > (any_ + anh / 2) else (bny + bnh + 16)
        pts = [(ax0, ay + ah / 2), (ax0, aout), (vx, aout), (vx, bout), (bx0, bout), (bx0, ty + th / 2)]
        d.edge(fr(ka), fr(kb), lab, color=color, dashed=dash, w=1.6, lx=lx,
               pts=pts, exit=(0, .5), entry=(0, .5))

    def gutter(ka, kb, slot, gapx=None):
        al, ar = col(ka); tl, tr = col(kb)
        if gapx is not None: gap0, gap1 = gapx
        elif (nb(kb)[0] > nb(ka)[0]): gap0, gap1 = ar, tl
        else: gap0, gap1 = tr, al
        return gap0 + (gap1 - gap0) * (0.05 + 0.037 * slot)

    # 통로마다 글자 높이를 따로 관리한다 — 같은 통로의 글자는 세로로 확실히 벌린다
    LAD = {}
    LGAP = 150
    def label_at(gm, lo, hi):
        """gm = 그 통로의 가운데 x. 같은 통로 글자끼리 LGAP 이상 떨어뜨린다."""
        band = LAD.setdefault(round(gm / 240), [])
        lo, hi = min(lo, hi), max(lo, hi)
        want = (lo + hi) / 2
        best, bestd = None, 1e18
        y = lo + 40
        while y <= hi - 40:
            if all(abs(y - u) >= LGAP for u in band):
                dd = abs(y - want)
                if dd < bestd: best, bestd = y, dd
            y += 16
        if best is None:                      # 자리가 없으면 가장 덜 붐비는 곳으로
            best = max(lo + 40, min(hi - 40, want))
            while any(abs(best - u) < LGAP * 0.6 for u in band) and best < hi - 40:
                best += 24
        band.append(best); return best

    def route(ka, kb, lab, vx, color=TOPIC, dash=False, gm=None):
        """상자 왼쪽 통로 → 노드 밖 → 세로 통로 → 상대 노드로.
           글자는 그 세로 통로 바로 옆, 다른 글자와 겹치지 않는 높이에 놓는다."""
        ax, ay, aw, ah = bx(ka); tx0, ty, tw, th = bx(kb)
        anx, any_, anw, anh = nb(ka); bnx, bny, bnw, bnh = nb(kb)
        ax0 = ax - 30; bx0 = tx0 - 30
        down = (bny + bnh / 2) > (any_ + anh / 2)
        aout = (any_ + anh + 16) if down else (any_ - 16)
        bout = (bny - 16) if down else (bny + bnh + 16)
        pts = [(ax0, ay + ah / 2), (ax0, aout), (vx, aout), (vx, bout), (bx0, bout), (bx0, ty + th / 2)]
        full = [(ax, ay + ah / 2)] + pts + [(tx0, ty + th / 2)]
        segs = [(full[i], full[i + 1]) for i in range(len(full) - 1)]
        tot = sum(abs(b_[0] - a_[0]) + abs(b_[1] - a_[1]) for a_, b_ in segs) or 1
        gm = vx if gm is None else gm
        ly = label_at(gm, min(aout, bout), max(aout, bout))
        acc = 0
        for i, (a_, b_) in enumerate(segs):
            L = abs(b_[0] - a_[0]) + abs(b_[1] - a_[1])
            if i == 3:                      # 세로 통로 구간
                acc += abs(ly - a_[1]); break
            acc += L
        lx = max(-0.98, min(0.98, 2 * acc / tot - 1))
        d.edge(fr(ka), fr(kb), lab, color=color, dashed=dash, w=1.6, lx=lx,
               loff=(0, 0), pts=pts, exit=(0, .5), entry=(0, .5))

    def gmid(ka, kb, gapx=None):
        al, ar = col(ka); tl, tr = col(kb)
        if gapx is not None: g0, g1 = gapx
        elif (nb(kb)[0] > nb(ka)[0]): g0, g1 = ar, tl
        else: g0, g1 = tr, al
        return (g0 + g1) / 2

    def hlink(ka, kb, lab, slot, color=TOPIC, dash=False, gapx=None):
        route(ka, kb, lab, gutter(ka, kb, slot, gapx), color, dash, gm=gmid(ka, kb, gapx))

    def near(ka, kb, lab, color=TOPIC, dash=False):
        seq[0] += 1
        vx = min(col(ka)[0], col(kb)[0]) - 22 - (seq[0] % 4) * 9
        route(ka, kb, lab, vx, color, dash, gm=vx - 172)

    def bundle(srcs, tgts, trunk, branches, slot, color=TOPIC, dash=False, gapx=None):
        """같은 토픽은 줄기 하나로 모았다가 갈림점에서 나눈다."""
        vx = gutter(srcs[0], tgts[0], slot, gapx)
        ys = [bx(k)[1] + bx(k)[3] / 2 for k in srcs + tgts]
        jy = sum(ys) / len(ys) + ((slot * 3) % 11) * 31 - 155
        j = d.rect(vx - 8, jy - 8, 16, 16, '', 'do',
                   extra=f'ellipse;fillColor={color};strokeColor={color};')
        for i, s_ in enumerate(srcs):
            ax, ay, aw, ah = bx(s_); anx, any_, anw, anh = nb(s_)
            ax0 = ax - 30 - i * 9
            aout = (any_ + anh + 16 + i * 8) if jy > (any_ + anh / 2) else (any_ - 16 - i * 8)
            gm = gmid(srcs[0], tgts[0], gapx)
            tlx = None; tloff = None
            if i == 0:                       # 줄기 이름도 사다리에 얹어 다른 글자와 벌린다
                ly = label_at(gm, min(aout, jy), max(aout, jy))
                seg = [(ax, ay + ah / 2), (ax0, ay + ah / 2), (ax0, aout), (vx, aout), (vx, jy)]
                sg = [(seg[q], seg[q + 1]) for q in range(len(seg) - 1)]
                tt = sum(abs(q2[0] - q1[0]) + abs(q2[1] - q1[1]) for q1, q2 in sg) or 1
                aa = sum(abs(sg[q][1][0] - sg[q][0][0]) + abs(sg[q][1][1] - sg[q][0][1]) for q in range(3))
                aa += abs(ly - aout)
                tlx = max(-0.98, min(0.98, 2 * aa / tt - 1)); tloff = (0, 0)
            d.edge(fr(s_), j, trunk if i == 0 else '', color=color, dashed=dash, w=1.8,
                   lx=tlx, loff=tloff,
                   pts=[(ax0, ay + ah / 2), (ax0, aout), (vx, aout)], exit=(0, .5))
        for bi, (k, lab) in enumerate(zip(tgts, branches)):
            tx0, ty, tw, th = bx(k); bnx, bny, bnw, bnh = nb(k)
            bx0 = tx0 - 30 - bi * 9
            off = 16 + (slot % 4) * 8 + bi * 6
            bout = (bny - off) if (bny + bnh / 2) > jy else (bny + bnh + off)
            bly = label_at(gmid(srcs[0], tgts[0], gapx), min(jy, bout), max(jy, bout))
            bseg = [(vx, jy), (vx, bout), (bx0, bout), (bx0, ty + th / 2), (tx0, ty + th / 2)]
            bsg = [(bseg[q], bseg[q + 1]) for q in range(len(bseg) - 1)]
            btt = sum(abs(q2[0] - q1[0]) + abs(q2[1] - q1[1]) for q1, q2 in bsg) or 1
            baa = abs(bly - jy)
            d.edge(j, fr(k), lab, color=color, dashed=dash, w=1.6,
                   lx=max(-0.98, min(0.98, 2 * baa / btt - 1)),
                   loff=(0, 0),
                   pts=[(vx, bout), (bx0, bout), (bx0, ty + th / 2)], entry=(0, .5))

    # ── 가운데 열 안에서 ─────────────────────────────────────────────────
    near('whint:hint', 'kpi:in', '/webcam2/target_hint · TargetHint\n10 Hz · BEST_EFFORT · 유효 2초')
    near('cui:svc_out', 'fm:svc_in', '/fleet/request_mission · /fleet/operator_command\n'
                                     '/fleet/manual_pair · 서비스', SVC, True)
    bundle(['zone:evt'], ['fm:in', 'kpi:in'],
           '/fleet/zone_event · String',
           ['교착 사건으로 기록에 남긴다', '교착이 스스로 풀린 비율의 원천'], 5, STAT,
           gapx=(GL - 210, GL - 40))
    bundle(['wreg:out'], ['whint:wd_in', 'fm:wd_in'],
           '/webcam1/worker_descriptor · WorkerDescriptor\n작업자 번호 · 복장 · 색 지문 128개 · 1초마다',
           ['같은 색 지문으로 재탐지 힌트를 만든다', '등록된 작업자를 배정 명단에 올린다'], 1,
           gapx=(GL - 330, GL - 160))

    # ── 가운데 ↔ 로봇 제어 PC ─────────────────────────────────────────────
    #    통로 슬롯을 갈래끼리 붙여 놓았다 : 1~3 명령 · 6~9 식별 · 11~14 상태
    #    16 서비스 · 18~19 안전 · 21~24 장비
    for rp, tag in (('r3', '/robot3'), ('r8', '/robot8')):
        k = lambda n, a='': f'{rp}_{n}' + (':' + a if a else '')
        t = ('t3' if rp == 'r3' else 't8')
        g = cg(rp)
        bundle(['fm:assigned', 'fm:mission'],
               [k('shuttle_mission', 'assigned_in'), k('amr_detection', 'assigned_in')],
               f'{tag}/mission/assigned · Mission',
               ['실을 자재와 베이 번호를 받는다', '이 로봇이 누구를 맡았는지는 여기서만 안다'], 1, CMD, gapx=g)
        bundle(['fm:finish'],
               [k('return_dock', 'finish_in'), k('follow_target', 'finish_in'),
                k('amr_detection', 'finish_in')],
               f'{tag}/mission/finish · Bool',
               ['복귀를 시작한다', '즉시 추종 중단 (1초 안)', '손으로 지은 짝을 푼다'], 3, CMD, gapx=g)
        hlink('wreg:out', k('amr_detection', 'wd_in'),
              f'/webcam1/worker_descriptor → {tag}\n색 지문을 짝짓기 기준값으로 건다', 6, gapx=g)
        hlink('whint:hint', k('follow_target', 'hint_in'),
              f'/webcam2/target_hint → {tag}\n내 담당 번호와 같을 때만 쓴다', 8, gapx=g)
        bundle([k('amr_detection', 'dist'), k('amr_detection', 'visible')], ['kpi:in'],
               f'{tag}/target/distance · Float32  ·  target/visible · Bool',
               ['추종거리 채점 기준값 · 놓침과 재획득의 원천'], 10, TOPIC, gapx=g)
        hlink(k('follow_target', 'status'), 'fm:in',
              f'{tag}/approach/status · String\nIDLE·SEARCHING·APPROACH·ARRIVED·FOLLOWING·STANDBY·FAILED',
              12, STAT, gapx=g)
        bundle([k('shuttle_mission', 'status'), k('return_dock', 'status')], ['fm:in'],
               f'{tag}/shuttle/status · String',
               ['READY·SHUTTLE·AT_BAY·WAITING·RETURNING·DOCK*·FAILED'], 14, STAT, gapx=g)
        hlink(k('shuttle_mission', 'token'), 'zone:tok_in',
              '/fleet/zone_token · ZoneToken (서비스)\n좁은 목 P1 · 도크 DOCK', 16, SVC, True, gapx=g)
        hlink(k('safety_watchdog', 'estop'), 'fm:in',
              f'{tag}/safety/estop · Bool\n참인 동안 모든 로봇 노드가 주행을 멈춘다', 18, SAFE, gapx=g)
        hlink(k('safety_watchdog', 'cmd'), t + ':in',
              'cmd_vel · TwistStamped\n정지 중 초당 20번 덮어쓴다 — 앞뒤 0 · 회전만', 19, SAFE, gapx=g)
        bundle([t + ':out'], [k('amr_detection', 'cam_in'), k('aruco_detector', 'cam_in')],
               'oakd/rgb/image_raw/compressed · oakd/stereo/…/compressedDepth\noakd/rgb/camera_info',
               ['컬러·거리를 한 쌍으로 묶어 쓴다', '같은 카메라를 아르코 검출도 읽는다'], 21, SENS, gapx=g)
        hlink(t + ':out', k('safety_watchdog', 'scan_in'),
              'scan · LaserScan\n전방 좌우 35° 최단거리', 23, SENS, gapx=g)
        hlink(t + ':out', 'fm:in',
              'battery_state · BatteryState\namcl_pose · PoseWithCovarianceStamped', 24, SENS, gapx=g)

        # ── 같은 PC 안 ────────────────────────────────────────────────────
        near(k('amr_detection', 'map'), k('follow_target', 'map_in'),
             'target/map_point · PointStamped\n대상의 지도 좌표')
        near(k('amr_detection', 'paired'), k('follow_target', 'paired_in'),
             'target/paired · target/worker_id\n참일 때만 따라간다')
        near(k('aruco_detector', 'out'), k('shuttle_mission', 'aruco_in'),
             'aruco/id · aruco/distance · aruco/pose\n베이 도착 판정과 정렬')
        near(k('shuttle_mission', 'arrived'), k('follow_target', 'arrived_in'),
             'shuttle/arrived · Bool\n대기 장소 도착')
        near(k('safety_watchdog', 'estop'), k('follow_target', 'estop_in'), 'safety/estop', SAFE)
