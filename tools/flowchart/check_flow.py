# -*- coding: utf-8 -*-
"""흐름이 끊긴 데를 찾는다.

  ① 갈래의 첫 칸에 들어오는 화살표가 있는가
     — 스스로 도는 것(타이머 · 기동 · 감시)이 아니면 누가 시작시키는지 밝혀야 한다
  ② 갈래의 마지막 칸에서 나가는 것이 있는가
     — 내보내거나 · 다른 칸으로 넘기거나 · 「여기서 끝」이라고 밝히거나 셋 중 하나

check.py 가 '선이 상자를 뚫는가' 를 보는 것과 짝이다. 둘 다 0건이어야 내보낸다.
"""
import os, re, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import nodes as N

SELF = ('타이머', '시작', '돈다', '감시', '갈래', '받으면', '0.05초마다', '무엇을 하나')
END = ('여기서 끝', '끝난다', '답한다', '알린다', '기록 파일에 남긴다', '버린다', '넘어간다',
       '표시', '폐기', '물러난다', '돌린다', '둔다', '지운다', '비운다', '응답',
       '내보낸다', '파일에 덧붙여', '읽는다', '이어서 한다')
CALLS = ('inner', 'near', 'hlink', 'link', 'bundle')

def args(src, fn):
    """그 함수 호출의 인자를 괄호 짝을 맞춰 잘라 낸다."""
    out = []
    for m in re.finditer(rf'\b{fn}\(', src):
        i = m.end(); depth = 1; start = i
        while i < len(src) and depth:
            if src[i] in '([': depth += 1
            elif src[i] in ')]': depth -= 1
            i += 1
        body = src[start:i-1]
        parts, d, cur = [], 0, ''
        for ch in body:
            if ch in '([': d += 1
            elif ch in ')]': d -= 1
            if ch == ',' and d == 0: parts.append(cur); cur = ''
            else: cur += ch
        parts.append(cur)
        out.append([p.strip() for p in parts])
    return out

def anchors(expr):
    """인자에서 앵커 이름만 뽑는다 — 'fm:in' · k('amr_detection', 'wd_in') · t + ':out'"""
    return set(re.findall(r"[:'\"]\s*([a-z0-9_]+)'", expr)) | set(re.findall(r":([a-z0-9_]+)'", expr))

def main():
    src = open(os.path.join(HERE, 'arrows.py'), encoding='utf-8').read()
    inc, out = set(), set()
    for fn in CALLS:
        for a in args(src, fn):
            if len(a) < 2: continue
            out |= anchors(a[0])
            inc |= anchors(a[1])
    bad_in, bad_out = [], []
    NODES = [N.WEBCAM1, N.WEBCAM2] + N.PC3_NODES + N.ROBOT_NODES
    for n in NODES:
        for ci, ch in enumerate(n['chains']):
            steps = [s for s in ch if s['k'] != 'dev']
            if not steps: continue
            f, l = steps[0], steps[-1]
            ft = f['t'].split('\n')[0]
            if not any(k in ft for k in SELF) and not (f.get('a') and f['a'] in inc):
                bad_in.append((n['name'], ci, ft[:52]))
            if l['k'] != 'io' and not any(k in l['t'] for k in END) \
               and not (l.get('a') and l['a'] in out):
                bad_out.append((n['name'], ci, l['t'].split('\n')[0][:52]))
    for t, rows in (('들어오는 화살표가 없는 갈래', bad_in), ('끝이 밝혀지지 않은 갈래', bad_out)):
        print(f'\n{t}: {len(rows)}건')
        for r in rows: print(f'   {r[0]:24} 갈래{r[1]}  {r[2]}')
    tot = sum(len(c) for n in NODES for c in n['chains'])
    print(f'\n노드 {len(NODES)}개 · 상자 {tot}개')
    return 1 if (bad_in or bad_out) else 0

if __name__ == '__main__':
    sys.exit(main())
