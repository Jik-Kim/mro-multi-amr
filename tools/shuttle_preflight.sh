#!/usr/bin/env bash
# 셔틀 실주행 시험 사전 점검 — 아무것도 띄우지 않고 '지금 환경이 시험을 견디는지'만 본다.
#
# 왜 있나: 9/7 실주행 실패 세 번은 명령을 틀려서가 아니라 환경 상태를 확인하지 않아서였다
#   (source 누락 / 같은 로봇을 문 중복 세션 / 도크에 안 물린 로봇).
#   docs/shuttle_test_guide.md B-4 의 확인 항목을 사람이 눈으로 하는 대신 여기서 한다.
#
# 쓰는 법 — `source tools/shuttle_env.sh` 를 마친 터미널에서 (가이드 B-2)
#   tools/shuttle_preflight.sh pre       # T1 띄우기 전   : 환경·로봇·중복 노드
#   tools/shuttle_preflight.sh nav       # T3 뜬 뒤       : + Nav2 active
#   tools/shuttle_preflight.sh mission   # 미션 발행 직전 : + 토큰·마커·구독 (기본값)
#
# 이 스크립트는 환경을 검사만 하고 바꾸지 않는다. 자식 프로세스라 여기서 source 해도 부모
#   터미널에 반영되지 않기 때문이다. 환경을 만드는 것은 `source tools/shuttle_env.sh` 쪽이다.
#
#   두 번째 인자로 네임스페이스를 바꾼다: tools/shuttle_preflight.sh mission /robot4
#
# 종료 코드 0 = 전부 통과. 1 = FAIL 있음. FAIL 이면 가이드의 어느 절을 볼지 같이 찍는다.

set -u

STAGE="${1:-mission}"
NS="${2:-/robot3}"
NSB="${NS#/}"                     # 앞 슬래시 없는 형태 (robot3)

case "$STAGE" in
  pre|nav|mission) ;;
  *) echo "쓰는 법: $0 [pre|nav|mission] [네임스페이스]"; exit 2 ;;
esac

if [ -t 1 ]; then R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; B=$'\e[1m'; N=$'\e[0m'
else R=; G=; Y=; B=; N=; fi

FAILS=0
WARNS=0
pass() { printf '  %sPASS%s  %s\n' "$G" "$N" "$1"; }
fail() { printf '  %sFAIL%s  %s\n' "$R" "$N" "$1"; [ $# -ge 2 ] && printf '        → %s\n' "$2"; FAILS=$((FAILS+1)); }
warn() { printf '  %sWARN%s  %s\n' "$Y" "$N" "$1"; [ $# -ge 2 ] && printf '        → %s\n' "$2"; WARNS=$((WARNS+1)); }
sect() { printf '\n%s%s%s\n' "$B" "$1" "$N"; }

# ros2 호출은 디스커버리가 어긋나면 영원히 멈춘다. 전부 timeout 을 씌운다.
r2() { timeout "${1}" ros2 "${@:2}" 2>/dev/null; }

printf '%s셔틀 시험 사전 점검%s  (단계: %s · 네임스페이스: %s)\n' "$B" "$N" "$STAGE" "$NS"

# ────────────────────────────────────────────────────────── 1. 환경 (가이드 B-2)
sect "1. 셸 환경 — source 4종"

if ! command -v ros2 >/dev/null 2>&1; then
  fail "ros2 명령이 없다" "source tools/shuttle_env.sh 부터 (가이드 B-2)"
  echo; echo "환경이 없으면 나머지 검사가 의미 없다. 중단한다."; exit 1
fi
pass "ros2 명령 있음"

if [ -n "${ROS_DOMAIN_ID:-}" ]; then
  pass "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
else
  fail "ROS_DOMAIN_ID 가 비었다" "source tools/shuttle_env.sh — /etc/turtlebot4_discovery/setup.bash 누락이 9/7 최다 원인이다 (가이드 B-2)"
fi

if [ -n "${ROS_DISCOVERY_SERVER:-}" ]; then
  pass "ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"
else
  fail "ROS_DISCOVERY_SERVER 가 비었다" "source tools/shuttle_env.sh (가이드 B-2)"
fi

# 슈퍼클라이언트가 아니면 ros2 node list 가 남의 노드를 못 본다 → 아래 중복 검사가 통째로 거짓 통과한다
if [ "${ROS_SUPER_CLIENT:-}" = "True" ]; then
  pass "ROS_SUPER_CLIENT=True (전체 노드가 보인다)"
else
  warn "ROS_SUPER_CLIENT=${ROS_SUPER_CLIENT:-없음}" \
       "아래 '중복 노드' 검사가 남의 PC 노드를 못 봐서 거짓 통과할 수 있다. 대화형 터미널에서 실행할 것"
fi

for pkg in turtlebot4_navigation mro_navigation mro_bringup mro_fleet; do
  if r2 10 pkg prefix "$pkg" >/dev/null; then
    pass "패키지 $pkg 찾음"
  else
    case "$pkg" in
      turtlebot4_navigation) fail "패키지 $pkg 없음" "turtlebot4_ws source 누락 — 셔틀 노드가 임포트에서 즉사한다. source tools/shuttle_env.sh" ;;
      *) fail "패키지 $pkg 없음" "source tools/shuttle_env.sh build — 미빌드이거나 ws source 누락 (가이드 B-1·B-2)" ;;
    esac
  fi
done

# ────────────────────────────────────────────────────────── 2. 로봇
sect "2. 로봇 $NS"

TOPICS="$(r2 25 topic list)"
NTOPIC="$(printf '%s\n' "$TOPICS" | grep -c "^$NS/")"
if [ "$NTOPIC" -gt 0 ]; then
  pass "$NS 토픽 $NTOPIC 개 보임"
else
  fail "$NS 토픽이 하나도 안 보인다" \
       "PC-F 디스커버리 서버(fastdds discovery -i 1 -l 0.0.0.0 -p 11811)가 로봇 전원보다 먼저 떠야 한다. 가이드 B-0"
fi

# is_docked — 이걸 안 보고 시작하면 미션 수신 직후 getDockedStatus() 가 무한 대기에 빠지고
#   T_lim 도 돌지 않아 FAILED 조차 안 나온다 (가이드 D-2). 시험이 통째로 멈추는 유일한 항목이다.
if printf '%s\n' "$TOPICS" | grep -q "^$NS/dock_status$"; then
  DS="$(timeout 15 ros2 topic echo "$NS/dock_status" --once 2>/dev/null)"
  if printf '%s\n' "$DS" | grep -q 'is_docked: true'; then
    pass "도킹됨 (is_docked: true)"
  elif printf '%s\n' "$DS" | grep -q 'is_docked: false'; then
    fail "로봇이 도크에 안 물려 있다 (is_docked: false)" \
         "이 상태로 미션을 보내면 getDockedStatus() 가 무한 대기한다 — T_lim 도 안 돈다 (가이드 D-2)"
  else
    warn "dock_status 를 15 초 안에 못 받았다" "로봇 전원·디스커버리 확인"
  fi
else
  fail "$NS/dock_status 토픽이 없다" "로봇이 안 떴다"
fi

# ────────────────────────────────────────────────────────── 3. 중복·유령 노드
sect "3. 중복·유령 노드 — 9/7 에 T_lim 120 초를 날린 원인"

NODES="$(r2 25 node list | sort)"
if [ -z "$NODES" ]; then
  warn "노드 목록을 못 받았다" "디스커버리 확인"
else
  # (a) 같은 이름이 두 번 이상 — 다른 PC 나 이전 세션이 같은 로봇을 물고 있다는 뜻
  DUP="$(printf '%s\n' "$NODES" | uniq -d | grep "^$NS/" || true)"
  if [ -z "$DUP" ]; then
    pass "$NS 안에 같은 이름의 노드 중복 없음"
  else
    fail "$NS 노드가 중복이다:" "$(printf '%s' "$DUP" | tr '\n' ' ')"
    printf '        → 라이프사이클 활성화가 끝나지 않아 bt_navigator 가 active 에 못 간다 (가이드 B-4 ②)\n'
  fi

  # (b) 네임스페이스 없이 뜬 localization/nav2 한 벌 — 누가 namespace:= 를 빼고 띄운 것.
  #     [9/7 저녁 실측] 이것은 robot3 에 영향을 주지 않는다. 근거:
  #       · 그쪽 lifecycle_manager 의 node_names 가 ['map_server','amcl'] 이라 상대 이름으로
  #         /map_server·/amcl 로만 풀린다 — /robot3/* 를 관리 대상으로 잡지 않는다
  #       · 토픽이 전부 루트(/tf·/map·/scan·/bond)라 /robot3/tf·/robot3/scan 과 겹치지 않는다
  #       · 루트 /scan 은 발행자가 0 이라 그쪽 amcl 은 영원히 inactive 다
  #     그래서 FAIL 이 아니라 WARN 이다. 진짜 위험한 것은 (a) 의 '완전한 이름이 같은 중복' 이다.
  BARE="$(printf '%s\n' "$NODES" | grep -E '^/(amcl|map_server|lifecycle_manager_localization|lifecycle_manager_navigation|bt_navigator|controller_server|planner_server)$' || true)"
  if [ -z "$BARE" ]; then
    pass "네임스페이스 없는 Nav2/localization 스택 없음"
  else
    warn "네임스페이스 없이 뜬 노드가 있다:" "$(printf '%s' "$BARE" | tr '\n' ' ')"
    printf '        → 남의 세션이 돌고 있다는 신호일 뿐, %s 와는 토픽·관리대상이 겹치지 않는다 (9/7 실측)\n' "$NS"
    printf '        → 단, T1·T3 를 namespace:= 없이 띄우면 그때는 이것과 충돌한다. 항상 namespace 를 줄 것\n'
  fi

  # (c) pre 단계인데 이미 Nav2/AMCL 이 이 네임스페이스에 있다 = 이전 세션이나 다른 PC 의 잔재.
  #     이름만 남고 서비스가 응답하지 않는 '유령' 도 여기 걸린다 — 그 상태로 T3 를 띄우면
  #     이름 충돌로 bt_navigator 가 active 에 도달하지 못한다 (9/7).
  if [ "$STAGE" = "pre" ]; then
    STALE="$(printf '%s\n' "$NODES" | grep -E "^$NS/(amcl|map_server|bt_navigator|controller_server|planner_server|behavior_server|lifecycle_manager_localization|lifecycle_manager_navigation)$" || true)"
    if [ -z "$STALE" ]; then
      pass "$NS 에 Nav2/localization 잔재 없음 (깨끗한 시작)"
    else
      warn "$NS 에 Nav2/localization 노드가 이미 있다:" "$(printf '%s' "$STALE" | tr '\n' ' ')"
      printf '        → T1·T3 를 이미 띄웠다면 정상이다. 아직 안 띄웠다면 이전 세션·다른 PC 의 잔재다\n'
      printf '        → 판별: ros2 lifecycle get %s/bt_navigator 가 아무 응답도 없으면 유령이다\n' "$NS"
      printf '        → 유령이면 그 PC 에서 내리거나, PC-F 의 fastdds discovery 를 재시작한다\n'
    fi
  fi
fi

# ────────────────────────────────────────────────────────── 4. Nav2 (nav 단계부터)
if [ "$STAGE" = "nav" ] || [ "$STAGE" = "mission" ]; then
  sect "4. Nav2 — '떴다' 가 아니라 'active' 여야 한다"

  BT="$NS/bt_navigator"
  if ! printf '%s\n' "$NODES" | grep -q "^$BT$"; then
    fail "$BT 노드가 없다" "T3 (nav2.launch.py) 미기동 — 가이드 B-3 T3"
  else
    ST="$(timeout 15 ros2 lifecycle get "$BT" 2>/dev/null)"
    case "$ST" in
      *"active [3]"*)   pass "$BT = active [3]" ;;
      *"inactive"*)     fail "$BT = $ST" "라이프사이클 활성화가 끝나지 않았다. lifecycle_manager 로그 확인" ;;
      "")               fail "$BT 가 get_state 에 응답하지 않는다" \
                             "노드 이름은 보이는데 응답이 없다 = 죽은 세션이 디스커버리 서버에 남긴 껍데기이거나 중복이다. 3번 항목과 같이 볼 것" ;;
      *)                fail "$BT = $ST" "예상치 못한 상태" ;;
    esac
  fi
fi

# ────────────────────────────────────────────────────────── 5. 미션 발행 직전 (mission 단계)
if [ "$STAGE" = "mission" ]; then
  sect "5. 세마포어 · 마커 · 구독"

  # 세마포어 — 없으면 셔틀이 '단독 모드' 로 통과시켜 판정 5 를 통째로 못 한다 (가이드 A 표)
  if r2 20 service list | grep -q '^/fleet/zone_token$'; then
    pass "/fleet/zone_token 서비스 있음"
  else
    fail "/fleet/zone_token 서비스가 없다" \
         "zone_semaphore 미기동. 셔틀이 '단독 모드' 로 진행해 판정 5(토큰 시점)를 검증할 수 없다"
  fi

  # 마커 주입 — 셔틀의 신선도 판정이 1.0 s 라 '있다' 가 아니라 '계속 온다' 여야 한다
  for t in aruco/id aruco/distance; do
    HZ="$(timeout 12 ros2 topic hz "$NS/$t" --window 5 2>/dev/null | grep -m1 'average rate' | awk '{print $3}')"
    if [ -z "$HZ" ]; then
      fail "$NS/$t 가 발행되지 않는다" "마커 주입 미기동 — 셔틀이 '관측 None' 으로 오도착 판정한다"
    else
      RATE_OK="$(awk -v h="$HZ" 'BEGIN{print (h>=2.0)?"y":"n"}')"
      if [ "$RATE_OK" = y ]; then pass "$NS/$t $HZ Hz"
      else fail "$NS/$t 가 $HZ Hz 로 너무 느리다" "셔틀 신선도 판정이 1.0 s 다. 5 Hz 로 발행할 것"
      fi
    fi
  done

  # 셔틀이 미션 토픽을 실제로 구독하고 있는지 (가이드 B-4 ③)
  SUB="$(timeout 15 ros2 topic info "$NS/mission/assigned" --verbose 2>/dev/null)"
  NSUB="$(printf '%s\n' "$SUB" | grep -c 'Node name: shuttle_mission')"
  if [ "$NSUB" -eq 1 ]; then
    pass "$NS/mission/assigned 를 shuttle_mission 이 구독 중 (1개)"
  elif [ "$NSUB" -eq 0 ]; then
    fail "$NS/mission/assigned 에 shuttle_mission 구독이 없다" "T7 미기동, 또는 -r __ns:=$NS 누락"
  else
    fail "shuttle_mission 구독이 $NSUB 개다" "셔틀 노드가 중복으로 떠 있다. 하나만 남길 것"
  fi
fi

# ────────────────────────────────────────────────────────── 결과
printf '\n%s────────────────────────────────────%s\n' "$B" "$N"
if [ "$FAILS" -eq 0 ] && [ "$WARNS" -eq 0 ]; then
  printf '%s전부 통과%s — 다음 단계로 진행\n' "$G" "$N"; exit 0
elif [ "$FAILS" -eq 0 ]; then
  printf '%sFAIL 없음 · WARN %d건%s — 내용을 보고 진행 여부를 판단할 것\n' "$Y" "$WARNS" "$N"; exit 0
else
  printf '%sFAIL %d건 · WARN %d건%s — 고치기 전에 다음 단계로 넘어가지 말 것\n' "$R" "$FAILS" "$WARNS" "$N"
  printf '증상별 판별은 docs/shuttle_test_guide.md D절\n'; exit 1
fi
