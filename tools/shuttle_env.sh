# 셔틀 시험 터미널의 첫 줄. source 로 실행한다 — 실행(./)하면 아무 효과가 없다.
#
#   source tools/shuttle_env.sh          # source 4종 (모든 터미널)
#   source tools/shuttle_env.sh node     # + ROS_SUPER_CLIENT=False  (노드만 띄우는 터미널)
#   source tools/shuttle_env.sh nav2     # + ROS_SUPER_CLIENT=False  (T8 Nav2 터미널)
#   source tools/shuttle_env.sh build    # + colcon build 4패키지 후 source
#
# 왜 sh 파일을 따로 두나 — tools/shuttle_preflight.sh 는 자식 프로세스라 거기서 source 해도
#   부모 터미널의 환경은 바뀌지 않는다. 환경을 '바꾸는' 것과 '검사하는' 것은 분리해야 한다.
#   9/7 실패 1번(디스커버리 설정 누락)이 이 한 줄로 사라진다.

if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
  echo "이 파일은 source 로 실행해야 한다:  source tools/shuttle_env.sh"
  exit 1
fi

# 파이프나 $(...) 안에서 source 하면 서브셸이라 환경이 지금 터미널에 남지 않는다.
#   `source tools/shuttle_env.sh build | tail` 처럼 쓰면 조용히 아무 일도 안 일어난다.
if [ "${BASHPID:-$$}" != "$$" ]; then
  echo "경고: 서브셸에서 source 됐다 (파이프·\$(...) 안). 환경이 이 터미널에 남지 않는다."
  echo "      파이프 없이 그냥:  source tools/shuttle_env.sh ${1:-}"
fi

_SE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_SE_WS="$_SE_DIR/ros2_ws"
_SE_MODE="${1:-}"

# 모르는 모드는 **소리를 낸다 (9/10).** 예전에는 조용히 아래 else 로 떨어져 ROS_SUPER_CLIENT=True
#   가 됐다. 그러면 `node` 를 아직 안 받은 PC 에서 `source tools/shuttle_env.sh node` 를 쳐도
#   아무 일도 안 일어나고 **친 사람이 그걸 알 방법이 없다** — 위 9/8 의 '조용히 False 가 된다' 와
#   정확히 같은 종류의 함정이다. 오타(`nav`·`Node`)도 여기서 걸린다.
case "$_SE_MODE" in
  ''|node|nav2|build) ;;
  *)
    echo "경고: '$_SE_MODE' 는 모르는 모드다. 아는 것은 (없음)·node·nav2·build 뿐이다."
    echo "      이대로 두면 ROS_SUPER_CLIENT=True 로 간다. 저장소가 최신인지 확인할 것:"
    echo "      cd $_SE_DIR && git pull --ff-only && git log --oneline -1"
    ;;
esac

# ① 디스커버리 (ROS_DOMAIN_ID·ROS_DISCOVERY_SERVER). 이것이 빠지면 다른 터미널의 노드를 영원히 못 찾는다
source /etc/turtlebot4_discovery/setup.bash

# ② turtlebot4_ws — 셔틀 노드의 turtlebot4_navigation 임포트
source "$HOME/turtlebot4_ws/install/setup.bash"

# ③ 빌드 (요청했을 때만)
if [ "$_SE_MODE" = "build" ]; then
  echo "빌드: mro_interfaces mro_bringup mro_navigation mro_fleet"
  ( cd "$_SE_WS" && colcon build --symlink-install \
      --packages-select mro_interfaces mro_bringup mro_navigation mro_fleet ) || {
    echo "빌드 실패 — 위 오류를 먼저 고칠 것"; return 1
  }
fi

# ④ mro-multi-amr 작업공간
if [ -f "$_SE_WS/install/setup.bash" ]; then
  source "$_SE_WS/install/setup.bash"
else
  echo "install/ 이 없다 — 먼저 빌드할 것:  source tools/shuttle_env.sh build"
  return 1
fi

# 슈퍼클라이언트를 끄는 터미널은 T8(Nav2) 과 `node` 모드뿐이다. 나머지는 True 여야
# ros2 CLI 가 전체 노드를 본다.
#
# **여기서 True 를 명시적으로 넣는다 (9/8).** /etc/turtlebot4_discovery/setup.bash 는
#   `[ -t 0 ] && export ROS_SUPER_CLIENT=True || export ROS_SUPER_CLIENT=False`
# 로 **stdin 이 TTY 일 때만** True 를 준다. 사람이 치는 터미널은 True 지만 스크립트·SSH 비대화
# 실행·CI 에서는 조용히 False 가 되고, 그러면 디스커버리 서버가 전체 토픽·노드를 안 넘겨줘서
# `ros2 topic list` 가 거의 비어 나온다 — '노드가 안 보인다' 로 오진하기 딱 좋다 (가이드 D-5).
#
# ─ `node` 모드는 왜 있나 (9/10, 2대 동시 기동 실패 진단) ────────────────────────
# **아직 실주행 검증 전이다. 절차서 기본값이 아니고, 2대가 안 뜰 때 쓰는 조치다**
#   (docs/demo_run_procedure.md 6절).
#
# 슈퍼클라이언트는 '자기에게 필요한 것' 이 아니라 디스커버리 서버가 가진 **전체 참가자
# 명단**을 받는다. 그래서 슈퍼클라이언트가 늘수록, 참가자 하나가 뜰 때마다 그 엔드포인트
# 전부가 슈퍼클라이언트 수만큼 무선으로 복제된다.
#
# 절차서대로 띄우면 로봇 PC 한 대에 슈퍼클라이언트가 15벌이다 — T2 bag 1 · T3 localization 3
#   (map_server·amcl·lifecycle_manager) · T4 rviz 1 · T5 리그 2 · T6·T7 2 · T9 리그 2 ·
#   T10 1 · T11~T13 echo 3. **2대면 30벌.** 시스템 전체 참가자는 약 75벌이다.
# 여기에 Nav2 는 turtlebot4_navigation/nav2.launch.py 가 use_composition 을 'False' 로
#   하드코딩해서 **프로세스가 11개**다. 두 번째 Nav2 를 켜는 순간이 '명단이 제일 클 때' 와
#   '참가자 11개가 한꺼번에 들어올 때' 가 겹치는 지점이다.
#
# **다만 이것이 T8 실패의 유일한 원인은 아니다 (9/10 밤 bag 으로 정정).** 20:53 은 먼저 띄운
#   로봇8 이 bond 실패했고, 21:08 로봇3 은 초기자세가 아예 안 가서 실패했다 (셔틀 노드 로그
#   0건). '항상 두 번째가 실패한다' 는 성립하지 않는다 — 부하는 실패를 쉽게 만들 뿐이다.
#   증상을 보면 먼저 절차서 6-2 로 원인을 가른다.
#
# 전체 명단이 실제로 필요한 것은 **그래프를 조회하는 것들뿐**이다:
#   ros2 topic list/echo/hz · ros2 node list · ros2 param get · ros2 service call ·
#   ros2 bag record (타입 확인에 get_topic_names_and_types 를 쓴다)
# ros2 run / ros2 launch 로 노드만 띄우는 터미널은 자기가 쓰는 토픽만 매칭되면 되고, 그
#   매칭은 일반 클라이언트에게도 서버가 걸러서 준다. → T3·T5·T6·T7·T9·T10 이 `node` 다.
#   **PC 당 9벌, 2대면 18벌**이 명단 배포 대상에서 빠진다.
#
# **위 9/8 경고와 헷갈리지 말 것** — '조용히 False 가 되어 토픽이 안 보인다' 는
#   `ros2 topic list` 가 비어 나오는 **CLI 터미널** 이야기다. 노드만 띄우는 터미널은
#   해당 없다. 그래도 틀렸다면 증상이 기동 2분 안에 드러난다 (T6 초기자세 미전송,
#   T11·T12 상태 없음) — 그때는 `node` 를 빼고 다시 띄우면 원래대로다.
if [ "$_SE_MODE" = "nav2" ] || [ "$_SE_MODE" = "node" ]; then
  export ROS_SUPER_CLIENT=False
  _SE_WHO=$([ "$_SE_MODE" = "nav2" ] && echo "Nav2 터미널" || echo "노드 터미널")
  echo "환경 준비됨 ($_SE_WHO) — ROS_DOMAIN_ID=$ROS_DOMAIN_ID · ROS_SUPER_CLIENT=False"
  unset _SE_WHO
else
  export ROS_SUPER_CLIENT=True
  echo "환경 준비됨 — ROS_DOMAIN_ID=$ROS_DOMAIN_ID · ROS_SUPER_CLIENT=True · 디스커버리 $ROS_DISCOVERY_SERVER"
fi

unset _SE_DIR _SE_WS _SE_MODE
