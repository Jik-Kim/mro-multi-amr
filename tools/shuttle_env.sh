# 셔틀 시험 터미널의 첫 줄. source 로 실행한다 — 실행(./)하면 아무 효과가 없다.
#
#   source tools/shuttle_env.sh          # source 4종 (모든 터미널)
#   source tools/shuttle_env.sh nav2     # + ROS_SUPER_CLIENT=False  (T3 Nav2 터미널만)
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

# T3(Nav2) 터미널만 슈퍼클라이언트를 끈다. 나머지 터미널은 True 여야 ros2 CLI 가 전체 노드를 본다.
#
# **여기서 True 를 명시적으로 넣는다 (9/8).** /etc/turtlebot4_discovery/setup.bash 는
#   `[ -t 0 ] && export ROS_SUPER_CLIENT=True || export ROS_SUPER_CLIENT=False`
# 로 **stdin 이 TTY 일 때만** True 를 준다. 사람이 치는 터미널은 True 지만 스크립트·SSH 비대화
# 실행·CI 에서는 조용히 False 가 되고, 그러면 디스커버리 서버가 전체 토픽·노드를 안 넘겨줘서
# `ros2 topic list` 가 거의 비어 나온다 — '노드가 안 보인다' 로 오진하기 딱 좋다 (가이드 D-5).
if [ "$_SE_MODE" = "nav2" ]; then
  export ROS_SUPER_CLIENT=False
  echo "환경 준비됨 (Nav2 터미널) — ROS_DOMAIN_ID=$ROS_DOMAIN_ID · ROS_SUPER_CLIENT=False"
else
  export ROS_SUPER_CLIENT=True
  echo "환경 준비됨 — ROS_DOMAIN_ID=$ROS_DOMAIN_ID · ROS_SUPER_CLIENT=True · 디스커버리 $ROS_DISCOVERY_SERVER"
fi

unset _SE_DIR _SE_WS _SE_MODE
