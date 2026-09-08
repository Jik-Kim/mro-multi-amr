#!/usr/bin/env bash
# 로봇 명령을 대역으로 바꾸고, 실제 DDS/TF는 별도 도메인의 localhost에서 시험한다.
# 인자는 pytest에 전달한다. 예: bash tools/test_shuttle_offline.sh -q -s
set -e
SHUTTLE_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$SHUTTLE_REPO/ros2_ws/install/setup.bash"
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT ROS_STATIC_PEERS
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=201
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR="$(mktemp -d /tmp/shuttle-offline-ros.XXXXXX)"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="$SHUTTLE_REPO/ros2_ws/src/mro_navigation:${PYTHONPATH:-}"
cd "$SHUTTLE_REPO"
/usr/bin/python3 -m pytest \
    ros2_ws/src/mro_navigation/test/test_shuttle_mission_fsm.py \
    ros2_ws/src/mro_navigation/test/test_shuttle_arrival.py \
    ros2_ws/src/mro_navigation/test/test_shuttle_tf_updates.py "$@"
