"""Nav2의 최종 출력을 워치독 입력으로 바꾼다. 원본 TB4 설정은 보존한다."""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


SPIN_MAX_RAD_S = 0.3    # Nav2 Spin 상한. TB4 기본 1.0 — 아래 주석
SPIN_MIN_RAD_S = 0.15   # Nav2 Spin 하한. TB4 기본 0.4
INFLATION_RADIUS_M = 0.25   # arena.yaml nav2_inflation_radius (9/6 합의). 9/10 P1 세 번: 0.15·0.12 는 벽 0.20 m STUCK,
                            #   0.25 만 통과(13 s). 워치독 stop_dist 0.30 안으로 Nav2 가 안 들어가게 하는 값이다.
                            #   낮추면 '여유' 가 생기는 게 아니라 Nav2 가 그 여유를 다 써서 벽에 붙는다
COST_SCALING = 10.0         # TB4 기본 4.0. 값이 클수록 비용이 벽 근처에서만 진하다 — rviz 가 얇아 보인다. 행동은 거의 같다
