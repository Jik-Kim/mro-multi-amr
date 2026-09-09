# 로봇별 RobotControlNode가 담당 AMR의 최신 상태를 1 Hz로 집계한다.
std_msgs/Header header  # RobotControlNode가 메시지를 만든 ROS 시각
string robot_ns         # 상태의 소유 AMR namespace: /robot3, /robot8

string state            # 관제 화면에 보여줄 대표 운행 상태
string approach_state   # approach/status에서 받은 접근·추종 최신 상태
string shuttle_state    # shuttle/status에서 받은 셔틀·복귀 최신 상태
string pair_state       # 재식별 판정: PAIRED / GATE_FAIL / NO_TARGET

float32 battery         # battery_state.percentage 잔량, 범위 0.0 ~ 1.0
float64 x               # amcl_pose의 map 기준 x 좌표, m
float64 y               # amcl_pose의 map 기준 y 좌표, m
float64 yaw             # amcl_pose 방향각, rad, 범위 (-pi, pi]

string mission_id       # 현재 배정된 미션 ID, 미배정이면 빈 문자열
string paired_worker    # 현재 페어링된 작업자 ID, 미페어링이면 빈 문자열
bool estop              # safety/estop이 정지 중이면 true
