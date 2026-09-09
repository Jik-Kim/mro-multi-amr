# RobotControlNode 인스턴스가 담당 AMR 상태를 집계해 발행
std_msgs/Header header
string robot_ns
string state            # approach/status 문자열 + STANDBY / PAIRED / SHUTTLE / DOCKED
string approach_state   # approach/status 최신 원문
string shuttle_state    # shuttle/status 최신 원문
string pair_state       # PAIRED / GATE_FAIL / NO_TARGET
float32 battery         # 0.0 ~ 1.0
float64 x               # map 좌표
float64 y
float64 yaw
string mission_id
string paired_worker
bool estop              # safety_watchdog 가 정지 중이면 true
