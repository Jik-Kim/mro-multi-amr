# fleet_manager 가 로봇별로 집계해 발행 (갱신 주기 ≤ 2 s, BR-T08)
std_msgs/Header header
string robot_ns
string state            # approach/status 문자열 + STANDBY / PAIRED / SHUTTLE / DOCKED
float32 battery         # 0.0 ~ 1.0
float64 x               # map 좌표
float64 y
float64 yaw
string mission_id
string paired_worker
bool estop              # safety_watchdog 가 정지 중이면 true
