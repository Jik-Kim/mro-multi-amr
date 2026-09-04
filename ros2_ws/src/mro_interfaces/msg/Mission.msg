# 자재 요청 → 셔틀 → 합류 → 추종 → 종료까지 한 사이클. BRD v3.0 S3~S9
std_msgs/Header header
string mission_id
string worker_id
string robot_ns
string bay              # A | B | C (Z-04 베이, ArUco ID 와 동일)
uint8 state
string note

uint8 CREATED=0
uint8 ASSIGNED=1
uint8 SHUTTLE=2         # 도킹 → 베이 이동 중
uint8 AT_BAY=3          # ArUco 확인, 체류 중
uint8 WAITING=4         # Z-05 도착, 작업자 대기
uint8 PAIRED=5          # 재식별 게이트 통과
uint8 FOLLOWING=6
uint8 STANDBY=7
uint8 DONE=8
uint8 FAILED=9
