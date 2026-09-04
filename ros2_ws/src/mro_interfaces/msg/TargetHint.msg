# 웹캠2 전역 추적 → 대상 작업자의 map 좌표 힌트 (재탐지 회전 방향 결정용, BR-T27). 안전 판단에는 쓰지 않는다.
std_msgs/Header header
string worker_id
geometry_msgs/PointStamped point   # frame_id = map
float32 confidence
