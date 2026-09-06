# 웹캠1이 관제센터(Z-02)에서 등록한 작업자 외형 서술자. BRD v3.1 S1 / BR-T28
std_msgs/Header header
string worker_id        # W1, W2 (세션 내 고유)
string outfit_class     # worker_a | worker_b (작업별 복장 클래스)
float32[] color_hist    # HSV 색 히스토그램. 길이·bin 수는 docs/interfaces.md 에서 합의
string robot_ns         # 배정된 AMR 네임스페이스 (/robot3, /robot4). fleet_manager 가 채움
