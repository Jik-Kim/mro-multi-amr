# 개입 이벤트 (IPH 산출용, BR-T26). 운영자 버튼 또는 노드 자동 기록
std_msgs/Header header
string robot_ns
string kind             # REPAIR | MANUAL_MOVE | RESTART | TOW | MANUAL_DELIVERY
                        # | MANUAL_PAIR(FR-13 폴백 발동, v1.12 — 개입으로 계상한다)
                        # | SAFETY_POLICY(개입 아님, 별도 집계)
string note
