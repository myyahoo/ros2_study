# 로봇 암의 현재 상태 정보
builtin_interfaces/Time timestamp
float32[6] joint_angles  # 각 관절의 현재 각도 (단위: 라디안)
bool gripper_open        # 그리퍼 상태
string state             # "idle", "moving", "gripping", "error"
