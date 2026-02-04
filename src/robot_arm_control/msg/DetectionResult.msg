# YOLO 감지 결과
builtin_interfaces/Time timestamp
geometry_msgs/Point target_position  # 물체(볼펜)의 3D 위치
float32 confidence                    # 신뢰도 (0.0-1.0)
string object_class                   # 감지된 물체 클래스
geometry_msgs/Pose camera_pose        # 카메라의 기준 프레임에서의 위치
