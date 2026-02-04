# Robot Arm Control Configuration

## So-arm101 스펙
ARM_LINK_LENGTHS = [0.04, 0.15, 0.15, 0.04, 0.08, 0.05]  # m
ARM_DOF = 6  # Degrees of Freedom

## 모터 설정
MOTOR_IDS = [1, 2, 3, 4, 5, 6]  # Dynamixel 모터 ID
MOTOR_PORT = '/dev/ttyUSB0'
MOTOR_BAUDRATE = 1000000

## 그리퍼 설정
GRIPPER_MOTOR_ID = 7
GRIPPER_CLOSE_POSITION = 2048  # 닫힘 위치
GRIPPER_OPEN_POSITION = 512     # 열림 위치

## 카메라 설정
CAMERA_DEVICE = '/dev/video0'
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

## YOLO 설정
YOLO_MODEL = 'yolov10m'
YOLO_CONFIDENCE_THRESHOLD = 0.5
YOLO_NMS_THRESHOLD = 0.45

## 작업 관련 설정
PEN_DETECTION_TIMEOUT = 5.0     # 초
APPROACH_HEIGHT = 0.05          # m (펜 위쪽)
LIFT_HEIGHT = 0.15              # m (들어올리는 높이)
SIDE_MOVE_DISTANCE = 0.3        # m (옆으로 이동 거리)

## 역기구학 설정
IK_MAX_ITERATIONS = 1000
IK_ERROR_THRESHOLD = 0.001      # m

## 제어 루프
CONTROL_LOOP_RATE = 50          # Hz (팔 제어)
TASK_LOOP_RATE = 10             # Hz (작업 상태)
VISION_LOOP_RATE = 30           # Hz (카메라 처리)
