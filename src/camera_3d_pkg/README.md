# Camera 3D Package

Professional-grade 3D RGB-D camera driver package for ROS2.

## Features

- **Multi-Camera Support**: Simulated, RealSense, Kinect compatible
- **Complete RGB-D Streams**: RGB, Depth, and Infrared streams
- **Point Cloud Generation**: Real-time 3D point cloud processing
- **Camera Services**: Capture, configuration, and info queries
- **TF Broadcasting**: Automatic coordinate frame management
- **High Quality**: Production-ready error handling and logging

## Package Structure

```
camera_3d_pkg/
├── camera_3d_pkg/
│   ├── __init__.py
│   ├── camera_3d_node.py      # Main camera driver node
│   └── pointcloud_node.py     # Point cloud processing node
├── launch/
│   ├── camera_3d.launch.py    # Full system launch
│   └── camera_driver.launch.py # Driver only launch
├── srv/
│   ├── CaptureFrame.srv       # Capture image service
│   ├── GetCameraInfo.srv      # Get camera info service
│   └── ConfigureCamera.srv    # Configure camera service
└── test/
    └── test_camera_3d_pkg.py
```

## Installation

```bash
cd ~/ros2_ws/src
# Already created by setup
cd ~/ros2_ws
colcon build --packages-select camera_3d_pkg
source install/setup.bash
```

## Running

### Basic Launch (Simulated Camera)

```bash
ros2 launch camera_3d_pkg camera_driver.launch.py
```

### Full System with Point Cloud Processing

```bash
ros2 launch camera_3d_pkg camera_3d.launch.py
```

### With Custom Parameters

```bash
ros2 launch camera_3d_pkg camera_3d.launch.py \
    camera_type:=realsense \
    rgb_fps:=30 \
    depth_fps:=30 \
    enable_pointcloud:=true
```

## Topics

### Published Topics

- **`/camera/rgb/image_raw`** (sensor_msgs/Image)
  - RGB color image from camera

- **`/camera/rgb/camera_info`** (sensor_msgs/CameraInfo)
  - Camera intrinsic parameters for RGB

- **`/camera/depth/image_raw`** (sensor_msgs/Image)
  - Depth map in millimeters (mono16 encoding)

- **`/camera/depth/camera_info`** (sensor_msgs/CameraInfo)
  - Camera intrinsic parameters for depth

- **`/camera/ir/image_raw`** (sensor_msgs/Image)
  - Infrared image (if enabled)

- **`/camera/points`** (sensor_msgs/PointCloud2)
  - 3D point cloud with RGB color

- **`/camera/points_filtered`** (sensor_msgs/PointCloud2)
  - Filtered and downsampled point cloud

- **`/camera/object_centroid`** (std_msgs/Float32MultiArray)
  - Centroid of detected objects [x, y, z]

- **`/camera/plane_detection`** (std_msgs/Float32MultiArray)
  - Detected plane coefficients [a, b, c, d]

## Services

### `/camera/capture_frame` (camera_3d_pkg/CaptureFrame)

Capture and optionally save RGB and depth frames.

```bash
ros2 service call /camera/capture_frame camera_3d_pkg/srv/CaptureFrame \
  '{camera_type: simulated, save_image: true, output_path: "/tmp"}'
```

### `/camera/get_camera_info` (camera_3d_pkg/GetCameraInfo)

Query camera specifications.

```bash
ros2 service call /camera/get_camera_info camera_3d_pkg/srv/GetCameraInfo \
  '{camera_id: "0"}'
```

Response includes:
- Camera model
- Serial number
- Intrinsic parameters (fx, fy, cx, cy)
- Resolution and baseline

### `/camera/configure_camera` (camera_3d_pkg/ConfigureCamera)

Configure camera parameters at runtime.

```bash
ros2 service call /camera/configure_camera camera_3d_pkg/srv/ConfigureCamera \
  '{camera_id: "0", enable_rgb: true, enable_depth: true, enable_ir: false, rgb_fps: 30, depth_fps: 30}'
```

## Parameters

### Camera 3D Node

- `camera_type` (string, default: "simulated")
  - Type of camera: "simulated", "realsense", "kinect"

- `camera_id` (string, default: "0")
  - Camera device ID or index

- `rgb_fps` (int, default: 30)
  - RGB frame rate in Hz

- `depth_fps` (int, default: 30)
  - Depth frame rate in Hz

- `depth_width` (int, default: 640)
  - Depth image width in pixels

- `depth_height` (int, default: 480)
  - Depth image height in pixels

- `enable_rgb` (bool, default: true)
  - Enable RGB stream publishing

- `enable_depth` (bool, default: true)
  - Enable depth stream publishing

- `enable_ir` (bool, default: false)
  - Enable infrared stream publishing

- `enable_pointcloud` (bool, default: true)
  - Enable 3D point cloud generation

- `tf_prefix` (string, default: "camera")
  - Prefix for transform frame IDs

- `output_dir` (string, default: "/tmp/camera_captures")
  - Directory for saving captured images

### Point Cloud Node

- `camera_frame` (string, default: "camera_depth_optical_frame")
  - Camera frame ID for processing

- `min_distance` (float, default: 0.1)
  - Minimum distance for point filtering (meters)

- `max_distance` (float, default: 5.0)
  - Maximum distance for point filtering (meters)

- `voxel_size` (float, default: 0.01)
  - Voxel size for downsampling (meters)

- `enable_filtering` (bool, default: true)
  - Enable distance filtering

## Transform Frames

The package broadcasts the following TF frames:

```
world
└── camera_link
    ├── camera_rgb_optical_frame
    └── camera_depth_optical_frame
```

## Visualization

### RViz2

```bash
ros2 launch camera_3d_pkg camera_3d.launch.py
rviz2
```

In RViz2, add displays:
1. Image: `/camera/rgb/image_raw`
2. Image: `/camera/depth/image_raw`
3. PointCloud2: `/camera/points`

### rqt

```bash
rqt
```

Add plugins:
- Image View (subscribe to RGB or depth)
- TF Tree (view coordinate frames)

## Example Usage

### Python Client Example

```python
import rclpy
from rclpy.node import Node
from camera_3d_pkg.srv import CaptureFrame, GetCameraInfo
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraClient(Node):
    def __init__(self):
        super().__init__('camera_client')
        self.bridge = CvBridge()
        
        # Subscribe to images
        self.rgb_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.rgb_callback, 10
        )
        
        # Create service clients
        self.capture_cli = self.create_client(
            CaptureFrame, '/camera/capture_frame'
        )
        self.info_cli = self.create_client(
            GetCameraInfo, '/camera/get_camera_info'
        )
    
    def rgb_callback(self, msg):
        # Process RGB image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.get_logger().info(f"Received RGB image: {cv_image.shape}")
    
    def capture_frame(self):
        req = CaptureFrame.Request()
        req.camera_type = "simulated"
        req.save_image = True
        req.output_path = "/tmp/captures"
        
        future = self.capture_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info(f"Capture: {future.result().message}")

if __name__ == '__main__':
    rclpy.init()
    client = CameraClient()
    client.capture_frame()
    rclpy.spin(client)
```

## Extension Points

### Adding Real Camera Support

1. Modify `init_camera()` in `camera_3d_node.py`
2. Implement device-specific capture logic
3. Update camera parameters in launch files

### Custom Point Cloud Processing

1. Create a new node subscribing to `/camera/points`
2. Implement custom algorithms
3. Publish results to new topics

## Troubleshooting

### No image published
- Check parameter: `enable_rgb=true`, `enable_depth=true`
- Verify camera_type matches your hardware
- Check console for initialization errors

### Point cloud not generated
- Set `enable_pointcloud=true` in parameters
- Check if depth image is being published
- Verify frame IDs in configuration

### Services not responding
- Ensure nodes are running: `ros2 node list`
- Check service availability: `ros2 service list`
- Review node logs: `ros2 node info /camera/camera_3d_node`

## License

Apache License 2.0

## References

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [sensor_msgs](https://github.com/ros2/common_interfaces)
- [cv_bridge](https://github.com/ros-perception/vision_opencv)
