#!/bin/bash
# Build script for camera_3d_pkg
set -e

echo "=========================================="
echo "Building camera_3d_pkg"
echo "=========================================="

WORKSPACE_DIR="${PWD}"

# Check if we're in the right directory
if [ ! -f "setup.py" ] || [ ! -d "src" ]; then
    echo "Error: Please run this script from the workspace root"
    exit 1
fi

# Build the package
echo "Building camera_3d_pkg..."
colcon build --packages-select camera_3d_pkg

# Source the installation
source install/setup.bash

echo ""
echo "=========================================="
echo "Build completed successfully!"
echo "=========================================="
echo ""
echo "To run the camera driver:"
echo "  ros2 launch camera_3d_pkg camera_driver.launch.py"
echo ""
echo "To run the full system:"
echo "  ros2 launch camera_3d_pkg camera_3d.launch.py"
echo ""
echo "To test services:"
echo "  ros2 service call /camera/get_camera_info camera_3d_pkg/srv/GetCameraInfo '{camera_id: \"0\"}'"
echo ""
