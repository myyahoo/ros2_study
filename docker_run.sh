#!/bin/bash

#docker run --rm -it   --runtime=nvidia   --privileged   --device /dev/ttyACM0   --device /dev/ttyACM1   --device /dev/video0   --device /dev/video1   -v /dev:/dev   --network host   --ipc host   --shm-size=4g   -v $PWD:/root/ros2   -w /root/ros2   jazzy_full:latest

# docker xwindow를 사용할려면 terminal xhost +local:
#docker run -it --runtime=nvidia --gpus all --privileged -e DISPLAY=$DISPLAY -e "QT_X11_NO_MITSHM=1" -e YOLO_CONFIG_DIR=/tmp/Ultralytics -v /tmp/.X11-unix:/tmp/.X11-unix --hostname $(hostname) --network host --shm-size=4g --ipc host -v /dev:/dev  -v $PWD:/root/ros2   -w /root/ros2  humbble_full:latest /bin/bash
docker run -it --runtime=nvidia --gpus all -v /dev/dri:/dev/dri --privileged -e DISPLAY=$DISPLAY -e "QT_X11_NO_MITSHM=1" -e YOLO_CONFIG_DIR=/tmp/Ultralytics -v /tmp/.X11-unix:/tmp/.X11-unix --name humble --hostname $(hostname) --network host --shm-size=4g --ipc host -v /dev:/dev  -v $PWD:/home/tech/ros2   -w /home/tech/ros2  osrf/ros:humble-desktop-full  /bin/bash 

#docker exec -it 00 /bin/bash -c 'echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc'
docker exec -it -u tech  -w /home/tech/ros2 16 /bin/bash
