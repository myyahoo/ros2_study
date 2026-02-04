from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 

def generate_launch_description():
    # Launch argument 선언
    video_source_arg = DeclareLaunchArgument(
        'video_source',
        default_value='0',
        description='Video source (0=camera, or file path)'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8m.pt',
        description='Path to YOLO model'
    )
    
    # LaunchConfiguration으로 argument 참조
    video_source = LaunchConfiguration('video_source')
    model_path = LaunchConfiguration('model_path')
    
    node1 = Node(
        package="my_package",
        executable="video_person_detection",
        name="video_person_detection_node",
        output="screen",
        parameters=[
            {"video_source": video_source},
            {"model_path": model_path},
        ]
    )

    node2 = Node(
        package="my_package",
        executable="sub_node",
        name="sub_node",
        output="screen",
    )

    return LaunchDescription([
        video_source_arg,
        model_path_arg,
        node1,
        node2,
    ])