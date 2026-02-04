from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        'model1',
        default_value='hhhhh',
        description='Model parameter for pub_node'
    )

    
    pub_node = Node(
        package='my_package',
        executable='pub_node',
        name='pub_node',
        parameters=[{'model': LaunchConfiguration('model1')}] // Set parameter from launch argument
    ) 

    sub_node = Node(
        package='my_package',
        executable='sub_node',
        name='sub_node'
    )

    image_publisher = Node(
        package='my_opencv_pkg',
        executable='img_pub',
        name='image_publisher'
    )

    image_subscriber = Node(
        package='my_opencv_pkg',
        executable='img_sub',
        name='image_subscriber'
    )

    return LaunchDescription([
        model_arg,
        pub_node,
        sub_node,
        image_publisher,
        image_subscriber
    ])