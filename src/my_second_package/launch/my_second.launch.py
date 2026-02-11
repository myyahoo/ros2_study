from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    arg1 = DeclareLaunchArgument(
        'arg1',
        default_value='default_value',
        description='An example launch argument'
    )

    # Define a node to be launched
    my_node = Node(
        package='my_second_package',
        executable='pub_node',
        name='my_pub_node',
        output='screen',
        parameters=[{'arg1': LaunchConfiguration('arg1')}]
    )

    # Create and return the LaunchDescription object
    return LaunchDescription([
        arg1,
        my_node
    ])
