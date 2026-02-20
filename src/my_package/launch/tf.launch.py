

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    tf_combined = Node(
        package='my_package',
        executable='tf_combined',
        name='tf_combined_node',
        output='screen',    
    )

    if_distance = Node(
        package='my_package',
        executable='tf_distance_pub',
        name='tf_distance_pub_node',
        output='screen',    
    )

    return LaunchDescription([
        tf_combined,
        if_distance
    ])