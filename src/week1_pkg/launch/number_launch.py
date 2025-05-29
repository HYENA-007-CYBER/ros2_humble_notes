from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='week1_pkg',
            executable='number_publisher',
            name='number_publisher',
            output='screen'
        ),
        Node(
            package='week1_pkg',
            executable='square_subscriber',
            name='square_subscriber',
            output='screen'
        )
    ])
