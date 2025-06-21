from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare("my_robot_description")

    xacro_path = PathJoinSubstitution([pkg_share, "urdf", "four_wheel_bot.xacro"])
    world_path = PathJoinSubstitution([pkg_share, "worlds", "my_world.world"])

    robot_description = Command(["xacro ", xacro_path])

    return LaunchDescription([
        # Launch Gazebo with your custom world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py"
                ])
            ]),
            launch_arguments={"world": world_path}.items()
        ),

        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}]
        ),

        # Spawn Robot in Gazebo
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-topic", "robot_description",
                "-entity", "four_wheel_bot"
            ],
            output="screen"
        ),

        # Run Obstacle Stop Node
        Node(
            package="my_robot_description",
            executable="obstacle_stop",
            output="screen"
        )
    ])

