from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("waiter_orchestrator"),
        "config",
        "orchestrator.yaml"
    ])

    return LaunchDescription([
        Node(
            package="waiter_orchestrator",
            executable="orchestrator",   # console_scripts name from setup.py
            name="waiter_orchestrator",
            output="screen",
            parameters=[config_file],
        )
    ])
