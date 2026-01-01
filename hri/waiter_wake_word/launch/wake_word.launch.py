from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'wake_word_model',
            default_value='hey_jarvis',
            description='OpenWakeWord model name'
        ),
        DeclareLaunchArgument(
            'threshold',
            default_value='0.55',
            description='Wake word detection threshold'
        ),

        Node(
            package='waiter_wake_word',
            executable='wake_word_node',
            name='wake_word_node',
            output='screen',
            parameters=[{
                'wake_word_model': LaunchConfiguration('wake_word_model'),
                'threshold': LaunchConfiguration('threshold'),
            }],
        ),
    ])
