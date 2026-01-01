from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waiter_speech_to_text',
            executable='speech_to_text_node',
            name='speech_to_text',
            output='screen',
            parameters=[]
        )
    ])
