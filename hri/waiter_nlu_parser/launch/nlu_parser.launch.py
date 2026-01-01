from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="waiter_nlu_parser",
            executable="nlu_node",
            name="waiter_nlu_parser",
            output="screen",
            parameters=[{
                "stt_topic": "/stt/text",
                "model": "gpt-5-mini",
                "api_key_env": "OPENAI_API_KEY",
                "menu_json_path": "",
            }]
        )
    ])
