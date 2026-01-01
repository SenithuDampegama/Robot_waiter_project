from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="waiter_dialog_manager",
            executable="dialog_manager",
            name="dialog_manager",
            output="screen",
            parameters=[{
                # topics (match your real system)
                "topics.wake": "/wake_word/activated",
                "topics.stt_text": "/stt/text",
                "topics.order_json": "/order/json",
                "topics.need_clarification": "/order/need_clarification",
                "topics.clarification_question": "/order/clarification_question",
                "topics.orders_out": "/orders",
                "topics.robot_state": "/robot/state",
                "topics.led": "/led/command",
                "topics.oled": "/oled/text",
                # timing
                "stt_timeout_s": 10.0,
                "nlu_timeout_s": 12.0,
                "max_retries": 2,
                "idle_timeout_s": 60.0,
            }],
        ),
    ])
