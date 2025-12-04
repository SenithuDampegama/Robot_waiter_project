"""
File: launch/crowd_detector.launch.py
Author: Senithu Dampegama
Student Number: 24035891
Description: Launch description for spinning up the CrowdDetectorNode with tunable params.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Build the ROS 2 LaunchDescription for the YOLOv8 slow-zone detector."""
    return LaunchDescription([
        Node(
            package='crowd_detector',
            executable='crowd_detector_node',
            name='crowd_detector',
            output='screen',
            parameters=[
                {
                    'image_topic': '/oakd/rgb/preview/image_raw',
                    'crowd_alert_topic': '/crowd_alert',
                    'crowd_conf_topic': '/crowd_confidence',
                    'conf_threshold': 0.5,
                    'device': 'cuda',   # use 'cpu' on laptop if no GPU
                }
            ]
        )
    ])
