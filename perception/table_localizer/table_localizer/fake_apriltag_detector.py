# table_localizer/fake_apriltag_detector.py

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class FakeAprilTagDetector(Node):
    """
    Simple fake detector that publishes a PoseStamped for a tag
    in the camera frame on /tag_pose_cam.

    This bypasses apriltag_msgs and just lets us test the localizer.
    """

    def __init__(self):
        super().__init__('fake_apriltag_detector')

        self.declare_parameter('topic', '/tag_pose_cam')
        self.declare_parameter('camera_frame', 'camera_link')

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

        self.pub = self.create_publisher(PoseStamped, self.topic, 10)
        self.timer = self.create_timer(0.2, self.timer_callback)  # 5 Hz

        self.t = 0.0
        self.get_logger().info(
            f"FakeAprilTagDetector started, publishing PoseStamped on {self.topic} "
            f"with frame_id={self.camera_frame}"
        )

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame

        # simple motion so we can see changes
        msg.pose.position.x = 1.0 + 0.3 * math.sin(self.t)
        msg.pose.position.y = 0.1 * math.cos(self.t)
        msg.pose.position.z = 0.8

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.pub.publish(msg)
        self.t += 0.2


def main(args=None):
    rclpy.init(args=args)
    node = FakeAprilTagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
