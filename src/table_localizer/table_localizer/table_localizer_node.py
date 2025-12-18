"""
File: table_localizer/table_localizer_node.py
Author: Senithu Dampegama
Student Number: 24035891
Description: Converts noisy AprilTag detections into a smooth table pose
             estimate in the map frame for the Robot Waiter project.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose


class TableLocalizerNode(Node):
    """
    AprilTag-based table localizer.

    Input:
      - PoseStamped on /tag_pose_cam, in camera frame.

    TF:
      - map -> ... -> camera_link must exist.

    Output:
      - PoseStamped on /table_pose in map frame, optionally smoothed.
    """

    def __init__(self):
        super().__init__('table_localizer')

        # Parameters
        self.declare_parameter('input_topic', '/tag_pose_cam')
        self.declare_parameter('output_topic', '/table_pose')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('debug_log_detections', True)
        self.declare_parameter('smoothing_alpha', 0.2)  # 0 = no update, 1 = no smoothing

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.debug_log = self.get_parameter('debug_log_detections').get_parameter_value().bool_value
        self.alpha = float(self.get_parameter('smoothing_alpha').get_parameter_value().double_value)

        # TF buffer + listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Sub + pub
        self.sub = self.create_subscription(
            PoseStamped,
            input_topic,
            self.tag_pose_callback,
            10,
        )
        self.pub = self.create_publisher(PoseStamped, output_topic, 10)

        # Internal state for smoothing
        self.filtered_pose: Pose | None = None

        self.get_logger().info(
            f"Table Localizer started.\n"
            f"  input_topic     = {input_topic}\n"
            f"  output_topic    = {output_topic}\n"
            f"  camera_frame    = {self.camera_frame}\n"
            f"  target_frame    = {self.target_frame}\n"
            f"  smoothing_alpha = {self.alpha}"
        )

    # ------------ small helpers ------------

    @staticmethod
    def _lerp(a: float, b: float, t: float) -> float:
        return a + t * (b - a)

    def _smooth_pose(self, new_pose: Pose) -> Pose:
        """Exponential moving average on position + quaternion (approx)."""
        if self.filtered_pose is None:
            # First sample, just take it
            self.filtered_pose = Pose()
            self.filtered_pose.position = new_pose.position
            self.filtered_pose.orientation = new_pose.orientation
            return self.filtered_pose

        a = self.alpha
        fp = self.filtered_pose
        np = new_pose

        # Position EMA
        fp.position.x = self._lerp(fp.position.x, np.position.x, a)
        fp.position.y = self._lerp(fp.position.y, np.position.y, a)
        fp.position.z = self._lerp(fp.position.z, np.position.z, a)

        # Quaternion "lerp then normalise" (good enough for small steps)
        q1 = fp.orientation
        q2 = np.orientation

        # Avoid long-way-around if quaternions are opposite
        dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w
        if dot < 0.0:
            q2 = Quaternion(
                x=-q2.x,
                y=-q2.y,
                z=-q2.z,
                w=-q2.w,
            )

        q = Quaternion()
        q.x = self._lerp(q1.x, q2.x, a)
        q.y = self._lerp(q1.y, q2.y, a)
        q.z = self._lerp(q1.z, q2.z, a)
        q.w = self._lerp(q1.w, q2.w, a)

        # Normalise
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm > 1e-6:
            q.x /= norm
            q.y /= norm
            q.z /= norm
            q.w /= norm

        fp.orientation = q

        return fp

    # ------------ callback ------------

    def tag_pose_callback(self, msg: PoseStamped):
        # Ensure frame_id is set
        if not msg.header.frame_id:
            msg.header.frame_id = self.camera_frame

        # Lookup transform target_frame <- source_frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,          # target
                msg.header.frame_id,        # source
                Time(),                     # time = latest
                timeout=Duration(seconds=0.1)
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"TF lookup failed {msg.header.frame_id} -> {self.target_frame}: {ex}"
            )
            return

        # Transform the *pose* into target frame
        pose_map: Pose = do_transform_pose(msg.pose, tf)

        # Apply smoothing
        smoothed = self._smooth_pose(pose_map)

        # Wrap back into PoseStamped in target frame
        out = PoseStamped()
        out.header.stamp = tf.header.stamp
        out.header.frame_id = self.target_frame
        out.pose = smoothed

        self.pub.publish(out)

        if self.debug_log:
            p = out.pose.position
            self.get_logger().info(
                f"Table pose in {self.target_frame}: "
                f"({p.x:.2f}, {p.y:.2f}, {p.z:.2f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TableLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
