import datetime
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CaptureNode(Node):
    """
    Subscribe to the camera image topic and let the user save frames
    for calibration.

    Controls in the OpenCV window:
      - 's' : save current frame
      - 'q' : quit
    """

    def __init__(self):
        super().__init__('cam_capture')

        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('output_dir', 'calib_images')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value

        self.output_dir = Path(output_dir).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self.frame = None
        self.saved_count = 0

        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )

        self.get_logger().info(f'Listening to: {image_topic}')
        self.get_logger().info(f'Saving calibration images to: {self.output_dir}')

        # timer just to drive the OpenCV GUI loop
        self.timer = self.create_timer(0.05, self.gui_step)

    def image_callback(self, msg: Image):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def gui_step(self):
        if self.frame is None:
            return

        img = self.frame.copy()
        cv2.putText(
            img,
            f'Images: {self.saved_count}  (s=save, q=quit)',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )

        cv2.imshow('cam_calibration_capture', img)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            filename = self.output_dir / f'img_{ts}.png'
            cv2.imwrite(str(filename), self.frame)
            self.saved_count += 1
            self.get_logger().info(f'Saved {filename}')

        elif key == ord('q'):
            self.get_logger().info('Quit requested, shutting down.')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()

