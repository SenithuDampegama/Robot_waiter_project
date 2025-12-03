import time
from threading import Lock

import cv2
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge

from rclpy.qos import qos_profile_sensor_data


class CrowdDetectorNode(Node):
    """Crowd detector node for Robot Waiter.

    - Subscribes to a camera image topic.
    - Runs YOLOv8n person detection on each frame.
    - Publishes:
        /crowd_alert      (std_msgs/Bool)
        /crowd_confidence (std_msgs/Float32)

    A "person" is class id 0 in the COCO label set.
    """

    def __init__(self) -> None:
        super().__init__('crowd_detector_node')

        # ---------------------
        # Parameters
        # ---------------------
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('crowd_alert_topic', '/crowd_alert')
        self.declare_parameter('crowd_conf_topic', '/crowd_confidence')
        self.declare_parameter('person_class_id', 0)  # COCO: 0 = person
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('device', 'cuda')      # 'cuda' or 'cpu'

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        crowd_alert_topic = self.get_parameter('crowd_alert_topic').get_parameter_value().string_value
        crowd_conf_topic = self.get_parameter('crowd_conf_topic').get_parameter_value().string_value

        self.person_class_id = int(self.get_parameter('person_class_id').value)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.device = self.get_parameter('device').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.fps_lock = Lock()
        self.fps_ema: float | None = None

        # ---------------------
        # Load YOLOv8n model
        # ---------------------
        self.get_logger().info('Loading YOLOv8n… (first run may download weights)')
        t0 = time.time()
        self.model = YOLO('yolov8n.pt')
        self.model.to(self.device)
        self.get_logger().info(f'Model loaded in {time.time() - t0:.2f}s on {self.device}')

        # ---------------------
        # Publishers
        # ---------------------
        self.pub_alert = self.create_publisher(Bool, crowd_alert_topic, 10)
        self.pub_conf = self.create_publisher(Float32, crowd_conf_topic, 10)

        # ---------------------
        # Subscriber (sensor QoS for OAK-D)
        # ---------------------
        self.sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f'Subscribed to: {image_topic}')
        self.get_logger().info(f'Publishing Bool on: {crowd_alert_topic}')
        self.get_logger().info(f'Publishing Float32 on: {crowd_conf_topic}')

    # ------------------------------------------------------------------
    # Image callback
    # ------------------------------------------------------------------
    def image_callback(self, msg: Image) -> None:
        """Handle incoming images and run YOLO detection."""
        # Convert ROS Image -> OpenCV BGR8
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'cv_bridge error: {exc}')
            return

        start = time.time()

        # Run YOLO inference
        try:
            results = self.model(
                frame,
                verbose=False,
                device=self.device,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'YOLO inference error: {exc}')
            return

        detections = results[0]
        boxes = detections.boxes

        person_found = False
        best_conf = 0.0

        # Process detections if present
        if boxes is not None and len(boxes) > 0:
            cls = boxes.cls.detach().cpu().numpy().astype(int)
            conf = boxes.conf.detach().cpu().numpy()

            for c, s in zip(cls, conf):
                if c == self.person_class_id and s >= self.conf_threshold:
                    person_found = True
                    score = float(s)
                    if score > best_conf:
                        best_conf = score

        # ---------------------
        # Publish outputs
        # ---------------------
        alert_msg = Bool()
        alert_msg.data = person_found
        self.pub_alert.publish(alert_msg)

        conf_msg = Float32()
        conf_msg.data = best_conf if person_found else 0.0
        self.pub_conf.publish(conf_msg)

        # ---------------------
        # FPS debug (EMA)
        # ---------------------
        dt = time.time() - start
        fps = 1.0 / dt if dt > 0.0 else 0.0

        with self.fps_lock:
            if self.fps_ema is None:
                self.fps_ema = fps
            else:
                alpha = 0.1
                self.fps_ema = alpha * fps + (1.0 - alpha) * self.fps_ema

        # Log at ~1 Hz
        if int(time.time()) % 1 == 0:
            self.get_logger().info(
                f'[crowd_detector] person={person_found}, '
                f'conf={best_conf:.2f}, fps={self.fps_ema:.1f}',
            )


# ----------------------------------------------------------------------
# Main entry point
# ----------------------------------------------------------------------

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CrowdDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
