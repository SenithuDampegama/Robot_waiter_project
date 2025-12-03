# Crowd Detector (Slow Zone) – Robot Waiter Project

YOLOv8n-based person detection node for the TurtleBot4 "Robot Waiter" platform. It watches the RGB camera stream and signals when the robot should slow down in crowded areas.

## Features
- Uses Ultralytics YOLOv8n (PyTorch) for person detection (COCO person class ID 0).
- Subscribes to a camera topic (default `/oakd/rgb/preview/image_raw`).
- Publishes:
  - `/crowd_alert` (`std_msgs/Bool`) – `True` whenever a person exceeds the confidence threshold.
  - `/crowd_confidence` (`std_msgs/Float32`) – highest detected person confidence, or `0.0` if none.
- Tunable parameters: `image_topic`, `conf_threshold`, `device`, `person_class_id`.
- Designed to run on Jetson Orin Nano and discrete GPU laptops.

## ROS 2 Interfaces
- **Node name:** `crowd_detector_node`
- **Subscriptions:** `sensor_msgs/Image` on `image_topic` (default `/oakd/rgb/preview/image_raw`).
- **Publications:**
  - `/crowd_alert` – `std_msgs/Bool`
  - `/crowd_confidence` – `std_msgs/Float32`
- **Parameters:**
  - `image_topic` (string, default `/oakd/rgb/preview/image_raw`)
  - `crowd_alert_topic` (string, default `/crowd_alert`)
  - `crowd_conf_topic` (string, default `/crowd_confidence`)
  - `person_class_id` (int, default `0`)
  - `conf_threshold` (float, default `0.5`)
  - `device` (string, `cuda` or `cpu`, default `cuda`)

## Dependencies / Installation
This is an `ament_python` package.

Python & ROS dependencies:
- `ultralytics`
- `opencv-python`
- `numpy==1.26.4` (ROS Humble + `cv_bridge` requires NumPy 1.x)
- `cv_bridge` (`ros-humble-cv-bridge`)
- `rclpy`, `sensor_msgs`, `std_msgs`

Example installs:
```bash
sudo apt install ros-humble-cv-bridge ros-humble-image-tools
pip install ultralytics opencv-python numpy==1.26.4
```

## Build & Run (robot_waiter_ws)
```bash
cd ~/robot_waiter_ws
colcon build --packages-select crowd_detector
source install/setup.bash
```

Launch with the default camera:
```bash
ros2 launch crowd_detector crowd_detector.launch.py
```

Override the image topic:
```bash
ros2 launch crowd_detector crowd_detector.launch.py image_topic:=/camera/image_raw
```

Monitor the outputs:
```bash
ros2 topic echo /crowd_alert
ros2 topic echo /crowd_confidence
```

## Performance Notes
- Tested at ~80–100 FPS on an RTX 4090 laptop (simulated images).
- Acceptance criteria: ≥10 FPS on GPU/CPU, detects humans within 1–3 m, continuously publishes `/crowd_alert` when people are visible.

## Integration Hint
Use `/crowd_alert` to switch the orchestrator/Nav2 stack into a slow-mode profile: when `True`, clamp maximum velocity or select a "slow zone" behavior tree branch; when `False`, revert to normal cruising speed.

## Testing
- Webcam: `ros2 run image_tools cam2image` and launch this node with `image_topic:=/image`.
- TurtleBot4 simulation: launch the TB4 Ignition bringup, point the simulated camera at a human model, and set `image_topic` to the simulator's camera topic when launching the detector.
