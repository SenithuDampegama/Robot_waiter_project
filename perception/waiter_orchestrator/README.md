# 🍽️ Robot Waiter Project (ROS 2 Humble)

**Author:** Senithu Dampegama  
**Coursework:** Behavioural Robotics / ROS 2 Integration  
**Platform:** TurtleBot4 (Real + Gazebo Ignition)  
**ROS Distro:** Humble Hawksbill (Ubuntu 22.04)

---

## 1. Project Overview

This project implements a **Robot Waiter system** using ROS 2, designed to operate both in **simulation** and on a **real TurtleBot4**.

The system is composed of modular ROS 2 packages covering:

- 📷 Camera calibration
- 👥 Crowd detection (slow-zone safety)
- 🪑 Table localisation (AprilTag-based)
- 🧠 **Central Orchestrator FSM** (task-level behaviour & integration)

The **Orchestrator FSM** coordinates the full waiter workflow:

```
WAITING_FOR_ORDER
→ NAV_TO_KITCHEN
→ WAITINGING_KITCHEN
→ NAV_TO_TABLE
→ DELIVERING
→ RETURNING
→ WAITING_FOR_ORDER
```

All integration is performed via **ROS 2 topics**, allowing subsystems to be mocked or replaced independently.

---

## 2. Workspace Layout

```text
robot_waiter_ws/
├── src/
│   ├── waiter_orchestrator/   # Central FSM (core integration)
│   ├── cam_calibration/       # Camera calibration tools
│   ├── crowd_detector/        # YOLOv8n crowd detection (slow zone)
│   └── table_localizer/       # AprilTag-based table localisation
├── build/
├── install/
└── log/
```

> ⚠️ Always work inside `robot_waiter_ws` (not `tb4_ws`).

---

## 3. Prerequisites

### 3.1 ROS 2

- Ubuntu **22.04**
- ROS 2 **Humble**

```bash
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO   # should print: humble
```

---

### 3.2 System Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-tools \
  ros-humble-rqt-image-view \
  ros-humble-tf2-ros
```

---

### 3.3 Python Dependencies (IMPORTANT)

ROS Humble + `cv_bridge` requires **NumPy 1.x**.

```bash
python3 -m pip uninstall -y numpy
python3 -m pip install "numpy<2"
python3 -m pip install ultralytics opencv-python
```

Verify:

```bash
python3 -c "import numpy; print(numpy.__version__)"
python3 -c "from cv_bridge import CvBridge; print('cv_bridge OK')"
```

---

## 4. Build the Workspace

```bash
cd ~/robot_waiter_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Verify packages:

```bash
ros2 pkg list | grep -E "waiter|crowd|table|calibration"
```

---

## 5. Core Integration — Orchestrator FSM ⭐

This is the **main integration component** of the project.

### 5.1 Launch the Orchestrator

```bash
source /opt/ros/humble/setup.bash
source ~/robot_waiter_ws/install/setup.bash

ros2 launch waiter_orchestrator orchestrator.launch.py
```

Expected output:

```text
[FSM] WAITING_FOR_ORDER System started
```

---

### 5.2 Observe Mission State

In a new terminal:

```bash
ros2 topic echo /mission_state
```

---

### 5.3 Send a Mock Order

```bash
ros2 topic pub -1 /orders std_msgs/msg/String \
  "{data: 'order_id=001;table=A'}"
```

Expected transitions:

```text
WAITING_FOR_ORDER
NAV_TO_KITCHEN
WAITING_KITCHEN
```

---

### 5.4 Simulate Kitchen Ready

```bash
ros2 topic pub -1 /kitchen_status std_msgs/msg/String "{data: 'READY'}"
```

---

### 5.5 Simulate Table Pose

```bash
ros2 topic pub -1 /table_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 1.0, y: 0.2, z: 0.0},
    orientation: {w: 1.0}
  }
}"
```

Expected completion:

```text
NAV_TO_TABLE
DELIVERING
RETURNING
WAITING_FOR_ORDER
```

---

### 5.6 Verify Navigation Goal Output

```bash
ros2 topic echo /goal_pose --once
```

Example output:

```yaml
frame_id: map
position:
  x: 1.0
  y: 0.2
orientation:
  w: 1.0
```

---

## 6. Crowd Detector (Slow Zone)

YOLOv8n-based person detector for safety and speed control.

### 6.1 Local Webcam Test

```bash
ros2 run image_tools cam2image --ros-args -r image:=/image
```

```bash
ros2 run crowd_detector crowd_detector_node \
  --ros-args -p image_topic:=/image -p device:=cpu
```

Monitor:

```bash
ros2 topic echo /crowd_alert
ros2 topic echo /crowd_confidence
```

---

### 6.2 Gazebo Ignition (TurtleBot4)

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
```

```bash
ros2 launch crowd_detector crowd_detector.launch.py \
  image_topic:=/oakd/rgb/preview/image_raw \
  device:=cpu
```

---

## 7. Table Localizer (AprilTag)

### 7.1 Quick Local Test (No Camera Required)

```bash
ros2 run tf2_ros static_transform_publisher \
  1 0 0  0 0 0  map camera_link
```

```bash
ros2 run table_localizer table_localizer_node
```

```bash
ros2 run table_localizer fake_apriltag_detector
```

Monitor:

```bash
ros2 topic echo /table_pose
```

---

## 8. Camera Calibration (Optional / Hardware)

### 8.1 Capture Images

```bash
ros2 run cam_calibration capture
```

Keys:
- `s` → save image
- `q` → quit

---

### 8.2 Run Calibration

```bash
ros2 run cam_calibration calibrate
```

Output:
- `camera_calibration.yaml`
- RMS reprojection error logged

---

## 9. Evidence for Assessment

For marking and reporting:

- FSM terminal showing full state transitions
- `/mission_state` echo output
- `/goal_pose` output
- `rqt_graph` screenshot (optional)
- Video recording of full workflow

This satisfies:
- **Task 1:** State monitoring & behaviour
- **Task 2:** System integration

---

## 10. Mocking & Integration Notes

Some subsystems were mocked to allow **full system-level integration testing** despite incomplete hardware or team dependencies.

This approach is standard in professional robotics workflows.

---

## 11. Conclusion

This project demonstrates a **fully integrated ROS 2 robot system** with:

- Modular architecture
- Robust FSM-based coordination
- Topic-driven integration
- Simulation and hardware readiness

---

📌 *This README enables full reproduction of the system without additional explanation.*
