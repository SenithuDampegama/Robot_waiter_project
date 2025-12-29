# Crowd Detector (Slow Zone) – Robot Waiter Project

YOLOv8n-based person detection node for the TurtleBot4 **Robot Waiter** platform. This document is a **fully reproducible, step‑by‑step guide** covering environment setup, build, debugging pitfalls, webcam testing, and **Gazebo Ignition (TB4) simulation**.

---

## What this node does
- Subscribes to an RGB camera topic (`sensor_msgs/Image`).
- Runs **YOLOv8n** and checks for **COCO person class (ID 0)**.
- Publishes:
  - **`/crowd_alert`** (`std_msgs/Bool`) → `True` when a person is detected above threshold.
  - **`/crowd_confidence`** (`std_msgs/Float32`) → highest confidence (or `0.0`).
- Intended use: **slow‑zone control** in crowded areas (feeds Nav2 / orchestrator logic).

---

## Workspace layout (important)
This guide assumes:
```text
~/robot_waiter_ws/
├── src/
│   └── crowd_detector/
├── build/
├── install/
└── log/
```

Always work inside **`robot_waiter_ws`** (not `tb4_ws`).

---

## Prerequisites

### ROS 2
- **ROS 2 Humble** (Ubuntu 22.04)

```bash
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO   # should print: humble
```

### System dependencies
```bash
sudo apt update
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-tools \
  ros-humble-rqt-image-view
```

### Python dependencies (CRITICAL)
ROS Humble + `cv_bridge` **requires NumPy 1.x**.

```bash
python3 -m pip uninstall -y numpy
python3 -m pip install "numpy<2"
python3 -m pip install ultralytics opencv-python
```

Verify:
```bash
python3 -c "import numpy; print(numpy.__version__)"   # 1.x
python3 -c "from cv_bridge import CvBridge; print('cv_bridge OK')"
```

---

## Build the package

```bash
cd ~/robot_waiter_ws
colcon build --packages-select crowd_detector
source install/setup.bash
```

Verify the executable:
```bash
ros2 pkg executables crowd_detector
# Expected: crowd_detector  crowd_detector_node
```

---

## Launch file (supports overrides)
The launch file **accepts parameters** (fixed during debugging):

```bash
ros2 launch crowd_detector crowd_detector.launch.py \
  image_topic:=/oakd/rgb/preview/image_raw \
  device:=cpu
```

Parameters:
- `image_topic` (string)
- `crowd_alert_topic` (string)
- `crowd_conf_topic` (string)
- `conf_threshold` (float)
- `device` (`cpu` or `cuda`)

---

## Sanity test (webcam – fastest)

1. Start a camera publisher:
```bash
ros2 run image_tools cam2image --ros-args -r image:=/image
```

2. Run the detector:
```bash
ros2 run crowd_detector crowd_detector_node \
  --ros-args -p image_topic:=/image -p device:=cpu
```

3. Observe output:
```bash
ros2 topic echo /crowd_alert
ros2 topic echo /crowd_confidence
```

You should see:
```text
data: true
```
when a person is visible.

---

## Gazebo Ignition (TurtleBot4) test – reproducible

### 1. Launch TB4 Ignition
```bash
source /opt/ros/humble/setup.bash
source ~/robot_waiter_ws/install/setup.bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
```

### 2. Find the simulated camera topic
```bash
ros2 topic list | grep -Ei "image|camera"
```
Typical result:
```text
/oakd/rgb/preview/image_raw
```

Verify frames are flowing:
```bash
ros2 topic hz /oakd/rgb/preview/image_raw
```

(Optional visual check):
```bash
ros2 run rqt_image_view rqt_image_view
```

### 3. Run the crowd detector on the sim camera
```bash
ros2 launch crowd_detector crowd_detector.launch.py \
  image_topic:=/oakd/rgb/preview/image_raw \
  device:=cpu
```

### 4. Monitor outputs
```bash
ros2 topic echo /crowd_alert
ros2 topic echo /crowd_confidence
```

### 5. Trigger detection
- Insert a **human / actor** model in Gazebo Ignition **in front of the robot camera**.
- When visible, `/crowd_alert` becomes `True`.

---

## Expected log output (normal)
```text
[crowd_detector] person=True, conf=0.89, fps=38.7
```

Meaning:
- `person=True` → human detected
- `conf=0.89` → confidence
- `fps=38.7` → real‑time inference speed

---

## Common pitfalls (and fixes)

### ❌ `/crowd_alert` shows nothing
- Cause: no images arriving on `image_topic`.
- Fix: verify publisher exists:
```bash
ros2 topic info <image_topic>
```

### ❌ NumPy / cv_bridge crash
Error:
```text
AttributeError: _ARRAY_API not found
```
Fix:
```bash
pip uninstall numpy
pip install "numpy<2"
```

### ❌ `image_topic:=...` ignored
- Cause: hard‑coded params in launch file.
- Fix: use the updated launch file with `DeclareLaunchArgument`.

---

## Safety system note (TB4)
Warnings like:
```text
Ignoring velocities commanded while an autonomous behavior is running
Reflex exceeded runtime without clearing hazard
```

are **normal**. TurtleBot4 safety and autonomy layers correctly override motion. The crowd detector **does not bypass safety**.

---

## Integration with Nav2 / Robot Waiter
Use `/crowd_alert` as a **behavioral input**:
- `True` → reduce max velocity / enter slow‑zone BT branch
- `False` → normal cruising speed

---

## Reproducibility checklist
- ROS 2 Humble sourced
- NumPy **1.x** installed
- `cv_bridge` imports OK
- Camera topic publishing
- Launch file rebuilt
- `/crowd_alert` toggles when a person appears

---

**Author:** Senithu Dampegama  
**Project:** Robot Waiter – Crowd‑Aware Navigation

