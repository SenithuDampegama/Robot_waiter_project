<!--
File: README.md
Author: Senithu Dampegama
Student Number: 24035891
Description: Documentation for the table_localizer ROS 2 package.
-->

# table_localizer

AprilTag-based table localization package for the Robot Waiter project.

This ROS 2 (Humble) package consumes AprilTag detections in the **camera frame** and publishes a smoothed, stable estimate of the table pose in the **map frame**.

## Nodes

### `table_localizer_node`

**Type:** `rclpy` node

**Subscriptions**
- `/tag_pose_cam` (`geometry_msgs/PoseStamped`)
  - Pose of the AprilTag in the `camera_link` frame.

**TF required**
- `map` → `camera_link` (can be provided by Nav2 / robot state publisher / static transform).

**Publications**
- `/table_pose` (`geometry_msgs/PoseStamped`)
  - Smoothed pose of the table in the `map` frame.

The node applies a simple exponential moving average on both position and orientation to reduce noise from the raw detections.

### `fake_apriltag_detector`

Helper node used for development and testing.

**Publications**
- `/tag_pose_cam` (`geometry_msgs/PoseStamped`)
  - Simulated AprilTag pose in the `camera_link` frame. This lets us test the localizer without a real camera or detector.

## Quick start (local test)

In three separate terminals:

```bash
# Terminal 1 – static TF from map to camera_link
ros2 run tf2_ros static_transform_publisher \
    1.0 0.0 0.0  0.0 0.0 0.0  map camera_link
```

```bash
# Terminal 2 – table localizer
ros2 run table_localizer table_localizer_node
```

```bash
# Terminal 3 – fake detector
ros2 run table_localizer fake_apriltag_detector
```

Then, in a fourth terminal:

```bash
ros2 topic echo /table_pose
```

You should see a smoothly changing `PoseStamped` in the `map` frame.

## Integration notes

* In the real robot, replace `fake_apriltag_detector` with a real AprilTag detector that publishes `PoseStamped` on `/tag_pose_cam` in the `camera_link` frame.
* The package is designed to live under the `perception/` directory of the `Robot_waiter_project` repository.
