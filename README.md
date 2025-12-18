<!--
File: README.md
Author: Senithu Dampegama
Student Number: 24035891
Description: Workspace-level documentation for Robot Waiter ROS 2 stack.
-->

# robot_waiter_ws – ROS 2 Workspace

Contains all ROS2 packages for the Robot Waiter group project.

Current packages:

- `cam_calibration` – capture/calibrate TurtleBot4 camera intrinsics.
- `table_localizer` – AprilTag-based table pose estimation.
- `crowd_detector` – YOLOv8 slow-zone detector for people.
- (future) `orchestrator_fsm`

## Build Workspace

```
cd robot_waiter_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Workspace Layout

```
robot_waiter_ws/
├── src/
│   ├── cam_calibration/
│   ├── table_localizer/
│   └── crowd_detector/
├── build/      (ignored)
├── install/    (ignored)
└── log/        (ignored)
```

## Git Rules

Do NOT commit:

* build/
* install/
* log/
* .colcon*

All source code must stay inside `src/`.
