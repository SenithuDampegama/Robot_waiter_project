<!--
File: README.md
Author: Senithu Dampegama
Student Number: 24035891
Description: Documentation for the cam_calibration package.
-->

# cam_calibration

Camera calibration tools for the TurtleBot4 using ROS 2 Humble.

This package provides:

- `cam_calibration.capture` — interactive image capture tool (OpenCV GUI). Saves frames with timestamps so calibration datasets remain organized.
- `cam_calibration.calibrate` — intrinsic calibration pipeline with configurable board geometry.
- YAML export for intrinsics/extrinsics compatible with `camera_info_manager`.

## Build

```
cd ~/robot_waiter_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select cam_calibration
source install/setup.bash
```

## Capture Images

```bash
ros2 run cam_calibration capture
```

Keys:

* **s** = save frame
* **q** = quit

Images stored in: `~/calib_images`

## Run Calibration

```bash
ros2 run cam_calibration calibrate --images ~/calib_images --output camera.yaml
```

Expected Outputs:

* camera.yaml (intrinsic params)
* tf_cam_optical.yaml (extrinsics)

## Definition of Done

- Calibration YAML committed.
- Reprojection < 0.5 px (logged after calibration).
- TF validated in RViz with reprojection overlay.
- Screenshot included in PR to document results.
- Docs updated to reflect any parameter tweaks.
