# cam_calibration

Camera calibration tools for the TurtleBot4 using ROS 2 Humble.

This package provides:

* `cam_calibration.capture` — interactive image capture tool (OpenCV GUI).
* `cam_calibration.calibrate` — intrinsic + extrinsic calibration pipeline.
* YAML export for intrinsics/extrinsics.

## Build

```
cd ~/robot_waiter_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select cam_calibration
source install/setup.bash
```

## Capture Images

```
ros2 run cam_calibration capture
```

Keys:

* **s** = save frame
* **q** = quit

Images stored in: `~/calib_images`

## Run Calibration

```
ros2 run cam_calibration calibrate --images ~/calib_images --output camera.yaml
```

Expected Outputs:

* camera.yaml (intrinsic params)
* tf_cam_optical.yaml (extrinsics)

## Definition of Done

* Calibration YAML committed
* Reprojection < 0.5 px
* TF validated in RViz
* Screenshot included in PR
* Docs updated
