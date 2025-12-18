"""
File: cam_calibration/calibrate.py
Author: Senithu Dampegama
Student Number: 24035891
Description: Batch-calibrate a pinhole camera from checkerboard images and
             export OpenCV YAML intrinsics for the Robot Waiter project.
"""

from pathlib import Path
import glob

import cv2
import numpy as np
import rclpy
from rclpy.node import Node


class CalibrateNode(Node):
    """
    Run mono camera calibration from a set of checkerboard images and
    write an OpenCV-style YAML file with intrinsics + distortion.
    """

    def __init__(self):
        super().__init__('cam_calibrate')

        # Parameters you can tweak later
        self.declare_parameter('images_pattern', 'calib_images/*.png')
        self.declare_parameter('board_rows', 6)        # inner corners
        self.declare_parameter('board_cols', 9)        # inner corners
        self.declare_parameter('square_size', 0.025)   # meters
        self.declare_parameter('output_yaml', 'camera_calibration.yaml')
        self.declare_parameter('camera_frame', 'camera_link')

        self.run()

    def run(self):
        pattern = self.get_parameter('images_pattern').get_parameter_value().string_value
        rows = self.get_parameter('board_rows').value
        cols = self.get_parameter('board_cols').value
        square_size = float(self.get_parameter('square_size').value)
        yaml_path = Path(
            self.get_parameter('output_yaml').get_parameter_value().string_value
        ).expanduser()
        camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

        image_files = sorted(glob.glob(pattern))
        if not image_files:
            self.get_logger().error(f'No images found for pattern: {pattern}')
            return

        self.get_logger().info(f'Using {len(image_files)} images for calibration')

        # Prepare object points for one board
        objp = np.zeros((rows * cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        objp *= square_size

        objpoints = []   # 3D points in the checkerboard coordinate frame
        imgpoints = []   # 2D points in the image plane

        gray_shape = None

        for fname in image_files:
            img = cv2.imread(fname)
            if img is None:
                self.get_logger().warn(f'Failed to read {fname}, skipping')
                continue

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray_shape = gray.shape[::-1]

            ret, corners = cv2.findChessboardCorners(gray, (cols, rows), None)

            if not ret:
                self.get_logger().warn(f'Chessboard NOT found in {fname}, skipping')
                continue

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            objpoints.append(objp)
            imgpoints.append(corners2)

        if len(objpoints) < 5:
            self.get_logger().error(
                f'Only {len(objpoints)} valid boards detected (need >= 5).'
            )
            return

        self.get_logger().info('Running cv2.calibrateCamera...')
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray_shape, None, None
        )

        # RMS reprojection error acts as quick sanity check on calibration quality.
        total_error = 0.0
        total_points = 0

        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(
                objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
            )
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)
            total_error += error ** 2
            total_points += len(objpoints[i])

        rms_error = (total_error / total_points) ** 0.5
        self.get_logger().info(f'RMS reprojection error: {rms_error:.4f} px')

        # --- Write a simple YAML file (no PyYAML dependency) ---
        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        with yaml_path.open('w') as f:
            f.write('%YAML:1.0\n')
            f.write('---\n')
            f.write('camera_name: turtlebot4_oakd\n')
            f.write(f'camera_frame: {camera_frame}\n')
            f.write(f'image_width: {gray_shape[0]}\n')
            f.write(f'image_height: {gray_shape[1]}\n')
            f.write('distortion_model: plumb_bob\n')

            flat_k = camera_matrix.flatten()
            flat_d = dist_coeffs.flatten()

            f.write('camera_matrix:\n')
            f.write('  rows: 3\n  cols: 3\n  data: [' +
                    ', '.join(f'{v:.8f}' for v in flat_k) + ']\n')

            f.write('distortion_coefficients:\n')
            f.write('  rows: 1\n')
            f.write(f'  cols: {len(flat_d)}\n')
            f.write('  data: [' +
                    ', '.join(f'{v:.8f}' for v in flat_d) + ']\n')

        self.get_logger().info(f'Calibration YAML written to: {yaml_path}')

        if rms_error < 0.5:
            self.get_logger().info('✅ Reprojection error < 0.5 px (acceptance criterion met)')
        else:
            self.get_logger().warn('⚠️ Reprojection error >= 0.5 px – consider taking more/better images.')


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateNode()
    node.destroy_node()
    rclpy.shutdown()
