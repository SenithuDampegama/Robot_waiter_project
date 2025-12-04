"""
File: cam_calibration/setup.py
Author: Senithu Dampegama
Student Number: 24035891
Description: Package setup script for the cam_calibration ROS 2 ament_python package.
"""

from setuptools import setup

package_name = 'cam_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Senithu',
    maintainer_email='senithu@todo.todo',
    description='Camera calibration tools for TurtleBot4 (intrinsic + extrinsic).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'capture = cam_calibration.capture:main',
            'calibrate = cam_calibration.calibrate:main',
        ],
    },
)
