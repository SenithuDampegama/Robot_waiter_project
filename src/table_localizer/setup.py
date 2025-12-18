"""
File: table_localizer/setup.py
Author: Senithu Dampegama
Student Number: 24035891
Description: ament_python setup script for the table_localizer package.
"""

from setuptools import setup

package_name = 'table_localizer'

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
    maintainer='senithu',
    maintainer_email='senithu@todo.todo',
    description='AprilTag-based table localization node.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'table_localizer_node = table_localizer.table_localizer_node:main',
            'fake_apriltag_detector = table_localizer.fake_apriltag_detector:main',
        ],
    },
)
