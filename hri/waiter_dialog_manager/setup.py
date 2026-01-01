from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'waiter_dialog_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament package index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # package manifest
        ('share/' + package_name, ['package.xml']),

        # install launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),

        # install config files (optional; safe even if empty)
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='senithu',
    maintainer_email='senexdamx@gmail.com',
    description='Dialog manager FSM for Robot Waiter speech interaction flow',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dialog_manager = waiter_dialog_manager.dialog_manager_node:main',
        ],
    },
)
