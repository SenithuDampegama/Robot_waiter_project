from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'waiter_orchestrator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='senithu',
    maintainer_email='senexdamx@gmail.com',
    description='High-level FSM orchestrator for TurtleBot4 waiter robot',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'orchestrator = waiter_orchestrator.orchestrator_node:main',
        ],
    },
)
