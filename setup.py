import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtlebot_patrol'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Apoorva Tiwari',
    maintainer_email='apoorva@todo.todo',
    description='Autonomous TurtleBot3 patrol, obstacle avoidance, emergency stop, and telemetry system',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'lidar_obstacle_detector = turtlebot_patrol.lidar_obstacle_detector:main',
            'patrol_controller = turtlebot_patrol.patrol_controller:main',
            'emergency_stop_server = turtlebot_patrol.emergency_stop_server:main',
            'telemetry_logger = turtlebot_patrol.telemetry_logger:main',
        ],
    },
)
