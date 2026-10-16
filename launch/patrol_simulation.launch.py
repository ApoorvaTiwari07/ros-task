#!/usr/bin/env python3
"""
patrol_simulation.launch.py

Unified launch file for Autonomous TurtleBot Patrol & Obstacle Avoidance:
1. Launches TurtleBot3 in Gazebo world with obstacles.
2. Starts emergency_stop_server node.
3. Starts lidar_obstacle_detector node.
4. Starts patrol_controller node.
5. Starts telemetry_logger node.
6. Optionally starts RViz2 with visualization markers.

Usage:
    ros2 launch turtlebot_patrol patrol_simulation.launch.py
    ros2 launch turtlebot_patrol patrol_simulation.launch.py rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_turtlebot_patrol = get_package_share_directory('turtlebot_patrol')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model = LaunchConfiguration('model', default='burger')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    rviz_config_arg = LaunchConfiguration('rviz', default='false')

    # Parameters file path
    params_file = os.path.join(
        pkg_turtlebot_patrol,
        'config',
        'patrol_params.yaml'
    )

    # Ensure TURTLEBOT3_MODEL environment variable is burger if not already set
    set_tb3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value=model
    )

    # 1. Include TurtleBot3 Gazebo World (includes 9 obstacle pillars / cylinders)
    tb3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items()
    )

    # 2. Emergency Stop Service Node
    emergency_stop_node = Node(
        package='turtlebot_patrol',
        executable='emergency_stop_server',
        name='emergency_stop_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # 3. LiDAR Obstacle Detector Node
    lidar_detector_node = Node(
        package='turtlebot_patrol',
        executable='lidar_obstacle_detector',
        name='lidar_obstacle_detector',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # 4. Patrol Controller Node
    patrol_controller_node = Node(
        package='turtlebot_patrol',
        executable='patrol_controller',
        name='patrol_controller',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # 5. Telemetry Logger Node
    telemetry_logger_node = Node(
        package='turtlebot_patrol',
        executable='telemetry_logger',
        name='telemetry_logger',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # 6. Optional RViz2 Visualization Node
    rviz_file = os.path.join(pkg_turtlebot_patrol, 'rviz', 'patrol.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz_config_arg)
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'model',
            default_value='burger',
            description='TurtleBot3 model: burger, waffle, or waffle_pi'
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Initial x spawn position in world'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Initial y spawn position in world'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 for visualization'
        ),

        set_tb3_model,
        tb3_gazebo_launch,
        emergency_stop_node,
        lidar_detector_node,
        patrol_controller_node,
        telemetry_logger_node,
        rviz_node,
    ])
