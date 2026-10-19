import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_turtlebot_patrol = get_package_share_directory('turtlebot_patrol')
    default_params_file = os.path.join(
        pkg_turtlebot_patrol, 'config', 'patrol_params.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', default=default_params_file)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('params_file', default_value=default_params_file,
                              description='Path to ROS 2 parameters YAML'),
        Node(
            package='turtlebot_patrol',
            executable='lidar_obstacle_detector',
            name='lidar_obstacle_detector',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='turtlebot_patrol',
            executable='patrol_controller',
            name='patrol_controller',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='turtlebot_patrol',
            executable='emergency_stop_server',
            name='emergency_stop_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='turtlebot_patrol',
            executable='telemetry_logger',
            name='telemetry_logger',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),
    ])
