import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_turtlebot_patrol = get_package_share_directory('turtlebot_patrol')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Default parameters file
    default_params_file = os.path.join(
        pkg_turtlebot_patrol, 'config', 'patrol_params.yaml'
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', default=default_params_file)
    gui = LaunchConfiguration('gui', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    # Gazebo World file
    world = os.path.join(
        pkg_turtlebot3_gazebo, 'worlds', 'turtlebot3_world.world'
    )

    # Ensure TURTLEBOT3_MODEL is burger
    set_tb3_model_cmd = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL', value='burger'
    )

    # Gazebo Server (simulation core)
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Gazebo Client (GUI)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui)
    )

    # Robot State Publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawn TurtleBot3 Burger in Gazebo
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # 1. Perception Node
    lidar_node = Node(
        package='turtlebot_patrol',
        executable='lidar_obstacle_detector',
        name='lidar_obstacle_detector',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # 2. Control Node
    controller_node = Node(
        package='turtlebot_patrol',
        executable='patrol_controller',
        name='patrol_controller',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # 3. Emergency Stop Server
    estop_node = Node(
        package='turtlebot_patrol',
        executable='emergency_stop_server',
        name='emergency_stop_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # 4. Telemetry Logger
    telemetry_node = Node(
        package='turtlebot_patrol',
        executable='telemetry_logger',
        name='telemetry_logger',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(set_tb3_model_cmd)
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true',
                                        description='Use simulation (Gazebo) clock'))
    ld.add_action(DeclareLaunchArgument('params_file', default_value=default_params_file,
                                        description='Path to ROS 2 parameters YAML'))
    ld.add_action(DeclareLaunchArgument('gui', default_value='true',
                                        description='Set to false to run headless'))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='-2.0',
                                        description='Initial x spawn coordinate'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='-0.5',
                                        description='Initial y spawn coordinate'))

    # Add simulation actions
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    # Add software stack nodes
    ld.add_action(lidar_node)
    ld.add_action(controller_node)
    ld.add_action(estop_node)
    ld.add_action(telemetry_node)

    return ld
