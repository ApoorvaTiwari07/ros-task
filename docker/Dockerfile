# Dockerfile for Autonomous TurtleBot3 Patrol & Obstacle Avoidance
# Built on Ubuntu 22.04 with ROS 2 Humble & Gazebo Simulation

FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger
ENV ROS_DISTRO=humble

# Install Gazebo, TurtleBot3 simulation packages, and development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-teleop \
    ros-humble-turtlebot3-cartographer \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-visualization-msgs \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    nano \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Set up working workspace
ENV ROS_WS=/root/turtlebot_ws
WORKDIR ${ROS_WS}/src

# Copy turtlebot_patrol package into workspace
COPY . ${ROS_WS}/src/turtlebot_patrol

WORKDIR ${ROS_WS}

# Install dependencies and build package
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --packages-select turtlebot_patrol && \
    rm -rf /var/lib/apt/lists/*

# Copy entrypoint script
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "turtlebot_patrol", "patrol_simulation.launch.py"]
