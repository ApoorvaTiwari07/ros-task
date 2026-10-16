#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace if built
if [ -f "/root/turtlebot_ws/install/setup.bash" ]; then
    source /root/turtlebot_ws/install/setup.bash
fi

export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}

exec "$@"
