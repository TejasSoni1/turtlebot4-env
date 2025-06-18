#!/bin/bash
set -e
source /opt/ros/$ROS_DISTRO/setup.bash
if [ -f /root/turtlebot4_ws/install/setup.bash ]; then
  source /root/turtlebot4_ws/install/setup.bash
fi
exec "$@"
