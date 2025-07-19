#!/usr/bin/env bash
set -e

# Source ROS Noetic environment first (provided by base image)
source "/opt/ros/${ROS_DISTRO:-noetic}/setup.bash"
# If you already ran catkin_make inside /workspace, also source the overlay
[ -f /workspace/ros_ws/devel/setup.bash ] && source /workspace/ros_ws/devel/setup.bash

exec "$@"
