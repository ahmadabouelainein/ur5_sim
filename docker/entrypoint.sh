#!/usr/bin/env bash
set -e

# Source ROS Noetic environment first (provided by base image)
source "/opt/ros/noetic/setup.bash"
# If you already ran catkin_make inside /wss, also source the overlay
[ -f /ws/devel/setup.bash ] && source /ws/devel/setup.bash

exec "$@"
