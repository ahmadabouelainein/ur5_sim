#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
WS=/ur5_sim/ws
echo "[entrypoint] catkin_make incremental build…"
cd $WS && catkin_make
echo "sourcing workspace ..."

source $WS/devel/setup.bash     # overlay for this shell
rospack profile >/dev/null 2>&1 || true
roslaunch ur5_ros_gazebo ur5_motion_demo.launch
# exec "$@"                       # hand off to CMD (bash)
