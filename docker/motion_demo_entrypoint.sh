#!/usr/bin/env bash
#
#  Simple client that sends one joint move and one linear move.
#
set -euo pipefail

ROS_DISTRO=${ROS_DISTRO:-noetic}

source "/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f /ur5_sim/ws/devel/setup.bash ] && source /ur5_sim/ws/devel/setup.bash

echo "[motion_demo] Waiting for /move_joint action server…"
until rosnode list 2>/dev/null | grep -q motion_action_server; do
  sleep 1
done

echo "[motion_demo] Servers are up – running demo node."
exec rosrun ur5_ros_gazebo motion_demo_node
