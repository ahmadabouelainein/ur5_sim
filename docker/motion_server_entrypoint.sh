#!/usr/bin/env bash
#
#  Launch the UR5 Motion Library action server.
#
set -euo pipefail

ROS_DISTRO=${ROS_DISTRO:-noetic}
START_SIM=${START_SIM:-0}
GUI=${GUI:-true}

echo "[motion_server] ROS=$ROS_DISTRO  START_SIM=$START_SIM  GUI=$GUI"

# ─────────────  ROS environment  ────────────────────────────────────────────
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f /ws/devel/setup.bash ] && source /ws/devel/setup.bash

# ─────────────  Ensure ROS master  ──────────────────────────────────────────
if ! rostopic list &> /dev/null; then
  echo "[motion_server] Starting roscore…"
  roscore &
  ROSCORE_PID=$!
  until rostopic list &> /dev/null; do sleep 1; done
else
  ROSCORE_PID=""
fi

# ─────────────  Optional Gazebo + UR5  ──────────────────────────────────────
if [[ "$START_SIM" == "1" ]]; then
  echo "[motion_server] Launching UR5 Gazebo sim…"
  roslaunch ur5_ros_gazebo ur5_gazebo.launch --wait gui:=$GUI &
  SIM_PID=$!
else
  SIM_PID=""
fi

# ─────────────  Motion‑Library action server  ───────────────────────────────
echo "[motion_server] Launching motion_server.launch…"
roslaunch ur5_ros_gazebo motion_server.launch --wait &
SERVER_PID=$!

# ─────────────  Graceful shutdown  ──────────────────────────────────────────
cleanup() {
  echo "[motion_server] Shutting down…"
  kill ${SERVER_PID:-} ${SIM_PID:-} ${ROSCORE_PID:-} 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait $SERVER_PID
