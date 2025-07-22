#!/usr/bin/env bash
set -euo pipefail

# ------------------ env ---------------------------------------------------
ROS_DISTRO=${ROS_DISTRO:-noetic}
LLM_BACKEND=${LLM_BACKEND:-ollama}
LLM_MODEL=${LLM_MODEL:-llama3.8b-instruct}
START_SIM=${START_SIM:-0}
GUI=${GUI:-true}
OLLAMA_HOST=${OLLAMA_HOST:-http://ollama:11434}

echo "[copilot]  ROS=$ROS_DISTRO  BACKEND=$LLM_BACKEND  MODEL=$LLM_MODEL"

# ------------------ ROS env ------------------------------------------------
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f /ws/devel/setup.bash ] && source /ws/devel/setup.bash

# ------------------ roscore -----------------------------------------------
if ! rostopic list > /dev/null 2>&1; then
  echo "[copilot]  starting roscore…"
  roscore &
  ROSCORE_PID=$!
  until rostopic list > /dev/null 2>&1; do sleep 1; done
fi

# ------------------ optional sim & motion server --------------------------
if [ "$START_SIM" = "1" ]; then
  roslaunch ur5_ros_gazebo ur5_gazebo.launch --wait gui:=$GUI &
  SIM_PID=$!
  roslaunch ur5_ros_gazebo motion_server.launch --wait &
  MOTION_PID=$!
fi

# ------------------ wait for Ollama ---------------------------------------
if [ "$LLM_BACKEND" = "ollama" ]; then
  echo "[copilot] waiting for Ollama at $OLLAMA_HOST …"
  for i in {1..60}; do
    if curl -fsS "$OLLAMA_HOST/api/tags" > /dev/null 2>&1; then
      echo "[copilot] Ollama is up."; break; fi
    sleep 2
  done
fi

# ------------------ cleanup -----------------------------------------------
cleanup() {
  kill ${MOTION_PID:-} 2>/dev/null || true
  kill ${SIM_PID:-}    2>/dev/null || true
  kill ${ROSCORE_PID:-} 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ------------------ run REPL ----------------------------------------------
exec python3 /ur5_sim/ws/src/ur5_ros_gazebo/scripts/llm_copilot.py \
     --backend "$LLM_BACKEND" --model "$LLM_MODEL"
