#!/usr/bin/env bash
set -euo pipefail

# ───────────────────── env ------------------------------------------------
ROS_DISTRO=${ROS_DISTRO:-noetic}
LLM_BACKEND=${LLM_BACKEND:-ollama}
LLM_MODEL=${LLM_MODEL:-phi4-reasoning}
START_SIM=${START_SIM:-0}
GUI=${GUI:-true}
OLLAMA_HOST=${OLLAMA_HOST:-http://ollama:11434}

# ---- give ROS a sane default master/IP (needed for set -u) --------------
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
export ROS_IP=${ROS_IP:-127.0.0.1}

echo "[llm] ROS=$ROS_DISTRO  BACKEND=$LLM_BACKEND  MODEL=$LLM_MODEL"

# ───────────────────── ROS env -------------------------------------------
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f /ur5_sim/ws/devel/setup.bash ] && source /ur5_sim/ws/devel/setup.bash

# ───────────────────── roscore -------------------------------------------
if ! rostopic list >/dev/null 2>&1; then
  echo "[llm-$LLM_MODEL] starting roscore…"
  roscore &
  ROSCORE_PID=$!
  until rostopic list >/dev/null 2>&1; do sleep 1; done
fi

# ───────────────────── optional Gazebo sim -------------------------------
if [ "$START_SIM" = "1" ]; then
  roslaunch ur5_ros_gazebo ur5_gazebo.launch  --wait gui:=$GUI &
  SIM_PID=$!
  roslaunch ur5_ros_gazebo motion_server.launch --wait &
  MOTION_PID=$!
fi

# ───────────────────── wait for Ollama -----------------------------------
if [ "$LLM_BACKEND" = "ollama" ]; then
  echo "[llm-$LLM_MODEL] waiting for Ollama at $OLLAMA_HOST …"
  for _ in {1..60}; do
    curl -fsS "$OLLAMA_HOST/api/tags" >/dev/null 2>&1 && { echo "[ai] Ollama is up."; break; }
    sleep 2
  done
fi

# ───────────────────── cleanup on exit -----------------------------------
cleanup() {
  kill ${MOTION_PID:-} 2>/dev/null || true
  kill ${SIM_PID:-}    2>/dev/null || true
  kill ${ROSCORE_PID:-} 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ───────────────────── launch the LLM REPL -------------------------------
exec python3 /ur5_sim/ws/src/ur_motion_api/scripts/llm.py \
     --backend "$LLM_BACKEND" --model "$LLM_MODEL"
