# LLM-enabled UR5 Control Suite
 A Dockerized ROS Noetic and Gazebo environment for UR5 simulation, motion planning, and natural-language command generation via an LLM copilot.

## Overview

The project is organized as a ROS Noetic Catkin workspace packaged inside Docker. The top-level `ur5_sim/` folder contains a `ws/` directory with two primary ROS packages:

* **`ur5_ros_gazebo`** – C++ nodes, a MotionLibrary for trajectory generation, MoveJoint/MoveLinear action servers, and a sine‑wave demo client.
* **`ur_motion_api`** – Python action‑client wrappers and an LLM-powered REPL that translates natural language into MotionAPI calls.

Docker Compose services wire these pieces together, providing containers for development, the motion server, the sine demo, the LLM REPL, and an Ollama model host.

## Features

This package delivers a fully containerized UR5 simulation and control suite with:

* **Sinusoidal Joint Demo**: automated sine‑wave trajectories visualized in Gazebo, showcasing dynamic joint motion.
* **C++ MotionLibrary**: high‑performance trajectory planner supporting joint‑space interpolation and straight‑line Cartesian moves, backed by KDL inverse-kinematics.
* **Python MotionAPI**: intuitive action‑client wrapper exposing simple `move_joint` and `move_linear` primitives for rapid scripting.
* **LLM‑driven ai**: an interactive REPL powered by Ollama, translating natural-language commands into valid MotionAPI calls.

---

## Repository Structure

```plaintext
ur5_sim/                   # root
├─ Dockerfile              # multi‑stage build for ROS workspace
├─ docker-compose.yml      # services: dev, sine demo, motion server, ai, ollama
├─ ws/                     # Catkin workspace
│   └─ src/
│       ├─ universal_robot/ur_kinematics   # sparse‑cloned UR kinematics package
│       ├─ ur5_ros_gazebo/                  # C++ simulation & motion library
│       │   ├─ src/                         # source files and ROS nodes
│       │   └─ launch/                      # launch files for sim and demos
│       └─ ur_motion_api/                   # Python API & LLM ai
│           ├─ scripts/                     # motion_api.py, llm.py
│           └─ launch/                      # optional Python demos
└─ docker/                # docker helper scripts (entrypoints)
```

---

## Initial Setup

First, clone the sparse `ur_kinematics` subset of the ROS‑Industrial Universal Robot repository and build your Docker images:
> **Note:** Run these commands from the **root of the project directory** (`ur5_sim/`).

```bash
cd ws/src

git clone --depth 1 --filter=blob:none --sparse \
          --branch noetic-devel \
          https://github.com/ros-industrial/universal_robot.git

cd universal_robot
git sparse-checkout set ur_kinematics

cd ../..
docker compose build
```

---

## Quick Start

> **Note:** Run these commands from the **root of the project directory** (`ur5_sim/`).

1. **Start the Ollama LLM service**

   ```bash
   cd ur5_sim  # ensure you’re in the project root
   docker compose up -d ollama
   ```

2. **Launch the UR5 motion action server**

   ```bash
   cd ur5_sim  # project root still
   docker compose run --rm ur5_motion_server
   ```

3. **Start the LLM ai REPL**

   ```bash
   cd ur5_sim  # project root
   docker compose run --rm ur5_llm
   ```

At the `>>>` prompt, type natural language commands (e.g. `move up 10 cm`).

---

## Detailed Startup Guide

Below is a breakdown of each `docker compose` command and what it initializes within the system:

| Command                                     | Description                                                                                                                               |
| ------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| `docker compose up -d ollama`               | Pulls and starts the Ollama LLM daemon in detached mode, exposing the `/api/chat` endpoint on port 11434.                                 |
| `docker compose run --rm ur5_dev`           | Launches an interactive development container with full access to the workspace, Gazebo GUI, and ROS environment. For development purposes.                    |
| `docker compose run --rm ur5_sine_demo`     | Runs the sine-wave joint demo: publishes a sinusoidal trajectory on all UR5 joints to the Gazebo simulation.                              |
| `docker compose run --rm ur5_motion_server` | Builds the workspace, sources ROS, and starts the `motion_action_server` along with ROS core. |
| `docker compose run --rm ur5_llm`   | Starts the LLM ai REPL, waits for Ollama and the motion action servers, then interprets NL commands into MotionAPI calls.            |

---

## Node Overview

| Node Name                       | Package          | Executable / Script        | Purpose                                                     | Interfaces                                                                                                                               |
| ------------------------------- | ---------------- | -------------------------- | ----------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| **sine\_joint\_action\_client** | `ur5_ros_gazebo` | `sine_joint_action_client` | Publishes sine‑wave joint commands to controller            | Publishes `FollowJointTrajectory` to `/eff_joint_traj_controller/follow_joint_trajectory`                                                |
| **motion\_action\_server**      | `ur5_ros_gazebo` | `motion_action_server`     | Action server for MoveJoint and MoveLinear actions          | Subscribes `/joint_states`, offers `move_joint` and `move_linear` actions, sends to `/eff_joint_traj_controller/follow_joint_trajectory` |
| **motion\_demo\_node**          | `ur5_ros_gazebo` | `motion_demo_node`         | KDL MotionLibrary bring‑up and readiness check              | Logs readiness; no active topics/actions                                                                                                 |
| **motion\_api.py**              | `ur_motion_api`  | Python module              | High‑level API for action‑based UR5 control                 | Connects to `move_joint` & `move_linear` action servers, subscribes `/joint_states`                                                      |
| **llm.py**             | `ur_motion_api`  | `llm.py`           | REPL that translates NL commands via LLM to MotionAPI calls | Reads user input; calls Ollama; executes Python under `api` namespace                                                                    |

---

## Launch File Overview

| Launch File            | Package          | Purpose                                                                             |
| ---------------------- | ---------------- | ----------------------------------------------------------------------------------- |
| `ur5_motion_demo.launch` | `ur5_ros_gazebo` | Starts roscore and the `motion_action_server` node, setting up action interfaces.   |
| `sine_demo.launch`     | `ur5_ros_gazebo` | Runs the sine‑wave joint trajectory demo, publishing continuous joint commands.     |
| `ur_motion_api.launch`   | `ur_motion_api`  | Launches a Python demo node using MotionAPI for scripted joint and Cartesian moves. |

---

## Next Steps for Newcomers

1. **Spin up the containers** using the [Quick Start](#quick-start) commands to explore the motion server and LLM REPL.
2. **Study ROS actions and trajectories** such as `control_msgs/FollowJointTrajectoryAction` to understand how MotionLibrary builds and executes motion plans.
3. **Experiment with `motion_api.py`**—write small Python scripts with `move_joint` and `move_linear` to build confidence with action-based control.
4. **Explore the LLM workflow** in `llm.py` to customize prompts or add safety checks around generated code.
5. **Dive into kinematics** via the UR-specific IKFast solver in `ur_kinematics` for more advanced trajectory or robot adaptations.
6. **Expand the simulation** by adding sensors, alternate controllers, or complex demos such as pick-and-place sequences.

## License

This project is MIT licensed. See [LICENSE](LICENSE) for details.
