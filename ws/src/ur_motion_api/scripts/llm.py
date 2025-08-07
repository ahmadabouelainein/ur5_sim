#!/usr/bin/env python3
import os, sys, json, re, textwrap, requests, rospy
from motion_api import MotionAPI
import geometry_msgs.msg as gmsg
from geometry_msgs.msg import Pose, Point
from tf.transformations import quaternion_from_euler
import numpy as np
OLLAMA_HOST = os.getenv("OLLAMA_HOST", "http://localhost:11434")
MODEL       = os.getenv("LLM_MODEL",  "mistral:instruct")

SYSTEM_PROMPT = """
You are an autonomous *UR5 AI* writing **runnable Python** for MotionAPI.
Return **only** the Python code that fulfills the user's request – no prose, no fences.

── ROBOT & API ─────────────────────────────────────────────────────────
• Robot: Universal Robots UR5 (6 DoF).
• Joint controller order (0–5): shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3

• Action definitions:
    MoveJoint.action
      float64[] q_start
      float64[] q_target
      float64   v_max
      float64   a_max
      ---
      bool success
      ---
      float32 percent_complete

    MoveLinear.action
      geometry_msgs/Pose pose_start
      geometry_msgs/Pose pose_goal
      float64            v_lin
      float64            a_lin
      ---
      bool success
      ---
      float32 percent_complete

• MotionAPI helpers in scope:
    api.move_joint(q_start, q_goal, v_max=…, a_max=…)
    api.move_linear(pose_start, pose_goal, v_lin=…, a_lin=…)
    api.move_joint_sequence(q0, q1, v_max=…, a_max=…)
    api.move_linear_sequence(p0, p1, v_lin=…, a_lin=…)
    api._get_current_js()
    api._get_current_pose()
    Pose, Point, quaternion_from_euler, math

── CAPABILITIES ────────────────────────────────────────────────────────
1. Relative Cartesian moves: adjust p0 from api._get_current_pose(); call api.move_linear...
2. Absolute Cartesian moves: build Pose(); call api.move_linear...
3. Joint moves: modify list from api._get_current_js(); call api.move_joint...
4. Two-point linear paths: call api.move_linear_sequence(p0, p1, v_lin, a_lin)
5. Two-point joint sequences: call api.move_joint_sequence(q0, q1, v_max, a_max)
6. Unit conversions: cm→m, mm→m, deg→rad.

Defaults:
  v_max=0.10 rad/s, a_max=0.20 rad/s
  v_lin=0.01 m/s, a_lin=0.05 m/s
  angles assume degrees if |θ|≤180, else radians; wrap to [−π, π]

Rules:
  Output only Python code. Prefix calls with api. Use api helpers as given.
"""



def ollama_chat(history):
    body = {"model": MODEL, "messages": history, "stream": False}
    r = requests.post(f"{OLLAMA_HOST}/api/chat", json=body)
    if r.status_code == 404:  # fallback older daemon
        prompt = "\n".join(f"{m['role']}: {m['content']}" for m in history)
        r = requests.post(f"{OLLAMA_HOST}/api/generate",
                          json={"model": MODEL, "prompt": prompt, "stream": False})
    r.raise_for_status()
    data = r.json()
    return data.get("message", {}).get("content", data.get("response", ""))

_code_fence_re = re.compile(r"```(?:python)?\s*(.*?)```", re.S | re.I)

def extract_code(raw: str) -> str:
    m = _code_fence_re.search(raw)
    code = m.group(1) if m else raw
    # strip leading prose lines that don't look like code
    lines = []
    for ln in code.splitlines():
      if ln.strip().startswith("#"): continue
      if ln.strip() and not ln.startswith((" ", "\t")) and not ln.strip().startswith(("api.", "Pose", "Point", "q", "p")):
          # looks like stray prose token -> skip
          continue
      lines.append(ln)
    code = "\n".join(lines)
    code = textwrap.dedent(code).strip()
    return code

def sanitize_code(code: str) -> str:
    # map geometry_msgs.Point/Pose to bare Point/Pose
    code = re.sub(r"geometry_msgs(?:\.msg)?\.Point", "Point", code)
    code = re.sub(r"geometry_msgs(?:\.msg)?\.Pose",  "Pose",  code)
    return code

def main():
    rospy.init_node("llm", anonymous=True)
    api = MotionAPI()

    # execution namespace
    ns = {
        "api": api,
        "rospy": rospy,
        "Pose": Pose,
        "Point": Point,
        "quaternion_from_euler": quaternion_from_euler,
        "np": np
    }

    print("LLM ai ready.  Type natural language commands.\nPress Ctrl-D to exit.\n")
    history = [{"role": "system", "content": SYSTEM_PROMPT.strip()}]

    while not rospy.is_shutdown():
        try:
            user = input(">>> ")
        except EOFError:
            break
        if not user.strip():
            continue
        history.append({"role": "user", "content": user})

        raw = ollama_chat(history)
        llm_code = sanitize_code(extract_code(raw))
        print("\n--- generated code ---")
        print(textwrap.indent(llm_code, "    "))
        print("----------------------")

        try:
            exec(llm_code, ns)
            history.append({"role": "assistant", "content": llm_code})
        except Exception as e:
            print(f"❌ Error executing code: {e}")
            history.append({"role": "assistant",
                            "content": f"Execution error: {e}"})

if __name__ == "__main__":
    main()
