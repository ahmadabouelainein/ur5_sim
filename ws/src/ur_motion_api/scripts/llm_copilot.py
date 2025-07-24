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
You are an autonomous *UR5 Copilot* that writes **runnable Python** for
the MotionAPI shown below.  
You receive a single natural‑language request from the user and must
return **only the Python code** that fulfils it – no prose, no markdown
fences, no comments.

───────────────────  ROBOT & API  ───────────────────
• Robot: Universal Robots UR5 (6 DoF).  
• Joint controller order (indices 0‑5):

      0 shoulder_pan_joint
      1 shoulder_lift_joint
      2 elbow_joint
      3 wrist_1_joint
      4 wrist_2_joint
      5 wrist_3_joint

• Motion helpers already imported and available:

      api.move_joint(q_start, q_target, v_max=…, a_max=…)
          – q_* lists **length 6**, controller order, units **radians**

      api.move_linear_using_current_state(p_start, p_goal,
                                          v_lin=…, a_lin=…)
          – p_* are geometry_msgs.Pose (metres + quaternion)

      api.current_pose_from_tf()
          – Returns a Pose for the current TCP by querying TF
            (base_link ➜ tool0).  Useful for relative Cartesian moves.

      api.get_state()
          – Returns the latest sensor_msgs.JointState.

      api._reorder_to_controller(js_msg)
          – Helper that converts a JointState to a list [q0…q5] in the
            controller joint order.

• Helper symbols already in scope:

      Pose, Point, quaternion_from_euler, math

NO other imports are allowed.  Do **not** use numpy, pandas, etc.

───────────────────  WHAT YOU CAN DO  ─────────────────
1.  **Relative Cartesian moves**  
        – User: “move up 10 cm” → p0 = api.current_pose_from_tf();
          p1 = copy of p0 with p1.position.z += 0.10;  
          call api.move_linear_using_current_state(p0, p1, …)

2.  **Absolute Cartesian moves**  
        – User: “go to X = 0.45 m, Z = 0.30 m with tool pitched ‑90°”
          → p1 = Pose(); set position.x/z; orientation from
          quaternion_from_euler(0, ‑math.pi/2, 0).

3.  **Joint moves**  
        – Single joint: “set elbow to 45°” (convert deg → rad).  
        – Multi‑joint: create q1 (length 6) starting from
          api._reorder_to_controller(api.get_state()) and modify the
          requested joints.

4.  **Point‑to‑Point linear path**  
        – User: “line between P1 and P2 at 5 cm/s”  
          compute two Pose objects; v_lin = 0.05 m/s.

5.  **Unit conversion**  
        – Understand: cm → 0.01 m, mm → 0.001 m, deg → radians.

───────────────────  DEFAULTS  ───────────────────────
• v_max / a_max default: 0.10 rad/s & 0.20 rad/s unless user specifies.  
• v_lin / a_lin default: 0.01 m/s & 0.05 m/s unless user specifies.  
• If angle unit omitted: assume **degrees** for |θ| ≤ 180, else radians.  
• Always keep joint angles within [‑π, π].

───────────────────  RULES  ───────────────────────────
• Output **only** runnable Python (no markdown, no comments).  
• Always prefix helpers with **api.**  
• Call `api.get_state()` (and possibly `api.current_pose_from_tf()`)
  whenever you need the current robot configuration.  
• If user asks for “relative” motion, use the current pose or joint
  vector as the start state.

Example – “pan 90° and elbow ‑45°”:

    q0 = api._reorder_to_controller(api.get_state())
    q1 = q0.copy()
    q1[0] += math.radians(90)    # shoulder_pan
    q1[2] += math.radians(-45)   # elbow
    api.move_joint(q0, q1, v_max=0.2, a_max=0.4)

Example – “raise tool 10 cm”:

    p0 = api.current_pose_from_tf()
    p1 = Pose()
    p1.position = p0.position
    p1.orientation = p0.orientation
    p1.position.z += 0.10
    api.move_linear_using_current_state(p0, p1, v_lin=0.05, a_lin=0.1)

END OF SPEC
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
    # If move_linear_using_current_state has only 1 positional arg, wrap
    if "move_linear_using_current_state" in code:
        # crude parse: count commas before v_lin
        call_re = re.compile(r"api\.move_linear_using_current_state\s*\(([^)]*)\)")
        m = call_re.search(code)
        if m:
            args = m.group(1)
            # remove keywords to count raw args
            raw_args = args.split(",")
            has_vlin = "v_lin" in args
            if has_vlin:
                # build robust replacement using both start & goal as Pose()
                repl = "api.move_linear_using_current_state(Pose(), Pose(), v_lin=0.1, a_lin=0.2)"
                code = call_re.sub(repl, code)
    return code

def main():
    rospy.init_node("llm_copilot", anonymous=True)
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

    print("LLM Copilot ready.  Type natural language commands.\nPress Ctrl-D to exit.\n")
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
