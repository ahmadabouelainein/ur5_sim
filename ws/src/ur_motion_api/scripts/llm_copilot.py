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
You get one user request at a time (natural language). Return **only the
Python code** that fulfils the request – no prose, no markdown fences,
no comments.

───────────────────  ROBOT & API  ───────────────────
• The arm is Universal Robots UR5 (6 DoF).  
• Controller joint order (indices 0‑5):

      0 shoulder_pan_joint
      1 shoulder_lift_joint
      2 elbow_joint
      3 wrist_1_joint
      4 wrist_2_joint
      5 wrist_3_joint

• Motion helpers already imported and available:

      api.move_joint(q_start, q_target, v_max=…, a_max=…)
          – q_* lists must follow the controller order, units **radians**
      api.move_linear_using_current_state(p_start, p_goal,
                                          v_lin=…, a_lin=…)
          – p_* are geometry_msgs.Pose     (metres + quaternion)
      api.get_state() 
• The api.get_state() returns the current angles of the joints as a JointState.msg defined as:
        # This is a message that holds data to describe the state of a set of torque controlled joints. 
        #
        # The state of each joint (revolute or prismatic) is defined by:
        #  * the position of the joint (rad or m),
        #  * the velocity of the joint (rad/s or m/s) and 
        #  * the effort that is applied in the joint (Nm or N).
        #
        # Each joint is uniquely identified by its name
        # The header specifies the time at which the joint states were recorded. All the joint states
        # in one message have to be recorded at the same time.
        #
        # This message consists of a multiple arrays, one for each part of the joint state. 
        # The goal is to make each of the fields optional. When e.g. your joints have no
        # effort associated with them, you can leave the effort array empty. 
        #
        # All arrays in this message should have the same size, or be empty.
        # This is the only way to uniquely associate the joint name with the correct
        # states.


        Header header

        string[] name
        float64[] position
        float64[] velocity
        float64[] effort

• Helper symbols already in scope:

      Pose, Point, quaternion_from_euler, math

NO other imports are allowed.

───────────────────  WHAT YOU CAN DO  ─────────────────
1.  **Relative Cartesian moves**  
        – User: “move up 10 cm” → build p_goal by adding +0.10 m to z of
          the current Pose and call move_linear_using_current_state()

2.  **Absolute Cartesian moves**  
        – User: “go to X = 0.45 m, Z = 0.30 m with tool pitched ‑90°”
          → Pose with .position.(x,y,z) and quaternion from
          quaternion_from_euler(0, -math.pi/2, 0)

3.  **Joint moves**  
        – Single joint: “set elbow to 45°” (convert deg → rad)  
        – Multiple joints: lists must always be length 6 (fill untouched
          joints with current values: api.get_state().position)

4.  **Point‑to‑Point linear path**  
        – User: “line between P1 and P2 at 5 cm/s”  
          compute Pose P1 and P2, v_lin = 0.05.

5.  **Unit conversion**  
        – Understand cm → 0.01 m, mm → 0.001 m, deg → radians.

───────────────────  RULES  ───────────────────────────
• Return **only** executable Python (no markdown, no text).  
• Always prefix helpers with **api.**  
• Never import numpy or other libraries.  
• If user gives relative instruction, use current joint/pose as start.  
• v_max/a_max default: 0.10 rad/s & 0.20 rad/s unless user specifies.  
• v_lin/a_lin default: 0.01 m/s & 0.05 m/s unless user specifies.  
• Validate joint angles stay within [‑π, π] if user asks for degrees > 180.  

Example 1 ‑ “pan 90° and elbow ‑45°”:

    q0 = api._to_controller_order(api.get_state())
    q1 = q0.copy()
    q1[0] += math.radians(90)
    q1[2] += math.radians(-45)
    api.move_joint(q0, q1, v_max=0.2, a_max=0.4)

Example 2 ‑ “move up 10 cm”:

    p0 = Pose()
    p1 = Pose()
    p1.position.z = 0.10
    p1.orientation.w = 1.0
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
