#!/usr/bin/env python3
"""
LLM Copilot for UR5 motion – chat in plain English and let the robot move.

Dependencies
------------
pip install ollama llama-cpp-python pydantic termcolor
"""

import os, json, sys, argparse
from typing import List, Optional
from termcolor import cprint

# --- Task‑3 Motion API -------------------------------------------------------
try:
    from ur_motion_api.motion_api import MotionAPI
except ImportError as exc:
    sys.stderr.write("❌  Could not import MotionAPI – did you build the workspace?\n")
    raise exc

api = MotionAPI()   # blocks until servers are ready

# --------------------------------------------------------------------------- #
# 1. Tool wrappers (these become JSON‑schema tools for Ollama)                #
# --------------------------------------------------------------------------- #
def llm_move_joint(
    point1: List[float],
    point2: List[float],
    velocity: float = 0.3,
    acceleration: float = 0.6,
) -> str:
    """
    Move robot joints from `point1` to `point2`.

    Args:
        point1: 6‑element list of start joint angles [rad].
        point2: 6‑element list of target joint angles [rad].
        velocity: joint speed [rad/s] (0–1.5).
        acceleration: joint accel [rad/s²] (0–3).

    Returns:
        Confirmation text describing the executed motion.
    """
    api.move_joint(point1, point2, velocity, acceleration)
    return f"✅ Executed joint move to {point2} @ {velocity} rad/s."

def llm_move_linear(
    pose1: dict,
    pose2: dict,
    velocity: float = 0.1,
    acceleration: float = 0.2,
) -> str:
    """
    Move robot TCP linearly from `pose1` to `pose2`.

    Args:
        pose1: dict with position (x,y,z) [m] and quaternion (x,y,z,w).
        pose2: dict with position (x,y,z) [m] and quaternion (x,y,z,w).
        velocity: Cartesian speed [m/s] (0–0.25).
        acceleration: Cartesian accel [m/s²] (0–0.5).

    Returns:
        Confirmation text describing the executed motion.
    """
    api.move_linear(pose1, pose2, velocity, acceleration)
    return f"✅ Executed linear move to {pose2['position']} @ {velocity} m/s."

TOOLS = [llm_move_joint, llm_move_linear]

# --------------------------------------------------------------------------- #
# 2. LLM back‑ends                                                            #
# --------------------------------------------------------------------------- #
class OllamaBackend:
    def __init__(self, model: str = "llama3.8b-instruct"):
        from ollama import chat  # lazy import
        self.chat_fn = chat
        self.model = model
        self.history = []

    def _stream_print(self, resp):
        for chunk in resp:
            content = chunk['message']['content']
            sys.stdout.write(content); sys.stdout.flush()

    def chat(self, user_msg: str):
        self.history.append({"role": "user", "content": user_msg})
        resp = self.chat_fn(
            model=self.model,
            messages=self.history,
            tools=TOOLS,
            stream=True,
        )
        tool_calls = []
        buffer = ""
        for chunk in resp:
            role = chunk['message']['role']
            content = chunk['message']['content']
            if role == "assistant":
                buffer += content
                sys.stdout.write(content); sys.stdout.flush()
            if 'tool_calls' in chunk['message']:
                tool_calls.extend(chunk['message']['tool_calls'])

        # handle any tool invocations
        for call in tool_calls:
            name = call['name']; args = json.loads(call['arguments'])
            result = globals()[name](**args)
            cprint(f"\n🔧 {name} -> {result}", "green")
            # expose result back to model so it can respond
            self.history.append({
                "role": "tool",
                "name": name,
                "content": result,
            })
        self.history.append({"role": "assistant", "content": buffer})

class LlamaCppBackend:
    """
    Fallback backend: prompt the model to emit a JSON block like
      { "function": "llm_move_joint", "args": {...} }
    Small models may hallucinate – but it's dependency‑free.
    """
    def __init__(self, model_path: str):
        from llama_cpp import Llama
        self.llm = Llama(model_path=model_path, n_ctx=4096)
        self.system_prompt = (
            "You are an expert UR5 motion planner. "
            "When the user asks to move, reply ONLY with a JSON dict:\n"
            "{ \"function\": \"llm_move_joint|llm_move_linear\", \"args\": {...} }"
        )

    def chat(self, user_msg: str):
        prompt = f"{self.system_prompt}\nUser: {user_msg}\nAssistant:"
        output = self.llm(prompt)["choices"][0]["text"].strip()
        try:
            call = json.loads(output)
            func = globals()[call["function"]]
            result = func(**call["args"])
            cprint(f"\n🔧 {call['function']} -> {result}", "green")
        except Exception as exc:
            cprint(f"\n⚠️  Failed to parse/execute: {exc}\nLLM said:\n{output}", "red")

# --------------------------------------------------------------------------- #
# 3. Main REPL                                                                #
# --------------------------------------------------------------------------- #
def main():
    parser = argparse.ArgumentParser(description="LLM Copilot for UR5.")
    parser.add_argument("--backend", choices=["ollama", "llamacpp"], default="ollama")
    parser.add_argument("--model", default="llama3.8b-instruct",
                        help="Ollama model (or GGUF path for llama‑cpp)")
    args = parser.parse_args()

    if args.backend == "ollama":
        cprint("🚀 Using Ollama backend", "cyan")
        bot = OllamaBackend(model=args.model)
    else:
        cprint("🚀 Using llama‑cpp‑python backend", "cyan")
        bot = LlamaCppBackend(model_path=args.model)

    cprint("Type your command (e.g. 'Move the arm 20 cm up')\nCtrl‑C to exit.\n", "yellow")
    try:
        while True:
            user = input("\n👤 You: ")
            bot.chat(user)
    except (KeyboardInterrupt, EOFError):
        cprint("\nBye!", "yellow")

if __name__ == "__main__":
    main()
