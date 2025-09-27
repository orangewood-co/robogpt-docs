# RoboGPT Node Stack (Block Programming & Flow Execution)

This document explains the `robogpt_node_stack` module: its purpose, internal layout, launch interfaces, and the Python components that transform visual/JSON “block” flows into executable robot programs. It complements the higher‑level bring‑up docs by focusing specifically on the visual programming / sequence tools layer.

---

## 1. Overview

The node stack enables a low‑code pipeline:

1. A web/visual editor emits a compiled JSON flow (graph of nodes).
2. The stack validates and (optionally) converts that flow into a deterministic Python program (`programs/<name>.py`).
3. Tools (robot actions, utility functions) are dynamically discovered and executed in sequence or through generated code.
4. Execution status, validation errors, and logging are surfaced through Pusher events back to the UI.

Core responsibilities:
- Real‑time ingestion of user / UI events (save pose, compile flow, run flow).
- Validation and sanitization (loop structure, required node arguments, removal of “utility” nodes from control chain).
- Code generation (convert node graph to readable, idempotent Python script).
- Runtime execution (direct in‑memory sequence execution OR dynamic import of generated scripts).
- Tool discovery and binding (reflect over `tools.py` to produce executable catalogue).

Why this structure?
- Separation of concerns: parsing / validation / code generation / execution are independent modules for easier testing and evolution.
- Extensibility: adding new tools only requires defining a class with `tool_name` + `_execute` in `tools.py`.
- Resilience: generated code can be versioned, diffed, audited, and re‑run offline.

---

## 2. Directory Layout (subset)

```
robogpt_node_stack/block_programming/
├─ api_calling/          # Event-driven entry scripts (Pusher listeners)
│  ├─ sequence_save.py   # Receives compiled JSON → validates → generates code
│  ├─ sequence_executor.py # Receives compiled JSON → executes directly (no file)
│  ├─ save_pose.py       # Saves named poses / joint configs via tools
│  └─ run_program.py     # Executes previously generated program modules
├─ node_flow/
│  ├─ flow_parser.py     # Converts JSON graph → linear execution sequence
│  └─ flow_executor.py   # Runs a prepared sequence against loaded tools
├─ flow_program_generator/
│  ├─ node_to_python.py  # Sequence → Python code generator
│  └─ loop.py            # Loop expansion helpers for codegen
├─ flow_validator/
│  ├─ validator.py       # Structural & argument validation
│  └─ web_logger.py      # Surfaces success/error flags back to UI
├─ tools/
│  ├─ tools.py           # Concrete tool classes (each with tool_name, _execute)
│  ├─ tool_loader.py     # Discovers & instantiates tools (ROS node context)
│  ├─ tool_initializers.py # Shared state injection (e.g., MasterNode, Bot_Mode)
│  └─ tool_segregator.py # Categorization (control vs utility) used by parser
├─ programs/
│  └─ <generated>.py     # Auto-generated runnable scripts
└─ launch/
	└─ nodes.launch.py    # Unified launch: save / execute modes
```

---

## 3. Launch File: `nodes.launch.py`

The launch file orchestrates which event listener nodes to start based on a single `mode` argument.

Arguments:
| Name | Default | Choices | Purpose |
|------|---------|---------|---------|
| mode | save | save, execute | Select workflow: generate code vs execute raw sequence |
| robot_name | ec63 | — | Robot identifier (passed to tools) |
| robot_ip | 192.168.1.201 | — | Controller IP (forwarded to tools) |
| use_sim | false | true/false | Simulation toggle propagated to tools |
| planner_type | internal | internal/moveit | Motion planner hint for tools |

Startup logic:
- In save mode: starts `sequence_save.py` (compiles + generates code) plus common nodes.
- In execute mode: starts `sequence_executor.py` (direct execution) plus common nodes.
- Common nodes always started:
  - `save_pose.py` (listen for pose save events on `private-chat` channel)
  - `run_program.py` (dynamic import and execution of generated scripts on demand)

Why timers or ordering aren’t used: These nodes are passive listeners and can initialize concurrently; they only act when events arrive.

---

## 4. Event Channels & Pusher Integration

| File | Channel | Event | Purpose |
|------|---------|-------|---------|
| sequence_save.py | private-robogpt | compiled-json | Receive compiled flow for validation + code generation |
| sequence_executor.py | private-robogpt | compiled-json | Receive compiled flow for immediate execution |
| run_program.py | private-robogpt | run | Request to execute already generated program |
| save_pose.py | private-chat | evt::save-pose | Save a named pose or joint state |

Security / filtering: `user_auth` methods compare incoming user IDs to environment variables (`USER_ID` or `CLIENT_ID`). Extend with signature/HMAC if stricter auth is required.

---

## 5. Flow Processing Pipeline

1. User presses “Compile” → frontend emits `compiled-json` with: `userId`, `programName`, `compiled` (node array).
2. `JsonFlowParser.get_execution_sequence()`:
	- Loads tool category mapping from `ToolSegregator`.
	- Rewires connections to skip “utility” nodes while preserving control order.
	- Produces linear list of execution dicts (each with id, type, data, trimmed connections).
3. Validation (`FlowValidator`): loop shape & argument presence. Failing steps call `WebLogger._raise_flag` with error context.
4. Code generation (`SequenceToPython.sequence_to_python`) optionally filters loop nodes and renders template with step comments.
5. Program storage: saved into `programs/<program_name>.py` (lowercased underscore name).
6. Execution:
	- Direct mode: `FlowExecutor.execute_sequence()` iterates sequence and calls matching tool `_execute`.
	- Script mode: `run_program.py` reloads module and calls either function with same name or `execute_node_sequence()`.

---

## 6. Tool Discovery & Execution

`ToolLoader` dynamically imports `tools.py`, instantiates classes that expose `tool_name` and `_execute`. It also subscribes to `/robot_status` (message: `RobotStatus`) to capture `bot_mode` and injects it via `tool_initializers` so tools can adapt behavior.

Matching logic:
- Exact tool-name match first.
- Fallback to fuzzy match using `difflib.get_close_matches` (both in executor and generator) for resilience to UI label variations.

Extending tools:
1. Add a class in `tools.py` with a unique `tool_name` attribute.
2. Implement `_execute(**kwargs)` returning bool or result payload.
3. (Optional) use `tool_initializers` for shared state (e.g., ROS node handle).
4. No launch changes required—auto-discovered at next startup.

---

## 7. Generated Program Structure

Generated scripts follow a deterministic scaffold:
```
#!/usr/bin/python3 -u
import sys, getpass
sys.path.append('/home/<user>/orangewood_ws/src')
from robogpt_node_stack.block_programming.tools.tools import *

def execute_node_sequence():
	 """Execute the node sequence as defined in the visual programming interface."""
	 # Initialize tools
	 <tool_var> = <ToolClass>()
	 # Step N: <Readable Title>
	 resultN = <tool_var>._execute(param=value,...)
	 if not resultN: return False
	 ...
	 return True

if __name__ == "__main__":
	 execute_node_sequence()
```

Loop constructs are expanded inline via `loop.py` before being written, and loop bodies are filtered so duplicates aren’t emitted outside their control scope.

---

## 8. Error Handling & Logging

| Layer | Mechanism | Notes |
|-------|-----------|-------|
| Parser | Raises ValueError for missing start node | Upstream callers should catch and surface to UI |
| Validator | `_raise_flag(msg, flag='error')` via `WebLogger` | Emits success/error statuses |
| Executor | Per-node try/except with printed trace | Can be upgraded to structured ROS logging |
| Codegen | Catches tool mapping failures and inserts warning comments | Execution continues for remaining nodes |

Enhancements to consider: unify logging through rclpy logger & attach correlation IDs from event payload.

---

## 9. Typical Usage Scenarios

### A) Author new flow
1. Build blocks visually, press Compile.
2. `sequence_save` validates & generates script.
3. Operator invokes Run → `run_program` imports and executes script.

### B) Rapid iteration (no code file)
1. Compile flow.
2. `sequence_executor` executes in memory immediately (start with launch `mode:=execute`).

### C) Pose authoring
1. UI triggers `evt::save-pose` with metadata.
2. `save_pose.py` routes to tool `save_pose` or `save_joint`.
3. Stored positions become available to subsequent tools.

---

## 10. Environment Variables Consumed

| Variable | Purpose |
|----------|---------|
| PUSHER_APP_ID / PUSHER_KEY / PUSHER_SECRET / PUSHER_CLUSTER | Auth & connection for Pusher channels |
| DEFAULT_PUSHER_KEY | API key used for channel operations |
| USER_ID / CLIENT_ID | Used by `user_auth` to restrict execution |

Missing variables result in silent or printed warnings; consider enforcing via a startup validation tool.

---

## 11. Security & Safety Considerations

- Dynamic import (`run_program.py`) reloads modules—ensure generated code directory is not world‑writable in production.
- Tool interface executes arbitrary robot commands; consider a capability whitelist per `use_case`.
- Add future signing step: hash + signature embedded in generated scripts; verify before execution.

---

## Appendix A — Key Python Components

### A.1 sequence_save.py
Workflow: Pusher listener → parse → validate loops & args → generate Python script. Uses: `JsonFlowParser`, `SequenceToPython`, `FlowValidator`, `WebLogger`.

Important methods:
- `_update_sequence(event_data)` – Ingest JSON, generate script on success.
- `_fixing_program_name(name)` – Sanitize program filename.

### A.2 sequence_executor.py
Directly executes node sequence (no file). Calls `FlowExecutor.execute_sequence()`. (Note: contains a likely bug: assigns `self.client.event_handler = lambda ...: self.update_sequence` but method is `_update_sequence` — fix recommended.)

### A.3 run_program.py
Dynamic import/reload of scripts; finds a function named after sanitized program OR `execute_node_sequence` fallback. Enforces simple user auth.

### A.4 save_pose.py
Listens for `evt::save-pose`; dispatches to `save_pose` or `save_joint` tool instances resolved by `ToolLoader`.

### A.5 flow_parser.py
Generates linear execution order while stripping “utility” nodes via graph rewiring. Functions: `get_execution_sequence`, `flow_info`, `_find_node_by_id`.

### A.6 flow_executor.py
Matches node types to tool instances (exact or fuzzy). Methods: `execute_sequence`, `_get_best_match_tool`, `_execute_node`.

### A.7 node_to_python.py (SequenceToPython)
Converts execution sequence to a standalone script. Handles:
- Fuzzy tool mapping.
- Loop node filtering via `Loop.loop_nodes`.
- Step annotation + early abort on failure.

### A.8 tool_loader.py (ToolLoader)
Singleton pattern + ROS node context. Discovers classes in `tools.py` with `tool_name` + `_execute`. Subscribes to `/robot_status` to propagate `bot_mode` into `tool_initializers`.

---

## Appendix B — Future Improvements (Suggestions)

| Area | Enhancement |
|------|-------------|
| Auth | Replace env ID check with signed tokens per event |
| Validation | Add schema versioning + JSON schema enforcement |
| Logging | Structured ROS2 logging with namespace per program run |
| Tooling | Hot-reload detection & diff reporting for regenerated scripts |
| Safety | Dry-run mode: validate all tools support parameters before execution |
| UI Feedback | Emit granular per-step success/failure events back over Pusher |

---

## 12. Quick Start

Generate & Run (two-phase):
```bash
ros2 launch block_programming nodes.launch.py mode:=save robot_name:=ec63
# After compile event from UI → script saved
ros2 launch block_programming nodes.launch.py mode:=execute robot_name:=ec63
```

Direct execution (no intermediate script):
```bash
ros2 launch block_programming nodes.launch.py mode:=execute robot_name:=ec63
```

Save a pose (UI triggers event): ensure `save_pose.py` node is running (any mode) and send `evt::save-pose`.

---

Maintainers: Orangewood Robotics Team
