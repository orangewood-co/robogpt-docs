# RoboGPT Troubleshooting Guide

This page is a practical, task-first checklist for diagnosing and fixing issues in RoboGPT. It’s intentionally concise and organized by the high-level modules documented in dev-docs. We’ll fill in details and commands per section next.

Scope (high-level modules only):
- Agentic Layer (Pre-execution workflow)
- RoboGPT Agents
- Process Manager
- RoboGPT Nodes
- RoboGPT Perception
- RoboGPT Tools

---

## How to use this guide
1) Run the Preflight checks once per session/machine.
2) Identify the failing module (by symptom) and jump to its section.

---

## Preflight (run these before deep dives)
- Environment
  - ROS 2 overlay sourced (humble/iron/rolling as applicable)
  - Python virtualenv/uv environment active (if used)
  - Workspace built without errors
- System & network
  - Internet connectivity (if using cloud models/services)
  - Robot/controller reachable (if using hardware mode)
  - Time sync reasonable (NTP) to avoid cert/token drift
- Credentials & keys
  - Required API keys present (only if the selected module needs them) in bashrc

What to capture for escalation:
- OS, ROS 2 distro, GPU/CPU, Python version
- Exact commands you ran and full terminal output
- Any modified parameters/env vars

---

## Agentic Layer (Pre‑Execution Workflow)
Purpose: Middleware between the web app and RoboGPT core; routes/validates prompts and calls backend tools over RPC.

Symptoms (examples):
- No response to frontend prompts
- Tool calls never return / time out
- Validation blocks all prompts unexpectedly

Quick checks:
- Is the agentic process running? Is it connected to the queue/broker?
- Is RoboGPT Core reachable for RPC tool calls?
- Go to tools/backend and manually try to call tools via rpc_clients

Diagnostics to run:
- Process status/log tail
- Connectivity check to message broker/backend

---

## RoboGPT Agents
Purpose: Agent runtime, ROS 2 node glue, tool discovery/binding, and optional realtime bridge to the web app.

Symptoms (examples):
- Launch fails with parameter/type errors
- Tools not discovered/loaded. Or you get No available robots for task on webapp
- Agent runs but doesn’t react to events
- Issue in pusher or agent excecuting tools.

Quick checks:
- Launch arguments (use_case as list, robot_name, use_sim, planner_type)
- This issue is because the robot loader is unable to load robot wrappers. Check robot connection and move group name if using sim
- Agent might be getting confused in tools, re-start the agents or tweak prompts. 
- Check the bashrc with all the keys present over there. pusher and openai api keys both 

Common fixes:
- Check the input args and there data types
- check if the robot loader raises some issue in terminal,check robot connection - Ping robot ip - check moveit nodes and move group name is right or not (If using sim or planner moveit)

---

## Process Manager
Purpose: Central orchestration for launching, monitoring, restarting, and terminating ROS 2 processes.

Symptoms (examples):
- Launch service succeeds but process isn’t running
- Restart/kill doesn’t affect child processes
- Running multiple nodes / residual nodes 

Diagnostics to run:
- List processes and statuses.

Common fixes:
- Graceful kill then relaunch with original args. Go to webapp and click stop robogpt. Don't interrupt the terminal of process manager with Ctrl+C. 
- Verify shell environment sourcing for launch/run. Check if workspace sourcing is in bashrc or not. 

---

## RoboGPT Node Stack 
Purpose: ROS 2 nodes that make up the runtime (support, utilities, and integrations).

Symptoms (examples):
- Not getting Generated successfully message on webapp on clicking compile.
- Run button not working

Quick checks:
- Check terminal for logs and any errors in there
- check if code is generated in programs folder or not

---

## RoboGPT Perception
Purpose: Camera bring‑up, detection/segmentation, world‑context reasoning, ArUco pose, AutoTrain.

Symptoms (examples):
- Camera streams missing or mismatched names
- Pose calculator returns empty/invalid poses
- Segmentation service timeouts

Quick checks:
- Camera setup file present and correct (names/types/roles)
- Image/depth topics exist under the configured camera namespace

Diagnostics to run:
- Verify topics for image, depth, and pointcloud
- Call services (pose, segment, analyze) with minimal inputs

Common fixes:
- Re‑run camera setup to regenerate config
- Launch vision bringup with explicit camera parameters

What to capture for escalation:
- Camera config, topic list, service call payloads and responses

---

## RoboGPT Tools
Purpose: Tool definitions/implementations discoverable by the agent runtime and invoked by the model.

Symptoms (examples):
- Gripper tool not working on prompts
- Error in execution of simple move commands, saving pose commands

Quick checks:
- check -- if gripper is connected -- Robotiq gripper node is live -- 
- test running gripper open service from terminal 
- Do a sudo chmod 777 /dev/ttyUSB0 and re-run gripper node via webapp 
- Error in moveing robot - check if tools are loaded (check terminal logs) 
- check sdk issue if any args is missing
- In sim try moving robot via moveit - rviz if moves then check run wrapper tests



## Appendix — Quick commands (to be filled in)
We will populate module‑specific commands here (e.g., list parameters, check topics, call services) as we refine each section.


## Common Issue faced during testing - Status 


*Team inputs and issue status will be here*
