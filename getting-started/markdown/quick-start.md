### Quick Start

---
#### 1. Platform Orchestrated Start
To be updated soon

Refer (when ready) to platform deployment docs for one-click bring-up, remote orchestration, credentials, and monitoring.

---
#### 2. Manual Module Run (Debug / Local Development)
Minimal cheat sheet of launch/run commands. For explanations see the corresponding dev-doc pages (links in brackets).

##### Core Agent Layer  [See: robogpt-agents]
```bash
# Run agent only
ros2 launch robogpt_agents run_agent.launch.py use_case:='["base"]' robot_name:=ec63 use_sim:=false robot_ip:=192.168.1.200 planner_type:=internal

# Run agent in simulation
ros2 launch robogpt_agents run_agent.launch.py use_case:='["base"]' robot_name:=ec63 use_sim:=true planner_type:=internal
```

##### Integrated Bring-Up  [See: robogpt-agents / hardware bringup]
```bash
# Full stack (agent + hardware + vision + block programming)
ros2 launch robogpt_startup robogpt_bringup.launch.py robot_name:=ec63 robot_ip:=192.168.1.200 use_sim:=false use_case:='["base"]' add_camera:=true add_gripper:=true enable_vision:=true planner_type:=internal

# Simulation variant (skips hardware_bringup)
ros2 launch robogpt_startup robogpt_bringup.launch.py robot_name:=ec63 use_sim:=true use_case:='["base"]' enable_vision:=true

# Legacy startup launcher
ros2 launch robogpt_startup startup.launch.py robot_name:=ec63 use_sim:=true use_case:=base vision_sim:=on
```

##### Hardware Bringup Direct  [See: hardware_bringup]
```bash
# Internal planner (no MoveIt)
ros2 launch hardware_bringup hardware_bringup.launch.py planner_type:=internal robot_name:=ec63 robot_ip:=192.168.1.200 add_camera:=true add_gripper:=true add_holder:=true

# Elite + MoveIt (ec series)
ros2 launch hardware_bringup hardware_bringup.launch.py planner_type:=moveit robot_name:=ec66 robot_ip:=192.168.1.200 auto_connect:=True moveit_delay:=5.0 add_camera:=true add_holder:=true

# OWL + MoveIt
ros2 launch hardware_bringup hardware_bringup.launch.py planner_type:=moveit robot_name:=owl68 robot_ip:=192.168.1.200 sampling_freq:=250 max_vel:=0.9 add_camera:=true
```

##### Individual Drivers / Status  [See: hardware_drivers]
```bash
# Elite arm driver only
ros2 launch elite_arm_driver bringup.launch.py ip_address:=192.168.1.200 arm_type:=ec66 add_camera:=true add_holder:=true

# Owl MoveIt driver node (stand-alone)
ros2 run owl_moveit_driver owl_moveit2_driver.py --ros-args -p robot_ip:=192.168.1.200 -p sampling_freq:=250 -p max_vel:=0.87

# Robot status publisher
ros2 run robot_status_publisher ec_status.py --ros-args -p robot_name:=ec63 -p robot_ip:=192.168.1.200 -p use_sim:=false
```

##### Vision / Perception  [See: robogpt-perception]
```bash
# Full vision bringup
ros2 launch robogpt_perception vision_bringup.launch.py

# Detection only
ros2 launch robogpt_perception detection_bringup.launch.py

# World context (semantic / context layer)
ros2 launch robogpt_perception world_context_bringup.launch.py

# Camera setup helpers
ros2 run robogpt_perception camera_setup.py
```

##### Block Programming / Node Stack  [See: robogpt-nodes]
```bash
# Listen for flow compile (generate Python program)
ros2 launch block_programming nodes.launch.py mode:=save robot_name:=ec63 robot_ip:=192.168.1.200 use_sim:=false

# Direct execute incoming flow (no file generation)
ros2 launch block_programming nodes.launch.py mode:=execute robot_name:=ec63 robot_ip:=192.168.1.200 use_sim:=false
```

##### Prompt Validator / Tool Tester  [See: prompt-validator]
```bash
ros2 run prompt_validator tool_tester.py --ros-args -p robot_name:=ec63 -p robot_ip:=192.168.1.200 -p use_sim:=false
```

##### Gripper Control  [See: robotiq_gripper]
```bash
ros2 run robotiq_gripper robotiqGripperService.py
```

##### Process Manager  [See: process-manager]
```bash
# Server
ros2 run process_manager process_manager_server.py
```

##### Agentic Workflow (Alternate / Experimental)  [See: agentic-layer]
```bash
# Run experimental agent workflow main entry
python -m robogpt-agentic-workflow.main
```

##### Misc Utilities
```bash
# Generate / update skill definitions
ros2 run robogpt_startup sending_skill_data.py

# Startup connector API (FastAPI service)
python /home/$USER/orangewood_ws/src/robogpt/core_stack/robogpt_startup/startup/connect_platform.py
```

---
Use these commands as a quick operational checklist. For parameter semantics and deeper architecture see the linked dev-doc pages.