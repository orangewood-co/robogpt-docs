# RoboGPT Process Manager

Centralized orchestration layer for spawning, monitoring, restarting, and terminating ROS 2 launch files (or run executables) in a controlled, queryable manner. It exposes a ROS 2 service API so other nodes (agent, UI backend, automation scripts) can manage processes without shell access.

---

## 1. Goals & Rationale

Why a dedicated manager instead of ad‑hoc shell scripts:
- Consistent lifecycle semantics (launch → monitor → restart/kill) across heterogeneous subsystems (agent, perception, drivers, vision, tools).
- Safe termination: kills full process groups (parent + children) to avoid orphaned nodes.
- Introspection: list currently managed processes, their PIDs, arguments, and status.
- Future expansion: node-level operations (parameter inspection, restart) via the same interface.

Design pillars:
| Pillar | Implementation Detail |
|--------|------------------------|
| Isolation | Each launch executed in its own process group (setsid) so group signals are clean |
| Observability | Metadata stored in `process_info` dict (command, pid, pgid, args, status, children) |
| Resilience | Restart keeps original arguments; graceful SIGTERM escalates to SIGKILL if needed |
| Extensibility | Additional services for node-level ops already scaffolded (TODO stubs) |

---

## 2. Directory Layout

```
core_stack/process_manager/
├─ manager_server/
│  ├─ manager.py               # Core logic: launch, kill, restart, list, wait
│  └─ process_manager_server.py# ROS2 service node exposing API
├─ manager_clients/
│  └─ process_manager_client.py# Convenience client wrapper
├─ srv/                        # Custom service definitions
│  ├─ LaunchProcess.srv
│  ├─ KillProcess.srv
│  ├─ ListProcesses.srv
│  ├─ RestartProcess.srv
│  ├─ ListNodes.srv
│  ├─ KillNode.srv
│  ├─ RestartNode.srv
│  └─ GetNodeParameters.srv
├─ CMakeLists.txt / package.xml
└─ include/process_manager/    # (Reserved for future C++ or headers)
```

---

## 3. High-Level Flow

1. Client sends `LaunchProcess` request ⇒ server calls `ProcessManager.launch_with_delay()`.
2. Process is started under `bash -lc` ensuring `~/.bashrc` sources the ROS environment.
3. Manager records process group ID (PGID) + child PIDs (via `pgrep -g`).
4. Status queries call `get_process_status()` to poll underlying `Popen` objects.
5. Kill/restart operations use group signals: SIGTERM with timeout fallback to SIGKILL.
6. Listing builds enriched snapshot merging static metadata + dynamic status.

---

## 4. Service API

### 4.1 LaunchProcess.srv
Request
```
string name
string launch_package
string launch_file
string[] args
int32 delay
bool show_logs
string type   # 'launch' or 'run'
---
bool success
int32 pid
```
Notes:
- `type` selects underlying ros2 subcommand: `ros2 launch <pkg> <file>` vs `ros2 run <pkg> <executable>`.
- `delay` is applied *after* spawn to allow ordered multi-stage bring-up (e.g., network stabilization).
- `show_logs=true` attaches child stdout/stderr to terminal; otherwise they are piped (available via `get_process_output`).

### 4.2 KillProcess.srv
```
string name
bool force
---
bool success
```
Force = immediate SIGKILL of process group; else graceful SIGTERM → optional escalation after 3s.

### 4.3 ListProcesses.srv
```
---
string[] names
string[] statuses
```
Each status is the last cached `status` field (not always synchronized with `current_status`—see improvement ideas).

### 4.4 RestartProcess.srv
```
string name
int32 delay
---
bool success
```
Kills then re-launches with original args & launch type preserved.

### 4.5 ListNodes.srv / KillNode.srv / RestartNode.srv / GetNodeParameters.srv
These are placeholders; current implementation returns defaults / TODO stubs except ListNodes (which shells `ros2 node list`). Future support will allow targeted ROS node lifecycle control independent of process groups.

---

## 5. Core Class: `ProcessManager` (manager.py)

| Method | Purpose | Key Internals |
|--------|---------|---------------|
| launch_with_delay | Start ros2 launch/run in new process group; register metadata | uses `bash -lc` + `os.setsid()`; records child PIDs via `pgrep -g` |
| _get_child_pids | Collect group members | fallback to parent PID on failure |
| get_process_status | Poll exit code | returns running / terminated with exit code |
| list_processes | Build dict snapshot | merges cached info + dynamic status |
| kill_process | Signal process group | Graceful SIGTERM path w/ escalation; updates status |
| kill_all_processes | Iterate & kill | Calls `kill_process` for each |
| restart_process | Kill + relaunch | Reuses stored command/args/type |
| wait_for_process | Blocking wait | Returns exit code or None on timeout |
| wait_for_process_async | Threaded wait | Spawns daemon thread; invokes callback(name, exit_code) |
| get_process_output | Retrieve stdout/stderr | Non-blocking: 1s communicate timeout |
| list_ros_nodes | Shell out to `ros2 node list` | Returns sanitized list |
| kill_ros_node / restart_ros_node / get_node_parameters / monitor_node_parameter | Stubs | Marked for future enhancement |

### 5.1 Data Structures
`self.processes`: name → `subprocess.Popen`
`self.process_info`: name → metadata dict
```
{
	'agent_module': {
		'command': ['ros2','launch','robogpt_agents','run_agent.launch.py'],
		'pid': 12345,
		'pgid': 12345,
		'child_pids': [12345, 12367],
		'launch_package': 'robogpt_agents',
		'launch_file': 'run_agent.launch.py',
		'args': [],
		'status': 'running',
		'show_logs': true,
		'type': 'launch'
	}
}
```
Note: `type` is used on restart; ensure it’s added (currently referenced—verify in future patch that it is stored; if missing add to `process_info`).

---

## 6. Client Library (`process_manager_client.py`)

Provides ergonomic Python methods mirroring service calls. Each:
1. Waits for service with timeout.
2. Constructs request message.
3. Spins until future completes.
4. Normalizes return into dict / bool / list.

Usage sketch:
```python
from robogpt.core_stack.process_manager.manager_clients.process_manager_client import ProcessManagerClient

client = ProcessManagerClient()
client.launch_process('agent_module','robogpt_agents','run_agent.launch.py', show_logs=True)
print(client.list_processes())
client.restart_process('agent_module', delay=2)
client.kill_process('agent_module')
client.shutdown()
```

Threading: A `MultiThreadedExecutor` is created if client spawns its own node, enabling concurrent service waits.

---

## 7. Lifecycle & Signal Semantics

| Action | Signal(s) | Escalation |
|--------|-----------|------------|
| Graceful kill | SIGTERM (process group) | After 3s, SIGKILL if any child still alive |
| Force kill | SIGKILL (immediate) | None |
| Restart | SIGTERM → re-launch | Reuses stored metadata |

Using process groups ensures nested launch-spawned nodes exit together, preventing zombie build-ups after iterative development cycles.

---

## 8. Typical Scenarios

### 8.1 Layered Bring-Up
Sequentially launch drivers, perception, agent with explicit delays to avoid race conditions:
```python
client.launch_process('drivers','hardware_bringup','hardware_bringup.launch.py',['planner_type:=internal'], delay=0)
client.launch_process('vision','robogpt_perception','vision_bringup.launch.py', delay=3)
client.launch_process('agent','robogpt_agents','run_agent.launch.py', ['use_case:=["base"]'], delay=6)
```

### 8.2 Hot Reload Agent
```python
client.restart_process('agent', delay=1)
```

### 8.3 Graceful Shutdown All
```python
for name in list(client.list_processes().keys()):
		client.kill_process(name)
```

---

## 9. Error Handling & Edge Cases

| Situation | Current Behavior | Suggested Improvement |
|-----------|------------------|-----------------------|
| Launch fails (bad package/file) | Exception logged; success=False | Surface reason in response (stderr tail) |
| Lost child after crash | Status may still show running until poll updates | Periodic cleanup thread to prune defunct processes |
| Restart missing metadata | Returns False | Add explicit validation + service error field |
| Service timeout | Logs error, returns None | Retry with backoff |
| Node operations (kill_node, etc.) | Stub returns False | Implement via `ros2 lifecycle` or direct signal map |

---

## 10. Security Considerations

- Launch API can execute arbitrary ROS launch/run commands—restrict exposure (e.g., namespace remap, permission gate, or wrapper node checking allow‑list).
- Consider integrating an auth token parameter for each service request.
- Validate `args` for shell injection risk (currently safe—passed as array, but review when adding dynamic string interpolation).

---

## 11. Future Enhancements

| Category | Idea |
|----------|------|
| Metrics | Track start time, uptime, restart count |
| Health | Periodic liveness probe (CPU, memory) via psutil |
| Node Ops | Implement parameter fetch, dynamic reconfigure, targeted restarts |
| Streaming Logs | Service/Topic to tail logs of managed processes |
| Policy | Max restart retries + exponential backoff |
| Persistence | Save/restore last managed set on reboot |

---

## Appendix A — Service Contracts (Summary)

| Service | Purpose |
|---------|---------|
| process_manager/launch | Start new ROS process (launch/run) |
| process_manager/kill | Terminate managed process |
| process_manager/list | Enumerate managed processes |
| process_manager/restart | Restart a managed process |
| process_manager/list_nodes | List active ROS nodes (workspace) |
| process_manager/kill_node | (Stub) Kill a specific ROS node |
| process_manager/restart_node | (Stub) Restart a ROS node |
| process_manager/get_node_parameters | (Stub) Fetch parameters of a node |

---

## Appendix B — Internal Command Construction

Launch command path:
1. Build array: `['ros2', type, launch_package, launch_file] + args`.
2. Quote each part for safety.
3. Wrap with `source ~/.bashrc && exec ...` to ensure environment overlays are active.
4. Spawn via `subprocess.Popen(['bash','-lc', full_cmd], preexec_fn=os.setsid)`.

This guarantees consistent environment (ament overlays, PATH, PYTHONPATH) without requiring external wrappers.

---

## 12. Quick Start

Terminal 1 – start server:
```bash
ros2 run process_manager process_manager_server.py
```

Terminal 2 – interact via Python shell:
```python
from robogpt.core_stack.process_manager.manager_clients.process_manager_client import ProcessManagerClient
client = ProcessManagerClient()
client.launch_process('agent','robogpt_agents','run_agent.launch.py',['use_case:=["base"]'], show_logs=True)
print(client.list_processes())
client.kill_process('agent')
client.shutdown()
```

---

Maintainers: Orangewood Robotics Team
