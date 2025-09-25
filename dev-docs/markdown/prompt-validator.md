# Prompt Validator for RoboGPT
A ROS2 package for validating robot state, configurations, and commands in the Orangewood Robotics ecosystem.

### Overview
The Prompt Validator package provides a set of tools to validate robot states, check saved poses and joints, verify object detection, and test robot connectivity. It exposes these validation tools through a RabbitMQ RPC interface, making them available to other components in the system.


## Package Structure
```bash
prompt_validator/
├── CMakeLists.txt        - Build configuration
├── package.xml           - Package metadata
├── Readme.md             - This documentation
├── scripts/
│   ├── rpc_client.py     - Client for testing the RPC server
│   ├── tool_tester.py    - RPC server exposing validation tools
│   └── validator_tools.py - Core validation functionality
```

### Components
Validator Tools (validator_tools.py)
The core validation functionality is implemented in the ValidatorTools class. 

It provides the following tools:

| Tool Name | Arguments | Description |
|:----------:|:------------|:----------:|
| **check_saved_pose** | pose_name (str) | Checks if a pose exists in saved poses (returns boolean) | 
| **check_saved_joint** | joint_name | Checks if a joint configuration exists in saved joints (returns boolean) | 
| **check_robot_connection** | None | Checks if the robot is connected by pinging its IP (returns boolean) |
| **check_object** | object_name (str) | Checks if an object is detected by the vision system (returns boolean) | 
| **check_remote_mode** | None| Checks if robot is in remote mode or not before runninf any robot releated command (returns boolean) |


### RPC Server (tool_tester.py)
Exposes the validator tools over RabbitMQ RPC, allowing remote calls to the validation functions. The server:

- Initializes a ROS2 node for parameter access
- Establishes a secure RabbitMQ connection
- Handles incoming RPC requests and routes them to the appropriate validation tool
- Formats and returns the response to the client

### Test Client (rpc_client.py)
A simple client for testing the RPC server. It demonstrates how to:

- Connect to the RabbitMQ server
- Make calls to the validation tools
- Process responses
#### Installation and Setup
Prerequisites
- ROS2 Humble or later
- Python 3.8+
- pika (RabbitMQ client library)
- Access to Orangewood RoboGPT ecosystem

Building from Source

- Clone the repository into your Orangewood workspace:
```bash
cd ~/orangewood_ws/src
git clone https://github.com/orangewoodlabs/prompt_validator.git
```
- Build the package:
```bash
cd ~/orangewood_ws
colcon build --packages-select prompt_validator --symlink-install
```
- Source the workspace:
```bash
source ~/orangewood_ws/install/setup.bash
```
### Usage

Starting the RPC Server
```bash
ros2 run prompt_validator tool_tester.py
```
Testing with the Client
```bash
python3 ~/orangewood_ws/src/prompt_validator/scripts/rpc_client.py
```

To call a specific tool, modify the client call in rpc_client.py:
```bash
response = robot_tools_client.call("check_object", {
    "object_name": "ball"
})
```

#### Integration with Other Systems

The validator tools can be called from any system that can connect to the RabbitMQ server. The message format is:
```bash
tool_name#{"arg1": "value1", "arg2": "value2"}
```

For example:
```bash
check_saved_pose#{"pose_name": "home"}
```
