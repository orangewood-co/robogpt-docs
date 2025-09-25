# RoboGPT Tools

This repository consists of all the tools divided into different components (based on the applications) which will be called by RoboGPT agents. This toolkit provides all the robot and camera base components and skills along with some ready-to-use industrial application skills.

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [Applications](#applications-applications)
  - [Base Skills](#base-skills-applicationsbase)
    - [Core Robot Control](#core-robot-control)
    - [Motion Control](#motion-control)
    - [Position Management](#position-management)
    - [Utility Functions](#utility-functions)
  - [Vision & Analysis Skills](#vision--analysis-skills)
    - [Camera Analysis](#camera-analysis)
    - [Object Detection](#object-detection)
  - [Advanced Skills](#advanced-skills)
    - [Automated Testing](#automated-testing)
    - [Machine Learning](#machine-learning)
  - [ArUco Marker Skills](#aruco-marker-skills)
    - [Reference State Management](#reference-state-management)
  - [Industrial Application Skills](#industrial-application-skills)
    - [Machine Tending](#machine-tending-applicationsmachine_tending)
    - [Palletization](#palletization-applicationspalletization)
    - [Pick and Place](#pick-and-place-applicationspick_and_place)
- [Utilities](#utilities-applicationsutilities)
- [Robot Configuration](#robot-configuration-robot_config)
- [Robot SDK](#robot-sdk-robotsdk)
  - [OWL EC SDK](#owl-ec-sdk-sdkowl_ec_sdk)
  - [OWL Simulation](#owl-simulation-sdknowlsim_sdk)
  - [Hardware Wrappers](#hardware-wrappers-wrappers)
- [Architecture Features](#architecture-features)
  - [Finite State Machine (FSM) Pattern](#finite-state-machine-fsm-pattern)
  - [Error Handling Strategy](#error-handling-strategy)
- [Getting Started](#getting-started)
- [Development](#development)
  - [Adding New Features](#adding-new-features)
  - [Contributing](#contributing)


## Architecture Overview

### 1. Applications (`/applications`)
Central hub for all task-specific implementations. Contains modular skills organized by industrial automation categories:
- Base skills that serve as building blocks
- Specialized applications for specific industrial tasks
- Utility functions for common operations

### 2. Robot Configuration (`/robot_config`)
Centralized configuration management through JSON files:
- Robot specifications and capabilities
- Pre-defined poses and trajectories
- System-wide parameters and settings

### 3. Robot SDK (`/robot_sdk`)
Hardware abstraction and simulation interfaces:
- SDKs for different robot series
- Simulation capabilities
- Hardware abstraction layers

---

## Applications (`/applications`)

### Base Skills (`/applications/base`)

#### Core Robot Control

- **robotiq_gripper_implementation**: Gripper Control Interface  
  Activates/Deactivates the robotiq gripper with optional span control.

- **get_joint_implementation**: Joint State Monitoring  
  Returns the current joint values of the robot.

- **get_pose_implementation**: End Effector Pose Tracking  
  Returns the current pose of robot end effector.

- **get_zone_pose_implementation**: Predefined Position Management  
  Requests the pose of the robot at a particular zone using fuzzy matching.

- **hand_teach_implementation**: Teaching Mode Control  
  Switches the hand teach/gravity mode on or off based on config boolean.

- **control_IO_implementation**: Digital I/O Control  
  Activates/Deactivates the IO pin with True for HIGH and False for LOW.

#### Motion Control

- **move_translate_implementation**: Relative Motion Control  
  Tool to move robot in specific directions (left/right, forward/backward, up/down) using X/Y/Z coordinates in meters.

- **move_to_joint_implementation**: Joint Position Control  
  Tool to move robot to particular joint angles or named joint configurations.

- **move_to_pose_implementation**: Cartesian Position Control  
  Tool to move the robot to a particular pose or position in cartesian space.

#### Position Management

- **save_pose_implementation**: Position Storage  
  Tool to save the current position/pose of the robot with a given zone name.

- **save_joint_angles_implementation**: Joint Configuration Storage  
  Save the current joint angles of the robot in a JSON file under a given name for future use.

#### Utility Functions

- **delay_implementation**: Timing Control  
  Adds delay in the script with validation for reasonable delay times.

- **send_message_to_webapp_implementation**: UI Communication  
  Send a particular message to webapp using Pusher service.

### Vision & Analysis Skills

#### Camera Analysis

- **analyze_camera_frame_implementation**: Visual Analysis Interface  
  Tool to see what is around the robot from camera with natural language prompts.

- **compare_images_implementation**: Image Comparison Tool  
  Tool to compare two images and return the difference between them based on given prompt.

#### Object Detection

- **get_best_match_implementation**: Object Matching Service  
  Function to get the best match of an object using fuzzy string matching.

- **get_object_list_implementation**: Object Inventory  
  Function to get the list of objects detected by the robot vision system.

- **check_pose_implementation**: Object Position Calculator  
  Tool to calculate the given object's position in cartesian plane relative to specified frame.

### Advanced Skills

#### Automated Testing

- **industry_test_implementation**: Waypoint Testing Framework  
  Tool to test specific waypoints in multiple cycles for industrial applications with progress tracking and error handling.

#### Machine Learning

- **auto_train_implementation**: Object Training Interface  
  Tool that trains an object detection model to remember objects once shown, with configurable dataset size and training epochs.

- **auto_scan_implementation**: Training Data Collection  
  Tool to start the scanning of an object for autotrain by moving robot through predefined waypoints.

### ArUco Marker Skills

#### Reference State Management

- **get_reference_state_implementation**: ArUco State Reader  
  Get the reference state of the plane formed by 4 ArUco markers for calibration purposes.

- **save_reference_state_implementation**: Reference State Storage  
  Calculate and save the reference state of the plane formed by 4 ArUco markers with transformation matrices.

- **calculate_new_pose_state_implementation**: Dynamic Pose Calculator  
  Calculate the new pose of the robot based on the transformation of the plane formed by 4 ArUco markers.

### Industrial Application Skills

#### Machine Tending (`/applications/machine_tending`)
- **machine_tending_implementation**: Machine Tending Operations  
  Performs machine tending operations on CNC using ArUco markers for calibration.

- **machine_tending_with_feedback_implementation**: Machine Tending with Feedback  
  Performs machine tending operations with an inbuilt feedback loop.

#### Palletization (`/applications/palletization`)
- **palletization_implementation**: Palletization Operations  
  Tool to perform palletization operation/application for a pattern, providing the pallet and box dimensions.

- **Pattern1**: Standard Stacking Pattern  
  A normal stacking pattern where boxes are arranged in a grid layout on the pallet. The positions of the boxes are calculated based on the pallet dimensions, box dimensions, and a reference edge. Additional spacing is added between boxes for better stacking stability.

#### Pick and Place (`/applications/pick_and_place`)
- **pick_implementation**: Object Picking Tool  
  Tool to pick an object using a specified gripper. Validates inputs, determines target pose, moves to the target pose, and closes the gripper.

- **place_implementation**: Object Placement Tool  
  Tool to place an object at a specified pose. Validates inputs, determines target pose, moves to the target pose, and opens the gripper.

- **pick_and_place_implementation**: Combined Pick and Place  
  Tool to pick an object from a specified pose and place it at another pose. Combines the functionalities of pick and place tools.

## Utilities (`/applications/utilities`)

Support functions and services for the RoboGPT system:

- **helper_services.py**: External Service Integration  
  - `ExternalServices`: Core service class for integrating with ROS services
    - `get_object_pose()`: Finds object location in 3D space relative to the robot
    - `get_aruco_pose()`: Retrieves ArUco marker positions for calibration
    - `switch_sim_gripper()`: Controls gripper in simulation environment
    - `analyze_image()`: GPT-4o Vision integration for camera feed analysis
    - `compare_images()`: Image comparison for before/after analysis
    - `send_auto_train_goal()`: Initiates object detection model training

- **param_read_write.py**: ROS Parameter Management  
  - `ParameterWriter`: Advanced parameter setting utility
    - Automatic type detection for all ROS2 parameter types
    - Batch parameter operations with caching
    - Thread-safe service client management
  - `ParameterReader`: Parameter retrieval utility
    - Type-safe parameter reading with conversion
    - Batch parameter retrieval with proper error handling

- **robot_loader.py**: Robot Initialization and Management  
  - `RobotLoader`: Dynamic robot configuration loader
    - `is_robot_connected()`: Network connectivity validation
    - `get_matched_robot()`: Fuzzy matching for robot model selection
    - `load_robots()`: Dynamic SDK loading and robot wrapper initialization

- **robotiqgripper.py**: Robotiq Gripper Control Interface  
  - `RobotiqGripperClient`: Comprehensive gripper service client
    - `reset()`, `activate()`: Gripper initialization commands
    - `open()`, `close()`: Basic gripper operations
    - `go_to_position()`: Precise position control with force feedback
    - `go_to_width()`: Width-based control in millimeters

- **skill_initializers.py**: System Configuration and Globals  
  - Global configuration management for skills system
  - File path definitions for configuration files
  - Color coding utilities for debugging
  - Parameter service instances

- **utils.py**: Mathematical and Utility Functions  
  - `generate_scan_waypoints()`: Creates scanning patterns for object training
  - `markers_to_plane()`: Calculates plane transformation from ArUco markers
  - `pose_to_matrix()`: Converts 6DOF pose to transformation matrix
  - `matrix_to_pose()`: Converts transformation matrix back to pose format

## Robot Configuration (`/robot_config`)

- **bot_details.json**: Robot specifications containing list of robots and their details:
  - `robot_ip` (str): IP address for communication
  - `robot_group` (str): Category of robot
  - `gripper_group` (str): Type of gripper attached 
  - `time_out` (int): Connection timeout when connecting using IP
  - `gripper_enable` (bool): Enable/disable gripper with the robot

- **robot_pose.json**: List of saved poses as per robot specifications

## Robot SDK (`/robot_sdk`)

### OWL EC SDK (`/SDK/owl_ec_sdk`)
- **robot.py**: EC series interface
  - Real-time control capabilities
  - State monitoring and feedback
  - Safety systems integration
  - Comprehensive error handling

### OWL Simulation (`/SDK/owl_sim_sdk`)
- **robot.py**: Simulation core
  - Physics engine integration
  - Virtual sensor simulation
  - Environment modeling
- **Robot Models**:
  - `ec612.py`: EC612 robot simulation
  - `ec64.py`: EC64 robot simulation
  - `owl68.py`: OWL68 robot simulation

### Hardware Wrappers (`/wrappers`)
- **68_wrapper.py**: OWL68 abstraction layer
- **ec_wrapper.py**: EC series abstraction layer
- **sim_wrapper.py**: Simulation abstraction layer
  - Hardware abstraction layer implementation
  - Common interface using SDK
  - Real/simulation switching capability

## Architecture Features

### Finite State Machine (FSM) Pattern

All implementation classes follow a structured FSM approach with standardized states:

1. **Input Validation**: Parameter type checking and range validation
2. **Robot Selection Validation**: Ensures valid robot instance selection  
3. **Core Functionality Execution**: Main skill implementation
4. **Error Handling and Recovery**: Graceful failure management
5. **Result Return**: Consistent response formatting

### Error Handling Strategy

- Comprehensive exception handling with descriptive error messages
- Validation of robot selection against available robot instances
- Input parameter validation with type checking
- Graceful degradation and fallback mechanisms
- User-friendly error reporting through webapp integration


## Getting Started

1. Clone the repository:
   ```bash
   git clone https://github.com/orangewood-co/robogpt_tools.git
   ```

2. This is used with `robogpt_agents` for loading the robot, gripper model and defining the use_cases i.e. skills to use

## Development

### Adding New Features
1. Extend base skills in `applications/base/skills.py`
2. Create new application modules with new application name folder
3. Update configuration files
4. Test in simulation
5. Deploy to hardware

### Contributing

When adding new skills:

1. Follow the FSM pattern for consistent state management
2. Implement comprehensive error handling and validation
3. Use consistent naming, define all input types and documentation
4. Include safety checks and bounds validation
5. Test with multiple robot configurations
6. Update this documentation with new skill descriptions