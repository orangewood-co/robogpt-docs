
# 🤖 RoboGPT Documentation

**Your comprehensive guide to all RoboGPT software versions and repositories**


## 📋 Overview

This central hub contains all RoboGPT versions repo links, providing easy access to each repository's information, and development resources.

---

## 🚀 Installation Guide

### Prerequisites

Before installing RoboGPT v4, ensure you have the following:

- **Ubuntu 22.04** (recommended) or **Ubuntu 20.04**
- **ROS2 Humble** (for Ubuntu 22.04) or **ROS2 Galactic** (for Ubuntu 20.04)  |  *Recommeneded is ROS2 Humble*
- **Python 3.8+**
- **Git** installed and configured
- **colcon** build tools
- **rosdep** initialized

### Method 1: Manual Installation

#### Step 1: Create Workspace Structure

```bash
# Create the main workspace directory
mkdir -p ~/orangewood_ws/src
cd ~/orangewood_ws/src
```

#### Step 2: Clone All Required Repositories

```bash
# Core RoboGPT v4 repositories
git clone -b dev https://github.com/orangewood-co/robogpt.git
git clone -b dev https://github.com/orangewood-co/robogpt_tools.git
git clone -b dev https://github.com/orangewood-co/robogpt_perception.git
git clone -b main https://bitbucket.org/owl-dev/orangewood_simstack_ros2.git orangewood_simstack
git clone -b main https://github.com/orangewood-co/robogpt-agentic-workflow.git
git clone -b master https://github.com/orangewood-co/prompt_validator.git
git clone -b main https://github.com/orangewood-co/robogpt_external_hardware.git
git clone -b main https://github.com/orangewood-co/robogpt_node_stack.git 

# Third-party packages
git clone -b main https://github.com/orbbec/OrbbecSDK_ROS2.git OrbbecSDK_ROS2
git clone -b main https://github.com/AndrejOrsula/pymoveit2.git
```

#### Step 3: Install Dependencies

```bash
# Navigate to workspace root
cd ~/orangewood_ws

# Install ROS dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Install Python dependencies
pip install -r src/robogpt/install/setup/requirements.txt

# Install third party dependencies
sudo . /src/robogpt/install/.install_dependencies.sh

# Sourcing platfrom shell function 
echo "source /home/${USER}/orangewood_ws/src/robogpt/core_stack/robogpt_startup/startup/startup.sh" >> ~/.bashrc

```

#### Step 4: Build the Workspace

```bash
# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
echo "source ~/orangewood_ws/install/setup.bash" >> ~/.bashrc
```

#### Step 5: Ngrok Setup

Steps to setup ngrok

        1. Go to  https://ngrok.com/ and Login/Sign up

        2. On Top left Go to Setup and Installation 

        3. You will get commands to install the ngrok. Run those command in your terminal to install ngrok cli

        4. Then Run the authtoken command below the Installation commands 

        5. Go to Domains on Top left. You will see a Table with ID and Domain ending with '.ngrok-free.app'. Copy that domain

        6. echo "export NGROK_URL=your_url_here" >> ~/.bashrc


### Method 2: Automated Installation

#### Quick Setup with Script

#### Working on it 

### Environment Configuration

Run ``` key_setup.sh ``` in your workspace for required environment variables:

```bash
. /home/${USER}/orangewood_ws/src/robogpt/install/setup/key_setup.sh
```

### Troubleshooting

**Common Issues:**

1. **Missing dependencies**: Run `rosdep install --from-paths src --ignore-src -r -y` again
2. **Build failures**: Check if all repositories were cloned correctly
3. **Python import errors**: Ensure requirements.txt was installed: `pip install -r src/robogpt/robogpt_startup/setup/requirements.txt`
4. **Environment variables**: Source the workspace: `source ~/orangewood_ws/install/setup.bash`

---

## RoboGPT v4 (Development)

Latest version of robogpt with all the new tech and some wow type features also known as cutting-edge features.


| Repository | Description | Main Branch | Status | Owner |
|:----------:|:------------|:----------:|:------:|:--------:|
| [**robogpt**](https://github.com/orangewood-co/robogpt.git) | Contains core packages like agents, hardware stack | `dev` | [![Status](https://img.shields.io/badge/status-in_development-yellow.svg)]() | Yuvraj |
| [**robogpt_tools**](https://github.com/orangewood-co/robogpt_tools.git) | Contains Applications/skills and Robot configurations | `dev` | [![Status](https://img.shields.io/badge/status-in_development-yellow.svg)]() | Jiya |
| [**robogpt_perception**](https://github.com/orangewood-co/robogpt_perception.git) | Contains Perpection packages, Vision models and Third party camera pkgs | `dev` | [![Status](https://img.shields.io/badge/status-in_development-yellow.svg)]() | Manan |
| [**orangewood_simstack**](https://bitbucket.org/owl-dev/orangewood_simstack_ros2/src/main/) | Contains All the Simluation releated packages | `main` | [![Status](https://img.shields.io/badge/status-active-success.svg)]() | Rudranil |
| [**robogpt--agentic-workflow**](https://github.com/orangewood-co/robogpt-agentic-workflow.git) | Contains code for robogpt agentic approach for prompts| `main` | [![Status](https://img.shields.io/badge/status-in_development-yellow.svg)]() | Tushar |
| [**prompt_validator**](https://github.com/orangewood-co/prompt_validator.git) | Contains tools for agentic workflow | `master` | [![Status](https://img.shields.io/badge/status-in_development-yellow.svg)]() | Yuvraj |
| [**robogpt_external_hardware**](https://github.com/orangewood-co/robogpt_external_hardware.git) | Contains packages and codebase for external hardware like PLCs, sensors etc| `main` | [![Status](https://img.shields.io/badge/status-in_development-yellow.svg)]() | Yuvraj |
| [**robogpt node stack**](https://github.com/orangewood-co/robogpt_node_stack.git) | Contains core packages releated block programming| `main` | [![Status](https://img.shields.io/badge/status-in_development-yellow.svg)]() | Yuvraj |




### Third Party Pakcages
These should be installed along the other Robogpt v4 packages/code-base in the same folder


| Repository | Description |
|:----------:|:--------------------|
| [**OrbbecROS2**](https://github.com/orbbec/OrbbecSDK_ROS2.git) | SDK/ROS2 package for orbbec camera |
| [**pymoveit2**](https://github.com/AndrejOrsula/pymoveit2.git) | SDK for simulation/Moveit2 in python|


---
## Additional Repositories/Links

| Repository | Description |
|:----------:|:--------------------|
| [**Owl Robot Client**](https://github.com/orangewood-co/owl_robot_client_SDK.git) | SDK for OWL Robots|
| [**Owl GUI**](https://github.com/orangewood-co/OWL_GUI.git) | Application GUI for OWl robots|
| [**Robogpt Skills Guide**](https://github.com/orangewood-co/robogpt_skills_doc) | Complete info about skills in robogpt|
| [**V4 File Structure**](https://miro.com/welcomeonboard/NGhEUGxwWUlnYVIzckRpTW5hTjJVZkgwNDVWdGRFakszejhKUlZRdUt4SFhFcnE1Ui9sVXVRdTdLbXNjQU5TZFJyQUdVT1VaSVhueHFlbDJIcWRtY3NacDZJdUJDSmpPUmhZd1E5SmRoVWx5a2kzNmtScmQ0S09nVUcrVFkyMmR0R2lncW1vRmFBVnlLcVJzTmdFdlNRPT0hdjE=?share_link_id=888135151033) | File structure chart of V4 Codebase|

