
# 🤖 RoboGPT Documentation

**Your comprehensive guide to all RoboGPT software versions and repositories**


## 📋 Overview

This central hub contains all RoboGPT versions repo links, providing easy access to each repository's information, and development resources.

---

## 🚀 Installation Guide

### Read this first (no prior experience required)

You can install RoboGPT in two ways:

- Method 1 — Docker (recommended for beginners): No ROS install on your PC. Everything runs inside a prebuilt container image.
- Method 2 — Manual/Local (advanced): Install ROS and all tools on your PC yourself.

What you'll need (Method 1 — Docker):

- Ubuntu 22.04 (recommended) or 20.04
- Internet connection, ~20–30 GB free disk space
- Basic terminal access (we show every command to copy-paste)

How long this takes:

- First-time Docker build and repository clone can take 15–45 minutes depending on your internet speed.

Mini glossary:

- Image: A prebuilt recipe containing software. Think “frozen app snapshot.”
- Container: A running instance of an image. Think “app is running now.”
- Volume: A shared folder between your PC and the container so your code persists.
- Workspace: Your RoboGPT source code folder at `~/orangewood_ws` inside the container.

---

## Prerequisites for a fresh Ubuntu 22.04

Follow these steps if your computer is brand new or missing required tools.

Notes:
- If you use Method 1 (Docker), installing ROS 2 on the host is optional.
- If you use Method 2 (Manual), installing ROS 2 on the host is required.

#### A) Install basic tools

```bash
sudo apt update
sudo apt install -y git curl gnupg ca-certificates lsb-release software-properties-common
```

#### B) Install Docker Engine (required for Method 1)

1) Add Docker’s official repository

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```

2) Install Docker Engine + Compose plugin

```bash
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

3) Enable non-root Docker usage (log out/in after this)

```bash
sudo usermod -aG docker $USER
newgrp docker
```

4) Verify Docker

```bash
docker --version
docker run --rm hello-world
```

Reference: https://docs.docker.com/engine/install/ubuntu/

#### C) Install ROS 2 Humble (required for Method 2; optional for Method 1)

1) Set locale (recommended)

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

2) Add ROS 2 apt repository

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release software-properties-common
sudo add-apt-repository universe -y || true
sudo mkdir -p /etc/apt/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

3) Install ROS 2 Humble

Choose one (desktop is larger, includes GUI tools; ros-base is smaller):

```bash
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
# or smaller:
# sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions python3-rosdep
```

4) Initialize rosdep and source ROS 2

```bash
sudo rosdep init || true
rosdep update
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

Reference: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

## Method 1: Docker-based Installation (Recommended)

This method uses the RoboGPT Docker image and a mounted source volume for v4 development. It isolates dependencies and works without ROS 2 installed on the host.

#### Step 1: Clone the Docker setup repository

```bash
git clone git@github.com:orangewood-co/robogpt_image.git
cd ~/robogpt_image
git checkout v0.2-dev
git pull
```

#### Option A: Single-line setup

```bash
cd ~/robogpt_image/setup
sudo chmod +x setup.sh
./setup.sh
```

#### Option B: Step-by-step setup

1) Install Docker and host prerequisites (runs an automated installer)

```bash
cd ~/robogpt_image/setup
./setup_host.sh
```

What this does: installs Docker and required host packages for running the container.

Verify Docker:

```bash
docker --version
```

If you see a permission error with Docker:

- Log out and log back in (or reboot) to apply group changes
- Then try `docker ps` again; if it still fails, prefix with `sudo` as a temporary workaround

2) Build the RoboGPT Docker image

```bash
cd ~/robogpt_image/build
./build_app_image.sh
```

When prompted, choose 1 for a non‑cached build (recommended if dependencies changed).

Verify the image exists:

```bash
docker images --all | grep robogpt
```

3) Prepare the v4 development volume and clone all repos

```bash
cd ~/robogpt_image
mkdir -p v4
cp build/clone_repos.sh v4/
cd ~/robogpt_image/v4
./clone_repos.sh
```

This clones the RoboGPT v4 codebase and third-party packages into `~/robogpt_image/v4`.

4) Enable RoboGPT helper commands

```bash
echo "source '$HOME/robogpt_image/setup/.robogpt_functions.sh'" >> "$HOME/.bashrc"
source ~/.bashrc
```

Verify helpers are available:

```bash
robogpt -h
```

#### Step 2: Start the dev container

```bash
# Stop any previous container (safe to run even if none)
robogpt stop

# Start dev container with volume mounted and env sourced
robogpt dev

# To open multiple terminals once 'robogpt dev' is running, use:
robogpt tab
```

This opens a shell in the container with your `v4` sources mounted and will build the workspace on first run.

#### Step 3: Inside the container — set up dependencies with rosdep

Run these commands in the container shell:

```bash
# Initialize and update rosdep (first time only)
sudo rosdep init || true
rosdep update

# Install ROS package dependencies from sources
cd ~/orangewood_ws
rosdep install --from-paths src --ignore-src -r -y

# Python dependencies from the RoboGPT repo
pip3 install -r src/robogpt/install/setup/requirements.txt

# Any additional system dependencies scripted by the repo
sudo bash src/robogpt/install/.install_dependencies.sh

# Build the workspace (dev brings this up, but you can rebuild manually anytime)
colcon build --symlink-install
source install/setup.bash
echo "source ~/orangewood_ws/install/setup.bash" >> ~/.bashrc
```

Verify build:

```bash
ros2 --version
colcon list | head -n 20
```

#### Step 4: Optional environment setup in the container

```bash
# Export platform/runtime variables required by RoboGPT
. src/robogpt/install/setup/key_setup.sh
```

#### Step 5: Ngrok Setup (Outside Docker, on host)

Steps to setup ngrok

        1. Go to  https://ngrok.com/ and Login/Sign up

        2. On Top left Go to Setup and Installation 

        3. You will get commands to install the ngrok. Run those command in your terminal to install ngrok cli

        4. Then Run the authtoken command below the Installation commands 

        5. Go to Domains on Top left. You will see a Table with ID and Domain ending with '.ngrok-free.app'. Copy that domain

        6. Run this command -> echo "export NGROK_URL=your_url_here" >> ~/.bashrc

#### Robogpt helper commands (host)

- `robogpt dev` — Start container with volume mounted and env sourced; builds workspace.
- `robogpt tab` — Open an additional terminal attached to the running container.
- `robogpt stop` — Stop the running RoboGPT container.

#### Docker tips

```bash
# List containers and images
docker ps -a
docker images --all

# Stop/remove containers and images (careful: destructive)
docker stop $(docker ps -q)
docker rm $(docker ps -aq)
docker rmi $(docker images -q)

# Prune everything including volumes
docker system prune -a --volumes

# Save container changes as a new image
docker commit robogpt robogpt
```

> NOTE: If you run robogpt inside docker you have to run the ngrok command outside docker on the port 8000, the docker will be able to access it via the host network because the port is exposed.

---

## Method 2: Manual/Local Installation

Choose this if you already know ROS 2 and prefer a local install without Docker.

### Option 1

#### Step 1: Create Workspace Structure

```bash
# Create the main workspace directory
mkdir -p ~/orangewood_ws/src
cd ~/orangewood_ws/src
```

#### Step 2: Clone All Required Repositories

```bash
# Core RoboGPT v4 repositories
git clone -b main https://github.com/orangewood-co/robogpt.git
git clone -b main https://github.com/orangewood-co/robogpt_tools.git
git clone -b main https://github.com/orangewood-co/robogpt_perception.git
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
pip3 install -r src/robogpt/install/setup/requirements.txt

# Install third party dependencies
sudo bash src/robogpt/install/.install_dependencies.sh

# Sourcing platfrom shell function 
echo "source /home/${USER}/orangewood_ws/src/robogpt/core_stack/robogpt_startup/startup/startup.sh" >> ~/.bashrc

```

Verify dependencies:

```bash
rosdep check --from-paths src || true
```

#### Step 4: Build the Workspace

```bash
# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
echo "source ~/orangewood_ws/install/setup.bash" >> ~/.bashrc
```

Verify build:

```bash
ros2 --version
colcon list | head -n 20
```

#### Step 5: Ngrok Setup

Steps to setup ngrok

        1. Go to  https://ngrok.com/ and Login/Sign up

        2. On Top left Go to Setup and Installation 

        3. You will get commands to install the ngrok. Run those command in your terminal to install ngrok cli

        4. Then Run the authtoken command below the Installation commands 

        5. Go to Domains on Top left. You will see a Table with ID and Domain ending with '.ngrok-free.app'. Copy that domain

        6. Run this command -> echo "export NGROK_URL=your_url_here" >> ~/.bashrc


### Option 2: Automated Installation
This section has moved to Method 1 (Docker-based Installation). See above for a fully automated setup using the Docker image and helper scripts.

### Environment Configuration

Run ``` key_setup.sh ``` in your workspace for required environment variables:

```bash
. /home/${USER}/orangewood_ws/src/robogpt/install/setup/key_setup.sh
```

### Troubleshooting

**Common Issues:**

1. **Missing dependencies**: Run `rosdep install --from-paths src --ignore-src -r -y` again
2. **Build failures**: Check if all repositories were cloned correctly
3. **Python import errors**: Ensure requirements were installed: `pip3 install -r src/robogpt/install/setup/requirements.txt`
4. **Environment variables**: Source the workspace: `source ~/orangewood_ws/install/setup.bash`
5. **Inside Docker**: If `rosdep update` fails, make sure you ran `sudo rosdep init` once inside the container and you have internet access.

Additional tips for beginners:

- Docker permission denied: log out/in or reboot after installing Docker. Then try `docker ps` again. As a temporary workaround, prefix commands with `sudo`.
- Slow downloads or name resolution issues: try `rosdep update --include-eol-distros` or re-run later; ensure your internet/DNS is stable.
- Reclaim space: `docker system prune -a --volumes` will remove cached images/containers (destructive; use with care).
- Re-run the dev container build: `robogpt stop` then `robogpt dev`.

---

## Quick glossary

- Docker: A way to package and run software so it works the same everywhere.
- Image: The packaged software template. You build it once.
- Container: A running copy of the image where you do your work.
- Volume: A folder shared between your PC and the container so your files persist.
- ROS 2: A robotics framework used by RoboGPT.

---

## RoboGPT v4 (Development)

Latest version of robogpt with all the new tech and some wow type features also known as cutting-edge features.


| Repository | Description | Main Branch | Status | Owner |
|:----------:|:------------|:----------:|:------:|:--------:|
| [**robogpt**](https://github.com/orangewood-co/robogpt.git) | Contains core packages like agents, hardware stack | `main` | [![Status](https://img.shields.io/badge/status-in_development-yellow.svg)]() | Yuvraj |
| [**robogpt_tools**](https://github.com/orangewood-co/robogpt_tools.git) | Contains Applications/skills and Robot configurations | `main` | [![Status](https://img.shields.io/badge/status-in_development-yellow.svg)]() | Jiya |
| [**robogpt_perception**](https://github.com/orangewood-co/robogpt_perception.git) | Contains Perpection packages, Vision models and Third party camera pkgs | `main` | [![Status](https://img.shields.io/badge/status-in_development-yellow.svg)]() | Manan |
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

