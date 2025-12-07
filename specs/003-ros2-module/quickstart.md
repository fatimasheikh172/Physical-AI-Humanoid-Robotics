# Quickstart Guide: Module 1 - The Robotic Nervous System (ROS 2)

**Feature**: Module 1 - The Robotic Nervous System (ROS 2)
**Date**: 2025-12-07
**Guide Version**: 1.0

## Overview

This quickstart guide will help you get Module 1 of the Physical AI & Humanoid Robotics textbook up and running for development. This module focuses on ROS 2 fundamentals: architecture, communication patterns, and how to bridge AI agents with ROS controllers.

## Prerequisites

Before starting, ensure you have the following installed:

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS (primary development environment)
- **Minimum Hardware**: 8GB RAM, 50GB free disk space, multi-core processor
- **Additional Tools**: Git, Python 3.10+ (system Python), Docker (optional for isolated environments)

### ROS 2 Specific Requirements
- **ROS 2 Distribution**: Humble Hawksbill (LTS) - this is critical for compatibility
- **Python**: 3.10 (system Python for ROS 2 compatibility)
- **Build Tools**: colcon (for building ROS 2 packages), vcs (for workspace management)

## Setting Up the Development Environment

### 1. Install Ubuntu 22.04 (if not already installed)

If you're not already on Ubuntu 22.04, you have two options:

#### Option A: Dual Boot or Virtual Machine
- Download Ubuntu 22.04 LTS from ubuntu.com
- Install alongside or replace your current OS (dual boot)
- Or create a VM with at least 4 cores and 8GB RAM

#### Option B: Using Windows Subsystem for Linux (WSL2)
```bash
# Enable WSL2 on Windows
wsl --install -d Ubuntu-22.04
```

### 2. Install ROS 2 Humble Hawksbill

```bash
# Add the ROS 2 repository and key
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrongs/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install ROS 2
sudo apt update
sudo apt install -y ros-humble-desktop
```

### 3. Install ROS 2 Python tools and development packages

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update

# Install additional Python dependencies
sudo apt install -y python3-pip python3-rosinstall
```

### 4. Setup ROS 2 environment

```bash
# Add ROS 2 setup to your bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5. Clone and setup the textbook repository

```bash
# Create ROS 2 workspace for the textbook
mkdir -p ~/ros2_textbook_ws/src
cd ~/ros2_textbook_ws/src

# Clone the repository
git clone https://github.com/your-org/physical-ai-textbook.git

# Navigate to the ROS 2 module labs
cd ~/ros2_textbook_ws/src/physical-ai-textbook/ros2_labs
```

### 6. Install additional Python dependencies for the textbook

```bash
# Create a virtual environment for the textbook
cd ~/ros2_textbook_ws
python3 -m venv textbook_venv
source textbook_venv/bin/activate

# Install Python dependencies for AI integration and textbook tools
pip install rclpy openai python-jose[cryptography] passlib[bcrypt] qdrant-client

# Install dependencies for the textbook platform
pip install -r ~/ros2_textbook_ws/src/physical-ai-textbook/backend/requirements.txt
```

## Building and Running Module 1 Labs

### 1. Build the Module 1 packages

```bash
cd ~/ros2_textbook_ws

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Build all packages in the workspace
colcon build --packages-select week1_foundation week2_communication week3_ai_bridge

# Source the workspace setup file
source install/setup.bash
```

### 2. Run Week 1 - ROS 2 Foundations Lab

```bash
# Make sure you're in the workspace and have sourced both ROS 2 and workspace
cd ~/ros2_textbook_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run the Week 1 publisher node
ros2 run week1_foundation simple_publisher

# In another terminal, run the subscriber node
cd ~/ros2_textbook_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run week1_foundation simple_subscriber
```

### 3. Run Week 2 - Communication Layer Lab

```bash
# Run the topic-based robot control
ros2 run week2_communication robot_motion_topic

# Run the service-based robot control
ros2 run week2_communication robot_control_service

# Run the action-based robot navigation
ros2 run week2_communication robot_navigation_action
```

### 4. Run Week 3 - AI + ROS Bridge Lab

```bash
# Run the AI decision node
ros2 run week3_ai_bridge ai_decision_node

# Run the robot controller that receives AI commands
ros2 run week3_ai_bridge robot_controller
```

## Running the Textbook Interface

### 1. Start the Backend API

```bash
# Activate the textbook virtual environment
cd ~/ros2_textbook_ws
source textbook_venv/bin/activate

# Navigate to backend directory and start the API server
cd ~/ros2_textbook_ws/src/physical-ai-textbook/backend
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

### 2. Start the Frontend Application

```bash
# In a new terminal, navigate to the project root
cd ~/ros2_textbook_ws/src/physical-ai-textbook

# Install Node.js dependencies if not already installed
npm install

# Start the Docusaurus-based textbook interface
npm start
```

The textbook interface will be available at `http://localhost:3000`.

## Key ROS 2 Commands for Module 1

### Essential ROS 2 Commands

```bash
# List all active nodes
ros2 node list

# List all active topics
ros2 topic list

# Echo messages on a specific topic (replace <topic_name>)
ros2 topic echo /<topic_name>

# List all services
ros2 service list

# Get information about a specific node
ros2 node info <node_name>

# Run a node from a package
ros2 run <package_name> <executable_name>

# Launch a complete system with launch files
ros2 launch <package_name> <launch_file>.py
```

### Module 1 Specific Commands

```bash
# Week 1: Check your simple publisher/subscriber nodes
ros2 run week1_foundation simple_publisher
ros2 run week1_foundation simple_subscriber

# Week 2: Test communication patterns
ros2 run week2_communication robot_motion_topic
ros2 service call /robot_control <service_type> <request_data>
ros2 action send_goal /robot_navigation <action_type> <goal_data>

# Week 3: Test AI-ROS integration
ros2 run week3_ai_bridge ai_decision_node
ros2 run week3_ai_bridge robot_controller
```

## Running Simulations (Gazebo and RViz)

### 1. Install Gazebo and RViz

```bash
sudo apt install -y ros-humble-gazebo-* ros-humble-rviz2 ros-humble-robot-state-publisher ros-humble-joint-state-publisher
```

### 2. Launch the Week 3 Simulation Environment

```bash
# Source the environment
cd ~/ros2_textbook_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch the simulation with URDF robot model
ros2 launch week3_ai_bridge simulation_launch.py
```

### 3. Visualize in RViz

After launching the simulation, open a new terminal and run:

```bash
# Source the environment
cd ~/ros2_textbook_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start RViz for visualization
rviz2
```

In RViz, add displays for:
- RobotModel (set robot description to `/robot_description`)
- LaserScan or PointCloud for sensor data
- TF for transformation frames
- Topic displays for custom messages

## Common Issues and Troubleshooting

### Environment Setup Issues

1. **ROS 2 command not found**:
   ```bash
   # Ensure ROS 2 is sourced
   source /opt/ros/humble/setup.bash
   ```

2. **Package build failures**:
   ```bash
   # Clean build and try again
   rm -rf build/ install/ log/
   colcon build
   ```

3. **Python import errors**:
   ```bash
   # Ensure you're using the correct Python and virtual environment
   which python3  # Should point to system Python for ROS 2
   source ~/ros2_textbook_ws/textbook_venv/bin/activate  # For AI integration
   ```

### Troubleshooting AI Tutor Integration

1. **AI tutor not responding**: Check that your API key is properly configured in the backend environment variables.
2. **ROS 2 specific queries not handled correctly**: Ensure the AI tutor's knowledge base includes ROS 2 documentation and error patterns.

### Simulation Issues
1. **RViz crashes or displays errors**: This often happens with graphics drivers; try running with software rendering.
2. **Gazebo models not loading**: Check that URDF files are properly formatted and paths are correct.

## Development Workflow for Module 1

### Creating New Content

1. **Add new ROS 2 nodes** in the appropriate week directory:
   - Create Python files in `~/ros2_textbook_ws/src/physical-ai-textbook/ros2_labs/weekX_*/src/`
   - Follow ROS 2 node structure with proper lifecycle management

2. **Add new textbook content**:
   - Add content pages in the `/docs/modules/01-ros2-fundamentals/` directory
   - Update sidebar navigation in `sidebars.js`

3. **Add new assessments**:
   - Create assessment questions in the database or through the admin interface
   - Link to appropriate module and week

### Testing Module Components

1. **Test ROS 2 nodes locally**:
   ```bash
   cd ~/ros2_textbook_ws
   source install/setup.bash
   ros2 run <package_name> <node_name>
   ```

2. **Test API endpoints**:
   - Use the built-in API documentation at `http://localhost:8000/docs`
   - Verify progress tracking, assessments, and AI tutor endpoints

3. **Test end-to-end functionality**:
   - Go through each week's content as a student would
   - Verify the flow from theory to lab to assessment

## Next Steps

1. Complete the Week 1 content: ROS 2 Foundations
2. Proceed to Week 2: Communication Layer
3. Complete Week 3: AI-ROS Bridge and Humanoid Modeling
4. Take the final module assessment
5. Build the integrated project connecting AI agents to humanoid robot controls

For detailed information about each week's content, API specifications, and advanced tutorials, refer to:
- The textbook content at `http://localhost:3000`
- The API documentation at `http://localhost:8000/docs`
- Individual README files in each week's lab directory