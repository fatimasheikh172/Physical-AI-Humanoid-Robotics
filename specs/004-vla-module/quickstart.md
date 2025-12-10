# Quickstart Guide: Module 4 - Vision-Language-Action (VLA)

## Overview

This quickstart guide will help you set up and run the Vision-Language-Action (VLA) module. This system integrates voice recognition, LLM-based planning, and robotic action execution to enable natural human-robot interaction.

## Prerequisites

### System Requirements
- Operating System: Ubuntu 22.04 LTS (for ROS 2 Humble compatibility)
- RAM: 8GB minimum, 16GB recommended
- Storage: 5GB free space
- Internet connection (for OpenAI Whisper and LLM APIs)
- Microphone for voice input
- Camera for computer vision (optional for basic functionality)

### Software Requirements
- ROS 2 Humble Hawksbill
- Python 3.10 or higher
- OpenAI API key
- (Optional) Anthropic API key for Claude integration
- Pip package manager

## Installation

### 1. ROS 2 Setup

1. Install ROS 2 Humble following the official installation guide:
   ```bash
   # Add ROS 2 repository and key
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
   echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list
   sudo apt update
   sudo apt install -y ros-humble-desktop ros-humble-ros-dev-tools
   ```

2. Install additional dependencies:
   ```bash
   sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
   sudo rosdep init
   rosdep update
   ```

3. Source ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

### 2. Repository Setup

1. Clone the course repository:
   ```bash
   git clone https://github.com/[your-org]/ai-native-robotic-education.git
   cd ai-native-robotic-education
   ```

2. Navigate to the VLA module directory:
   ```bash
   cd backend/vla_module
   ```

### 3. Python Environment Setup

1. Create a virtual environment:
   ```bash
   python3 -m venv vla_env
   source vla_env/bin/activate  # On Windows: vla_env\Scripts\activate
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   # If requirements.txt doesn't exist, install required packages:
   pip install rclpy openai opencv-python torch torchvision torchaudio anthropic
   ```

### 4. API Keys Configuration

1. Create a `.env` file in the `vla_module` directory:
   ```bash
   touch .env
   ```

2. Add your API keys to the `.env` file:
   ```
   OPENAI_API_KEY=your_openai_api_key_here
   ANTHROPIC_API_KEY=your_anthropic_api_key_here  # Optional
   ```

## Running the VLA Module

### 1. Launch the Complete System

1. Open a new terminal and source ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   cd /path/to/ai-native-robotic-education
   source backend/vla_module/vla_env/bin/activate
   ```

2. Launch the VLA module:
   ```bash
   ros2 launch vla_module vla_module.launch.py
   ```

3. The system will start with the following components:
   - Voice recognition node
   - LLM planning node
   - Action execution node
   - Vision perception node (if camera is available)

### 2. Voice Command Interface

1. Once the system is running, you can issue voice commands directly to the robot.

2. The system will process your command through the pipeline:
   - Voice recognition → LLM planning → Action sequence generation → Robot execution

### 3. CLI Interface

Alternatively, you can use the CLI to test the system:

1. In a new terminal:
   ```bash
   source /opt/ros/humble/setup.bash
   cd /path/to/ai-native-robotic-education
   source backend/vla_module/vla_env/bin/activate
   ```

2. Run the CLI interface:
   ```bash
   python -m vla_module.cli.vla_cli --command "Move to the kitchen and find the red cup"
   ```

## Basic Examples

### Example 1: Simple Navigation Command

1. Say: "Please move to the living room"
2. The system will:
   - Recognize the voice command
   - Plan a path to the living room
   - Execute the navigation action
3. The robot will move to the living room and report completion

### Example 2: Object Detection Command

1. Say: "What objects do you see on the table?"
2. The system will:
   - Activate the vision system
   - Detect objects on the table
   - Generate a response describing the objects
3. The robot will speak or print the list of detected objects

### Example 3: Complex Manipulation Command

1. Say: "Pick up the blue marker from the desk and place it in the drawer"
2. The system will:
   - Plan a sequence: navigate to desk → detect blue marker → pick up → navigate to drawer → place
   - Execute each action in sequence
3. The robot will complete the multi-step manipulation task

## Testing the Components

### 1. Test Voice Recognition

1. Run the voice recognition test:
   ```bash
   ros2 run vla_module test_voice_recognition
   ```

### 2. Test LLM Planning

1. Run the planning test with a text command:
   ```bash
   python -m vla_module.llm_planning.test_planning --command "Clean the room"
   ```

### 3. Test Action Execution

1. Run action execution tests:
   ```bash
   ros2 run vla_module test_action_execution
   ```

## Troubleshooting

### Common Issues

1. **API Connection Errors**:
   - Verify your API keys are correctly set in the `.env` file
   - Check your internet connection
   - Ensure your API key has sufficient quota

2. **ROS 2 Communication Errors**:
   - Check that all nodes are running: `ros2 node list`
   - Verify topic connections: `ros2 topic list`
   - Ensure ROS_DOMAIN_ID is consistent across terminals: `export ROS_DOMAIN_ID=0`

3. **Audio Input Issues**:
   - Test your microphone with: `arecord -d 3 test.wav`
   - Check audio device permissions
   - Verify audio input levels in system settings

4. **Vision Processing Errors**:
   - Ensure camera is properly connected and detected
   - Check camera permissions
   - Verify OpenCV installation

### Verification Commands

1. Check that all VLA nodes are running:
   ```bash
   ros2 node list | grep vla
   ```

2. Check VLA-specific topics:
   ```bash
   ros2 topic list | grep vla
   ```

3. Test voice command processing:
   ```bash
   ros2 run vla_module test_voice_to_action --text "Move forward 1 meter"
   ```

## Next Steps

After successfully completing this quickstart:

1. Work through the full VLA module curriculum
2. Experiment with different voice commands
3. Customize the LLM planning prompts for your specific robot
4. Implement the capstone project: "The Autonomous Humanoid"