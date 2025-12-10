# Quickstart Guide: Module 02 - Digital Twin for Robotic Systems

## Prerequisites

Before starting this module, ensure your system meets the following requirements:

### System Requirements
- Operating System: Ubuntu 20.04 or 22.04 (for Gazebo/ROS components)
- RAM: 8GB minimum, 16GB recommended
- Storage: 10GB free space
- GPU: Dedicated graphics card recommended for Unity visualization

### Software Requirements
- ROS 2 Humble Hawksbill or Rolling (with Gazebo Classic)
- Unity LTS version (2022.3.x or later)
- Unity Robotics Hub package
- Python 3.8 or higher
- Git
- Docker (optional, for containerized environments)

## Installation

### 1. ROS 2 and Gazebo Setup

1. Install ROS 2 Humble following the official installation guide:
   ```bash
   # Add ROS 2 repository and key
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
   echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list
   sudo apt update
   sudo apt install -y ros-humble-desktop ros-humble-ros-gz
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

### 2. Unity Setup

1. Download and install Unity Hub from the Unity website
2. Install Unity LTS version (2022.3.x or later) through Unity Hub
3. Create or sign in to a Unity ID
4. Import the Unity Robotics Hub package from the Package Manager

### 3. Clone the Course Repository

```bash
git clone https://github.com/[your-org]/ai-native-robotic-education.git
cd ai-native-robotic-education
```

### 4. Backend Setup

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Create a virtual environment:
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   # If requirements.txt doesn't exist, install required packages:
   pip install rclpy numpy open3d transforms3d
   ```

## Running the Digital Twin

### 1. Launch Gazebo Simulation

1. Open a new terminal and source ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   cd /path/to/ai-native-robotic-education
   ```

2. Launch the Gazebo simulation with ROS bridge:
   ```bash
   ros2 launch digital_twin_gazebo demo.launch.py
   ```

3. You should see the Gazebo simulation window with your robot model loaded.

### 2. Launch Unity Visualization

1. Open Unity Hub and create a new project or open an existing one
2. Import the Unity assets from `unity/digital_twin_visualization/Assets`
3. Open the main scene (`Scenes/MainScene.unity`)
4. In the Unity Editor, run the scene. The robot visualization should connect to the ROS network and show the same state as Gazebo.

### 3. Run Perception Pipeline

1. In another terminal, source ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   cd /path/to/ai-native-robotic-education
   ```

2. Run a simple perception node:
   ```bash
   ros2 run digital_twin_perception lidar_obstacle_detector
   ```

## Basic Examples

### Example 1: Simple Robot Movement

1. Launch the Gazebo simulation: `ros2 launch digital_twin_gazebo turtlebot3_world.launch.py`
2. In another terminal, send movement commands: `ros2 run turtlesim turtle_teleop_key`
3. Observe the robot moving in both Gazebo and Unity visualizations

### Example 2: Sensor Data Visualization

1. Launch the simulation with sensors enabled: `ros2 launch digital_twin_gazebo sensor_demo.launch.py`
2. View sensor data in RViz2: `ros2 run rviz2 rviz2`
3. Configure RViz2 to visualize LiDAR scans, depth camera images, and IMU data
4. Check Unity visualization for synchronized sensor representations

### Example 3: Parameter Tuning

1. Run the physics parameter tuning script:
   ```bash
   ros2 run digital_twin_gazebo tune_friction_params --robot_name turtlebot3 --target_stop_distance 0.5
   ```
2. Observe how parameter changes affect robot behavior in both simulators

## Troubleshooting

### Common Issues

1. **ROS Network Connection Issues**:
   - Ensure ROS_DOMAIN_ID is the same in all terminals: `export ROS_DOMAIN_ID=0`
   - Check firewall settings to allow ROS communication

2. **Unity-ROS Bridge Connection**:
   - Verify that Unity Robotics Hub is properly configured
   - Check that the ROS IP address in Unity matches the ROS machine's IP
   - Ensure both systems are on the same network if running on separate machines

3. **Gazebo Simulation Lag**:
   - Check system resources (CPU, RAM, GPU)
   - Reduce simulation complexity or quality settings
   - Ensure proper GPU drivers are installed

4. **Python Import Errors**:
   - Activate the correct Python virtual environment
   - Install missing packages using pip

### Verification Commands

1. Check that ROS nodes are running:
   ```bash
   ros2 node list
   ```

2. Check available topics:
   ```bash
   ros2 topic list
   ```

3. Verify sensor data is being published:
   ```bash
   ros2 topic echo /robot/lidar_scan sensor_msgs/msg/LaserScan
   ```

## Next Steps

After successfully completing the quickstart:

1. Work through Week 1 labs to understand Gazebo physics and sensor configuration
2. Proceed to Week 2 to learn Unity visualization and bridging
3. Complete Week 3 for perception pipeline integration and capstone project