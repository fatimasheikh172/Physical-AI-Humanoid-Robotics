# Digital Twin System for Robotic Simulation

This repository contains a complete digital twin system for robotic simulation that combines Gazebo physics simulation, ROS 2 communication, Unity visualization, and perception processing. This system enables realistic simulation of robots with accurate physics, sensors, and real-time visualization.

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Prerequisites](#prerequisites)
4. [Installation](#installation)
5. [Usage](#usage)
6. [Components](#components)
7. [Configuration](#configuration)
8. [Troubleshooting](#troubleshooting)
9. [Testing](#testing)
10. [Performance](#performance)
11. [Extending the System](#extending-the-system)

## Overview

The digital twin system consists of:

- **Gazebo Physics Simulation**: Accurate physics simulation with configurable parameters
- **Sensor Simulation**: LiDAR, depth camera, and IMU simulation with realistic noise models
- **ROS 2 Integration**: Standard ROS 2 interfaces for communication
- **Unity Visualization**: Real-time visualization with high-fidelity rendering
- **Perception Pipeline**: Processing nodes for sensor data analysis
- **Parameter Tuning**: Tools for optimizing simulation fidelity

## Architecture

```text
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Unity        │    │     ROS 2        │    │   Gazebo       │
│ Visualization  │◄──►│   Communication  │◄──►│   Physics      │
│                │    │     Bridge       │    │   Simulation   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                             │
                    ┌──────────────────┐
                    │  Perception      │
                    │    Pipeline      │
                    └──────────────────┘
```

The system uses a bridge architecture where Gazebo handles physics and sensor simulation, Unity provides visualization, and ROS 2 facilitates communication between components.

## Prerequisites

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
- Docker (optional)

## Installation

### 1. ROS 2 and Gazebo Setup

1. Install ROS 2 Humble:
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

### 3. Clone the Repository

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
   pip install rclpy numpy scipy transforms3d matplotlib pyyaml opencv-python cv-bridge
   ```

## Usage

### 1. Launch Complete Digital Twin System

The easiest way to start the entire system is to run the main launch file:

```bash
# Terminal 1: Source ROS 2 and launch the complete system
source /opt/ros/humble/setup.bash
cd backend
source venv/bin/activate
ros2 launch digital_twin complete_digital_twin.launch.py
```

### 2. Launch Components Separately

For more control over the system, you can launch components separately:

#### Gazebo Simulation
```bash
source /opt/ros/humble/setup.bash
cd backend
source venv/bin/activate
# Launch Gazebo with robot and sensors
ros2 launch digital_twin_gazebo demo.launch.py
```

#### Perception Nodes
```bash
source /opt/ros/humble/setup.bash
cd backend
source venv/bin/activate
# Launch all perception nodes
ros2 run digital_twin_perception perception_demo
# Or run specific nodes:
ros2 run digital_twin_perception imu_analysis_node
ros2 run digital_twin_perception sensor_fidelity_analysis
```

#### Unity Visualization
1. Open Unity Hub and launch Unity
2. Open the project at `unity/digital_twin_visualization/`
3. Make sure the ROS-TCP connection settings match your ROS 2 setup
4. Press Play to start the visualization

### 3. Parameter Tuning

To optimize physics parameters:

```bash
python3 backend/digital_twin/gazebo/physics/param_tuning_script.py --robot-name turtlebot3 --output-file tuned_params.yaml
```

## Components

### Gazebo Simulation
Located in `backend/digital_twin/gazebo/`

- **Models**: Robot and environment models in URDF/SDF format
- **Worlds**: Environment definitions
- **Sensors**: LiDAR, depth camera, and IMU configurations
- **Physics**: Configuration files for physics parameters

### ROS Bridge
Located in `backend/digital_twin/ros_bridge/`

- Standard ROS 2 nodes for communication between components
- Publishers and subscribers for sensor and control data

### Perception Pipeline
Located in `backend/digital_twin/perception/`

- **LiDAR Processing**: Point cloud analysis and obstacle detection
- **Depth Camera**: 3D scene understanding
- **IMU Analysis**: Inertial measurement unit data processing
- **Sensor Fusion**: Integration of multiple sensor data streams

### Unity Visualization
Located in `unity/digital_twin_visualization/`

- Real-time visualization of robot and environment
- Synchronized with Gazebo physics simulation
- Human-robot interaction elements

## Configuration

### Robot Configuration
Robot-specific parameters are defined in JSON files in `backend/digital_twin/config/`:

```json
{
  "robot_name": "turtlebot3",
  "sensors": {
    "lidar": {
      "enabled": true,
      "topic": "/robot/lidar_scan",
      "range_min": 0.1,
      "range_max": 10.0
    },
    "depth_camera": {
      "enabled": true,
      "topic": "/robot/depth_camera/image_raw"
    },
    "imu": {
      "enabled": true,
      "topic": "/robot/imu"
    }
  },
  "physics": {
    "gravity": [0, 0, -9.81],
    "solver_type": "ode",
    "max_step_size": 0.001
  }
}
```

### Simulation Configuration
Global simulation parameters can be adjusted in `config/simulation_config.json`.

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

## Testing

The system includes tools for generating test reports and analyzing simulation fidelity:

1. Run the test report generator:
   ```bash
   python3 backend/digital_twin/utils/test_report_generator.py --include-viz
   ```

2. The report will be saved to `backend/digital_twin/test_reports/` with:
   - Fidelity metrics comparing simulated vs expected behavior
   - Performance metrics
   - Visualizations
   - Recommendations for improvement

## Performance

### Optimizing for Performance

1. **Real-time Simulation**: The system is designed to run at real-time speeds (1x) or faster
2. **Sensor Update Rates**: Configurable rates for different sensors based on requirements
3. **Visualization Quality**: Adjustable in Unity based on hardware capabilities
4. **Domain Randomization**: Can be configured for training vs testing scenarios

### Monitoring

Monitor system performance using:
- Standard ROS 2 tools like `ros2 topic hz` for topic rates
- System monitoring tools for CPU/Memory usage
- Custom analysis nodes for specific metrics

## Extending the System

### Adding New Robots

1. Create URDF model in `backend/digital_twin/gazebo/models/`
2. Add sensor configurations in `backend/digital_twin/gazebo/sensors/`
3. Create appropriate launch files in `backend/digital_twin/gazebo/launch/`
4. Update configuration files in `backend/digital_twin/config/`

### Adding New Sensors

1. Define sensor in robot's URDF
2. Create appropriate Gazebo plugin configuration
3. Add perception processing node in `backend/digital_twin/perception/`
4. Update launch files to include new nodes

### Custom Perception Modules

Add new perception functionality in `backend/digital_twin/perception/` following the same patterns as existing modules.

---

## Support

For questions and issues, please open an issue on the GitHub repository.

## License

This project is licensed under the Apache 2.0 License - see the LICENSE file for details.