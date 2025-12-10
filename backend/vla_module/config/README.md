# VLA Module Configuration Templates

This directory contains configuration templates for different robot models that can be used with the Vision-Language-Action (VLA) module.

## Available Robot Configurations

### 1. TurtleBot3 Waffle Pi
**File**: `turtlebot3_waffle_pi.yaml`
**Use Case**: Educational and research applications, indoor navigation and manipulation
**Features**: Differential drive, 2D LiDAR, 3D camera, arm manipulator

### 2. Fetch Robotics Mobile Manipulator
**File**: `fetch_robotics.yaml`
**Use Case**: Warehouse automation, object manipulation tasks
**Features**: Omnidirectional base, 7-DOF arm, 3D camera, gripper

### 3. PR2 Humanoid Assistant
**File**: `pr2.yaml`
**Use Case**: Human-robot interaction, complex manipulation
**Features**: Two 7-DOF arms, tilting laser scanner, stereo cameras, head pan/tilt

### 4. Spot Quadruped Robot
**File**: `spot.yaml`
**Use Case**: Inspection, surveillance, navigating challenging terrain
**Features**: Four-legged mobility, articulated arms, 360-degree cameras

### 5. Custom Humanoid Robot
**File**: `humanoid_custom.yaml`
**Use Case**: Advanced HRI, bipedal navigation, human-like manipulation
**Features**: Bipedal legs, dual arms, head with stereo cameras, torso mobility

---

Each robot configuration file defines:
- Physical capabilities (navigation, manipulation, perception)
- Action types supported
- Hardware-specific parameters
- Safety constraints
- Performance parameters
- Default configurations for the VLA system

To use a specific robot configuration, set the `ROBOT_CONFIG_PATH` environment variable to point to your chosen configuration file before running the VLA system.