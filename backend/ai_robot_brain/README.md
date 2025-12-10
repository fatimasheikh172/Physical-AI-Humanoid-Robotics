# Module 3: The AI-Robot Brain (NVIDIA Isaac™)

This module implements the AI-Robot Brain using NVIDIA Isaac ecosystem for advanced perception and training. It includes Isaac Sim for photorealistic simulation and synthetic data generation, Isaac ROS for hardware-accelerated VSLAM and navigation, and Nav2 adapted for bipedal humanoid movement path planning.

## Features

- Isaac Sim integration for photorealistic simulation
- Hardware-accelerated perception using Isaac ROS
- Visual-inertial SLAM for localization and mapping
- Bipedal navigation with Nav2
- Synthetic dataset generation with ground truth annotations
- Perception model training with synthetic data

## Architecture

The module is organized into several key components:

1. **Isaac Sim Bridge** - Connects Isaac Sim with ROS 2 ecosystem
2. **Perception Pipeline** - Processes sensor data using hardware-accelerated perception
3. **VSLAM Node** - Implements visual-inertial SLAM using Isaac ROS
4. **Synthetic Data Generator** - Creates photorealistic datasets with ground truth
5. **Training Node** - Trains perception models with synthetic datasets

## Setup

1. Install NVIDIA Isaac Sim and ROS 2 Humble
2. Install Isaac ROS packages
3. Clone this repository
4. Install Python dependencies: `pip3 install -r requirements.txt`

## Usage

1. Launch Isaac Sim with: `python -m omni.isaac.kit --exec="path/to/your/scene.py"`
2. Start the ROS bridge: `ros2 launch ai_robot_brain isaac_sim_bridge.launch.py`
3. Run perception: `ros2 run ai_robot_brain perception_node`
4. Generate synthetic datasets: `ros2 run ai_robot_brain synthetic_generator`
5. Train perception models: `ros2 run ai_robot_brain training_node`

## Configuration

The system can be configured for different robot models using the configuration files in the config/ directory.

## Documentation

See the docs/ directory for detailed documentation on each component and implementation guide.