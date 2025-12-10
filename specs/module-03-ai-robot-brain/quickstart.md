# Quickstart Guide: Module 03 - The AI-Robot Brain (NVIDIA Isaac™)

## Overview

This guide provides quick instructions to set up and run the AI-Robot Brain module with NVIDIA Isaac ecosystem. You'll learn to use Isaac Sim for photorealistic simulation and synthetic data generation, Isaac ROS for hardware-accelerated VSLAM, and Nav2 for bipedal humanoid navigation.

## Prerequisites

### System Requirements
- Operating System: Ubuntu 20.04/22.04 or Windows 10/11 (with WSL2 for ROS 2)
- GPU: NVIDIA Turing, Ampere, or Ada Lovelace GPU (RTX 20 series or later; RTX 3060 or better recommended)
- RAM: 32GB minimum, 64GB recommended
- Storage: 500GB SSD free space minimum
- CUDA: 11.8 or newer
- Network: Internet access for Isaac Sim asset downloads

### Software Requirements
- NVIDIA Isaac Sim (latest version)
- ROS 2 Humble Hawksbill or Rolling
- Isaac ROS packages
- Python 3.8+ with CUDA-enabled libraries
- Docker (optional, for containerized environments)

## Installation

### 1. Isaac Sim Setup

1. Download Isaac Sim from NVIDIA Omniverse:
   ```bash
   # Visit https://developer.nvidia.com/isaac-sim and download the latest version
   # Or use Omniverse launcher to install Isaac Sim
   ```

2. Install Isaac Sim:
   ```bash
   # Unzip downloaded package
   unzip isaac-sim-package-2023.1.1.zip
   cd isaac-sim-package-2023.1.1
   
   # Execute installer
   bash install_isaac_sim.sh
   ```

3. Configure Isaac Sim:
   ```bash
   # Add Isaac Sim to your path
   echo 'source /path/to/isaac-sim/set_python_env.sh' >> ~/.bashrc
   source ~/.bashrc
   ```

4. Verify Isaac Sim installation:
   ```bash
   python -c "import omni; print('Isaac Sim OK')"
   ```

### 2. ROS 2 and Isaac ROS Setup

1. Install ROS 2 Humble:
   ```bash
   # Add ROS 2 repository
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
   echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list
   
   # Install ROS 2
   sudo apt update
   sudo apt install -y ros-humble-desktop ros-humble-ros-base
   sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
   ```

2. Install Isaac ROS dependencies:
   ```bash
   # Source ROS 2
   source /opt/ros/humble/setup.bash
   
   # Create a workspace
   mkdir -p ~/isaac_ws/src
   cd ~/isaac_ws
   
   # Install Isaac ROS packages via apt
   sudo apt update
   sudo apt install -y ros-humble-isaac-ros-point-cloud-builder ros-humble-isaac-ros-segmentation ros-humble-isaac-ros-augmenter ros-humble-isaac-ros-visual-inertial-odometry
   ```

3. Install Python dependencies:
   ```bash
   pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
   pip3 install open3d opencv-python numpy scipy matplotlib
   pip3 install omniverse-isaac-gym-envs  # If using for reinforcement learning
   ```

### 3. Clone the Course Repository

```bash
git clone https://github.com/[your-org]/ai-native-robotic-education.git
cd ai-native-robotic-education
```

## Running the AI-Robot Brain

### 1. Launch Isaac Sim with Humanoid Robot

1. Start Isaac Sim:
   ```bash
   # Source the Isaac Sim environment
   cd /path/to/isaac-sim
   bash setup_conda_env.sh  # If using conda environment
   # Or source the provided script
   source set_python_env.sh
   
   # Launch Isaac Sim application
   python -m omni.isaac.kit --exec="path/to/your/humanoid_scene.py"
   ```

2. Load a humanoid robot into Isaac Sim:
   - Open Isaac Sim editor
   - Create a new scene or open an existing one
   - Import your humanoid robot model (URDF or USD format)
   - Ensure sensors are properly configured (RGB cameras, depth, IMU)

### 2. Launch Isaac ROS Bridge

1. In a new terminal, source ROS 2 and Isaac ROS:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ws/install/setup.bash  # Isaac ROS packages
   ```

2. Launch the Isaac ROS bridge:
   ```bash
   ros2 launch ai_robot_brain isaac_sim_bridge.launch.py
   ```

### 3. Launch Perception Node with Hardware Acceleration

1. Start a perception node for object detection:
   ```bash
   ros2 run ai_robot_brain perception_node --ros-args -p model_path:=/path/to/yolo_model.plan
   ```

2. View perception results:
   ```bash
   # In another terminal
   ros2 run rviz2 rviz2
   # Add displays for camera feed and detections
   ```

### 4. Launch VSLAM with Hardware Acceleration

1. Start the VSLAM pipeline:
   ```bash
   ros2 launch ai_robot_brain vslam_pipeline.launch.py
   ```

### 5. Launch Bipedal Navigation with Nav2

1. Start Nav2 with bipedal configuration:
   ```bash
   ros2 launch ai_robot_brain bipedal_nav2.launch.py
   ```

## Generating Synthetic Datasets

### 1. Basic Dataset Generation

1. Run the synthetic dataset generation:
   ```bash
   ros2 run ai_robot_brain synthetic_dataset_generator \
     --scene-path=/path/to/scenes/my_office.usd \
     --output-path=/path/to/dataset/output \
     --annotations="bounding_boxes,segmentation,depth" \
     --image-count=10000
   ```

### 2. Advanced Dataset Generation with Domain Randomization

1. Generate dataset with domain randomization:
   ```bash
   ros2 run ai_robot_brain synthetic_dataset_generator \
     --scene-path=/path/to/scenes/my_office.usd \
     --output-path=/path/to/dataset/output \
     --domain-randomization=true \
     --lighting-variation=true \
     --texture-variation=true \
     --physics-enabled=true
   ```

## Training Perception Models

### 1. Train Object Detection Model

1. Train a model with synthetic data:
   ```bash
   ros2 run ai_robot_brain train_perception_model \
     --model-architecture=yolo \
     --dataset-path=/path/to/synthetic/dataset \
     --output-path=/path/to/trained/model \
     --epochs=100 \
     --batch-size=16
   ```

### 2. Evaluate Model Performance

1. Evaluate model on validation data:
   ```bash
   ros2 run ai_robot_brain evaluate_model \
     --model-path=/path/to/trained/model \
     --dataset-path=/path/to/evaluation/dataset
   ```

## Basic Examples

### Example 1: Perception Pipeline with Isaac Sim

1. Launch Isaac Sim with a humanoid robot in a simple environment:
   ```bash
   python -m omni.isaac.kit --exec="examples/humanoid_perception_example.py"
   ```

2. In another terminal, run the perception pipeline:
   ```bash
   ros2 launch ai_robot_brain perception_pipeline.launch.py
   ```

3. View the results in RViz2 or Isaac Sim viewport.

### Example 2: VSLAM in Isaac Sim

1. Launch Isaac Sim with a humanoid robot in a mapped environment:
   ```bash
   python -m omni.isaac.kit --exec="examples/vslam_demo.py"
   ```

2. Run Isaac ROS VSLAM pipeline:
   ```bash
   ros2 launch ai_robot_brain vslam_pipeline.launch.py
   ```

3. Visualize the map and robot pose in RViz2.

### Example 3: Bipedal Navigation

1. Launch the full pipeline:
   ```bash
   ros2 launch ai_robot_brain full_pipeline.launch.py  # Combines perception, VSLAM, navigation
   ```

2. Send navigation goals:
   ```bash
   ros2 run nav2_msgs navigation_goal_publisher --x 5.0 --y 3.0 --theta 0.0
   ```

## Troubleshooting

### Common Issues

1. **GPU/CUDA Errors**:
   - Ensure CUDA version matches Isaac Sim requirements
   - Check that NVIDIA drivers are properly installed: `nvidia-smi`
   - Verify CUDA installation: `nvcc --version`

2. **Isaac Sim Crashes**:
   - Increase GPU power limits: `sudo nvidia-smi -pl [wattage]`
   - Ensure sufficient VRAM (8GB+ recommended)
   - Check Isaac Sim logs in `~/.nvidia-omniverse/logs/`

3. **ROS Communication Issues**:
   - Ensure both Isaac Sim and ROS processes are on the same ROS_DOMAIN_ID
   - Check ROS IP settings if running across multiple machines
   - Verify that Isaac ROS bridge is properly running

4. **Perception Performance**:
   - Check GPU utilization: `nvidia-smi`
   - Ensure models are optimized with TensorRT
   - Reduce input resolution if needed for real-time performance

5. **VSLAM Tracking Loss**:
   - Ensure sufficient lighting in simulation
   - Check camera parameters match physical equivalents
   - Verify IMU integration is properly configured

### Verification Commands

1. Check that Isaac ROS nodes are running:
   ```bash
   ros2 node list | grep isaac
   ```

2. Check available topics:
   ```bash
   ros2 topic list | grep -E "(perception|vslam|nav)"
   ```

3. Verify perception data is being published:
   ```bash
   ros2 topic echo /perception/detections
   ```

4. Check VSLAM pose:
   ```bash
   ros2 topic echo /vslam/pose
   ```

## Next Steps

After completing the quickstart:

1. Explore advanced perception models and training techniques
2. Experiment with domain randomization for better sim-to-real transfer
3. Implement reinforcement learning for bipedal locomotion in simulation
4. Integrate all modules (1-3) for a complete AI-robot brain system