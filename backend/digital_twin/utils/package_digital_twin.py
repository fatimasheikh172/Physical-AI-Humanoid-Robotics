#!/usr/bin/env python3

"""
Digital Twin Packaging Script

This script packages the digital twin system for reproducible runs,
including all necessary components, configuration files, and documentation.
"""

import os
import shutil
import zipfile
import tarfile
import json
from pathlib import Path
from datetime import datetime
import argparse


class DigitalTwinPackager:
    def __init__(self, source_dir: str, output_dir: str):
        self.source_dir = Path(source_dir)
        self.output_dir = Path(output_dir)
        self.package_info = {
            'name': 'digital_twin_package',
            'version': '1.0.0',
            'created': datetime.now().isoformat(),
            'components': []
        }
        
    def create_package_structure(self, package_path: Path):
        """Create the directory structure for the package"""
        dirs = [
            'gazebo/models',
            'gazebo/worlds', 
            'gazebo/launch',
            'gazebo/sensors',
            'gazebo/physics',
            'ros_bridge/nodes',
            'perception/lidar_processing',
            'perception/depth_camera',
            'perception/imu_analysis',
            'utils/scripts',
            'docs',
            'config'
        ]
        
        for dir_path in dirs:
            (package_path / dir_path).mkdir(parents=True, exist_ok=True)
    
    def copy_gazebo_components(self, package_path: Path):
        """Copy Gazebo-related components to the package"""
        gazebo_source = self.source_dir / 'gazebo'
        gazebo_dest = package_path / 'gazebo'
        
        # Copy models
        models_source = gazebo_source / 'models'
        if models_source.exists():
            shutil.copytree(models_source, gazebo_dest / 'models', dirs_exist_ok=True)
            self.package_info['components'].append('gazebo_models')
        
        # Copy worlds
        worlds_source = gazebo_source / 'models' / 'worlds'  # Note: worlds are in models folder based on plan
        if worlds_source.exists():
            shutil.copytree(worlds_source, gazebo_dest / 'worlds', dirs_exist_ok=True)
            self.package_info['components'].append('gazebo_worlds')
        
        # Copy launch files
        launch_source = gazebo_source / 'launch'
        if launch_source.exists():
            shutil.copytree(launch_source, gazebo_dest / 'launch', dirs_exist_ok=True)
            self.package_info['components'].append('gazebo_launch')
        
        # Copy sensors
        sensors_source = gazebo_source / 'sensors'
        if sensors_source.exists():
            shutil.copytree(sensors_source, gazebo_dest / 'sensors', dirs_exist_ok=True)
            self.package_info['components'].append('gazebo_sensors')
        
        # Copy physics
        physics_source = gazebo_source / 'physics'
        if physics_source.exists():
            shutil.copytree(physics_source, gazebo_dest / 'physics', dirs_exist_ok=True)
            self.package_info['components'].append('gazebo_physics')
    
    def copy_ros_bridge_components(self, package_path: Path):
        """Copy ROS bridge components to the package"""
        ros_bridge_source = self.source_dir / 'ros_bridge'
        ros_bridge_dest = package_path / 'ros_bridge'
        
        if ros_bridge_source.exists():
            shutil.copytree(ros_bridge_source, ros_bridge_dest, dirs_exist_ok=True)
            self.package_info['components'].append('ros_bridge')
    
    def copy_perception_components(self, package_path: Path):
        """Copy perception components to the package"""
        perception_source = self.source_dir / 'perception'
        perception_dest = package_path / 'perception'
        
        if perception_source.exists():
            shutil.copytree(perception_source, perception_dest, dirs_exist_ok=True)
            self.package_info['components'].append('perception_modules')
    
    def copy_utils_and_scripts(self, package_path: Path):
        """Copy utility scripts to the package"""
        utils_source = self.source_dir / 'utils'
        utils_dest = package_path / 'utils'
        
        if utils_source.exists():
            shutil.copytree(utils_source, utils_dest, dirs_exist_ok=True)
            self.package_info['components'].append('utility_scripts')
    
    def copy_config_files(self, package_path: Path):
        """Copy configuration files to the package"""
        # Create default config files if they don't exist
        config_dest = package_path / 'config'
        
        # Create default robot config
        robot_config = {
            'robot_name': 'default_robot',
            'sensors': {
                'lidar': {
                    'enabled': True,
                    'topic': '/robot/lidar_scan',
                    'range_min': 0.1,
                    'range_max': 10.0
                },
                'depth_camera': {
                    'enabled': True,
                    'topic': '/robot/depth_camera/image_raw'
                },
                'imu': {
                    'enabled': True,
                    'topic': '/robot/imu'
                }
            },
            'physics': {
                'gravity': [0, 0, -9.81],
                'solver_type': 'ode',
                'max_step_size': 0.001
            }
        }
        
        with open(config_dest / 'robot_config.json', 'w') as f:
            json.dump(robot_config, f, indent=2)
        
        # Create simulation config
        sim_config = {
            'simulation': {
                'real_time_factor': 1.0,
                'max_update_rate': 1000,
                'paused': False
            },
            'visualization': {
                'show_sensors': True,
                'show_contacts': False
            }
        }
        
        with open(config_dest / 'simulation_config.json', 'w') as f:
            json.dump(sim_config, f, indent=2)
        
        self.package_info['components'].append('config_files')
    
    def create_documentation(self, package_path: Path):
        """Create documentation for the package"""
        docs_dest = package_path / 'docs'
        
        # Create README
        readme_content = f"""# Digital Twin Package

## Package Information
- Name: {self.package_info['name']}
- Version: {self.package_info['version']}
- Created: {self.package_info['created']}

## Components Included
{chr(10).join(['- ' + comp for comp in self.package_info['components']])}

## Setup Instructions

1. Extract this package to your desired location
2. Make sure you have ROS 2 Humble installed
3. Source your ROS 2 installation: `source /opt/ros/humble/setup.bash`
4. Navigate to the package directory
5. Install Python dependencies: `pip install -r requirements.txt` (if available)

## Running the Digital Twin

### 1. Launch Gazebo Simulation
```bash
cd gazebo/launch
ros2 launch demo.launch.py
```

### 2. Launch Perception Nodes
```bash
# Launch all perception nodes
ros2 launch perception_pipeline.launch.py

# Or launch specific nodes:
ros2 run digital_twin_perception imu_analysis_node
ros2 run digital_twin_perception perception_demo
```

### 3. Launch Unity Visualization (Separate Terminal)
Open the Unity project in unity/digital_twin_visualization/ and run the main scene

## Configuration

Modify configuration files in the `config/` directory to customize the simulation:
- `robot_config.json` - Robot-specific parameters
- `simulation_config.json` - Simulation parameters

## Troubleshooting

1. If sensors are not publishing data:
   - Check that the Gazebo simulation is running
   - Verify ROS domain ID matches across all terminals
   - Use `ros2 topic list` to confirm topics exist

2. If Unity visualization is not connecting:
   - Check that the ROS-TCP bridge is properly configured
   - Verify IP addresses match between Unity and ROS
   - Check firewall settings

3. Performance issues:
   - Reduce simulation complexity
   - Lower visualization quality in Unity
   - Check system resources (CPU, RAM, GPU)

For more detailed documentation, please refer to the course materials.
"""
        
        with open(docs_dest / 'README.md', 'w') as f:
            f.write(readme_content)
        
        # Create quick start guide
        quickstart_content = """# Quick Start Guide

## Prerequisites
- ROS 2 Humble
- Gazebo Classic
- Python 3.8+
- Unity LTS (for visualization)

## Setup
1. Source ROS: `source /opt/ros/humble/setup.bash`
2. Install dependencies: `pip install rclpy numpy scipy`

## Running the Demo
1. Launch simulation: `ros2 launch gazebo/demo.launch.py`
2. Launch perception: `ros2 launch perception/perception_pipeline.launch.py`
3. Open Unity project and run scene

## Expected Output
- Robot model should appear in Gazebo
- Sensor data should be published to ROS topics
- Perception nodes should process sensor data
- Unity visualization should sync with Gazebo
"""
        
        with open(docs_dest / 'QUICKSTART.md', 'w') as f:
            f.write(quickstart_content)
        
        self.package_info['components'].append('documentation')
    
    def create_requirements_file(self, package_path: Path):
        """Create a requirements file for the package"""
        requirements_content = """# Python dependencies for Digital Twin System
rclpy>=3.0.0
numpy>=1.19.0
scipy>=1.7.0
transforms3d>=0.4.0
matplotlib>=3.5.0
PyYAML>=6.0
opencv-python>=4.5.0
cv-bridge>=3.0.0
"""
        
        with open(package_path / 'requirements.txt', 'w') as f:
            f.write(requirements_content)
    
    def create_main_launch_file(self, package_path: Path):
        """Create a main launch file to start the complete pipeline"""
        launch_content = """from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('digital_twin_gazebo')
    
    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'demo.launch.py')
        )
    )
    
    # IMU Analysis node
    imu_analysis_node = Node(
        package='digital_twin_perception',
        executable='imu_analysis_node',
        name='imu_analysis_node',
        parameters=[
            {'imu_topic': '/robot/imu'},
            {'publish_frequency': 10.0}
        ],
        output='screen'
    )
    
    # Perception demo node
    perception_demo_node = Node(
        package='digital_twin_perception',
        executable='perception_demo',
        name='perception_demo_node',
        parameters=[
            {'lidar_topic': '/robot/lidar_scan'},
            {'depth_camera_topic': '/robot/depth_camera/image_raw'},
            {'imu_topic': '/robot/imu'}
        ],
        output='screen'
    )
    
    # Sensor fidelity analysis node
    sensor_fidelity_node = Node(
        package='digital_twin_perception',
        executable='sensor_fidelity_analysis',
        name='sensor_fidelity_analyzer',
        parameters=[
            {'lidar_topic': '/robot/lidar_scan'},
            {'imu_topic': '/robot/imu'},
            {'odom_topic': '/robot/odom'}
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        imu_analysis_node,
        perception_demo_node,
        sensor_fidelity_node
    ])
"""
        
        # Create launch directory and main launch file
        launch_dir = package_path / 'launch'
        launch_dir.mkdir(exist_ok=True)
        
        with open(launch_dir / 'digital_twin_pipeline.launch.py', 'w') as f:
            f.write(launch_content)
    
    def package_digital_twin(self, package_name: str = None) -> str:
        """Create the complete digital twin package"""
        if package_name is None:
            package_name = f"digital_twin_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        # Create package directory
        package_path = self.output_dir / package_name
        package_path.mkdir(parents=True, exist_ok=True)
        
        print(f"Creating digital twin package at: {package_path}")
        
        # Create package structure
        self.create_package_structure(package_path)
        
        # Copy components
        self.copy_gazebo_components(package_path)
        self.copy_ros_bridge_components(package_path)
        self.copy_perception_components(package_path)
        self.copy_utils_and_scripts(package_path)
        self.copy_config_files(package_path)
        self.create_documentation(package_path)
        self.create_requirements_file(package_path)
        self.create_main_launch_file(package_path)
        
        # Create package info file
        with open(package_path / 'package_info.json', 'w') as f:
            json.dump(self.package_info, f, indent=2)
        
        # Create archive
        archive_path = self.output_dir / f"{package_name}.zip"
        with zipfile.ZipFile(archive_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
            for root, dirs, files in os.walk(package_path):
                for file in files:
                    file_path = os.path.join(root, file)
                    arc_path = os.path.relpath(file_path, start=self.output_dir)
                    zipf.write(file_path, arc_path)
        
        print(f"Package created successfully: {archive_path}")
        print(f"Components included: {self.package_info['components']}")
        
        return str(archive_path)


def main():
    parser = argparse.ArgumentParser(description='Package Digital Twin for Reproducible Runs')
    parser.add_argument('--source-dir', type=str, 
                        default='E:/Hackhaton/ai-native-book/backend/digital_twin',
                        help='Source directory of the digital twin system')
    parser.add_argument('--output-dir', type=str, 
                        default='E:/Hackhaton/ai-native-book/dist',
                        help='Output directory for the package')
    parser.add_argument('--package-name', type=str,
                        help='Name for the package (auto-generated if not provided)')
    
    args = parser.parse_args()
    
    # Ensure output directory exists
    output_path = Path(args.output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Create packager
    packager = DigitalTwinPackager(args.source_dir, output_path)
    
    # Create package
    package_path = packager.package_digital_twin(args.package_name)
    
    print(f"Digital twin packaged successfully at: {package_path}")


if __name__ == '__main__':
    main()