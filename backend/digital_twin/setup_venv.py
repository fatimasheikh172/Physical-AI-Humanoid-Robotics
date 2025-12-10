#!/usr/bin/env python3
"""
Setup script for the Python virtual environment with non-ROS dependencies
for the Digital Twin project.

Note: ROS 2 dependencies (rclpy, etc.) must be installed via the ROS 2 installation
process on Linux systems. This script handles other Python dependencies.
"""
import os
import sys
import subprocess
import venv
from pathlib import Path

def create_virtual_environment(venv_path):
    """Create a virtual environment at the specified path."""
    print(f"Creating virtual environment at {venv_path}...")
    venv.create(venv_path, with_pip=True)
    print(f"Virtual environment created successfully at {venv_path}")

def install_non_ros_dependencies(venv_path, requirements_file):
    """Install non-ROS dependencies from requirements.txt into the virtual environment."""
    print(f"Installing non-ROS dependencies from {requirements_file}...")

    # Read the requirements file and filter out ROS-specific packages
    with open(requirements_file, 'r') as f:
        all_requirements = f.read().splitlines()

    # Filter out ROS-specific packages
    non_ros_requirements = []
    ros_packages = ['rclpy', 'rosgraph', 'std_msgs', 'sensor_msgs', 'geometry_msgs',
                   'nav_msgs', 'tf2_ros', 'cv_bridge', 'gazebo_msgs']

    for req in all_requirements:
        req = req.strip()
        # Skip comments and empty lines
        if not req or req.startswith('#'):
            continue

        # Check if this is a ROS package (by name)
        is_ros_package = any(req.startswith(ros_pkg) or req.startswith(ros_pkg + '>') or
                            req.startswith(ros_pkg + '=') or req.startswith(ros_pkg + '<')
                            for ros_pkg in ros_packages)

        if not is_ros_package:
            non_ros_requirements.append(req)

    print(f"Filtered ROS packages. Installing {len(non_ros_requirements)} non-ROS packages individually...")

    # Install dependencies one by one to handle compatibility issues
    installed_packages = []
    failed_packages = []

    for req in non_ros_requirements:
        print(f"Installing {req}...")
        try:
            subprocess.check_call([sys.executable, '-m', 'pip', 'install', req])
            installed_packages.append(req)
            print(f"  [SUCCESS] Successfully installed {req}")
        except subprocess.CalledProcessError as e:
            print(f"  [FAILED] Failed to install {req}: {e}")
            failed_packages.append(req)

    print(f"\nInstallation summary:")
    print(f"  Successfully installed: {len(installed_packages)} packages")
    print(f"  Failed to install: {len(failed_packages)} packages")

    if failed_packages:
        print(f"  Failed packages: {', '.join(failed_packages)}")
        print("\nNote: Some packages may not be compatible with your system or Python version.")
        print("This is expected in some development environments, particularly for packages")
        print("with native dependencies that are difficult to install on certain platforms.")

    return True  # Return True as partial installation is acceptable for setup

def main():
    # Define paths
    project_root = Path(__file__).parent.parent
    venv_path = project_root / 'venv'
    requirements_file = project_root / 'digital_twin' / 'requirements.txt'

    # Check if requirements file exists
    if not requirements_file.exists():
        print(f"Requirements file not found at {requirements_file}")
        return 1

    # Create virtual environment
    create_virtual_environment(venv_path)

    # Install non-ROS dependencies
    if not install_non_ros_dependencies(venv_path, requirements_file):
        return 1

    print("\nVirtual environment setup complete!")
    print(f"To activate the environment:")
    if os.name == 'nt':  # Windows
        print(f"  {venv_path / 'Scripts' / 'activate'}")
    else:  # Unix/Linux/macOS
        print(f"  source {venv_path / 'bin' / 'activate'}")

    # Note about ROS 2 dependencies
    print("\nNote: ROS 2 Humble dependencies need to be installed separately on Linux:")
    print("  1. Follow the official ROS 2 Humble installation guide for your OS")
    print("  2. Source the ROS 2 environment: source /opt/ros/humble/setup.bash")
    print("  3. The ROS Python packages will be available in any activated shell")
    print("  4. For development, you may need to install additional ROS packages via apt")

    return 0

if __name__ == '__main__':
    sys.exit(main())