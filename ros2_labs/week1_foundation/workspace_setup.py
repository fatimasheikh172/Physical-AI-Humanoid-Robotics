# ROS 2 Workspace Setup

## Overview

A ROS 2 workspace is a directory where you will develop and build ROS 2 packages. This workspace will contain all the code for the textbook labs.

## Creating a Workspace

Let's create a workspace for the textbook modules:

```bash
# Create the workspace directory structure
mkdir -p ~/ros2_textbook_ws/src
cd ~/ros2_textbook_ws

# The 'src' directory is where you'll place your source code
# All packages will be subdirectories within 'src'
```

## Understanding the Workspace Structure

```
ros2_textbook_ws/          # Your workspace root
├── src/                   # Source folder (where packages go)
│   ├── week1_foundation/  # Week 1 package source
│   ├── week2_communication/ # Week 2 package source
│   └── week3_ai_bridge/   # Week 3 package source
├── build/                 # Build space (created during compilation)
├── install/               # Install space (created during compilation)
└── log/                   # Logs from build process
```

## Building the Workspace

After adding packages to the `src` directory (as we've already done with the package definition files), you build the workspace using `colcon`:

```bash
cd ~/ros2_textbook_ws

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Build all packages in the workspace
colcon build

# Source the local setup file to use the built packages
source install/setup.bash
```

## Package Management

### Adding a New Package

To add a new package to your workspace:

1. Navigate to the `src` directory:
   ```bash
   cd ~/ros2_textbook_ws/src
   ```

2. Create the package using ros2 pkg:
   ```bash
   ros2 pkg create --build-type ament_python my_new_package
   ```

### Building Specific Packages

If you only want to build specific packages:

```bash
cd ~/ros2_textbook_ws

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Build only week1_foundation package
colcon build --packages-select week1_foundation

# Or build multiple specific packages
colcon build --packages-select week1_foundation week2_communication
```

## Understanding the Build System

- **colcon**: The build tool for ROS 2. It builds packages in parallel and handles dependencies.
- **ament**: The meta build system that colcon uses for building ROS 2 packages.
- **build directory**: Contains temporary files from the build process.
- **install directory**: Contains the final built packages, organized like a ROS 2 installation.

## Best Practices

1. **Always source the ROS 2 environment** before running any ROS 2 commands:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Source the workspace setup after building**:
   ```bash
   source install/setup.bash
   ```

3. **Use isolated terminals** for different ROS 2 workspaces to avoid conflicts.

## Verification

After setting up your workspace and building it, you can verify everything works:

```bash
# List ROS 2 packages in your workspace
ros2 pkg list | grep week
```

This should show the packages we've created: `week1_foundation`, `week2_communication`, and `week3_ai_bridge`.

You're now ready to work with ROS 2 packages for your textbook labs!