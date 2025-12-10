# Week 2: Unity Setup

## Overview

In Week 2, you'll set up the Unity environment and establish the bridge between ROS 2 and Unity. This creates the visualization layer of your digital twin system, allowing you to see the simulated robot and its environment in a high-fidelity 3D environment.

## Learning Objectives

By the end of this week, you will be able to:

- Install and configure Unity with the Robotics Hub package
- Set up the ROS-TCP bridge between Gazebo and Unity
- Import robot models into Unity
- Create basic visualization scenes

## Prerequisites

Before starting this week, ensure you have:

- Completed Week 1 (Gazebo Physics & Sensors)
- Unity LTS (2022.3.x or later) installed
- Unity Robotics Hub package installed
- Working Gazebo simulation with sensors
- Basic understanding of Unity interface

## Unity Installation and Setup

### Installing Unity Hub and Unity LTS

1. Download Unity Hub from the Unity website
2. Install Unity Hub and create/sign in to a Unity ID
3. Through Unity Hub, install the latest LTS version of Unity (2022.3.x or later)
4. Install the following modules during installation:
   - Windows Build Support (if on Windows)
   - Linux Build Support (if on Linux)
   - Android Build Support (optional)

### Installing Unity Robotics Hub

1. Open Unity Hub and create a new 3D project
2. In the Unity Editor, go to Window > Package Manager
3. In the Package Manager window:
   - Select "Unity Registry" from the dropdown
   - Search for "ROS TCP Connector" or "Robotics"
   - Install the "ROS TCP Connector" package
   - Search for and install "Unity Robotics Hierarchy Generator" (optional but useful)

## ROS-TCP Bridge Setup

### Configuring ROS-TCP Connection

The ROS-TCP bridge allows communication between ROS 2 and Unity. Here's how to set it up:

1. In Unity, create a new scene (File > New Scene)
2. In the Hierarchy, right-click and select "ROS Settings" to create a ROS Settings object
3. In the Inspector for the ROS Settings component:
   - Set the "ROS IP Address" to the IP address of your ROS 2 machine
   - Set the "ROS TCP Port" to 10000 (default)
   - Set "Connection Mode" to either "Server" or "Client" depending on your setup

### Testing the Connection

1. In a terminal, source your ROS 2 installation:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. Run the ROS TCP server node:
   ```bash
   ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000
   ```
3. In Unity, run the scene and check the console for connection status

## Robot Model Import

### Preparing Robot Models for Unity

Robot models need to be converted from URDF/SDF to Unity-compatible formats. Here are the approaches:

#### Method 1: Manual Import
1. Use a URDF-to-Unity converter tool (such as the one from Unity Robotics)
2. Import the converted model into Unity's Assets folder
3. Adjust scaling if necessary (Unity units are typically in meters)

#### Method 2: URDF Importer in Unity
1. Install the "URDF Importer" package from Unity Package Manager
2. Create a URDF asset in Unity
3. Point the importer to your URDF file and any accompanying mesh files
4. Import and set up the robot model in your scene

### Coordinate System Considerations

Unity uses a left-handed coordinate system (X-right, Y-up, Z-forward), while ROS uses a right-handed system (X-forward, Y-left, Z-up). Ensure proper conversion:

1. When importing robot models, you may need to rotate them to match coordinate systems
2. Typically, a rotation of X:90° and Z:180° converts from ROS to Unity coordinates
3. For specific parts, you may need additional transformations

## Basic Scene Setup

### Creating a Basic Scene

1. Create a new 3D scene
2. Import your robot model and place it in the scene
3. Create a ground plane to represent the floor
4. Add lighting that matches your Gazebo environment
5. Set up cameras for visualization

### Basic Scene Components

Your basic scene should include:

1. **Environment**: Floor plane and any static obstacles
2. **Robot Model**: Imported with proper coordinate system alignment
3. **Lighting**: Directional light to match Gazebo's lighting
4. **Camera**: Main camera to view the scene
5. **Coordinate Visualization**: Optional axes to help with orientation

## Common Setup Issues and Solutions

### Connection Issues
- **Problem**: Unity cannot connect to ROS
- **Solution**: 
  - Check IP addresses match between Unity and ROS TCP endpoint
  - Verify firewall settings allow communication on the specified port
  - Ensure ROS TCP server is running before starting Unity scene

### Model Import Issues
- **Problem**: Robot model appears distorted in Unity
- **Solution**:
  - Verify coordinate system conversion during import
  - Check if the model needs to be re-oriented
  - Ensure mesh scaling is correct

### Performance Issues
- **Problem**: Unity scene runs slowly
- **Solution**:
  - Reduce polygon count in imported models
  - Optimize lighting and shadows
  - Use Unity Profiler to identify bottlenecks

## Next Steps

After completing the Unity setup, you should have:

1. A working Unity project with Robotics Hub installed
2. A connection established between ROS 2 and Unity
3. A robot model properly imported and oriented in the scene
4. A basic scene environment set up

The next step is to implement the ROS bridge functionality to synchronize your Unity visualization with the Gazebo simulation.