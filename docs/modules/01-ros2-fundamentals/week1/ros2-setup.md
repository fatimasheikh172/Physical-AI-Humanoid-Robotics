# ROS 2 Installation Guide

## Prerequisites

Before installing ROS 2, ensure you have:

- Ubuntu 22.04 LTS (as established in the previous guide)
- An internet connection
- Administrative privileges (sudo access)

## Choosing Your ROS 2 Distribution

For this course, we will use **ROS 2 Humble Hawksbill**, which is the current Long Term Support (LTS) version with support until May 2027. This provides the best stability for educational purposes.

## Installation Steps

### 1. Set up the ROS 2 apt Repository

First, add the ROS 2 repository to your system:

```bash
# Update your system
sudo apt update && sudo apt install -y curl gnupg lsb-release

# Add the ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrongs/ros-archive-keyring.gpg

# Add the ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2. Install ROS 2 Humble

```bash
# Update package list
sudo apt update

# Install the desktop package which includes Gazebo and other simulation tools
sudo apt install -y ros-humble-desktop

# Install additional packages for Python development
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### 3. Initialize rosdep

```bash
# Initialize rosdep
sudo rosdep init

# Update rosdep
rosdep update
```

### 4. Install Additional Development Tools

```bash
# Install colcon for building packages
sudo apt install -y python3-colcon-common-extensions

# Install vcstool for version control
sudo apt install -y python3-vcstool
```

### 5. Setup Environment

Add the following line to your `~/.bashrc` file to automatically source ROS 2 when opening a new terminal:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Verification

To verify your installation, try running a simple ROS 2 command:

```bash
# Check ROS 2 distribution
echo $ROS_DISTRO

# Should return "humble"
```

You can also run a simple demo:

```bash
# Open a new terminal and run the talker
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# In another terminal, run the listener
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

If you see messages being published by the talker and received by the listener, your installation was successful!

## Troubleshooting

### Common Issues

1. **Command not found**: Make sure to source your ROS 2 setup script:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Package installation fails**: Update your package list:
   ```bash
   sudo apt update
   ```

3. **Python import errors**: Ensure you're using Python 3 and that your environment is properly set up:
   ```bash
   python3 -c "import rclpy; print('rclpy imported successfully')"
   ```

## Next Steps

After successfully installing ROS 2 Humble, you're ready to proceed with creating your first ROS 2 workspace and packages. The next step is to set up a workspace structure for the textbook labs.