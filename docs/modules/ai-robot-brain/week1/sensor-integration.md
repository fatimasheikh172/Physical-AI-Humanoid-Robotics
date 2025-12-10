# Week 1: Isaac Sim Sensor Integration

## Learning Objectives

By the end of this section, you will be able to:
- Configure LiDAR, depth camera, and IMU sensors in Isaac Sim
- Validate sensor data quality and accuracy
- Set up Isaac ROS bridge for sensor data publishing
- Integrate sensors with the ROS 2 ecosystem

## Overview

This section focuses on integrating various sensor models with your Isaac Sim humanoid robot. We'll cover the setup and configuration of LiDAR, depth camera, and IMU sensors that form the perceptual foundation of our AI-Robot Brain.

## 1. LiDAR Sensor Integration

### Adding LiDAR to a Humanoid Robot

In Isaac Sim, LiDAR sensors are added to robots via USD files. Here's an example of how to add a LiDAR sensor to a humanoid robot:

```xml
<!-- In your robot's USD file -->
def RtxSensorLidar "lidar_sensor"
{
  over "Xformable" {
    add "xformOpOrder" = ["xformOp:translate", "xformOp:orient"]
    uniform token xformOp:orient:type = "Orient"
    float3 xformOp:orient:specifier = (0, 0, 0)
    uniform token xformOp:translate:type = "Translate"
    float3 xformOp:translate:specifier = (0, 0, 0.5)  # Position the LiDAR 0.5m above ground
  }

  # LiDAR configuration
  float horizontalOpeningAngle = 6.28318548202515  # 360 degrees in radians
  float verticalOpeningAngle = 0.261799395084381  # ~15 degrees vertical FoV
  float horizontalResolution = 0.00872664612606168  # ~0.5 degree resolution
  float maxRange = 25.0
  float minRange = 0.1
  int samples = 360  # For 360-degree scanning
  float rpm = 600  # Rotations per minute
  float skew = 0

  # Physics properties required for sensor
  physics:approximation = "convexDecomposition"
  physics:collisionEnabled = 1
  physics:frictionEnabled = 1
  physics:restitutionEnabled = 0
  physics:rigidBodyEnabled = 1

  # Add the sensor plugin
  add "apiSchemas" = [
    "IsaacSensor",
    "IsaacROS Lidar"
  ]

  # Isaac ROS specific properties
  string publisherTopic = "/robot/lidar_scan"
  string frameId = "lidar_frame"
  float updateRate = 10.0
}
```

### Configuring LiDAR Parameters

For realistic humanoid perception, configure the LiDAR with appropriate parameters:

- **Range**: 0.1m to 25m for indoor humanoid navigation
- **Resolution**: 0.5° for balance between accuracy and performance
- **Update Rate**: 10Hz for real-time navigation
- **Field of View**: 360° horizontal, 15-30° vertical

## 2. Depth Camera Integration

### Adding Depth Camera to Humanoid Robot

Depth cameras provide 3D scene understanding capabilities for the humanoid:

```xml
def RtxSensorStereoCamera "depth_camera"
{
  # Position relative to robot
  over "Xformable" {
    add "xformOpOrder" = ["xformOp:translate", "xformOp:orient"]
    uniform token xformOp:orient:type = "Orient"
    float3 xformOp:orient:specifier = (0, 0, 0)  # Facing forward
    uniform token xformOp:translate:type = "Translate"
    float3 xformOp:translate:specifier = (0.2, 0, 0.8)  # On robot's head/chest
  }

  # Camera configuration
  float focalLength = 24.0
  float horizontalAperture = 20.955
  float verticalAperture = 15.2908
  int resolution:width = 640
  int resolution:height = 480
  float clippingRange:far = 10.0
  float clippingRange:near = 0.1

  # Isaac ROS depth camera parameters
  add "apiSchemas" = [
    "IsaacSensor",
    "IsaacROS Depth Camera"
  ]

  # ROS topic configuration
  string colorPublisherTopic = "/robot/depth_camera/color/image_rect_color"
  string depthPublisherTopic = "/robot/depth_camera/depth_registered/image_rect"
  string infoPublisherTopic = "/robot/depth_camera/color/camera_info"
  string frameId = "depth_camera_frame"
  float updateRate = 30.0
}
```

### Depth Camera Configuration Guidelines

- **Resolution**: 640×480 for real-time processing with good detail
- **Field of View**: 60° horizontal for humanoid perspective
- **Range**: 0.1m to 10m for indoor navigation
- **Update Rate**: 30Hz for real-time depth perception

## 3. IMU Integration

### Adding IMU to Humanoid Robot

An IMU is crucial for humanoid balance and state estimation:

```xml
def Imu "imu_sensor"
{
  # Position relative to robot base
  over "Xformable" {
    add "xformOpOrder" = ["xformOp:translate", "xformOp:orient"]
    uniform token xformOp:orient:type = "Orient"
    float3 xformOp:orient:specifier = (0, 0, 0)  # Aligned with robot frame
    uniform token xformOp:translate:type = "Translate"
    float3 xformOp:translate:specifier = (0, 0, 0.5)  # At robot's center of mass
  }

  # Physics properties
  physics:approximation = "convexDecomposition"
  physics:collisionEnabled = 1
  physics:frictionEnabled = 1
  physics:restitutionEnabled = 0
  physics:rigidBodyEnabled = 1

  # Isaac ROS IMU configuration
  add "apiSchemas" = [
    "IsaacSensor",
    "IsaacROS IMU"
  ]

  string publisherTopic = "/robot/imu"
  string frameId = "imu_frame"
  float updateRate = 100.0  # 100Hz for accurate state estimation
}
```

### IMU Configuration Guidelines

- **Update Rate**: 100Hz for accurate humanoid state estimation
- **Accuracy**: Match parameters to real-world IMU specifications
- **Placement**: Near robot's center of mass for accurate readings
- **Frame**: Aligned with robot's principal axes

## 4. Isaac ROS Bridge Setup

### Establishing the ROS Bridge

The Isaac ROS Bridge connects Isaac Sim with the ROS 2 ecosystem:

```python
# Python script to set up Isaac ROS bridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from std_msgs.msg import String
import numpy as np


class IsaacSimBridge(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge')
        
        # Declare parameters
        self.declare_parameter('robot_name', 'humanoid_robot')
        self.declare_parameter('lidar_topic', '/robot/lidar_scan')
        self.declare_parameter('depth_camera_topic', '/robot/depth_camera/image_rect_color')
        self.declare_parameter('imu_topic', '/robot/imu')
        
        # Get parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.depth_camera_topic = self.get_parameter('depth_camera_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        
        # Subscribers for Isaac Sim sensor data
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self.lidar_callback,
            10
        )
        
        self.depth_subscription = self.create_subscription(
            Image,
            self.depth_camera_topic,
            self.depth_callback,
            10
        )
        
        self.imu_subscription = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10
        )
        
        # Publishers for processed data
        self.processed_lidar_publisher = self.create_publisher(LaserScan, '/processed/lidar_scan', 10)
        self.processed_depth_publisher = self.create_publisher(Image, '/processed/depth_image', 10)
        self.status_publisher = self.create_publisher(String, '/robot/status', 10)
        
        self.get_logger().info(f'Isaac Sim Bridge initialized for robot: {self.robot_name}')
    
    def lidar_callback(self, msg):
        """Process incoming LiDAR data from Isaac Sim"""
        # Process LiDAR data for perception algorithms
        self.get_logger().debug(f'Received LiDAR data with {len(msg.ranges)} range readings')
        
        # Validate data
        if not self.validate_lidar_data(msg):
            self.get_logger().warn('Invalid LiDAR data received')
            return
        
        # Process data (remove this comment and add actual processing)
        # processed_msg = self.process_lidar_data(msg)
        # self.processed_lidar_publisher.publish(processed_msg)
    
    def depth_callback(self, msg):
        """Process incoming depth camera data from Isaac Sim"""
        self.get_logger().debug(f'Received depth image: {msg.width}x{msg.height}')
    
    def imu_callback(self, msg):
        """Process incoming IMU data from Isaac Sim"""
        self.get_logger().debug(f'Received IMU data: linear_acc=({msg.linear_acceleration.x:.3f}, {msg.linear_acceleration.y:.3f}, {msg.linear_acceleration.z:.3f})')
    
    def validate_lidar_data(self, msg):
        """Validate LiDAR message integrity"""
        # Check if ranges array is valid
        if not msg.ranges:
            return False
        
        # Check for valid range values
        for r in msg.ranges:
            if r < msg.range_min or r > msg.range_max:
                if not np.isnan(r) and not np.isinf(r):
                    return False
        
        return True


def main(args=None):
    rclpy.init(args=args)
    bridge_node = IsaacSimBridge()
    
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()