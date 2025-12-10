# Week 2: Isaac ROS VSLAM (Visual Simultaneous Localization and Mapping)

## Learning Objectives

By the end of this week, you will be able to:
- Implement Isaac ROS hardware-accelerated VSLAM pipeline
- Configure GPU-accelerated perception nodes for real-time performance
- Integrate VSLAM with ROS 2 transform system for localization
- Optimize VSLAM parameters for humanoid robot navigation
- Validate VSLAM accuracy against ground truth simulation data

## Overview

Week 2 focuses on implementing Isaac ROS for hardware-accelerated Visual SLAM (VSLAM). You'll learn to leverage Isaac ROS's hardware-accelerated perception nodes that utilize GPU acceleration through CUDA and TensorRT for real-time localization and mapping on humanoid robots.

## 1. Isaac ROS VSLAM Pipeline

### Understanding Isaac ROS Hardware Acceleration

Isaac ROS packages are specifically designed to leverage NVIDIA GPU hardware for accelerated perception and navigation. Key components include:

- **Isaac ROS Visual Inertial Odometry (VIO)**: Combines visual and inertial measurements for robust pose estimation
- **Isaac ROS Stereo Disparity**: Computes depth from stereo camera inputs
- **Isaac ROS Point Cloud Builder**: Constructs 3D point clouds from depth data
- **Isaac ROS AprilTag Detection**: Detects fiducial markers for precise localization

### Installing Isaac ROS Packages

First, install the necessary Isaac ROS packages:

```bash
# Update package list
sudo apt update

# Install Isaac ROS packages
sudo apt install -y ros-humble-isaac-ros-point-cloud-builder
sudo apt install -y ros-humble-isaac-ros-segmentation
sudo apt install -y ros-humble-isaac-ros-augmenter
sudo apt install -y ros-humble-isaac-ros-visual-inertial-odometry
sudo apt install -y ros-humble-isaac-ros-stereo-disparity
sudo apt install -y ros-humble-isaac-ros-april-tag-3d
sudo apt install -y ros-humble-isaac-ros-gems

# Verify installation
ros2 pkg list | grep isaac_ros
```

## 2. VSLAM Node Implementation

### Creating a VSLAM Node

Now let's implement a VSLAM node that leverages Isaac ROS hardware acceleration:

```python
# vslam_node.py
#!/usr/bin/env python3

"""
Isaac ROS VSLAM Node for Humanoid Robot Navigation

This node implements hardware-accelerated Visual SLAM using Isaac ROS packages
for real-time localization and mapping on humanoid robots.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
import message_filters
import time


class IsaacVSLAMNode(Node):
    def __init__(self):
        super().__init__('isaac_vslam_node')
        
        # Declare parameters
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 480)
        self.declare_parameter('update_rate', 30.0)
        self.declare_parameter('max_features', 1000)
        self.declare_parameter('tracking_quality_threshold', 0.7)
        self.declare_parameter('relocalization_enabled', True)
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        
        # Get parameters
        self.input_width = self.get_parameter('input_width').value
        self.input_height = self.get_parameter('input_height').value
        self.update_rate = self.get_parameter('update_rate').value
        self.max_features = self.get_parameter('max_features').value
        self.tracking_quality_threshold = self.get_parameter('tracking_quality_threshold').value
        self.relocalization_enabled = self.get_parameter('relocalization_enabled').value
        self.use_gpu = self.get_parameter('use_gpu').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # Initialize GPU acceleration if available
        if self.use_gpu:
            self.get_logger().info('Using GPU acceleration for VSLAM')
            # Isaac ROS nodes will automatically use GPU when available
        else:
            self.get_logger().info('Using CPU for VSLAM (not recommended)')
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create publishers for VSLAM results
        self.pose_publisher = self.create_publisher(PoseStamped, '/vslam/pose', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/vslam/odometry', 10)
        self.status_publisher = self.create_publisher(String, '/vslam/status', 10)
        
        # QoS profile for sensor data
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        
        # Initialize camera info
        self.left_camera_info = None
        self.right_camera_info = None
        
        # Create subscribers for stereo pair
        self.left_camera_sub = message_filters.Subscriber(
            self, Image, '/camera/left/image_rect_color', qos_profile=sensor_qos)
        self.right_camera_sub = message_filters.Subscriber(
            self, Image, '/camera/right/image_rect_color', qos_profile=sensor_qos)
        self.left_cam_info_sub = self.create_subscription(
            CameraInfo, '/camera/left/camera_info', self.left_cam_info_callback, 10)
        self.right_cam_info_sub = self.create_subscription(
            CameraInfo, '/camera/right/camera_info', self.right_cam_info_callback, 10)
        
        # Synchronize stereo images
        self.sync = message_filters.ApproximateTimeSynchronizer(
            (self.left_camera_sub, self.right_camera_sub), 
            queue_size=10, 
            slop=0.1
        )
        self.sync.registerCallback(self.stereo_callback)
        
        # Store latest pose estimates
        self.current_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0])  # w,x,y,z quaternion
        }
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.last_update_time = time.time()
        
        # Tracking status
        self.tracking_status = 'Initializing'  # Initializing, Tracking, Lost, Recovering
        self.tracking_confidence = 0.0
        
        # Create timer for publishing pose updates
        self.pose_timer = self.create_timer(1.0/self.update_rate, self.publish_pose)
        
        self.get_logger().info('Isaac VSLAM node initialized')

    def left_cam_info_callback(self, msg):
        """Store left camera calibration info"""
        self.left_camera_info = msg

    def right_cam_info_callback(self, msg):
        """Store right camera calibration info"""
        self.right_camera_info = msg

    def stereo_callback(self, left_msg, right_msg):
        """Process synchronized stereo images for VSLAM"""
        try:
            # In a real implementation, this would interface with Isaac ROS VSLAM
            # For this example, we'll simulate processing
            
            # Validate camera info
            if not self.left_camera_info or not self.right_camera_info:
                self.get_logger().debug('Waiting for camera calibration info')
                return
            
            # Check if images are properly synchronized
            time_diff = abs(left_msg.header.stamp.sec - right_msg.header.stamp.sec + 
                           (left_msg.header.stamp.nanosec - right_msg.header.stamp.nanosec)/1e9)
            if time_diff > 0.05:  # More than 50ms difference
                self.get_logger().warn(f'Stereo images not properly synchronized: {time_diff:.3f}s')
            
            # Simulate visual feature extraction and matching
            # In real implementation, this would use Isaac ROS VIO
            features_tracked = self.extract_and_match_features(left_msg, right_msg)
            
            if features_tracked > 50:  # Minimum features for reliable tracking
                # Estimate pose change based on feature motion
                pose_change = self.estimate_pose_change(features_tracked)
                
                # Update current pose
                self.update_pose_estimate(pose_change)
                
                # Update tracking status
                self.tracking_status = 'Tracking'
                self.tracking_confidence = min(1.0, features_tracked / 200)  # Normalize to 0-1
            else:
                # Not enough features - tracking may be lost
                if self.tracking_confidence > 0.3:
                    self.tracking_confidence -= 0.1  # Gradually decrease confidence
                else:
                    self.tracking_status = 'Lost'
                
                # If relocalization is enabled and tracking is lost, attempt recovery
                if self.relocalization_enabled and self.tracking_status == 'Lost':
                    self.attempt_relocalization()
            
            # Log tracking status periodically
            current_time = time.time()
            if current_time - self.last_log_time > 5.0:  # Log every 5 seconds
                self.get_logger().info(
                    f'VSLAM Status: {self.tracking_status}, '
                    f'Features: {features_tracked}, '
                    f'Confidence: {self.tracking_confidence:.2f}'
                )
                self.last_log_time = current_time
                
        except Exception as e:
            self.get_logger().error(f'Error in stereo callback: {e}')
            self.tracking_status = 'Error'

    def extract_and_match_features(self, left_msg, right_msg):
        """Extract and match visual features between stereo images (simulated)"""
        # In a real implementation, this would use Isaac ROS Vision modules
        # For simulation, return a value based on image similarity
        
        # Simulate feature detection by returning a count based on image variance
        # (in a real system, this would be actual feature detection)
        height, width = self.input_height, self.input_width
        expected_features = int(min(1000, (height * width) / 100))  # Rough estimation
        
        # Add some randomness to simulate varying feature density
        features_detected = int(expected_features * np.random.uniform(0.7, 1.3))
        features_detected = max(0, min(features_detected, self.max_features))
        
        return features_detected

    def estimate_pose_change(self, features_tracked):
        """Estimate pose change from feature motion (simulated)"""
        # In real implementation, this would compute pose from matched features
        # For simulation, we'll return a small random pose change
        
        # Simulate pose change based on robot movement and tracking confidence
        dt = time.time() - self.last_update_time
        if dt > 1.0:
            dt = 0.1  # Limit dt to reasonable values
        
        # Simulate small pose changes based on robot movement
        linear_change = np.random.uniform(-0.01, 0.01, 3) * dt  # Small movements per second
        angular_change = np.random.uniform(-0.01, 0.01, 3) * dt  # Small rotations per second
        
        # Add bias based on number of tracked features (more features = more reliable)
        reliability_factor = min(1.0, features_tracked / 100)
        linear_change *= reliability_factor
        angular_change *= reliability_factor
        
        return {'linear': linear_change, 'angular': angular_change}

    def update_pose_estimate(self, pose_change):
        """Update the current pose estimate based on estimated change"""
        # Update position (integrate linear change)
        self.current_pose['position'] += pose_change['linear']
        
        # Update orientation (integrate angular change)
        # Convert small angle approximation to quaternion
        dq = self.small_angle_to_quat(pose_change['angular'])
        current_quat = self.current_pose['orientation']
        
        # Multiply quaternions to get new orientation
        new_quat = self.quat_multiply(dq, current_quat)
        self.current_pose['orientation'] = new_quat
        
        # Normalize to ensure unit quaternion
        norm = np.linalg.norm(self.current_pose['orientation'])
        if norm > 0:
            self.current_pose['orientation'] /= norm
        
        self.last_update_time = time.time()
    
    def small_angle_to_quat(self, angle_vec):
        """Convert small angle vector to quaternion"""
        angle = np.linalg.norm(angle_vec)
        if angle < 1e-6:
            return np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
        
        axis = angle_vec / angle
        half_angle = angle / 2.0
        sin_half = np.sin(half_angle)
        
        return np.array([
            np.cos(half_angle),
            axis[0] * sin_half,
            axis[1] * sin_half,
            axis[2] * sin_half
        ])
    
    def quat_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        
        return np.array([w, x, y, z])

    def attempt_relocalization(self):
        """Attempt to relocalize if tracking is lost"""
        self.get_logger().info('Attempting relocalization...')
        # In a real implementation, this would search for known landmarks or features
        # For simulation, we'll just reset to a known position if we have prior map data
        self.tracking_status = 'Recovering'

    def publish_pose(self):
        """Publish current pose estimate"""
        try:
            # Create pose message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.map_frame
            
            # Set position
            pose_msg.pose.position.x = float(self.current_pose['position'][0])
            pose_msg.pose.position.y = float(self.current_pose['position'][1])
            pose_msg.pose.position.z = float(self.current_pose['position'][2])
            
            # Set orientation
            pose_msg.pose.orientation.w = float(self.current_pose['orientation'][0])
            pose_msg.pose.orientation.x = float(self.current_pose['orientation'][1])
            pose_msg.pose.orientation.y = float(self.current_pose['orientation'][2])
            pose_msg.pose.orientation.z = float(self.current_pose['orientation'][3])
            
            # Publish pose
            self.pose_publisher.publish(pose_msg)
            
            # Create and publish transform
            t = TransformStamped()
            t.header.stamp = pose_msg.header.stamp
            t.header.frame_id = self.map_frame
            t.child_frame_id = self.odom_frame
            
            t.transform.translation.x = pose_msg.pose.position.x
            t.transform.translation.y = pose_msg.pose.position.y
            t.transform.translation.z = pose_msg.pose.position.z
            
            t.transform.rotation.w = pose_msg.pose.orientation.w
            t.transform.rotation.x = pose_msg.pose.orientation.x
            t.transform.rotation.y = pose_msg.pose.orientation.y
            t.transform.rotation.z = pose_msg.pose.orientation.z
            
            self.tf_broadcaster.sendTransform(t)
            
            # Create odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = pose_msg.header.stamp
            odom_msg.header.frame_id = self.map_frame
            odom_msg.child_frame_id = self.base_frame
            
            odom_msg.pose.pose = pose_msg.pose  # Use the same pose
            # For simplicity, we're not setting twist in this example
            
            # Publish odometry
            self.odom_publisher.publish(odom_msg)
            
            # Publish status
            status_msg = String()
            status_msg.data = f'Status: {self.tracking_status}, Confidence: {self.tracking_confidence:.2f}'
            self.status_publisher.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing pose: {e}')

    def get_current_pose(self):
        """Get the current pose estimate"""
        return self.current_pose, self.tracking_status, self.tracking_confidence


def main(args=None):
    rclpy.init(args=args)
    
    vslam_node = IsaacVSLAMNode()
    
    try:
        rclpy.spin(vslam_node)
    except KeyboardInterrupt:
        vslam_node.get_logger().info('Shutting down Isaac VSLAM node...')
    finally:
        vslam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()