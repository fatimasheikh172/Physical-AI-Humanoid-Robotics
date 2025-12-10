#!/usr/bin/env python3

"""
Perception Demo for Digital Twin System

This demo consumes simulated sensor streams (LiDAR, depth camera, IMU) 
to demonstrate perception capabilities in the digital twin environment.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from std_msgs.msg import String, Float64
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import TransformStamped
import traceback
import math
from typing import Optional


class PerceptionDemoNode(Node):
    def __init__(self):
        super().__init__('perception_demo_node')
        
        # Declare parameters
        self.declare_parameter('lidar_topic', '/robot/lidar_scan')
        self.declare_parameter('depth_camera_topic', '/robot/depth_camera/image_raw')
        self.declare_parameter('imu_topic', '/robot/imu')
        self.declare_parameter('demo_frequency', 1.0)
        self.declare_parameter('max_history_size', 50)
        self.declare_parameter('enable_visualization', True)
        
        # Get parameters
        try:
            self.lidar_topic = self.get_parameter('lidar_topic').value
            self.depth_camera_topic = self.get_parameter('depth_camera_topic').value
            self.imu_topic = self.get_parameter('imu_topic').value
            self.demo_frequency = float(self.get_parameter('demo_frequency').value)
            self.max_history_size = int(self.get_parameter('max_history_size').value)
            self.enable_visualization = self.get_parameter('enable_visualization').value
            
            # Validate parameters
            if self.demo_frequency <= 0:
                self.get_logger().warn(f'Invalid demo_frequency {self.demo_frequency}, using default 1.0')
                self.demo_frequency = 1.0
        except Exception as e:
            self.get_logger().error(f'Error getting parameters: {e}')
            self.get_logger().error(traceback.format_exc())
            # Use defaults
            self.lidar_topic = '/robot/lidar_scan'
            self.depth_camera_topic = '/robot/depth_camera/image_raw'
            self.imu_topic = '/robot/imu'
            self.demo_frequency = 1.0
            self.max_history_size = 50
            self.enable_visualization = True
        
        # QoS profiles for different sensor types
        default_qos = QoSProfile(depth=10)
        default_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        
        # Create subscribers for sensor streams
        try:
            self.lidar_subscription = self.create_subscription(
                LaserScan,
                self.lidar_topic,
                self.lidar_callback,
                default_qos
            )
            
            self.depth_subscription = self.create_subscription(
                Image,
                self.depth_camera_topic,
                self.depth_callback,
                default_qos
            )
            
            self.imu_subscription = self.create_subscription(
                Imu,
                self.imu_topic,
                self.imu_callback,
                default_qos
            )
            
            self.get_logger().info(f'Subscribed to LiDAR: {self.lidar_topic}')
            self.get_logger().info(f'Subscribed to Depth: {self.depth_camera_topic}')
            self.get_logger().info(f'Subscribed to IMU: {self.imu_topic}')
        except Exception as e:
            self.get_logger().error(f'Error creating subscriptions: {e}')
            self.get_logger().error(traceback.format_exc())
            return
        
        # Publisher for demo status
        try:
            self.demo_status_publisher = self.create_publisher(String, '/perception/demo_status', 10)
        except Exception as e:
            self.get_logger().error(f'Error creating publisher: {e}')
            self.get_logger().error(traceback.format_exc())
            return
        
        # Initialize variables to store sensor data
        self.latest_lidar_data = None
        self.latest_depth_data = None
        self.latest_imu_data = None
        
        # Data history for analysis
        self.lidar_history = []
        self.imu_history = []
        
        # CV Bridge for image processing
        try:
            self.cv_bridge = CvBridge()
        except Exception as e:
            self.get_logger().error(f'Error initializing CV Bridge: {e}')
            self.cv_bridge = None
        
        # Timer for running perception demo
        try:
            self.timer = self.create_timer(1.0 / self.demo_frequency, self.run_perception_demo)
            self.get_logger().info(f'Perception Demo running at {self.demo_frequency} Hz')
        except Exception as e:
            self.get_logger().error(f'Error creating timer: {e}')
            self.get_logger().error(traceback.format_exc())
            return
        
        self.get_logger().info('Perception Demo Node initialized')

    def lidar_callback(self, msg):
        """Process incoming LiDAR data with error handling"""
        try:
            # Validate message
            if not self.validate_lidar_message(msg):
                self.get_logger().warn('Invalid LiDAR message received, skipping')
                return
                
            self.latest_lidar_data = msg
            
            # Add to history
            self.lidar_history.append(msg)
            if len(self.lidar_history) > self.max_history_size:
                self.lidar_history.pop(0)
                
            self.get_logger().debug('Received and stored LiDAR data')
        except Exception as e:
            self.get_logger().error(f'Error in LiDAR callback: {e}')
            self.get_logger().error(traceback.format_exc())

    def depth_callback(self, msg):
        """Process incoming depth camera data with error handling"""
        try:
            self.latest_depth_data = msg
            self.get_logger().debug('Received depth camera data')
        except Exception as e:
            self.get_logger().error(f'Error in depth callback: {e}')
            self.get_logger().error(traceback.format_exc())

    def imu_callback(self, msg):
        """Process incoming IMU data with error handling"""
        try:
            # Validate message
            if not self.validate_imu_message(msg):
                self.get_logger().warn('Invalid IMU message received, skipping')
                return
                
            self.latest_imu_data = msg
            
            # Add to history
            self.imu_history.append(msg)
            if len(self.imu_history) > self.max_history_size:
                self.imu_history.pop(0)
                
            self.get_logger().debug('Received and stored IMU data')
        except Exception as e:
            self.get_logger().error(f'Error in IMU callback: {e}')
            self.get_logger().error(traceback.format_exc())

    def validate_lidar_message(self, msg):
        """Validate LiDAR message data"""
        try:
            # Check if ranges array is valid
            if not msg.ranges:
                self.get_logger().debug('LiDAR message has no ranges')
                return False
                
            # Check if values are within expected range
            for r in msg.ranges:
                if not (msg.range_min <= r <= msg.range_max) and not math.isnan(r):
                    # Values can be out of range (for invalid readings), that's okay
                    pass
            
            return True
        except Exception as e:
            self.get_logger().debug(f'Error validating LiDAR message: {e}')
            return False

    def validate_imu_message(self, msg):
        """Validate IMU message data"""
        try:
            # Check orientation quaternion is normalized
            norm = math.sqrt(
                msg.orientation.x**2 + 
                msg.orientation.y**2 + 
                msg.orientation.z**2 + 
                msg.orientation.w**2
            )
            if abs(norm - 1.0) > 0.1:  # Tolerance for normalization
                self.get_logger().debug(f'Quaternion not normalized: {norm}')
                # Return True anyway, as we can handle normalization in calculations
                return True
            
            # Check for NaN or Inf values
            for val in [
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
            ]:
                if math.isnan(val) or math.isinf(val):
                    self.get_logger().debug(f'Invalid value in IMU message: {val}')
                    return False
                    
            return True
        except Exception as e:
            self.get_logger().debug(f'Error validating IMU message: {e}')
            return False

    def calculate_orientation_euler(self, orientation):
        """Convert quaternion to Euler angles (roll, pitch, yaw) with error handling"""
        try:
            # Validate quaternion
            norm = math.sqrt(
                orientation.x**2 + 
                orientation.y**2 + 
                orientation.z**2 + 
                orientation.w**2
            )
            if abs(norm - 1.0) > 0.1:
                self.get_logger().warn(f'Quaternion not normalized in euler calculation: {norm}')
                # Normalize quaternion
                if norm != 0:
                    orientation.x /= norm
                    orientation.y /= norm
                    orientation.z /= norm
                    orientation.w /= norm
            
            # Create rotation object from quaternion
            rot = R.from_quat([
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w
            ])
            
            # Get Euler angles in radians
            euler = rot.as_euler('xyz')
            
            return euler[0], euler[1], euler[2]  # roll, pitch, yaw
        except Exception as e:
            self.get_logger().error(f'Error calculating Euler angles: {e}')
            self.get_logger().error(traceback.format_exc())
            # Return zeros as fallback
            return 0.0, 0.0, 0.0

    def detect_obstacles_lidar(self, lidar_data):
        """Detect obstacles using LiDAR data with error handling"""
        if lidar_data is None:
            return []
            
        try:
            # Find distances less than threshold to detect obstacles
            threshold = 1.0  # meters
            valid_ranges = []
            valid_indices = []
            
            for i, r in enumerate(lidar_data.ranges):
                if lidar_data.range_min <= r <= lidar_data.range_max:
                    valid_ranges.append(r)
                    valid_indices.append(i)
            
            obstacle_indices = [i for i, r in enumerate(valid_ranges) if r < threshold]
            
            obstacles = []
            for idx in obstacle_indices:
                original_idx = valid_indices[idx]
                angle = lidar_data.angle_min + original_idx * lidar_data.angle_increment
                distance = valid_ranges[idx]
                obstacles.append({
                    'angle': angle,
                    'distance': distance,
                    'x': distance * math.cos(angle),
                    'y': distance * math.sin(angle)
                })
            
            return obstacles
        except Exception as e:
            self.get_logger().error(f'Error detecting obstacles from LiDAR: {e}')
            self.get_logger().error(traceback.format_exc())
            return []

    def process_depth_image(self, depth_image_msg):
        """Process depth camera image with error handling"""
        if depth_image_msg is None or self.cv_bridge is None:
            return None
            
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(depth_image_msg, '32FC1')
            
            # Find distances in the center region of the image
            height, width = cv_image.shape
            center_region = cv_image[height//3:2*height//3, width//3:2*width//3]
            
            # Calculate average distance in center region (avoiding invalid values)
            valid_depths = center_region[np.isfinite(center_region) & (center_region > 0)]
            if len(valid_depths) > 0:
                avg_distance = np.mean(valid_depths)
                return avg_distance
            else:
                return None
                
        except cv2.error as e:
            self.get_logger().error(f'OpenCV error processing depth image: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')
            self.get_logger().error(traceback.format_exc())
            return None

    def run_perception_demo(self):
        """Main perception demo function with error handling"""
        try:
            demo_status_msg = String()
            
            # Check if we have all required sensor data
            if not all([self.latest_lidar_data, self.latest_depth_data, self.latest_imu_data]):
                demo_status_msg.data = 'Waiting for sensor data...'
                # Only publish if we have a publisher
                try:
                    self.demo_status_publisher.publish(demo_status_msg)
                except Exception as e:
                    self.get_logger().error(f'Error publishing demo status: {e}')
                return

            # Process LiDAR data - detect obstacles
            obstacles = self.detect_obstacles_lidar(self.latest_lidar_data)
            obstacle_count = len(obstacles)
            
            # Process depth camera data
            avg_depth_distance = self.process_depth_image(self.latest_depth_data)
            
            # Process IMU data - get orientation
            roll, pitch, yaw = self.calculate_orientation_euler(self.latest_imu_data.orientation)
            
            # Perform basic fusion of sensor data
            perception_results = self.fuse_sensor_data(obstacles, avg_depth_distance, roll, pitch, yaw)
            
            # Log perception results
            self.get_logger().info(f'Perception Demo Results:')
            self.get_logger().info(f'  - Obstacles detected: {obstacle_count}')
            if avg_depth_distance is not None:
                self.get_logger().info(f'  - Average depth distance (center): {avg_depth_distance:.2f}m')
            self.get_logger().info(f'  - Robot orientation: R={roll:.2f}, P={pitch:.2f}, Y={yaw:.2f}')
            self.get_logger().info(f'  - Fusion status: {perception_results}')
            
            # Publish demo status
            status_str = f'Perception demo running... Obstacles: {obstacle_count}'
            if avg_depth_distance is not None:
                status_str += f', Avg depth: {avg_depth_distance:.2f}m'
            demo_status_msg.data = status_str
            
            try:
                self.demo_status_publisher.publish(demo_status_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing demo status: {e}')
                
        except Exception as e:
            self.get_logger().error(f'Error in perception demo: {e}')
            self.get_logger().error(traceback.format_exc())

    def fuse_sensor_data(self, obstacles, depth_distance, roll, pitch, yaw):
        """Fuse data from multiple sensors for comprehensive understanding with error handling"""
        try:
            results = {
                'obstacle_warning': len(obstacles) > 0,
                'obstacle_count': len(obstacles),
                'obstacle_positions': [(obs['x'], obs['y']) for obs in obstacles][:5],  # First 5 obstacles
                'close_depth_object': depth_distance is not None and depth_distance < 2.0,  # Within 2m
                'orientation_stable': abs(roll) < 0.3 and abs(pitch) < 0.3,  # Within ~17 degrees
                'timestamp': self.get_clock().now().to_msg()
            }
            
            # Generate perception summary
            summary_parts = []
            if results['obstacle_warning']:
                summary_parts.append(f'{len(obstacles)} obstacle(s) detected')
            if results['close_depth_object']:
                summary_parts.append('close object detected by depth camera')
            if not results['orientation_stable']:
                summary_parts.append('robot orientation unstable')
            
            if summary_parts:
                results['summary'] = '; '.join(summary_parts)
            else:
                results['summary'] = 'environment appears clear and robot stable'
            
            return results
        except Exception as e:
            self.get_logger().error(f'Error fusing sensor data: {e}')
            self.get_logger().error(traceback.format_exc())
            # Return empty results as fallback
            return {
                'obstacle_warning': False,
                'obstacle_count': 0,
                'obstacle_positions': [],
                'close_depth_object': False,
                'orientation_stable': True,
                'summary': 'error in sensor fusion',
                'timestamp': self.get_clock().now().to_msg()
            }


def main(args=None):
    rclpy.init(args=args)
    
    perception_demo_node = PerceptionDemoNode()
    
    try:
        rclpy.spin(perception_demo_node)
    except KeyboardInterrupt:
        perception_demo_node.get_logger().info('Received interrupt signal')
    except Exception as e:
        perception_demo_node.get_logger().error(f'Error during spin: {e}')
        perception_demo_node.get_logger().error(traceback.format_exc())
    finally:
        perception_demo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()