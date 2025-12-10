#!/usr/bin/env python3

"""
IMU Data Analysis Node for Digital Twin System

This node processes IMU sensor data to analyze robot state,
detect anomalies, and provide insights about the robot's motion.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import traceback
from typing import Optional


class ImuAnalysisNode(Node):
    def __init__(self):
        super().__init__('imu_analysis_node')
        
        # Declare parameters with defaults
        self.declare_parameter('imu_topic', '/robot/imu')
        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('anomaly_threshold', 2.0)
        self.declare_parameter('max_history_size', 100)
        self.declare_parameter('enable_anomaly_detection', True)
        
        # Get parameters with error handling
        try:
            self.imu_topic = self.get_parameter('imu_topic').value
            self.publish_frequency = float(self.get_parameter('publish_frequency').value)
            self.anomaly_threshold = float(self.get_parameter('anomaly_threshold').value)
            self.max_history_size = int(self.get_parameter('max_history_size').value)
            self.enable_anomaly_detection = self.get_parameter('enable_anomaly_detection').value
            
            # Validate parameters
            if self.publish_frequency <= 0:
                self.get_logger().warn(f'Invalid publish_frequency {self.publish_frequency}, using default 10.0')
                self.publish_frequency = 10.0
            if self.max_history_size <= 0:
                self.get_logger().warn(f'Invalid max_history_size {self.max_history_size}, using default 100')
                self.max_history_size = 100
        except Exception as e:
            self.get_logger().error(f'Error getting parameters: {e}')
            self.get_logger().error(traceback.format_exc())
            # Use defaults
            self.imu_topic = '/robot/imu'
            self.publish_frequency = 10.0
            self.anomaly_threshold = 2.0
            self.max_history_size = 100
            self.enable_anomaly_detection = True
        
        # QoS profile for subscribers
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        
        # Subscribe to IMU data with error handling
        try:
            self.imu_subscription = self.create_subscription(
                Imu,
                self.imu_topic,
                self.imu_callback,
                qos_profile
            )
            self.get_logger().info(f'Subscribed to IMU topic: {self.imu_topic}')
        except Exception as e:
            self.get_logger().error(f'Error subscribing to IMU topic {self.imu_topic}: {e}')
            self.get_logger().error(traceback.format_exc())
            return
        
        # Publishers for analysis results
        try:
            self.roll_publisher = self.create_publisher(Float64, '/robot/imu/roll', 10)
            self.pitch_publisher = self.create_publisher(Float64, '/robot/imu/pitch', 10)
            self.yaw_publisher = self.create_publisher(Float64, '/robot/imu/yaw', 10)
            self.linear_acceleration_magnitude_publisher = self.create_publisher(
                Float64, '/robot/imu/linear_acceleration_magnitude', 10
            )
            self.angular_velocity_magnitude_publisher = self.create_publisher(
                Float64, '/robot/imu/angular_velocity_magnitude', 10
            )
            self.get_logger().info('All publishers created successfully')
        except Exception as e:
            self.get_logger().error(f'Error creating publishers: {e}')
            self.get_logger().error(traceback.format_exc())
            return
        
        # IMU data storage
        self.latest_imu_data = None
        self.imu_data_history = []
        self.error_count = 0
        self.max_error_count_before_shutdown = 100  # Prevent infinite error loops
        
        # Timer for publishing analysis
        try:
            self.timer = self.create_timer(1.0 / self.publish_frequency, self.publish_analysis)
            self.get_logger().info(f'Timer created with frequency: {self.publish_frequency} Hz')
        except Exception as e:
            self.get_logger().error(f'Error creating timer: {e}')
            self.get_logger().error(traceback.format_exc())
            return
        
        self.get_logger().info(f'IMU Analysis Node started, subscribed to {self.imu_topic}')
    
    def imu_callback(self, msg):
        """Process incoming IMU data with error handling"""
        try:
            # Validate incoming message
            if not self.validate_imu_message(msg):
                self.get_logger().warn('Received invalid IMU message, skipping')
                return
            
            self.latest_imu_data = msg
            
            # Store in history for analysis
            self.imu_data_history.append(msg)
            if len(self.imu_data_history) > self.max_history_size:
                self.imu_data_history.pop(0)
            
            # Check for anomalies if enabled
            if self.enable_anomaly_detection:
                self.check_imu_anomalies(msg)
                
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Error in IMU callback: {e}')
            self.get_logger().error(traceback.format_exc())
            
            # Prevent infinite error loops
            if self.error_count > self.max_error_count_before_shutdown:
                self.get_logger().error('Too many errors, shutting down node')
                self.destroy_node()
    
    def validate_imu_message(self, msg: Imu) -> bool:
        """Validate IMU message for valid data"""
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
                return False
            
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
    
    def check_imu_anomalies(self, imu_msg):
        """Check for anomalies in IMU data with error handling"""
        try:
            # Calculate linear acceleration magnitude
            lin_acc_mag = math.sqrt(
                imu_msg.linear_acceleration.x**2 + 
                imu_msg.linear_acceleration.y**2 + 
                imu_msg.linear_acceleration.z**2
            )
            
            # Calculate angular velocity magnitude
            ang_vel_mag = math.sqrt(
                imu_msg.angular_velocity.x**2 + 
                imu_msg.angular_velocity.y**2 + 
                imu_msg.angular_velocity.z**2
            )
            
            # Define reasonable thresholds for anomalies
            lin_acc_threshold = 50.0  # m/s^2 (much higher than gravity)
            ang_vel_threshold = 10.0  # rad/s
            
            if lin_acc_mag > lin_acc_threshold:
                self.get_logger().warn(f'High linear acceleration detected: {lin_acc_mag:.2f} m/s^2')
            
            if ang_vel_mag > ang_vel_threshold:
                self.get_logger().warn(f'High angular velocity detected: {ang_vel_mag:.2f} rad/s')
                
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Error checking IMU anomalies: {e}')
            self.get_logger().error(traceback.format_exc())
    
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
            self.error_count += 1
            self.get_logger().error(f'Error calculating Euler angles: {e}')
            self.get_logger().error(traceback.format_exc())
            # Return zeros as fallback
            return 0.0, 0.0, 0.0
    
    def publish_analysis(self):
        """Publish analysis results with error handling"""
        try:
            if self.latest_imu_data is None:
                self.get_logger().debug('No IMU data to publish analysis')
                return
                
            # Calculate Euler angles
            roll, pitch, yaw = self.calculate_orientation_euler(self.latest_imu_data.orientation)
            
            # Publish roll
            try:
                roll_msg = Float64()
                roll_msg.data = float(roll)
                self.roll_publisher.publish(roll_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing roll: {e}')
                self.get_logger().error(traceback.format_exc())
            
            # Publish pitch
            try:
                pitch_msg = Float64()
                pitch_msg.data = float(pitch)
                self.pitch_publisher.publish(pitch_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing pitch: {e}')
                self.get_logger().error(traceback.format_exc())
            
            # Publish yaw
            try:
                yaw_msg = Float64()
                yaw_msg.data = float(yaw)
                self.yaw_publisher.publish(yaw_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing yaw: {e}')
                self.get_logger().error(traceback.format_exc())
            
            # Calculate and publish linear acceleration magnitude
            try:
                lin_acc_mag = math.sqrt(
                    self.latest_imu_data.linear_acceleration.x**2 + 
                    self.latest_imu_data.linear_acceleration.y**2 + 
                    self.latest_imu_data.linear_acceleration.z**2
                )
                lin_acc_mag_msg = Float64()
                lin_acc_mag_msg.data = float(lin_acc_mag)
                self.linear_acceleration_magnitude_publisher.publish(lin_acc_mag_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing linear acceleration magnitude: {e}')
                self.get_logger().error(traceback.format_exc())
            
            # Calculate and publish angular velocity magnitude
            try:
                ang_vel_mag = math.sqrt(
                    self.latest_imu_data.angular_velocity.x**2 + 
                    self.latest_imu_data.angular_velocity.y**2 + 
                    self.latest_imu_data.angular_velocity.z**2
                )
                ang_vel_mag_msg = Float64()
                ang_vel_mag_msg.data = float(ang_vel_mag)
                self.angular_velocity_magnitude_publisher.publish(ang_vel_mag_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing angular velocity magnitude: {e}')
                self.get_logger().error(traceback.format_exc())
            
            self.get_logger().debug(f'Published IMU analysis: R={roll:.3f}, P={pitch:.3f}, Y={yaw:.3f}')
            
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Error in publish_analysis: {e}')
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    
    imu_analysis_node = ImuAnalysisNode()
    
    try:
        rclpy.spin(imu_analysis_node)
    except KeyboardInterrupt:
        imu_analysis_node.get_logger().info('Received interrupt signal')
    except Exception as e:
        imu_analysis_node.get_logger().error(f'Error during spin: {e}')
        imu_analysis_node.get_logger().error(traceback.format_exc())
    finally:
        imu_analysis_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()