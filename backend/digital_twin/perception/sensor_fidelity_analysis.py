#!/usr/bin/env python3

"""
Sensor Data Analysis Tool for Digital Twin System

This tool compares simulated sensor data to expected behavior and identifies errors.
It analyzes the fidelity of the simulation by comparing against known models or expected values.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray, String
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.distance import cdist
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
import math
import time
from collections import deque
import statistics
import traceback


class SensorFidelityAnalyzer(Node):
    def __init__(self):
        super().__init__('sensor_fidelity_analyzer')
        
        # Declare parameters
        self.declare_parameter('lidar_topic', '/robot/lidar_scan')
        self.declare_parameter('imu_topic', '/robot/imu')
        self.declare_parameter('odom_topic', '/robot/odom')
        self.declare_parameter('analysis_window_size', 100)
        self.declare_parameter('publish_frequency', 1.0)
        self.declare_parameter('enable_visualization', False)
        self.declare_parameter('save_reports', True)
        
        # Get parameters
        try:
            self.lidar_topic = self.get_parameter('lidar_topic').value
            self.imu_topic = self.get_parameter('imu_topic').value
            self.odom_topic = self.get_parameter('odom_topic').value
            self.analysis_window_size = int(self.get_parameter('analysis_window_size').value)
            self.publish_frequency = float(self.get_parameter('publish_frequency').value)
            self.enable_visualization = self.get_parameter('enable_visualization').value
            self.save_reports = self.get_parameter('save_reports').value
            
            # Validate parameters
            if self.publish_frequency <= 0:
                self.get_logger().warn(f'Invalid publish_frequency {self.publish_frequency}, using default 1.0')
                self.publish_frequency = 1.0
            if self.analysis_window_size <= 0:
                self.get_logger().warn(f'Invalid analysis_window_size {self.analysis_window_size}, using default 100')
                self.analysis_window_size = 100
        except Exception as e:
            self.get_logger().error(f'Error getting parameters: {e}')
            self.get_logger().error(traceback.format_exc())
            # Use defaults
            self.lidar_topic = '/robot/lidar_scan'
            self.imu_topic = '/robot/imu'
            self.odom_topic = '/robot/odom'
            self.analysis_window_size = 100
            self.publish_frequency = 1.0
            self.enable_visualization = False
            self.save_reports = True
        
        # QoS profile
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        
        # Create subscribers for sensor streams
        try:
            self.lidar_subscription = self.create_subscription(
                LaserScan,
                self.lidar_topic,
                self.lidar_callback,
                qos_profile
            )
            
            self.imu_subscription = self.create_subscription(
                Imu,
                self.imu_topic,
                self.imu_callback,
                qos_profile
            )
            
            self.odom_subscription = self.create_subscription(
                Odometry,
                self.odom_topic,
                self.odom_callback,
                qos_profile
            )
            
            self.get_logger().info(f'Subscribed to LiDAR: {self.lidar_topic}')
            self.get_logger().info(f'Subscribed to IMU: {self.imu_topic}')
            self.get_logger().info(f'Subscribed to Odometry: {self.odom_topic}')
        except Exception as e:
            self.get_logger().error(f'Error creating subscriptions: {e}')
            self.get_logger().error(traceback.format_exc())
            return
        
        # Publishers for analysis results
        try:
            self.analysis_report_publisher = self.create_publisher(
                String, '/sensor_analysis/report', 10
            )
            self.error_metrics_publisher = self.create_publisher(
                Float64MultiArray, '/sensor_analysis/error_metrics', 10
            )
        except Exception as e:
            self.get_logger().error(f'Error creating publishers: {e}')
            self.get_logger().error(traceback.format_exc())
            return
        
        # Initialize variables
        self.lidar_data_history = deque(maxlen=self.analysis_window_size)
        self.imu_data_history = deque(maxlen=self.analysis_window_size)
        self.odom_data_history = deque(maxlen=self.analysis_window_size)
        
        # CV Bridge for image processing (even though we're not using images in this node)
        try:
            self.cv_bridge = CvBridge()
        except Exception as e:
            self.get_logger().error(f'Error initializing CV Bridge: {e}')
            self.cv_bridge = None
        
        # Timer for analysis
        try:
            self.timer = self.create_timer(1.0 / self.publish_frequency, self.run_analysis)
            self.get_logger().info(f'Sensor analyzer running at {self.publish_frequency} Hz')
        except Exception as e:
            self.get_logger().error(f'Error creating analysis timer: {e}')
            self.get_logger().error(traceback.format_exc())
            return
        
        # Analysis results
        self.analysis_results = {}
        self.analysis_count = 0
        
        # Report storage
        self.reports_dir = 'sensor_analysis_reports'
        
        self.get_logger().info('Sensor Fidelity Analyzer initialized')

    def lidar_callback(self, msg):
        """Process incoming LiDAR data with error handling"""
        try:
            # Validate message
            if not self.validate_lidar_message(msg):
                self.get_logger().warn('Invalid LiDAR message received, skipping')
                return
            
            self.lidar_data_history.append({
                'timestamp': time.time(),
                'ranges': list(msg.ranges),
                'intensities': list(msg.intensities),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max
            })
            self.get_logger().debug(f'Received LiDAR data with {len(msg.ranges)} ranges')
        except Exception as e:
            self.get_logger().error(f'Error in LiDAR callback: {e}')
            self.get_logger().error(traceback.format_exc())

    def imu_callback(self, msg):
        """Process incoming IMU data with error handling"""
        try:
            # Validate message
            if not self.validate_imu_message(msg):
                self.get_logger().warn('Invalid IMU message received, skipping')
                return
            
            self.imu_data_history.append({
                'timestamp': time.time(),
                'orientation': msg.orientation,
                'angular_velocity': msg.angular_velocity,
                'linear_acceleration': msg.linear_acceleration
            })
            self.get_logger().debug('Received IMU data')
        except Exception as e:
            self.get_logger().error(f'Error in IMU callback: {e}')
            self.get_logger().error(traceback.format_exc())

    def odom_callback(self, msg):
        """Process incoming odometry data with error handling"""
        try:
            self.odom_data_history.append({
                'timestamp': time.time(),
                'pose': msg.pose.pose,
                'twist': msg.twist.twist
            })
            self.get_logger().debug('Received odometry data')
        except Exception as e:
            self.get_logger().error(f'Error in odometry callback: {e}')
            self.get_logger().error(traceback.format_exc())

    def validate_lidar_message(self, msg):
        """Validate LiDAR message data"""
        try:
            # Check if ranges array is valid
            if not msg.ranges:
                self.get_logger().debug('LiDAR message has no ranges')
                return False
            return True
        except Exception as e:
            self.get_logger().debug(f'Error validating LiDAR message: {e}')
            return False

    def validate_imu_message(self, msg):
        """Validate IMU message data"""
        try:
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

    def analyze_lidar_data(self):
        """Analyze LiDAR data for fidelity with error handling"""
        try:
            if len(self.lidar_data_history) < 2:
                return {'error': 'Insufficient data for analysis'}
            
            # Get the latest scan
            latest_scan = self.lidar_data_history[-1]
            
            # Calculate valid ranges
            valid_ranges = [
                r for r in latest_scan['ranges'] 
                if latest_scan['range_min'] <= r <= latest_scan['range_max'] and not math.isnan(r)
            ]
            
            if not valid_ranges:
                return {'error': 'No valid ranges in LiDAR data'}
            
            # Calculate metrics
            metrics = {
                'avg_range': statistics.mean(valid_ranges),
                'min_range': min(valid_ranges),
                'max_range': max(valid_ranges),
                'range_variance': statistics.variance(valid_ranges) if len(valid_ranges) > 1 else 0,
                'num_invalid_ranges': len(latest_scan['ranges']) - len(valid_ranges),
                'total_ranges': len(latest_scan['ranges'])
            }
            
            # Compare to expected values (these would come from ground truth in a real implementation)
            expected_min_avg_range = 0.1  # meters
            expected_max_avg_range = 10.0  # meters
            
            range_fidelity = 1.0
            if metrics['avg_range'] < expected_min_avg_range or metrics['avg_range'] > expected_max_avg_range:
                range_fidelity = 0.5  # reduced fidelity score
            
            metrics['range_fidelity_score'] = range_fidelity
            metrics['expected_range_check'] = expected_min_avg_range <= metrics['avg_range'] <= expected_max_avg_range
            
            # Check for temporal consistency
            if len(self.lidar_data_history) >= 2:
                prev_scan = self.lidar_data_history[-2]
                prev_valid_ranges = [
                    r for r in prev_scan['ranges'] 
                    if prev_scan['range_min'] <= r <= prev_scan['range_max'] and not math.isnan(r)
                ]
                
                if prev_valid_ranges:
                    range_change = abs(statistics.mean(valid_ranges) - statistics.mean(prev_valid_ranges))
                    metrics['temporal_consistency'] = range_change < 1.0  # meters change threshold
            
            return metrics
        except Exception as e:
            self.get_logger().error(f'Error analyzing LiDAR data: {e}')
            self.get_logger().error(traceback.format_exc())
            return {'error': f'Analysis failed: {str(e)}'}

    def analyze_imu_data(self):
        """Analyze IMU data for fidelity with error handling"""
        try:
            if len(self.imu_data_history) < 2:
                return {'error': 'Insufficient data for analysis'}
            
            latest_imu = self.imu_data_history[-1]
            
            # Calculate linear acceleration magnitude
            lin_acc_mag = math.sqrt(
                latest_imu['linear_acceleration'].x**2 + 
                latest_imu['linear_acceleration'].y**2 + 
                latest_imu['linear_acceleration'].z**2
            )
            
            # Calculate angular velocity magnitude
            ang_vel_mag = math.sqrt(
                latest_imu['angular_velocity'].x**2 + 
                latest_imu['angular_velocity'].y**2 + 
                latest_imu['angular_velocity'].z**2
            )
            
            # Extract orientation (roll, pitch, yaw)
            try:
                rot = R.from_quat([
                    latest_imu['orientation'].x,
                    latest_imu['orientation'].y,
                    latest_imu['orientation'].z,
                    latest_imu['orientation'].w
                ])
                euler = rot.as_euler('xyz')
                roll, pitch, yaw = euler[0], euler[1], euler[2]
            except Exception as e:
                self.get_logger().error(f'Error calculating Euler angles: {e}')
                roll, pitch, yaw = 0.0, 0.0, 0.0  # default values
            
            # Calculate metrics
            metrics = {
                'linear_acceleration_magnitude': lin_acc_mag,
                'angular_velocity_magnitude': ang_vel_mag,
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw,
                'gravity_check': abs(lin_acc_mag - 9.81) < 2.0  # Check if linear acceleration is close to gravity
            }
            
            # Check angular velocity reasonableness
            metrics['angular_velocity_reasonable'] = ang_vel_mag < 10.0
            
            # Check orientation limits
            metrics['orientation_reasonable'] = abs(roll) < 1.57 and abs(pitch) < 1.57  # Within 90 degrees
            
            return metrics
        except Exception as e:
            self.get_logger().error(f'Error analyzing IMU data: {e}')
            self.get_logger().error(traceback.format_exc())
            return {'error': f'Analysis failed: {str(e)}'}

    def analyze_odom_data(self):
        """Analyze odometry data for fidelity with error handling"""
        try:
            if len(self.odom_data_history) < 2:
                return {'error': 'Insufficient data for analysis'}
            
            # Calculate metrics based on odometry
            latest_odom = self.odom_data_history[-1]
            prev_odom = self.odom_data_history[-2]
            
            # Position change
            dx = latest_odom['pose'].position.x - prev_odom['pose'].position.x
            dy = latest_odom['pose'].position.y - prev_odom['pose'].position.y
            dz = latest_odom['pose'].position.z - prev_odom['pose'].position.z
            pos_change = math.sqrt(dx**2 + dy**2 + dz**2)
            
            # Calculate orientation difference
            try:
                latest_rot = R.from_quat([
                    latest_odom['pose'].orientation.x,
                    latest_odom['pose'].orientation.y,
                    latest_odom['pose'].orientation.z,
                    latest_odom['pose'].orientation.w
                ])
                prev_rot = R.from_quat([
                    prev_odom['pose'].orientation.x,
                    prev_odom['pose'].orientation.y,
                    prev_odom['pose'].orientation.z,
                    prev_odom['pose'].orientation.w
                ])
                # Calculate rotation difference
                rot_diff = latest_rot * prev_rot.inv()
                angle_change = rot_diff.magnitude()
            except Exception as e:
                self.get_logger().error(f'Error calculating orientation difference: {e}')
                angle_change = 0.0  # default value
            
            # Velocity magnitude
            vel_mag = math.sqrt(
                latest_odom['twist'].linear.x**2 +
                latest_odom['twist'].linear.y**2 +
                latest_odom['twist'].linear.z**2
            )
            
            # Calculate metrics
            metrics = {
                'position_change': pos_change,
                'orientation_change': angle_change,
                'linear_velocity_magnitude': vel_mag,
                'angular_velocity_magnitude': math.sqrt(
                    latest_odom['twist'].angular.x**2 +
                    latest_odom['twist'].angular.y**2 +
                    latest_odom['twist'].angular.z**2
                )
            }
            
            return metrics
        except Exception as e:
            self.get_logger().error(f'Error analyzing odometry data: {e}')
            self.get_logger().error(traceback.format_exc())
            return {'error': f'Analysis failed: {str(e)}'}

    def run_analysis(self):
        """Main analysis function with error handling"""
        try:
            # Increment analysis count
            self.analysis_count += 1
            
            # Perform analysis on each sensor data
            lidar_analysis = self.analyze_lidar_data()
            imu_analysis = self.analyze_imu_data()
            odom_analysis = self.analyze_odom_data()
            
            # Compile results
            self.analysis_results = {
                'timestamp': time.time(),
                'analysis_count': self.analysis_count,
                'lidar_analysis': lidar_analysis,
                'imu_analysis': imu_analysis,
                'odom_analysis': odom_analysis
            }
            
            # Generate a summary report
            report = self.generate_analysis_report()
            
            # Publish report
            try:
                report_msg = String()
                report_msg.data = report
                self.analysis_report_publisher.publish(report_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing analysis report: {e}')
            
            # Publish error metrics as array
            try:
                error_metrics = self.extract_error_metrics()
                metrics_msg = Float64MultiArray()
                metrics_msg.data = error_metrics
                self.error_metrics_publisher.publish(metrics_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing error metrics: {e}')
            
            self.get_logger().info(f'Analysis #{self.analysis_count} completed')
            
            # Save report if enabled
            if self.save_reports:
                self.save_analysis_report(report)
                
        except Exception as e:
            self.get_logger().error(f'Error in run_analysis: {e}')
            self.get_logger().error(traceback.format_exc())

    def generate_analysis_report(self):
        """Generate a human-readable analysis report with error handling"""
        try:
            if not self.analysis_results:
                return "No analysis results available"
            
            report_lines = [
                f"Sensor Fidelity Analysis Report - {time.strftime('%Y-%m-%d %H:%M:%S')}",
                f"Analysis Number: {self.analysis_results['analysis_count']}",
                "=" * 60
            ]
            
            # LiDAR Analysis
            lidar = self.analysis_results['lidar_analysis']
            if 'error' not in lidar:
                report_lines.append("LiDAR Analysis:")
                report_lines.append(f"  - Average range: {lidar['avg_range']:.2f}m")
                report_lines.append(f"  - Min range: {lidar['min_range']:.2f}m")
                report_lines.append(f"  - Max range: {lidar['max_range']:.2f}m")
                report_lines.append(f"  - Range fidelity: {'PASS' if lidar['expected_range_check'] else 'FAIL'}")
                if 'temporal_consistency' in lidar:
                    report_lines.append(f"  - Temporal consistency: {lidar['temporal_consistency']}")
            else:
                report_lines.append(f"LiDAR Analysis: {lidar['error']}")
            
            # IMU Analysis
            imu = self.analysis_results['imu_analysis']
            if 'error' not in imu:
                report_lines.append("\nIMU Analysis:")
                report_lines.append(f"  - Linear acceleration magnitude: {imu['linear_acceleration_magnitude']:.2f} m/s²")
                report_lines.append(f"  - Angular velocity magnitude: {imu['angular_velocity_magnitude']:.2f} rad/s")
                report_lines.append(f"  - Roll: {imu['roll']:.3f}, Pitch: {imu['pitch']:.3f}, Yaw: {imu['yaw']:.3f}")
                report_lines.append(f"  - Gravity check: {'PASS' if imu['gravity_check'] else 'FAIL'}")
            else:
                report_lines.append(f"IMU Analysis: {imu['error']}")
            
            # Odometry Analysis
            odom = self.analysis_results['odom_analysis']
            if 'error' not in odom:
                report_lines.append("\nOdometry Analysis:")
                report_lines.append(f"  - Position change: {odom['position_change']:.3f}m")
                report_lines.append(f"  - Orientation change: {odom['orientation_change']:.3f} rad")
                report_lines.append(f"  - Velocity magnitude: {odom['linear_velocity_magnitude']:.3f} m/s")
            else:
                report_lines.append(f"Odometry Analysis: {odom['error']}")
            
            return "\n".join(report_lines)
        except Exception as e:
            self.get_logger().error(f'Error generating analysis report: {e}')
            self.get_logger().error(traceback.format_exc())
            return f"Error generating report: {str(e)}"

    def extract_error_metrics(self):
        """Extract error metrics for quantitative analysis with error handling"""
        try:
            metrics = []
            
            # Extract relevant metrics from analysis
            lidar = self.analysis_results.get('lidar_analysis', {})
            imu = self.analysis_results.get('imu_analysis', {})
            odom = self.analysis_results.get('odom_analysis', {})
            
            # LiDAR metrics
            metrics.append(lidar.get('avg_range', 0.0))
            metrics.append(1.0 if lidar.get('expected_range_check', False) else 0.0)
            
            # IMU metrics
            metrics.append(imu.get('linear_acceleration_magnitude', 0.0))
            metrics.append(1.0 if imu.get('gravity_check', False) else 0.0)
            
            # Odometry metrics
            metrics.append(odom.get('linear_velocity_magnitude', 0.0))
            
            return metrics
        except Exception as e:
            self.get_logger().error(f'Error extracting error metrics: {e}')
            self.get_logger().error(traceback.format_exc())
            return [0.0] * 5  # Return default metrics

    def save_analysis_report(self, report):
        """Save analysis report to file"""
        try:
            import os
            os.makedirs(self.reports_dir, exist_ok=True)
            
            filename = f"{self.reports_dir}/analysis_report_{self.analysis_results['analysis_count']:04d}.txt"
            with open(filename, 'w') as f:
                f.write(report)
                
            self.get_logger().info(f"Analysis report saved to {filename}")
        except Exception as e:
            self.get_logger().error(f'Error saving analysis report: {e}')
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    
    analyzer = SensorFidelityAnalyzer()
    
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        analyzer.get_logger().info('Received interrupt signal')
    except Exception as e:
        analyzer.get_logger().error(f'Error during spin: {e}')
        analyzer.get_logger().error(traceback.format_exc())
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()