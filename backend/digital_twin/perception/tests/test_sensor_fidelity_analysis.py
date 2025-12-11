#!/usr/bin/env python3
"""
Sensor fidelity analysis test for the Digital Twin project.

This test analyzes the fidelity of simulated sensor data compared to expected values.
"""
import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Vector3
from builtin_interfaces.msg import Time
import time
import math
import numpy as np


class SensorFidelityTest(Node):
    def __init__(self):
        super().__init__('sensor_fidelity_test')
        
        # Initialize flags and data storage
        self.lidar_data_samples = []
        self.imu_data_samples = []
        self.lidar_test_complete = False
        self.imu_test_complete = False
        
        # Maximum samples to collect for analysis
        self.max_samples = 50
        
        # Create subscribers for sensor data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/robot/lidar_scan',
            self.lidar_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/robot/imu/data',
            self.imu_callback,
            10
        )
        
        # Timeout for tests
        self.test_timeout = 15.0  # seconds

    def lidar_callback(self, msg):
        """Callback for LiDAR data."""
        if len(self.lidar_data_samples) < self.max_samples:
            # Store key attributes of the scan for analysis
            sample = {
                'header': msg.header,
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'time_increment': msg.time_increment,
                'scan_time': msg.scan_time,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
                'ranges': list(msg.ranges),  # Convert to list for storage
                'intensities': list(msg.intensities) if len(msg.intensities) > 0 else []
            }
            self.lidar_data_samples.append(sample)
            
            self.get_logger().debug(f"Lidar sample {len(self.lidar_data_samples)} collected")
            
            # Mark test as complete if we've collected enough samples
            if len(self.lidar_data_samples) >= self.max_samples:
                self.lidar_test_complete = True

    def imu_callback(self, msg):
        """Callback for IMU data."""
        if len(self.imu_data_samples) < self.max_samples:
            # Store key attributes of the IMU data for analysis
            sample = {
                'header': msg.header,
                'orientation': {
                    'x': msg.orientation.x,
                    'y': msg.orientation.y,
                    'z': msg.orientation.z,
                    'w': msg.orientation.w
                },
                'angular_velocity': {
                    'x': msg.angular_velocity.x,
                    'y': msg.angular_velocity.y,
                    'z': msg.angular_velocity.z
                },
                'linear_acceleration': {
                    'x': msg.linear_acceleration.x,
                    'y': msg.linear_acceleration.y,
                    'z': msg.linear_acceleration.z
                }
            }
            self.imu_data_samples.append(sample)
            
            self.get_logger().debug(f"IMU sample {len(self.imu_data_samples)} collected")
            
            # Mark test as complete if we've collected enough samples
            if len(self.imu_data_samples) >= self.max_samples:
                self.imu_test_complete = True

    def analyze_lidar_fidelity(self):
        """Analyze LiDAR sensor fidelity."""
        if len(self.lidar_data_samples) == 0:
            self.get_logger().error("No LiDAR data collected for analysis")
            return False
            
        # Check if LiDAR parameters are consistent across samples
        first_sample = self.lidar_data_samples[0]
        for i, sample in enumerate(self.lidar_data_samples[1:], 1):
            if (sample['angle_min'] != first_sample['angle_min'] or
                sample['angle_max'] != first_sample['angle_max'] or
                sample['angle_increment'] != first_sample['angle_increment'] or
                sample['range_min'] != first_sample['range_min'] or
                sample['range_max'] != first_sample['range_max']):
                self.get_logger().warn(f"LIDAR parameters changed between sample 0 and {i}")
        
        # Analyze range data for typical LiDAR characteristics
        all_ranges = []
        valid_ranges = []
        
        for sample in self.lidar_data_samples:
            all_ranges.extend(sample['ranges'])
            # Filter out invalid ranges (inf, nan, etc.)
            valid_ranges.extend([r for r in sample['ranges'] if r != float('inf') and not math.isnan(r) and r >= sample['range_min'] and r <= sample['range_max']])
        
        if len(valid_ranges) == 0:
            self.get_logger().error("No valid range data found in LiDAR samples")
            return False
        
        # Calculate statistics
        mean_range = sum(valid_ranges) / len(valid_ranges)
        range_variance = sum((r - mean_range) ** 2 for r in valid_ranges) / len(valid_ranges)
        
        self.get_logger().info(f"LiDAR Analysis:")
        self.get_logger().info(f"  Total samples: {len(self.lidar_data_samples)}")
        self.get_logger().info(f"  Total ranges analyzed: {len(all_ranges)}")
        self.get_logger().info(f"  Valid ranges: {len(valid_ranges)}")
        self.get_logger().info(f"  Mean valid range: {mean_range:.3f}m")
        self.get_logger().info(f"  Range variance: {range_variance:.6f}")
        
        # Check for expected LiDAR characteristics
        # 1. Most values should be within range_min and range_max
        in_range_pct = len(valid_ranges) / len(all_ranges) * 100
        self.get_logger().info(f"  Valid range percentage: {in_range_pct:.1f}%")
        
        # 2. Check if there are reasonable variations in the data (not all the same)
        unique_ranges = len(set([round(r, 3) for r in valid_ranges]))  # Round to 3 decimal places
        diversity_ratio = unique_ranges / len(valid_ranges) if len(valid_ranges) > 0 else 0
        self.get_logger().info(f"  Range diversity ratio: {diversity_ratio:.3f}")
        
        # Criteria for passing: at least 50% of readings should be valid and there should be good diversity
        if in_range_pct >= 50.0 and diversity_ratio >= 0.1:
            self.get_logger().info("  ✓ LiDAR fidelity analysis passed")
            return True
        else:
            self.get_logger().info("  ✗ LiDAR fidelity analysis failed - insufficient valid or diverse data")
            return False

    def analyze_imu_fidelity(self):
        """Analyze IMU sensor fidelity."""
        if len(self.imu_data_samples) == 0:
            self.get_logger().error("No IMU data collected for analysis")
            return False
        
        # Check orientations are normalized quaternions
        orientations_valid = 0
        total_orientations = 0
        
        # Check angular velocities and linear accelerations
        angular_velocities = []
        linear_accelerations = []
        
        for sample in self.imu_data_samples:
            # Check quaternion normalization
            quat = sample['orientation']
            norm = math.sqrt(quat['x']**2 + quat['y']**2 + quat['z']**2 + quat['w']**2)
            total_orientations += 1
            if abs(norm - 1.0) < 0.1:  # Allow some tolerance
                orientations_valid += 1
            
            # Collect angular velocity and linear acceleration data
            ang_vel = sample['angular_velocity']
            lin_acc = sample['linear_acceleration']
            
            angular_velocities.append(math.sqrt(ang_vel['x']**2 + ang_vel['y']**2 + ang_vel['z']**2))
            linear_accelerations.append(math.sqrt(lin_acc['x']**2 + lin_acc['y']**2 + lin_acc['z']**2))
        
        # Calculate statistics
        if len(angular_velocities) > 0:
            avg_ang_vel = sum(angular_velocities) / len(angular_velocities)
            avg_lin_acc = sum(linear_accelerations) / len(linear_accelerations)
        else:
            avg_ang_vel = 0.0
            avg_lin_acc = 0.0
        
        self.get_logger().info(f"IMU Analysis:")
        self.get_logger().info(f"  Total samples: {len(self.imu_data_samples)}")
        self.get_logger().info(f"  Valid orientations: {orientations_valid}/{total_orientations} ({orientations_valid/total_orientations*100:.1f}%)")
        self.get_logger().info(f"  Average angular velocity magnitude: {avg_ang_vel:.6f}")
        self.get_logger().info(f"  Average linear acceleration magnitude: {avg_lin_acc:.6f}")
        
        # Criteria for passing: at least 90% of orientations should be valid quaternions
        if orientations_valid / total_orientations >= 0.9:
            self.get_logger().info("  ✓ IMU fidelity analysis passed")
            return True
        else:
            self.get_logger().info("  ✗ IMU fidelity analysis failed - invalid quaternion data")
            return False

    def test_sensor_fidelity_analysis(self):
        """
        Test the fidelity of simulated sensor data.
        
        Checks:
        1. LiDAR data has expected characteristics (range limits, diversity)
        2. IMU data has valid quaternion orientations
        3. Sensor data varies appropriately over time
        """
        self.get_logger().info("Starting sensor fidelity analysis test...")
        self.get_logger().info(f"Collecting sensor data for {self.test_timeout} seconds...")
        
        # Wait for data collection
        start_time = time.time()
        while time.time() - start_time < self.test_timeout:
            time.sleep(0.1)
            
            # Check if we've collected enough samples for both sensors
            if self.lidar_test_complete and self.imu_test_complete:
                self.get_logger().info("Required samples collected, stopping early...")
                break
        
        # Analyze LiDAR fidelity
        lidar_pass = self.analyze_lidar_fidelity()
        
        # Analyze IMU fidelity
        imu_pass = self.analyze_imu_fidelity()
        
        # Overall result
        all_pass = lidar_pass and imu_pass
        
        if all_pass:
            self.get_logger().info("✓ Sensor fidelity analysis test PASSED")
            return True
        else:
            self.get_logger().info("✗ Sensor fidelity analysis test FAILED")
            if not lidar_pass:
                self.get_logger().info("  - LiDAR analysis failed")
            if not imu_pass:
                self.get_logger().info("  - IMU analysis failed")
            return False


def main():
    rclpy.init()
    
    try:
        test_node = SensorFidelityTest()
        
        # Run the sensor fidelity analysis test
        result = test_node.test_sensor_fidelity_analysis()
        
        if result:
            print("\n✓ Sensor fidelity analysis test completed successfully!")
            exit_code = 0
        else:
            print("\n✗ Sensor fidelity analysis test failed!")
            exit_code = 1
            
    except KeyboardInterrupt:
        print("Test interrupted by user")
        exit_code = 1
    finally:
        rclpy.shutdown()
        
    return exit_code


if __name__ == '__main__':
    exit(main())