#!/usr/bin/env python3
"""
Sensor validation test for the Digital Twin project.

This test verifies that sensor topics are publishing data correctly.
"""
import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header
import time
import threading


class SensorValidationTest(Node):
    def __init__(self):
        super().__init__('sensor_validation_test')
        
        # Initialize flags to track if messages are received
        self.lidar_received = False
        self.camera_received = False
        self.imu_received = False
        self.lidar_msg_count = 0
        self.camera_msg_count = 0
        self.imu_msg_count = 0
        
        # Create subscribers for different sensor types
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/robot/lidar_scan',  # Typical topic name for LiDAR
            self.lidar_callback,
            10
        )
        
        self.camera_sub = self.create_subscription(
            Image,
            '/robot/camera/image_raw',  # Typical topic name for camera
            self.camera_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/robot/imu/data',  # Typical topic name for IMU
            self.imu_callback,
            10
        )
        
        # Timeout for tests
        self.test_timeout = 10.0  # seconds

    def lidar_callback(self, msg):
        """Callback for LiDAR data."""
        self.lidar_received = True
        self.lidar_msg_count += 1
        self.get_logger().debug(f"LiDAR message received, count: {self.lidar_msg_count}")

    def camera_callback(self, msg):
        """Callback for camera data."""
        self.camera_received = True
        self.camera_msg_count += 1
        self.get_logger().debug(f"Camera message received, count: {self.camera_msg_count}")

    def imu_callback(self, msg):
        """Callback for IMU data."""
        self.imu_received = True
        self.imu_msg_count += 1
        self.get_logger().debug(f"IMU message received, count: {self.imu_msg_count}")

    def test_sensor_topics_publishing(self):
        """
        Test that sensor topics are publishing data.
        
        Checks:
        1. LiDAR sensor publishes LaserScan messages
        2. Camera sensor publishes Image messages
        3. IMU sensor publishes IMU messages
        """
        self.get_logger().info("Starting sensor validation test...")
        self.get_logger().info("Listening for sensor data for 10 seconds...")
        
        # Wait for 10 seconds to collect sensor data
        start_time = time.time()
        while time.time() - start_time < self.test_timeout:
            time.sleep(0.1)
            
            # Check if we've received data from all sensors
            if self.lidar_received and self.camera_received and self.imu_received:
                self.get_logger().info("All sensor data received, stopping early...")
                break
        
        # Print summary
        self.get_logger().info(f"Sensor message counts:")
        self.get_logger().info(f"  LiDAR: {self.lidar_msg_count}")
        self.get_logger().info(f"  Camera: {self.camera_msg_count}")
        self.get_logger().info(f"  IMU: {self.imu_msg_count}")
        
        # Check if all sensors published data
        all_sensors_working = (
            self.lidar_received and 
            self.camera_received and 
            self.imu_received
        )
        
        if all_sensors_working:
            self.get_logger().info("✓ Sensor validation test PASSED")
            return True
        else:
            failed_sensors = []
            if not self.lidar_received:
                failed_sensors.append("LiDAR")
            if not self.camera_received:
                failed_sensors.append("Camera")
            if not self.imu_received:
                failed_sensors.append("IMU")
                
            self.get_logger().error(f"✗ Sensor validation test FAILED - no data from: {', '.join(failed_sensors)}")
            return False


def main():
    rclpy.init()
    
    try:
        test_node = SensorValidationTest()
        
        # Run the sensor validation test
        result = test_node.test_sensor_topics_publishing()
        
        if result:
            print("\n✓ Sensor validation test completed successfully!")
            exit_code = 0
        else:
            print("\n✗ Sensor validation test failed!")
            exit_code = 1
            
    except KeyboardInterrupt:
        print("Test interrupted by user")
        exit_code = 1
    finally:
        rclpy.shutdown()
        
    return exit_code


if __name__ == '__main__':
    exit(main())