#!/usr/bin/env python3
"""
Sensor Validation Script for Digital Twin Simulation

This script validates that all sensors publish correct message types and data is recordable:
- LiDAR: sensor_msgs/LaserScan with appropriate ranges
- Depth Camera: sensor_msgs/Image and sensor_msgs/PointCloud2
- IMU: sensor_msgs/Imu
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, PointCloud2, Imu
from std_msgs.msg import String
import time


class SensorValidator(Node):
    def __init__(self):
        super().__init__('sensor_validator')
        
        # Store latest sensor data
        self.lidar_data = None
        self.depth_image = None
        self.imu_data = None
        self.point_cloud = None
        
        # Create subscribers for each sensor type
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/robot/laser_scan',
            self.lidar_callback,
            10
        )
        
        self.depth_img_sub = self.create_subscription(
            Image,
            '/robot/depth_camera/image_raw',
            self.depth_img_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/robot/imu',
            self.imu_callback,
            10
        )
        
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/robot/depth_camera/depth/points',
            self.pointcloud_callback,
            10
        )
        
        self.get_logger().info('Sensor Validator Node initialized')
    
    def lidar_callback(self, msg):
        """Callback for LiDAR data"""
        self.lidar_data = msg
        self.get_logger().debug(f'Received LiDAR data with {len(msg.ranges)} ranges')
    
    def depth_img_callback(self, msg):
        """Callback for depth camera image"""
        self.depth_image = msg
        self.get_logger().debug(f'Received depth image: {msg.width}x{msg.height}')
    
    def imu_callback(self, msg):
        """Callback for IMU data"""
        self.imu_data = msg
        self.get_logger().debug('Received IMU data')
    
    def pointcloud_callback(self, msg):
        """Callback for point cloud data"""
        self.point_cloud = msg
        self.get_logger().debug(f'Received point cloud with {msg.height * msg.width} points')
    
    def validate_lidar_data(self):
        """Validate LiDAR data"""
        if self.lidar_data is None:
            self.get_logger().warn('No LiDAR data received')
            return False
        
        # Check that ranges array is not empty and has reasonable values
        if len(self.lidar_data.ranges) == 0:
            self.get_logger().error('LiDAR ranges array is empty')
            return False
        
        # Check for invalid values in ranges (NaN or infinity)
        for r in self.lidar_data.ranges:
            if r != r or r == float('inf'):  # Check for NaN or infinity
                self.get_logger().error(f'Invalid range value: {r}')
                return False
        
        # Check that values are within expected range
        for r in self.lidar_data.ranges:
            if r < self.lidar_data.range_min or r > self.lidar_data.range_max:
                if r != float('inf'):  # infinity is valid for "no object detected"
                    self.get_logger().warn(f'Range {r} outside expected bounds [{self.lidar_data.range_min}, {self.lidar_data.range_max}]')
        
        self.get_logger().info('LiDAR validation PASSED')
        return True
    
    def validate_depth_image(self):
        """Validate depth camera data"""
        if self.depth_image is None:
            self.get_logger().warn('No depth image data received')
            return False
        
        # Check that image has valid dimensions
        if self.depth_image.width <= 0 or self.depth_image.height <= 0:
            self.get_logger().error('Invalid image dimensions')
            return False
        
        # Check that data size matches expected size
        expected_size = self.depth_image.width * self.depth_image.height * 3  # RGB
        if len(self.depth_image.data) != expected_size:
            self.get_logger().error(f'Image data size mismatch: expected {expected_size}, got {len(self.depth_image.data)}')
            return False
        
        self.get_logger().info('Depth image validation PASSED')
        return True
    
    def validate_imu_data(self):
        """Validate IMU data"""
        if self.imu_data is None:
            self.get_logger().warn('No IMU data received')
            return False
        
        # Check the orientation quaternion is normalized
        q = self.imu_data.orientation
        norm = (q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)**0.5
        if abs(norm - 1.0) > 0.01:  # Allow small floating point error
            self.get_logger().warn(f'IMU orientation quaternion not normalized: {norm}')
        
        # Check that angular velocity and linear acceleration are finite
        ang_vel = self.imu_data.angular_velocity
        lin_acc = self.imu_data.linear_acceleration
        
        values_to_check = [
            ang_vel.x, ang_vel.y, ang_vel.z,
            lin_acc.x, lin_acc.y, lin_acc.z
        ]
        
        for val in values_to_check:
            if val != val:  # Check for NaN
                self.get_logger().error(f'IMU contains NaN value: {val}')
                return False
            if val == float('inf') or val == float('-inf'):
                self.get_logger().error(f'IMU contains infinite value: {val}')
                return False
        
        self.get_logger().info('IMU validation PASSED')
        return True
    
    def validate_pointcloud(self):
        """Validate point cloud data"""
        if self.point_cloud is None:
            self.get_logger().warn('No point cloud data received')
            return False
        
        # Check if point cloud has expected number of points
        expected_points = self.point_cloud.height * self.point_cloud.width
        if expected_points <= 0:
            self.get_logger().error(f'Invalid point cloud dimensions: {self.point_cloud.height}x{self.point_cloud.width}')
            return False
        
        self.get_logger().info(f'Point cloud validation PASSED with {expected_points} points')
        return True
    
    def run_validation(self, timeout=10):
        """Run the full validation process"""
        self.get_logger().info(f'Waiting up to {timeout} seconds for sensor data...')
        
        # Wait for data
        start_time = time.time()
        while (time.time() - start_time < timeout and 
               (self.lidar_data is None or 
                self.depth_image is None or 
                self.imu_data is None or 
                self.point_cloud is None)):
            
            # Allow ROS to process messages
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Validate each sensor type
        lidar_ok = self.validate_lidar_data()
        img_ok = self.validate_depth_image()
        imu_ok = self.validate_imu_data()
        pc_ok = self.validate_pointcloud()
        
        # Overall result
        all_ok = lidar_ok and img_ok and imu_ok and pc_ok
        
        self.get_logger().info(f'LiDAR: {"PASS" if lidar_ok else "FAIL"}')
        self.get_logger().info(f'Depth Image: {"PASS" if img_ok else "FAIL"}')
        self.get_logger().info(f'IMU: {"PASS" if imu_ok else "FAIL"}')
        self.get_logger().info(f'Point Cloud: {"PASS" if pc_ok else "FAIL"}')
        self.get_logger().info(f'Overall Sensor Validation: {"PASSED" if all_ok else "FAILED"}')
        
        return all_ok


def main(args=None):
    rclpy.init(args=args)
    
    validator = SensorValidator()
    
    try:
        # Run validation
        success = validator.run_validation(timeout=15)
        
        if not success:
            exit(1)  # Exit with error code if validation failed
    
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()