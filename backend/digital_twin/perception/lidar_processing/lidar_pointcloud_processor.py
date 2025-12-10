#!/usr/bin/env python3
"""
LiDAR PointCloud Processing Node for Digital Twin Perception

This node processes LiDAR data to generate point clouds and perform basic
analysis like obstacle detection and environment mapping.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import Header
import math
import numpy as np
from tf2_ros import TransformBroadcaster
import tf_transformations
from rclpy.qos import QoSProfile, ReliabilityPolicy


class LidarPointCloudProcessor(Node):
    def __init__(self):
        super().__init__('lidar_pointcloud_processor')
        
        # Create subscription to LiDAR scan
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/robot/laser_scan',
            self.scan_callback,
            qos_profile
        )
        
        # Create publisher for point cloud
        self.pc_pub = self.create_publisher(
            PointCloud2,
            '/robot/lidar_pointcloud',
            10
        )
        
        # Create publisher for obstacle detection
        self.obstacle_pub = self.create_publisher(
            PointStamped,
            '/robot/obstacle_point',
            10
        )
        
        # Parameters
        self.declare_parameter('min_obstacle_distance', 1.0)
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        
        self.get_logger().info('LiDAR PointCloud Processor Node initialized')
    
    def scan_callback(self, msg):
        """Callback function to process LiDAR scan and generate point cloud"""
        try:
            # Convert LaserScan to PointCloud2
            pc_msg = self.laser_scan_to_pointcloud2(msg)
            
            # Publish point cloud
            self.pc_pub.publish(pc_msg)
            
            # Perform obstacle detection
            obstacle_point = self.detect_obstacles(msg)
            if obstacle_point is not None:
                self.obstacle_pub.publish(obstacle_point)
        
        except Exception as e:
            self.get_logger().error(f'Error processing scan: {str(e)}')
    
    def laser_scan_to_pointcloud2(self, scan_msg):
        """Convert LaserScan message to PointCloud2 message"""
        # Calculate point count
        num_points = len(scan_msg.ranges)
        
        # Create PointCloud2 message
        pc_msg = PointCloud2()
        pc_msg.header = scan_msg.header
        pc_msg.height = 1
        pc_msg.width = num_points
        pc_msg.is_dense = False
        pc_msg.is_bigendian = False
        
        # Define point fields (x, y, z, intensity)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg.fields = fields
        pc_msg.point_step = 12  # 3 * 4 bytes per float
        pc_msg.row_step = pc_msg.point_step * pc_msg.width
        
        # Convert scan ranges to point cloud
        points = []
        for i, range_val in enumerate(scan_msg.ranges):
            if not (math.isnan(range_val) or math.isinf(range_val)) and scan_msg.range_min <= range_val <= scan_msg.range_max:
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                z = 0.0  # LiDAR is typically 2D, so z=0
                points.extend([x, y, z])
        
        # Pack points into binary data
        import struct
        pc_data = []
        for i in range(0, len(points), 3):
            pc_data.append(struct.pack('fff', points[i], points[i+1], points[i+2]))
        
        # Combine all point data
        packed_data = b''.join(pc_data)
        pc_msg.data = packed_data
        
        return pc_msg
    
    def detect_obstacles(self, scan_msg):
        """Detect obstacles from scan data and return the closest one"""
        min_distance = float('inf')
        closest_point = None
        
        for i, range_val in enumerate(scan_msg.ranges):
            if not (math.isnan(range_val) or math.isinf(range_val)) and scan_msg.range_min <= range_val <= scan_msg.range_max:
                if range_val < min_distance and range_val < self.min_obstacle_distance:
                    min_distance = range_val
                    angle = scan_msg.angle_min + i * scan_msg.angle_increment
                    x = range_val * math.cos(angle)
                    y = range_val * math.sin(angle)
                    
                    closest_point = PointStamped()
                    closest_point.header = scan_msg.header
                    closest_point.point.x = x
                    closest_point.point.y = y
                    closest_point.point.z = 0.0  # 2D LiDAR
        
        return closest_point


def main(args=None):
    rclpy.init(args=args)
    
    processor = LidarPointCloudProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()