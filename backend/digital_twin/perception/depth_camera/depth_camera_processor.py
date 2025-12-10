#!/usr/bin/env python3
"""
Depth Camera Processing Node for Digital Twin Perception

This node processes depth camera data to perform tasks like ground plane extraction,
obstacle detection, and 3D reconstruction from depth images.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, ReliabilityPolicy


class DepthCameraProcessor(Node):
    def __init__(self):
        super().__init__('depth_camera_processor')
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Create subscription to depth image
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.depth_sub = self.create_subscription(
            Image,
            '/robot/depth_camera/image_raw',
            self.depth_callback,
            qos_profile
        )
        
        # Create subscription to camera info
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/robot/depth_camera/camera_info',
            self.info_callback,
            qos_profile
        )
        
        # Create publisher for processed depth data
        self.ground_plane_pub = self.create_publisher(
            PointStamped,
            '/robot/ground_plane_point',
            10
        )
        
        # Create publisher for visualization markers
        self.marker_pub = self.create_publisher(
            Marker,
            '/robot/depth_camera/visualization_marker',
            10
        )
        
        # Store camera intrinsics
        self.camera_matrix = None
        self.distortion_coeffs = None
        
        # Parameters
        self.declare_parameter('ground_plane_threshold', 0.1)  # meters
        self.ground_plane_threshold = self.get_parameter('ground_plane_threshold').value
        
        self.get_logger().info('Depth Camera Processor Node initialized')
    
    def info_callback(self, msg):
        """Callback function to get camera intrinsics"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)
    
    def depth_callback(self, msg):
        """Callback function to process depth image"""
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            
            # Process the depth image
            ground_point = self.extract_ground_plane(cv_image)
            if ground_point is not None:
                self.ground_plane_pub.publish(ground_point)
            
            # Create visualization marker
            self.publish_visualization_marker(cv_image)
        
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')
    
    def extract_ground_plane(self, depth_image):
        """Extract ground plane from depth image using simple height thresholding"""
        if depth_image is None or depth_image.size == 0:
            return None
        
        # Get the center region of the image to determine ground plane height
        height, width = depth_image.shape
        center_h, center_w = height // 2, width // 2
        
        # Define a region of interest in the center bottom of the image
        roi_top = int(0.7 * height)
        roi_bottom = height
        roi_left = int(0.3 * width)
        roi_right = int(0.7 * width)
        
        roi = depth_image[roi_top:roi_bottom, roi_left:roi_right]
        
        # Filter out invalid depth values (NaN, infinity)
        valid_depths = roi[np.isfinite(roi) & (roi > 0)]
        
        if len(valid_depths) == 0:
            return None
        
        # Calculate the median depth in the region (assumed to be ground level)
        median_depth = np.median(valid_depths)
        
        # Find points close to the median (potential ground points)
        ground_mask = (np.abs(depth_image - median_depth) < self.ground_plane_threshold) & np.isfinite(depth_image)
        
        # Get coordinates of ground points
        ground_coords = np.where(ground_mask)
        
        if len(ground_coords[0]) == 0:
            return None
        
        # Take the first ground point as a representative
        h_idx, w_idx = ground_coords[0][0], ground_coords[1][0]
        depth_val = depth_image[h_idx, w_idx]
        
        # If we have camera intrinsics, convert pixel coordinates to 3D world coordinates
        if self.camera_matrix is not None:
            # Convert pixel coordinates to camera coordinates
            x_norm = (w_idx - self.camera_matrix[0, 2]) / self.camera_matrix[0, 0]
            y_norm = (h_idx - self.camera_matrix[1, 2]) / self.camera_matrix[1, 1]
            
            # Convert to 3D world coordinates
            x = x_norm * depth_val
            y = y_norm * depth_val
            z = depth_val
            
            # Create point message
            point_msg = PointStamped()
            point_msg.header = Header()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = 'camera_depth_frame'  # This should match your depth camera frame
            point_msg.point.x = x
            point_msg.point.y = y
            point_msg.point.z = z
            
            return point_msg
        else:
            # Return a point at the median depth without world coordinates
            point_msg = PointStamped()
            point_msg.header = Header()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = 'camera_depth_frame'
            point_msg.point.x = 0.0
            point_msg.point.y = 0.0
            point_msg.point.z = median_depth
            
            return point_msg
    
    def publish_visualization_marker(self, depth_image):
        """Create and publish a visualization marker for the processed depth data"""
        # Create a simple visualization marker
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'camera_depth_frame'
        
        marker.ns = "depth_camera"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set the position and scale of the marker
        # For this example, we'll just place it in front of the camera
        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Scale based on depth values (example: min depth = 0.5, max depth = 2.0)
        min_depth = np.min(depth_image[np.isfinite(depth_image)]) if np.any(np.isfinite(depth_image)) else 0.5
        max_depth = np.max(depth_image[np.isfinite(depth_image)]) if np.any(np.isfinite(depth_image)) else 2.0
        
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        # Color based on depth (blue for close, red for far)
        depth_ratio = (max_depth - min_depth) / (max_depth - min_depth) if (max_depth - min_depth) != 0 else 0
        marker.color.r = depth_ratio
        marker.color.g = 0.0
        marker.color.b = 1.0 - depth_ratio
        marker.color.a = 0.8
        
        marker.lifetime.sec = 1  # Visible for 1 second
        
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    
    processor = DepthCameraProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()