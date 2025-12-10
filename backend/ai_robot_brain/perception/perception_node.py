#!/usr/bin/env python3

"""
Perception Node for Digital Twin System

This node processes sensor data from Isaac Sim to perform perception tasks
including object detection, segmentation, and environment understanding.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu, PointCloud2
from std_msgs.msg import String, Float64
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
from cv_bridge import CvBridge
import tf2_ros
from tf2_ros import TransformException
import math
import torch
import torch.nn as nn
import torchvision.transforms as transforms


class IsaacPerceptionNode(Node):
    def __init__(self):
        super().__init__('isaac_perception_node')
        
        # Declare parameters
        self.declare_parameter('model_path', '/path/to/default/model.plan')
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('max_objects', 50)
        self.declare_parameter('use_tensorrt', True)
        self.declare_parameter('sensor_processing_frequency', 10.0)
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.max_objects = self.get_parameter('max_objects').value
        self.use_tensorrt = self.get_parameter('use_tensorrt').value
        self.sensor_frequency = self.get_parameter('sensor_processing_frequency').value
        
        # Initialize CV Bridge
        self.cv_bridge = CvBridge()
        
        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create subscribers for Isaac Sim sensor data
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/robot/lidar_scan',
            self.lidar_callback,
            10
        )
        
        self.camera_subscription = self.create_subscription(
            Image,
            '/robot/camera/image_raw',
            self.camera_callback,
            10
        )
        
        self.imu_subscription = self.create_subscription(
            Imu,
            '/robot/imu',
            self.imu_callback,
            10
        )
        
        # Publishers for perception results
        self.detection_publisher = self.create_publisher(String, '/perception/detections', 10)
        self.segmentation_publisher = self.create_publisher(Image, '/perception/segmentation', 10)
        self.obstacle_publisher = self.create_publisher(PointStamped, '/perception/obstacles', 10)
        self.vis_publisher = self.create_publisher(MarkerArray, '/perception/visualization', 10)
        
        # Initialize perception model (TensorRT optimized)
        self.initialize_perception_model()
        
        # Store latest sensor data
        self.latest_lidar_data = None
        self.latest_camera_data = None
        self.latest_imu_data = None
        
        # Timer for processing pipeline
        self.processing_timer = self.create_timer(1.0 / self.sensor_frequency, self.perception_pipeline)
        
        self.get_logger().info('Isaac Perception Node initialized')

    def initialize_perception_model(self):
        """Initialize the perception model with TensorRT optimization"""
        try:
            if self.use_tensorrt:
                self.get_logger().info(f'Loading TensorRT optimized model from: {self.model_path}')
                # In a real Isaac ROS implementation, this would load a TensorRT plan file
                # For simulation, we'll create a mock model
                self.perception_model = self.create_mock_perception_model()
            else:
                self.get_logger().info(f'Loading standard model from: {self.model_path}')
                # Standard PyTorch/TensorFlow model loading would go here
                self.perception_model = self.create_mock_perception_model()
                
            self.get_logger().info('Perception model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load perception model: {e}')
            self.perception_model = self.create_mock_perception_model()

    def create_mock_perception_model(self):
        """Create a mock perception model for simulation purposes"""
        class MockPerceptionModel:
            def infer(self, input_data):
                # Simulate object detection results
                height, width = input_data.shape[:2]
                
                # Generate mock detections (in real implementation, this would be actual model inference)
                num_detections = np.random.poisson(3)  # Average of 3 detections per frame
                detections = []
                
                for i in range(min(num_detections, 10)):  # Max 10 detections
                    # Generate random bounding box
                    x_min = int(np.random.uniform(0, width * 0.7))
                    y_min = int(np.random.uniform(0, height * 0.7))
                    width_bb = int(np.random.uniform(0.1 * width, 0.3 * width))
                    height_bb = int(np.random.uniform(0.1 * height, 0.3 * height))
                    
                    x_max = min(x_min + width_bb, width)
                    y_max = min(y_min + height_bb, height)
                    
                    # Random class and confidence
                    classes = ['person', 'obstacle', 'landmark', 'tool', 'wall']
                    class_name = np.random.choice(classes)
                    confidence = np.random.uniform(0.6, 0.98)
                    
                    if confidence > self.confidence_threshold:
                        detection = {
                            'class': class_name,
                            'confidence': confidence,
                            'bbox': [x_min, y_min, x_max, y_max],
                            'center': [(x_min + x_max) / 2, (y_min + y_max) / 2]
                        }
                        detections.append(detection)
                
                return detections
        
        return MockPerceptionModel()

    def lidar_callback(self, msg):
        """Process incoming LiDAR data from Isaac Sim"""
        self.latest_lidar_data = msg
        self.get_logger().debug(f'LiDAR data received with {len(msg.ranges)} range values')

    def camera_callback(self, msg):
        """Process incoming camera data from Isaac Sim"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_camera_data = cv_image
            self.get_logger().debug('Camera data received and converted')
        except Exception as e:
            self.get_logger().error(f'Error converting camera image: {e}')

    def imu_callback(self, msg):
        """Process incoming IMU data from Isaac Sim"""
        self.latest_imu_data = msg
        self.get_logger().debug('IMU data received')

    def perception_pipeline(self):
        """Main perception processing pipeline"""
        if self.latest_camera_data is None or self.latest_lidar_data is None:
            return
        
        try:
            # Process camera image with perception model
            camera_detections = self.process_camera_perception(self.latest_camera_data)
            
            # Process LiDAR data for obstacle detection
            lidar_obstacles = self.process_lidar_obstacles(self.latest_lidar_data)
            
            # Fuse camera and LiDAR data
            fused_detections = self.fuse_sensor_data(camera_detections, lidar_obstacles)
            
            # Publish results
            self.publish_perception_results(fused_detections)
            
            # Create visualization markers
            self.publish_visualization_markers(fused_detections)
            
            self.get_logger().info(f'Detected {len(fused_detections)} objects in scene')
        except Exception as e:
            self.get_logger().error(f'Error in perception pipeline: {e}')

    def process_camera_perception(self, image):
        """Process camera image with perception model"""
        try:
            # Run inference with the perception model
            detections = self.perception_model.infer(image)
            
            # Transform image coordinates to world coordinates using camera calibration
            # (In real implementation, would use actual camera calibration)
            transformed_detections = []
            for detection in detections:
                # For simulation, we'll just keep the image-space detection
                # In real implementation, would transform to world coordinates
                transformed_detection = detection.copy()
                transformed_detections.append(transformed_detection)
            
            return transformed_detections
        except Exception as e:
            self.get_logger().error(f'Error processing camera perception: {e}')
            return []

    def process_lidar_obstacles(self, lidar_msg):
        """Process LiDAR data to detect obstacles"""
        try:
            obstacles = []
            
            # Process ranges to detect obstacles
            for i, distance in enumerate(lidar_msg.ranges):
                if lidar_msg.range_min <= distance <= lidar_msg.range_max:
                    # Convert index to angle
                    angle = lidar_msg.angle_min + i * lidar_msg.angle_increment
                    
                    # Check if distance is below threshold (indicating obstacle)
                    if distance < 2.0:  # Obstacle threshold in meters
                        obstacle = {
                            'angle': angle,
                            'distance': distance,
                            'x': distance * math.cos(angle),
                            'y': distance * math.sin(angle),
                            'type': 'obstacle'
                        }
                        obstacles.append(obstacle)
            
            return obstacles
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR obstacles: {e}')
            return []

    def fuse_sensor_data(self, camera_detections, lidar_obstacles):
        """Fuse detections from camera and LiDAR data"""
        # In a real implementation, this would perform complex sensor fusion
        # For simulation, we'll just combine the results
        fused_results = {
            'camera_detections': camera_detections,
            'lidar_obstacles': lidar_obstacles,
            'fused_objects': []
        }
        
        # Simple fusion: associate LiDAR obstacles with camera detections
        for lidar_obj in lidar_obstacles:
            # Project LiDAR point into camera frame (simplified)
            # In real implementation, would need proper extrinsic calibration
            associated = False
            for cam_obj in camera_detections:
                # For simulation, associate if they're roughly in the same direction
                if abs(lidar_obj['angle']) < 0.5:  # Within 30 degrees
                    fused_obj = {
                        'type': cam_obj['class'],
                        'confidence': (cam_obj['confidence'] + 0.8) / 2,  # Average with high lidar confidence
                        'position': {'x': lidar_obj['x'], 'y': lidar_obj['y']},
                        'bbox': cam_obj['bbox']
                    }
                    fused_results['fused_objects'].append(fused_obj)
                    associated = True
                    break
            
            if not associated:
                # Add just the LiDAR obstacle
                fused_obj = {
                    'type': 'obstacle',
                    'confidence': 0.8,
                    'position': {'x': lidar_obj['x'], 'y': lidar_obj['y']},
                    'bbox': None
                }
                fused_results['fused_objects'].append(fused_obj)
        
        return fused_results

    def publish_perception_results(self, fused_detections):
        """Publish perception results to ROS topics"""
        try:
            # Publish detection summary
            if fused_detections and 'fused_objects' in fused_detections:
                detection_summary = f"Perception: {len(fused_detections['fused_objects'])} objects detected"
                
                summary_msg = String()
                summary_msg.data = detection_summary
                self.detection_publisher.publish(summary_msg)
                
                # Publish individual obstacle positions
                for obj in fused_detections['fused_objects']:
                    if 'position' in obj:
                        # Publish to obstacle topic
                        point_msg = PointStamped()
                        point_msg.header.stamp = self.get_clock().now().to_msg()
                        point_msg.header.frame_id = 'map'  # Assuming global reference frame
                        
                        point_msg.point.x = obj['position']['x']
                        point_msg.point.y = obj['position']['y']
                        point_msg.point.z = 0.0  # Ground plane
                        
                        self.obstacle_publisher.publish(point_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing perception results: {e}')

    def publish_visualization_markers(self, fused_detections):
        """Publish visualization markers for RViz2"""
        try:
            marker_array = MarkerArray()
            
            if not fused_detections or 'fused_objects' not in fused_detections:
                return
            
            for i, obj in enumerate(fused_detections['fused_objects']):
                # Create a marker for each detected object
                marker = Marker()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = 'map'  # Assuming global reference frame
                marker.ns = 'perception'
                marker.id = i
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                # Set position
                if 'position' in obj:
                    marker.pose.position.x = obj['position']['x']
                    marker.pose.position.y = obj['position']['y']
                    marker.pose.position.z = 0.5  # Height above ground
                
                # Set scale (adjust based on object type or confidence)
                marker.scale.x = 0.3  # 30cm cube
                marker.scale.y = 0.3
                marker.scale.z = 0.3
                
                # Set color based on object type
                if 'type' in obj:
                    if obj['type'] == 'person':
                        marker.color.r = 0.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0  # Green for person
                        marker.color.a = 0.8
                    elif obj['type'] == 'obstacle':
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0  # Red for obstacle
                        marker.color.a = 0.8
                    elif obj['type'] == 'landmark':
                        marker.color.r = 0.0
                        marker.color.g = 0.0
                        marker.color.b = 1.0  # Blue for landmark
                        marker.color.a = 0.8
                    else:
                        marker.color.r = 1.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0  # Yellow for others
                        marker.color.a = 0.8
                
                # Set lifetime
                marker.lifetime.sec = 1  # Visible for 1 second
                
                marker_array.markers.append(marker)
                
                # Add text label
                text_marker = Marker()
                text_marker.header = marker.header
                text_marker.ns = 'perception_labels'
                text_marker.id = i + 1000  # Separate ID space for labels
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = marker.pose.position.x
                text_marker.pose.position.y = marker.pose.position.y
                text_marker.pose.position.z = marker.pose.position.z + 0.5  # Above the object
                text_marker.scale.z = 0.3  # Text height
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                text_marker.text = f"{obj['type']}: {obj.get('confidence', 0):.2f}"
                
                marker_array.markers.append(text_marker)
        
            self.vis_publisher.publish(marker_array)
        except Exception as e:
            self.get_logger().error(f'Error publishing visualization markers: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    perception_node = IsaacPerceptionNode()
    
    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        perception_node.get_logger().info('Perception node shutting down')
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()