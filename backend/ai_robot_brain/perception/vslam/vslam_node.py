#!/usr/bin/env python3

"""
VSLAM Node for Isaac ROS
Implements hardware-accelerated visual SLAM for humanoid navigation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
import tf2_ros
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import threading
from collections import deque
import time


class IsaacVSLAMNode(Node):
    def __init__(self):
        super().__init__('isaac_vslam_node')
        
        # Declare parameters
        self.declare_parameter('tracking_quality_threshold', 0.7)
        self.declare_parameter('relocalization_enabled', True)
        self.declare_parameter('map_cleanup_interval', 10)
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('max_features', 2000)
        self.declare_parameter('min_matches', 20)
        
        # Get parameters
        self.tracking_quality_threshold = self.get_parameter('tracking_quality_threshold').get_parameter_value().double_value
        self.relocalization_enabled = self.get_parameter('relocalization_enabled').get_parameter_value().bool_value
        self.map_cleanup_interval = self.get_parameter('map_cleanup_interval').get_parameter_value().integer_value
        self.use_gpu = self.get_parameter('use_gpu').get_parameter_value().bool_value
        self.max_features = self.get_parameter('max_features').get_parameter_value().integer_value
        self.min_matches = self.get_parameter('min_matches').get_parameter_value().integer_value

        # Initialize CvBridge
        self.cv_bridge = CvBridge()

        # Setup TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers for VSLAM results
        self.pose_publisher = self.create_publisher(PoseStamped, '/vslam/pose', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/vslam/odometry', 10)
        
        # Subscribers for sensor data
        self.rgb_subscriber = self.create_subscription(
            Image,
            '/rgb/image_raw',
            self.image_callback,
            10
        )
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # Timer for VSLAM processing
        self.vslam_timer = self.create_timer(0.033, self.vslam_pipeline)  # ~30 Hz

        # Storage for sensor data
        self.latest_image = None
        self.latest_imu = None
        self.previous_image = None
        self.image_lock = threading.Lock()
        
        # VSLAM state variables
        self.current_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),  # x, y, z, w quaternion
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        self.tracking_status = 'Initializing'  # 'Initializing', 'Tracking', 'Lost', 'Recovering'
        self.feature_tracks = {}  # Track features across frames
        self.map_points = {}  # 3D points in the map
        self.frame_id = 0
        
        # Tracking history
        self.pose_history = deque(maxlen=100)
        
        self.get_logger().info('Isaac VSLAM Node initialized')

    def image_callback(self, msg):
        """Process incoming RGB image for VSLAM"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')  # Using grayscale for efficiency
            with self.image_lock:
                self.previous_image = self.latest_image
                self.latest_image = cv_image
                self.latest_image_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    
    def imu_callback(self, msg):
        """Process incoming IMU data to aid VSLAM"""
        self.latest_imu = msg
    
    def vslam_pipeline(self):
        """Main VSLAM processing pipeline"""
        with self.image_lock:
            if self.latest_image is None or self.previous_image is None:
                return
        
        # Copy these values to avoid race conditions within this method
        current_image = self.latest_image.copy()
        previous_image = self.previous_image.copy()
        
        # Perform visual-inertial odometry
        try:
            # Track features between previous and current frame
            poses_delta = self.track_features(previous_image, current_image)
            
            # Update current pose based on delta
            if poses_delta is not None:
                self.update_pose(poses_delta)
            
            # Publish pose and transform
            self.publish_pose()
            self.broadcast_transform()
            
            # Update tracking status
            if self.current_pose is not None:
                self.tracking_status = 'Tracking'
            
            # Log pose information periodically
            if self.frame_id % 30 == 0:  # Log every 30 frames
                pos = self.current_pose['position']
                self.get_logger().info(f'VSLAM Pose: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}')
                
        except Exception as e:
            self.get_logger().error(f'VSLAM processing error: {e}')
            self.tracking_status = 'Lost'
    
    def track_features(self, prev_img, curr_img):
        """Track features between consecutive images"""
        # In a real Isaac ROS implementation, this would use hardware-accelerated
        # optical flow tracking like Isaac ROS Visual Inertial Odometry (VIO) package
        # For simulation, we'll implement a basic feature tracking algorithm
        
        try:
            # Detect features in the previous image
            # Using Shi-Tomasi corner detector as a simple example
            prev_corners = cv2.goodFeaturesToTrack(
                prev_img,
                maxCorners=self.max_features,
                qualityLevel=0.01,
                minDistance=10,
                blockSize=3
            )
            
            if prev_corners is None or len(prev_corners) < self.min_matches:
                self.get_logger().debug(f'Not enough features detected: {len(prev_corners) if prev_corners is not None else 0}')
                return None
            
            # Track features using Lucas-Kanade optical flow
            curr_corners, status, err = cv2.calcOpticalFlowPyrLK(
                prev_img, curr_img, 
                prev_corners, None, 
                winSize=(21, 21),
                maxLevel=3,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
            )
            
            # Filter for good matches
            good_prev = prev_corners[status.ravel() == 1]
            good_curr = curr_corners[status.ravel() == 1]
            
            if len(good_prev) < self.min_matches:
                self.get_logger().debug(f'Not enough good matches: {len(good_prev)}')
                return None
            
            # Compute relative pose change based on tracked features
            # This is a simplified approach - in real implementation, use VIO algorithms
            displacement = np.mean(good_curr - good_prev, axis=0)
            
            # Convert pixel displacement to metric displacement using camera intrinsics
            # This is a rough approximation - in reality would use full VIO pipeline
            # For now, just use a simple scaling factor
            scale_factor = 0.001  # Rough conversion from pixels to meters
            metric_displacement = displacement * scale_factor
            
            # Return pose delta (simplified)
            pose_delta = {
                'translation': metric_displacement,
                'rotation': np.array([0.0, 0.0, 0.0])  # Simplified
            }
            
            return pose_delta
        except Exception as e:
            self.get_logger().error(f'Feature tracking error: {e}')
            self.tracking_status = 'Lost'
            return None
    
    def update_pose(self, pose_delta):
        """Update the current pose based on the computed delta"""
        self.current_pose['position'] += pose_delta['translation']
        
        # Update rotation (simplified)
        # In reality, would integrate rotation properly
        self.current_pose['timestamp'] = self.get_clock().now().nanoseconds / 1e9
        
        # Store in history
        self.pose_history.append({
            'position': self.current_pose['position'].copy(),
            'timestamp': self.current_pose['timestamp']
        })
        
        self.frame_id += 1
    
    def publish_pose(self):
        """Publish the current pose as PoseStamped and Odometry"""
        try:
            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'  # Assuming map is the global frame
            
            pose_msg.pose.position.x = float(self.current_pose['position'][0])
            pose_msg.pose.position.y = float(self.current_pose['position'][1])
            pose_msg.pose.position.z = float(self.current_pose['position'][2])
            
            # For now, using identity quaternion - in real implementation would track rotation
            pose_msg.pose.orientation.x = float(self.current_pose['orientation'][0])
            pose_msg.pose.orientation.y = float(self.current_pose['orientation'][1])
            pose_msg.pose.orientation.z = float(self.current_pose['orientation'][2])
            pose_msg.pose.orientation.w = float(self.current_pose['orientation'][3])
            
            self.pose_publisher.publish(pose_msg)
            
            # Create Odometry message
            odom_msg = Odometry()
            odom_msg.header = pose_msg.header
            odom_msg.child_frame_id = 'vslam_body'
            odom_msg.pose.pose = pose_msg.pose  # For now, just copy the pose
            
            # Would add velocity and covariance in real implementation
            self.odom_publisher.publish(odom_msg)
        except Exception as e:
            self.get_logger().error(f'Pose publishing error: {e}')
    
    def broadcast_transform(self):
        """Broadcast the transform between map and camera frame"""
        try:
            t = TransformStamped()
            
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'vslam_body'
            
            t.transform.translation.x = float(self.current_pose['position'][0])
            t.transform.translation.y = float(self.current_pose['position'][1])
            t.transform.translation.z = float(self.current_pose['position'][2])
            
            t.transform.rotation.x = float(self.current_pose['orientation'][0])
            t.transform.rotation.y = float(self.current_pose['orientation'][1])
            t.transform.rotation.z = float(self.current_pose['orientation'][2])
            t.transform.rotation.w = float(self.current_pose['orientation'][3])
            
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f'Transform broadcasting error: {e}')
    
    def initialize_tracking(self):
        """Initialize the VSLAM tracking system"""
        self.get_logger().info('Initializing VSLAM tracking...')
        # In real Isaac ROS implementation, this would initialize
        # the hardware-accelerated VIO pipeline
        self.tracking_status = 'Initializing'
    
    def relocalize(self):
        """Attempt to relocalize if tracking is lost"""
        if self.relocalization_enabled:
            self.get_logger().info('Attempting relocalization...')
            # Implementation would depend on Isaac ROS specific relocalization approach
            # For now, just reset to origin
            self.current_pose['position'] = np.array([0.0, 0.0, 0.0])
            self.tracking_status = 'Recovering'


def main(args=None):
    rclpy.init(args=args)
    
    vslam_node = IsaacVSLAMNode()
    
    try:
        rclpy.spin(vslam_node)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()