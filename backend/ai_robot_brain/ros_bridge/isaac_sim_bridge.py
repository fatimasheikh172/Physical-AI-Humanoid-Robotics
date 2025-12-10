#!/usr/bin/env python3

"""
Isaac Sim Bridge Node
This node acts as a bridge between Isaac Sim and ROS 2 ecosystem.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu, MagneticField
from geometry_msgs.msg import PoseStamped, Twist
from builtin_interfaces.msg import Time
import numpy as np
import tf2_ros


class IsaacSimBridgeNode(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge')
        
        # Declare parameters
        self.declare_parameter('robot_model_path', '')
        self.declare_parameter('sensor_config_path', '')
        
        # Get parameters
        self.robot_model_path = self.get_parameter('robot_model_path').get_parameter_value().string_value
        self.sensor_config_path = self.get_parameter('sensor_config_path').get_parameter_value().string_value

        # Setup TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publishers for Isaac Sim data (simulating what would come from Isaac Sim)
        self.rgb_publisher = self.create_publisher(Image, '/rgb/image_raw', 10)
        self.depth_publisher = self.create_publisher(Image, '/depth/image_raw', 10)
        self.cam_info_publisher = self.create_publisher(CameraInfo, '/rgb/camera_info', 10)
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.mag_publisher = self.create_publisher(MagneticField, '/imu/mag', 10)
        
        # Timers to simulate data publishing
        self.timer = self.create_timer(0.1, self.publish_simulation_data)  # 10 Hz
        
        self.get_logger().info('Isaac Sim Bridge Node initialized')

    def publish_simulation_data(self):
        """Simulate publishing sensor data that would come from Isaac Sim"""
        # Publish dummy RGB image
        rgb_msg = Image()
        rgb_msg.header.stamp = self.get_clock().now().to_msg()
        rgb_msg.header.frame_id = 'camera_rgb_optical_frame'
        rgb_msg.height = 480
        rgb_msg.width = 640
        rgb_msg.encoding = 'rgb8'
        rgb_msg.is_bigendian = False
        rgb_msg.step = 640 * 3  # width * channels
        rgb_msg.data = list(np.random.randint(0, 255, 640 * 480 * 3, dtype=np.uint8))
        self.rgb_publisher.publish(rgb_msg)

        # Publish dummy depth image
        depth_msg = Image()
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        depth_msg.height = 480
        depth_msg.width = 640
        depth_msg.encoding = '32FC1'
        depth_msg.is_bigendian = False
        depth_msg.step = 640 * 4  # width * bytes per float
        depth_values = np.random.uniform(0.1, 10.0, (480, 640)).astype(np.float32)
        depth_msg.data = depth_values.tobytes()
        self.depth_publisher.publish(depth_msg)

        # Publish dummy camera info
        cam_info_msg = CameraInfo()
        cam_info_msg.header.stamp = self.get_clock().now().to_msg()
        cam_info_msg.header.frame_id = 'camera_rgb_optical_frame'
        cam_info_msg.height = 480
        cam_info_msg.width = 640
        # Dummy camera matrix
        cam_info_msg.k = [554.0, 0.0, 320.0, 0.0, 554.0, 240.0, 0.0, 0.0, 1.0]
        # Dummy distortion coefficients
        cam_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.cam_info_publisher.publish(cam_info_msg)

        # Publish dummy IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        # Random orientation for simulation
        imu_msg.orientation.x = np.random.uniform(-1.0, 1.0)
        imu_msg.orientation.y = np.random.uniform(-1.0, 1.0)
        imu_msg.orientation.z = np.random.uniform(-1.0, 1.0)
        imu_msg.orientation.w = np.random.uniform(-1.0, 1.0)
        
        # Fill covariance matrices (set to 0 for simulation)
        imu_msg.orientation_covariance = [0.0] * 9
        imu_msg.angular_velocity_covariance = [0.0] * 9
        imu_msg.linear_acceleration_covariance = [0.0] * 9
        self.imu_publisher.publish(imu_msg)

        self.get_logger().debug('Published simulation sensor data')


def main(args=None):
    rclpy.init(args=args)
    
    isaac_sim_bridge = IsaacSimBridgeNode()
    
    try:
        rclpy.spin(isaac_sim_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        isaac_sim_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()