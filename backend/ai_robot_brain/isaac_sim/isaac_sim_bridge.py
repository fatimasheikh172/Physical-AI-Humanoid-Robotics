#!/usr/bin/env python3

"""
Isaac Sim Bridge Node for Digital Twin System

This node bridges Isaac Sim with the ROS 2 ecosystem, enabling photorealistic
simulation with synthetic data generation for the digital twin system.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Float64
import numpy as np
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math


class IsaacSimBridgeNode(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge_node')
        
        # Declare parameters
        self.declare_parameter('robot_name', 'humanoid_robot')
        self.declare_parameter('publish_frequency', 30.0)
        self.declare_parameter('use_domain_randomization', True)
        self.declare_parameter('synthetic_data_quality', 'photorealistic')
        
        # Get parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.use_domain_randomization = self.get_parameter('use_domain_randomization').value
        self.synthetic_data_quality = self.get_parameter('synthetic_data_quality').value
        
        # Create publishers for simulated sensor data
        self.lidar_publisher = self.create_publisher(LaserScan, '/robot/lidar_scan', 10)
        self.camera_publisher = self.create_publisher(Image, '/robot/camera/image_raw', 10)
        self.depth_publisher = self.create_publisher(Image, '/robot/camera/depth/image_raw', 10)
        self.imu_publisher = self.create_publisher(Imu, '/robot/imu', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/robot/odom', 10)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/robot/pointcloud', 10)
        
        # Create timer for publishing simulated data
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.publish_simulated_data)
        
        # Initialize CV Bridge
        self.cv_bridge = CvBridge()
        
        # TF broadcaster for robot transforms
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Robot state variables
        self.robot_position = np.array([0.0, 0.0, 0.0])
        self.robot_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # w, x, y, z quaternion
        self.robot_linear_velocity = np.array([0.0, 0.0, 0.0])
        self.robot_angular_velocity = np.array([0.0, 0.0, 0.0])
        
        # Initialize Isaac Sim environment (simulated)
        self.initialize_simulation_environment()
        
        self.get_logger().info(f'Isaac Sim Bridge Node initialized for {self.robot_name}')

    def initialize_simulation_environment(self):
        """Initialize the simulation environment with Isaac Sim parameters"""
        # In a real implementation, this would interface with Isaac Sim
        # For this simulation, we'll set up mock parameters
        self.sim_params = {
            'gravity': np.array([0, 0, -9.81]),  # m/s^2
            'friction_coefficient': 0.5,
            'damping_linear': 0.1,
            'damping_angular': 0.1,
            'sensor_noise_levels': {
                'lidar': 0.01,      # meters
                'camera': 0.02,     # pixel noise
                'imu': 0.001,       # rad/s for gyroscope, m/s^2 for accelerometer
                'odom': 0.05        # meters
            }
        }
        
        # Create simulated sensor configurations
        self.lidar_config = {
            'range_min': 0.1,
            'range_max': 25.0,
            'angle_min': -math.pi,
            'angle_max': math.pi,
            'angle_increment': 0.0174533,  # 1 degree
            'samples': 360
        }
        
        self.camera_config = {
            'width': 640,
            'height': 480,
            'fov': 1.0472  # 60 degrees in radians
        }
        
        self.get_logger().info('Simulation environment initialized')

    def publish_simulated_data(self):
        """Publish simulated sensor data from Isaac Sim"""
        # Update robot state (simple motion model for simulation)
        self.update_robot_state()
        
        # Publish simulated LiDAR data
        self.publish_simulated_lidar()
        
        # Publish simulated camera data
        self.publish_simulated_camera()
        
        # Publish simulated depth camera data
        self.publish_simulated_depth()
        
        # Publish simulated IMU data
        self.publish_simulated_imu()
        
        # Publish simulated odometry
        self.publish_simulated_odom()
        
        # Publish simulated point cloud
        self.publish_simulated_pointcloud()
        
        # Broadcast transforms
        self.broadcast_transforms()

    def update_robot_state(self):
        """Update robot state based on simple motion model"""
        # Simple motion with slight random perturbations to simulate physics
        dt = 1.0 / self.publish_frequency
        
        # Add small random perturbations to simulate realistic movement
        linear_noise = np.random.normal(0, 0.01, 3)
        angular_noise = np.random.normal(0, 0.005, 3)
        
        # Update position based on velocity
        self.robot_position += (self.robot_linear_velocity + linear_noise) * dt
        
        # Update orientation based on angular velocity
        # Convert angular velocity to quaternion change
        angular_speed = np.linalg.norm(self.robot_angular_velocity)
        if angular_speed > 1e-6:
            axis = self.robot_angular_velocity / angular_speed
            angle = angular_speed * dt
            
            # Create rotation quaternion from axis-angle
            dq = np.array([
                math.cos(angle/2),
                math.sin(angle/2) * axis[0],
                math.sin(angle/2) * axis[1],
                math.sin(angle/2) * axis[2]
            ])
            
            # Multiply to get new orientation
            self.robot_orientation = self.quat_multiply(dq, self.robot_orientation)
            # Normalize quaternion
            self.robot_orientation = self.robot_orientation / np.linalg.norm(self.robot_orientation)
        
        # Example: Oscillatory motion for demonstration
        time_elapsed = self.get_clock().now().nanoseconds / 1e9
        self.robot_linear_velocity[0] = 0.3 * math.sin(time_elapsed * 0.5)  # Forward/back motion
        self.robot_angular_velocity[2] = 0.1 * math.cos(time_elapsed * 0.3)  # Turning motion

    def publish_simulated_lidar(self):
        """Publish simulated LiDAR data"""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.robot_name}/lidar_link'
        
        # Set laser scan parameters
        msg.angle_min = self.lidar_config['angle_min']
        msg.angle_max = self.lidar_config['angle_max']
        msg.angle_increment = self.lidar_config['angle_increment']
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.publish_frequency
        msg.range_min = self.lidar_config['range_min']
        msg.range_max = self.lidar_config['range_max']
        
        # Generate simulated ranges (in a real sim we'd get this from Isaac Sim)
        num_ranges = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        ranges = []
        
        for i in range(num_ranges):
            angle = msg.angle_min + i * msg.angle_increment
            
            # Simulate an environment with some obstacles
            if abs(math.sin(angle * 3)) < 0.2:  # Simulate obstacles in certain directions
                distance = 2.0 + 0.5 * math.sin(angle * 5)  # Objects at ~2m with variation
            else:
                distance = self.lidar_config['range_max'] - 1.0  # Far objects
            
            # Add noise to simulate sensor imperfections
            noise = np.random.normal(0, self.sim_params['sensor_noise_levels']['lidar'])
            noisy_distance = max(self.lidar_config['range_min'], min(self.lidar_config['range_max'], distance + noise))
            
            ranges.append(noisy_distance)
        
        msg.ranges = ranges
        msg.intensities = [100.0] * len(ranges)  # Constant intensity for simulation
        
        self.lidar_publisher.publish(msg)

    def publish_simulated_camera(self):
        """Publish simulated RGB camera data"""
        # Create a simulated image (in real implementation, this comes from Isaac Sim)
        height, width = self.camera_config['height'], self.camera_config['width']
        
        # Create a simulated scene with geometric shapes
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Add some simulated objects to the image
        center_x, center_y = width // 2, height // 2
        
        # Add a colored circle (could represent an object)
        cv2.circle(img, (center_x + 100, center_y), 30, (255, 0, 0), -1)  # Blue circle
        cv2.circle(img, (center_x - 100, center_y + 50), 40, (0, 255, 0), -1)  # Green circle
        cv2.rectangle(img, (center_x - 50, center_y - 50), (center_x + 50, center_y + 50), (0, 0, 255), -1)  # Red square
        
        # Add some random noise to simulate camera noise
        noise = np.random.normal(0, 10, img.shape).astype(np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        
        # Convert to ROS Image message
        img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = f'{self.robot_name}/camera_color_optical_frame'
        
        self.camera_publisher.publish(img_msg)

    def publish_simulated_depth(self):
        """Publish simulated depth camera data"""
        # Create a simulated depth image
        height, width = self.camera_config['height'], self.camera_config['width']
        
        # Create depth image with distance values
        depth_img = np.zeros((height, width), dtype=np.float32)
        
        # Simulate depth based on the same objects as in the RGB image
        center_x, center_y = width // 2, height // 2
        
        # Set depth values for different regions (in meters)
        for y in range(height):
            for x in range(width):
                # Calculate distance from center for circular objects
                dist_to_center = math.sqrt((x - center_x)**2 + (y - center_y)**2)
                
                if dist_to_center < 60:  # Center region
                    depth_img[y, x] = 3.0  # 3 meters away
                elif 70 < dist_to_center < 110:  # Around center
                    depth_img[y, x] = 2.5  # 2.5 meters away
                elif abs(x - (center_x + 100)) < 35 and abs(y - center_y) < 35:  # Blue circle area
                    depth_img[y, x] = 2.0  # 2 meters away
                elif abs(x - (center_x - 100)) < 45 and abs(y - (center_y + 50)) < 45:  # Green circle area
                    depth_img[y, x] = 1.8  # 1.8 meters away
                else:
                    depth_img[y, x] = 10.0  # Background (max distance)
        
        # Add some noise to simulate depth sensor inaccuracy
        noise = np.random.normal(0, 0.02, depth_img.shape).astype(np.float32)
        depth_img = np.clip(depth_img + noise, 0.1, 25.0)  # Clamp to valid range
        
        # Convert to ROS Image message
        depth_msg = self.cv_bridge.cv2_to_imgmsg(depth_img, encoding="32FC1")
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.header.frame_id = f'{self.robot_name}/camera_depth_optical_frame'
        
        self.depth_publisher.publish(depth_msg)

    def publish_simulated_imu(self):
        """Publish simulated IMU data with realistic noise"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.robot_name}/imu_link'
        
        # Set orientation (use robot's current orientation)
        msg.orientation.w = float(self.robot_orientation[0])
        msg.orientation.x = float(self.robot_orientation[1])
        msg.orientation.y = float(self.robot_orientation[2])
        msg.orientation.z = float(self.robot_orientation[3])
        
        # Add noise to orientation
        orientation_noise = np.random.normal(0, self.sim_params['sensor_noise_levels']['imu']/10, 3)
        # For simplicity, we'll just keep the original orientation in this simulation
        
        # Set angular velocity (with noise)
        msg.angular_velocity.x = float(self.robot_angular_velocity[0] + 
                                      np.random.normal(0, self.sim_params['sensor_noise_levels']['imu']))
        msg.angular_velocity.y = float(self.robot_angular_velocity[1] + 
                                      np.random.normal(0, self.sim_params['sensor_noise_levels']['imu']))
        msg.angular_velocity.z = float(self.robot_angular_velocity[2] + 
                                      np.random.normal(0, self.sim_params['sensor_noise_levels']['imu']))
        
        # Set linear acceleration (simulate gravity and movement)
        # Transform gravity vector to robot's coordinate frame
        gravity_world = self.sim_params['gravity']
        
        # Convert robot orientation quaternion to rotation matrix
        qw, qx, qy, qz = self.robot_orientation
        rotation_matrix = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ])
        
        # Calculate gravity in robot's frame
        gravity_robot_frame = rotation_matrix @ gravity_world
        
        # Add linear acceleration due to movement
        linear_acc_robot_frame = rotation_matrix @ self.robot_linear_velocity * 0.1  # Simplified
        
        # Combine gravity and movement acceleration
        total_linear_acc = gravity_robot_frame + linear_acc_robot_frame
        
        # Add noise
        noise = np.random.normal(0, self.sim_params['sensor_noise_levels']['imu'], 3)
        total_linear_acc += noise
        
        msg.linear_acceleration.x = float(total_linear_acc[0])
        msg.linear_acceleration.y = float(total_linear_acc[1])
        msg.linear_acceleration.z = float(total_linear_acc[2])
        
        # Set covariance (indicating uncertainty in measurements)
        # Note: Setting all to zero for simulation - in real systems, these would reflect sensor characteristics
        msg.orientation_covariance = [0.0] * 9
        msg.angular_velocity_covariance = [0.0] * 9
        msg.linear_acceleration_covariance = [0.0] * 9
        
        self.imu_publisher.publish(msg)

    def publish_simulated_odom(self):
        """Publish simulated odometry data"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = f'{self.robot_name}/base_link'
        
        # Set pose
        msg.pose.pose.position.x = float(self.robot_position[0])
        msg.pose.pose.position.y = float(self.robot_position[1])
        msg.pose.pose.position.z = float(self.robot_position[2])
        
        msg.pose.pose.orientation.w = float(self.robot_orientation[0])
        msg.pose.pose.orientation.x = float(self.robot_orientation[1])
        msg.pose.pose.orientation.y = float(self.robot_orientation[2])
        msg.pose.pose.orientation.z = float(self.robot_orientation[3])
        
        # Add noise to pose
        pose_noise = np.random.normal(0, self.sim_params['sensor_noise_levels']['odom'], 3)
        msg.pose.pose.position.x += pose_noise[0]
        msg.pose.pose.position.y += pose_noise[1]
        msg.pose.pose.position.z += pose_noise[2]
        
        # Set twist (velocity)
        msg.twist.twist.linear.x = float(self.robot_linear_velocity[0])
        msg.twist.twist.linear.y = float(self.robot_linear_velocity[1])
        msg.twist.twist.linear.z = float(self.robot_linear_velocity[2])
        
        msg.twist.twist.angular.x = float(self.robot_angular_velocity[0])
        msg.twist.twist.angular.y = float(self.robot_angular_velocity[1])
        msg.twist.twist.angular.z = float(self.robot_angular_velocity[2])
        
        # Add noise to velocities
        linear_vel_noise = np.random.normal(0, self.sim_params['sensor_noise_levels']['odom']*0.1, 3)
        angular_vel_noise = np.random.normal(0, self.sim_params['sensor_noise_levels']['odom']*0.05, 3)
        
        msg.twist.twist.linear.x += linear_vel_noise[0]
        msg.twist.twist.linear.y += linear_vel_noise[1]
        msg.twist.twist.linear.z += linear_vel_noise[2]
        
        msg.twist.twist.angular.x += angular_vel_noise[0]
        msg.twist.twist.angular.y += angular_vel_noise[1]
        msg.twist.twist.angular.z += angular_vel_noise[2]
        
        self.odom_publisher.publish(msg)

    def publish_simulated_pointcloud(self):
        """Publish simulated point cloud data from depth camera"""
        # In a real implementation, this would convert depth image to point cloud
        # For this simulation, we'll create a simple point cloud
        pass  # Implementation would depend on actual depth data processing

    def broadcast_transforms(self):
        """Broadcast TF transforms for robot state"""
        # Create transform for robot base
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = f'{self.robot_name}/base_link'
        
        t.transform.translation.x = float(self.robot_position[0])
        t.transform.translation.y = float(self.robot_position[1])
        t.transform.translation.z = float(self.robot_position[2])
        
        t.transform.rotation.w = float(self.robot_orientation[0])
        t.transform.rotation.x = float(self.robot_orientation[1])
        t.transform.rotation.y = float(self.robot_orientation[2])
        t.transform.rotation.z = float(self.robot_orientation[3])
        
        self.tf_broadcaster.sendTransform(t)
        
        # Create transform for LiDAR sensor
        lidar_t = TransformStamped()
        lidar_t.header.stamp = self.get_clock().now().to_msg()
        lidar_t.header.frame_id = f'{self.robot_name}/base_link'
        lidar_t.child_frame_id = f'{self.robot_name}/lidar_link'
        
        # LiDAR is positioned on top of the robot
        lidar_t.transform.translation.x = 0.0
        lidar_t.transform.translation.y = 0.0
        lidar_t.transform.translation.z = 0.3  # 30cm above base
        lidar_t.transform.rotation.w = 1.0
        lidar_t.transform.rotation.x = 0.0
        lidar_t.transform.rotation.y = 0.0
        lidar_t.transform.rotation.z = 0.0
        
        self.tf_broadcaster.sendTransform(lidar_t)
        
        # Create transform for camera
        camera_t = TransformStamped()
        camera_t.header.stamp = self.get_clock().now().to_msg()
        camera_t.header.frame_id = f'{self.robot_name}/base_link'
        camera_t.child_frame_id = f'{self.robot_name}/camera_color_optical_frame'
        
        # Camera is positioned at front of robot
        camera_t.transform.translation.x = 0.2  # 20cm in front of base
        camera_t.transform.translation.y = 0.0
        camera_t.transform.translation.z = 0.2  # 20cm above base
        camera_t.transform.rotation.w = 1.0  # Looking forward
        camera_t.transform.rotation.x = 0.0
        camera_t.transform.rotation.y = 0.0
        camera_t.transform.rotation.z = 0.0
        
        self.tf_broadcaster.sendTransform(camera_t)

    def quat_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        
        return np.array([w, x, y, z])


def main(args=None):
    rclpy.init(args=args)
    
    bridge_node = IsaacSimBridgeNode()
    
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        bridge_node.get_logger().info('Isaac Sim Bridge interrupted by user')
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()