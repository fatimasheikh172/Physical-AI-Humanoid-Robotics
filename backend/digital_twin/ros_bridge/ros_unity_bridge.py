from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image, Imu
import json
import threading
from .bridge_node import DigitalTwinBridgeNode


class ROSUnityBridgeNode(DigitalTwinBridgeNode):
    """
    Enhanced ROS 2 node that serves as a bridge between Gazebo simulation
    and Unity visualization for the digital twin system with better integration.
    """
    
    def __init__(self):
        super().__init__('ros_unity_bridge')

        # Additional publishers for Unity visualization
        self.unity_lidar_publisher = self.create_publisher(
            String,
            '/unity/lidar_data',
            10
        )

        self.unity_depth_image_publisher = self.create_publisher(
            String,
            '/unity/depth_image',
            10
        )

        self.unity_imu_publisher = self.create_publisher(
            String,
            '/unity/imu_data',
            10
        )

        # Subscriber to receive commands from Unity
        self.unity_command_subscriber = self.create_subscription(
            String,
            '/unity/commands',
            self.unity_command_callback,
            10
        )

        # Publisher for robot commands from Unity
        self.unity_command_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Set up subscriptions for all sensors
        self.setup_subscriptions()

        self.get_logger().info('ROS-Unity Bridge Node initialized')

    def unity_command_callback(self, msg):
        """
        Callback function to handle commands from Unity
        """
        try:
            # Parse the command message from Unity
            command_data = json.loads(msg.data)

            # Extract command type and parameters
            command_type = command_data.get('type', '')
            params = command_data.get('params', {})

            if command_type == 'velocity_command':
                # Create a Twist message from the command
                twist_cmd = Twist()
                twist_cmd.linear.x = params.get('linear_x', 0.0)
                twist_cmd.linear.y = params.get('linear_y', 0.0)
                twist_cmd.linear.z = params.get('linear_z', 0.0)
                twist_cmd.angular.x = params.get('angular_x', 0.0)
                twist_cmd.angular.y = params.get('angular_y', 0.0)
                twist_cmd.angular.z = params.get('angular_z', 0.0)

                # Publish the command to the robot
                self.unity_command_publisher.publish(twist_cmd)

                self.get_logger().info(f'Received command from Unity: {twist_cmd}')

            # Add more command types as needed

        except json.JSONDecodeError:
            self.get_logger().error(f'Failed to decode command message from Unity: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing command from Unity: {str(e)}')

    def lidar_callback(self, msg):
        """
        Override to handle LiDAR messages and forward to Unity
        """
        # Call parent implementation
        super().sensor_callback(msg)

        # Prepare LiDAR data for Unity
        lidar_data = {
            'ranges': [r if r == r else -1.0 for r in msg.ranges],  # Replace NaN with -1
            'intensities': list(msg.intensities),
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'time_increment': msg.time_increment,
            'scan_time': msg.scan_time,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'timestamp': self.get_clock().now().seconds_nanoseconds()
        }

        # Publish to Unity
        lidar_msg = String()
        lidar_msg.data = json.dumps(lidar_data)
        self.unity_lidar_publisher.publish(lidar_msg)

    def depth_image_callback(self, msg):
        """
        Callback to handle depth camera messages and forward to Unity
        """
        # For this implementation, we'll just send metadata
        # In a real implementation, you'd send the actual image data
        depth_data = {
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
            'timestamp': self.get_clock().now().seconds_nanoseconds()
        }

        depth_msg = String()
        depth_msg.data = json.dumps(depth_data)
        self.unity_depth_image_publisher.publish(depth_msg)

    def imu_callback(self, msg):
        """
        Callback to handle IMU messages and forward to Unity
        """
        imu_data = {
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
            },
            'timestamp': self.get_clock().now().seconds_nanoseconds()
        }

        imu_msg = String()
        imu_msg.data = json.dumps(imu_data)
        self.unity_imu_publisher.publish(imu_msg)

    def setup_subscriptions(self):
        """
        Additional method to set up all required subscriptions
        """
        # LiDAR subscription
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/robot/laser_scan',
            self.lidar_callback,
            10
        )

        # Depth camera subscription
        self.depth_image_subscriber = self.create_subscription(
            Image,
            '/robot/depth_camera/image_raw',
            self.depth_image_callback,
            10
        )

        # IMU subscription
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/robot/imu',
            self.imu_callback,
            10
        )


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    
    bridge_node = ROSUnityBridgeNode()
    
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()