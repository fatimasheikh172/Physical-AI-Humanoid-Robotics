from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import json


class DigitalTwinBridgeNode(Node):
    """
    A ROS 2 node that serves as a bridge between Gazebo simulation
    and Unity visualization for the digital twin system.
    """
    
    def __init__(self):
        super().__init__('digital_twin_bridge')
        
        # Publisher for Unity to subscribe to robot states
        self.unity_robot_state_publisher = self.create_publisher(
            String, 
            '/unity/robot_state', 
            10
        )
        
        # Publisher for Unity to subscribe to sensor data
        self.unity_sensor_publisher = self.create_publisher(
            String, 
            '/unity/sensor_data', 
            10
        )
        
        # Subscriber to receive robot odometry from Gazebo
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/robot/odom',
            self.odom_callback,
            10
        )
        
        # Subscriber to receive sensor data from Gazebo
        self.sensor_subscriber = self.create_subscription(
            String,  # Using String for flexibility; could be specific sensor msg types
            '/gazebo/sensor_data',
            self.sensor_callback,
            10
        )
        
        # Timer for periodic updates to Unity
        self.timer = self.create_timer(0.1, self.publish_to_unity)  # 10Hz
        
        self.get_logger().info('Digital Twin Bridge Node initialized')

    def odom_callback(self, msg):
        """
        Callback function to handle odometry messages from Gazebo
        """
        # Extract position and orientation from odometry message
        position = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
        
        orientation = {
            'x': msg.pose.pose.orientation.x,
            'y': msg.pose.pose.orientation.y,
            'z': msg.pose.pose.orientation.z,
            'w': msg.pose.pose.orientation.w
        }
        
        # Store the robot state for publishing to Unity
        self.robot_state = {
            'timestamp': self.get_clock().now().seconds_nanoseconds(),
            'position': position,
            'orientation': orientation
        }

    def sensor_callback(self, msg):
        """
        Callback function to handle sensor data from Gazebo
        """
        # Store the sensor data for publishing to Unity
        self.sensor_data = {
            'timestamp': self.get_clock().now().seconds_nanoseconds(),
            'data': msg.data  # This could be parsed further depending on sensor type
        }

    def publish_to_unity(self):
        """
        Publish the robot state and sensor data to Unity
        """
        # Publish robot state if available
        if hasattr(self, 'robot_state'):
            robot_state_msg = String()
            robot_state_msg.data = json.dumps(self.robot_state)
            self.unity_robot_state_publisher.publish(robot_state_msg)
        
        # Publish sensor data if available
        if hasattr(self, 'sensor_data'):
            sensor_data_msg = String()
            sensor_data_msg.data = json.dumps(self.sensor_data)
            self.unity_sensor_publisher.publish(sensor_data_msg)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    
    bridge_node = DigitalTwinBridgeNode()
    
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()