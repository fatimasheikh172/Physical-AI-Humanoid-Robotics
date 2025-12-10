from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json


class CommandSubscriber(Node):
    """
    A ROS 2 node that subscribes to commands from Unity
    and forwards them to the robot simulation.
    """
    
    def __init__(self):
        super().__init__('command_subscriber')
        
        # Subscriber for commands from Unity
        self.command_subscriber = self.create_subscription(
            String,
            '/unity/commands',
            self.command_callback,
            10
        )
        
        # Publisher for robot commands
        self.robot_command_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.get_logger().info('Command Subscriber Node initialized')

    def command_callback(self, msg):
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
                self.robot_command_publisher.publish(twist_cmd)
                
                self.get_logger().info(f'Forwarded command to robot: {twist_cmd}')
            
            # Add more command types as needed
            
        except json.JSONDecodeError:
            self.get_logger().error(f'Failed to decode command message: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    
    subscriber_node = CommandSubscriber()
    
    try:
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()