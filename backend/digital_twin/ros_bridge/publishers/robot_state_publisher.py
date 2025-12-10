from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan, Image, Imu
import json


class RobotStatePublisher(Node):
    """
    A ROS 2 node that publishes robot state information
    for the digital twin system.
    """
    
    def __init__(self):
        super().__init__('robot_state_publisher')
        
        # Publisher for robot pose
        self.pose_publisher = self.create_publisher(
            String, 
            '/digital_twin/robot_pose', 
            10
        )
        
        # Publisher for joint states (placeholder)
        self.joint_states_publisher = self.create_publisher(
            String,
            '/digital_twin/joint_states',
            10
        )
        
        self.get_logger().info('Robot State Publisher Node initialized')

    def publish_pose(self, position, orientation):
        """
        Publish robot pose to the digital twin system
        """
        pose_data = {
            'position': position,
            'orientation': orientation,
            'timestamp': self.get_clock().now().seconds_nanoseconds()
        }
        
        pose_msg = String()
        pose_msg.data = json.dumps(pose_data)
        self.pose_publisher.publish(pose_msg)

    def publish_joint_states(self, joint_states):
        """
        Publish joint states to the digital twin system
        """
        joint_data = {
            'joint_states': joint_states,
            'timestamp': self.get_clock().now().seconds_nanoseconds()
        }
        
        joint_msg = String()
        joint_msg.data = json.dumps(joint_data)
        self.joint_states_publisher.publish(joint_msg)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    
    publisher_node = RobotStatePublisher()
    
    try:
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()