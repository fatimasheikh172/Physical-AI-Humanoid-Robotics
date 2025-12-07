# Demo nodes for Week 1 - Simple Publisher and Subscriber

# This file contains both the publisher and subscriber nodes
# In a real implementation, these would be in separate files

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    """
    A simple publisher node that publishes messages to a topic
    """
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


class SimpleSubscriber(Node):
    """
    A simple subscriber node that subscribes to messages from a topic
    """
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    # This function would normally take a command line argument
    # to determine whether to run the publisher or subscriber
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == 'subscriber':
        node = SimpleSubscriber()
    else:
        # Default to publisher
        node = SimplePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()