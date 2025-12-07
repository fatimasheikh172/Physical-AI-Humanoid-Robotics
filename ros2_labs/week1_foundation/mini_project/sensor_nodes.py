# Week 1 Mini Project - Custom Publisher/Subscriber

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


class SensorPublisher(Node):
    """
    A publisher node that simulates publishing random sensor data
    """
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0
        
        # Simulate some sensor reading
        import random
        self.sensor_types = ['temperature', 'humidity', 'pressure', 'light']
        self.get_logger().info('Sensor publisher node initialized')

    def timer_callback(self):
        import random
        sensor_type = random.choice(self.sensor_types)
        sensor_value = round(random.uniform(10.0, 40.0), 2)  # Simulate sensor value
        msg = String()
        msg.data = f'{sensor_type}: {sensor_value}'
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published sensor data: "{msg.data}"')
        self.counter += 1


class SensorSubscriber(Node):
    """
    A subscriber node that receives and visualizes sensor data
    """
    def __init__(self):
        super().__init__('sensor_subscriber')
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.data_log = []
        self.get_logger().info('Sensor subscriber node initialized')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received sensor data: "{msg.data}"')
        self.data_log.append(msg.data)
        
        # Simple visualization: log last 5 readings
        if len(self.data_log) > 5:
            self.data_log.pop(0)  # Remove oldest entry
        
        self.get_logger().info(f'Last 5 readings: {self.data_log}')


def main(args=None):
    rclpy.init(args=args)
    
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == 'subscriber':
        node = SensorSubscriber()
    else:
        # Default to publisher
        node = SensorPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()