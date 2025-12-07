# Lab Activity: Quality of Service (QoS) Concepts in ROS 2

## Objective
Understand and implement different Quality of Service (QoS) profiles in ROS 2 to control message delivery behavior and system reliability.

## Prerequisites
- Understanding of ROS 2 topics, publishers, and subscribers
- Completed previous communication labs

## Background

Quality of Service (QoS) is a crucial concept in ROS 2 that allows fine-grained control over how messages are delivered between nodes. QoS settings determine the behavior of communication in terms of reliability, durability, history, and other factors.

## QoS Policies Explained

### 1. Reliability Policy
Controls whether messages are delivered reliably or on best-effort basis:
- **RELIABLE**: All messages are guaranteed to be delivered (may block if delivery fails)
- **BEST_EFFORT**: Messages may be lost, but no blocking occurs (suitable for video streams)
- **SYSTEM_DEFAULT**: Use the system's default policy

### 2. Durability Policy
Controls how messages are stored and delivered to late-joining nodes:
- **TRANSIENT_LOCAL**: Late joiners receive the most recent message for the topic
- **VOLATILE**: Late joiners receive no historical messages (default for real-time systems)
- **SYSTEM_DEFAULT**: Use the system's default policy

### 3. History Policy
Controls how many messages are stored:
- **KEEP_LAST**: Store a fixed number of most recent messages
- **KEEP_ALL**: Store all messages (limited by resource constraints)
- **SYSTEM_DEFAULT**: Use the system's default policy

### 4. Deadline Policy
Defines the maximum time between consecutive messages.

### 5. Lifespan Policy
Defines how long messages remain valid in the system.

## Lab Activities

### Activity 1: Basic QoS Configuration

#### Task
Create a publisher and subscriber pair with different QoS profiles to observe the effects.

#### Implementation Steps

1. **Create a publisher with RELIABLE settings** (`qos_publisher_reliable.py`):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class QoSPublisherReliable(Node):
    def __init__(self):
        super().__init__('qos_publisher_reliable')
        # Set up QoS with reliable delivery and keep last 10 messages
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.publisher_ = self.create_publisher(String, 'qos_test_topic', qos_profile)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Reliable message {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    qos_publisher = QoSPublisherReliable()
    
    try:
        rclpy.spin(qos_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        qos_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

2. **Create a subscriber with matching QoS** (`qos_subscriber_reliable.py`):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class QoSSubscriberReliable(Node):
    def __init__(self):
        super().__init__('qos_subscriber_reliable')
        # Match the publisher's QoS settings
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.subscription = self.create_subscription(
            String,
            'qos_test_topic',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Subscribed RELIABLE: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    qos_subscriber = QoSSubscriberReliable()
    
    try:
        rclpy.spin(qos_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        qos_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

3. **Create a subscriber with BEST_EFFORT settings** (`qos_subscriber_best_effort.py`):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class QoSSubscriberBestEffort(Node):
    def __init__(self):
        super().__init__('qos_subscriber_best_effort')
        # Use best effort instead of reliable
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.subscription = self.create_subscription(
            String,
            'qos_test_topic',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Subscribed BEST EFFORT: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    qos_subscriber = QoSSubscriberBestEffort()
    
    try:
        rclpy.spin(qos_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        qos_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Testing Procedure
1. Run the reliable publisher: `ros2 run week2_communication qos_publisher_reliable`
2. In another terminal, run the reliable subscriber: `ros2 run week2_communication qos_subscriber_reliable`
3. In a third terminal, run the best effort subscriber: `ros2 run week2_communication qos_subscriber_best_effort`
4. Observe the differences in message reception between the subscribers

### Activity 2: Durability Testing

#### Task
Compare messages received by TRANSIENT_LOCAL vs VOLATILE subscribers when they join after publishing has started.

#### Implementation Steps

1. **Create a publisher with TRANSIENT_LOCAL durability** (`durability_publisher.py`):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

class DurabilityPublisher(Node):
    def __init__(self):
        super().__init__('durability_publisher')
        # Publish with transient local durability
        qos_profile = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_ALL
        )
        self.publisher_ = self.create_publisher(String, 'durability_test_topic', qos_profile)
        timer_period = 5  # Send messages every 5 seconds to allow for joining late
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Transient message {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    durability_publisher = DurabilityPublisher()
    
    try:
        rclpy.spin(durability_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        durability_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

2. **Create subscribers with different durability settings** (`durability_subscribers.py`):

First, a volatile subscriber:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

class VolatileSubscriber(Node):
    def __init__(self):
        super().__init__('volatile_subscriber')
        # Use volatile durability - won't receive historical messages
        qos_profile = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.subscription = self.create_subscription(
            String,
            'durability_test_topic',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Volatile subscriber ready - will not receive historical messages')

    def listener_callback(self, msg):
        self.get_logger().info(f'Volatile subscriber received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    volatile_subscriber = VolatileSubscriber()
    
    try:
        rclpy.spin(volatile_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        volatile_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

And a transient local subscriber:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

class TransientLocalSubscriber(Node):
    def __init__(self):
        super().__init__('transient_local_subscriber')
        # Use transient local durability - will receive historical messages
        qos_profile = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.subscription = self.create_subscription(
            String,
            'durability_test_topic',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Transient local subscriber ready - will attempt to receive historical messages')

    def listener_callback(self, msg):
        self.get_logger().info(f'Transient local subscriber received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    transient_local_subscriber = TransientLocalSubscriber()
    
    try:
        rclpy.spin(transient_local_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        transient_local_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Testing Procedure
1. Start the publisher: `ros2 run week2_communication durability_publisher`
2. Wait for several messages to be published
3. Start the volatile subscriber: `ros2 run week2_communication volatile_subscriber`
4. Wait and then start the transient local subscriber: `ros2 run week2_communication transient_local_subscriber`
5. Observe which subscriber receives the historical messages

### Activity 3: QoS Matching and Mismatching

#### Task
Explore what happens when publishers and subscribers have mismatched QoS policies.

#### Implementation Steps

1. **Create a publisher with strict QoS requirements** (`strict_publisher.py`):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class StrictPublisher(Node):
    def __init__(self):
        super().__init__('strict_publisher')
        # Very specific QoS settings
        qos_profile = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.publisher_ = self.create_publisher(String, 'strict_qos_topic', qos_profile)
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Strict QoS message {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    strict_publisher = StrictPublisher()
    
    try:
        rclpy.spin(strict_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        strict_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

2. **Create a subscriber with different QoS settings** (`relaxed_subscriber.py`):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class RelaxedSubscriber(Node):
    def __init__(self):
        super().__init__('relaxed_subscriber')
        # Different QoS settings that may not match publisher
        qos_profile = QoSProfile(
            depth=10,  # Different depth
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Different reliability
            durability=DurabilityPolicy.VOLATILE,  # Different durability
            history=HistoryPolicy.KEEP_ALL  # Different history
        )
        self.subscription = self.create_subscription(
            String,
            'strict_qos_topic',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Relaxed subscriber with mismatched QoS started')

    def listener_callback(self, msg):
        self.get_logger().info(f'Relaxed subscriber received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    relaxed_subscriber = RelaxedSubscriber()
    
    try:
        rclpy.spin(relaxed_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        relaxed_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Testing Procedure
1. Start the strict publisher: `ros2 run week2_communication strict_publisher`
2. Start the relaxed subscriber: `ros2 run week2_communication relaxed_subscriber`
3. Observe what messages (if any) are received
4. Check for any warnings or errors about QoS incompatibility

## Performance Testing Lab

### Task Description
Compare performance metrics under different QoS settings.

### Exercises
1. **Latency Test**: Measure message delivery time with different reliability settings
2. **Throughput Test**: Measure messages per second with different history policies
3. **Resource Usage**: Monitor CPU and memory consumption with different QoS settings

### Example Performance Monitoring Code:
```python
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

def create_latency_test_qos():
    # QoS profile for latency testing
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,  # or BEST_EFFORT
        history=HistoryPolicy.KEEP_LAST
    )

def create_throughput_test_qos():
    # QoS profile for throughput testing
    return QoSProfile(
        depth=100,  # Larger buffer for handling bursts
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_ALL
    )
```

## Expected Outcomes

After completing this lab, you should:
1. Understand how QoS policies affect communication behavior
2. Be able to configure appropriate QoS settings for different use cases
3. Know how to diagnose QoS-related communication issues
4. Appreciate the trade-offs between different QoS configurations
5. Be able to optimize QoS settings based on performance requirements

## Use Case Recommendations

- **RELIABLE**: Safety-critical communications, control commands, configuration updates
- **BEST_EFFORT**: Video streams, some sensor data where occasional loss is acceptable
- **TRANSIENT_LOCAL**: Parameter updates, system status messages, maps that new nodes should receive
- **VOLATILE**: Real-time sensor data, where only the most current information matters
- **KEEP_LAST**: When you only care about the most recent values
- **KEEP_ALL**: When historical data is important (logging, debugging)

## Troubleshooting Tips

- Use `ros2 topic info -v <topic_name>` to see QoS settings of existing topics
- When communication fails, check QoS compatibility between publisher and subscriber
- For performance issues, consider adjusting history depth and reliability settings
- Monitor ROS 2 logging for QoS incompatibility warnings