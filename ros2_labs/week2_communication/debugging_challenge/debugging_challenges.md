# Week 2 Debugging Challenge Pack

## Overview
This challenge pack contains deliberately introduced bugs in ROS 2 code samples. Your task is to identify and fix the bugs, providing insight into common ROS 2 troubleshooting scenarios.

---

## Challenge 1: The Broken Publisher (Node Issue)

### Code Sample: `broken_publisher.py`
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BrokenPublisher(Node):
    def __init__(self):
        super().init('broken_publisher')  # BUG: Incorrect method name
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

def main(args=None):
    rclpy.init(args=args)
    node = BrokenPublisher()
    rclpy.spin(node)  # This will fail
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Task:
1. Identify the bug
2. Explain why this causes an error
3. Provide the correct code
4. What error message would you expect to see?

---

## Challenge 2: The Broken Service (Interface Issue)

### Code Sample: `broken_service.py`
```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool  # Wrong service type

class BrokenService(Node):
    def __init__(self):
        super().__init__('broken_service')
        self.srv = self.create_service(SetBool, '/wrong_service_name', self.callback)  # Inconsistent
        self.get_logger().info('Service created')

    def callback(self, request, response):
        # Service expects SetBool (request with data: bool), but we'll treat it as a custom service
        self.get_logger().info(f'Received request: {request.some_field_not_in_SetBool}')  # Field doesn't exist
        response.success = True
        response.message = "Done"
        return response  # This will fail due to wrong response type

def main(args=None):
    rclpy.init(args=args)
    node = BrokenService()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Task:
1. Identify the interface mismatch error
2. Explain why this code won't work as intended
3. Correct the service definition to match what's being tried to achieve
4. What error messages would you expect?

---

## Challenge 3: The Broken Action (Callback Issue)

### Code Sample: `broken_action.py`
```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rcl_interfaces.action import UnknownAction  # This action type doesn't exist for this challenge

class BrokenAction(Node):
    def __init__(self):
        super().__init__('broken_action_server')
        # Trying to create an action server for an action that doesn't exist
        self._action_server = ActionServer(
            self,
            UnknownAction,  # Non-existent action type
            'nonexistent_action',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        # Even if the action existed, there's a bug here too
        feedback_msg = UnknownAction.Feedback()  # Can't create feedback for non-existent action
        result = UnknownAction.Result()
        
        # Simulate some work
        for i in range(1, 11):
            if goal_handle.status == 'cancelled':  # BUG: status is not checked this way
                goal_handle.canceled()
                result.completed = False
                return result
                
            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Progress: {i}/10')
            
            # Sleep
            import time
            time.sleep(1)
        
        goal_handle.succeed()
        result.completed = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = BrokenAction()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Action server failed: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Task:
1. Identify the missing action interface issue
2. Note the incorrect status check in the loop
3. Explain what would happen when this code runs
4. How would you fix it properly?

---

## Challenge 4: The Topic Subscription Problem (QoS Mismatch)

### Code Sample: `publisher_qos.py`
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

class PublisherQoS(Node):
    def __init__(self):
        super().__init__('publisher_qos')
        # Publishing with reliable QoS
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.publisher_ = self.create_publisher(String, 'qos_test_topic', qos_profile)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Message {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherQoS()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Sample: `subscriber_qos.py`
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SubscriberQoS(Node):
    def __init__(self):
        super().__init__('subscriber_qos')
        # Subscribing with best effort - incompatible with publisher
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT  # Incompatible with publisher's RELIABLE
        )
        self.subscription = self.create_subscription(
            String,
            'qos_test_topic',
            self.listener_callback,
            qos_profile
        )
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'Subscribed: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberQoS()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Task:
1. Identify the QoS mismatch
2. Explain what will happen when running both nodes together
3. How would you fix the QoS to make them compatible?
4. What are the implications of your fix?

---

## Challenge 5: The Launch File Problem

### Code Sample: `broken_launch.py`
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='week2_communication',  # Correct package name?
            executable='broken_publisher',  # Is this the right executable name?
            name='publisher_node',
            output='screen',
        ),
        Node(
            package='week2_communication',
            executable='broken_subscriber',  # Does this executable exist?
            name='subscriber_node',
            parameters=[
                {'param1': 'value1'}  # Are these parameters valid?
            ],
            output='screen',
        )
    ])
```

### Task:
1. Identify potential issues in the launch configuration
2. What would happen if you tried to execute this launch file?
3. How would you verify the package and executable names?
4. What tools would you use to debug a launch file issue?

---

## Challenge 6: The Parameter Problem

### Code Sample: `parameter_node.py`
```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with incorrect types
        self.declare_parameter('robot_name', 'unknown')  # Default value type doesn't match use
        self.declare_parameter('connection_timeout', 30.5)  # Declaring float but using as int
        self.declare_parameter('max_velocity', 1.0)
        
        # Attempting to get parameters incorrectly
        robot_name = self.get_parameter('robot_name')  # Returns parameter structure, not value
        timeout = self.get_parameter('connection_timeout').value  # Correct way, but declared wrong
        max_vel = self.get_parameter('nonexistent_param', 2.0)  # Wrong default value method
        
        self.get_logger().info(f'Robot: {robot_name}, Timeout: {timeout}, Max Vel: {max_vel}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    try:
        rclpy.spin_once(node, timeout_sec=1)  # Short spin to see log
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Task:
1. Identify the parameter handling bugs
2. Explain the correct way to declare and retrieve parameters
3. How would you set parameters from a launch file or command line?
4. What error messages might you expect?

---

## Challenge 7: The Memory Issue (Subscription Callback)

### Code Sample: `memory_leak_subscriber.py`
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MemoryLeakSubscriber(Node):
    def __init__(self):
        super().__init__('memory_leak_subscriber')
        self.subscription = self.create_subscription(
            String,
            'memory_test_topic',
            self.listener_callback,
            10)
        self.subscription
        self.message_history = []  # Growing list without bounds

    def listener_callback(self, msg):
        # Bug: Never clearing the history, causing memory growth
        self.message_history.append(msg.data)
        self.get_logger().info(f'Received: "{msg.data}". Total messages: {len(self.message_history)}')

def main(args=None):
    rclpy.init(args=args)
    node = MemoryLeakSubscriber()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Task:
1. Identify the memory issue
2. Suggest a proper way to handle message history
3. How would you detect this issue in a running system?
4. What tools can be used to monitor memory usage?

---

## Solutions Guide

For each challenge, provide:

1. **Bug Identification**: What is the specific bug?
2. **Root Cause**: Why does this cause problems?
3. **Solution**: How to fix the issue?
4. **Prevention**: How could this be prevented in the future?

### Challenge 1 Solution Template:
1. Bug: Incorrect initialization method name `super().init()` instead of `super().__init__()`
2. Root Cause: The Node class expects `__init__()` method to be called, leading to initialization failures
3. Solution: Change to `super().__init__('broken_publisher')`
4. Prevention: Use proper IDE with linting, follow Python constructor patterns consistently

---

## Challenge Completion

After identifying and understanding these bugs:
1. Create corrected versions of the code
2. Explain what error messages you'd expect to see
3. Describe how you'd approach debugging similar errors in the future
4. Summarize common debugging strategies for ROS 2 systems

Submit your findings as a report with:
- Identified bugs and their root causes
- Corrected code for each challenge
- Debugging strategies and tools used
- Lessons learned about ROS 2 development