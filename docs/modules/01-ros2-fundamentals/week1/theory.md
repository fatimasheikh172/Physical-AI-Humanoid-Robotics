# Module 1: The Robotic Nervous System (ROS 2)

## Introduction: Why Robots Need a Nervous System

Just as humans have a nervous system that connects the brain to the rest of the body, robots need a "nervous system" to connect their processing units (brains) to their sensors and actuators (bodies). The Robot Operating System 2 (ROS 2) serves as the robotic nervous system, enabling complex robotic behaviors by connecting all the components of a robot in a coordinated and effective manner.

## The Robotic Nervous System: An analogy

Human nervous system structure:
- **Central nervous system**: Brain and spinal cord (computation and coordination)
- **Peripheral nervous system**: Nerves that carry signals to/from organs (communication)
- **Sensory neurons**: Detect stimuli from environment (sensors)
- **Motor neurons**: Transmit signals to muscles (actuators)

ROS 2 nervous system structure:
- **ROS 2 Core Infrastructure**: Master nodes and communication middleware (computation and coordination)
- **Nodes and Topics**: Communication channels between components (nerves)
- **Sensors**: Subscribers to sensor data (sensory neurons)
- **Actuators**: Publishers of control commands (motor neurons)

## ROS 2 Nodes: Your Robotic Brain Cells

A ROS 2 node is the fundamental building block of a robotic application, similar to how a neuron is the basic unit of the nervous system. Each node performs a specific function, such as processing sensor data, making decisions, or controlling actuators.

### Creating a Node

In ROS 2, nodes are created using client libraries like rclpy (Python) or rclcpp (C++):

```python
import rclpy
from rclpy.node import Node

class RoboticBrainCell(Node):
    def __init__(self):
        super().__init__('brain_cell')
        self.get_logger().info('Robotic neuron initialized')

def main(args=None):
    rclpy.init(args=args)
    node = RoboticBrainCell()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Communication Patterns

Nodes communicate using three primary patterns:
- **Publish/Subscribe**: Asynchronous communication (like sensory perception)
- **Request/Response**: Synchronous communication (like reflexive responses)
- **Action-based**: Asynchronous long-running tasks (like complex behaviors)

## Topics: The Neural Pathways

Topics in ROS 2 are communication channels where nodes publish and subscribe to messages. Think of topics as the neural pathways in a nervous system, carrying information between different parts of the robot.

### Creating Publishers and Subscribers

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NervousSystemHub(Node):
    def __init__(self):
        super().__init__('nervous_system_hub')

        # Publisher (like a motor neuron sending signals to muscles)
        self.publisher = self.create_publisher(String, 'robot_commands', 10)

        # Subscriber (like a sensory neuron receiving environmental data)
        self.subscription = self.create_subscription(
            String,
            'sensory_input',
            self.sensory_callback,
            10)

    def sensory_callback(self, msg):
        self.get_logger().info(f'Received sensory input: {msg.data}')
        # Process and send motor commands
        command_msg = String()
        command_msg.data = f'Response to: {msg.data}'
        self.publisher.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NervousSystemHub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Services: The Reflex Arcs

Services in ROS 2 provide request/response communication, similar to a reflex arc in the biological nervous system. When a specific condition is met, a service call is made to trigger an immediate, synchronous response.

### Defining and Using Services

```python
# Service definition would typically be in a .srv file like:
# ---
# # request
# string request
# ---
# # response
# string response

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ReflexController(Node):
    def __init__(self):
        super().__init__('reflex_controller')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Reflex response: {request.a} + {request.b} = {response.sum}')
        return response
```

## Quality of Service (QoS): Nervous System Priorities

Just as the human nervous system prioritizes different types of information differently (life-threatening stimuli get priority), ROS 2 allows you to set Quality of Service policies to prioritize different communications:

- **Reliability**: How critical is it that a message is delivered?
- **Durability**: How long should messages persist?
- **History**: How many recent messages should be retained?
- **Deadline**: How quickly must the communication happen?