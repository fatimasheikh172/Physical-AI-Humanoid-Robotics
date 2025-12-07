"""
AI Tutor for Week 1 ROS 2 Concepts
Implements the AI tutor functionality for ROS 2 concepts
"""
from typing import Dict, Any
import json


class ROS2Tutor:
    """
    AI Tutor class to provide explanations for ROS 2 concepts
    """
    
    def __init__(self):
        # Knowledge base for Week 1 concepts
        self.knowledge_base = {
            "ros_definition": {
                "explanation": "ROS stands for Robot Operating System. It's not actually an operating system but a middleware framework that provides services like hardware abstraction, device drivers, libraries, and message-passing between processes.",
                "examples": [
                    "ROS helps different parts of a robot communicate",
                    "ROS provides tools for testing and debugging robot software",
                    "ROS allows code reuse across different robotic platforms"
                ]
            },
            "dds_explanation": {
                "explanation": "DDS (Data Distribution Service) is the underlying communication middleware for ROS 2. It provides a standardized way for different parts of the system to exchange data reliably.",
                "key_features": [
                    "Data-centric communication",
                    "Quality of Service (QoS) policies",
                    "Automatic discovery of participants",
                    "Language and platform independence"
                ]
            },
            "ros_vs_ros2": {
                "explanation": "ROS 2 addresses limitations in ROS 1 with improvements like real-time support, better security, improved multi-robot systems support, and commercial viability.",
                "differences": [
                    "ROS 2 uses DDS for communication, ROS 1 used its own protocol",
                    "ROS 2 has built-in security features",
                    "ROS 2 supports real-time systems better",
                    "ROS 2 has better support for multi-robot systems"
                ]
            },
            "node_topic_service": {
                "explanation": "In ROS 2, nodes are processes that perform computation. They communicate with each other using topics (publish/subscribe), services (request/response), and actions (for long-running tasks).",
                "components": {
                    "nodes": "Processes that perform computation",
                    "topics": "Asynchronous data streams",
                    "services": "Synchronous request/response communication",
                    "actions": "Asynchronous communication for long-running tasks with feedback"
                }
            }
        }
    
    async def explain_concept(self, concept: str) -> str:
        """
        Provide an explanation for a ROS 2 concept in beginner-friendly language
        """
        concept = concept.lower().replace(" ", "_")
        
        if concept in self.knowledge_base:
            concept_data = self.knowledge_base[concept]
            explanation = f"## {concept.replace('_', ' ').title()}\n\n"
            explanation += f"{concept_data['explanation']}\n\n"
            
            if 'examples' in concept_data:
                explanation += "### Examples:\n"
                for example in concept_data['examples']:
                    explanation += f"- {example}\n"
            
            if 'key_features' in concept_data:
                explanation += "### Key Features:\n"
                for feature in concept_data['key_features']:
                    explanation += f"- {feature}\n"
            
            if 'differences' in concept_data:
                explanation += "### Key Differences:\n"
                for diff in concept_data['differences']:
                    explanation += f"- {diff}\n"
            
            if 'components' in concept_data:
                explanation += "### Components:\n"
                for component, description in concept_data['components'].items():
                    explanation += f"- **{component}**: {description}\n"
            
            return explanation
        else:
            return f"I don't have information about '{concept.replace('_', ' ')}'. Please ask about ROS concepts like 'ROS definition', 'DDS explanation', 'ROS vs ROS2', or 'node topic service'."
    
    async def debug_error(self, error_message: str) -> str:
        """
        Provide debugging assistance for common ROS 2 errors
        """
        error_message = error_message.lower()
        
        # Common error patterns and solutions
        if "command not found" in error_message or "ros2" in error_message:
            return ("It looks like you haven't sourced your ROS 2 environment. Try running:\n"
                   "```bash\nsource /opt/ros/humble/setup.bash\n```\n"
                   "If you want this to happen automatically, add that line to your ~/.bashrc file.")
        
        elif "module" in error_message and "rclpy" in error_message:
            return ("This error suggests that rclpy (Python ROS client library) is not available. "
                   "Make sure ROS 2 is properly installed and sourced. "
                   "Try running: `source /opt/ros/humble/setup.bash` before running your Python script.")
        
        elif "node" in error_message and "initialized" in error_message:
            return ("This error typically occurs when rclpy hasn't been initialized before creating a node. "
                   "Make sure your Python ROS 2 node starts with `rclpy.init()` before creating the node object.")
        
        elif "permission" in error_message or "access" in error_message:
            return ("This might be a permissions issue. Try using `sudo` if necessary, but be careful. "
                   "For ROS 2 operations, you typically don't need sudo. Check file permissions if working with files.")
        
        elif "build" in error_message or "colcon" in error_message:
            return ("Building errors often happen due to missing dependencies or incorrect workspace setup. "
                   "Make sure to run `source /opt/ros/humble/setup.bash` first, and verify all dependencies are installed. "
                   "You might need to install additional packages with `rosdep install --from-paths src --ignore-src -r -y`")
        
        else:
            return (f"I'm not sure about the specific error '{error_message}', but here are some general ROS 2 troubleshooting steps:\n"
                   "- Make sure ROS 2 environment is sourced: `source /opt/ros/humble/setup.bash`\n"
                   "- Check that your Python packages are accessible\n"
                   "- Verify that your workspace is properly built\n"
                   "- Ensure all dependencies are installed")
    
    async def provide_example(self, topic: str) -> str:
        """
        Provide a practical example related to a ROS 2 topic
        """
        topic = topic.lower()
        
        if "publisher" in topic or "topic" in topic:
            return ("### Simple Publisher Example\n\n"
                   "```python\n"
                   "import rclpy\n"
                   "from rclpy.node import Node\n"
                   "from std_msgs.msg import String\n\n"
                   
                   "class SimplePublisher(Node):\n"
                   "    def __init__(self):\n"
                   "        super().__init__('simple_publisher')\n"
                   "        self.publisher_ = self.create_publisher(String, 'topic', 10)\n"
                   "        timer_period = 0.5  # seconds\n"
                   "        self.timer = self.create_timer(timer_period, self.timer_callback)\n"
                   "        self.i = 0\n\n"
                   
                   "    def timer_callback(self):\n"
                   "        msg = String()\n"
                   "        msg.data = f'Hello World: {self.i}'\n"
                   "        self.publisher_.publish(msg)\n"
                   "        self.get_logger().info(f'Publishing: \"{msg.data}\"')\n"
                   "        self.i += 1\n"
                   "```\n\n"
                   "To run this publisher: `ros2 run package_name simple_publisher`")
        
        elif "subscriber" in topic:
            return ("### Simple Subscriber Example\n\n"
                   "```python\n"
                   "import rclpy\n"
                   "from rclpy.node import Node\n"
                   "from std_msgs.msg import String\n\n"
                   
                   "class SimpleSubscriber(Node):\n"
                   "    def __init__(self):\n"
                   "        super().__init__('simple_subscriber')\n"
                   "        self.subscription = self.create_subscription(\n"
                   "            String,\n"
                   "            'topic',\n"
                   "            self.listener_callback,\n"
                   "            10)\n"
                   "        self.subscription  # prevent unused variable warning\n\n"
                   
                   "    def listener_callback(self, msg):\n"
                   "        self.get_logger().info(f'I heard: \"{msg.data}\"')\n"
                   "```\n\n"
                   "To run this subscriber: `ros2 run package_name simple_subscriber`")
        
        elif "service" in topic:
            return ("### Simple Service Example\n\n"
                   "Services in ROS 2 allow for synchronous request/response communication:\n\n"
                   
                   "To create a service, you first need a service definition file (.srv),\n"
                   "then implement a service server and client.\n\n"
                   
                   "To call a service: `ros2 service call /service_name service_type '{request_fields}'`\n\n"
                   
                   "Services are useful when you need to get specific information or\n"
                   "trigger a specific action with confirmation.")
        
        else:
            return f"I don't have a specific example for '{topic}', but you can ask about publishers, subscribers, or services for practical code examples."


# Global instance of the tutor
ros2_tutor = ROS2Tutor()