"""
Base ROS 2 node structure for the Vision-Language-Action (VLA) module.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Image
import logging
import asyncio
import threading
from typing import Optional, Callable, Any

from .config import get_config
from .utils import setup_logger
from .error_handling import log_exception, VLAException


class BaseVLANode(Node):
    """
    Base class for all VLA module ROS 2 nodes.
    Provides common functionality and structure.
    """
    
    def __init__(self, node_name: str):
        # Initialize the ROS 2 node
        super().__init__(node_name)
        
        # Get the VLA configuration
        self.config = get_config()
        
        # Set up logging
        self.logger = setup_logger(node_name)
        self.get_logger().info(f"Initializing {node_name}")
        
        # Initialize common attributes
        self._is_running = False
        self._async_loop = None
        self._async_thread = None
        
        # Set up common publishers/subscribers if needed
        self.status_publisher = self.create_publisher(
            String,
            f'/{node_name}/status',
            qos_profile=qos_profile_sensor_data
        )
        
        # Create a timer for periodic tasks (1Hz by default)
        self.timer = self.create_timer(1.0, self._default_timer_callback)
        
        self.get_logger().info(f"{node_name} initialized successfully")
    
    def _default_timer_callback(self):
        """Default timer callback - can be overridden by subclasses"""
        if self._is_running:
            # Publish status
            status_msg = String()
            status_msg.data = "RUNNING"
            self.status_publisher.publish(status_msg)
    
    def start_async_loop(self):
        """Start the asyncio event loop in a separate thread"""
        if self._async_loop is None:
            self._async_loop = asyncio.new_event_loop()
            
            def run_loop():
                asyncio.set_event_loop(self._async_loop)
                self._async_loop.run_forever()
            
            self._async_thread = threading.Thread(target=run_loop, daemon=True)
            self._async_thread.start()
    
    def stop_async_loop(self):
        """Stop the asyncio event loop"""
        if self._async_loop:
            self._async_loop.call_soon_threadsafe(self._async_loop.stop)
            if self._async_thread:
                self._async_thread.join()
    
    def run_async(self, coro):
        """Run a coroutine in the asyncio event loop"""
        if self._async_loop is None:
            self.start_async_loop()
        future = asyncio.run_coroutine_threadsafe(coro, self._async_loop)
        return future
    
    @log_exception()
    def safe_execute(self, func: Callable, *args, **kwargs) -> Any:
        """
        Safely execute a function, catching and logging exceptions.
        
        Args:
            func: Function to execute
            *args: Arguments to pass to the function
            **kwargs: Keyword arguments to pass to the function
            
        Returns:
            Result of the function execution
        """
        try:
            return func(*args, **kwargs)
        except Exception as e:
            self.get_logger().error(f"Error in {func.__name__}: {str(e)}")
            raise VLAException(f"Error executing {func.__name__}: {str(e)}") from e
    
    def on_start(self):
        """Called when the node starts - override in subclasses"""
        self._is_running = True
        self.get_logger().info(f"{self.get_name()} started")
    
    def on_stop(self):
        """Called when the node stops - override in subclasses"""
        self._is_running = False
        self.get_logger().info(f"{self.get_name()} stopped")
    
    def destroy_node(self):
        """Clean up when the node is destroyed"""
        self.on_stop()
        self.stop_async_loop()
        super().destroy_node()


class VoiceCommandNode(BaseVLANode):
    """
    Base class for voice command processing nodes.
    """
    
    def __init__(self, node_name: str = "voice_command_node"):
        super().__init__(node_name)
        
        # Publishers
        self.command_publisher = self.create_publisher(
            String,
            '/vla/voice_commands',
            qos_profile=qos_profile_sensor_data
        )
        
        # Subscribers
        self.audio_subscriber = self.create_subscription(
            String,  # In a real system, this would be a custom audio message type
            '/audio/input',
            self.audio_callback,
            qos_profile=qos_profile_sensor_data
        )
        
        self.get_logger().info("VoiceCommandNode initialized")
    
    def audio_callback(self, msg):
        """Callback for audio input - to be overridden"""
        self.get_logger().info(f"Received audio message: {len(msg.data)} chars")
    
    def publish_command(self, command: str):
        """Publish a recognized command"""
        cmd_msg = String()
        cmd_msg.data = command
        self.command_publisher.publish(cmd_msg)
        self.get_logger().info(f"Published command: {command}")


class CognitivePlannerNode(BaseVLANode):
    """
    Base class for cognitive planning nodes.
    """
    
    def __init__(self, node_name: str = "cognitive_planner_node"):
        super().__init__(node_name)
        
        # Publishers
        self.plan_publisher = self.create_publisher(
            String,  # In a real system, this would be a custom plan message type
            '/vla/cognitive_plans',
            qos_profile=qos_profile_sensor_data
        )
        
        # Subscribers
        self.command_subscriber = self.create_subscription(
            String,
            '/vla/voice_commands',
            self.command_callback,
            qos_profile=qos_profile_sensor_data
        )
        
        self.get_logger().info("CognitivePlannerNode initialized")
    
    def command_callback(self, msg):
        """Callback for command input - to be overridden"""
        self.get_logger().info(f"Received command: {msg.data}")
    
    def publish_plan(self, plan: str):
        """Publish a generated plan"""
        plan_msg = String()
        plan_msg.data = plan
        self.plan_publisher.publish(plan_msg)
        self.get_logger().info(f"Published plan")


class ActionExecutionNode(BaseVLANode):
    """
    Base class for action execution nodes.
    """
    
    def __init__(self, node_name: str = "action_execution_node"):
        super().__init__(node_name)
        
        # Publishers
        self.execution_status_publisher = self.create_publisher(
            String,  # In a real system, this would be a custom execution status message type
            '/vla/action_execution_status',
            qos_profile=qos_profile_sensor_data
        )
        
        # Subscribers
        self.plan_subscriber = self.create_subscription(
            String,
            '/vla/cognitive_plans',
            self.plan_callback,
            qos_profile=qos_profile_sensor_data
        )
        
        self.get_logger().info("ActionExecutionNode initialized")
    
    def plan_callback(self, msg):
        """Callback for plan input - to be overridden"""
        self.get_logger().info(f"Received plan: {msg.data}")
    
    def publish_execution_status(self, status: str):
        """Publish action execution status"""
        status_msg = String()
        status_msg.data = status
        self.execution_status_publisher.publish(status_msg)
        self.get_logger().info(f"Published execution status: {status}")


class VisionPerceptionNode(BaseVLANode):
    """
    Base class for vision perception nodes.
    """
    
    def __init__(self, node_name: str = "vision_perception_node"):
        super().__init__(node_name)
        
        # Publishers
        self.vision_result_publisher = self.create_publisher(
            String,  # In a real system, this would be a custom vision result message type
            '/vla/vision_results',
            qos_profile=qos_profile_sensor_data
        )
        
        # Subscribers
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile=qos_profile_sensor_data
        )
        
        self.get_logger().info("VisionPerceptionNode initialized")
    
    def image_callback(self, msg):
        """Callback for image input - to be overridden"""
        self.get_logger().info(f"Received image: {msg.width}x{msg.height}")
    
    def publish_vision_result(self, result: str):
        """Publish vision perception result"""
        result_msg = String()
        result_msg.data = result
        self.vision_result_publisher.publish(result_msg)
        self.get_logger().info(f"Published vision result")


def create_and_spin_node(node_class, node_name: str = None):
    """
    Helper function to create a node and spin it.
    
    Args:
        node_class: Node class to instantiate
        node_name: Optional name for the node
    """
    rclpy.init()
    
    node = node_class(node_name) if node_name else node_class()
    node.on_start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.on_stop()
        node.destroy_node()
        rclpy.shutdown()