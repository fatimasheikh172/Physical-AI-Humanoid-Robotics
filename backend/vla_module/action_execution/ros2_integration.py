"""
Third-party integration for ROS 2 (Robot Operating System 2) in the Vision-Language-Action (VLA) module.

This module provides the integration layer between the VLA system and ROS 2,
enabling communication with robots and other ROS 2-based systems.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor

import asyncio
import logging
import threading
from typing import Dict, Any, Optional, List
from dataclasses import dataclass

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import Action, ActionResponse, ExecutionStatus
from ..core.data_models import ActionModel, ActionResponseModel
from .action_sequencer import get_action_sequencer
from .robot_controller import get_robot_controller


class ROS2Interface(Node):
    """
    ROS 2 interface node that handles communication with the ROS 2 ecosystem.
    """
    
    def __init__(self, node_name: str = "vla_ros2_interface"):
        # Initialize ROS 2 node
        super().__init__(node_name)
        
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Create action clients for various robot actions
        self.action_clients: Dict[str, ActionClient] = {}
        
        # Service clients
        self.service_clients = {}
        
        # Publishers and subscribers
        self.publishers = {}
        self.subscribers = {}
        
        # Action execution tracking
        self.active_goals = {}
        self.goal_futures = {}
        
        # Threading for async integration
        self.loop = None
        self.executor = None
        self.thread = None
        
        self.logger.info("ROS2Interface initialized")
    
    def start_ros2_spinning(self):
        """Start the ROS 2 spinning in a separate thread."""
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self)
        
        def spin():
            self.executor.spin()
        
        self.thread = threading.Thread(target=spin, daemon=True)
        self.thread.start()
        self.logger.info("ROS 2 spinning started in separate thread")
    
    def stop_ros2_spinning(self):
        """Stop the ROS 2 spinning."""
        if self.executor:
            self.executor.shutdown()
        if self.thread:
            self.thread.join()
        self.logger.info("ROS 2 spinning stopped")
    
    def register_action_client(self, action_type: str, action_interface, action_topic: str):
        """
        Register an action client for a specific action type.
        
        Args:
            action_type: Type of action (e.g., 'move_to', 'pick_up', 'place')
            action_interface: ROS 2 action interface class
            action_topic: Topic name for the action
        """
        try:
            # Create action client
            action_client = ActionClient(
                self,
                action_interface,
                action_topic
            )
            
            # Wait for action server to be available
            self.logger.info(f"Waiting for action server: {action_topic}")
            action_client.wait_for_server(timeout_sec=10.0)  # Wait up to 10 seconds
            
            # Store the action client
            self.action_clients[action_type] = action_client
            self.logger.info(f"Action client registered for: {action_type} at {action_topic}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to register action client for {action_type} at {action_topic}: {e}")
            return False
    
    def register_service_client(self, service_type: str, service_interface, service_topic: str):
        """
        Register a service client for a specific service type.
        
        Args:
            service_type: Type of service
            service_interface: ROS 2 service interface class
            service_topic: Topic name for the service
        """
        try:
            # Create service client
            service_client = self.create_client(
                service_interface,
                service_topic
            )
            
            # Wait for service to be available
            self.logger.info(f"Waiting for service: {service_topic}")
            while not service_client.wait_for_service(timeout_sec=1.0):
                self.logger.info(f"Service {service_topic} not available, waiting again...")
            
            # Store the service client
            self.service_clients[service_type] = service_client
            self.logger.info(f"Service client registered for: {service_type} at {service_topic}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to register service client for {service_type} at {service_topic}: {e}")
            return False
    
    @log_exception()
    async def execute_action_via_ros2(self, action: ActionModel) -> ActionResponseModel:
        """
        Execute an action through the ROS 2 interface.
        
        Args:
            action: ActionModel to execute via ROS 2
            
        Returns:
            ActionResponseModel with execution results
        """
        try:
            self.logger.info(f"Executing action via ROS 2: {action.action_id} ({action.action_type})")
            
            # Check if we have an action client for this action type
            if action.action_type not in self.action_clients:
                error_msg = f"No action client registered for action type: {action.action_type}"
                self.logger.error(error_msg)
                raise VLAException(error_msg, VLAErrorType.ACTION_EXECUTION_ERROR)
            
            action_client = self.action_clients[action.action_type]
            
            # Create goal message from action parameters
            goal_msg = self._create_goal_message(action)
            
            # Send goal to action server
            self.logger.debug(f"Sending goal for action {action.action_id}")
            goal_future = action_client.send_goal_async(
                goal_msg,
                feedback_callback=self._handle_feedback
            )
            
            # Store the future for tracking
            goal_id = f"{action.action_id}_{int(time.time())}"
            self.goal_futures[goal_id] = goal_future
            
            # Wait for result with timeout
            timeout = action.timeout
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                # In a real implementation, we would wait for the goal future
                # For this simulation, we'll handle the goal directly
                await asyncio.sleep(0.1)  # Yield control briefly
                
                # Check if goal is done (simulated)
                # In a real system, we would check goal_future.done()
                execution_time = time.time() - start_time
                if execution_time > 0.5:  # Simulated execution time
                    # Create a simulated response
                    success = await self._simulate_action_execution(action)
                    break
            else:
                # Timeout occurred
                self.logger.warning(f"Action {action.action_id} timed out after {timeout}s")
                success = False
            
            # Clean up goal future
            if goal_id in self.goal_futures:
                del self.goal_futures[goal_id]
            
            # Create response based on result
            status = ExecutionStatus.COMPLETED if success else ExecutionStatus.FAILED
            completion_percentage = 1.0 if success else 0.0
            
            response = ActionResponseModel.create(
                sequence_id=action.sequence_id or "unknown_sequence",
                status=status,
                completion_percentage=completion_percentage,
                result_summary=f"Action {action.action_id} {'completed' if success else 'failed'} via ROS 2",
                action_logs=[],
                timestamp=time.time()
            )
            
            self.logger.info(f"Action {action.action_id} completed with status: {status.name}")
            return response
            
        except Exception as e:
            self.logger.error(f"Error executing action via ROS 2 {action.action_id}: {e}")
            raise VLAException(
                f"ROS 2 action execution error: {str(e)}", 
                VLAErrorType.ACTION_EXECUTION_ERROR,
                e
            )
    
    def _create_goal_message(self, action: ActionModel):
        """
        Create a ROS 2 goal message from an ActionModel.
        
        Args:
            action: ActionModel to convert to ROS 2 message
            
        Returns:
            ROS 2 goal message object
        """
        # This is a placeholder implementation - in a real system this would create
        # the appropriate ROS 2 message type based on the action type
        # For now, we'll return a dictionary that represents the goal
        goal_msg = {
            'action_id': action.action_id,
            'action_type': action.action_type,
            'parameters': action.parameters,
            'timeout': action.timeout
        }
        
        return goal_msg
    
    async def _simulate_action_execution(self, action: ActionModel) -> bool:
        """
        Simulate action execution for demonstration purposes.
        
        Args:
            action: ActionModel to simulate execution for
            
        Returns:
            True if successful, False otherwise
        """
        # Simulate different execution times based on action type
        action_type = action.action_type
        if action_type in ['move_to', 'navigate']:
            delay = 2.0  # Navigation takes longer
        elif action_type in ['grasp', 'pick_up', 'place']:
            delay = 3.0  # Manipulation takes longer
        elif action_type in ['detect_object']:
            delay = 1.0  # Perception is faster
        else:
            delay = 1.5  # Default
        
        # Add some randomness
        import random
        delay *= (0.8 + random.random() * 0.4)  # ±20% variation
        
        # Simulate execution delay
        await asyncio.sleep(min(delay, action.timeout))
        
        # Simulate success/failure based on various factors
        # In a real implementation, this would be based on actual robot feedback
        success_probability = 0.95  # 95% success rate in simulation
        success = random.random() < success_probability
        
        return success
    
    def _handle_feedback(self, feedback_msg):
        """
        Handle feedback from ongoing action execution.
        
        Args:
            feedback_msg: Feedback message from the action server
        """
        self.logger.debug(f"Received feedback: {feedback_msg}")
        # In a real implementation, this would update execution progress
        # and potentially adjust execution parameters
    
    @log_exception()
    async def execute_action_sequence_via_ros2(self, action_sequence: 'ActionSequenceModel') -> List[ActionResponseModel]:
        """
        Execute an entire action sequence through the ROS 2 interface.
        
        Args:
            action_sequence: ActionSequenceModel to execute via ROS 2
            
        Returns:
            List of ActionResponseModel for each executed action
        """
        try:
            self.logger.info(f"Executing action sequence via ROS 2: {action_sequence.sequence_id}")
            
            responses = []
            failed_actions = 0
            
            # Update sequence status
            action_sequence.execution_status = ExecutionStatus.EXECUTING
            
            # Execute each action in sequence
            for i, action in enumerate(action_sequence.actions):
                try:
                    # Execute the action via ROS 2
                    response = await self.execute_action_via_ros2(action)
                    responses.append(response)
                    
                    # Check if action succeeded
                    if response.status == ExecutionStatus.FAILED:
                        failed_actions += 1
                        self.logger.warning(f"Action {action.action_id} failed in sequence")
                        
                        # Determine if we should continue based on failure policy
                        continue_on_failure = self.config.get('continue_on_action_failure', False)
                        if not continue_on_failure and action.action_type not in ['monitoring', 'logging']:
                            self.logger.info("Stopping execution due to critical action failure")
                            break
                    else:
                        self.logger.debug(f"Action {i+1}/{len(action_sequence.actions)} completed successfully")
                
                except Exception as e:
                    self.logger.error(f"Critical error executing action {action.action_id}: {e}")
                    failed_actions += 1
                    
                    # Create a failure response
                    failed_response = ActionResponseModel.create(
                        sequence_id=action_sequence.sequence_id,
                        status=ExecutionStatus.FAILED,
                        completion_percentage=i / len(action_sequence.actions) if action_sequence.actions else 0.0,
                        result_summary=f"Action {action.action_id} failed with error: {str(e)}",
                        action_logs=[],
                        timestamp=0.0
                    )
                    responses.append(failed_response)
                    
                    # Decide whether to continue based on failure policy
                    continue_on_failure = self.config.get('continue_on_action_failure', False)
                    if not continue_on_failure:
                        self.logger.info("Stopping execution due to critical action failure")
                        break
            
            # Update sequence status based on results
            if failed_actions == 0:
                action_sequence.execution_status = ExecutionStatus.COMPLETED
            elif failed_actions == len(action_sequence.actions):
                action_sequence.execution_status = ExecutionStatus.FAILED
            else:
                action_sequence.execution_status = ExecutionStatus.PARTIAL_SUCCESS
            
            completion_percentage = (len(action_sequence.actions) - failed_actions) / len(action_sequence.actions) if action_sequence.actions else 0.0
            self.logger.info(f"Action sequence {action_sequence.sequence_id} completed with {completion_percentage:.2%} success rate")
            
            return responses
            
        except Exception as e:
            self.logger.error(f"Error executing action sequence via ROS 2 {action_sequence.sequence_id}: {e}")
            action_sequence.execution_status = ExecutionStatus.FAILED
            raise VLAException(
                f"ROS 2 action sequence execution error: {str(e)}", 
                VLAErrorType.ACTION_EXECUTION_ERROR,
                e
            )
    
    def get_registered_action_clients(self) -> List[str]:
        """
        Get a list of action types with registered clients.
        
        Returns:
            List of action type strings
        """
        return list(self.action_clients.keys())
    
    def get_registered_service_clients(self) -> List[str]:
        """
        Get a list of service types with registered clients.
        
        Returns:
            List of service type strings
        """
        return list(self.service_clients.keys())


class ROS2Integration:
    """
    Higher-level abstraction for ROS 2 integration that manages the ROS 2 interface
    and coordinates with other VLA components.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Initialize ROS 2 interface
        self.ros2_interface = None
        
        # Action sequencer and robot controller
        self.action_sequencer = get_action_sequencer()
        self.robot_controller = get_robot_controller()
        
        # Initialize ROS 2 communication when ready
        self._initialize_ros2_communication()
        
        self.logger.info("ROS2Integration initialized")
    
    def _initialize_ros2_communication(self):
        """
        Initialize ROS 2 communication by setting up action and service clients
        based on configuration and robot capabilities.
        """
        # Only initialize if ROS 2 is available
        if not self._is_ros2_available():
            self.logger.warning("ROS 2 not available, skipping initialization")
            return
        
        try:
            # Initialize rclpy if not already initialized
            if not rclpy.ok():
                rclpy.init()
            
            # Create the ROS 2 interface node
            self.ros2_interface = ROS2Interface()
            
            # Start ROS 2 spinning in a separate thread
            self.ros2_interface.start_ros2_spinning()
            
            # Register action clients based on robot capabilities
            self._register_action_clients()
            
            # Register service clients
            self._register_service_clients()
            
            self.logger.info("ROS 2 communication initialized successfully")
            
        except Exception as e:
            self.logger.error(f"Error initializing ROS 2 communication: {e}")
            # In a real system, you might want to handle this differently
            # For now, we'll just log the error and continue
    
    def _is_ros2_available(self) -> bool:
        """
        Check if ROS 2 is available in the current environment.
        
        Returns:
            True if ROS 2 is available, False otherwise
        """
        try:
            # Try to import rclpy
            import rclpy
            # Try to check if ROS 2 daemon is running
            # This is a basic check - in practice you'd want more robust verification
            return rclpy.utilities.ok()
        except ImportError:
            self.logger.warning("rclpy not available - ROS 2 integration disabled")
            return False
        except Exception:
            self.logger.warning("ROS 2 utilities not ok - ROS 2 integration disabled")
            return False
    
    def _register_action_clients(self):
        """Register action clients based on robot capabilities and configuration."""
        if not self.ros2_interface:
            return
        
        # Common action types with their ROS 2 interfaces
        # These would be determined based on the actual robot configuration
        action_mappings = {
            'move_to': ('nav2_msgs/action/NavigateToPose', '/navigate_to_pose'),
            'pick_up': ('manipulation_msgs/action/PickUp', '/pick_up_object'),
            'place': ('manipulation_msgs/action/Place', '/place_object'),
            'detect_object': ('vision_msgs/action/DetectObjects', '/detect_objects'),
            'grasp': ('manipulation_msgs/action/Grasp', '/grasp_object'),
            'release': ('manipulation_msgs/action/Release', '/release_object'),
            'navigate': ('nav2_msgs/action/NavigateThroughPoses', '/navigate_through_poses')
        }
        
        for action_type, (action_interface_name, topic_name) in action_mappings.items():
            try:
                # We would import the proper action interface here
                # For now, we'll use a generic approach
                # In a real implementation, the actual ROS 2 action interfaces would be imported
                self.ros2_interface.register_action_client(
                    action_type=action_type,
                    action_interface=None,  # Actual interface would be imported
                    action_topic=topic_name
                )
            except Exception as e:
                self.logger.warning(f"Could not register action client for {action_type}: {e}")
    
    def _register_service_clients(self):
        """Register service clients for non-action-based communication."""
        if not self.ros2_interface:
            return
        
        # Common services with their interfaces
        # These would be determined based on the actual robot configuration
        service_mappings = {
            'get_robot_state': ('std_srvs/srv/Trigger', '/get_robot_state'),
            'set_motor_power': ('std_srvs/srv/SetBool', '/set_motor_power'),
            'reset_odom': ('std_srvs/srv/Trigger', '/reset_odometry')
        }
        
        for service_type, (service_interface_name, topic_name) in service_mappings.items():
            try:
                # In a real implementation, the actual ROS 2 service interfaces would be imported
                self.ros2_interface.register_service_client(
                    service_type=service_type,
                    service_interface=None,  # Actual interface would be imported
                    service_topic=topic_name
                )
            except Exception as e:
                self.logger.warning(f"Could not register service client for {service_type}: {e}")
    
    @log_exception()
    async def execute_action(self, action: ActionModel) -> ActionResponseModel:
        """
        Execute a single action using the ROS 2 interface.
        
        Args:
            action: ActionModel to execute
            
        Returns:
            ActionResponseModel with execution results
        """
        if not self.ros2_interface:
            error_msg = "ROS 2 interface not available"
            self.logger.error(error_msg)
            raise VLAException(error_msg, VLAErrorType.COMMUNICATION_ERROR)
        
        return await self.ros2_interface.execute_action_via_ros2(action)
    
    @log_exception()
    async def execute_action_sequence(self, action_sequence: 'ActionSequenceModel') -> List[ActionResponseModel]:
        """
        Execute an action sequence using the ROS 2 interface.
        
        Args:
            action_sequence: ActionSequenceModel to execute
            
        Returns:
            List of ActionResponseModel for each executed action
        """
        if not self.ros2_interface:
            error_msg = "ROS 2 interface not available"
            self.logger.error(error_msg)
            raise VLAException(error_msg, VLAErrorType.COMMUNICATION_ERROR)
        
        return await self.ros2_interface.execute_action_sequence_via_ros2(action_sequence)
    
    def shutdown(self):
        """Shut down the ROS 2 integration."""
        if self.ros2_interface:
            self.ros2_interface.stop_ros2_spinning()
            self.ros2_interface.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()
        
        self.logger.info("ROS 2 integration shut down")


# Global ROS 2 integration instance
_ros2_integration = None


def get_ros2_integration() -> ROS2Integration:
    """
    Get the global ROS 2 integration instance.
    
    Returns:
        ROS2Integration instance
    """
    global _ros2_integration
    if _ros2_integration is None:
        _ros2_integration = ROS2Integration()
    return _ros2_integration


async def execute_action_via_ros2(action: ActionModel) -> ActionResponseModel:
    """
    Convenience function to execute an action via ROS 2.
    
    Args:
        action: ActionModel to execute
        
    Returns:
        ActionResponseModel with execution results
    """
    integration = get_ros2_integration()
    return await integration.execute_action(action)


async def execute_action_sequence_via_ros2(action_sequence: 'ActionSequenceModel') -> List[ActionResponseModel]:
    """
    Convenience function to execute an action sequence via ROS 2.
    
    Args:
        action_sequence: ActionSequenceModel to execute
        
    Returns:
        List of ActionResponseModel for each executed action
    """
    integration = get_ros2_integration()
    return await integration.execute_action_sequence(action_sequence)


def get_available_ros2_actions() -> List[str]:
    """
    Get a list of action types available via ROS 2.
    
    Returns:
        List of action type strings
    """
    integration = get_ros2_integration()
    if integration.ros2_interface:
        return integration.ros2_interface.get_registered_action_clients()
    return []