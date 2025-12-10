"""
ROS 2 Action Client for the Vision-Language-Action (VLA) module.

This module implements the ROS 2 action client interface used to communicate
with robot action servers for executing planned tasks.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile

import logging
import asyncio
import time
from typing import Dict, Any, Optional, List

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import Action, ActionSequence, ActionResponse, ExecutionStatus
from ..core.data_models import ActionModel, ActionSequenceModel, ActionResponseModel, ActionLogModel
from .action_sequencer import get_action_sequencer


class ROS2ActionClient(Node):
    """
    ROS 2 action client that communicates with robot action servers.
    """
    
    def __init__(self, node_name: str = "vla_action_client"):
        super().__init__(node_name)
        
        self.config = get_config()
        self.logger = setup_logger(node_name)
        
        # Action client storage
        self.action_clients = {}
        
        # Execution tracking
        self.active_goals = {}
        self.execution_history = []
        
        self.logger.info("ROS2ActionClient initialized")
    
    def initialize_action_clients(self, action_interfaces: Dict[str, Any]):
        """
        Initialize action clients based on the action interfaces.
        
        Args:
            action_interfaces: Dictionary mapping action types to ROS action interfaces
        """
        for action_type, action_interface in action_interfaces.items():
            try:
                # Create action client for the specified type
                client = ActionClient(
                    self,
                    action_interface,
                    f'/robot/{action_type}_action'  # Standard action naming
                )
                
                # Wait for action server to be available
                self.logger.info(f"Waiting for action server: /robot/{action_type}_action")
                client.wait_for_server(timeout_sec=10.0)  # Wait up to 10 seconds
                
                self.action_clients[action_type] = client
                self.logger.info(f"Action client initialized for: {action_type}")
                
            except Exception as e:
                self.logger.error(f"Failed to initialize action client for {action_type}: {e}")
    
    @log_exception()
    async def execute_action(self, action: ActionModel) -> ActionResponseModel:
        """
        Execute a single action using the appropriate ROS 2 action client.
        
        Args:
            action: ActionModel to execute
            
        Returns:
            ActionResponseModel with execution results
        """
        try:
            start_time = time.time()
            self.logger.info(f"Executing action: {action.action_id} of type {action.action_type}")
            
            # Find the appropriate action client for this action type
            if action.action_type not in self.action_clients:
                error_msg = f"No action client found for type: {action.action_type}"
                self.logger.error(error_msg)
                raise VLAException(error_msg, VLAErrorType.ACTION_EXECUTION_ERROR)
            
            action_client = self.action_clients[action.action_type]
            
            # Create action goal based on action type
            goal_msg = self._create_goal_message(action)
            
            # Send goal to action server
            goal_future = action_client.send_goal_async(goal_msg)
            
            # Wait for result with timeout
            timeout = action.timeout
            start_wait_time = time.time()
            
            while time.time() - start_wait_time < timeout:
                # In a real implementation, we would wait for the goal result
                # For now, simulate the execution with a simple approach
                await asyncio.sleep(0.1)  # Small delay to yield control
                
                # In a real implementation, we would check on the goal status
                # and return when the action completes or fails
                
                # For this implementation, we'll use a simulated execution
                success = await self._simulate_action_execution(action)
                break  # Exit loop after simulation
            
            end_time = time.time()
            execution_time = end_time - start_time
            
            # Create action log entry
            action_status = ExecutionStatus.COMPLETED if success else ExecutionStatus.FAILED
            action_log = ActionLogModel.create(
                action.action_id,
                action_status,
                start_time,
                end_time
            )
            
            # Create response
            response = ActionResponseModel.create(
                sequence_id="simulated_sequence",  # In real implementation, this would come from context
                status=action_status,
                completion_percentage=1.0 if success else 0.0,
                result_summary=f"Action {action.action_id} {'completed' if success else 'failed'} in {execution_time:.2f}s",
                action_logs=[action_log]
            )
            
            self.logger.info(f"Action {action.action_id} execution completed with status: {action_status.name}")
            return response
            
        except Exception as e:
            self.logger.error(f"Error executing action {action.action_id}: {e}")
            raise VLAException(
                f"Action execution error: {str(e)}", 
                VLAErrorType.ACTION_EXECUTION_ERROR,
                e
            )
    
    @log_exception()
    async def execute_action_sequence(self, action_sequence: ActionSequenceModel) -> List[ActionResponseModel]:
        """
        Execute an entire action sequence with proper ordering and error handling.
        
        Args:
            action_sequence: ActionSequenceModel to execute
            
        Returns:
            List of ActionResponseModel for each executed action
        """
        try:
            self.logger.info(f"Executing action sequence: {action_sequence.sequence_id} with {len(action_sequence.actions)} actions")
            
            responses = []
            failed_actions = 0
            
            # Update sequence status
            action_sequence.execution_status = ExecutionStatus.EXECUTING
            
            # Execute each action in sequence
            for i, action in enumerate(action_sequence.actions):
                try:
                    # Execute the action
                    response = await self.execute_action(action)
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
                        completion_percentage=i / len(action_sequence.actions) if len(action_sequence.actions) > 0 else 0.0,
                        result_summary=f"Action {action.action_id} failed with error: {str(e)}",
                        action_logs=[]
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
            self.logger.error(f"Error executing action sequence {action_sequence.sequence_id}: {e}")
            action_sequence.execution_status = ExecutionStatus.FAILED
            raise VLAException(
                f"Action sequence execution error: {str(e)}", 
                VLAErrorType.ACTION_EXECUTION_ERROR,
                e
            )
    
    def _create_goal_message(self, action: ActionModel):
        """
        Create a ROS 2 goal message for the specified action.
        
        Args:
            action: ActionModel to create goal for
            
        Returns:
            ROS 2 goal message
        """
        # This is a placeholder implementation
        # In a real implementation, this would create the appropriate goal message
        # based on the action type and parameters
        
        # For now, return a simple dictionary that would represent the goal
        # In a real ROS 2 implementation, this would be a specific message type
        goal_msg = {
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
            delay = 1.0  # Navigation takes longer
        elif action_type in ['grasp', 'pick_up', 'place']:
            delay = 2.0  # Manipulation takes longer
        elif action_type in ['detect_object']:
            delay = 0.5  # Perception is faster
        else:
            delay = 1.0  # Default
        
        # Add some randomness
        import random
        delay *= (0.9 + random.random() * 0.2)  # ±10% variation
        
        # Simulate execution delay
        await asyncio.sleep(min(delay, action.timeout))
        
        # Simulate success/failure based on various factors
        # In a real implementation, this would be based on actual robot feedback
        success_probability = 0.9  # 90% success rate in simulation
        success = random.random() < success_probability
        
        return success
    
    @log_exception()
    async def execute_action_with_retry(self, action: ActionModel) -> ActionResponseModel:
        """
        Execute an action with retry logic based on action's retry_count parameter.
        
        Args:
            action: ActionModel to execute with retry logic
            
        Returns:
            ActionResponseModel with final execution results
        """
        max_retries = action.retry_count
        attempt = 0
        
        while attempt <= max_retries:
            try:
                self.logger.info(f"Attempting action {action.action_id} (attempt {attempt + 1} of {max_retries + 1})")
                
                response = await self.execute_action(action)
                
                # If successful, return the response
                if response.status != ExecutionStatus.FAILED:
                    return response
                
                # If failed but not at max retries, wait and try again
                if attempt < max_retries:
                    retry_delay = 0.5 * (attempt + 1)  # Increase delay with retries
                    self.logger.info(f"Action failed, retrying in {retry_delay}s...")
                    await asyncio.sleep(retry_delay)
                
                attempt += 1
                
            except Exception as e:
                self.logger.error(f"Error on attempt {attempt + 1} for action {action.action_id}: {e}")
                if attempt >= max_retries:
                    raise
                attempt += 1
                retry_delay = 0.5 * (attempt + 1)
                await asyncio.sleep(retry_delay)
        
        # If we get here, all attempts failed
        raise VLAException(
            f"Action {action.action_id} failed after {max_retries + 1} attempts", 
            VLAErrorType.ACTION_EXECUTION_ERROR
        )
    
    def get_active_goals(self) -> Dict[str, Any]:
        """
        Get the currently active goals.
        
        Returns:
            Dictionary of active goals
        """
        return self.active_goals.copy()


# Global action client instance
_action_client = None


def get_action_client(node_name: str = "vla_action_client") -> ROS2ActionClient:
    """
    Get the global ROS 2 action client instance.
    
    Args:
        node_name: Name for the node (default: vla_action_client)
        
    Returns:
        ROS2ActionClient instance
    """
    global _action_client
    if _action_client is None:
        # We'll initialize with a minimal set of action interfaces
        # In a real system, these would be determined dynamically
        action_interfaces = {
            'move_to': None,  # Placeholder - would be a real action interface in ROS 2
            'pick_up': None,
            'place': None,
            'detect_object': None,
            'grasp': None,
            'release': None,
            'navigate': None,
        }
        
        # Since we're working in a simulator for now, we'll just create the client without initializing real action clients
        rclpy.init(args=None)  # Initialize ROS 2 if not already done
        _action_client = ROS2ActionClient(node_name)
        
        # In a real implementation, we would initialize the clients here
        # _action_client.initialize_action_clients(action_interfaces)
    
    return _action_client