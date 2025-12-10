"""
Robot Controller for the Vision-Language-Action (VLA) module.

This module provides the interface between the action execution system and the
actual robot hardware/simulation, managing robot state and executing action sequences.
"""

import asyncio
import logging
from typing import Dict, List, Any, Optional
from dataclasses import dataclass
import time

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import Action, ActionSequence, ActionResponse, ExecutionStatus
from ..core.data_models import ActionModel, ActionSequenceModel, ActionResponseModel, RobotStateModel
from .ros2_action_client import get_action_client
from .robot_capability_validator import get_capability_validator
from .safety_monitor import get_safety_monitor
from .action_sequencer import get_action_sequencer


@dataclass
class ExecutionMetrics:
    """Metrics for action execution performance."""
    execution_time: float
    success_rate: float
    average_action_time: float
    total_actions: int
    successful_actions: int
    failed_actions: int
    resource_usage: Dict[str, float]  # CPU, memory, etc.


class RobotController:
    """
    Controls the robot by executing action sequences and managing robot state.
    This is the primary interface between the VLA action execution system and the physical/simulated robot.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Initialize components
        self.action_client = get_action_client()
        self.capability_validator = get_capability_validator()
        self.safety_monitor = get_safety_monitor()
        self.action_sequencer = get_action_sequencer()
        
        # Robot state tracking
        self.current_robot_state: Optional[RobotStateModel] = None
        self.robot_is_connected = False
        
        # Active execution tracking
        self.active_execution_sequences = []
        self.action_history = []
        
        # Performance metrics
        self.execution_performance = ExecutionMetrics(
            execution_time=0.0,
            success_rate=0.0,
            average_action_time=0.0,
            total_actions=0,
            successful_actions=0,
            failed_actions=0,
            resource_usage={}
        )
        
        self.logger.info("RobotController initialized")
    
    @log_exception()
    async def connect_to_robot(self) -> bool:
        """
        Establish connection to the robot.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.logger.info("Connecting to robot...")
            
            # In a real implementation, this would connect to the robot
            # For now, we'll simulate the connection 
            await asyncio.sleep(0.5)  # Simulate connection time
            
            # Test if we can communicate with action servers
            if await self._test_robot_connectivity():
                self.robot_is_connected = True
                self.current_robot_state = await self._get_initial_robot_state()
                self.logger.info("Successfully connected to robot")
                return True
            else:
                self.logger.error("Failed to establish robot connectivity")
                return False
                
        except Exception as e:
            self.logger.error(f"Error connecting to robot: {e}")
            self.robot_is_connected = False
            return False
    
    async def _test_robot_connectivity(self) -> bool:
        """
        Test if robot is reachable and responding to commands.
        
        Returns:
            True if robot is responsive, False otherwise
        """
        try:
            # In a real system, this would send a simple test command to the robot
            # For simulation purposes, we'll just return True
            return True
        except Exception as e:
            self.logger.warning(f"Robot connectivity test failed: {e}")
            return False
    
    async def _get_initial_robot_state(self) -> RobotStateModel:
        """
        Get the initial robot state after connection.
        
        Returns:
            RobotStateModel with initial robot state
        """
        try:
            # In a real system, this would query the robot for its current state
            # For now, we'll return a default initial state
            
            # Create a default robot state
            initial_state = RobotStateModel.create(
                robot_id="vla_default_robot",
                position={"x": 0.0, "y": 0.0, "z": 0.0},
                orientation={"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                battery_level=1.0,
                is_charging=False,
                joints=[],
                gripper_state={"position": 0.0, "max_effort": 100.0, "is_grasping": False},
                capabilities=self.config.robot_capabilities,
                mode="idle",
                timestamp=time.time()
            )
            
            return initial_state
            
        except Exception as e:
            self.logger.error(f"Error getting initial robot state: {e}")
            raise VLAException(
                f"Initial robot state error: {str(e)}", 
                VLAErrorType.ROBOT_COMMUNICATION_ERROR,
                e
            )
    
    @log_exception()
    async def execute_action_sequence(self, action_sequence: ActionSequenceModel) -> List[ActionResponseModel]:
        """
        Execute an action sequence on the robot.
        
        Args:
            action_sequence: ActionSequenceModel to execute
            
        Returns:
            List of ActionResponseModel for each action in the sequence
        """
        if not self.robot_is_connected:
            error_msg = "Cannot execute actions: robot is not connected"
            self.logger.error(error_msg)
            raise VLAException(error_msg, VLAErrorType.ROBOT_COMMUNICATION_ERROR)
        
        try:
            self.logger.info(f"Executing action sequence {action_sequence.sequence_id} with {len(action_sequence.actions)} actions")
            
            # Validate the entire sequence before execution
            validation_errors = await self._validate_sequence_for_execution(action_sequence)
            if validation_errors:
                error_msg = f"Action sequence validation failed: {validation_errors}"
                self.logger.error(error_msg)
                raise VLAException(error_msg, VLAErrorType.VALIDATION_ERROR)
            
            # Add to active executions
            self.active_execution_sequences.append(action_sequence.sequence_id)
            
            # Execute each action in the sequence
            responses = []
            action_count = 0
            
            for i, action in enumerate(action_sequence.actions):
                try:
                    action_count += 1
                    
                    self.logger.info(f"Executing action {i+1}/{len(action_sequence.actions)}: {action.action_id}")
                    
                    # Execute the action
                    response = await self.execute_single_action(action)
                    responses.append(response)
                    
                    # Update robot state if the action modifies it
                    await self._update_robot_state_based_on_action(action, response)
                    
                    # Check if action succeeded
                    if response.status != ExecutionStatus.COMPLETED:
                        self.logger.warning(f"Action {action.action_id} did not complete successfully: {response.status.name}")
                        
                        # Check if we should continue based on configuration
                        continue_on_failure = self.config.continue_on_action_failure
                        if not continue_on_failure:
                            self.logger.info("Stopping execution due to action failure (configured to stop on failure)")
                            break
                    else:
                        self.logger.debug(f"Action {i+1} completed successfully")
                
                except Exception as e:
                    self.logger.error(f"Error executing action {action.action_id}: {e}")
                    
                    # Create error response for this action
                    error_response = ActionResponseModel.create(
                        sequence_id=action_sequence.sequence_id,
                        status=ExecutionStatus.FAILED,
                        completion_percentage=i / len(action_sequence.actions) if len(action_sequence.actions) > 0 else 0.0,
                        result_summary=f"Action execution failed: {str(e)}",
                        action_logs=[],
                        timestamp=time.time()
                    )
                    responses.append(error_response)
                    
                    # Decide whether to continue based on configuration
                    continue_on_failure = self.config.continue_on_action_failure
                    if not continue_on_failure:
                        self.logger.info("Stopping execution due to action error (configured to stop on failure)")
                        break
            
            # Calculate metrics
            successful_count = sum(1 for r in responses if r.status == ExecutionStatus.COMPLETED)
            success_rate = successful_count / len(responses) if responses else 0.0
            
            # Update performance metrics
            self.execution_performance.successful_actions += successful_count
            self.execution_performance.failed_actions += len(responses) - successful_count
            self.execution_performance.total_actions += len(responses)
            self.execution_performance.success_rate = (
                self.execution_performance.successful_actions / 
                (self.execution_performance.total_actions or 1)
            )
            
            # Log execution summary
            self.logger.info(f"Action sequence {action_sequence.sequence_id} completed: "
                           f"{successful_count}/{len(responses)} actions succeeded, "
                           f"success rate: {success_rate:.2%}")
            
            # Remove from active executions
            if action_sequence.sequence_id in self.active_execution_sequences:
                self.active_execution_sequences.remove(action_sequence.sequence_id)
            
            return responses
            
        except Exception as e:
            self.logger.error(f"Error executing action sequence {action_sequence.sequence_id}: {e}")
            
            # Remove from active executions in case of error
            if action_sequence.sequence_id in self.active_execution_sequences:
                self.active_execution_sequences.remove(action_sequence.sequence_id)
            
            raise VLAException(
                f"Action sequence execution error: {str(e)}", 
                VLAErrorType.ACTION_EXECUTION_ERROR,
                e
            )
    
    @log_exception()
    async def execute_single_action(self, action: ActionModel) -> ActionResponseModel:
        """
        Execute a single action on the robot.
        
        Args:
            action: ActionModel to execute
            
        Returns:
            ActionResponseModel with execution results
        """
        if not self.robot_is_connected:
            error_msg = f"Cannot execute action {action.action_id}: robot is not connected"
            self.logger.error(error_msg)
            raise VLAException(error_msg, VLAErrorType.ROBOT_COMMUNICATION_ERROR)
        
        try:
            start_time = time.time()
            self.logger.info(f"Executing single action: {action.action_id} ({action.action_type})")
            
            # Validate action against robot capabilities
            capability_validation = await self.capability_validator.validate_action(
                action=action,
                robot_state=self.current_robot_state
            )
            
            if not capability_validation.is_valid:
                error_msg = f"Action {action.action_id} failed capability validation: {capability_validation.error_messages}"
                self.logger.error(error_msg)
                return ActionResponseModel.create(
                    sequence_id="unknown_sequence",  # This would be properly set in a real context
                    status=ExecutionStatus.FAILED,
                    completion_percentage=0.0,
                    result_summary=error_msg,
                    action_logs=[],
                    timestamp=time.time()
                )
            
            # Check safety before execution
            safety_violations = await self.safety_monitor.check_action_safety(action, self.current_robot_state)
            if safety_violations:
                error_msg = f"Safety violations detected: {[v.description for v in safety_violations]}"
                self.logger.error(error_msg)
                
                # Trigger safety handling
                for violation in safety_violations:
                    await self.safety_monitor.handle_safety_violation(violation)
                
                return ActionResponseModel.create(
                    sequence_id="unknown_sequence",  # Would be properly set in execution context
                    status=ExecutionStatus.FAILED,
                    completion_percentage=0.0,
                    result_summary=error_msg,
                    action_logs=[],
                    timestamp=time.time()
                )
            
            # Execute the action via the ROS 2 action client
            response = await self.action_client.execute_action_with_retry(action)
            
            # Calculate execution time
            execution_time = time.time() - start_time
            
            self.logger.info(f"Action {action.action_id} completed with status: {response.status.name} in {execution_time:.2f}s")
            
            # Update performance metrics
            self.execution_performance.execution_time += execution_time
            if response.status == ExecutionStatus.COMPLETED:
                self.execution_performance.successful_actions += 1
            else:
                self.execution_performance.failed_actions += 1
            
            self.execution_performance.total_actions += 1
            self.execution_performance.average_action_time = (
                self.execution_performance.execution_time / self.execution_performance.total_actions
            )
            
            # Add to action history
            self.action_history.append({
                'action_id': action.action_id,
                'execution_time': execution_time,
                'status': response.status.name,
                'timestamp': time.time()
            })
            
            # Limit history size
            if len(self.action_history) > self.config.get('action_history_size', 1000):
                self.action_history = self.action_history[-self.config.get('action_history_size', 1000):]
            
            return response
            
        except Exception as e:
            self.logger.error(f"Error executing action {action.action_id}: {e}")
            
            # Log the failure
            self.action_history.append({
                'action_id': action.action_id,
                'execution_time': time.time() - start_time,
                'status': 'FAILED',
                'error': str(e),
                'timestamp': time.time()
            })
            
            # Create failure response
            response = ActionResponseModel.create(
                sequence_id="unknown_sequence",  # Would be properly set in execution context
                status=ExecutionStatus.FAILED,
                completion_percentage=0.0,
                result_summary=f"Action execution error: {str(e)}",
                action_logs=[],
                timestamp=time.time()
            )
            
            # Increment failure metrics
            self.execution_performance.failed_actions += 1
            self.execution_performance.total_actions += 1
            
            return response
    
    @log_exception()
    async def _validate_sequence_for_execution(self, action_sequence: ActionSequenceModel) -> List[str]:
        """
        Validate that an action sequence is safe and feasible for execution.
        
        Args:
            action_sequence: ActionSequenceModel to validate
            
        Returns:
            List of validation errors or empty list if valid
        """
        errors = []
        
        # Validate sequence structure
        if not action_sequence.actions:
            errors.append("Action sequence has no actions")
        
        # Validate each action in the sequence
        for i, action in enumerate(action_sequence.actions):
            action_errors = await self._validate_action_for_execution(action, i)
            errors.extend([f"Action {i+1} ({action.action_id}): {error}" for error in action_errors])
        
        # Validate robot state availability
        if not self.current_robot_state:
            errors.append("No robot state available for execution validation")
        
        # Validate overall sequence feasibility
        feasibility_errors = await self._check_sequence_feasibility(action_sequence)
        errors.extend(feasibility_errors)
        
        return errors
    
    async def _validate_action_for_execution(self, action: ActionModel, index: int) -> List[str]:
        """
        Validate a single action within the execution context.
        
        Args:
            action: ActionModel to validate
            index: Index of the action in the sequence
            
        Returns:
            List of validation errors or empty list if valid
        """
        errors = []
        
        # Check that action has required fields
        if not action.action_id:
            errors.append("Missing action_id")
        
        if not action.action_type:
            errors.append("Missing action_type")
        
        # Validate action type against supported types
        if action.action_type not in self.config.supported_action_types:
            errors.append(f"Action type '{action.action_type}' not supported")
        
        # Validate timeout is reasonable
        if action.timeout <= 0 or action.timeout > 300:  # Limit to 5 minutes max
            errors.append(f"Action timeout {action.timeout}s is invalid (0-300s)")
        
        # Validate parameters for specific action types
        param_errors = self._validate_action_parameters(action)
        errors.extend(param_errors)
        
        return errors
    
    def _validate_action_parameters(self, action: ActionModel) -> List[str]:
        """
        Validate action-specific parameters.
        
        Args:
            action: ActionModel to validate parameters for
            
        Returns:
            List of parameter validation errors or empty list if valid
        """
        errors = []
        
        action_type = action.action_type
        params = action.parameters
        
        # For navigation actions, verify location parameters
        if action_type in ['move_to', 'navigate', 'go_to']:
            required_location_keys = ['x', 'y', 'z']
            if 'target_location' in params:
                location = params['target_location']
                for key in required_location_keys:
                    if key not in location:
                        errors.append(f"Missing required location key: {key}")
                    elif not isinstance(location[key], (int, float)):
                        errors.append(f"Location key {key} must be numeric")
        
        # For manipulation actions, verify object parameters
        elif action_type in ['pick_up', 'place', 'grasp']:
            if 'target_object' not in params and 'target_location' not in params:
                errors.append("Manipulation action requires either target_object or target_location")
        
        return errors
    
    async def _check_sequence_feasibility(self, action_sequence: ActionSequenceModel) -> List[str]:
        """
        Check if the action sequence is feasible in the current robot state.
        
        Args:
            action_sequence: ActionSequenceModel to check feasibility for
            
        Returns:
            List of feasibility errors or empty list if feasible
        """
        errors = []
        
        # Check for resource conflicts within the sequence
        resource_usage = {}
        for i, action in enumerate(action_sequence.actions):
            resource_type = self._get_action_resource_type(action.action_type)
            
            # Check for incompatible resource usage
            if resource_type in resource_usage:
                # Same resource needed at different times - check if compatible
                prev_action_idx = resource_usage[resource_type]
                prev_action = action_sequence.actions[prev_action_idx]
                
                # For some resources, concurrent use might be incompatible
                if resource_type in ['navigation', 'manipulation'] and action.action_type != prev_action.action_type:
                    # Different action types competing for same resource
                    errors.append(f"Resource conflict: Action {i} and action {prev_action_idx} compete for {resource_type} resource")
            
            resource_usage[resource_type] = i
        
        # Check if overall sequence is too long
        if len(action_sequence.actions) > self.config.max_actions_per_sequence:
            errors.append(
                f"Action sequence has {len(action_sequence.actions)} actions, "
                f"exceeding maximum of {self.config.max_actions_per_sequence}"
            )
        
        return errors
    
    def _get_action_resource_type(self, action_type: str) -> str:
        """
        Determine what type of resource an action requires.
        
        Args:
            action_type: Type of action
            
        Returns:
            Resource type string
        """
        # Map action types to resource categories
        action_to_resource = {
            'move_to': 'navigation',
            'navigate': 'navigation',
            'go_to': 'navigation',
            'pick_up': 'manipulation',
            'place': 'manipulation',
            'grasp': 'manipulation',
            'release': 'manipulation',
            'detect_object': 'perception',
            'find_object': 'perception',
            'inspect': 'perception'
        }
        
        return action_to_resource.get(action_type, 'general')
    
    async def _update_robot_state_based_on_action(self, action: ActionModel, response: ActionResponseModel):
        """
        Update the robot state based on the action that was executed.
        
        Args:
            action: ActionModel that was executed
            response: ActionResponseModel with execution results
        """
        if response.status != ExecutionStatus.COMPLETED:
            # If action didn't complete successfully, we might not want to update state
            # In a real implementation, we might have partial updates based on what succeeded
            return
        
        # Update robot state based on action type
        if action.action_type in ['move_to', 'navigate', 'go_to']:
            # Update position if navigation was successful
            target_location = action.parameters.get('target_location')
            if target_location and self.current_robot_state:
                self.current_robot_state.position = {
                    'x': target_location.get('x', self.current_robot_state.position['x']),
                    'y': target_location.get('y', self.current_robot_state.position['y']),
                    'z': target_location.get('z', self.current_robot_state.position['z'])
                }
        
        elif action.action_type in ['pick_up', 'grasp']:
            # Update gripper state to indicate object is held
            if self.current_robot_state and self.current_robot_state.gripper_state:
                self.current_robot_state.gripper_state['is_grasping'] = True
                self.current_robot_state.gripper_state['held_object'] = action.parameters.get('target_object', 'unknown_object')
        
        elif action.action_type in ['place', 'release']:
            # Update gripper state to indicate object is no longer held
            if self.current_robot_state and self.current_robot_state.gripper_state:
                self.current_robot_state.gripper_state['is_grasping'] = False
                self.current_robot_state.gripper_state['held_object'] = None
        
        # Update other relevant state fields as needed based on action type
        # This is a simplified example - in a real system, each action type would
        # update relevant parts of the robot state
    
    @log_exception()
    async def get_robot_state(self) -> Optional[RobotStateModel]:
        """
        Get the current robot state.
        
        Returns:
            RobotStateModel or None if unavailable
        """
        if not self.robot_is_connected:
            self.logger.warning("Cannot get robot state: robot is not connected")
            return None
        
        try:
            # In a real implementation, this would query the robot for its current state
            # For now, we'll return the cached state
            return self.current_robot_state
            
        except Exception as e:
            self.logger.error(f"Error getting robot state: {e}")
            return None
    
    @log_exception()
    async def emergency_stop(self) -> bool:
        """
        Issue emergency stop to the robot.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            self.logger.warning("Emergency stop issued!")
            
            # In a real implementation, this would send an emergency stop command to the robot
            # For now, we'll just update the local state
            
            if self.current_robot_state:
                self.current_robot_state.mode = "emergency_stop"
                self.current_robot_state.is_emergency_stopped = True
            
            # Cancel all active action sequences
            for seq_id in self.active_execution_sequences:
                self.logger.info(f"Cancelling execution sequence: {seq_id}")
            
            self.active_execution_sequences.clear()
            self.logger.info("Emergency stop issued and all executions cancelled")
            return True
            
        except Exception as e:
            self.logger.error(f"Error issuing emergency stop: {e}")
            return False
    
    def get_execution_performance(self) -> ExecutionMetrics:
        """
        Get performance metrics for action executions.
        
        Returns:
            ExecutionMetrics with execution statistics
        """
        return self.execution_performance
    
    def get_active_execution_sequences(self) -> List[str]:
        """
        Get a list of currently active execution sequences.
        
        Returns:
            List of sequence IDs
        """
        return self.active_execution_sequences.copy()
    
    def get_action_history(self) -> List[Dict[str, Any]]:
        """
        Get the history of recently executed actions.
        
        Returns:
            List of action execution records
        """
        return self.action_history.copy()


# Global robot controller instance
_robot_controller = None


def get_robot_controller() -> RobotController:
    """Get the global robot controller instance."""
    global _robot_controller
    if _robot_controller is None:
        _robot_controller = RobotController()
    return _robot_controller


async def execute_action_sequence(action_sequence: ActionSequenceModel) -> List[ActionResponseModel]:
    """Convenience function to execute an action sequence."""
    controller = get_robot_controller()
    return await controller.execute_action_sequence(action_sequence)


async def execute_single_action(action: ActionModel) -> ActionResponseModel:
    """Convenience function to execute a single action."""
    controller = get_robot_controller()
    return await controller.execute_single_action(action)


async def get_robot_state() -> Optional[RobotStateModel]:
    """Convenience function to get robot state."""
    controller = get_robot_controller()
    return await controller.get_robot_state()