"""
Action Sequencer for the Vision-Language-Action (VLA) module.

This module converts cognitive plans into executable action sequences
with proper dependencies, priorities, and error handling.
"""

import asyncio
import logging
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass
import time

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger, retry_on_failure
from ..core.message_types import (
    Action, ActionSequence, Task, 
    ExecutionStatus, CommandType
)
from ..core.data_models import ActionModel, ActionSequenceModel, TaskModel
from .cognitive_planner import CognitivePlanner, get_cognitive_planner


@dataclass
class ActionDependency:
    """Represents a dependency between actions."""
    dependent_action_id: str
    prerequisite_action_id: str
    dependency_type: str  # 'before', 'after', 'concurrent_with', 'mutex_with'


@dataclass
class ActionPriority:
    """Represents priority settings for an action."""
    action_id: str
    priority_level: int  # Higher number means higher priority
    execution_group: str  # Actions in the same group can execute concurrently


class ActionSequencer:
    """
    Converts cognitive plans into executable action sequences.
    Determines execution order, manages dependencies, and handles error recovery.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Initialize cognitive planner
        self.cognitive_planner = get_cognitive_planner()
        
        # Action type to ROS action mapping
        self.action_mapping = {
            'move_to': 'nav2_msgs/action/NavigateToPose',
            'pick_up': 'manipulation_msgs/action/PickUp',
            'place': 'manipulation_msgs/action/Place',
            'detect_object': 'vision_msgs/action/DetectObjects',
            'grasp': 'manipulation_msgs/action/Grasp',
            'release': 'manipulation_msgs/action/Release',
            'rotate': 'geometry_msgs/action/Rotate',
            'stop': 'std_msgs/action/Stop',
        }
        
        # Default timeouts for different action types
        self.default_timeouts = {
            'move_to': 30.0,
            'pick_up': 45.0,
            'place': 30.0,
            'detect_object': 10.0,
            'grasp': 20.0,
            'release': 15.0,
            'rotate': 10.0,
            'stop': 5.0,
        }
        
        self.logger.info("ActionSequencer initialized")
    
    @log_exception()
    async def sequence_plan(self, cognitive_plan: 'CognitivePlanModel') -> ActionSequenceModel:
        """
        Convert a cognitive plan into an executable action sequence.
        
        Args:
            cognitive_plan: CognitivePlanModel to convert
            
        Returns:
            ActionSequenceModel ready for execution
        """
        try:
            self.logger.info(f"Sequencing plan with {len(cognitive_plan.task_decomposition)} tasks")
            
            # Validate the plan first
            validation_errors = await self.cognitive_planner.validate_plan(cognitive_plan)
            if validation_errors:
                raise VLAException(
                    f"Cannot sequence plan with validation errors: {validation_errors}", 
                    VLAErrorType.VALIDATION_ERROR
                )
            
            # Convert tasks to actions
            actions = self._convert_tasks_to_actions(cognitive_plan.task_decomposition)
            
            # Determine execution order with dependencies
            ordered_actions = self._determine_execution_order(actions, cognitive_plan)
            
            # Create action sequence
            action_sequence = ActionSequenceModel.create(
                plan_id=cognitive_plan.plan_id,
                actions=ordered_actions
            )
            
            # Calculate estimated duration
            action_sequence.estimated_duration = self._calculate_estimated_duration(ordered_actions)
            
            # Set initial execution status
            action_sequence.execution_status = ExecutionStatus.PENDING
            
            self.logger.info(f"Created action sequence with {len(ordered_actions)} actions")
            return action_sequence
            
        except Exception as e:
            self.logger.error(f"Error sequencing plan: {e}")
            raise VLAException(
                f"Error in action sequencing: {str(e)}", 
                VLAErrorType.PLANNING_ERROR,
                e
            )
    
    def _convert_tasks_to_actions(self, tasks: List[TaskModel]) -> List[ActionModel]:
        """
        Convert TaskModels to ActionModels.
        
        Args:
            tasks: List of TaskModels to convert
            
        Returns:
            List of ActionModels
        """
        actions = []
        
        for task in tasks:
            try:
                # Map task type to action type
                action_type = self._map_task_to_action(task.task_type)
                
                # Set timeout based on action type
                timeout = self.default_timeouts.get(task.task_type, 30.0)
                
                # Create action parameters based on task parameters
                action_parameters = self._create_action_parameters(task)
                
                # Create ActionModel
                action = ActionModel.create(
                    action_type=action_type,
                    parameters=action_parameters,
                    timeout=timeout,
                    retry_count=task.parameters.get('retry_count', 1)
                )
                
                # Update the action ID to match the task ID
                action.action_id = task.task_id
                
                actions.append(action)
                
            except Exception as e:
                self.logger.error(f"Error converting task to action: {e}")
                # Skip invalid tasks but continue processing others
                continue
        
        return actions
    
    def _map_task_to_action(self, task_type: str) -> str:
        """
        Map a task type to an appropriate action type.
        
        Args:
            task_type: Type of task to map
            
        Returns:
            Corresponding action type
        """
        # Common mappings
        task_to_action_map = {
            'navigation': 'move_to',
            'manipulation': 'pick_up',  # This would need to be more nuanced in practice
            'perception': 'detect_object',
            'composite': 'execute_sequence'
        }
        
        # If it's already an action type, return as-is
        if task_type in self.action_mapping:
            return task_type
        
        # Otherwise, try to map from task type
        return task_to_action_map.get(task_type, task_type)
    
    def _create_action_parameters(self, task: TaskModel) -> Dict[str, Any]:
        """
        Create action parameters from task parameters.
        
        Args:
            task: TaskModel to extract parameters from
            
        Returns:
            Dictionary of action parameters
        """
        # Start with task parameters
        params = task.parameters.copy()
        
        # Add any additional parameters based on action type
        if task.task_type == 'move_to':
            # Ensure target location is properly formatted
            if 'target_location' not in params:
                params['target_location'] = {
                    'x': 0.0,
                    'y': 0.0,
                    'z': 0.0,
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                }
        
        elif task.task_type == 'pick_up':
            # Ensure target object is specified
            if 'target_object' not in params:
                params['target_object'] = {
                    'name': 'unknown',
                    'pose': {'x': 0.0, 'y': 0.0, 'z': 0.0}
                }
        
        elif task.task_type == 'detect_object':
            # Ensure object type is specified
            if 'object_type' not in params:
                params['object_type'] = 'any'
        
        # Add timestamps and metadata
        params['created_at'] = time.time()
        params['task_description'] = task.task_description
        
        return params
    
    def _determine_execution_order(self, actions: List[ActionModel], cognitive_plan: 'CognitivePlanModel') -> List[ActionModel]:
        """
        Determine the execution order of actions considering dependencies.
        
        Args:
            actions: List of ActionModels to order
            cognitive_plan: Original CognitivePlanModel for context
            
        Returns:
            Ordered list of ActionModels
        """
        # For now, return actions in their original order
        # In a real implementation, this would analyze dependencies and create an execution graph
        
        # Create a dependency graph based on task relationships in the cognitive plan
        # This would be derived from the plan's execution context or task decomposition
        ordered_actions = []
        
        # For now, a simple linear ordering
        # TODO: Implement dependency resolution and parallel execution logic
        for action in actions:
            ordered_actions.append(action)
        
        # In a more sophisticated implementation, we would:
        # 1. Build a dependency graph based on task relationships
        # 2. Perform topological sort to determine execution order
        # 3. Group actions that can run in parallel
        # 4. Respect safety and resource constraints
        
        return ordered_actions
    
    def _calculate_estimated_duration(self, actions: List[ActionModel]) -> float:
        """
        Calculate estimated duration for the action sequence.
        
        Args:
            actions: List of ActionModels to calculate duration for
            
        Returns:
            Estimated duration in seconds
        """
        total_duration = 0.0
        
        for action in actions:
            # Use the action's timeout as the estimate
            # In a real system, this would be based on historical data
            estimated_time = action.timeout
            
            # Add potential retry time
            estimated_time = estimated_time * (action.retry_count + 1)
            
            total_duration += estimated_time
        
        # Apply a safety factor (multiply by 1.2 to account for unexpected delays)
        return total_duration * 1.2
    
    @log_exception()
    def validate_action_sequence(self, action_sequence: ActionSequenceModel) -> List[str]:
        """
        Validate an action sequence for execution readiness.
        
        Args:
            action_sequence: ActionSequenceModel to validate
            
        Returns:
            List of validation errors or empty list if valid
        """
        errors = []
        
        # Check if sequence has actions
        if not action_sequence.actions:
            errors.append("Action sequence contains no actions")
        
        # Validate each action
        for i, action in enumerate(action_sequence.actions):
            if not action.action_id:
                errors.append(f"Action {i+1} has no action_id")
            
            if not action.action_type:
                errors.append(f"Action {action.action_id} has no action_type")
        
        # Check for circular dependencies (simplified check)
        # In a real implementation, this would be more sophisticated
        if len(action_sequence.actions) > len(set(a.action_id for a in action_sequence.actions)):
            errors.append("Action sequence contains duplicate action IDs")
        
        return errors
    
    @log_exception()
    async def optimize_sequence(self, action_sequence: ActionSequenceModel) -> ActionSequenceModel:
        """
        Optimize an action sequence for better execution performance.
        
        Args:
            action_sequence: ActionSequenceModel to optimize
            
        Returns:
            Optimized ActionSequenceModel
        """
        # For now, return the same sequence
        # In a real implementation, this would perform optimizations like:
        # - Merging similar actions
        # - Reordering for efficiency
        # - Parallelizing independent actions
        # - Resource allocation optimization
        
        self.logger.info(f"Optimization not yet implemented, returning original sequence with {len(action_sequence.actions)} actions")
        return action_sequence
    
    @retry_on_failure(max_retries=2, delay=1.0)
    @log_exception()
    async def execute_action_sequence(self, action_sequence: ActionSequenceModel) -> bool:
        """
        Execute an action sequence (placeholder implementation).
        
        Args:
            action_sequence: ActionSequenceModel to execute
            
        Returns:
            True if successful, False otherwise
        """
        # This is a placeholder - actual execution would happen in the action execution module
        self.logger.info(f"Action sequence execution is a placeholder. Would execute {len(action_sequence.actions)} actions")
        return True


# Global action sequencer instance
_action_sequencer = None


def get_action_sequencer() -> ActionSequencer:
    """Get the global action sequencer instance."""
    global _action_sequencer
    if _action_sequencer is None:
        _action_sequencer = ActionSequencer()
    return _action_sequencer


def sequence_plan(cognitive_plan: 'CognitivePlanModel') -> ActionSequenceModel:
    """Convenience function to sequence a cognitive plan."""
    sequencer = get_action_sequencer()
    return asyncio.run(sequencer.sequence_plan(cognitive_plan))