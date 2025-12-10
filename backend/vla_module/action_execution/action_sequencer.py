"""
Action Sequencer for the Vision-Language-Action (VLA) module.

This module creates executable action sequences from cognitive plans,
handling dependencies, parallelization opportunities, and validation.
"""

import asyncio
import logging
from typing import Dict, List, Any, Set, Tuple
from enum import Enum
import time
import uuid

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import Action, ActionSequence, ExecutionStatus
from ..core.data_models import ActionModel, ActionSequenceModel, CognitivePlanModel
from .safety_monitor import SafetyMonitor, get_safety_monitor
from .robot_capability_validator import RobotCapabilityValidator, get_capability_validator


@dataclass
class ActionDependencyGraph:
    """Represents dependencies between actions in a sequence."""
    action_id: str
    dependencies: List[str]  # Action IDs that must complete before this action
    dependent_on: List[str]  # Action IDs that depend on this action


class ActionSequencer:
    """
    Creates executable action sequences from cognitive plans with proper 
    dependencies, parallelization, and safety checks.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Initialize safety and capability validators
        self.safety_monitor = get_safety_monitor()
        self.capability_validator = get_capability_validator()
        
        # Cache for dependency calculations
        self.dependency_cache = {}
        
        self.logger.info("ActionSequencer initialized")
    
    @log_exception()
    async def sequence_plan(self, cognitive_plan: CognitivePlanModel) -> ActionSequenceModel:
        """
        Convert a cognitive plan into an executable action sequence.
        
        Args:
            cognitive_plan: CognitivePlanModel to convert to action sequence
            
        Returns:
            ActionSequenceModel with executable actions and proper ordering
        """
        try:
            self.logger.info(f"Sequencing cognitive plan: {cognitive_plan.plan_id}")
            
            # Get the current robot state for validation
            robot_state = await self.get_robot_state()
            
            # Extract actions from the cognitive plan's task decomposition
            actions = []
            action_counter = 1
            
            for task in cognitive_plan.task_decomposition:
                action = self._create_action_from_task(task, cognitive_plan.plan_id, action_counter)
                actions.append(action)
                action_counter += 1
            
            # Create dependency graph based on task decomposition
            dependency_graph = self._create_dependency_graph(actions, cognitive_plan)
            
            # Determine execution order considering dependencies
            execution_order = self._determine_execution_order(actions, dependency_graph)
            
            # Create action sequence model
            action_sequence = ActionSequenceModel.create(
                plan_id=cognitive_plan.plan_id,
                actions=actions,
                dependencies=dependency_graph,
                execution_order=execution_order
            )
            
            # Validate the sequence against safety requirements
            validation_result = await self._validate_sequence_safety(action_sequence, cognitive_plan.execution_context)
            if not validation_result.is_valid:
                error_msg = f"Action sequence failed safety validation: {validation_result.error_messages}"
                self.logger.error(error_msg)
                raise VLAException(error_msg, VLAErrorType.VALIDATION_ERROR)
            
            # Validate against robot capabilities
            capability_result = await self._validate_sequence_capabilities(action_sequence, robot_state)
            if not capability_result.is_valid:
                error_msg = f"Action sequence failed capability validation: {capability_result.error_messages}"
                self.logger.error(error_msg)
                raise VLAException(error_msg, VLAErrorType.VALIDATION_ERROR)
            
            self.logger.info(f"Successfully sequenced plan into sequence with {len(actions)} actions")
            return action_sequence
            
        except Exception as e:
            self.logger.error(f"Error in action sequencing: {e}")
            raise VLAException(
                f"Action sequencing error: {str(e)}", 
                VLAErrorType.PLANNING_ERROR,
                e
            )
    
    def _create_action_from_task(self, task: 'TaskModel', plan_id: str, action_counter: int) -> ActionModel:
        """
        Create an ActionModel from a TaskModel.
        
        Args:
            task: TaskModel to convert
            plan_id: Plan identifier
            action_counter: Counter for generating unique action ID
            
        Returns:
            ActionModel representation of the task
        """
        # Map task type to action type
        # In a real system, this mapping would be more sophisticated
        task_to_action_map = {
            'navigation': 'move_to',
            'manipulation': 'pick_up',  # This would need to be more nuanced in practice
            'perception': 'detect_object',
            'composite': 'execute_sequence'
        }
        
        action_type = task_to_action_map.get(task.task_type, task.task_type)
        
        # Create action parameters based on task parameters
        action_params = task.parameters.copy()
        action_params['plan_id'] = plan_id
        action_params['task_id'] = task.task_id
        
        # Create the ActionModel
        action = ActionModel.create(
            action_type=action_type,
            parameters=action_params,
            timeout=task.parameters.get('timeout', self.config.default_action_timeout),
            retry_count=task.parameters.get('retry_count', self.config.action_retry_count),
            priority=task.priority
        )
        
        # Generate action ID based on plan and counter
        action.action_id = f"{plan_id[:8]}_act_{action_counter:03d}"
        
        return action
    
    def _create_dependency_graph(self, actions: List[ActionModel], cognitive_plan: CognitivePlanModel) -> Dict[str, List[str]]:
        """
        Create a dependency graph based on the cognitive plan's structure.
        
        Args:
            actions: List of actions to create dependencies for
            cognitive_plan: Original cognitive plan for reference
            
        Returns:
            Dictionary mapping action IDs to their dependency lists
        """
        dependencies = {action.action_id: [] for action in actions}
        
        # In a real implementation, this would extract dependencies from the cognitive plan
        # For now, we'll create a simple linear dependency based on the order
        for i in range(1, len(actions)):
            # Make each action dependent on the previous one (simple linear dependency)
            dependencies[actions[i].action_id].append(actions[i-1].action_id)
        
        # In a more sophisticated implementation, we would analyze the plan for:
        # - Resource dependencies (two actions that use the same robot resource)
        # - Data dependencies (actions that require outputs of previous actions)
        # - Safety dependencies (actions that must precede safety checks)
        
        return dependencies
    
    def _determine_execution_order(self, actions: List[ActionModel], dependency_graph: Dict[str, List[str]]) -> List[str]:
        """
        Determine the execution order considering dependencies.
        
        Args:
            actions: List of actions to order
            dependency_graph: Dependency relationships
            
        Returns:
            List of action IDs in execution order
        """
        # This implements a topological sort algorithm to respect dependencies
        # For simplicity, we'll just return the order as determined by dependencies
        
        # Calculate in-degrees for each action
        in_degree = {action.action_id: 0 for action in actions}
        for action_id, deps in dependency_graph.items():
            for dep_id in deps:
                if dep_id in in_degree:
                    in_degree[action_id] += 1
        
        # Initialize queue with actions that have no dependencies
        queue = []
        for action_id, degree in in_degree.items():
            if degree == 0:
                queue.append(action_id)
        
        execution_order = []
        while queue:
            current_action_id = queue.pop(0)
            execution_order.append(current_action_id)
            
            # Update in-degrees for dependent actions
            for action_id in dependency_graph:
                if current_action_id in dependency_graph[action_id]:
                    in_degree[action_id] -= 1
                    if in_degree[action_id] == 0:
                        queue.append(action_id)
        
        # Check for cycles
        if len(execution_order) != len(actions):
            raise VLAException(
                "Circular dependency detected in action sequence",
                VLAErrorType.PLANNING_ERROR
            )
        
        return execution_order
    
    @log_exception()
    async def _validate_sequence_safety(self, action_sequence: ActionSequenceModel, execution_context: Dict[str, Any]) -> ValidationResult:
        """
        Validate the action sequence for safety concerns.
        
        Args:
            action_sequence: ActionSequenceModel to validate
            execution_context: Context information for validation
            
        Returns:
            ValidationResult with validation results
        """
        try:
            errors = []
            
            # Check for safety constraints in each action
            for action in action_sequence.actions:
                action_errors = await self.safety_monitor.validate_action_safety(
                    action=action,
                    execution_context=execution_context
                )
                errors.extend(action_errors)
            
            # Check for safety in the sequence as a whole
            sequence_errors = self._validate_sequence_wide_safety(action_sequence)
            errors.extend(sequence_errors)
            
            return ValidationResult(
                is_valid=len(errors) == 0,
                error_messages=errors
            )
            
        except Exception as e:
            self.logger.error(f"Error validating sequence safety: {e}")
            raise VLAException(
                f"Safety validation error: {str(e)}", 
                VLAErrorType.VALIDATION_ERROR,
                e
            )
    
    def _validate_sequence_wide_safety(self, action_sequence: ActionSequenceModel) -> List[str]:
        """
        Perform safety checks across the entire action sequence.
        
        Args:
            action_sequence: ActionSequenceModel to validate
            
        Returns:
            List of safety validation errors
        """
        errors = []
        
        # Check for resource conflicts (e.g., multiple actions using the same manipulator)
        resource_usage = {}
        for action in action_sequence.actions:
            # Simple resource check - in reality this would be more complex
            resource_type = self._get_required_resource_type(action.action_type)
            if resource_type in resource_usage:
                # Multiple actions requiring the same resource should be scheduled properly
                # For now, we just log this as a potential issue
                self.logger.warning(f"Multiple actions requiring {resource_type} resource: {action_sequence.actions[resource_usage[resource_type]].action_id}, {action.action_id}")
            else:
                resource_usage[resource_type] = action.action_id
        
        # Check for excessive force or movement combinations
        consecutive_high_force = 0
        for action in action_sequence.actions:
            if action.action_type in ['grasp', 'manipulate'] and action.parameters.get('force', 0) > self.config.max_safe_force:
                consecutive_high_force += 1
                if consecutive_high_force > 2:  # More than 2 consecutive high-force actions
                    errors.append(f"Too many consecutive high-force operations may cause damage")
            else:
                consecutive_high_force = 0
        
        # Check for navigation followed immediately by manipulation (robot might be unstable)
        prev_action_type = None
        for i, action in enumerate(action_sequence.actions):
            if prev_action_type == 'move_to' and action.action_type in ['grasp', 'place']:
                errors.append(f"Action {i+1} is manipulation immediately following navigation, may need stabilization time")
            
            prev_action_type = action.action_type
        
        return errors
    
    def _get_required_resource_type(self, action_type: str) -> str:
        """
        Get the type of resource required for an action.
        
        Args:
            action_type: Type of action
            
        Returns:
            Resource type string
        """
        resource_types = {
            'move_to': 'navigation',
            'navigate': 'navigation',
            'pick_up': 'manipulation',
            'place': 'manipulation',
            'grasp': 'manipulation',
            'release': 'manipulation',
            'detect_object': 'perception',
            'rotate': 'navigation',
        }
        
        return resource_types.get(action_type, 'general')
    
    @log_exception()
    async def _validate_sequence_capabilities(self, action_sequence: ActionSequenceModel, robot_state: 'RobotStateModel') -> ValidationResult:
        """
        Validate that the action sequence is within robot capabilities.
        
        Args:
            action_sequence: ActionSequenceModel to validate
            robot_state: Current RobotStateModel for context
            
        Returns:
            ValidationResult with validation results
        """
        try:
            # Use the robot capability validator to check each action
            capability_result = await self.capability_validator.validate_plan_against_capabilities(
                action_sequence.actions,
                robot_state
            )
            
            # Convert to our validation result format
            return ValidationResult(
                is_valid=len(capability_result.error_messages) == 0,
                error_messages=capability_result.error_messages
            )
            
        except Exception as e:
            self.logger.error(f"Error validating sequence capabilities: {e}")
            raise VLAException(
                f"Capability validation error: {str(e)}", 
                VLAErrorType.VALIDATION_ERROR,
                e
            )
    
    @log_exception()
    async def optimize_sequence(self, action_sequence: ActionSequenceModel) -> ActionSequenceModel:
        """
        Optimize an action sequence for performance and efficiency.
        
        Args:
            action_sequence: ActionSequenceModel to optimize
            
        Returns:
            Optimized ActionSequenceModel
        """
        try:
            self.logger.info(f"Optimizing action sequence: {action_sequence.sequence_id}")
            
            # Identify actions that can be safely parallelized
            parallelizable_groups = self._identify_parallelizable_actions(action_sequence)
            
            # Create optimized sequence based on parallelization opportunities
            optimized_actions = []
            
            # Add actions to the optimized sequence
            for group in parallelizable_groups:
                if len(group) == 1:
                    # Single action - add directly
                    optimized_actions.append(group[0])
                else:
                    # Multiple actions that can run in parallel
                    # In a real implementation, these would be marked specially
                    optimized_actions.extend(group)
            
            # Create new sequence with optimized actions
            optimized_sequence = ActionSequenceModel(
                sequence_id=action_sequence.sequence_id + "_optimized",
                plan_id=action_sequence.plan_id,
                actions=optimized_actions,
                dependencies=action_sequence.dependencies,  # Keep original dependencies
                execution_order=self._determine_execution_order(optimized_actions, action_sequence.dependencies)
            )
            
            self.logger.info(f"Optimized sequence has {len(optimized_actions)} actions")
            return optimized_sequence
            
        except Exception as e:
            self.logger.error(f"Error optimizing action sequence: {e}")
            raise VLAException(
                f"Action sequence optimization error: {str(e)}", 
                VLAErrorType.OPTIMIZATION_ERROR,
                e
            )
    
    def _identify_parallelizable_actions(self, action_sequence: ActionSequenceModel) -> List[List[ActionModel]]:
        """
        Identify groups of actions that can be safely executed in parallel.
        
        Args:
            action_sequence: ActionSequenceModel to analyze
            
        Returns:
            List of lists, where each inner list contains actions that can run in parallel
        """
        # In a simple implementation, we'll group actions that don't require the same resources
        # This is a simplified algorithm - a real implementation would be more sophisticated
        
        groups = []
        current_group = []
        
        # For this simplified version, group consecutive actions that don't conflict on resources
        for action in action_sequence.actions:
            resource = self._get_required_resource_type(action.action_type)
            
            # Check if current group is compatible with this action
            can_add_to_group = True
            for existing_action in current_group:
                existing_resource = self._get_required_resource_type(existing_action.action_type)
                if resource == existing_resource and resource in ['navigation', 'manipulation']:  # Exclusive resources
                    can_add_to_group = False
                    break
            
            if can_add_to_group:
                current_group.append(action)
            else:
                if current_group:
                    groups.append(current_group)
                current_group = [action]
        
        # Add the last group if it exists
        if current_group:
            groups.append(current_group)
        
        return groups
    
    def get_robot_state(self) -> 'RobotStateModel':
        """
        Get the current robot state.
        In a real implementation, this would interface with the robot controller.
        For now, returning a placeholder.
        """
        # Placeholder implementation - in a real system, this would get the actual robot state
        from .data_models import RobotStateModel
        return RobotStateModel.create(
            robot_id="placeholder_robot",
            position={'x': 0.0, 'y': 0.0, 'z': 0.0},
            orientation={'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        )


# Global sequencer instance
_action_sequencer = None


def get_action_sequencer() -> ActionSequencer:
    """Get the global action sequencer instance."""
    global _action_sequencer
    if _action_sequencer is None:
        _action_sequencer = ActionSequencer()
    return _action_sequencer


async def sequence_plan(cognitive_plan: CognitivePlanModel) -> ActionSequenceModel:
    """Convenience function to sequence a cognitive plan into an action sequence."""
    sequencer = get_action_sequencer()
    return await sequencer.sequence_plan(cognitive_plan)


async def optimize_sequence(action_sequence: ActionSequenceModel) -> ActionSequenceModel:
    """Convenience function to optimize an action sequence."""
    sequencer = get_action_sequencer()
    return await sequencer.optimize_sequence(action_sequence)