"""
Cognitive Planner for the Vision-Language-Action (VLA) module.

This module translates natural language commands into executable action plans
using large language models and incorporates robot capabilities and constraints.
"""

import asyncio
import logging
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, asdict
from enum import Enum
import json

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import CognitivePlan, Task
from ..core.data_models import CognitivePlanModel, TaskModel
from ..core.vla_manager import get_vla_manager
from .llm_client import LLMClient, get_llm_client


@dataclass
class SafetyCheck:
    """Represents a safety check to be performed before action execution."""
    check_id: str
    check_type: str  # collision_avoidance, force_limits, boundary_check, etc.
    parameters: Dict[str, Any]
    error_message: str


@dataclass
class PlanContext:
    """Contextual information for planning."""
    robot_id: str
    robot_capabilities: List[str]
    environment_map: Optional[Dict[str, Any]]
    current_robot_state: Optional[Dict[str, Any]]
    user_preferences: Optional[Dict[str, Any]]


class CognitivePlanner:
    """
    Main cognitive planner class that translates natural language into action sequences.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Initialize LLM client
        self.llm_client = get_llm_client()
        
        # Initialize with VLA manager data
        self.vla_manager = get_vla_manager()
        
        # Plan validation settings
        self.min_plan_confidence = 0.7  # Minimum acceptable plan confidence
        self.max_tasks_per_plan = 20    # Maximum number of tasks per plan
        self.max_plan_depth = 3         # Maximum nesting depth for composite tasks
        
        self.logger.info("CognitivePlanner initialized")
    
    @log_exception()
    async def plan(self, voice_command: 'VoiceCommand') -> CognitivePlanModel:
        """
        Generate a cognitive plan from a voice command.

        Args:
            voice_command: VoiceCommand object containing the natural language command

        Returns:
            CognitivePlanModel with the planned actions
        """
        try:
            self.logger.info(f"Planning for command: '{voice_command.transcript}' from user {voice_command.user_id}")

            # Get current robot state
            robot_state = await self.vla_manager.get_robot_state()

            # Prepare context for planning
            context = await self._prepare_plan_context(voice_command, robot_state)

            # Generate the plan using the LLM
            plan_data = await self.llm_client.generate_plan_from_command(
                command=voice_command.transcript,
                robot_capabilities=context.robot_capabilities,
                environment_context={
                    'environment_map': context.environment_map or {},
                    'current_state': context.current_robot_state or {},
                    'user_preferences': context.user_preferences or {}
                }
            )

            # Create task models from the plan data
            task_models = self._create_tasks_from_plan_data(plan_data)

            # Validate the plan
            is_valid, validation_errors = self._validate_plan(task_models, plan_data)
            if not is_valid:
                self.logger.error(f"Generated plan failed validation: {validation_errors}")
                raise VLAException(
                    f"Plan validation failed: {validation_errors}",
                    VLAErrorType.PLANNING_ERROR
                )

            # Create safety checks for the plan
            safety_checks = self._create_safety_checks(task_models, context)

            # Add safety checks to the execution context
            execution_context = plan_data.get('execution_context', {})
            execution_context['safety_checks'] = [asdict(check) for check in safety_checks]

            # Create the cognitive plan model
            cognitive_plan = CognitivePlanModel.create(
                command_id=voice_command.command_id,
                llm_model=self.config.llm_model,
                llm_response=json.dumps(plan_data),
                task_decomposition=task_models,
                execution_context=execution_context
            )
            cognitive_plan.confidence = self._calculate_plan_confidence(plan_data)

            self.logger.info(f"Generated plan with {len(task_models)} tasks for command")

            return cognitive_plan

        except Exception as e:
            self.logger.error(f"Error in cognitive planning: {e}")
            raise VLAException(
                f"Cognitive planning error: {str(e)}",
                VLAErrorType.PLANNING_ERROR,
                e
            )

    @log_exception()
    async def validate_plan(self, cognitive_plan: CognitivePlanModel) -> List[str]:
        """
        Validate a cognitive plan against robot capabilities and safety constraints.

        Args:
            cognitive_plan: CognitivePlanModel to validate

        Returns:
            List of validation errors or empty list if valid
        """
        try:
            # Get robot capabilities
            robot_state = await self.vla_manager.get_robot_state()
            robot_capabilities = robot_state.capabilities if robot_state else []

            validation_errors = []

            # Validate each task against robot capabilities
            for i, task in enumerate(cognitive_plan.task_decomposition):
                # Check if robot has capability to perform task
                if task.task_type not in robot_capabilities:
                    validation_errors.append(
                        f"Task {i+1} ({task.task_type}) not supported by robot capabilities: {robot_capabilities}"
                    )

                # Validate task parameters
                param_errors = self._validate_task_parameters(task)
                validation_errors.extend(param_errors)

            # Validate plan consistency
            plan_consistency_errors = self._validate_plan_consistency(cognitive_plan)
            validation_errors.extend(plan_consistency_errors)

            # Validate safety constraints
            safety_errors = self._validate_safety_constraints(cognitive_plan)
            validation_errors.extend(safety_errors)

            if validation_errors:
                self.logger.warning(f"Plan validation found errors: {validation_errors}")
            else:
                self.logger.info("Plan validation passed")

            return validation_errors

        except Exception as e:
            self.logger.error(f"Error validating plan: {e}")
            raise VLAException(
                f"Plan validation error: {str(e)}",
                VLAErrorType.VALIDATION_ERROR,
                e
            )

    async def plan_and_execute(self, voice_command: 'VoiceCommand') -> CognitivePlanModel:
        """
        Generate a cognitive plan from a voice command and execute it via the VLA manager.

        Args:
            voice_command: VoiceCommand object containing the natural language command

        Returns:
            CognitivePlanModel that was planned and sent for execution
        """
        try:
            # First, generate the plan
            cognitive_plan = await self.plan(voice_command)

            # Then, pass it to the VLA manager for execution
            if self.vla_manager:
                await self.vla_manager.process_cognitive_plan(cognitive_plan)
            else:
                self.logger.warning("VLA manager not available for execution, plan created but not executed")

            return cognitive_plan

        except Exception as e:
            self.logger.error(f"Error in planning and execution: {e}")
            raise VLAException(
                f"Planning and execution error: {str(e)}",
                VLAErrorType.PLANNING_ERROR,
                e
            )
    
    @log_exception()
    async def validate_plan(self, cognitive_plan: CognitivePlanModel) -> List[str]:
        """
        Validate a cognitive plan against robot capabilities and safety constraints.
        
        Args:
            cognitive_plan: CognitivePlanModel to validate
            
        Returns:
            List of validation errors or empty list if valid
        """
        try:
            # Get robot capabilities
            robot_state = await self.vla_manager.get_robot_state()
            robot_capabilities = robot_state.capabilities if robot_state else []
            
            validation_errors = []
            
            # Validate each task against robot capabilities
            for i, task in enumerate(cognitive_plan.task_decomposition):
                # Check if robot has capability to perform task
                if task.task_type not in robot_capabilities:
                    validation_errors.append(
                        f"Task {i+1} ({task.task_type}) not supported by robot capabilities: {robot_capabilities}"
                    )
                
                # Validate task parameters
                param_errors = self._validate_task_parameters(task)
                validation_errors.extend(param_errors)
            
            # Validate plan consistency
            plan_consistency_errors = self._validate_plan_consistency(cognitive_plan)
            validation_errors.extend(plan_consistency_errors)
            
            # Validate safety constraints
            safety_errors = self._validate_safety_constraints(cognitive_plan)
            validation_errors.extend(safety_errors)
            
            if validation_errors:
                self.logger.warning(f"Plan validation found errors: {validation_errors}")
            else:
                self.logger.info("Plan validation passed")
            
            return validation_errors
            
        except Exception as e:
            self.logger.error(f"Error validating plan: {e}")
            raise VLAException(
                f"Plan validation error: {str(e)}", 
                VLAErrorType.VALIDATION_ERROR,
                e
            )
    
    async def _prepare_plan_context(self, voice_command: 'VoiceCommand', robot_state: Optional[any]) -> PlanContext:
        """Prepare contextual information for planning."""
        # Get robot capabilities from configuration
        robot_capabilities = self.config.robot_capabilities
        
        # Prepare environmental context
        environment_map = None
        current_robot_state = None
        
        if robot_state:
            current_robot_state = {
                'position': robot_state.position,
                'orientation': robot_state.orientation,
                'battery_level': robot_state.battery_level,
                'current_action': robot_state.current_action
            }
        
        # For now, we'll use empty dicts for environment map and user preferences
        # In a real implementation, these would come from perception and user profile systems
        return PlanContext(
            robot_id="default_robot",
            robot_capabilities=robot_capabilities,
            environment_map=environment_map,
            current_robot_state=current_robot_state,
            user_preferences={}
        )
    
    def _create_tasks_from_plan_data(self, plan_data: Dict[str, Any]) -> List[TaskModel]:
        """Create TaskModels from plan data returned by LLM."""
        task_models = []
        
        decomposition = plan_data.get('decomposition', [])
        
        for i, task_data in enumerate(decomposition):
            try:
                # Create TaskModel from the data
                task_model = TaskModel(
                    task_id=task_data.get('step_id', f"task_{i}"),
                    task_description=task_data.get('description', ''),
                    task_type=task_data.get('action_type', 'generic'),
                    priority=task_data.get('priority', 1),
                    parameters=task_data.get('parameters', {})
                )
                
                # Validate the created task
                task_model.validate()
                task_models.append(task_model)
                
            except Exception as e:
                self.logger.error(f"Error creating task from data: {e}")
                # Skip invalid tasks or raise an exception
                continue
        
        return task_models
    
    def _validate_plan(self, task_models: List[TaskModel], plan_data: Dict[str, Any]) -> tuple[bool, List[str]]:
        """Validate the overall plan structure."""
        errors = []
        
        # Check if there are tasks to execute
        if not task_models:
            errors.append("Plan contains no tasks")
        
        # Check if number of tasks exceeds limit
        if len(task_models) > self.max_tasks_per_plan:
            errors.append(f"Plan contains too many tasks ({len(task_models)}, max {self.max_tasks_per_plan})")
        
        # Check for duplicate task IDs
        task_ids = [task.task_id for task in task_models]
        if len(task_ids) != len(set(task_ids)):
            errors.append("Plan contains duplicate task IDs")
        
        # Validate each task individually
        for task in task_models:
            try:
                task.validate()
            except ValueError as e:
                errors.append(f"Task validation error: {str(e)}")
        
        return len(errors) == 0, errors
    
    def _validate_task_parameters(self, task: TaskModel) -> List[str]:
        """Validate parameters for a specific task."""
        errors = []
        
        # Define expected parameters for each task type
        expected_params = {
            'move_to': {'target_location'},
            'pick_up': {'target_object', 'approach_direction'},
            'place': {'target_location', 'placement_surface'},
            'detect_object': {'object_type', 'search_area'},
            'grasp': {'target_object', 'grasp_type'},
            'release': {'release_method'}
        }
        
        task_type = task.task_type
        if task_type in expected_params:
            required_params = expected_params[task_type]
            
            for param in required_params:
                if param not in task.parameters:
                    errors.append(f"Task type '{task_type}' missing required parameter: '{param}'")
        
        return errors
    
    def _validate_plan_consistency(self, cognitive_plan: CognitivePlanModel) -> List[str]:
        """Validate consistency within the plan."""
        errors = []
        
        # Check for logical ordering and dependencies
        # For now, we'll just ensure all required fields are present
        
        if not cognitive_plan.plan_id:
            errors.append("Plan ID is empty")
        
        if not cognitive_plan.command_id:
            errors.append("Command ID is empty")
        
        if not cognitive_plan.task_decomposition:
            errors.append("Plan has no tasks")
        
        return errors
    
    def _validate_safety_constraints(self, cognitive_plan: CognitivePlanModel) -> List[str]:
        """Validate safety constraints for the plan."""
        errors = []
        
        # Check if plan respects safety distance limits
        navigation_tasks = [task for task in cognitive_plan.task_decomposition if task.task_type == 'move_to']
        
        for task in navigation_tasks:
            target_location = task.parameters.get('target_location', {})
            x = target_location.get('x', 0)
            y = target_location.get('y', 0)
            
            # Calculate distance from origin (or from current position in a real implementation)
            distance = (x**2 + y**2)**0.5
            
            if distance > self.config.max_navigation_distance:
                errors.append(
                    f"Navigation task exceeds max navigation distance: {distance} > {self.config.max_navigation_distance}"
                )
        
        # Check if plan contains only allowed action types
        for task in cognitive_plan.task_decomposition:
            if task.task_type not in self.config.allowed_action_types:
                errors.append(f"Action type not allowed: {task.task_type}")
        
        return errors
    
    def _create_safety_checks(self, task_models: List[TaskModel], context: PlanContext) -> List[SafetyCheck]:
        """Create safety checks for the plan."""
        safety_checks = []
        
        for task in task_models:
            if task.task_type == 'move_to':
                # Add collision avoidance check
                check = SafetyCheck(
                    check_id=f"safety_collision_{task.task_id}",
                    check_type='collision_avoidance',
                    parameters={
                        'target_location': task.parameters.get('target_location', {}),
                        'path_tolerance': 0.1  # meters
                    },
                    error_message='Collision detected during navigation'
                )
                safety_checks.append(check)
            
            elif task.task_type in ['pick_up', 'place', 'grasp', 'release']:
                # Add force limit check
                check = SafetyCheck(
                    check_id=f"safety_force_{task.task_id}",
                    check_type='force_limits',
                    parameters={
                        'max_force': 50.0,  # Newtons
                        'force_threshold': task.parameters.get('force_limit', 50.0)
                    },
                    error_message='Excessive force detected during manipulation'
                )
                safety_checks.append(check)
        
        return safety_checks
    
    def _calculate_plan_confidence(self, plan_data: Dict[str, Any]) -> float:
        """Calculate the confidence of the plan."""
        # For now, return a default confidence based on plan complexity
        # In a real implementation, this would be based on LLM's confidence scores
        # or the quality of the plan decomposition
        
        task_count = len(plan_data.get('decomposition', []))
        
        # Lower confidence for more complex plans
        base_confidence = 0.9
        complexity_penalty = min(0.2, task_count * 0.01)  # Max 20% penalty
        
        return max(0.5, base_confidence - complexity_penalty)


# Global cognitive planner instance
_cognitive_planner = None


def get_cognitive_planner() -> CognitivePlanner:
    """Get the global cognitive planner instance."""
    global _cognitive_planner
    if _cognitive_planner is None:
        _cognitive_planner = CognitivePlanner()
    return _cognitive_planner