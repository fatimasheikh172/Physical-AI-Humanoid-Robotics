"""
Perception pipeline integration with LLM planning for the Vision-Language-Action (VLA) module.

This module connects computer vision perception outputs with the LLM-based cognitive planning
system, enabling visual information to influence robotic action plans.
"""

import asyncio
import logging
from typing import Dict, List, Any, Optional
from dataclasses import dataclass
import json
import time

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import VisionObservation, CognitivePlan, Action
from ..core.data_models import VisionObservationModel, CognitivePlanModel, ActionModel
from .computer_vision_processor import get_vision_processor
from ..llm_planning.cognitive_planner import get_cognitive_planner


@dataclass
class PerceptionContext:
    """Context information for perception-enhanced planning."""
    detected_objects: List['DetectedObjectModel']
    environment_map: Dict[str, Any]
    robot_position: Dict[str, float]
    time_of_day: str  # morning, afternoon, evening, night


class PerceptionLLMIntegrator:
    """
    Integrates perception data from computer vision with LLM-based cognitive planning.
    Uses visual information to enhance and contextualize action plans.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Initialize components
        self.vision_processor = get_vision_processor()
        self.cognitive_planner = get_cognitive_planner()
        
        # Context memory for planning
        self.context_history = []
        self.max_context_items = getattr(self.config, 'max_perception_context_items', 10)
        
        self.logger.info("PerceptionLLMIntegrator initialized")
    
    @log_exception()
    async def integrate_perception_into_plan(
        self, 
        cognitive_plan: CognitivePlanModel, 
        vision_observation: VisionObservationModel
    ) -> CognitivePlanModel:
        """
        Enhance an existing cognitive plan with perception information.
        
        Args:
            cognitive_plan: Original CognitivePlanModel
            vision_observation: VisionObservationModel with perception data
            
        Returns:
            Enhanced CognitivePlanModel with perception context
        """
        try:
            self.logger.info(f"Integrating perception data into plan: {cognitive_plan.plan_id}")
            
            # Create enhanced plan based on original with perception context
            enhanced_plan = CognitivePlanModel(
                plan_id=f"{cognitive_plan.plan_id}_enhanced",
                command_id=cognitive_plan.command_id,
                llm_model=cognitive_plan.llm_model,
                llm_response=cognitive_plan.llm_response,
                task_decomposition=cognitive_plan.task_decomposition.copy(),
                execution_context=cognitive_plan.execution_context.copy(),
                confidence=cognitive_plan.confidence,
                created_at=time.time()
            )
            
            # Enhance execution context with perception data
            enhanced_plan.execution_context['perception_enhancement'] = {
                'objects_detected': [obj.to_dict() for obj in vision_observation.objects_detected],
                'environment_map': vision_observation.environment_map,
                'timestamp': vision_observation.timestamp,
                'confidence_boost': self._calculate_confidence_boost(vision_observation)
            }
            
            # Possibly update tasks based on new perception data
            updated_tasks = await self._update_tasks_with_perception(
                cognitive_plan.task_decomposition, 
                vision_observation
            )
            enhanced_plan.task_decomposition = updated_tasks
            
            # Update confidence based on perception quality
            perception_confidence = self._calculate_perception_confidence(vision_observation)
            enhanced_plan.confidence = min(1.0, cognitive_plan.confidence * (1 + perception_confidence * 0.1))
            
            # Add to context history
            self._add_to_context_history(vision_observation, enhanced_plan)
            
            self.logger.info(f"Perception integration completed for plan: {enhanced_plan.plan_id}")
            return enhanced_plan
            
        except Exception as e:
            self.logger.error(f"Error in perception-LLM integration: {e}")
            raise VLAException(
                f"Perception-LLM integration error: {str(e)}", 
                VLAErrorType.INTEGRATION_ERROR,
                e
            )
    
    @log_exception()
    async def create_perception_guided_plan(
        self, 
        command: str, 
        user_id: str,
        robot_state: Optional['RobotStateModel'] = None
    ) -> CognitivePlanModel:
        """
        Create a cognitive plan guided by current perception data.
        
        Args:
            command: Natural language command
            user_id: ID of the user issuing the command
            robot_state: Current RobotStateModel for context
            
        Returns:
            CognitivePlanModel enhanced with perception context
        """
        try:
            self.logger.info(f"Creating perception-guided plan for command: {command}")
            
            # Get latest perception data
            current_observation = await self._get_current_environment_state(robot_state)
            
            # Create perception context
            perception_context = await self._build_perception_context(current_observation, robot_state)
            
            # Enhance the original command with perception context
            enhanced_command = self._enhance_command_with_perception(command, perception_context)
            
            self.logger.debug(f"Enhanced command: {enhanced_command}")
            
            # Create voice command model for cognitive planner
            from ..core.data_models import VoiceCommandModel
            voice_command = VoiceCommandModel.create(
                transcript=enhanced_command,
                user_id=user_id,
                language=getattr(self.config, 'default_language', 'en')
            )
            
            # Generate plan with the enhanced command
            cognitive_plan = await self.cognitive_planner.plan(voice_command)
            
            # Further enhance the plan with detailed perception data
            if current_observation:
                cognitive_plan = await self.integrate_perception_into_plan(
                    cognitive_plan, 
                    current_observation
                )
            
            self.logger.info(f"Created perception-guided plan: {cognitive_plan.plan_id}")
            return cognitive_plan
            
        except Exception as e:
            self.logger.error(f"Error creating perception-guided plan: {e}")
            raise VLAException(
                f"Perception-guided planning error: {str(e)}", 
                VLAErrorType.PLANNING_ERROR,
                e
            )
    
    async def _get_current_environment_state(self, robot_state: Optional['RobotStateModel'] = None) -> Optional[VisionObservationModel]:
        """
        Get the current environment state through perception.
        
        Args:
            robot_state: Current robot state for context
            
        Returns:
            VisionObservationModel with current state or None if not available
        """
        try:
            # In a real system, this would trigger active perception (take picture, process it)
            # For now, we'll simulate this by calling the vision processor
            # In practice, this might access the most recent observation from a perception buffer
            
            # If we have robot position info, we can enhance with spatial context
            if robot_state:
                self.logger.debug(f"Getting environment state at position {robot_state.position}")
            
            # Return the most recent observation from our context history, or simulate one
            if self.context_history:
                return self.context_history[-1]['observation']
            else:
                # In a real system, we would actively acquire vision data
                # For now, return None which indicates no recent perception data
                return None
                
        except Exception as e:
            self.logger.error(f"Error getting current environment state: {e}")
            return None
    
    async def _build_perception_context(
        self, 
        observation: Optional[VisionObservationModel], 
        robot_state: Optional['RobotStateModel']
    ) -> PerceptionContext:
        """
        Build perception context from observation and robot state.
        
        Args:
            observation: Current VisionObservationModel
            robot_state: Current RobotStateModel
            
        Returns:
            PerceptionContext with environment information
        """
        detected_objects = []
        environment_map = {}
        robot_position = {"x": 0.0, "y": 0.0, "z": 0.0}
        
        if observation:
            detected_objects = observation.objects_detected
            environment_map = observation.environment_map or {}
        
        if robot_state and robot_state.position:
            robot_position = robot_state.position
        
        # Determine time of day approximately
        current_hour = time.localtime().tm_hour
        if 6 <= current_hour < 12:
            time_of_day = "morning"
        elif 12 <= current_hour < 18:
            time_of_day = "afternoon"
        elif 18 <= current_hour < 22:
            time_of_day = "evening"
        else:
            time_of_day = "night"
        
        return PerceptionContext(
            detected_objects=detected_objects,
            environment_map=environment_map,
            robot_position=robot_position,
            time_of_day=time_of_day
        )
    
    def _enhance_command_with_perception(self, command: str, perception_context: PerceptionContext) -> str:
        """
        Enhance a natural language command with perception context.
        
        Args:
            command: Original natural language command
            perception_context: PerceptionContext with environment information
            
        Returns:
            Enhanced command string incorporating perception data
        """
        # Create an enhanced command that includes available perception information
        # This helps the LLM make better plans with current context
        
        enhancements = []
        
        # Add information about detected objects
        if perception_context.detected_objects:
            object_names = [obj.name for obj in perception_context.detected_objects[:5]]  # Limit to first 5 objects
            enhancements.append(f"The environment currently contains: {', '.join(object_names)}.")
        
        # Add information about robot position
        pos = perception_context.robot_position
        enhancements.append(f"The robot is currently at position (x={pos['x']:.2f}, y={pos['y']:.2f}, z={pos['z']:.2f}).")
        
        # Add time of day context
        enhancements.append(f"It is currently {perception_context.time_of_day}.")
        
        # Build enhanced command
        enhanced_command = f"{command} Context: {' '.join(enhancements)} Based on the current environment, make appropriate adjustments to the plan."
        
        return enhanced_command
    
    async def _update_tasks_with_perception(
        self, 
        original_tasks: List['TaskModel'], 
        vision_observation: VisionObservationModel
    ) -> List['TaskModel']:
        """
        Update tasks based on new perception information.
        
        Args:
            original_tasks: List of original TaskModels
            vision_observation: VisionObservationModel with new information
            
        Returns:
            Updated list of TaskModels
        """
        try:
            updated_tasks = []
            
            for i, task in enumerate(original_tasks):
                # Check if the task needs updating based on perception
                updated_task = await self._update_task_if_needed(task, vision_observation)
                updated_tasks.append(updated_task)
            
            self.logger.debug(f"Updated {len([i for i in range(len(original_tasks)) if original_tasks[i] != updated_tasks[i])} tasks based on perception")
            return updated_tasks
            
        except Exception as e:
            self.logger.error(f"Error updating tasks with perception: {e}")
            # Return original tasks if update fails
            return original_tasks
    
    async def _update_task_if_needed(
        self, 
        task: 'TaskModel', 
        vision_observation: VisionObservationModel
    ) -> 'TaskModel':
        """
        Update a task if perception data suggests changes are needed.
        
        Args:
            task: Original TaskModel
            vision_observation: VisionObservationModel with new information
            
        Returns:
            Updated TaskModel (could be the same as original)
        """
        updated_task = task  # Start with original task
        
        # Example: Update navigation tasks if new obstacles detected
        if task.task_type == 'navigation':
            # Check if the target location is still accessible based on new perception
            target_location = task.parameters.get('target_location')
            if target_location:
                # In a real implementation, check for new obstacles between robot and target
                # For now, just check if target location is significantly different from perceived objects
                for obj in vision_observation.objects_detected:
                    obj_pos = obj.position_3d
                    if obj_pos:
                        # Check if object is blocking the path to target (simplified check)
                        # In a real implementation, this would use path planning algorithms
                        pass
        
        # Example: Update manipulation tasks based on object availability
        elif task.task_type == 'manipulation':
            target_object = task.parameters.get('target_object', {}).get('name', '').lower()
            if target_object:
                # Check if target object is available in the current perception
                target_obj_found = any(
                    target_object in obj.name.lower() 
                    for obj in vision_observation.objects_detected
                )
                
                if not target_obj_found:
                    # Modify task to search for the object or report it's not found
                    updated_task = self._modify_task_for_missing_object(task, target_object)
        
        return updated_task
    
    def _modify_task_for_missing_object(self, task: 'TaskModel', object_name: str) -> 'TaskModel':
        """
        Modify a task when the target object is not found in perception.
        
        Args:
            task: Original TaskModel
            object_name: Name of the missing object
            
        Returns:
            Modified TaskModel
        """
        # Create a new task that searches for the missing object first
        search_task = TaskModel.create(
            task_id=f"search_{task.task_id}",
            task_description=f"Search for {object_name} in environment",
            task_type="search",
            priority=task.priority + 1,  # Higher priority than original task
            parameters={
                'object_to_find': object_name,
                'search_pattern': 'spiral_outward',
                'max_search_time': 30.0
            }
        )
        
        # Update the original task to depend on finding the object
        task.parameters['required_conditions'] = [f"object_{object_name}_located"]
        
        # In a full implementation, we'd return a modified task list with both search and original task
        # For this single-task update, we'll just modify the parameters
        task.parameters['fallback_behavior'] = f"Report that {object_name} was not found"
        
        return task
    
    def _calculate_confidence_boost(self, vision_observation: VisionObservationModel) -> float:
        """
        Calculate confidence boost based on perception quality.
        
        Args:
            vision_observation: VisionObservationModel to evaluate
            
        Returns:
            Confidence boost value between 0.0 and 1.0
        """
        if not vision_observation.objects_detected:
            return 0.0  # No objects detected, no boost
        
        # Calculate boost based on number of detected objects and their confidence
        avg_confidence = sum(obj.confidence for obj in vision_observation.objects_detected) / len(vision_observation.objects_detected)
        
        # Boost is proportional to number of objects and their average confidence
        boost = min(1.0, (len(vision_observation.objects_detected) / 10.0) * avg_confidence)
        
        return boost
    
    def _calculate_perception_confidence(self, vision_observation: VisionObservationModel) -> float:
        """
        Calculate overall perception confidence.
        
        Args:
            vision_observation: VisionObservationModel to evaluate
            
        Returns:
            Overall perception confidence between 0.0 and 1.0
        """
        if not vision_observation.objects_detected:
            return 0.1  # Very low confidence when no objects detected
        
        # Calculate confidence based on factors like number of objects, avg confidence, etc.
        avg_object_confidence = sum(obj.confidence for obj in vision_observation.objects_detected) / len(vision_observation.objects_detected)
        count_factor = min(1.0, len(vision_observation.objects_detected) / 20.0)  # Cap at 20 objects
        
        # Combine factors
        perception_confidence = (avg_object_confidence * 0.7) + (count_factor * 0.3)
        
        return perception_confidence
    
    def _add_to_context_history(self, observation: VisionObservationModel, plan: CognitivePlanModel):
        """
        Add an observation-plan pair to the context history.
        
        Args:
            observation: VisionObservationModel that was used
            plan: CognitivePlanModel that was generated/enhanced
        """
        context_entry = {
            'observation': observation,
            'plan': plan,
            'timestamp': time.time()
        }
        
        self.context_history.append(context_entry)
        
        # Limit history size
        if len(self.context_history) > self.max_context_items:
            self.context_history = self.context_history[-self.max_context_items:]
    
    @log_exception()
    async def validate_plan_with_perception(
        self, 
        cognitive_plan: CognitivePlanModel, 
        vision_observation: Optional[VisionObservationModel] = None
    ) -> List[str]:
        """
        Validate a cognitive plan against current perception data.
        
        Args:
            cognitive_plan: CognitivePlanModel to validate
            vision_observation: Optional VisionObservationModel to validate against
            
        Returns:
            List of validation errors, empty if plan is valid
        """
        errors = []
        
        # If no vision observation provided, try to get current one
        if not vision_observation:
            vision_observation = await self._get_current_environment_state()
        
        if not vision_observation:
            # If no perception data available, just validate plan structure
            self.logger.warning("No perception data available for plan validation, only structural validation possible")
            return self.cognitive_planner.validate_plan(cognitive_plan)
        
        # Validate each task against current observations
        for i, task in enumerate(cognitive_plan.task_decomposition):
            task_errors = await self._validate_task_with_perception(task, vision_observation)
            errors.extend([f"Task {i+1} ({task.task_id}): {error}" for error in task_errors])
        
        # Validate plan consistency with environment
        consistency_errors = await self._validate_plan_environmental_consistency(cognitive_plan, vision_observation)
        errors.extend(consistency_errors)
        
        return errors
    
    async def _validate_task_with_perception(
        self, 
        task: 'TaskModel', 
        vision_observation: VisionObservationModel
    ) -> List[str]:
        """
        Validate a single task against perception data.
        
        Args:
            task: TaskModel to validate
            vision_observation: VisionObservationModel to validate against
            
        Returns:
            List of validation errors for the task
        """
        errors = []
        
        # Validate navigation tasks against environment obstacles
        if task.task_type == 'navigation':
            target_location = task.parameters.get('target_location')
            if target_location:
                # Check if there are obstacles at the target location
                for obj in vision_observation.objects_detected:
                    # Simplified check - in reality this would use more sophisticated spatial reasoning
                    pass
        
        # Validate manipulation tasks against object availability
        elif task.task_type == 'manipulation':
            target_object = task.parameters.get('target_object', {}).get('name', '')
            if target_object:
                # Check if object exists in environment
                obj_exists = any(
                    target_object.lower() in obj.name.lower() 
                    for obj in vision_observation.objects_detected
                )
                
                if not obj_exists:
                    errors.append(f"Target object '{target_object}' not found in current environment")
        
        # Validate perception tasks against current observations
        elif task.task_type == 'perception':
            required_object = task.parameters.get('target_object', {}).get('name', '')
            if required_object:
                # Check if object is already detected
                obj_already_detected = any(
                    required_object.lower() in obj.name.lower()
                    for obj in vision_observation.objects_detected
                )
                
                if obj_already_detected:
                    errors.append(f"Object '{required_object}' is already detected in environment")
        
        return errors
    
    async def _validate_plan_environmental_consistency(
        self, 
        cognitive_plan: CognitivePlanModel, 
        vision_observation: VisionObservationModel
    ) -> List[str]:
        """
        Validate the plan against the current environment for consistency.
        
        Args:
            cognitive_plan: CognitivePlanModel to validate
            vision_observation: VisionObservationModel with current environment state
            
        Returns:
            List of environmental consistency errors
        """
        errors = []
        
        # Check for environmental contradictions
        # Example: A plan to "go to the kitchen" but the robot is already in the kitchen
        robot_location = vision_observation.environment_map.get('robot_location', {}).get('room', 'unknown')
        for task in cognitive_plan.task_decomposition:
            if task.task_type == 'navigation' and 'kitchen' in task.task_description.lower():
                if robot_location == 'kitchen':
                    errors.append(f"Robot is already in the kitchen, navigation task may be unnecessary")
        
        return errors


# Global perception-LLM integrator instance
_perception_integrator = None


def get_perception_integrator() -> PerceptionLLMIntegrator:
    """Get the global perception-LLM integrator instance."""
    global _perception_integrator
    if _perception_integrator is None:
        _perception_integrator = PerceptionLLMIntegrator()
    return _perception_integrator


async def integrate_perception_into_plan(
    cognitive_plan: CognitivePlanModel, 
    vision_observation: VisionObservationModel
) -> CognitivePlanModel:
    """Convenience function to integrate perception into a cognitive plan."""
    integrator = get_perception_integrator()
    return await integrator.integrate_perception_into_plan(cognitive_plan, vision_observation)


async def create_perception_guided_plan(
    command: str,
    user_id: str,
    robot_state: Optional['RobotStateModel'] = None
) -> CognitivePlanModel:
    """Convenience function to create a perception-guided plan."""
    integrator = get_perception_integrator()
    return await integrator.create_perception_guided_plan(command, user_id, robot_state)


async def validate_plan_with_perception(
    cognitive_plan: CognitivePlanModel,
    vision_observation: Optional[VisionObservationModel] = None
) -> List[str]:
    """Convenience function to validate a plan with perception data."""
    integrator = get_perception_integrator()
    return await integrator.validate_plan_with_perception(cognitive_plan, vision_observation)