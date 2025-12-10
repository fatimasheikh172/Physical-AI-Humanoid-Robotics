"""
Object identification and manipulation integration for the Vision-Language-Action (VLA) module.

This module integrates object detection and identification capabilities with 
manipulation actions, enabling the robot to perceive objects and perform actions on them.
"""

import asyncio
import logging
import math
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import CognitivePlan, Action, ActionResponse, ExecutionStatus
from ..core.data_models import CognitivePlanModel, ActionModel, ActionResponseModel, VisionObservationModel, DetectedObjectModel
from ..vision_perception.vision_processor import get_vision_processor
from ..action_execution.robot_controller import get_robot_controller
from ..llm_planning.cognitive_planner import get_cognitive_planner
from .path_planning_integrator import get_path_integrator
from .obstacle_navigation import get_obstacle_navigation_manager


@dataclass
class IdentifiedObject:
    """Represents an identified object with properties for manipulation."""
    object_id: str
    name: str
    position_3d: Dict[str, float]  # 3D position in space
    bounding_box_2d: Dict[str, float]  # 2D bounding box (x_min, y_min, x_max, y_max)
    confidence: float  # Detection confidence
    object_type: str  # Type classification (e.g., "cup", "book", "bottle")
    properties: Dict[str, Any]  # Additional properties like color, size, etc.


@dataclass
class ObjectManipulationPlan:
    """A plan for manipulating a specific object."""
    object_details: IdentifiedObject
    manipulation_action: str  # "pick_up", "place", "grasp", "release", etc.
    target_location: Optional[Dict[str, float]]  # For placing or moving objects
    grasp_configuration: Optional[Dict[str, Any]]  # Grip configuration for manipulation
    safety_validation_passed: bool = False


class ObjectManipulationIntegrator:
    """
    Integrates object identification with manipulation capabilities to enable 
    the robot to perceive objects and perform appropriate manipulation actions.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Initialize components
        self.vision_processor = get_vision_processor()
        self.robot_controller = get_robot_controller()
        self.cognitive_planner = get_cognitive_planner()
        self.path_integrator = get_path_integrator()
        self.obstacle_navigation = get_obstacle_navigation_manager()
        
        # Object tracking
        self.tracked_objects = {}
        self.manipulation_targets = []
        
        # Manipulation parameters
        self.grasp_approach_distance = getattr(self.config, 'grasp_approach_distance', 0.15)  # meters
        self.place_offset_distance = getattr(self.config, 'place_offset_distance', 0.10)  # meters
        self.manipulation_accuracy_threshold = getattr(self.config, 'manipulation_accuracy_threshold', 0.02)  # meters
        self.object_stability_window = getattr(self.config, 'object_stability_window', 2.0)  # seconds
        
        self.logger.info("ObjectManipulationIntegrator initialized")
    
    @log_exception()
    async def integrate_object_identification_with_manipulation(
        self, 
        cognitive_plan: CognitivePlanModel, 
        robot_state: 'RobotStateModel'
    ) -> CognitivePlanModel:
        """
        Integrate object identification information with manipulation tasks in a cognitive plan.
        
        Args:
            cognitive_plan: CognitivePlanModel to enhance with object identification information
            robot_state: Current RobotStateModel for context
            
        Returns:
            Enhanced CognitivePlanModel with object identification and manipulation details
        """
        try:
            self.logger.info(f"Integrating object identification with manipulation for plan: {cognitive_plan.plan_id}")
            
            # Identify tasks that require object identification or manipulation
            identification_tasks = []
            manipulation_tasks = []
            
            for task in cognitive_plan.task_decomposition:
                if 'detect' in task.task_type.lower() or 'identify' in task.task_type.lower() or 'find' in task.task_type.lower():
                    identification_tasks.append(task)
                elif 'pick' in task.task_type.lower() or 'grasp' in task.task_type.lower() or 'place' in task.task_type.lower() or 'manipul' in task.task_type.lower():
                    manipulation_tasks.append(task)
            
            # For each manipulation task, find corresponding object identification requirements
            updated_tasks = []
            for task in cognitive_plan.task_decomposition:
                if task in manipulation_tasks:
                    # Enhance manipulation task with object identification information
                    enhanced_task = await self._enhance_manipulation_task_with_object_info(task, robot_state)
                    updated_tasks.append(enhanced_task)
                elif task in identification_tasks:
                    # Potentially enhance object identification task with manipulation context
                    enhanced_task = await self._enhance_identification_task_with_manipulation_context(task, robot_state)
                    updated_tasks.append(enhanced_task)
                else:
                    # Regular task, no enhancement needed
                    updated_tasks.append(task)
            
            # Create enhanced cognitive plan
            enhanced_plan = CognitivePlanModel.create(
                plan_id=f"{cognitive_plan.plan_id}_with_obj_manip",
                command_id=cognitive_plan.command_id,
                llm_model=cognitive_plan.llm_model,
                llm_response=cognitive_plan.llm_response,
                task_decomposition=updated_tasks,
                execution_context=cognitive_plan.execution_context,
                confidence=cognitive_plan.confidence,
                created_at=time.time()
            )
            
            self.logger.info(f"Object identification-manipulation integration completed for plan {enhanced_plan.plan_id}")
            return enhanced_plan
            
        except Exception as e:
            self.logger.error(f"Error integrating object identification with manipulation: {e}")
            raise VLAException(
                f"Object identification-manipulation integration error: {str(e)}", 
                VLAErrorType.INTEGRATION_ERROR,
                e
            )
    
    async def _enhance_manipulation_task_with_object_info(
        self, 
        task: 'TaskModel', 
        robot_state: 'RobotStateModel'
    ) -> 'TaskModel':
        """
        Enhance a manipulation task with object identification information.
        
        Args:
            task: TaskModel to enhance
            robot_state: Current RobotStateModel for context
            
        Returns:
            Enhanced TaskModel with object information
        """
        try:
            self.logger.debug(f"Enhancing manipulation task: {task.task_id}")
            
            # Get the target object name from task parameters
            target_object_name = None
            if 'target_object' in task.parameters:
                target_object_name = task.parameters['target_object'].get('name', '').lower()
            elif 'object_name' in task.parameters:
                target_object_name = task.parameters['object_name'].lower()
            elif 'action_object' in task.parameters:
                target_object_name = task.parameters['action_object'].lower()
            
            if not target_object_name:
                # Extract object name from task description if not in parameters
                desc = task.task_description.lower()
                # This is a simplified extraction - in reality, NLP would be needed
                # to extract the target object from the description
                for word in desc.split():
                    if word in ['cup', 'ball', 'book', 'bottle', 'toy', 'object', 'item']:
                        target_object_name = word
                        break
            
            if not target_object_name:
                self.logger.warning(f"No target object found for manipulation task {task.task_id}")
                return task  # Return unchanged
            
            # Identify objects in the environment that match the target
            vision_observation = await self._get_current_environment_observation(robot_state)
            if not vision_observation:
                self.logger.warning(f"No vision observation available for task {task.task_id}")
                # Add a perception action to the task to detect the object
                task.parameters['object_detection_required'] = True
                task.parameters['target_object_name'] = target_object_name
                return task
            
            # Find matching objects in the observation
            matching_objects = await self._find_matching_objects(
                vision_observation, 
                target_object_name,
                robot_state
            )
            
            if not matching_objects:
                self.logger.warning(f"No matching objects found for {target_object_name}")
                # Add perception requirement to task
                task.parameters['object_detection_required'] = True
                task.parameters['target_object_name'] = target_object_name
                return task
            
            # Get the best matching object (highest confidence or closest)
            best_match = await self._select_best_object_match(matching_objects, robot_state)
            
            # Enhance the task with object information
            enhanced_task = TaskModel(
                task_id=task.task_id,
                task_description=task.task_description,
                task_type=task.task_type,
                priority=task.priority,
                parameters={
                    **task.parameters,  # Keep original parameters
                    'target_object_details': self._identified_object_to_dict(best_match),
                    'target_object_position_3d': best_match.position_3d,
                    'target_object_bounding_box': best_match.bounding_box_2d,
                    'detection_confidence': best_match.confidence,
                    'object_approach_position': await self._calculate_approach_position(
                        best_match.position_3d,
                        robot_state.position
                    )
                }
            )
            
            self.logger.debug(f"Enhanced task {task.task_id} with object information: {best_match.name}")
            return enhanced_task
            
        except Exception as e:
            self.logger.error(f"Error enhancing manipulation task with object info: {e}")
            # Return original task if enhancement fails
            return task
    
    async def _enhance_identification_task_with_manipulation_context(
        self, 
        task: 'TaskModel', 
        robot_state: 'RobotStateModel'
    ) -> 'TaskModel':
        """
        Enhance an object identification task with manipulation context.
        
        Args:
            task: TaskModel to enhance with manipulation context
            robot_state: Current RobotStateModel for context
            
        Returns:
            Enhanced TaskModel with manipulation context
        """
        try:
            self.logger.debug(f"Enhancing identification task with manipulation context: {task.task_id}")
            
            # Update the task to indicate it's for manipulation preparation
            enhanced_task = TaskModel(
                task_id=task.task_id,
                task_description=task.task_description,
                task_type=task.task_type,
                priority=task.priority,
                parameters={
                    **task.parameters,
                    'for_manipulation': True,
                    'manipulation_context': {
                        'robot_position': robot_state.position,
                        'arm_reach': getattr(self.config, 'robot_arm_reach', 0.8),  # meters
                        'gripper_capabilities': getattr(self.config, 'robot_gripper_capabilities', {})
                    }
                }
            )
            
            return enhanced_task
            
        except Exception as e:
            self.logger.error(f"Error enhancing identification task with manipulation context: {e}")
            # Return original task if enhancement fails
            return task
    
    async def _get_current_environment_observation(self, robot_state: 'RobotStateModel') -> Optional[VisionObservationModel]:
        """
        Get current environment observation with object detections.
        
        Args:
            robot_state: Current RobotStateModel for context
            
        Returns:
            Latest VisionObservationModel or None if unavailable
        """
        try:
            # In a real implementation, this would trigger vision processing
            # or retrieve the most recent observation
            # For simulation, we'll create a mock observation
            await asyncio.sleep(0.1)  # Simulate observation delay
            
            # Create a mock observation based on the environment context
            # In a real system, this would come from the vision system
            
            # For demonstration, we'll create some mock objects based on common robot environments
            mock_objects = []
            
            # Add objects based on the area robot is in (if known)
            if robot_state.position:
                current_x, current_y = robot_state.position.get('x', 0), robot_state.position.get('y', 0)
                
                # Example: if robot is near x=2, y=1, add some objects in that area
                obj_x, obj_y = current_x + 1.2, current_y + 0.8
                mock_objects.append(
                    DetectedObjectModel.create(
                        object_id=f"obj_{int(time.time()*1000)}_cup",
                        name="red_cup",
                        confidence=0.95,
                        bounding_box={'x_min': 0.2, 'y_min': 0.3, 'x_max': 0.5, 'y_max': 0.7},
                        position_3d={'x': obj_x, 'y': obj_y, 'z': 0.8},
                        properties={'color': 'red', 'type': 'cup', 'material': 'plastic'}
                    )
                )
                
                # Add another object nearby
                obj_x2, obj_y2 = current_x + 1.5, current_y + 0.5
                mock_objects.append(
                    DetectedObjectModel.create(
                        object_id=f"obj_{int(time.time()*1000)}_book",
                        name="blue_book",
                        confidence=0.88,
                        bounding_box={'x_min': 0.6, 'y_min': 0.2, 'x_max': 0.9, 'y_max': 0.8},
                        position_3d={'x': obj_x2, 'y': obj_y2, 'z': 0.85}, 
                        properties={'color': 'blue', 'type': 'book', 'material': 'paper', 'thickness': 0.03}
                    )
                )
            
            if mock_objects:
                observation = VisionObservationModel.create(
                    observation_id=f"vis_obs_{int(time.time()*1000)}",
                    state_id=robot_state.state_id,
                    timestamp=time.time(),
                    objects_detected=mock_objects,
                    environment_map={},
                    processing_metrics={'objects_count': len(mock_objects)}
                )
                return observation
            else:
                # Return empty observation if no mock objects were created
                return VisionObservationModel.create(
                    observation_id=f"vis_obs_{int(time.time()*1000)}",
                    state_id=robot_state.state_id,
                    timestamp=time.time(),
                    objects_detected=[],
                    environment_map={},
                    processing_metrics={'objects_count': 0}
                )
                
        except Exception as e:
            self.logger.error(f"Error getting environment observation: {e}")
            return None
    
    async def _find_matching_objects(
        self, 
        observation: VisionObservationModel, 
        target_object_name: str,
        robot_state: Optional['RobotStateModel'] = None
    ) -> List[IdentifiedObject]:
        """
        Find objects in the observation that match the target name.
        
        Args:
            observation: VisionObservationModel with detected objects
            target_object_name: Name of the target object to find
            robot_state: Optional RobotStateModel for context
            
        Returns:
            List of IdentifiedObject that match the target
        """
        try:
            matching_objects = []
            robot_position = robot_state.position if robot_state and robot_state.position else {'x': 0, 'y': 0, 'z': 0}
            
            for obj in observation.objects_detected:
                # Check if object name matches target (with some fuzzy matching)
                object_name = obj.name.lower()
                
                # Exact match or substring match
                if (target_object_name in object_name or 
                    object_name in target_object_name or
                    self._fuzzy_match(target_object_name, object_name)):
                    
                    # Create IdentifiedObject from DetectedObjectModel
                    identified_obj = IdentifiedObject(
                        object_id=obj.object_id,
                        name=obj.name,
                        position_3d=obj.position_3d or {'x': 0, 'y': 0, 'z': 0},
                        bounding_box_2d=obj.bounding_box,
                        confidence=obj.confidence,
                        object_type=obj.properties.get('type', 'unknown'),
                        properties=obj.properties
                    )
                    
                    matching_objects.append(identified_obj)
            
            # Sort by confidence (descending) if no robot position context
            # Otherwise sort by distance to robot (ascending)
            if robot_state and robot_state.position:
                def distance_to_robot(obj):
                    pos = obj.position_3d
                    robot_pos = robot_state.position
                    dx = pos['x'] - robot_pos['x']
                    dy = pos['y'] - robot_pos['y']
                    dz = pos['z'] - robot_pos['z']
                    return math.sqrt(dx*dx + dy*dy + dz*dz)
                
                matching_objects.sort(key=distance_to_robot)
            else:
                matching_objects.sort(key=lambda o: o.confidence, reverse=True)
            
            self.logger.debug(f"Found {len(matching_objects)} objects matching '{target_object_name}'")
            return matching_objects
            
        except Exception as e:
            self.logger.error(f"Error finding matching objects: {e}")
            return []
    
    def _fuzzy_match(self, str1: str, str2: str) -> bool:
        """
        Simple fuzzy matching between two strings.
        
        Args:
            str1: First string
            str2: Second string
            
        Returns:
            True if strings are similar enough, False otherwise
        """
        # Simplified fuzzy matching - in a real implementation, 
        # this would use proper string similarity algorithms
        str1_clean = str1.replace('_', ' ').replace('-', ' ')
        str2_clean = str2.replace('_', ' ').replace('-', ' ')
        
        # Check if one string contains the other or vice versa
        if str1_clean in str2_clean or str2_clean in str1_clean:
            return True
        
        # Check for common variations (e.g., "cup" vs "cupcake" - not matching)
        str1_words = set(str1_clean.split())
        str2_words = set(str2_clean.split())
        
        # If there's substantial overlap in terms of words
        common_words = str1_words.intersection(str2_words)
        if len(common_words) > 0 and (len(common_words) >= min(len(str1_words), len(str2_words)) * 0.5):
            return True
        
        return False
    
    async def _select_best_object_match(
        self, 
        matching_objects: List[IdentifiedObject], 
        robot_state: 'RobotStateModel'
    ) -> Optional[IdentifiedObject]:
        """
        Select the best object match based on multiple criteria.
        
        Args:
            matching_objects: List of potential object matches
            robot_state: Current RobotStateModel for context
            
        Returns:
            Best IdentifiedObject match or None if none found
        """
        if not matching_objects:
            return None
        
        robot_position = robot_state.position if robot_state.position else {'x': 0, 'y': 0, 'z': 0}
        
        # Score each object based on multiple criteria
        scored_objects = []
        for obj in matching_objects:
            # Distance score (closer is better)
            distance_3d = math.sqrt(
                (obj.position_3d['x'] - robot_position['x'])**2 +
                (obj.position_3d['y'] - robot_position['y'])**2 +
                (obj.position_3d['z'] - robot_position['z'])**2
            )
            distance_score = max(0.0, 1.0 - (distance_3d / 10.0))  # Normalize to 0-1 scale
            
            # Confidence score
            confidence_score = obj.confidence
            
            # Accessibility score based on robot arm reach
            arm_reach = getattr(self.config, 'robot_arm_reach', 0.8)
            accessibility_score = min(1.0, arm_reach / max(distance_3d, 0.1))
            
            # Combined score
            combined_score = (
                confidence_score * 0.5 +  # Confidence is most important
                distance_score * 0.3 +    # Closer is preferred
                accessibility_score * 0.2 # More accessible is preferred
            )
            
            scored_objects.append((obj, combined_score))
        
        # Return the object with the highest score
        best_match = max(scored_objects, key=lambda x: x[1])
        return best_match[0]
    
    def _identified_object_to_dict(self, obj: IdentifiedObject) -> Dict[str, Any]:
        """
        Convert an IdentifiedObject to a dictionary representation.
        
        Args:
            obj: IdentifiedObject to convert
            
        Returns:
            Dictionary representation of the object
        """
        return {
            'object_id': obj.object_id,
            'name': obj.name,
            'position_3d': obj.position_3d,
            'bounding_box_2d': obj.bounding_box_2d,
            'confidence': obj.confidence,
            'object_type': obj.object_type,
            'properties': obj.properties
        }
    
    async def _calculate_approach_position(
        self, 
        object_position: Dict[str, float], 
        robot_position: Dict[str, float]
    ) -> Dict[str, float]:
        """
        Calculate a safe approach position for object manipulation.
        
        Args:
            object_position: Position of the target object
            robot_position: Current position of the robot
            
        Returns:
            Position where the robot should approach the object
        """
        # Calculate vector from robot to object
        dx = object_position['x'] - robot_position['x']
        dy = object_position['y'] - robot_position['y']
        dz = object_position['z'] - robot_position['z']  # We might ignore z for ground navigation
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Normalize the vector and extend it by the approach distance
        approach_distance = self.grasp_approach_distance
        
        if distance > approach_distance:
            # Calculate approach position by moving from robot toward object by approach_distance
            approach_ratio = approach_distance / distance if distance > 0 else 0
            approach_pos = {
                'x': robot_position['x'] + dx * approach_ratio,
                'y': robot_position['y'] + dy * approach_ratio,
                'z': robot_position['z'] + dz * approach_ratio  # Keep Z consistent with target
            }
        else:
            # If already close enough, just approach slightly closer
            approach_ratio = 0.9  # 90% of remaining distance
            approach_pos = {
                'x': robot_position['x'] + dx * approach_ratio,
                'y': robot_position['y'] + dy * approach_ratio,
                'z': robot_position['z'] + dz * approach_ratio
            }
        
        return approach_pos
    
    @log_exception()
    async def validate_manipulation_feasibility(
        self, 
        identified_object: IdentifiedObject, 
        manipulation_action: str, 
        robot_state: 'RobotStateModel'
    ) -> Tuple[bool, List[str]]:
        """
        Validate that a manipulation action is feasible for the identified object.
        
        Args:
            identified_object: IdentifiedObject to validate manipulation for
            manipulation_action: Type of manipulation action ("pick_up", "place", etc.)
            robot_state: Current RobotStateModel for context
            
        Returns:
            Tuple of (feasible, list of issues if not feasible)
        """
        try:
            issues = []
            
            # Check if robot is capable of the required manipulation
            if manipulation_action.lower() in ['pick_up', 'grasp'] and 'manipulation' not in robot_state.capabilities:
                issues.append("Robot does not have manipulation capability")
            
            # Check if object is within reach
            object_position = identified_object.position_3d
            robot_position = robot_state.position
            if robot_position:
                distance = math.sqrt(
                    (object_position['x'] - robot_position['x'])**2 +
                    (object_position['y'] - robot_position['y'])**2 +
                    (object_position['z'] - robot_position['z'])**2
                )
                
                max_reach = getattr(self.config, 'robot_arm_reach', 0.8)
                if distance > max_reach:
                    issues.append(f"Object is out of reach (distance: {distance:.2f}m, max reach: {max_reach}m)")
            
            # Check object properties for manipulation feasibility
            obj_properties = identified_object.properties
            
            # Check if object is too heavy to pick up (if applicable)
            if manipulation_action.lower() in ['pick_up', 'grasp']:
                object_weight = obj_properties.get('weight', 0.0)
                payload_capacity = getattr(self.config, 'robot_payload_capacity', 3.0)
                
                if object_weight > payload_capacity:
                    issues.append(f"Object too heavy to pick up (weight: {object_weight}kg, capacity: {payload_capacity}kg)")
            
            # Check object size for manipulation
            if manipulation_action.lower() in ['pick_up', 'grasp']:
                object_size = obj_properties.get('size', obj_properties.get('dimension', {}))
                if object_size:
                    max_size = getattr(self.config, 'max_manipulable_size', {'width': 0.3, 'height': 0.3, 'depth': 0.3})
                    
                    for dim in ['width', 'height', 'depth']:
                        if dim in object_size and dim in max_size:
                            if object_size[dim] > max_size[dim]:
                                issues.append(f"Object too large in {dim} dimension ({object_size[dim]}m > {max_size[dim]}m)")
            
            # Check if surface is appropriate for placing (if applicable)
            if manipulation_action.lower() == 'place':
                # Verify there's an appropriate surface at the target location
                # This would involve checking the environment map in a real implementation
                pass
            
            is_feasible = len(issues) == 0
            return is_feasible, issues
            
        except Exception as e:
            self.logger.error(f"Error validating manipulation feasibility: {e}")
            return False, [f"Feasibility validation error: {str(e)}"]
    
    def get_tracked_objects(self) -> Dict[str, IdentifiedObject]:
        """
        Get all currently tracked objects.
        
        Returns:
            Dictionary mapping object IDs to IdentifiedObject
        """
        return self.tracked_objects.copy()
    
    def get_manipulation_targets(self) -> List[Dict[str, Any]]:
        """
        Get list of potential manipulation targets.
        
        Returns:
            List of potential manipulation targets with details
        """
        return self.manipulation_targets.copy()


# Global object manipulation integrator instance
_object_manipulation_integrator = None


def get_object_manipulation_integrator() -> ObjectManipulationIntegrator:
    """Get the global object manipulation integrator instance."""
    global _object_manipulation_integrator
    if _object_manipulation_integrator is None:
        _object_manipulation_integrator = ObjectManipulationIntegrator()
    return _object_manipulation_integrator


async def integrate_object_identification_with_manipulation(
    cognitive_plan: CognitivePlanModel, 
    robot_state: 'RobotStateModel'
) -> CognitivePlanModel:
    """Convenience function to integrate object identification with manipulation."""
    integrator = get_object_manipulation_integrator()
    return await integrator.integrate_object_identification_with_manipulation(cognitive_plan, robot_state)


async def validate_manipulation_feasibility(
    identified_object: IdentifiedObject, 
    manipulation_action: str, 
    robot_state: 'RobotStateModel'
) -> Tuple[bool, List[str]]:
    """Convenience function to validate manipulation feasibility."""
    integrator = get_object_manipulation_integrator()
    return await integrator.validate_manipulation_feasibility(identified_object, manipulation_action, robot_state)


def get_tracked_objects() -> Dict[str, IdentifiedObject]:
    """Convenience function to get tracked objects."""
    integrator = get_object_manipulation_integrator()
    return integrator.get_tracked_objects()