"""
Path planning integration with cognitive planning for the Vision-Language-Action (VLA) module.

This module integrates path planning capabilities with the cognitive planning system,
enabling the robot to reason about navigation and environmental constraints at a high level.
"""

import asyncio
import logging
import numpy as np
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import CognitivePlan, Task, Action, RobotState
from ..core.data_models import CognitivePlanModel, TaskModel, ActionModel, RobotStateModel
from ..llm_planning.cognitive_planner import get_cognitive_planner
from ..vision_perception.vision_processor import get_vision_processor
from .full_pipeline_integrator import get_vla_pipeline_integrator


@dataclass
class PathPlanningRequest:
    """Request for path planning with cognitive context."""
    start_position: Dict[str, float]
    target_position: Dict[str, float]
    environment_context: Dict[str, Any]
    robot_capabilities: List[str]
    constraints: Dict[str, Any]  # Safety, time, energy, etc.


@dataclass
class PathPlanningResult:
    """Result from path planning operation."""
    path: List[Dict[str, float]]  # Waypoints
    estimated_distance: float
    estimated_time: float
    safety_metrics: Dict[str, float]
    cognitive_plan_refinement: Optional[CognitivePlanModel] = None


class PathCognitiveIntegrator:
    """
    Integrates path planning capabilities with cognitive planning.
    Enables high-level reasoning about navigation and environment constraints.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Initialize components
        self.cognitive_planner = get_cognitive_planner()
        self.vision_processor = get_vision_processor()
        self.pipeline_integrator = get_vla_pipeline_integrator()
        
        # Path planning parameters
        self.safety_margin = getattr(self.config, 'path_safety_margin', 0.3)  # meters
        self.max_path_length = getattr(self.config, 'max_path_length', 50.0)  # meters
        self.path_resolution = getattr(self.config, 'path_resolution', 0.1)   # meters between waypoints
        
        # Cognitive planning integration parameters
        self.navigation_task_keywords = [
            'navigate', 'move to', 'go to', 'travel', 'reach', 'approach', 
            'walk', 'drive', 'go', 'move', 'head to', 'proceed to'
        ]
        
        self.logger.info("PathCognitiveIntegrator initialized")
    
    @log_exception()
    async def integrate_path_planning_with_cognitive_plan(
        self, 
        cognitive_plan: CognitivePlanModel, 
        robot_state: RobotStateModel
    ) -> CognitivePlanModel:
        """
        Integrate path planning information with an existing cognitive plan.
        
        Args:
            cognitive_plan: Original CognitivePlanModel to enhance
            robot_state: Current RobotStateModel with position information
            
        Returns:
            Enhanced CognitivePlanModel with path planning information
        """
        try:
            self.logger.info(f"Integrating path planning with cognitive plan: {cognitive_plan.plan_id}")
            
            # Identify navigation-related tasks in the cognitive plan
            navigation_tasks = []
            for task in cognitive_plan.task_decomposition:
                if self._is_navigation_task(task):
                    navigation_tasks.append(task)
            
            if not navigation_tasks:
                self.logger.info("No navigation tasks found in cognitive plan, skipping path planning integration")
                return cognitive_plan
            
            # Enhance the plan with path planning information
            enhanced_tasks = []
            for task in cognitive_plan.task_decomposition:
                if self._is_navigation_task(task):
                    # Enhance navigation tasks with path planning
                    enhanced_task = await self._enhance_navigation_task(task, robot_state, cognitive_plan.execution_context)
                    enhanced_tasks.append(enhanced_task)
                else:
                    # Keep non-navigation tasks unchanged
                    enhanced_tasks.append(task)
            
            # Create an enhanced cognitive plan
            enhanced_plan = CognitivePlanModel(
                plan_id=f"{cognitive_plan.plan_id}_with_paths",
                command_id=cognitive_plan.command_id,
                llm_response=cognitive_plan.llm_response,
                task_decomposition=enhanced_tasks,
                execution_context=cognitive_plan.execution_context,
                confidence=cognitive_plan.confidence,
                created_at=cognitive_plan.created_at
            )
            
            self.logger.info(f"Path planning integration completed for plan: {enhanced_plan.plan_id}")
            return enhanced_plan
            
        except Exception as e:
            self.logger.error(f"Error in path-cognitive integration: {e}")
            raise VLAException(
                f"Path-cognitive integration error: {str(e)}", 
                VLAErrorType.INTEGRATION_ERROR,
                e
            )
    
    def _is_navigation_task(self, task: TaskModel) -> bool:
        """
        Determine if a task is related to navigation/movement.
        
        Args:
            task: TaskModel to check
            
        Returns:
            True if the task involves navigation, False otherwise
        """
        task_description = task.task_description.lower()
        
        # Check for navigation-related keywords in task description
        for keyword in self.navigation_task_keywords:
            if keyword in task_description:
                return True
        
        # Check for navigation-related action types
        navigation_actions = [
            'move_to', 'navigate', 'go_to', 'travel', 'reach', 'approach',
            'waypoint', 'path_follow', 'drive_to', 'walk_to', 'move_forward',
            'move_backward', 'turn_left', 'turn_right', 'rotate'
        ]
        
        if hasattr(task, 'action_type') and task.action_type in navigation_actions:
            return True
        
        # Check for location-related parameters
        if hasattr(task, 'parameters') and any(param in self.config.location_parameters 
                                             for param in task.parameters.keys()):
            return True
        
        return False
    
    async def _enhance_navigation_task(
        self, 
        task: TaskModel, 
        robot_state: RobotStateModel, 
        execution_context: Dict[str, Any]
    ) -> TaskModel:
        """
        Enhance a navigation task with path planning information.
        
        Args:
            task: TaskModel to enhance
            robot_state: Current RobotStateModel
            execution_context: Context information for execution
            
        Returns:
            Enhanced TaskModel with path planning information
        """
        try:
            # Extract target location from task parameters
            target_location = self._extract_target_location(task)
            if not target_location:
                self.logger.warning(f"No target location found in task {task.task_id}, cannot plan path")
                return task  # Return unchanged if no target
            
            # Get current position from robot state
            current_position = robot_state.position if robot_state.position else {'x': 0.0, 'y': 0.0, 'z': 0.0}
            
            # Prepare path planning request
            path_request = PathPlanningRequest(
                start_position=current_position,
                target_position=target_location,
                environment_context=execution_context.get('environment_map', {}),
                robot_capabilities=robot_state.capabilities if hasattr(robot_state, 'capabilities') else [],
                constraints=execution_context.get('constraints', {})
            )
            
            # Plan path
            path_result = await self._plan_path_with_context(path_request)
            
            # Enhance task with path information
            enhanced_task = TaskModel(
                task_id=task.task_id,
                task_description=task.task_description,
                task_type=task.task_type,
                priority=task.priority,
                parameters={
                    **task.parameters,  # Keep original parameters
                    'planned_path': path_result.path,
                    'estimated_distance': path_result.estimated_distance,
                    'estimated_time': path_result.estimated_time,
                    'safety_metrics': path_result.safety_metrics
                }
            )
            
            self.logger.debug(f"Enhanced task {task.task_id} with path planning information")
            return enhanced_task
            
        except Exception as e:
            self.logger.error(f"Error enhancing navigation task {task.task_id}: {e}")
            # Return original task if enhancement fails
            return task
    
    def _extract_target_location(self, task: TaskModel) -> Optional[Dict[str, float]]:
        """
        Extract target location from task parameters.
        
        Args:
            task: TaskModel to extract location from
            
        Returns:
            Target location dictionary or None if not found
        """
        # Look for common location parameter names
        location_params = ['target_location', 'destination', 'target_position', 'goal_position']
        
        for param_name in location_params:
            if param_name in task.parameters:
                location = task.parameters[param_name]
                # Validate that location has required coordinates
                if isinstance(location, dict) and 'x' in location and 'y' in location:
                    return location
        
        # For some tasks, location might be embedded in description
        # This is a simplified extraction - real implementation would use NLP
        desc = task.task_description.lower()
        if 'kitchen' in desc:
            return self.config.get('kitchen_location', {'x': 5.0, 'y': 3.0, 'z': 0.0})
        elif 'office' in desc:
            return self.config.get('office_location', {'x': -2.0, 'y': 1.5, 'z': 0.0})
        elif 'living room' in desc:
            return self.config.get('living_room_location', {'x': 0.0, 'y': 0.0, 'z': 0.0})
        elif 'bedroom' in desc:
            return self.config.get('bedroom_location', {'x': 4.0, 'y': -2.0, 'z': 0.0})
        
        return None
    
    async def _plan_path_with_context(self, request: PathPlanningRequest) -> PathPlanningResult:
        """
        Plan a path considering the cognitive and environmental context.
        
        Args:
            request: PathPlanningRequest with all necessary information
            
        Returns:
            PathPlanningResult with path and metrics
        """
        try:
            self.logger.debug(f"Planning path from {request.start_position} to {request.target_position}")
            
            # In a real implementation, this would call an actual path planner
            # For now, we'll use a simple simulated path planning approach
            # that considers obstacles from the environment context
            
            # Simulate path planning delay
            await asyncio.sleep(0.1)  # 100ms simulation delay
            
            # Generate a simple path (in a real system, this would be actual path planning)
            path = self._generate_path(request.start_position, request.target_position)
            
            # Calculate path metrics
            distance = self._calculate_path_distance(path)
            time_estimate = self._estimate_travel_time(distance, request.robot_capabilities)
            
            # Calculate safety metrics (simulated)
            safety_metrics = {
                'obstacle_clearance': 0.8,  # Average distance to nearest obstacle
                'path_complexity': len(path) / max(1, distance),  # Waypoints per meter
                'collision_risk': 0.1  # Estimated risk of collision
            }
            
            result = PathPlanningResult(
                path=path,
                estimated_distance=distance,
                estimated_time=time_estimate,
                safety_metrics=safety_metrics
            )
            
            self.logger.debug(f"Path planned: {len(path)} waypoints, distance: {distance:.2f}m, time: {time_estimate:.2f}s")
            return result
            
        except Exception as e:
            self.logger.error(f"Error in path planning with context: {e}")
            # Return a direct path if planning fails
            direct_path = [request.start_position, request.target_position]
            
            distance = self._calculate_direct_distance(
                request.start_position, 
                request.target_position
            )
            time_estimate = self._estimate_travel_time(distance, request.robot_capabilities)
            
            safety_metrics = {
                'obstacle_clearance': 0.0,
                'path_complexity': 1.0,
                'collision_risk': 1.0
            }
            
            return PathPlanningResult(
                path=direct_path,
                estimated_distance=distance,
                estimated_time=time_estimate,
                safety_metrics=safety_metrics
            )
    
    def _generate_path(self, start: Dict[str, float], end: Dict[str, float]) -> List[Dict[str, float]]:
        """
        Generate a simple path between two points (simulated implementation).
        
        Args:
            start: Start position
            end: End position
            
        Returns:
            List of waypoints forming the path
        """
        # This is a simplified path generation - in reality, this would call
        # a proper path planning algorithm considering obstacles
        path = []
        
        # Create a straight line path with intermediate waypoints
        steps = max(2, int(self._calculate_direct_distance(start, end) / self.path_resolution))
        
        for i in range(steps + 1):
            ratio = i / steps
            waypoint = {
                'x': start['x'] + (end['x'] - start['x']) * ratio,
                'y': start['y'] + (end['y'] - start['y']) * ratio,
                'z': start['z'] + (end['z'] - start['z']) * ratio if 'z' in start and 'z' in end else 0.0
            }
            path.append(waypoint)
        
        return path
    
    def _calculate_path_distance(self, path: List[Dict[str, float]]) -> float:
        """
        Calculate the total distance of a path.
        
        Args:
            path: List of waypoints
            
        Returns:
            Total distance in meters
        """
        if len(path) < 2:
            return 0.0
        
        total_distance = 0.0
        for i in range(1, len(path)):
            segment_distance = self._calculate_direct_distance(path[i-1], path[i])
            total_distance += segment_distance
        
        return total_distance
    
    def _calculate_direct_distance(self, pos1: Dict[str, float], pos2: Dict[str, float]) -> float:
        """
        Calculate direct distance between two points.
        
        Args:
            pos1: First position
            pos2: Second position
            
        Returns:
            Distance in meters
        """
        dx = pos2['x'] - pos1['x']
        dy = pos2['y'] - pos1['y']
        dz = pos2.get('z', 0) - pos1.get('z', 0)  # Default to 0 if z is not provided
        
        return (dx*dx + dy*dy + dz*dz)**0.5
    
    def _estimate_travel_time(self, distance: float, robot_capabilities: List[str]) -> float:
        """
        Estimate travel time based on distance and robot capabilities.
        
        Args:
            distance: Distance to travel in meters
            robot_capabilities: List of robot capabilities
            
        Returns:
            Estimated travel time in seconds
        """
        # Base speed assumption (can be customized based on capabilities)
        base_speed = 0.5  # m/s
        
        # Adjust speed based on robot capabilities
        if 'fast_navigation' in robot_capabilities:
            base_speed *= 1.5
        elif 'slow_navigation' in robot_capabilities:
            base_speed *= 0.7
        
        # Add time for turns and maneuvers
        maneuver_time_factor = 0.2  # 20% extra for maneuvers
        
        base_time = distance / base_speed
        total_time = base_time * (1 + maneuver_time_factor)
        
        return total_time
    
    @log_exception()
    async def create_navigation_augmented_plan(
        self, 
        command: str, 
        robot_state: RobotStateModel
    ) -> CognitivePlanModel:
        """
        Create a cognitive plan with integrated path planning information.
        
        Args:
            command: Natural language command
            robot_state: Current RobotStateModel
            
        Returns:
            CognitivePlanModel enhanced with path information
        """
        try:
            self.logger.info(f"Creating navigation-augmented plan for command: {command}")
            
            # First, create cognitive plan without path information
            voice_command = VoiceCommandModel.create(transcript=command, user_id="system")
            cognitive_plan = await self.cognitive_planner.plan(voice_command)
            
            if not cognitive_plan:
                raise VLAException(
                    f"Failed to create cognitive plan for command: {command}", 
                    VLAErrorType.PLANNING_ERROR
                )
            
            # Then, integrate path planning information
            augmented_plan = await self.integrate_path_planning_with_cognitive_plan(
                cognitive_plan, 
                robot_state
            )
            
            self.logger.info(f"Created navigation-augmented plan: {augmented_plan.plan_id}")
            return augmented_plan
            
        except Exception as e:
            self.logger.error(f"Error creating navigation-augmented plan: {e}")
            raise VLAException(
                f"Navigation-augmented planning error: {str(e)}", 
                VLAErrorType.PLANNING_ERROR,
                e
            )
    
    def get_path_safety_metrics(self, path: List[Dict[str, float]], environment_map: Dict[str, Any]) -> Dict[str, float]:
        """
        Calculate safety metrics for a path based on environmental data.
        
        Args:
            path: List of waypoints representing the path
            environment_map: Current environment information
            
        Returns:
            Dictionary with safety metrics
        """
        # In a real implementation, this would use the environment map to check
        # for obstacles along the path and calculate safety metrics
        
        # For now, return simulated safety metrics
        return {
            'average_clearance': 0.8,  # Average distance to obstacles
            'min_clearance': 0.3,      # Minimum distance to obstacles
            'obstacle_density': 0.2,   # Fraction of path near obstacles
            'risk_score': 0.15         # Overall risk score (0-1)
        }


# Global path-cognitive integrator instance
_path_integrator = None


def get_path_integrator() -> PathCognitiveIntegrator:
    """Get the global path-cognitive integrator instance."""
    global _path_integrator
    if _path_integrator is None:
        _path_integrator = PathCognitiveIntegrator()
    return _path_integrator


async def integrate_path_planning_with_plan(
    cognitive_plan: CognitivePlanModel, 
    robot_state: RobotStateModel
) -> CognitivePlanModel:
    """Convenience function to integrate path planning with a cognitive plan."""
    integrator = get_path_integrator()
    return await integrator.integrate_path_planning_with_cognitive_plan(cognitive_plan, robot_state)


async def create_navigation_augmented_plan(
    command: str, 
    robot_state: RobotStateModel
) -> CognitivePlanModel:
    """Convenience function to create a navigation-augmented cognitive plan."""
    integrator = get_path_integrator()
    return await integrator.create_navigation_augmented_plan(command, robot_state)


def get_path_safety_metrics(path: List[Dict[str, float]], environment_map: Dict[str, Any]) -> Dict[str, float]:
    """Convenience function to get path safety metrics."""
    integrator = get_path_integrator()
    return integrator.get_path_safety_metrics(path, environment_map)