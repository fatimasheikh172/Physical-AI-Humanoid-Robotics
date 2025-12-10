"""
Manipulation Controller for the Vision-Language-Action (VLA) module.

This module handles fine-grained manipulation tasks including pick-and-place
operations, grasping, and object manipulation using the robot's end effector.
"""

import asyncio
import logging
from typing import Dict, Any, Optional, List
from dataclasses import dataclass
import math

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import Action, ActionResponse, ExecutionStatus
from .robot_controller import RobotController, get_robot_controller


@dataclass
class GraspConfig:
    """Configuration for grasping an object."""
    approach_direction: Dict[str, float]  # x, y, z vector
    grasp_width: float  # in meters
    grasp_force: float  # in Newtons
    grasp_type: str  # 'pinch', 'wrap', 'suction', etc.
    clearance_distance: float = 0.05  # in meters


@dataclass
class ManipulationConfig:
    """Configuration for manipulation tasks."""
    max_force: float  # Maximum force allowed during manipulation
    max_velocity: float  # Maximum velocity during manipulation
    max_acceleration: float  # Maximum acceleration during manipulation
    force_tolerance: float  # Tolerance for force control
    position_tolerance: float  # Tolerance for positioning accuracy


class ManipulationController:
    """
    Controller for robot manipulation tasks such as pick-and-place, grasping, etc.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Initialize robot controller
        self.robot_controller = get_robot_controller()
        
        # Manipulation configurations
        self.manipulation_config = self._load_manipulation_config()
        
        # Active manipulation tasks
        self.active_manipulations = {}
        
        # Object properties database (in a real system, this would be more sophisticated)
        self.object_properties_db = self._initialize_object_properties_db()
        
        self.logger.info("ManipulationController initialized")
    
    def _load_manipulation_config(self) -> ManipulationConfig:
        """Load manipulation-specific configuration from system config."""
        return ManipulationConfig(
            max_force=getattr(self.config, 'max_manipulation_force', 50.0),
            max_velocity=getattr(self.config, 'max_manipulation_velocity', 0.5),
            max_acceleration=getattr(self.config, 'max_manipulation_acceleration', 0.5),
            force_tolerance=getattr(self.config, 'manipulation_force_tolerance', 5.0),
            position_tolerance=getattr(self.config, 'manipulation_position_tolerance', 0.01)  # 1 cm
        )
    
    def _initialize_object_properties_db(self) -> Dict[str, Dict[str, Any]]:
        """Initialize a database of known object properties."""
        return {
            'cube': {
                'shape': 'cube',
                'dimensions': {'width': 0.05, 'height': 0.05, 'depth': 0.05},  # 5cm cube
                'weight': 0.1,  # 100g
                'material': 'plastic',
                'density': 1000,  # kg/m^3
                'friction_coefficient': 0.5
            },
            'cylinder': {
                'shape': 'cylinder',
                'dimensions': {'radius': 0.025, 'height': 0.08},  # 5cm diameter, 8cm tall
                'weight': 0.15,  # 150g
                'material': 'metal',
                'density': 7850,  # kg/m^3 (steel)
                'friction_coefficient': 0.7
            },
            'ball': {
                'shape': 'sphere',
                'dimensions': {'radius': 0.03},  # 6cm diameter
                'weight': 0.08,  # 80g
                'material': 'rubber',
                'density': 1200,  # kg/m^3
                'friction_coefficient': 0.8
            }
        }
    
    @log_exception()
    async def execute_pick_up_action(self, action: Action) -> ActionResponse:
        """
        Execute a pick-up action.
        
        Args:
            action: Action containing pick-up parameters
            
        Returns:
            ActionResponse with execution results
        """
        try:
            self.logger.info(f"Starting pick-up action: {action.action_id}")
            
            # Extract parameters
            target_object = action.parameters.get('target_object')
            if not target_object:
                raise ValueError("No target object specified for pick-up action")
            
            # Get object properties
            object_name = target_object.get('name', 'unknown_object')
            object_properties = self.object_properties_db.get(object_name.lower())
            if not object_properties:
                self.logger.warning(f"Unknown object type '{object_name}', using generic properties")
                object_properties = {
                    'weight': 0.1,
                    'friction_coefficient': 0.5,
                    'dimensions': {'width': 0.05, 'height': 0.05, 'depth': 0.05}
                }
            
            # Validate payload capacity
            max_payload = getattr(self.config, 'manipulation_payload_capacity', 5.0)
            if object_properties['weight'] > max_payload:
                error_msg = f"Object too heavy: {object_properties['weight']}kg > {max_payload}kg capacity"
                self.logger.error(error_msg)
                return ActionResponse(
                    response_id=f"resp_{action.action_id}",
                    sequence_id=action.parameters.get('sequence_id', 'unknown'),
                    status=ExecutionStatus.FAILED,
                    completion_percentage=0.0,
                    result_summary=error_msg,
                    action_logs=[],
                    timestamp=0.0
                )
            
            # Determine grasp configuration
            grasp_config = self._determine_grasp_for_object(object_properties, target_object)
            
            # Execute approach phase
            approach_successful = await self._execute_approach_phase(target_object, grasp_config)
            if not approach_successful:
                error_msg = "Approach phase failed, cannot proceed with grasp"
                self.logger.error(error_msg)
                return ActionResponse(
                    response_id=f"resp_{action.action_id}",
                    sequence_id=action.parameters.get('sequence_id', 'unknown'),
                    status=ExecutionStatus.FAILED,
                    completion_percentage=0.3,  # 30% complete
                    result_summary=error_msg,
                    action_logs=[],
                    timestamp=0.0
                )
            
            # Execute grasp phase
            grasp_successful = await self._execute_grasp_phase(grasp_config)
            if not grasp_successful:
                error_msg = "Grasp phase failed"
                self.logger.error(error_msg)
                # Try to retreat safely
                await self._execute_retreat_phase()
                return ActionResponse(
                    response_id=f"resp_{action.action_id}",
                    sequence_id=action.parameters.get('sequence_id', 'unknown'),
                    status=ExecutionStatus.FAILED,
                    completion_percentage=0.6,  # 60% complete
                    result_summary=error_msg,
                    action_logs=[],
                    timestamp=0.0
                )
            
            # Execute lift phase
            lift_successful = await self._execute_lift_phase()
            if not lift_successful:
                error_msg = "Lift phase failed"
                self.logger.error(error_msg)
                # Try to release and retreat
                await self._execute_release_phase()
                await self._execute_retreat_phase()
                return ActionResponse(
                    response_id=f"resp_{action.action_id}",
                    sequence_id=action.parameters.get('sequence_id', 'unknown'),
                    status=ExecutionStatus.FAILED,
                    completion_percentage=0.8,  # 80% complete
                    result_summary=error_msg,
                    action_logs=[],
                    timestamp=0.0
                )
            
            # Update robot state to reflect holding object
            robot_state = await self.robot_controller.get_robot_state()
            if robot_state:
                robot_state.gripper_state['is_holding'] = True
                robot_state.gripper_state['held_object'] = object_name
                await self.robot_controller.update_robot_state(robot_state)
            
            self.logger.info(f"Pick-up action completed successfully: {action.action_id}")
            return ActionResponse(
                response_id=f"resp_{action.action_id}",
                sequence_id=action.parameters.get('sequence_id', 'unknown'),
                status=ExecutionStatus.COMPLETED,
                completion_percentage=1.0,
                result_summary=f"Successfully picked up {object_name}",
                action_logs=[],
                timestamp=0.0
            )
            
        except Exception as e:
            self.logger.error(f"Error in pick-up action: {e}")
            return ActionResponse(
                response_id=f"resp_{action.action_id}",
                sequence_id=action.parameters.get('sequence_id', 'unknown'),
                status=ExecutionStatus.FAILED,
                completion_percentage=0.0,
                result_summary=f"Error in pick-up action: {str(e)}",
                action_logs=[],
                timestamp=0.0
            )
    
    @log_exception()
    async def execute_place_action(self, action: Action) -> ActionResponse:
        """
        Execute a place action.
        
        Args:
            action: Action containing place parameters
            
        Returns:
            ActionResponse with execution results
        """
        try:
            self.logger.info(f"Starting place action: {action.action_id}")
            
            # Check if robot is holding an object
            robot_state = await self.robot_controller.get_robot_state()
            if robot_state and not robot_state.gripper_state.get('is_holding', False):
                error_msg = "Cannot place - robot is not holding any object"
                self.logger.error(error_msg)
                return ActionResponse(
                    response_id=f"resp_{action.action_id}",
                    sequence_id=action.parameters.get('sequence_id', 'unknown'),
                    status=ExecutionStatus.FAILED,
                    completion_percentage=0.0,
                    result_summary=error_msg,
                    action_logs=[],
                    timestamp=0.0
                )
            
            # Extract placement parameters
            target_location = action.parameters.get('target_location')
            if not target_location:
                raise ValueError("No target location specified for place action")
            
            # Execute approach phase to placement location
            approach_successful = await self._execute_approach_phase({'pose': target_location}, None)
            if not approach_successful:
                error_msg = "Approach to placement location failed"
                self.logger.error(error_msg)
                return ActionResponse(
                    response_id=f"resp_{action.action_id}",
                    sequence_id=action.parameters.get('sequence_id', 'unknown'),
                    status=ExecutionStatus.FAILED,
                    completion_percentage=0.5,  # 50% complete
                    result_summary=error_msg,
                    action_logs=[],
                    timestamp=0.0
                )
            
            # Execute release phase
            release_successful = await self._execute_release_phase()
            if not release_successful:
                error_msg = "Release phase failed"
                self.logger.error(error_msg)
                return ActionResponse(
                    response_id=f"resp_{action.action_id}",
                    sequence_id=action.parameters.get('sequence_id', 'unknown'),
                    status=ExecutionStatus.FAILED,
                    completion_percentage=0.8,  # 80% complete
                    result_summary=error_msg,
                    action_logs=[],
                    timestamp=0.0
                )
            
            # Execute retreat phase
            retreat_successful = await self._execute_retreat_phase()
            if not retreat_successful:
                self.logger.warning("Retreat after placement had issues")
            }
            
            # Update robot state to reflect no longer holding object
            if robot_state:
                robot_state.gripper_state['is_holding'] = False
                robot_state.gripper_state['held_object'] = None
                await self.robot_controller.update_robot_state(robot_state)
            
            held_object = robot_state.gripper_state.get('held_object', 'object') if robot_state else 'object'
            self.logger.info(f"Place action completed successfully: {action.action_id}")
            return ActionResponse(
                response_id=f"resp_{action.action_id}",
                sequence_id=action.parameters.get('sequence_id', 'unknown'),
                status=ExecutionStatus.COMPLETED,
                completion_percentage=1.0,
                result_summary=f"Successfully placed {held_object}",
                action_logs=[],
                timestamp=0.0
            )
            
        except Exception as e:
            self.logger.error(f"Error in place action: {e}")
            return ActionResponse(
                response_id=f"resp_{action.action_id}",
                sequence_id=action.parameters.get('sequence_id', 'unknown'),
                status=ExecutionStatus.FAILED,
                completion_percentage=0.0,
                result_summary=f"Error in place action: {str(e)}",
                action_logs=[],
                timestamp=0.0
            )
    
    @log_exception()
    async def execute_grasp_action(self, action: Action) -> ActionResponse:
        """
        Execute a grasp action without pick-up motion.
        
        Args:
            action: Action containing grasp parameters
            
        Returns:
            ActionResponse with execution results
        """
        try:
            self.logger.info(f"Starting grasp action: {action.action_id}")
            
            # Extract grasp parameters
            grasp_config = self._extract_grasp_config(action)
            
            # Execute the grasp
            grasp_successful = await self._execute_grasp_phase(grasp_config)
            
            if grasp_successful:
                self.logger.info(f"Grasp action completed successfully: {action.action_id}")
                return ActionResponse(
                    response_id=f"resp_{action.action_id}",
                    sequence_id=action.parameters.get('sequence_id', 'unknown'),
                    status=ExecutionStatus.COMPLETED,
                    completion_percentage=1.0,
                    result_summary="Successfully grasped object",
                    action_logs=[],
                    timestamp=0.0
                )
            else:
                error_msg = "Grasp action failed"
                self.logger.error(error_msg)
                return ActionResponse(
                    response_id=f"resp_{action.action_id}",
                    sequence_id=action.parameters.get('sequence_id', 'unknown'),
                    status=ExecutionStatus.FAILED,
                    completion_percentage=0.0,
                    result_summary=error_msg,
                    action_logs=[],
                    timestamp=0.0
                )
            
        except Exception as e:
            self.logger.error(f"Error in grasp action: {e}")
            return ActionResponse(
                response_id=f"resp_{action.action_id}",
                sequence_id=action.parameters.get('sequence_id', 'unknown'),
                status=ExecutionStatus.FAILED,
                completion_percentage=0.0,
                result_summary=f"Error in grasp action: {str(e)}",
                action_logs=[],
                timestamp=0.0
            )
    
    @log_exception()
    async def execute_release_action(self, action: Action) -> ActionResponse:
        """
        Execute a release action.
        
        Args:
            action: Action containing release parameters
            
        Returns:
            ActionResponse with execution results
        """
        try:
            self.logger.info(f"Starting release action: {action.action_id}")
            
            # Extract release parameters
            release_method = action.parameters.get('release_method', 'open_gripper')
            
            # Execute the release
            release_successful = await self._execute_release_phase(release_method)
            
            if release_successful:
                self.logger.info(f"Release action completed successfully: {action.action_id}")
                
                # Update robot state to reflect no longer holding object
                robot_state = await self.robot_controller.get_robot_state()
                if robot_state:
                    robot_state.gripper_state['is_holding'] = False
                    robot_state.gripper_state['held_object'] = None
                    await self.robot_controller.update_robot_state(robot_state)
                
                return ActionResponse(
                    response_id=f"resp_{action.action_id}",
                    sequence_id=action.parameters.get('sequence_id', 'unknown'),
                    status=ExecutionStatus.COMPLETED,
                    completion_percentage=1.0,
                    result_summary="Successfully released object",
                    action_logs=[],
                    timestamp=0.0
                )
            else:
                error_msg = "Release action failed"
                self.logger.error(error_msg)
                return ActionResponse(
                    response_id=f"resp_{action.action_id}",
                    sequence_id=action.parameters.get('sequence_id', 'unknown'),
                    status=ExecutionStatus.FAILED,
                    completion_percentage=0.0,
                    result_summary=error_msg,
                    action_logs=[],
                    timestamp=0.0
                )
            
        except Exception as e:
            self.logger.error(f"Error in release action: {e}")
            return ActionResponse(
                response_id=f"resp_{action.action_id}",
                sequence_id=action.parameters.get('sequence_id', 'unknown'),
                status=ExecutionStatus.FAILED,
                completion_percentage=0.0,
                result_summary=f"Error in release action: {str(e)}",
                action_logs=[],
                timestamp=0.0
            )
    
    def _determine_grasp_for_object(self, object_properties: Dict[str, Any], target_object: Dict[str, Any]) -> GraspConfig:
        """
        Determine appropriate grasp configuration for an object.
        
        Args:
            object_properties: Properties of the object to grasp
            target_object: Target object specification
            
        Returns:
            GraspConfig with appropriate settings
        """
        shape = object_properties.get('shape', 'unknown')
        
        # Calculate appropriate grasp width based on object dimensions
        dimensions = object_properties.get('dimensions', {})
        
        if shape == 'cube':
            # For cubes, approach from the top with a pinch grasp
            width = min(dimensions.get('width', 0.05), dimensions.get('depth', 0.05)) * 0.8  # 80% of smallest dimension
            approach_dir = {'x': 0.0, 'y': 0.0, 'z': -1.0}  # Approach from above
            grasp_type = 'pinch'
        elif shape == 'cylinder':
            # For cylinders, approach along the side with wrap grasp
            width = dimensions.get('radius', 0.025) * 2.0 * 1.1  # 110% of diameter
            approach_dir = {'x': 0.0, 'y': -1.0, 'z': 0.0}  # Approach from the side
            grasp_type = 'wrap'
        elif shape == 'sphere':
            # For spheres, approach from any direction with pinch grasp
            width = dimensions.get('radius', 0.03) * 2.0 * 0.9  # 90% of diameter
            approach_dir = {'x': 0.0, 'y': 0.0, 'z': -1.0}  # Approach from above
            grasp_type = 'pinch'
        else:
            # Generic grasp for unknown shapes
            width = 0.05  # Default 5cm
            approach_dir = {'x': 0.0, 'y': 0.0, 'z': -1.0}
            grasp_type = 'pinch'
        
        # Calculate appropriate force based on object weight and friction
        weight = object_properties.get('weight', 0.1)  # Default to 100g
        friction = object_properties.get('friction_coefficient', 0.5)
        # Force should be somewhat higher than the weight to ensure grip
        force = min(weight * 9.81 * (1.5 + friction), self.manipulation_config.max_force * 0.8)
        
        return GraspConfig(
            approach_direction=approach_dir,
            grasp_width=width,
            grasp_force=force,
            grasp_type=grasp_type
        )
    
    def _extract_grasp_config(self, action: Action) -> GraspConfig:
        """
        Extract grasp configuration from action parameters.
        
        Args:
            action: Action containing grasp parameters
            
        Returns:
            GraspConfig with extracted settings
        """
        params = action.parameters
        return GraspConfig(
            approach_direction=params.get('approach_direction', {'x': 0, 'y': 0, 'z': -1}),
            grasp_width=params.get('grasp_width', 0.05),
            grasp_force=params.get('grasp_force', 10.0),
            grasp_type=params.get('grasp_type', 'pinch'),
            clearance_distance=params.get('clearance_distance', 0.05)
        )
    
    @log_exception()
    async def _execute_approach_phase(self, target_object: Dict[str, Any], grasp_config: Optional[GraspConfig]) -> bool:
        """
        Execute the approach phase of a manipulation task.
        
        Args:
            target_object: Object to approach
            grasp_config: Grasp configuration for approach
            
        Returns:
            True if successful, False otherwise
        """
        try:
            self.logger.info("Executing approach phase...")
            
            # Calculate approach position and orientation
            target_pose = target_object.get('pose', {'position': {'x': 0, 'y': 0, 'z': 0}})
            position = target_pose.get('position', {'x': 0, 'y': 0, 'z': 0})
            
            # Calculate approach position (slightly above/aside from target)
            approach_pos = {
                'x': position['x'] + grasp_config.approach_direction['x'] * grasp_config.clearance_distance,
                'y': position['y'] + grasp_config.approach_direction['y'] * grasp_config.clearance_distance,
                'z': position['z'] + grasp_config.approach_direction['z'] * grasp_config.clearance_distance
            }
            
            # In a real implementation, this would send move commands to the robot
            # For now, we'll simulate the approach
            await asyncio.sleep(1.0)  # Simulate approach time
            
            self.logger.info("Approach phase completed successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Error in approach phase: {e}")
            return False
    
    @log_exception()
    async def _execute_grasp_phase(self, grasp_config: GraspConfig) -> bool:
        """
        Execute the grasp phase of a manipulation task.
        
        Args:
            grasp_config: Configuration for the grasp
            
        Returns:
            True if successful, False otherwise
        """
        try:
            self.logger.info("Executing grasp phase...")
            
            # Validate grasp parameters against robot capabilities
            if grasp_config.grasp_force > self.manipulation_config.max_force:
                self.logger.error(f"Grasp force {grasp_config.grasp_force}N exceeds maximum {self.manipulation_config.max_force}N")
                return False
            
            if grasp_config.grasp_width > getattr(self.config, 'max_gripper_width', 0.1):
                self.logger.error(f"Grasp width {grasp_config.grasp_width}m exceeds maximum gripper width")
                return False
            
            # In a real implementation, this would send grasp commands to the robot
            # For now, we'll simulate the grasp with a success probability
            await asyncio.sleep(0.8)  # Simulate grasp time
            
            # Simulate success with some probability based on parameters
            import random
            success_prob = 0.95  # High success rate in simulation
            success = random.random() < success_prob
            
            if success:
                self.logger.info("Grasp phase completed successfully")
            else:
                self.logger.error("Grasp phase failed")
            
            return success
            
        except Exception as e:
            self.logger.error(f"Error in grasp phase: {e}")
            return False
    
    @log_exception()
    async def _execute_lift_phase(self) -> bool:
        """
        Execute the lift phase of a pick-up task.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            self.logger.info("Executing lift phase...")
            
            # In a real implementation, this would send lift commands to the robot
            # For now, we'll simulate the lift
            await asyncio.sleep(0.5)  # Simulate lift time
            
            # Simulate success
            self.logger.info("Lift phase completed successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Error in lift phase: {e}")
            return False
    
    @log_exception()
    async def _execute_release_phase(self, method: str = 'open_gripper') -> bool:
        """
        Execute the release phase of a manipulation task.
        
        Args:
            method: Method to use for releasing (default: 'open_gripper')
            
        Returns:
            True if successful, False otherwise
        """
        try:
            self.logger.info("Executing release phase...")
            
            # In a real implementation, this would send release commands to the robot
            # For now, we'll simulate the release
            await asyncio.sleep(0.5)  # Simulate release time
            
            # Simulate success
            self.logger.info("Release phase completed successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Error in release phase: {e}")
            return False
    
    @log_exception()
    async def _execute_retreat_phase(self) -> bool:
        """
        Execute the retreat phase to move away from the object after manipulation.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            self.logger.info("Executing retreat phase...")
            
            # In a real implementation, this would send retreat commands to the robot
            # For now, we'll simulate the retreat
            await asyncio.sleep(0.5)  # Simulate retreat time
            
            # Simulate success
            self.logger.info("Retreat phase completed successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Error in retreat phase: {e}")
            return False
    
    @log_exception()
    async def validate_manipulation_action(self, action: Action) -> List[str]:
        """
        Validate a manipulation action against robot capabilities and constraints.
        
        Args:
            action: Action to validate
            
        Returns:
            List of validation errors or empty list if valid
        """
        errors = []
        
        action_type = action.action_type
        
        if action_type in ['pick_up', 'place', 'grasp', 'release']:
            # Check if robot has manipulation capability
            if 'manipulation' not in self.config.robot_capabilities:
                errors.append("Robot does not have manipulation capability")
        
        if action_type in ['pick_up', 'place']:
            # Validate required parameters
            if action_type == 'pick_up':
                if 'target_object' not in action.parameters:
                    errors.append("pick_up action requires 'target_object' parameter")
                else:
                    target_obj = action.parameters['target_object']
                    if not target_obj.get('name'):
                        errors.append("target_object must have a 'name'")
                    if not target_obj.get('pose'):
                        errors.append("target_object must have a 'pose' with position")
            
            elif action_type == 'place':
                if 'target_location' not in action.parameters:
                    errors.append("place action requires 'target_location' parameter")
                
                # Check if robot is holding an object
                robot_state = await self.robot_controller.get_robot_state()
                if robot_state and not robot_state.gripper_state.get('is_holding', False):
                    if action_type == 'place':
                        errors.append("Cannot place object - robot is not holding anything")
        
        elif action_type in ['grasp', 'release']:
            # These are more advanced manipulation tasks
            if action_type == 'grasp':
                required_params = ['grasp_width', 'grasp_force']
                for param in required_params:
                    if param not in action.parameters:
                        errors.append(f"grasp action requires '{param}' parameter")
        
        # Validate force parameters against robot limits
        if 'grasp_force' in action.parameters:
            force = action.parameters['grasp_force']
            if force > self.manipulation_config.max_force:
                errors.append(f"Grasp force {force}N exceeds maximum of {self.manipulation_config.max_force}N")
        
        # Validate other parameters based on action type
        if action.timeout > self.config.action_execution_timeout:
            errors.append(f"Action timeout {action.timeout}s exceeds maximum of {self.config.action_execution_timeout}s")
        
        return errors


# Global manipulation controller instance
_manipulation_controller = None


def get_manipulation_controller() -> ManipulationController:
    """Get the global manipulation controller instance."""
    global _manipulation_controller
    if _manipulation_controller is None:
        _manipulation_controller = ManipulationController()
    return _manipulation_controller


async def execute_manipulation_action(action: Action) -> ActionResponse:
    """Convenience function to execute a manipulation action."""
    controller = get_manipulation_controller()
    
    if action.action_type == 'pick_up':
        return await controller.execute_pick_up_action(action)
    elif action.action_type == 'place':
        return await controller.execute_place_action(action)
    elif action.action_type == 'grasp':
        return await controller.execute_grasp_action(action)
    elif action.action_type == 'release':
        return await controller.execute_release_action(action)
    else:
        # This is not a manipulation action
        raise ValueError(f"Action type {action.action_type} is not a manipulation action")