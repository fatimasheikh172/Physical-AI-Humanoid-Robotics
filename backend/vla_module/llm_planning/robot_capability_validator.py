"""
Robot capability validation for the Vision-Language-Action (VLA) module.

This module validates that planned actions are within the robot's 
actual capabilities and constraints.
"""

import logging
from typing import Dict, List, Any, Optional
from dataclasses import dataclass
import json

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import RobotState, Action
from ..core.data_models import ActionModel, RobotStateModel


@dataclass
class CapabilityConstraint:
    """A constraint related to a specific robot capability."""
    capability_name: str
    constraint_type: str  # numerical, categorical, boolean
    allowed_values: Optional[List[Any]] = None
    min_value: Optional[float] = None
    max_value: Optional[float] = None
    units: Optional[str] = None
    description: Optional[str] = None


@dataclass
class ValidationResult:
    """Result of a capability validation."""
    is_valid: bool
    errors: List[str]
    warnings: List[str]
    suggested_alternatives: List[str]


class RobotCapabilityValidator:
    """
    Validates that planned actions are within the robot's actual capabilities and constraints.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Robot's actual capabilities and constraints
        self.capabilities = self._initialize_capabilities()
        
        self.logger.info("RobotCapabilityValidator initialized")
    
    def _initialize_capabilities(self) -> Dict[str, List[CapabilityConstraint]]:
        """Initialize the robot's capabilities and associated constraints."""
        return {
            'navigation': [
                CapabilityConstraint(
                    capability_name='max_speed',
                    constraint_type='numerical',
                    min_value=0.0,
                    max_value=getattr(self.config, 'max_navigation_speed', 1.0),
                    units='m/s',
                    description='Maximum speed for navigation'
                ),
                CapabilityConstraint(
                    capability_name='max_distance',
                    constraint_type='numerical',
                    min_value=0.0,
                    max_value=getattr(self.config, 'max_navigation_distance', 10.0),
                    units='m',
                    description='Maximum distance for navigation'
                ),
                CapabilityConstraint(
                    capability_name='min_turn_radius',
                    constraint_type='numerical',
                    min_value=0.05,
                    max_value=0.5,
                    units='m',
                    description='Minimum turning radius'
                )
            ],
            'manipulation': [
                CapabilityConstraint(
                    capability_name='max_force',
                    constraint_type='numerical',
                    min_value=0.0,
                    max_value=getattr(self.config, 'max_manipulation_force', 50.0),
                    units='N',
                    description='Maximum force that can be applied during manipulation'
                ),
                CapabilityConstraint(
                    capability_name='payload_capacity',
                    constraint_type='numerical',
                    min_value=0.0,
                    max_value=getattr(self.config, 'manipulation_payload_capacity', 5.0),
                    units='kg',
                    description='Maximum payload capacity for manipulation'
                ),
                CapabilityConstraint(
                    capability_name='reachable_workspace',
                    constraint_type='categorical',
                    allowed_values=['front', 'side', 'above', 'below'],
                    description='Workspace areas reachable by manipulator'
                )
            ],
            'perception': [
                CapabilityConstraint(
                    capability_name='max_detection_range',
                    constraint_type='numerical',
                    min_value=0.1,
                    max_value=getattr(self.config, 'max_perception_range', 5.0),
                    units='m',
                    description='Maximum distance for object detection'
                ),
                CapabilityConstraint(
                    capability_name='detection_accuracy',
                    constraint_type='numerical',
                    min_value=0.7,
                    max_value=1.0,
                    description='Minimum accuracy for positive object detection'
                )
            ],
            'locomotion': [
                CapabilityConstraint(
                    capability_name='max_climb_angle',
                    constraint_type='numerical',
                    min_value=0.0,
                    max_value=getattr(self.config, 'max_climb_angle', 15.0),
                    units='degrees',
                    description='Maximum angle robot can climb'
                ),
                CapabilityConstraint(
                    capability_name='max_step_height',
                    constraint_type='numerical',
                    min_value=0.01,
                    max_value=getattr(self.config, 'max_step_height', 0.15),
                    units='m',
                    description='Maximum step height the robot can handle'
                )
            ],
            'battery': [
                CapabilityConstraint(
                    capability_name='min_battery_threshold',
                    constraint_type='numerical',
                    min_value=0.1,
                    max_value=0.3,
                    description='Minimum battery level for safe operation'
                )
            ]
        }
    
    @log_exception()
    def validate_action_against_capabilities(self, action: ActionModel, robot_state: RobotStateModel) -> ValidationResult:
        """
        Validate that an action is within the robot's capabilities.
        
        Args:
            action: ActionModel to validate
            robot_state: Current RobotStateModel for context
            
        Returns:
            ValidationResult indicating if action is valid
        """
        errors = []
        warnings = []
        alternatives = []
        
        try:
            action_type = action.action_type
            
            # Check if the action type is supported by the robot
            if not self._is_action_type_supported(action_type):
                errors.append(f"Action type '{action_type}' is not supported by this robot")
                # Suggest alternatives if possible
                alternatives.extend(self._suggest_alternative_actions(action_type))
            
            # Validate specific action constraints
            action_errors, action_warnings, action_alternatives = self._validate_action_specific_constraints(
                action, robot_state
            )
            
            errors.extend(action_errors)
            warnings.extend(action_warnings)
            alternatives.extend(action_alternatives)
            
            # Check battery constraints
            if robot_state.battery_level < self.config.robot_type.get('min_battery_threshold', 0.2):
                warnings.append(f"Battery level ({robot_state.battery_level:.2%}) is below recommended threshold")
            
            # Check environment constraints if available
            if hasattr(robot_state, 'position') and action.parameters.get('target_location'):
                env_errors = self._validate_environment_constraints(
                    robot_state.position,
                    action.parameters['target_location']
                )
                errors.extend(env_errors)
            
            is_valid = len(errors) == 0
            
            return ValidationResult(
                is_valid=is_valid,
                errors=errors,
                warnings=warnings,
                suggested_alternatives=alternatives
            )
            
        except Exception as e:
            self.logger.error(f"Error validating action against capabilities: {e}")
            return ValidationResult(
                is_valid=False,
                errors=[f"Error validating action: {str(e)}"],
                warnings=[],
                suggested_alternatives=[]
            )
    
    @log_exception()
    def validate_plan_against_capabilities(self, actions: List[ActionModel], robot_state: RobotStateModel) -> ValidationResult:
        """
        Validate that a sequence of actions is within the robot's capabilities.
        
        Args:
            actions: List of ActionModels to validate
            robot_state: Current RobotStateModel for context
            
        Returns:
            ValidationResult indicating if plan is valid
        """
        all_errors = []
        all_warnings = []
        all_alternatives = []
        
        for i, action in enumerate(actions):
            result = self.validate_action_against_capabilities(action, robot_state)
            # Prefix errors/warnings with action index
            prefixed_errors = [f"Action {i+1} ({action.action_id}): {error}" for error in result.errors]
            prefixed_warnings = [f"Action {i+1} ({action.action_id}): {warning}" for warning in result.warnings]
            
            all_errors.extend(prefixed_errors)
            all_warnings.extend(prefixed_warnings)
            all_alternatives.extend(result.suggested_alternatives)
        
        return ValidationResult(
            is_valid=len(all_errors) == 0,
            errors=all_errors,
            warnings=all_warnings,
            suggested_alternatives=all_alternatives
        )
    
    def _is_action_type_supported(self, action_type: str) -> bool:
        """Check if the robot supports a specific action type."""
        # Map action types to capability groups
        action_to_capability = {
            'move_to': 'navigation',
            'navigate': 'navigation',
            'pick_up': 'manipulation',
            'place': 'manipulation',
            'grasp': 'manipulation',
            'release': 'manipulation',
            'detect_object': 'perception',
            'rotate': 'locomotion'
        }
        
        capability_group = action_to_capability.get(action_type)
        if capability_group is None:
            # Action type not mapped, assume unsupported
            return False
        
        return capability_group in self.capabilities
    
    def _validate_action_specific_constraints(self, action: ActionModel, robot_state: RobotStateModel) -> tuple[List[str], List[str], List[str]]:
        """Validate action-specific constraints."""
        errors = []
        warnings = []
        alternatives = []
        
        action_type = action.action_type
        
        if action_type in ['move_to', 'navigate']:
            # Validate navigation constraints
            errors.extend(self._validate_navigation_constraints(action, robot_state))
        
        elif action_type in ['pick_up', 'place', 'grasp', 'release']:
            # Validate manipulation constraints
            errors.extend(self._validate_manipulation_constraints(action, robot_state))
        
        elif action_type == 'detect_object':
            # Validate perception constraints
            errors.extend(self._validate_perception_constraints(action, robot_state))
        
        elif action_type == 'rotate':
            # Validate locomotion constraints
            errors.extend(self._validate_locomotion_constraints(action, robot_state))
        
        return errors, warnings, alternatives
    
    def _validate_navigation_constraints(self, action: ActionModel, robot_state: RobotStateModel) -> List[str]:
        """Validate navigation-specific constraints."""
        errors = []
        
        target_location = action.parameters.get('target_location')
        if target_location:
            # Check maximum distance
            max_distance_constraint = None
            for constraint in self.capabilities.get('navigation', []):
                if constraint.capability_name == 'max_distance':
                    max_distance_constraint = constraint
                    break
            
            if max_distance_constraint:
                # Calculate distance from current position to target
                if hasattr(robot_state, 'position'):
                    current_pos = robot_state.position
                    target_pos = target_location
                    
                    distance = (
                        (target_pos.get('x', 0) - current_pos.get('x', 0)) ** 2 +
                        (target_pos.get('y', 0) - current_pos.get('y', 0)) ** 2 +
                        (target_pos.get('z', 0) - current_pos.get('z', 0)) ** 2
                    ) ** 0.5
                    
                    if distance > max_distance_constraint.max_value:
                        errors.append(
                            f"Navigation target too far: {distance:.2f}{max_distance_constraint.units} "
                            f"exceeds maximum of {max_distance_constraint.max_value}{max_distance_constraint.units}"
                        )
                
            # Check speed constraints if provided
            requested_speed = action.parameters.get('speed')
            if requested_speed:
                max_speed_constraint = None
                for constraint in self.capabilities.get('navigation', []):
                    if constraint.capability_name == 'max_speed':
                        max_speed_constraint = constraint
                        break
                
                if max_speed_constraint and requested_speed > max_speed_constraint.max_value:
                    errors.append(
                        f"Requested speed {requested_speed}{max_speed_constraint.units} "
                        f"exceeds maximum of {max_speed_constraint.max_value}{max_speed_constraint.units}"
                    )
        
        return errors
    
    def _validate_manipulation_constraints(self, action: ActionModel, robot_state: RobotStateModel) -> List[str]:
        """Validate manipulation-specific constraints."""
        errors = []
        
        # Check object weight if picking up
        if action.action_type == 'pick_up':
            target_object = action.parameters.get('target_object', {})
            object_weight = target_object.get('weight', 0)
            
            payload_constraint = None
            for constraint in self.capabilities.get('manipulation', []):
                if constraint.capability_name == 'payload_capacity':
                    payload_constraint = constraint
                    break
            
            if payload_constraint and object_weight > payload_constraint.max_value:
                errors.append(
                    f"Object too heavy: {object_weight}{payload_constraint.units} "
                    f"exceeds maximum capacity of {payload_constraint.max_value}{payload_constraint.units}"
                )
        
        # Check force constraints
        requested_force = action.parameters.get('force_limit')
        if requested_force:
            force_constraint = None
            for constraint in self.capabilities.get('manipulation', []):
                if constraint.capability_name == 'max_force':
                    force_constraint = constraint
                    break
            
            if force_constraint and requested_force > force_constraint.max_value:
                errors.append(
                    f"Requested force {requested_force}{force_constraint.units} "
                    f"exceeds maximum of {force_constraint.max_value}{force_constraint.units}"
                )
        
        return errors
    
    def _validate_perception_constraints(self, action: ActionModel, robot_state: RobotStateModel) -> List[str]:
        """Validate perception-specific constraints."""
        errors = []
        
        # Check detection range if specified
        target_distance = action.parameters.get('max_detection_range')
        if target_distance:
            range_constraint = None
            for constraint in self.capabilities.get('perception', []):
                if constraint.capability_name == 'max_detection_range':
                    range_constraint = constraint
                    break
            
            if range_constraint and target_distance > range_constraint.max_value:
                errors.append(
                    f"Requested detection range {target_distance}{range_constraint.units} "
                    f"exceeds maximum of {range_constraint.max_value}{range_constraint.units}"
                )
        
        return errors
    
    def _validate_locomotion_constraints(self, action: ActionModel, robot_state: RobotStateModel) -> List[str]:
        """Validate locomotion-specific constraints."""
        errors = []
        
        # For rotation actions, check turn radius constraints
        if action.action_type == 'rotate':
            angle = action.parameters.get('angle', 0)
            
            # Check if angle is too tight based on min turn radius
            turn_radius_constraint = None
            for constraint in self.capabilities.get('navigation', []):
                if constraint.capability_name == 'min_turn_radius':
                    turn_radius_constraint = constraint
                    break
            
            # In a more detailed implementation, validate against physical constraints
        
        return errors
    
    def _validate_environment_constraints(self, current_pos: Dict[str, float], target_pos: Dict[str, float]) -> List[str]:
        """Validate constraints related to environment."""
        errors = []
        
        # In a real implementation, this would check against environment map
        # and other environmental constraints
        
        return errors
    
    def _suggest_alternative_actions(self, action_type: str) -> List[str]:
        """Suggest alternative actions if the requested action is not supported."""
        alternatives = {
            'move_to': ['navigate', 'go_to'],
            'navigate': ['move_to', 'go_to'],
            'pick_up': ['grasp', 'take'],
            'place': ['put', 'set_down'],
            'grasp': ['pick_up', 'grab'],
            'release': ['let_go', 'drop'],
            'detect_object': ['find_object', 'locate_object'],
            'rotate': ['turn', 'pivot']
        }
        
        return alternatives.get(action_type, [])
    
    def get_robot_capabilities_summary(self) -> Dict[str, Any]:
        """Get a summary of the robot's capabilities."""
        summary = {}
        
        for capability_group, constraints in self.capabilities.items():
            summary[capability_group] = [f"{c.capability_name}: {c.description}" for c in constraints]
        
        return summary


# Global instance
_capability_validator = None


def get_capability_validator() -> RobotCapabilityValidator:
    """Get the global robot capability validator instance."""
    global _capability_validator
    if _capability_validator is None:
        _capability_validator = RobotCapabilityValidator()
    return _capability_validator


def validate_action_against_capabilities(action: ActionModel, robot_state: RobotStateModel) -> ValidationResult:
    """Convenience function to validate an action against robot capabilities."""
    validator = get_capability_validator()
    return validator.validate_action_against_capabilities(action, robot_state)