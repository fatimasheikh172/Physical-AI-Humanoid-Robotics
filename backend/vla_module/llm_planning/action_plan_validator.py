"""
Action plan validation and safety checking for the Vision-Language-Action (VLA) module.

This module validates that action plans are safe, feasible, and within robot constraints.
"""

import logging
from typing import Dict, List, Any, Optional
from dataclasses import dataclass

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import CognitivePlan, ActionSequence
from ..core.data_models import ActionSequenceModel, CognitivePlanModel
from .robot_capability_validator import RobotCapabilityValidator, get_capability_validator


@dataclass
class SafetyCheckResult:
    """Result of a safety check."""
    is_safe: bool
    issues: List[str]
    recommendations: List[str]
    severity: str  # 'low', 'medium', 'high', 'critical'


class ActionPlanValidator:
    """
    Validates action plans for safety and feasibility.
    Performs both static analysis and dynamic validation against robot state.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Initialize validators
        self.capability_validator = get_capability_validator()
        
        # Safety thresholds
        self.collision_proximity_threshold = 0.5  # meters
        self.force_threshold_multiplier = 0.8     # 80% of max force
        self.navigation_altitude_threshold = 1.0  # maximum altitude change in m for a single movement
        
        self.logger.info("ActionPlanValidator initialized")
    
    @log_exception()
    def validate_cognitive_plan(self, cognitive_plan: CognitivePlanModel) -> List[str]:
        """
        Validate a cognitive plan for safety and feasibility.
        
        Args:
            cognitive_plan: CognitivePlanModel to validate
            
        Returns:
            List of validation errors or empty list if valid
        """
        try:
            self.logger.info(f"Validating cognitive plan: {cognitive_plan.plan_id}")
            
            errors = []
            
            # Validate plan structure
            plan_errors = self._validate_plan_structure(cognitive_plan)
            errors.extend(plan_errors)
            
            # Validate individual tasks
            task_errors = self._validate_tasks_in_plan(cognitive_plan)
            errors.extend(task_errors)
            
            # Validate safety constraints
            safety_errors = self._validate_plan_safety(cognitive_plan)
            errors.extend(safety_errors)
            
            # Check plan consistency
            consistency_errors = self._validate_plan_consistency(cognitive_plan)
            errors.extend(consistency_errors)
            
            # Validate against expected execution context
            context_errors = self._validate_plan_execution_context(cognitive_plan)
            errors.extend(context_errors)
            
            if errors:
                self.logger.warning(f"Cognitive plan validation found {len(errors)} errors: {errors[:3]}{'...' if len(errors) > 3 else ''}")
            else:
                self.logger.info("Cognitive plan validation passed")
            
            return errors
            
        except Exception as e:
            self.logger.error(f"Error validating cognitive plan: {e}")
            raise VLAException(
                f"Cognitive plan validation error: {str(e)}", 
                VLAErrorType.VALIDATION_ERROR,
                e
            )
    
    @log_exception()
    def validate_action_sequence(self, action_sequence: ActionSequenceModel, robot_state: Optional['RobotStateModel'] = None) -> List[str]:
        """
        Validate an action sequence for safety and feasibility.
        
        Args:
            action_sequence: ActionSequenceModel to validate
            robot_state: Optional RobotStateModel for context
            
        Returns:
            List of validation errors or empty list if valid
        """
        try:
            self.logger.info(f"Validating action sequence: {action_sequence.sequence_id}")
            
            errors = []
            
            # Validate sequence structure
            sequence_errors = self._validate_sequence_structure(action_sequence)
            errors.extend(sequence_errors)
            
            # Validate each action against robot capabilities
            if robot_state:
                capability_errors = self.capability_validator.validate_plan_against_capabilities(
                    action_sequence.actions, 
                    robot_state
                ).errors
                errors.extend(capability_errors)
            else:
                # If no robot state is available, at least validate the basic action structure
                for i, action in enumerate(action_sequence.actions):
                    basic_errors = self._validate_action_structure(action, i)
                    errors.extend(basic_errors)
            
            # Check for safety issues in the sequence
            safety_errors = self._validate_sequence_safety(action_sequence, robot_state)
            errors.extend(safety_errors)
            
            # Validate dependencies between actions
            dependency_errors = self._validate_action_dependencies(action_sequence)
            errors.extend(dependency_errors)
            
            # Check for resource conflicts
            resource_errors = self._validate_resource_conflicts(action_sequence)
            errors.extend(resource_errors)
            
            if errors:
                self.logger.warning(f"Action sequence validation found {len(errors)} errors: {errors[:3]}{'...' if len(errors) > 3 else ''}")
            else:
                self.logger.info("Action sequence validation passed")
            
            return errors
            
        except Exception as e:
            self.logger.error(f"Error validating action sequence: {e}")
            raise VLAException(
                f"Action sequence validation error: {str(e)}", 
                VLAErrorType.VALIDATION_ERROR,
                e
            )
    
    def _validate_plan_structure(self, cognitive_plan: CognitivePlanModel) -> List[str]:
        """Validate the structure of a cognitive plan."""
        errors = []
        
        if not cognitive_plan.plan_id:
            errors.append("Plan missing plan_id")
        
        if not cognitive_plan.command_id:
            errors.append("Plan missing command_id")
        
        if not cognitive_plan.task_decomposition:
            errors.append("Plan has no tasks decomposed")
        
        if not cognitive_plan.llm_model:
            errors.append("Plan missing LLM model information")
        
        if cognitive_plan.confidence < 0.0 or cognitive_plan.confidence > 1.0:
            errors.append(f"Plan confidence out of bounds: {cognitive_plan.confidence}")
        
        if not cognitive_plan.execution_context:
            errors.append("Plan missing execution context")
        
        return errors
    
    def _validate_tasks_in_plan(self, cognitive_plan: CognitivePlanModel) -> List[str]:
        """Validate the tasks within a cognitive plan."""
        errors = []
        
        for i, task in enumerate(cognitive_plan.task_decomposition):
            try:
                task.validate()
            except ValueError as e:
                errors.append(f"Task {i} validation error: {str(e)}")
        
        return errors
    
    def _validate_plan_safety(self, cognitive_plan: CognitivePlanModel) -> List[str]:
        """Validate safety aspects of a cognitive plan."""
        errors = []
        
        # Check for safety constraints in execution context
        safety_constraints = cognitive_plan.execution_context.get('safety_constraints', {})
        if not safety_constraints:
            errors.append("No safety constraints defined in execution context")
        
        # Check for safety analysis
        safety_analysis = cognitive_plan.to_message_types().execution_context.get('safety_analysis')
        if not safety_analysis:
            self.logger.warning("No safety analysis provided in plan")
        
        # Check if plan attempts to disable safety features
        for task in cognitive_plan.task_decomposition:
            if 'disable_safety' in str(task.parameters).lower():
                errors.append(f"Task {task.task_id} contains request to disable safety features")
        
        return errors
    
    def _validate_plan_consistency(self, cognitive_plan: CognitivePlanModel) -> List[str]:
        """Validate internal consistency of a cognitive plan."""
        errors = []
        
        # Check for conflicting constraints
        constraints = cognitive_plan.execution_context.get('constraints', {})
        
        # Example consistency checks
        if 'min_time' in constraints and 'max_time' in constraints:
            if constraints['min_time'] > constraints['max_time']:
                errors.append("Plan has inconsistent time constraints: min_time > max_time")
        
        return errors
    
    def _validate_plan_execution_context(self, cognitive_plan: CognitivePlanModel) -> List[str]:
        """Validate the execution context of a cognitive plan."""
        errors = []
        
        context = cognitive_plan.execution_context
        
        # Check environment map validity
        env_map = context.get('environment_map')
        if env_map:
            # Basic validation - this would be more sophisticated in practice
            if not isinstance(env_map, dict):
                errors.append("Environment map is not a dictionary")
        
        # Check robot capabilities
        robot_caps = context.get('robot_capabilities')
        if not robot_caps:
            errors.append("No robot capabilities specified in execution context")
        
        return errors
    
    def _validate_sequence_structure(self, action_sequence: ActionSequenceModel) -> List[str]:
        """Validate the structure of an action sequence."""
        errors = []
        
        if not action_sequence.sequence_id:
            errors.append("Action sequence missing sequence_id")
        
        if not action_sequence.plan_id:
            errors.append("Action sequence missing plan_id")
        
        if not action_sequence.actions:
            errors.append("Action sequence has no actions")
        
        # Check for valid execution status
        valid_statuses = ['PENDING', 'VALIDATED', 'APPROVED', 'EXECUTING', 'COMPLETED', 'FAILED', 'PAUSED', 'CANCELLED']
        if action_sequence.execution_status.name not in valid_statuses:
            errors.append(f"Invalid execution status: {action_sequence.execution_status}")
        
        return errors
    
    def _validate_action_structure(self, action: 'ActionModel', index: int) -> List[str]:
        """Validate the structure of a single action."""
        errors = []
        
        if not action.action_id:
            errors.append(f"Action {index} missing action_id")
        
        if not action.action_type:
            errors.append(f"Action {index} missing action_type")
        
        if not isinstance(action.parameters, dict):
            errors.append(f"Action {index} parameters not in dictionary format")
        
        if action.timeout <= 0:
            errors.append(f"Action {index} has invalid timeout: {action.timeout}")
        
        if action.retry_count < 0:
            errors.append(f"Action {index} has invalid retry_count: {action.retry_count}")
        
        return errors
    
    def _validate_sequence_safety(self, action_sequence: ActionSequenceModel, robot_state: Optional['RobotStateModel']) -> List[str]:
        """Validate safety aspects of an action sequence."""
        errors = []
        
        for i, action in enumerate(action_sequence.actions):
            action_errors = self._check_action_safety(action, i, robot_state)
            errors.extend(action_errors)
        
        # Check sequence-wide safety issues
        sequence_safety_errors = self._check_sequence_wide_safety(action_sequence, robot_state)
        errors.extend(sequence_safety_errors)
        
        return errors
    
    def _check_action_safety(self, action: 'ActionModel', index: int, robot_state: Optional['RobotStateModel']) -> List[str]:
        """Check safety aspects of a single action."""
        errors = []
        
        action_type = action.action_type
        params = action.parameters
        
        # Check for dangerous action types
        if action_type.lower() in ['destroy', 'damage', 'break', 'harm', 'injure']:
            errors.append(f"Action {index} ({action.action_id}) has potentially destructive action type: {action_type}")
        
        # Check navigation safety
        if action_type in ['move_to', 'navigate', 'go_to']:
            target_location = params.get('target_location', {})
            if target_location:
                # Check for potential collision routes
                # This is a simplified check - in reality, this would check against environment map
                if 'x' in target_location and 'y' in target_location:
                    if abs(target_location['x']) > self.config.max_navigation_distance or abs(target_location['y']) > self.config.max_navigation_distance:
                        errors.append(f"Action {index} navigation target too far")
        
        # Check manipulation safety
        if action_type in ['pick_up', 'grasp', 'place', 'release']:
            force_limit = params.get('force_limit')
            if force_limit:
                # Check against robot's force capabilities
                max_force = getattr(self.config, 'max_manipulation_force', 50.0)  # Default value
                safe_limit = max_force * self.force_threshold_multiplier
                
                if force_limit > safe_limit:
                    errors.append(f"Action {index} requested force limit ({force_limit}N) exceeds safe threshold ({safe_limit}N)")
        
        return errors
    
    def _check_sequence_wide_safety(self, action_sequence: ActionSequenceModel, robot_state: Optional['RobotStateModel']) -> List[str]:
        """Check safety aspects across the entire action sequence."""
        errors = []
        
        # Check for consecutive dangerous actions without safety checks
        dangerous_action_count = 0
        for i, action in enumerate(action_sequence.actions):
            if action.action_type.lower() in ['move_to', 'navigate'] and i > 0:
                # Check if previous action was also navigation without intermediate check
                prev_action = action_sequence.actions[i-1]
                if prev_action.action_type.lower() in ['move_to', 'navigate']:
                    dangerous_action_count += 1
                    if dangerous_action_count >= 3:
                        errors.append(f"Too many consecutive navigation actions without safety checks starting at action {i-2}")
                else:
                    dangerous_action_count = 0
            else:
                dangerous_action_count = 0
        
        # Check for rapid force changes
        force_changes = 0
        for action in action_sequence.actions:
            if 'force_limit' in action.parameters:
                # Count rapid changes in force requirements
                force_changes += 1
        if force_changes > len(action_sequence.actions) * 0.5:
            errors.append("Action sequence contains too many force-intensive actions")
        
        return errors
    
    def _validate_action_dependencies(self, action_sequence: ActionSequenceModel) -> List[str]:
        """Validate dependencies between actions."""
        errors = []
        
        # This would validate explicit dependencies if defined
        # For now, we'll just check for basic ordering consistency
        
        # Check if action sequence has any interdependencies that need validation
        # In a real implementation, this would parse dependency metadata and verify
        # that the execution order respects dependencies
        
        return errors
    
    def _validate_resource_conflicts(self, action_sequence: ActionSequenceModel) -> List[str]:
        """Validate that actions don't conflict over resources."""
        errors = []
        
        # Track resource usage
        used_resources = set()
        
        for i, action in enumerate(action_sequence.actions):
            # This is a simplified resource check
            # In a real implementation, this would be more sophisticated
            resource_key = f"resource_{action.action_type}"
            
            # Check if resource is already in use (simplified check)
            if resource_key in used_resources:
                self.logger.warning(f"Potential resource conflict at action {i} using {resource_key}")
            
            # For now, just note the resource
            used_resources.add(resource_key)
        
        return errors
    
    @log_exception()
    def perform_safety_check(self, action_sequence: ActionSequenceModel, robot_state: Optional['RobotStateModel'] = None) -> SafetyCheckResult:
        """
        Perform a comprehensive safety check on an action sequence.
        
        Args:
            action_sequence: ActionSequenceModel to check
            robot_state: Optional RobotStateModel for context
            
        Returns:
            SafetyCheckResult indicating safety status
        """
        try:
            self.logger.info(f"Performing safety check for sequence: {action_sequence.sequence_id}")
            
            # Perform all safety validations
            errors = self.validate_action_sequence(action_sequence, robot_state)
            
            # Determine severity based on number and type of errors
            critical_errors = [e for e in errors if any(word in e.lower() for word in ['collision', 'safety', 'harm', 'damage', 'dangerous'])]
            high_severity_errors = [e for e in errors if any(word in e.lower() for word in ['force', 'navigation', 'capability'])]
            
            if critical_errors:
                severity = 'critical'
            elif high_severity_errors:
                severity = 'high'
            elif errors:
                severity = 'medium'
            else:
                severity = 'low'
            
            # Create recommendations
            recommendations = self._generate_safety_recommendations(errors)
            
            is_safe = len(critical_errors) == 0
            
            result = SafetyCheckResult(
                is_safe=is_safe,
                issues=errors,
                recommendations=recommendations,
                severity=severity
            )
            
            self.logger.info(f"Safety check complete: is_safe={is_safe}, severity={severity}, issues_count={len(errors)}")
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error performing safety check: {e}")
            raise VLAException(
                f"Safety check error: {str(e)}", 
                VLAErrorType.SAFETY_ERROR,
                e
            )
    
    def _generate_safety_recommendations(self, issues: List[str]) -> List[str]:
        """Generate safety recommendations based on identified issues."""
        recommendations = []
        
        for issue in issues:
            if 'collision' in issue.lower():
                recommendations.append("Review navigation path for potential collisions")
            elif 'force' in issue.lower():
                recommendations.append("Reduce force parameters and add safety margins")
            elif 'capability' in issue.lower():
                recommendations.append("Modify action to match robot capabilities")
            elif 'navigation' in issue.lower() and 'far' in issue.lower():
                recommendations.append("Break navigation into shorter segments with safety checkpoints")
            elif 'safety' in issue.lower():
                recommendations.append("Add safety verification steps before action execution")
        
        return recommendations


# Global instance
_action_plan_validator = None


def get_action_plan_validator() -> ActionPlanValidator:
    """Get the global action plan validator instance."""
    global _action_plan_validator
    if _action_plan_validator is None:
        _action_plan_validator = ActionPlanValidator()
    return _action_plan_validator


def validate_cognitive_plan(cognitive_plan: CognitivePlanModel) -> List[str]:
    """Convenience function to validate a cognitive plan."""
    validator = get_action_plan_validator()
    return validator.validate_cognitive_plan(cognitive_plan)


def validate_action_sequence(action_sequence: ActionSequenceModel, robot_state: Optional['RobotStateModel'] = None) -> List[str]:
    """Convenience function to validate an action sequence."""
    validator = get_action_plan_validator()
    return validator.validate_action_sequence(action_sequence, robot_state)


def perform_safety_check(action_sequence: ActionSequenceModel, robot_state: Optional['RobotStateModel'] = None) -> SafetyCheckResult:
    """Convenience function to perform safety check."""
    validator = get_action_plan_validator()
    return validator.perform_safety_check(action_sequence, robot_state)