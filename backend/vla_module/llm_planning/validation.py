"""
Validation of cognitive planning latency and accuracy requirements for the VLA module.

This module validates that the cognitive planning component meets performance requirements:
- Latency under 2 seconds
- Sufficient accuracy for reliable plan generation
"""

import asyncio
import time
import logging
from typing import Dict, List, Any, Callable, Optional
from dataclasses import dataclass

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import VoiceCommand, CognitivePlan
from ..core.data_models import VoiceCommandModel
from .cognitive_planner import CognitivePlanner, get_cognitive_planner


@dataclass
class PerformanceRequirement:
    """Defines a performance requirement for validation."""
    name: str
    description: str
    threshold: float
    unit: str
    is_lower_better: bool  # True if lower values are better (e.g., latency)


@dataclass
class ValidationResult:
    """Result of a validation test."""
    requirement_met: bool
    measured_value: float
    threshold: float
    unit: str
    requirement_name: str
    error_message: Optional[str] = None


class CognitivePlanningValidator:
    """
    Validates that the cognitive planning component meets performance requirements:
    - Latency thresholds
    - Accuracy measures
    - Reliability metrics
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Initialize cognitive planner
        self.cognitive_planner = get_cognitive_planner()
        
        # Performance requirements
        self.performance_requirements = {
            'planning_latency': PerformanceRequirement(
                name='planning_latency',
                description='Time to generate a cognitive plan',
                threshold=self.config.planning_timeout,  # From config (e.g., 2.0 seconds)
                unit='seconds',
                is_lower_better=True
            ),
            'plan_quality_score': PerformanceRequirement(
                name='plan_quality_score',
                description='Quality score of generated plans (0-1 scale)',
                threshold=0.7,  # Minimum acceptable quality
                unit='score',
                is_lower_better=False
            ),
            'success_rate': PerformanceRequirement(
                name='success_rate',
                description='Percentage of successfully generated plans',
                threshold=0.9,  # 90% success rate
                unit='ratio',
                is_lower_better=False
            )
        }
        
        self.logger.info("CognitivePlanningValidator initialized")
    
    @log_exception()
    async def validate_latency_requirement(self, test_commands: List[str] = None) -> ValidationResult:
        """
        Validate that cognitive planning meets latency requirements.
        
        Args:
            test_commands: Optional list of test commands to use for validation.
                         If None, uses default test commands.
        
        Returns:
            ValidationResult indicating if latency requirement is met
        """
        try:
            # Use default test commands if none provided
            if test_commands is None:
                test_commands = [
                    "Move to the kitchen",
                    "Find the red ball and pick it up",
                    "Navigate to the office and wait near the desk",
                    "Go to the next room and come back",
                    "Turn around and move forward 2 meters"
                ]
            
            # Create mock voice commands
            voice_commands = []
            for i, cmd in enumerate(test_commands):
                voice_cmd = VoiceCommandModel.create(
                    transcript=cmd,
                    user_id=f"test_user_{i}",
                    language="en"
                )
                voice_commands.append(voice_cmd)
            
            # Time the planning process for each command
            total_time = 0.0
            successful_plans = 0
            
            for voice_cmd in voice_commands:
                try:
                    start_time = time.time()
                    
                    # Perform planning
                    cognitive_plan = await self.cognitive_planner.plan(voice_cmd)
                    
                    end_time = time.time()
                    planning_time = end_time - start_time
                    total_time += planning_time
                    
                    # Count successful plans
                    if cognitive_plan:
                        successful_plans += 1
                        
                except Exception as e:
                    self.logger.error(f"Error planning command '{voice_cmd.transcript}': {e}")
            
            # Calculate average latency
            avg_latency = total_time / len(voice_commands) if voice_commands else 0.0
            success_rate = successful_plans / len(voice_commands) if voice_commands else 0.0
            
            # Check against requirement
            requirement = self.performance_requirements['planning_latency']
            requirement_met = avg_latency <= requirement.threshold
            
            if requirement_met:
                result = ValidationResult(
                    requirement_met=True,
                    measured_value=avg_latency,
                    threshold=requirement.threshold,
                    unit=requirement.unit,
                    requirement_name=requirement.name
                )
                self.logger.info(f"Latency validation passed: {avg_latency:.2f}s avg < {requirement.threshold}s threshold")
            else:
                result = ValidationResult(
                    requirement_met=False,
                    measured_value=avg_latency,
                    threshold=requirement.threshold,
                    unit=requirement.unit,
                    requirement_name=requirement.name,
                    error_message=f"Average planning time {avg_latency:.2f}s exceeds threshold of {requirement.threshold}s"
                )
                self.logger.warning(f"Latency validation failed: {avg_latency:.2f}s avg > {requirement.threshold}s threshold")
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error validating latency requirement: {e}")
            raise VLAException(
                f"Latency validation error: {str(e)}", 
                VLAErrorType.VALIDATION_ERROR,
                e
            )
    
    @log_exception()
    async def validate_accuracy_requirement(self, test_cases: List[Dict[str, Any]] = None) -> ValidationResult:
        """
        Validate that cognitive planning meets accuracy requirements.
        This measures the quality and appropriateness of generated plans.
        
        Args:
            test_cases: Optional list of test cases with expected outcomes.
                       Each test case is a dict with 'command', 'expected_actions', etc.
        
        Returns:
            ValidationResult indicating if accuracy requirement is met
        """
        try:
            if test_cases is None:
                # Default test cases with expected action types
                test_cases = [
                    {
                        'command': 'Move to the kitchen',
                        'expected_action_types': ['move_to', 'navigate'],
                        'expected_task_count_range': (1, 3)
                    },
                    {
                        'command': 'Pick up the red cup',
                        'expected_action_types': ['detect_object', 'move_to', 'grasp', 'pick_up'],
                        'expected_task_count_range': (2, 5)
                    },
                    {
                        'command': 'Go to the desk and find the pen',
                        'expected_action_types': ['navigate', 'move_to', 'detect_object'],
                        'expected_task_count_range': (2, 4)
                    }
                ]
            
            quality_scores = []
            
            for test_case in test_cases:
                try:
                    # Create voice command
                    voice_cmd = VoiceCommandModel.create(
                        transcript=test_case['command'],
                        user_id="test_user",
                        language="en"
                    )
                    
                    # Generate plan
                    start_time = time.time()
                    cognitive_plan = await self.cognitive_planner.plan(voice_cmd)
                    planning_time = time.time() - start_time
                    
                    if cognitive_plan:
                        # Calculate quality score based on several metrics
                        quality_score = self._calculate_plan_quality(
                            cognitive_plan, 
                            test_case
                        )
                        quality_scores.append(quality_score)
                        
                        self.logger.debug(f"Plan quality for '{test_case['command']}': {quality_score:.2f}")
                    else:
                        # Failed to generate plan - assign low quality score
                        quality_scores.append(0.0)
                        
                except Exception as e:
                    self.logger.error(f"Error validating accuracy for command '{test_case['command']}': {e}")
                    quality_scores.append(0.0)
            
            # Calculate average quality score
            avg_quality = sum(quality_scores) / len(quality_scores) if quality_scores else 0.0
            
            # Check against requirement
            requirement = self.performance_requirements['plan_quality_score']
            requirement_met = avg_quality >= requirement.threshold
            
            if requirement_met:
                result = ValidationResult(
                    requirement_met=True,
                    measured_value=avg_quality,
                    threshold=requirement.threshold,
                    unit=requirement.unit,
                    requirement_name=requirement.name
                )
                self.logger.info(f"Accuracy validation passed: {avg_quality:.2f} > {requirement.threshold}")
            else:
                result = ValidationResult(
                    requirement_met=False,
                    measured_value=avg_quality,
                    threshold=requirement.threshold,
                    unit=requirement.unit,
                    requirement_name=requirement.name,
                    error_message=f"Average plan quality {avg_quality:.2f} is below threshold of {requirement.threshold}"
                )
                self.logger.warning(f"Accuracy validation failed: {avg_quality:.2f} < {requirement.threshold}")
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error validating accuracy requirement: {e}")
            raise VLAException(
                f"Accuracy validation error: {str(e)}", 
                VLAErrorType.VALIDATION_ERROR,
                e
            )
    
    @log_exception()
    async def validate_reliability_requirement(self, test_commands: List[str] = None, num_attempts: int = 10) -> ValidationResult:
        """
        Validate that cognitive planning meets reliability (success rate) requirements.
        
        Args:
            test_commands: Optional list of commands to test with
            num_attempts: Number of attempts for each command to test consistency
        
        Returns:
            ValidationResult indicating if reliability requirement is met
        """
        try:
            if test_commands is None:
                test_commands = [
                    "Move forward 1 meter",
                    "Turn left 90 degrees", 
                    "Find the closest chair"
                ]
            
            total_attempts = len(test_commands) * num_attempts
            successful_generations = 0
            
            for cmd in test_commands:
                for attempt in range(num_attempts):
                    try:
                        # Create voice command
                        voice_cmd = VoiceCommandModel.create(
                            transcript=cmd,
                            user_id=f"test_user_{attempt}",
                            language="en"
                        )
                        
                        # Attempt to generate plan
                        cognitive_plan = await self.cognitive_planner.plan(voice_cmd)
                        
                        # Check if plan was successfully generated
                        if cognitive_plan and cognitive_plan.task_decomposition:
                            successful_generations += 1
                            
                    except Exception as e:
                        self.logger.debug(f"Plan generation failed for '{cmd}' (attempt {attempt+1}): {e}")
            
            success_rate = successful_generations / total_attempts if total_attempts > 0 else 0.0
            
            # Check against requirement
            requirement = self.performance_requirements['success_rate']
            requirement_met = success_rate >= requirement.threshold
            
            if requirement_met:
                result = ValidationResult(
                    requirement_met=True,
                    measured_value=success_rate,
                    threshold=requirement.threshold,
                    unit=requirement.unit,
                    requirement_name=requirement.name
                )
                self.logger.info(f"Reliability validation passed: {success_rate:.2%} success rate >= {requirement.threshold:.2%} threshold")
            else:
                result = ValidationResult(
                    requirement_met=False,
                    measured_value=success_rate,
                    threshold=requirement.threshold,
                    unit=requirement.unit,
                    requirement_name=requirement.name,
                    error_message=f"Success rate {success_rate:.2%} is below threshold of {requirement.threshold:.2%}"
                )
                self.logger.warning(f"Reliability validation failed: {success_rate:.2%} < {requirement.threshold:.2%}")
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error validating reliability requirement: {e}")
            raise VLAException(
                f"Reliability validation error: {str(e)}", 
                VLAErrorType.VALIDATION_ERROR,
                e
            )
    
    @log_exception()
    async def validate_all_requirements(self) -> Dict[str, ValidationResult]:
        """
        Validate all cognitive planning requirements at once.
        
        Returns:
            Dictionary mapping requirement names to validation results
        """
        try:
            self.logger.info("Starting comprehensive cognitive planning validation")
            
            results = {}
            
            # Validate latency
            results['latency'] = await self.validate_latency_requirement()
            
            # Validate accuracy
            results['accuracy'] = await self.validate_accuracy_requirement()
            
            # Validate reliability
            results['reliability'] = await self.validate_reliability_requirement()
            
            # Log summary
            passed_count = sum(1 for result in results.values() if result.requirement_met)
            total_count = len(results)
            
            self.logger.info(f"Comprehensive validation summary: {passed_count}/{total_count} requirements passed")
            
            return results
            
        except Exception as e:
            self.logger.error(f"Error in comprehensive validation: {e}")
            raise VLAException(
                f"Comprehensive validation error: {str(e)}", 
                VLAErrorType.VALIDATION_ERROR,
                e
            )
    
    def _calculate_plan_quality(self, plan: 'CognitivePlanModel', expected: Dict[str, Any]) -> float:
        """
        Calculate a quality score for a generated plan based on expectations.
        
        Args:
            plan: Generated CognitivePlanModel
            expected: Expected outcomes for the command
            
        Returns:
            Quality score between 0.0 and 1.0
        """
        score = 0.0
        total_weight = 0.0
        
        # Weight factors for different quality aspects
        task_count_weight = 0.3
        action_type_weight = 0.4
        plan_confidence_weight = 0.3
        
        # 1. Task count quality (matches expected range)
        expected_range = expected.get('expected_task_count_range', (0, 10))
        actual_count = len(plan.task_decomposition)
        
        if expected_range[0] <= actual_count <= expected_range[1]:
            task_score = 1.0
        else:
            # Calculate score based on distance from expected range
            dist_to_range = min(
                abs(actual_count - expected_range[0]),
                abs(actual_count - expected_range[1])
            )
            # Normalize to 0-1 scale (penalty grows but capped)
            task_score = max(0.0, 1.0 - (dist_to_range * 0.1))
        
        score += task_score * task_count_weight
        total_weight += task_count_weight
        
        # 2. Action type quality (contains expected types)
        actual_action_types = [task.task_type for task in plan.task_decomposition]
        expected_action_types = expected.get('expected_action_types', [])
        
        matching_types = [act for act in actual_action_types if act in expected_action_types]
        if expected_action_types:
            action_type_score = len(matching_types) / len(expected_action_types)
        else:
            action_type_score = 1.0 if not actual_action_types else 0.0
        
        score += action_type_score * action_type_weight
        total_weight += action_type_weight
        
        # 3. Plan confidence quality (how confident the model was)
        confidence_score = min(1.0, plan.confidence)  # Cap at 1.0
        score += confidence_score * plan_confidence_weight
        total_weight += plan_confidence_weight
        
        # Normalize by total weight
        final_score = score / total_weight if total_weight > 0 else 0.0
        
        return final_score


# Global instance
_cognitive_planning_validator = None


def get_cognitive_planning_validator() -> CognitivePlanningValidator:
    """Get the global cognitive planning validator instance."""
    global _cognitive_planning_validator
    if _cognitive_planning_validator is None:
        _cognitive_planning_validator = CognitivePlanningValidator()
    return _cognitive_planning_validator


async def validate_planning_latency() -> ValidationResult:
    """Convenience function to validate planning latency."""
    validator = get_cognitive_planning_validator()
    return await validator.validate_latency_requirement()


async def validate_planning_accuracy() -> ValidationResult:
    """Convenience function to validate planning accuracy."""
    validator = get_cognitive_planning_validator()
    return await validator.validate_accuracy_requirement()


async def validate_planning_reliability() -> ValidationResult:
    """Convenience function to validate planning reliability."""
    validator = get_cognitive_planning_validator()
    return await validator.validate_reliability_requirement()


async def validate_all_planning_requirements() -> Dict[str, ValidationResult]:
    """Convenience function to validate all planning requirements."""
    validator = get_cognitive_planning_validator()
    return await validator.validate_all_requirements()