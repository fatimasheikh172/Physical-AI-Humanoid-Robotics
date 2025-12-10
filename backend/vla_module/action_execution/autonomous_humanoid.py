"""
Capstone Integration: The Autonomous Humanoid

This module implements the capstone project for the Vision-Language-Action (VLA) system,
bringing together all components in a complete autonomous humanoid robot that responds
to voice commands, plans actions, navigates, identifies objects, and manipulates them.
"""

import asyncio
import logging
import time
from typing import Dict, List, Any, Optional
from dataclasses import dataclass
import uuid

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import VoiceCommand, CognitivePlan, ActionSequence, ExecutionStatus
from ..core.data_models import VoiceCommandModel, CognitivePlanModel, ActionSequenceModel
from ..voice_recognition.voice_command_node import get_voice_command_node
from ..llm_planning.cognitive_planner import get_cognitive_planner
from ..action_execution.action_sequencer import get_action_sequencer
from ..action_execution.robot_controller import get_robot_controller
from ..action_execution.safety_monitor import get_safety_monitor
from ..vision_perception.vision_processor import get_vision_processor


@dataclass
class HumanoidCapabilities:
    """Capabilities of the autonomous humanoid robot."""
    navigation: bool = True  # Can navigate in the environment
    manipulation: bool = True  # Can manipulate objects
    perception: bool = True  # Can perceive environment
    voice_interaction: bool = True  # Can process voice commands
    grasp_and_release: bool = True  # Can grasp and release objects
    path_planning: bool = True  # Can plan paths in environment
    object_identification: bool = True  # Can identify objects
    motion_control: bool = True  # Can control movement


@dataclass
class CapstoneScenario:
    """A specific scenario for the capstone project."""
    scenario_id: str
    name: str
    description: str
    voice_command: str
    expected_outcomes: List[str]
    success_criteria: List[str]
    difficulty_level: str  # beginner, intermediate, advanced


class AutonomousHumanoid:
    """
    Main class for the Autonomous Humanoid capstone project.
    Integrates all VLA components into a cohesive autonomous system.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Initialize all VLA components
        self.voice_command_node = get_voice_command_node()
        self.cognitive_planner = get_cognitive_planner()
        self.action_sequencer = get_action_sequencer()
        self.robot_controller = get_robot_controller()
        self.safety_monitor = get_safety_monitor()
        self.vision_processor = get_vision_processor()
        
        # Humanoid capabilities
        self.capabilities = HumanoidCapabilities(
            navigation=getattr(self.config, 'humanoid_navigation_capable', True),
            manipulation=getattr(self.config, 'humanoid_manipulation_capable', True),
            perception=getattr(self.config, 'humanoid_perception_capable', True),
            voice_interaction=True,
            grasp_and_release=getattr(self.config, 'humanoid_grasping_capable', True),
            path_planning=True,
            object_identification=True,
            motion_control=True
        )
        
        # Capstone scenarios
        self.scenarios = self._define_capstone_scenarios()
        
        # Execution tracking
        self.active_executions = {}
        self.execution_history = []
        
        self.logger.info("AutonomousHumanoid initialized with full VLA integration")
    
    def _define_capstone_scenarios(self) -> List[CapstoneScenario]:
        """
        Define the capstone project scenarios.
        
        Returns:
            List of CapstoneScenario objects
        """
        scenarios = [
            CapstoneScenario(
                scenario_id="capstone_basic_nav_001",
                name="Basic Navigation",
                description="Robot navigates to a specified location and returns",
                voice_command="Go to the kitchen and come back",
                expected_outcomes=["robot navigates to kitchen", "robot returns to starting position"],
                success_criteria=["reaches target location", "returns to start", "no safety violations"],
                difficulty_level="beginner"
            ),
            CapstoneScenario(
                scenario_id="capstone_object_find_002",
                name="Object Identification",
                description="Robot identifies a specific object in the environment",
                voice_command="Find the red ball in the living room",
                expected_outcomes=["robot navigates to living room", "robot identifies red ball", "robot confirms detection"],
                success_criteria=["navigates to correct room", "detects specified object", "reports location"],
                difficulty_level="intermediate"
            ),
            CapstoneScenario(
                scenario_id="capstone_simple_manipulation_003",
                name="Simple Manipulation",
                description="Robot picks up an object and places it elsewhere",
                voice_command="Pick up the cup on the table and put it in the sink",
                expected_outcomes=["robot detects cup", "robot picks up cup", "robot places cup in sink"],
                success_criteria=["successfully grasps object", "correctly places object", "no damage to object"],
                difficulty_level="intermediate"
            ),
            CapstoneScenario(
                scenario_id="capstone_complex_task_004", 
                name="Complex Task Execution",
                description="Robot performs a multi-step task involving navigation, perception, and manipulation",
                voice_command="Go to the office, find the black pen on Jane's desk, and bring it to me",
                expected_outcomes=[
                    "robot navigates to office",
                    "robot identifies black pen", 
                    "robot grasps pen",
                    "robot returns to user"
                ],
                success_criteria=[
                    "completes all navigation steps",
                    "identifies correct object", 
                    "successfully manipulates object",
                    "returns object to user"
                ],
                difficulty_level="advanced"
            ),
            CapstoneScenario(
                scenario_id="capstone_autonomous_helper_005",
                name="Autonomous Helper",
                description="Robot acts as an autonomous assistant performing complex household tasks",
                voice_command="Clean up the dining table and put the dishes in the kitchen",
                expected_outcomes=[
                    "robot navigates to dining table",
                    "robot identifies multiple objects", 
                    "robot picks up multiple items",
                    "robot places items in designated locations"
                ],
                success_criteria=[
                    "correctly identifies items to be moved",
                    "navigates safely between locations",
                    "correctly manipulates multiple objects",
                    "completes task efficiently"
                ],
                difficulty_level="advanced"
            )
        ]
        
        self.logger.info(f"Defined {len(scenarios)} capstone scenarios")
        return scenarios
    
    @log_exception()
    async def execute_capstone_scenario(self, scenario_id: str, user_id: str = "default_user") -> Dict[str, Any]:
        """
        Execute a specific capstone scenario.
        
        Args:
            scenario_id: ID of the scenario to execute
            user_id: ID of the user requesting execution
            
        Returns:
            Dictionary with execution results
        """
        try:
            # Find the scenario
            scenario = next((sc for sc in self.scenarios if sc.scenario_id == scenario_id), None)
            if not scenario:
                raise VLAException(
                    f"Unknown capstone scenario: {scenario_id}",
                    VLAErrorType.VALIDATION_ERROR
                )
            
            self.logger.info(f"Starting capstone scenario: {scenario.name} ({scenario_id}) for user {user_id}")
            
            # Create execution ID
            execution_id = f"exec_{uuid.uuid4().hex[:8]}_{int(time.time())}"
            start_time = time.time()
            
            # Store execution in progress
            self.active_executions[execution_id] = {
                'scenario': scenario,
                'start_time': start_time,
                'status': 'executing',
                'steps_completed': 0,
                'total_steps': len(scenario.expected_outcomes)
            }
            
            try:
                # Step 1: Process the voice command
                self.logger.info(f"Step 1: Processing voice command: '{scenario.voice_command}'")
                voice_command = VoiceCommandModel.create(
                    transcript=scenario.voice_command,
                    user_id=user_id,
                    language=self.config.whisper_language
                )
                
                # Step 2: Generate cognitive plan
                self.logger.info("Step 2: Generating cognitive plan")
                cognitive_plan = await self.cognitive_planner.plan(voice_command)
                
                # Validate the plan against safety and humanoid capabilities
                self.logger.info("Step 2b: Validating cognitive plan")
                validation_errors = await self.cognitive_planner.validate_plan(cognitive_plan)
                if validation_errors:
                    raise VLAException(
                        f"Plan validation failed: {validation_errors}",
                        VLAErrorType.PLANNING_ERROR
                    )
                
                # Step 3: Convert plan to action sequence
                self.logger.info("Step 3: Converting plan to action sequence")
                action_sequence = await self.action_sequencer.sequence_plan(cognitive_plan)
                
                # Step 4: Execute action sequence
                self.logger.info("Step 4: Executing action sequence")
                action_responses = await self.robot_controller.execute_action_sequence(action_sequence)
                
                # Step 5: Validate completion against success criteria
                self.logger.info("Step 5: Validating execution against success criteria")
                success = await self._validate_scenario_completion(
                    scenario, 
                    action_responses, 
                    cognitive_plan
                )
                
                # Calculate metrics
                execution_time = time.time() - start_time
                completion_percentage = len([r for r in action_responses if r.status == ExecutionStatus.COMPLETED]) / len(action_responses) if action_responses else 0.0
                
                # Create result
                result = {
                    'execution_id': execution_id,
                    'scenario_id': scenario_id,
                    'scenario_name': scenario.name,
                    'user_id': user_id,
                    'success': success,
                    'execution_time': execution_time,
                    'completion_percentage': completion_percentage,
                    'action_count': len(action_responses),
                    'successful_actions': len([r for r in action_responses if r.status == ExecutionStatus.COMPLETED]),
                    'failed_actions': len([r for r in action_responses if r.status == ExecutionStatus.FAILED]),
                    'expected_outcomes_met': [],  # Would populate based on actual behavior
                    'success_criteria_met': [],   # Would populate based on actual behavior
                    'logs': []  # Execution logs
                }
                
                self.logger.info(f"Capstone scenario completed: {scenario.name}, Success: {success}")
                
                # Add to execution history
                self.execution_history.append(result)
                
                return result
                
            finally:
                # Clean up active execution
                if execution_id in self.active_executions:
                    del self.active_executions[execution_id]
        
        except Exception as e:
            self.logger.error(f"Error executing capstone scenario {scenario_id}: {e}")
            raise VLAException(
                f"Capstone execution error: {str(e)}", 
                VLAErrorType.CAPSTONE_EXECUTION_ERROR,
                e
            )
    
    @log_exception()
    async def _validate_scenario_completion(
        self, 
        scenario: CapstoneScenario, 
        action_responses: List['ActionResponseModel'], 
        cognitive_plan: CognitivePlanModel
    ) -> bool:
        """
        Validate that the scenario was completed successfully based on success criteria.
        
        Args:
            scenario: CapstoneScenario that was executed
            action_responses: List of action responses from execution
            cognitive_plan: CognitivePlan that was executed
            
        Returns:
            True if scenario completed successfully, False otherwise
        """
        try:
            # Check if all actions completed successfully
            all_actions_succeeded = all(resp.status == ExecutionStatus.COMPLETED for resp in action_responses)
            
            # Check if the task decomposition in the plan matches the expected outcomes
            expected_outcome_matches = 0
            for outcome in scenario.expected_outcomes:
                # This is a simplified check - in a real implementation, we'd need to 
                # correlate the actual robot behavior with the expected outcomes
                # For now, we'll check if certain key phrases appear in the action descriptions
                outcome_lower = outcome.lower()
                for task in cognitive_plan.task_decomposition:
                    if outcome_lower in task.task_description.lower():
                        expected_outcome_matches += 1
                        break
            
            # Calculate success based on multiple factors
            success = (
                all_actions_succeeded and
                len(action_responses) > 0 and
                expected_outcome_matches >= len(scenario.expected_outcomes) * 0.7  # At least 70% of outcomes matched
            )
            
            self.logger.debug(f"Scenario validation - All actions succeeded: {all_actions_succeeded}, "
                              f"Outcome matches: {expected_outcome_matches}/{len(scenario.expected_outcomes)}, "
                              f"Success: {success}")
            
            return success
            
        except Exception as e:
            self.logger.error(f"Error validating scenario completion: {e}")
            return False
    
    @log_exception()
    async def execute_voice_command_end_to_end(self, voice_command_text: str, user_id: str = "default_user") -> Dict[str, Any]:
        """
        Execute a complete end-to-end voice command through the full VLA pipeline.
        
        Args:
            voice_command_text: Voice command as text
            user_id: ID of the user issuing the command
            
        Returns:
            Dictionary with execution results
        """
        try:
            start_time = time.time()
            self.logger.info(f"Starting end-to-end execution for command: '{voice_command_text}' from user {user_id}")
            
            # Create voice command model
            voice_command = VoiceCommandModel.create(
                transcript=voice_command_text,
                user_id=user_id,
                language=self.config.whisper_language
            )
            
            # Validate command against safety constraints
            safety_violations = await self.safety_monitor.check_safety_before_action(voice_command)
            if safety_violations:
                error_msg = f"Safety violations detected: {[v.description for v in safety_violations]}"
                self.logger.error(error_msg)
                return {
                    'success': False,
                    'error': error_msg,
                    'safety_violations': [v.description for v in safety_violations],
                    'execution_time': time.time() - start_time
                }
            
            # Plan phase
            cognitive_plan = await self.cognitive_planner.plan(voice_command)
            
            # Validate plan
            validation_errors = await self.cognitive_planner.validate_plan(cognitive_plan)
            if validation_errors:
                error_msg = f"Plan validation failed: {validation_errors}"
                self.logger.error(error_msg)
                return {
                    'success': False,
                    'error': error_msg,
                    'validation_errors': validation_errors,
                    'execution_time': time.time() - start_time
                }
            
            # Sequence actions
            action_sequence = await self.action_sequencer.sequence_plan(cognitive_plan)
            
            # Execute actions
            action_responses = await self.robot_controller.execute_action_sequence(action_sequence)
            
            # Determine overall success
            all_successful = all(response.status == ExecutionStatus.COMPLETED for response in action_responses)
            
            # Calculate metrics
            execution_time = time.time() - start_time
            completion_rate = sum(1 for r in action_responses if r.status == ExecutionStatus.COMPLETED) / len(action_responses) if action_responses else 0.0
            
            result = {
                'success': all_successful,
                'execution_time': execution_time,
                'completion_rate': completion_rate,
                'action_count': len(action_responses),
                'successful_actions': sum(1 for r in action_responses if r.status == ExecutionStatus.COMPLETED),
                'failed_actions': sum(1 for r in action_responses if r.status == ExecutionStatus.FAILED),
                'command': voice_command_text,
                'user_id': user_id,
                'cognitive_plan_id': cognitive_plan.plan_id,
                'action_sequence_id': action_sequence.sequence_id,
                'action_responses': [
                    {
                        'action_id': resp.action_id,
                        'status': resp.status.name,
                        'result_summary': resp.result_summary
                    } for resp in action_responses
                ]
            }
            
            self.logger.info(f"End-to-end command execution completed: Success={all_successful}, Time={execution_time:.2f}s")
            return result
            
        except Exception as e:
            self.logger.error(f"Error in end-to-end execution: {e}")
            return {
                'success': False,
                'error': str(e),
                'execution_time': time.time() - start_time
            }
    
    @log_exception()
    async def run_capstone_demo(self, demo_type: str = "full") -> Dict[str, Any]:
        """
        Run a demonstration of the capstone project.
        
        Args:
            demo_type: Type of demo to run ("basic", "intermediate", "advanced", "full")
            
        Returns:
            Dictionary with demo results
        """
        try:
            self.logger.info(f"Running capstone demo: {demo_type}")
            
            # Select scenarios based on demo type
            if demo_type == "basic":
                scenarios = [s for s in self.scenarios if s.difficulty_level == "beginner"]
            elif demo_type == "intermediate":
                scenarios = [s for s in self.scenarios if s.difficulty_level == "intermediate"]
            elif demo_type == "advanced":
                scenarios = [s for s in self.scenarios if s.difficulty_level == "advanced"]
            else:  # "full"
                scenarios = self.scenarios
            
            results = {
                'demo_type': demo_type,
                'total_scenarios': len(scenarios),
                'successful_scenarios': 0,
                'failed_scenarios': 0,
                'total_execution_time': 0.0,
                'scenario_results': []
            }
            
            for scenario in scenarios:
                try:
                    result = await self.execute_capstone_scenario(scenario.scenario_id, user_id="demo_user")
                    
                    if result['success']:
                        results['successful_scenarios'] += 1
                    else:
                        results['failed_scenarios'] += 1
                    
                    results['total_execution_time'] += result['execution_time']
                    results['scenario_results'].append(result)
                    
                except Exception as e:
                    self.logger.error(f"Error executing scenario {scenario.scenario_id}: {e}")
                    results['failed_scenarios'] += 1
                    results['scenario_results'].append({
                        'scenario_id': scenario.scenario_id,
                        'success': False,
                        'error': str(e),
                        'execution_time': 0.0
                    })
            
            results['success_rate'] = results['successful_scenarios'] / results['total_scenarios'] if results['total_scenarios'] > 0 else 0.0
            results['average_execution_time'] = results['total_execution_time'] / results['total_scenarios'] if results['total_scenarios'] > 0 else 0.0
            
            self.logger.info(f"Capstone demo completed: {results['success_rate']:.2%} success rate")
            return results
            
        except Exception as e:
            self.logger.error(f"Error running capstone demo: {e}")
            raise VLAException(
                f"Capstone demo error: {str(e)}", 
                VLAErrorType.CAPSTONE_EXECUTION_ERROR,
                e
            )
    
    def get_available_scenarios(self) -> List[Dict[str, Any]]:
        """
        Get a list of available capstone scenarios.
        
        Returns:
            List of scenario information
        """
        return [
            {
                'scenario_id': scenario.scenario_id,
                'name': scenario.name,
                'description': scenario.description,
                'voice_command_example': scenario.voice_command,
                'expected_outcomes_count': len(scenario.expected_outcomes),
                'difficulty_level': scenario.difficulty_level
            }
            for scenario in self.scenarios
        ]
    
    def get_execution_history(self) -> List[Dict[str, Any]]:
        """
        Get execution history for the humanoid.
        
        Returns:
            List of execution records
        """
        return self.execution_history.copy()
    
    def get_active_executions(self) -> Dict[str, Dict[str, Any]]:
        """
        Get currently active executions.
        
        Returns:
            Dictionary of active execution information
        """
        return self.active_executions.copy()
    
    async def reset_humanoid_state(self):
        """
        Reset the humanoid to a known safe state.
        """
        self.logger.info("Resetting humanoid to safe state")
        
        # Stop any active robot motion
        await self.robot_controller.emergency_stop()
        
        # Clear any active executions
        self.active_executions.clear()
        
        # Reset components to initial state
        # (In a real implementation, this would reset all components)
        self.logger.info("Humanoid reset to safe state")


class CapstoneEvaluator:
    """
    Evaluates capstone project performance and provides grading/assessment.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
    
    def evaluate_execution(self, execution_result: Dict[str, Any]) -> Dict[str, Any]:
        """
        Evaluate a capstone execution and provide assessment.
        
        Args:
            execution_result: Result from capstone execution
            
        Returns:
            Evaluation with grade and feedback
        """
        try:
            # Calculate performance metrics
            success_score = 0.0
            if execution_result['success']:
                success_score = 1.0
            
            # Time efficiency score (perfect score for completing in ideal time, lower for slower)
            ideal_execution_time = self._get_ideal_time_for_scenario(execution_result['scenario_id'])
            time_efficiency = min(1.0, ideal_execution_time / execution_result['execution_time']) if execution_result['execution_time'] > 0 else 0.0
            
            # Action completion score
            action_completion_score = execution_result['completion_rate']
            
            # Overall score (weighted)
            overall_score = (
                success_score * 0.5 + 
                time_efficiency * 0.3 + 
                action_completion_score * 0.2
            )
            
            # Grade based on score
            if overall_score >= 0.9:
                grade = 'A'
                feedback = "Excellent performance! The robot completed the task flawlessly."
            elif overall_score >= 0.8:
                grade = 'B'
                feedback = "Good job! The robot successfully completed the task with minor inefficiencies."
            elif overall_score >= 0.7:
                grade = 'C'
                feedback = "Satisfactory performance. The robot completed the task but had some issues."
            elif overall_score >= 0.6:
                grade = 'D'
                feedback = "Below expectations. The robot partially completed the task."
            else:
                grade = 'F'
                feedback = "Unsatisfactory performance. The robot failed to complete the task."
            
            evaluation = {
                'grade': grade,
                'overall_score': overall_score,
                'success_score': success_score,
                'time_efficiency_score': time_efficiency,
                'action_completion_score': action_completion_score,
                'feedback': feedback,
                'strengths': self._identify_strengths(execution_result),
                'improvements': self._identify_improvements(execution_result)
            }
            
            return evaluation
            
        except Exception as e:
            self.logger.error(f"Error evaluating execution: {e}")
            return {
                'grade': 'F',
                'overall_score': 0.0,
                'feedback': f"Evaluation error: {str(e)}",
                'strengths': [],
                'improvements': []
            }
    
    def _get_ideal_time_for_scenario(self, scenario_id: str) -> float:
        """
        Get the ideal time for a scenario based on its complexity.
        
        Args:
            scenario_id: ID of the scenario
            
        Returns:
            Ideal execution time in seconds
        """
        # Define ideal times for different scenarios
        ideal_times = {
            'capstone_basic_nav_001': 30.0,      # Basic navigation
            'capstone_object_find_002': 60.0,    # Object identification with navigation
            'capstone_simple_manipulation_003': 90.0,  # Simple manipulation
            'capstone_complex_task_004': 120.0,  # Complex multi-step task
            'capstone_autonomous_helper_005': 180.0   # Advanced multi-object task
        }
        
        return ideal_times.get(scenario_id, 60.0)  # Default to 60s if not specified
    
    def _identify_strengths(self, execution_result: Dict[str, Any]) -> List[str]:
        """
        Identify strengths in the execution.
        
        Args:
            execution_result: Result from capstone execution
            
        Returns:
            List of strengths
        """
        strengths = []
        
        if execution_result['success']:
            strengths.append("Successfully completed the assigned task")
        
        if execution_result['completion_rate'] > 0.9:
            strengths.append("High action completion rate")
        
        if execution_result['execution_time'] < self._get_ideal_time_for_scenario(execution_result['scenario_id']) * 1.2:
            strengths.append("Efficient execution time")
        
        return strengths
    
    def _identify_improvements(self, execution_result: Dict[str, Any]) -> List[str]:
        """
        Identify areas for improvement.
        
        Args:
            execution_result: Result from capstone execution
            
        Returns:
            List of improvement areas
        """
        improvements = []
        
        if not execution_result['success']:
            improvements.append("Focus on task completion reliability")
        
        if execution_result['completion_rate'] < 0.8:
            improvements.append("Improve action execution success rate")
        
        if execution_result['execution_time'] > self._get_ideal_time_for_scenario(execution_result['scenario_id']) * 1.5:
            improvements.append("Work on execution efficiency")
        
        return improvements


# Global instances
_autonomous_humanoid = None
_capstone_evaluator = None


def get_autonomous_humanoid() -> AutonomousHumanoid:
    """
    Get the global autonomous humanoid instance.
    
    Returns:
        AutonomousHumanoid instance
    """
    global _autonomous_humanoid
    if _autonomous_humanoid is None:
        _autonomous_humanoid = AutonomousHumanoid()
    return _autonomous_humanoid


def get_capstone_evaluator() -> CapstoneEvaluator:
    """
    Get the global capstone evaluator instance.
    
    Returns:
        CapstoneEvaluator instance
    """
    global _capstone_evaluator
    if _capstone_evaluator is None:
        _capstone_evaluator = CapstoneEvaluator()
    return _capstone_evaluator


async def execute_capstone_scenario(scenario_id: str, user_id: str = "default_user") -> Dict[str, Any]:
    """
    Convenience function to execute a capstone scenario.
    
    Args:
        scenario_id: ID of the scenario to execute
        user_id: ID of the user requesting execution
        
    Returns:
        Dictionary with execution results
    """
    humanoid = get_autonomous_humanoid()
    return await humanoid.execute_capstone_scenario(scenario_id, user_id)


async def run_capstone_demo(demo_type: str = "full") -> Dict[str, Any]:
    """
    Convenience function to run a capstone demo.
    
    Args:
        demo_type: Type of demo to run
        
    Returns:
        Dictionary with demo results
    """
    humanoid = get_autonomous_humanoid()
    return await humanoid.run_capstone_demo(demo_type)


def evaluate_capstone_execution(execution_result: Dict[str, Any]) -> Dict[str, Any]:
    """
    Convenience function to evaluate a capstone execution.
    
    Args:
        execution_result: Result from capstone execution
        
    Returns:
        Evaluation results
    """
    evaluator = get_capstone_evaluator()
    return evaluator.evaluate_execution(execution_result)


def get_available_capstone_scenarios() -> List[Dict[str, Any]]:
    """
    Convenience function to get available capstone scenarios.
    
    Returns:
        List of scenario information
    """
    humanoid = get_autonomous_humanoid()
    return humanoid.get_available_scenarios()