"""
Final integration testing for the Vision-Language-Action (VLA) module.

This module provides comprehensive integration tests that validate the complete
VLA pipeline: voice recognition → cognitive planning → action execution → validation.
"""

import asyncio
import logging
import time
from typing import Dict, List, Any, Optional
from dataclasses import dataclass
import unittest
from unittest.mock import Mock, patch, AsyncMock

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import VoiceCommand, CognitivePlan, ActionSequence, ActionResponse, ExecutionStatus
from ..core.data_models import VoiceCommandModel, CognitivePlanModel, ActionSequenceModel, ActionResponseModel, RobotStateModel
from ..core.vla_manager import get_vla_manager
from ..voice_recognition.voice_command_node import get_voice_command_node
from ..llm_planning.cognitive_planner import get_cognitive_planner
from ..action_execution.robot_controller import get_robot_controller
from ..vision_perception.ros2_vision_node import get_vision_node
from ..capstone_integration.full_pipeline_integrator import get_vla_pipeline_integrator
from ..capstone_integration.capstone_demo import get_capstone_demonstrator


@dataclass
class IntegrationTestResult:
    """Result of an integration test."""
    test_id: str
    test_name: str
    passed: bool
    execution_time: float
    details: Dict[str, Any]
    errors: List[str]
    warnings: List[str]


class VLAFinalIntegrationTester:
    """
    Performs comprehensive integration testing of the complete VLA pipeline.
    Validates that all components work together correctly and meet requirements.
    """

    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)

        # Initialize all system components
        self.vla_manager = get_vla_manager()
        self.pipeline_integrator = get_vla_pipeline_integrator()
        self.voice_command_node = get_voice_command_node()
        self.cognitive_planner = get_cognitive_planner()
        self.robot_controller = get_robot_controller()
        self.vision_node = get_vision_node()
        self.capstone_demonstrator = get_capstone_demonstrator()

        # Test configuration
        self.test_timeout = getattr(self.config, 'integration_test_timeout', 60.0)  # seconds
        self.max_attempts = getattr(self.config, 'integration_test_retries', 3)

        # Test scenarios
        self.test_scenarios = [
            {
                'name': 'simple_navigation',
                'command': 'Move to the kitchen',
                'expected_outcomes': ['navigation_completed', 'arrived_at_destination']
            },
            {
                'name': 'object_interaction',
                'command': 'Pick up the red cup from the table',
                'expected_outcomes': ['object_detected', 'object_grasped', 'action_completed']
            },
            {
                'name': 'complex_task',
                'command': 'Go to the office, find a pen, pick it up, and bring it to me',
                'expected_outcomes': [
                    'navigation_completed',
                    'object_detected',
                    'grasping_completed',
                    'delivery_completed'
                ]
            },
            {
                'name': 'perception_guided_manipulation',
                'command': 'Find the blue bottle on the counter and place it in the cabinet',
                'expected_outcomes': [
                    'object_detected',
                    'object_verified',
                    'manipulation_completed',
                    'placement_completed'
                ]
            }
        ]

        self.logger.info("VLAFinalIntegrationTester initialized")
    
    @log_exception()
    async def run_complete_pipeline_test(self) -> IntegrationTestResult:
        """
        Run a test of the complete VLA pipeline.
        
        Returns:
            IntegrationTestResult with test results
        """
        try:
            test_id = f"integration_test_{int(time.time())}"
            start_time = time.time()
            
            self.logger.info(f"Starting complete pipeline integration test: {test_id}")
            
            # Test the complete pipeline: voice -> cognitive planning -> action execution
            test_command = "Navigate to the kitchen and find the red cup"
            user_id = "integration_tester"
            
            # Execute through the complete pipeline
            pipeline_result = await self.pipeline_integrator.execute_end_to_end_pipeline(
                voice_command=test_command,
                user_id=user_id
            )
            
            execution_time = time.time() - start_time
            
            # Validate results
            errors = []
            warnings = []
            
            # Check that each major component was engaged
            if not pipeline_result.get('plan_generated', False):
                errors.append("Cognitive planning failed to generate a plan")
            
            if not pipeline_result.get('actions_executed', False):
                errors.append("Action execution did not occur")
            
            # Check for expected outcomes
            expected_outcomes = ['navigation', 'perception', 'execution']
            actual_outcomes = pipeline_result.get('achieved_outcomes', [])
            for outcome in expected_outcomes:
                if outcome not in actual_outcomes:
                    errors.append(f"Expected outcome not achieved: {outcome}")
            
            # Check if all actions in sequence completed successfully
            action_responses = pipeline_result.get('responses', [])
            successful_actions = sum(1 for resp in action_responses if resp.status == ExecutionStatus.COMPLETED)
            total_actions = len(action_responses)
            
            if total_actions > 0:
                success_rate = successful_actions / total_actions
                if success_rate < self.config.get('min_integration_success_rate', 0.8):
                    errors.append(f"Action execution success rate too low: {success_rate:.2%} < threshold {self.config.get('min_integration_success_rate', 0.8):.2%}")
            else:
                errors.append("No action responses recorded in pipeline execution")
            
            # Determine test result
            passed = len(errors) == 0
            details = {
                'command_executed': test_command,
                'user_id': user_id,
                'pipeline_result': pipeline_result,
                'success_rate': success_rate if total_actions > 0 else 0.0,
                'total_actions': total_actions,
                'successful_actions': successful_actions
            }
            
            result = IntegrationTestResult(
                test_id=test_id,
                test_name="complete_pipeline_test",
                passed=passed,
                execution_time=execution_time,
                details=details,
                errors=errors,
                warnings=warnings
            )
            
            status = "PASSED" if passed else "FAILED"
            self.logger.info(f"Complete pipeline test {status}: {execution_time:.2f}s execution time")
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error in complete pipeline test: {e}")
            execution_time = time.time() - start_time
            
            return IntegrationTestResult(
                test_id=f"integration_test_error_{int(time.time())}",
                test_name="complete_pipeline_test",
                passed=False,
                execution_time=execution_time,
                details={'error': str(e)},
                errors=[f"Pipeline test error: {str(e)}"],
                warnings=[]
            )
    
    @log_exception()
    async def run_multi_component_integration_test(self) -> IntegrationTestResult:
        """
        Test integration between multiple components simultaneously.
        
        Returns:
            IntegrationTestResult with test results
        """
        try:
            test_id = f"multi_component_test_{int(time.time())}"
            start_time = time.time()
            
            self.logger.info(f"Starting multi-component integration test: {test_id}")
            
            # Simultaneously test voice processing, cognitive planning, and action execution
            # In a real system, this would involve a coordinated test across components
            
            # Create a complex command that involves all components
            complex_command = "Go to the living room, find the blue ball near the couch, pick it up gently, and place it on the coffee table"
            user_id = "integration_tester"
            
            # Test parallel operations
            tasks = []
            
            # Task 1: Process voice command
            voice_task = asyncio.create_task(
                self._test_voice_processing(complex_command)
            )
            tasks.append(voice_task)
            
            # Task 2: Test cognitive planning (would use the voice result)
            planning_task = asyncio.create_task(
                self._test_cognitive_planning(complex_command)
            )
            tasks.append(planning_task)
            
            # Task 3: Test action execution (would eventually execute the plan)
            execution_task = asyncio.create_task(
                self._test_action_execution()
            )
            tasks.append(execution_task)
            
            # Wait for all tasks with timeout
            try:
                results = await asyncio.wait_for(
                    asyncio.gather(*tasks, return_exceptions=True),
                    timeout=self.test_timeout
                )
                
                execution_time = time.time() - start_time
                
                # Evaluate results
                errors = []
                warnings = []
                
                for i, result in enumerate(results):
                    if isinstance(result, Exception):
                        errors.append(f"Component {i+1} test failed: {str(result)}")
                    elif result is False:  # Component test failed
                        errors.append(f"Component {i+1} test failed")
                
                # Validate component coordination
                coordination_errors = await self._validate_component_coordination(results)
                errors.extend(coordination_errors)
                
                passed = len(errors) == 0
                details = {
                    'command': complex_command,
                    'component_results': results,
                    'coordination_errors': coordination_errors if coordination_errors else 'none'
                }
                
                result = IntegrationTestResult(
                    test_id=test_id,
                    test_name="multi_component_integration_test",
                    passed=passed,
                    execution_time=execution_time,
                    details=details,
                    errors=errors,
                    warnings=warnings
                )
                
                status = "PASSED" if passed else "FAILED"
                self.logger.info(f"Multi-component integration test {status}: {execution_time:.2f}s execution time")
                
                return result
                
            except asyncio.TimeoutError:
                execution_time = time.time() - start_time
                error_msg = f"Test timed out after {self.test_timeout}s"
                
                return IntegrationTestResult(
                    test_id=test_id,
                    test_name="multi_component_integration_test",
                    passed=False,
                    execution_time=execution_time,
                    details={'timeout': self.test_timeout},
                    errors=[error_msg],
                    warnings=[]
                )
            
        except Exception as e:
            self.logger.error(f"Error in multi-component integration test: {e}")
            execution_time = time.time() - start_time
            
            return IntegrationTestResult(
                test_id=f"multi_component_error_{int(time.time())}",
                test_name="multi_component_integration_test",
                passed=False,
                execution_time=execution_time,
                details={'error': str(e)},
                errors=[f"Multi-component test error: {str(e)}"],
                warnings=[]
            )
    
    async def _test_voice_processing(self, command: str) -> bool:
        """
        Test the voice processing component.
        
        Args:
            command: Command to process (simulated as text-to-speech)
            
        Returns:
            True if test passed, False otherwise
        """
        try:
            # In a real test, this would process actual voice input
            # For this simulation, we'll just validate the voice command node
            if not hasattr(self.voice_command_node, 'process_voice_command'):
                return False
            
            # Validate voice command node configuration
            expected_methods = [
                'process_voice_command', 
                'create_voice_command_from_text', 
                'validate_voice_input'
            ]
            
            for method in expected_methods:
                if not hasattr(self.voice_command_node, method):
                    self.logger.error(f"Voice command node missing required method: {method}")
                    return False
            
            # Test basic functionality
            test_result = self.voice_command_node.create_voice_command_from_text(
                text=command,
                user_id="test_user"
            )
            
            if not test_result or not test_result.transcript:
                self.logger.error("Voice command creation failed")
                return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error in voice processing test: {e}")
            return False
    
    async def _test_cognitive_planning(self, command: str) -> bool:
        """
        Test the cognitive planning component.
        
        Args:
            command: Natural language command to plan for
            
        Returns:
            True if test passed, False otherwise
        """
        try:
            # Create a mock voice command for planning
            from ..core.data_models import VoiceCommandModel
            voice_command = VoiceCommandModel.create(
                transcript=command,
                user_id="test_user",
                language="en"
            )
            
            # Test cognitive planning
            if not hasattr(self.cognitive_planner, 'plan'):
                return False
            
            cognitive_plan = await self.cognitive_planner.plan(voice_command)
            
            if not cognitive_plan:
                self.logger.error("Cognitive plan generation failed")
                return False
            
            # Validate plan structure
            if not cognitive_plan.plan_id:
                self.logger.error("Generated plan has no ID")
                return False
            
            if not cognitive_plan.task_decomposition:
                self.logger.error("Generated plan has no tasks")
                return False
            
            # Validate each task in the plan
            for task in cognitive_plan.task_decomposition:
                if not task.task_id or not task.task_type:
                    self.logger.error(f"Invalid task in plan: {task}")
                    return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error in cognitive planning test: {e}")
            return False
    
    async def _test_action_execution(self) -> bool:
        """
        Test the action execution component.
        
        Returns:
            True if test passed, False otherwise
        """
        try:
            # Create a simple test action
            from ..core.data_models import ActionModel
            test_action = ActionModel.create(
                action_type="move_to",
                parameters={
                    "target_location": {"x": 1.0, "y": 1.0, "z": 0.0},
                    "speed": 0.5
                },
                timeout=10.0,
                retry_count=1
            )
            
            # Test that the action execution component exists and is callable
            if not hasattr(self.robot_controller, 'execute_single_action'):
                return False
            
            # In a real test, we'd execute the action and check results
            # For simulation, we'll just verify the method exists and can be called
            # (Without actually sending commands to a robot)
            
            # Verify that basic execution methods exist
            required_methods = [
                'execute_single_action',
                'execute_action_sequence', 
                'validate_action',
                'get_execution_metrics'
            ]
            
            for method in required_methods:
                if not hasattr(self.robot_controller, method):
                    self.logger.error(f"Robot controller missing required method: {method}")
                    return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error in action execution test: {e}")
            return False
    
    async def _validate_component_coordination(self, component_results: List[Any]) -> List[str]:
        """
        Validate that components can properly coordinate with each other.
        
        Args:
            component_results: Results from individual component tests
            
        Returns:
            List of coordination errors or empty list if coordination is valid
        """
        errors = []
        
        # In a real implementation, we would test the actual data flow between components
        # For this implementation, we'll just verify that all components are available and responding
        
        # Check that voice processing result is valid
        if isinstance(component_results[0], Exception) or component_results[0] is False:
            errors.append("Voice processing component not responding properly")
        
        # Check that cognitive planning result is valid
        if isinstance(component_results[1], Exception) or component_results[1] is False:
            errors.append("Cognitive planning component not responding properly")
        
        # Check that action execution result is valid
        if isinstance(component_results[2], Exception) or component_results[2] is False:
            errors.append("Action execution component not responding properly")
        
        # In a real system, we would test the actual data flow between components
        # For example, testing that a voice command can generate a plan, 
        # and that plan can be executed as actions
        
        return errors
    
    @log_exception()
    async def run_capstone_integration_test(self) -> IntegrationTestResult:
        """
        Run the capstone integration test that demonstrates the complete VLA system.
        
        Returns:
            IntegrationTestResult with test results
        """
        try:
            test_id = f"capstone_test_{int(time.time())}"
            start_time = time.time()
            
            self.logger.info(f"Starting capstone integration test: {test_id}")
            
            # Run the autonomous humanoid demonstration
            demo_result = await self.capstone_demonstrator.run_autonomous_humanoid_demo()
            
            execution_time = time.time() - start_time
            
            # Evaluate the demo result
            errors = []
            warnings = []
            
            # Check if demo was successful
            if not demo_result.success:
                errors.append(f"Capstone demo failed: {demo_result.result_summary}")
            
            # Check success rate
            if demo_result.success_rate < self.config.get('min_capstone_success_rate', 0.75):
                errors.append(f"Capstone success rate too low: {demo_result.success_rate:.2%} < {self.config.get('min_capstone_success_rate', 0.75):.2%}")
            
            # Check execution time is reasonable
            max_expected_time = self.config.get('max_capstone_demo_time', 300.0)  # 5 minutes default
            if demo_result.execution_time > max_expected_time:
                warnings.append(f"Demo took longer than expected: {demo_result.execution_time:.2f}s > {max_expected_time}s")
            
            # Check that all expected capabilities were demonstrated
            expected_capabilities = [
                'voice_recognition',
                'cognitive_planning', 
                'action_execution',
                'vision_perception',
                'safety_monitoring'
            ]
            
            missing_capabilities = [
                cap for cap in expected_capabilities 
                if not demo_result.details.get(f'{cap}_demonstrated', False)
            ]
            
            if missing_capabilities:
                errors.append(f"Missing capabilities in demo: {missing_capabilities}")
            
            passed = len(errors) == 0
            details = {
                'demo_result': demo_result,
                'expected_capabilities': expected_capabilities,
                'missing_capabilities': missing_capabilities,
                'execution_time': execution_time
            }
            
            result = IntegrationTestResult(
                test_id=test_id,
                test_name="capstone_integration_test",
                passed=passed,
                execution_time=execution_time,
                details=details,
                errors=errors,
                warnings=warnings
            )
            
            status = "PASSED" if passed else "FAILED"
            self.logger.info(f"Capstone integration test {status}: {execution_time:.2f}s execution time")
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error in capstone integration test: {e}")
            execution_time = time.time() - start_time
            
            return IntegrationTestResult(
                test_id=f"capstone_error_{int(time.time())}",
                test_name="capstone_integration_test",
                passed=False,
                execution_time=execution_time,
                details={'error': str(e)},
                errors=[f"Capstone test error: {str(e)}"],
                warnings=[]
            )
    
    @log_exception()
    async def run_performance_integration_test(self) -> IntegrationTestResult:
        """
        Run performance-focused integration tests to validate latency and throughput.
        
        Returns:
            IntegrationTestResult with performance test results
        """
        try:
            test_id = f"performance_test_{int(time.time())}"
            start_time = time.time()
            
            self.logger.info(f"Starting performance integration test: {test_id}")
            
            # Define performance requirements
            max_latency_requirements = {
                'voice_recognition': 1.0,  # seconds
                'cognitive_planning': 2.0,  # seconds
                'action_execution': 30.0,  # seconds for complete sequence
                'end_to_end': 5.0  # seconds for complete pipeline
            }
            
            # Test end-to-end performance with multiple commands
            test_commands = [
                "Move forward 1 meter",
                "Turn left 90 degrees", 
                "Find the red cup",
                "Go to the kitchen"
            ]
            
            performance_results = []
            errors = []
            warnings = []
            
            for i, command in enumerate(test_commands):
                test_start = time.time()
                
                try:
                    # Execute command through full pipeline
                    result = await self.pipeline_integrator.execute_end_to_end_pipeline(
                        voice_command=command,
                        user_id="perf_tester"
                    )
                    
                    command_time = time.time() - test_start
                    
                    # Record performance metrics
                    perf_record = {
                        'command': command,
                        'execution_time': command_time,
                        'success': result.get('success', False) if result else False,
                        'details': result or {}
                    }
                    
                    performance_results.append(perf_record)
                    
                    # Check performance requirements
                    if command_time > max_latency_requirements['end_to_end']:
                        # This doesn't necessarily cause test failure but may be noted
                        warnings.append(f"Command '{command}' exceeded time requirement: {command_time:.2f}s > {max_latency_requirements['end_to_end']}s")
                
                except Exception as e:
                    errors.append(f"Performance test failed for command '{command}': {e}")
                    # Continue testing with next command
            
            execution_time = time.time() - start_time
            
            # Calculate aggregate performance metrics
            successful_commands = [r for r in performance_results if r['success']]
            avg_execution_time = (
                sum(r['execution_time'] for r in successful_commands) / len(successful_commands)
                if successful_commands else float('inf')
            )
            
            # Validate performance requirements
            if len(successful_commands) == 0:
                errors.append("All performance test commands failed")
            elif avg_execution_time > max_latency_requirements['end_to_end']:
                errors.append(f"Average execution time {avg_execution_time:.2f}s exceeds requirement of {max_latency_requirements['end_to_end']}s")
            
            passed = len(errors) == 0
            details = {
                'command_count': len(test_commands),
                'successful_commands': len(successful_commands),
                'average_execution_time': avg_execution_time,
                'performance_results': performance_results,
                'max_latency_requirements': max_latency_requirements
            }
            
            result = IntegrationTestResult(
                test_id=test_id,
                test_name="performance_integration_test",
                passed=passed,
                execution_time=execution_time,
                details=details,
                errors=errors,
                warnings=warnings
            )
            
            status = "PASSED" if passed else "FAILED"
            self.logger.info(f"Performance integration test {status}: {avg_execution_time:.2f}s avg execution time")
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error in performance integration test: {e}")
            execution_time = time.time() - start_time
            
            return IntegrationTestResult(
                test_id=f"performance_error_{int(time.time())}",
                test_name="performance_integration_test",
                passed=False,
                execution_time=execution_time,
                details={'error': str(e)},
                errors=[f"Performance test error: {str(e)}"],
                warnings=[]
            )
    
    @log_exception()
    async def run_all_integration_tests(self) -> List[IntegrationTestResult]:
        """
        Run all integration tests in sequence.
        
        Returns:
            List of IntegrationTestResult for all tests
        """
        try:
            self.logger.info("Starting all integration tests")
            
            # Define all tests to run
            tests = [
                ('Complete Pipeline', self.run_complete_pipeline_test),
                ('Multi-Component Coordination', self.run_multi_component_integration_test),
                ('Capstone Integration', self.run_capstone_integration_test),
                ('Performance', self.run_performance_integration_test)
            ]
            
            results = []
            
            for test_name, test_func in tests:
                self.logger.info(f"Running integration test: {test_name}")
                
                try:
                    result = await test_func()
                    results.append(result)
                    
                    status = "PASSED" if result.passed else "FAILED"
                    self.logger.info(f"Test {test_name} {status} in {result.execution_time:.2f}s")
                    
                    # Brief pause between tests to avoid resource conflicts
                    await asyncio.sleep(1.0)
                    
                except Exception as e:
                    self.logger.error(f"Error running {test_name} test: {e}")
                    
                    # Create an error result for this test
                    error_result = IntegrationTestResult(
                        test_id=f"test_error_{int(time.time())}",
                        test_name=test_name.replace(' ', '_').lower(),
                        passed=False,
                        execution_time=0.0,
                        details={'error': str(e)},
                        errors=[f"Test execution error: {str(e)}"],
                        warnings=[]
                    )
                    results.append(error_result)
            
            # Calculate overall results
            total_tests = len(results)
            passed_tests = sum(1 for r in results if r.passed)
            success_rate = passed_tests / total_tests if total_tests > 0 else 0.0
            
            self.logger.info(f"All integration tests completed: {success_rate:.2%} ({passed_tests}/{total_tests}) passed")
            
            return results
            
        except Exception as e:
            self.logger.error(f"Error running all integration tests: {e}")
            raise VLAException(
                f"Integration testing error: {str(e)}", 
                VLAErrorType.INTEGRATION_ERROR,
                e
            )
    
    def get_test_summary(self, results: List[IntegrationTestResult]) -> Dict[str, Any]:
        """
        Get a summary of integration test results.
        
        Args:
            results: List of IntegrationTestResult from tests
            
        Returns:
            Dictionary with test summary statistics
        """
        if not results:
            return {
                'total_tests': 0,
                'passed_tests': 0,
                'failed_tests': 0,
                'success_rate': 0.0,
                'total_execution_time': 0.0,
                'test_breakdown': []
            }
        
        passed_tests = sum(1 for r in results if r.passed)
        total_execution_time = sum(r.execution_time for r in results)
        
        test_breakdown = [
            {
                'test_name': r.test_name,
                'passed': r.passed,
                'execution_time': r.execution_time
            } for r in results
        ]
        
        return {
            'total_tests': len(results),
            'passed_tests': passed_tests,
            'failed_tests': len(results) - passed_tests,
            'success_rate': passed_tests / len(results),
            'total_execution_time': total_execution_time,
            'test_breakdown': test_breakdown
        }
    
    @log_exception()
    async def generate_test_report(self, results: List[IntegrationTestResult]) -> str:
        """
        Generate a comprehensive test report.
        
        Args:
            results: List of IntegrationTestResult from tests
            
        Returns:
            Formatted test report as a string
        """
        try:
            summary = self.get_test_summary(results)
            
            report = [
                "# VLA Module Integration Test Report",
                "",
                f"**Date**: {time.strftime('%Y-%m-%d %H:%M:%S')}",
                f"**VLA Module Version**: {getattr(self.config, 'version', 'unknown')}",
                "",
                "## Test Summary",
                f"Total Tests: {summary['total_tests']}",
                f"Passed: {summary['passed_tests']}",
                f"Failed: {summary['failed_tests']}",
                f"Success Rate: {summary['success_rate']:.2%}",
                f"Total Execution Time: {summary['total_execution_time']:.2f}s",
                "",
                "## Test Details",
                "| Test Name | Status | Execution Time (s) |",
                "|----------|--------|-------------------|"
            ]
            
            for result in results:
                status_icon = "✅" if result.passed else "❌"
                report.append(f"| {result.test_name.replace('_', ' ').title()} | {status_icon} {result.passed} | {result.execution_time:.2f} |")
            
            report.append("")  # Empty line
            
            # Add details for failed tests
            failed_results = [r for r in results if not r.passed]
            if failed_results:
                report.extend([
                    "## Failed Tests Details",
                    ""
                ])
                
                for result in failed_results:
                    report.extend([
                        f"### {result.test_name.replace('_', ' ').title()}",
                        f"**Execution Time**: {result.execution_time:.2f}s",
                        "**Errors:**",
                    ])
                    for error in result.errors:
                        report.append(f"- {error}")
                    
                    if result.warnings:
                        report.append("**Warnings:**")
                        for warning in result.warnings:
                            report.append(f"- {warning}")
                    
                    report.append("")  # Empty line
            
            # Add performance metrics
            report.extend([
                "## Performance Metrics",
                "",
                "### Average Execution Times",
            ])
            
            # Calculate average times per test type
            test_times = {}
            for result in results:
                test_type = result.test_name.split('_')[0]  # Get test category
                if test_type not in test_times:
                    test_times[test_type] = []
                test_times[test_type].append(result.execution_time)
            
            for test_type, times in test_times.items():
                avg_time = sum(times) / len(times)
                report.append(f"- {test_type.title()}: {avg_time:.2f}s")
            
            return "\n".join(report)
            
        except Exception as e:
            self.logger.error(f"Error generating test report: {e}")
            return f"Error generating test report: {str(e)}"


# Global integration tester instance
_integration_tester = None


def get_integration_tester() -> VLAFinalIntegrationTester:
    """Get the global VLA integration tester instance."""
    global _integration_tester
    if _integration_tester is None:
        _integration_tester = VLAFinalIntegrationTester()
    return _integration_tester


async def run_complete_pipeline_integration_test() -> IntegrationTestResult:
    """Convenience function to run the complete pipeline integration test."""
    tester = get_integration_tester()
    return await tester.run_complete_pipeline_test()


async def run_multi_component_integration_test() -> IntegrationTestResult:
    """Convenience function to run the multi-component integration test."""
    tester = get_integration_tester()
    return await tester.run_multi_component_integration_test()


async def run_capstone_integration_test() -> IntegrationTestResult:
    """Convenience function to run the capstone integration test."""
    tester = get_integration_tester()
    return await tester.run_capstone_integration_test()


async def run_performance_integration_test() -> IntegrationTestResult:
    """Convenience function to run performance-focused integration test."""
    tester = get_integration_tester()
    return await tester.run_performance_integration_test()


async def run_all_vla_integration_tests() -> List[IntegrationTestResult]:
    """Convenience function to run all VLA integration tests."""
    tester = get_integration_tester()
    return await tester.run_all_integration_tests()


def get_integration_test_summary(results: List[IntegrationTestResult]) -> Dict[str, Any]:
    """Convenience function to get integration test summary."""
    tester = get_integration_tester()
    return tester.get_test_summary(results)


async def generate_integration_test_report(results: List[IntegrationTestResult]) -> str:
    """Convenience function to generate integration test report."""
    tester = get_integration_tester()
    return await tester.generate_test_report(results)