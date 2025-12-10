"""
Capstone project demonstration scenario for the Vision-Language-Action (VLA) module.

This module implements the full Autonomous Humanoid demonstration scenario
that showcases the complete VLA pipeline: voice command → cognitive planning → action execution.
"""

import asyncio
import logging
import time
from typing import Dict, List, Any, Optional
from dataclasses import dataclass

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import VoiceCommand, CognitivePlan, ActionSequence, ExecutionStatus
from ..core.data_models import VoiceCommandModel, CognitivePlanModel, ActionSequenceModel
from ..core.vla_manager import get_vla_manager
from ..voice_recognition.voice_command_node import get_voice_command_node
from ..llm_planning.cognitive_planner import get_cognitive_planner
from ..action_execution.robot_controller import get_robot_controller
from ..vision_perception.ros2_vision_node import get_vision_node
from .full_pipeline_integrator import get_vla_pipeline_integrator


@dataclass
class CapstoneDemoResult:
    """Result of a capstone demonstration run."""
    demo_id: str
    scenario: str
    start_time: float
    end_time: float
    execution_time: float
    success: bool
    success_rate: float
    total_actions: int
    successful_actions: int
    failed_actions: int
    errors: List[str]
    details: Dict[str, Any]


class VLACapstoneDemonstrator:
    """
    Manages and executes the VLA capstone project demonstration.
    This showcases the entire pipeline from voice command to robotic action.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Initialize system components
        self.vla_manager = get_vla_manager()
        self.pipeline_integrator = get_vla_pipeline_integrator()
        self.voice_command_node = get_voice_command_node()
        self.cognitive_planner = get_cognitive_planner()
        self.robot_controller = get_robot_controller()
        self.vision_node = get_vision_node()
        
        # Demo state tracking
        self.active_demos = {}
        self.demo_history = []
        
        # Available demo scenarios
        self.demo_scenarios = {
            'navigation': {
                'description': 'Robot navigates to a specified location',
                'commands': ['Go to the kitchen', 'Move to the office', 'Navigate to the entrance'],
                'expected_outcomes': ['robot navigates to target location', 'navigation completes safely']
            },
            'object_interaction': {
                'description': 'Robot identifies and interacts with objects',
                'commands': ['Find the red cup and pick it up', 'Detect the blue ball', 'Grasp the pen on the table'],
                'expected_outcomes': ['object detected', 'manipulation successful if applicable']
            },
            'complex_task': {
                'description': 'Robot performs multi-step complex tasks',
                'commands': [
                    'Go to John\'s desk, find a pen, and bring it to me', 
                    'Navigate to the kitchen, find a cup, pick it up, and place it on the counter',
                    'Move to the conference room, detect the presentation pointer, and wait there'
                ],
                'expected_outcomes': [
                    'multi-step task completed',
                    'all subtasks executed in correct order', 
                    'robot returns to user if required'
                ]
            },
            'autonomous_helper': {
                'description': 'Robot acts as an autonomous helper in daily tasks',
                'commands': [
                    'Clean up the dining table', 
                    'Organize the books on the shelf',
                    'Bring me the newspaper from the living room'
                ],
                'expected_outcomes': [
                    'environment tidied',
                    'objects properly organized',
                    'requested item brought to user'
                ]
            }
        }
        
        self.logger.info("VLACapstoneDemonstrator initialized")
    
    @log_exception()
    async def run_demo_scenario(self, scenario_name: str, custom_command: Optional[str] = None) -> CapstoneDemoResult:
        """
        Run a specific demo scenario.
        
        Args:
            scenario_name: Name of the scenario to run (navigation, object_interaction, etc.)
            custom_command: Optional custom command instead of predefined ones
            
        Returns:
            CapstoneDemoResult with execution results
        """
        try:
            demo_id = f"demo_{int(time.time())}_{scenario_name[:3]}"
            start_time = time.time()
            
            self.logger.info(f"Starting capstone demo {demo_id} for scenario: {scenario_name}")
            
            # Validate scenario
            if scenario_name not in self.demo_scenarios and not custom_command:
                raise VLAException(
                    f"Unknown demo scenario: {scenario_name}. Available scenarios: {list(self.demo_scenarios.keys())}", 
                    VLAErrorType.VALIDATION_ERROR
                )
            
            # Determine command to use
            if custom_command:
                command = custom_command
            else:
                # Select a command from the scenario
                command = self.demo_scenarios[scenario_name]['commands'][0]  # Use first command as default
            
            # Add to active demos
            self.active_demos[demo_id] = {
                'status': 'running',
                'scenario': scenario_name,
                'command': command,
                'start_time': start_time
            }
            
            self.logger.debug(f"Executing command: '{command}'")
            
            try:
                # Execute the full pipeline
                result = await self.pipeline_integrator.execute_end_to_end_pipeline(
                    voice_command=command,
                    user_id=f"demo_{demo_id}"
                )
                
                # Process results
                total_actions = result.get('total_actions', 0)
                successful_actions = result.get('successful_actions', 0)
                failed_actions = total_actions - successful_actions
                success_rate = result.get('success_rate', 0.0)
                
                demo_result = CapstoneDemoResult(
                    demo_id=demo_id,
                    scenario=scenario_name,
                    start_time=start_time,
                    end_time=time.time(),
                    execution_time=time.time() - start_time,
                    success=success_rate >= 0.8,  # Consider successful if 80%+ actions succeeded
                    success_rate=success_rate,
                    total_actions=total_actions,
                    successful_actions=successful_actions,
                    failed_actions=failed_actions,
                    errors=[],
                    details={'pipeline_result': result}
                )
                
                self.logger.info(f"Demo {demo_id} completed: {success_rate:.1%} success rate in {demo_result.execution_time:.2f}s")
                
            except Exception as e:
                # Handle execution errors
                self.logger.error(f"Demo {demo_id} execution failed: {e}")
                
                demo_result = CapstoneDemoResult(
                    demo_id=demo_id,
                    scenario=scenario_name,
                    start_time=start_time,
                    end_time=time.time(),
                    execution_time=time.time() - start_time,
                    success=False,
                    success_rate=0.0,
                    total_actions=0,
                    successful_actions=0,
                    failed_actions=0,
                    errors=[str(e)],
                    details={'exception': str(e), 'type': type(e).__name__}
                )
            
            # Update active demo status
            self.active_demos[demo_id]['status'] = 'completed'
            self.active_demos[demo_id]['result'] = demo_result
            
            # Add to history
            self.demo_history.append(demo_result)
            if len(self.demo_history) > self.config.max_demo_history:
                self.demo_history.pop(0)
            
            # Clean up from active demos after a delay
            asyncio.create_task(self._remove_from_active_demos(demo_id, delay=60.0))
            
            return demo_result
            
        except Exception as e:
            self.logger.error(f"Error running demo scenario {scenario_name}: {e}")
            
            # Create error result
            error_result = CapstoneDemoResult(
                demo_id=f"error_{int(time.time())}",
                scenario=scenario_name,
                start_time=start_time,
                end_time=time.time(),
                execution_time=time.time() - start_time,
                success=False,
                success_rate=0.0,
                total_actions=0,
                successful_actions=0,
                failed_actions=0,
                errors=[str(e)],
                details={'exception': str(e), 'type': type(e).__name__}
            )
            
            # Add to history
            self.demo_history.append(error_result)
            if len(self.demo_history) > self.config.max_demo_history:
                self.demo_history.pop(0)
            
            raise VLAException(
                f"Demo scenario execution error: {str(e)}", 
                VLAErrorType.CAPSTONE_EXECUTION_ERROR,
                e
            )
    
    async def _remove_from_active_demos(self, demo_id: str, delay: float = 60.0):
        """Remove a demo from active tracking after a delay."""
        await asyncio.sleep(delay)
        if demo_id in self.active_demos:
            del self.active_demos[demo_id]
    
    @log_exception()
    async def run_autonomous_humanoid_demo(self) -> CapstoneDemoResult:
        """
        Run the full Autonomous Humanoid capstone demonstration.
        
        This is the main capstone project demonstrating the complete VLA pipeline:
        Voice command → Cognitive planning → Action execution → Vision feedback
        """
        try:
            self.logger.info("Running Autonomous Humanoid capstone demonstration")
            
            # Use a complex command that demonstrates all VLA capabilities
            capstone_command = "Autonomous Humanoid: Go to the kitchen, find the red cup on the counter, pick it up, and bring it to me in the living room"
            
            # Run the command through the full pipeline
            result = await self.run_demo_scenario('complex_task', capstone_command)
            
            self.logger.info(f"Autonomous Humanoid demo completed with result: {'SUCCESS' if result.success else 'FAILED'}")
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error in Autonomous Humanoid demo: {e}")
            raise VLAException(
                f"Autonomous Humanoid demo error: {str(e)}", 
                VLAErrorType.CAPSTONE_EXECUTION_ERROR,
                e
            )
    
    @log_exception()
    async def run_multiple_demos(self, scenario_names: List[str], count_per_scenario: int = 1) -> List[CapstoneDemoResult]:
        """
        Run multiple demonstrations across different scenarios.
        
        Args:
            scenario_names: List of scenario names to run
            count_per_scenario: Number of times to run each scenario
            
        Returns:
            List of CapstoneDemoResult for all runs
        """
        try:
            self.logger.info(f"Running {count_per_scenario} demos for each of {len(scenario_names)} scenarios")
            
            all_results = []
            
            for scenario in scenario_names:
                for i in range(count_per_scenario):
                    self.logger.info(f"Running {scenario} demo {i+1}/{count_per_scenario}")
                    
                    try:
                        result = await self.run_demo_scenario(scenario)
                        all_results.append(result)
                        
                        self.logger.debug(f"Completed {scenario} demo {i+1}, success: {result.success}")
                        
                        # Brief pause between demos
                        await asyncio.sleep(2.0)
                        
                    except Exception as e:
                        self.logger.error(f"Error in {scenario} demo {i+1}: {e}")
                        
                        # Add error result
                        error_result = CapstoneDemoResult(
                            demo_id=f"error_{int(time.time())}_{scenario}_{i+1}",
                            scenario=scenario,
                            start_time=time.time(),
                            end_time=time.time(),
                            execution_time=0.0,
                            success=False,
                            success_rate=0.0,
                            total_actions=0,
                            successful_actions=0,
                            failed_actions=0,
                            errors=[str(e)],
                            details={'exception': str(e)}
                        )
                        all_results.append(error_result)
            
            # Calculate overall statistics
            total_runs = len(all_results)
            successful_runs = sum(1 for r in all_results if r.success)
            avg_execution_time = sum(r.execution_time for r in all_results) / total_runs if total_runs > 0 else 0.0
            avg_success_rate = sum(r.success_rate for r in all_results) / total_runs if total_runs > 0 else 0.0
            
            self.logger.info(
                f"Multiple demo run completed: {successful_runs}/{total_runs} successful, "
                f"avg execution time: {avg_execution_time:.2f}s, avg success rate: {avg_success_rate:.1%}"
            )
            
            return all_results
            
        except Exception as e:
            self.logger.error(f"Error in multiple demo run: {e}")
            raise VLAException(
                f"Multiple demo run error: {str(e)}", 
                VLAErrorType.CAPSTONE_EXECUTION_ERROR,
                e
            )
    
    def get_demo_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about demo performance.
        
        Returns:
            Dictionary with demo statistics
        """
        if not self.demo_history:
            return {
                'total_demos': 0,
                'successful_demos': 0,
                'success_rate': 0.0,
                'average_execution_time': 0.0,
                'recent_demos': []
            }
        
        total_demos = len(self.demo_history)
        successful_demos = sum(1 for demo in self.demo_history if demo.success)
        success_rate = successful_demos / total_demos if total_demos > 0 else 0.0
        avg_execution_time = sum(demo.execution_time for demo in self.demo_history) / total_demos if total_demos > 0 else 0.0
        
        # Get recent demos (last 10)
        recent_demos = [
            {
                'demo_id': demo.demo_id,
                'scenario': demo.scenario,
                'success': demo.success,
                'success_rate': demo.success_rate,
                'execution_time': demo.execution_time
            }
            for demo in self.demo_history[-10:]
        ]
        
        return {
            'total_demos': total_demos,
            'successful_demos': successful_demos,
            'success_rate': success_rate,
            'average_execution_time': avg_execution_time,
            'recent_demos': recent_demos
        }
    
    async def validate_demo_readiness(self) -> List[str]:
        """
        Validate that the system is ready for the capstone demonstration.
        
        Returns:
            List of readiness issues or empty list if ready
        """
        try:
            self.logger.info("Validating system readiness for capstone demonstration")
            
            issues = []
            
            # Check if all components are initialized and connected
            if not hasattr(self, 'pipeline_integrator') or self.pipeline_integrator is None:
                issues.append("Pipeline integrator not initialized")
            else:
                # Validate pipeline components
                integrator_issues = await self.pipeline_integrator.validate_full_pipeline()
                issues.extend([f"Pipeline: {issue}" for issue in integrator_issues])
            
            # Check voice recognition component
            if not hasattr(self, 'voice_command_node') or self.voice_command_node is None:
                issues.append("Voice command node not available")
            
            # Check cognitive planning component
            if not hasattr(self, 'cognitive_planner') or self.cognitive_planner is None:
                issues.append("Cognitive planner not available")
            
            # Check action execution component
            if not hasattr(self, 'robot_controller') or self.robot_controller is None:
                issues.append("Robot controller not available")
            
            # Check vision perception component
            if not hasattr(self, 'vision_node') or self.vision_node is None:
                issues.append("Vision node not available")
            
            # Check VLA manager
            if not hasattr(self, 'vla_manager') or self.vla_manager is None:
                issues.append("VLA manager not available")
            
            # Check robot state
            try:
                robot_state = await self.vla_manager.get_robot_state()
                if not robot_state:
                    issues.append("Robot state not available")
            except Exception as e:
                issues.append(f"Could not get robot state: {str(e)}")
            
            if issues:
                self.logger.warning(f"Demo readiness validation found issues: {issues[:3]}{'...' if len(issues) > 3 else ''}")
            else:
                self.logger.info("System is ready for capstone demonstration")
            
            return issues
            
        except Exception as e:
            self.logger.error(f"Error validating demo readiness: {e}")
            raise VLAException(
                f"Demo readiness validation error: {str(e)}", 
                VLAErrorType.VALIDATION_ERROR,
                e
            )
    
    def get_active_demos(self) -> Dict[str, Any]:
        """
        Get information about active demonstrations.
        
        Returns:
            Dictionary with active demo information
        """
        return {
            demo_id: {
                'scenario': details['scenario'],
                'command': details['command'],
                'start_time': details['start_time'],
                'duration': time.time() - details['start_time']
            }
            for demo_id, details in self.active_demos.items()
        }
    
    def get_demo_scenarios(self) -> Dict[str, Dict[str, Any]]:
        """
        Get available demo scenarios.
        
        Returns:
            Dictionary of available scenarios with descriptions
        """
        return self.demo_scenarios.copy()


# Global capstone demonstrator instance
_capstone_demonstrator = None


def get_capstone_demonstrator() -> VLACapstoneDemonstrator:
    """Get the global VLA capstone demonstrator instance."""
    global _capstone_demonstrator
    if _capstone_demonstrator is None:
        _capstone_demonstrator = VLACapstoneDemonstrator()
    return _capstone_demonstrator


async def run_capstone_demo(scenario_name: str, custom_command: Optional[str] = None) -> CapstoneDemoResult:
    """Convenience function to run a capstone demo."""
    demonstrator = get_capstone_demonstrator()
    return await demonstrator.run_demo_scenario(scenario_name, custom_command)


async def run_autonomous_humanoid_demo() -> CapstoneDemoResult:
    """Convenience function to run the main Autonomous Humanoid demo."""
    demonstrator = get_capstone_demonstrator()
    return await demonstrator.run_autonomous_humanoid_demo()


async def run_multiple_capstone_demos(scenario_names: List[str], count_per_scenario: int = 1) -> List[CapstoneDemoResult]:
    """Convenience function to run multiple capstone demos."""
    demonstrator = get_capstone_demonstrator()
    return await demonstrator.run_multiple_demos(scenario_names, count_per_scenario)


def get_capstone_demo_statistics() -> Dict[str, Any]:
    """Convenience function to get demo statistics."""
    demonstrator = get_capstone_demonstrator()
    return demonstrator.get_demo_statistics()


async def validate_capstone_readiness() -> List[str]:
    """Convenience function to validate demo readiness."""
    demonstrator = get_capstone_demonstrator()
    return await demonstrator.validate_demo_readiness()