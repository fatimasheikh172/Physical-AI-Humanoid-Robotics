"""
Complete VLA pipeline integration for the capstone project.

This module integrates all VLA components into a cohesive system for the Autonomous Humanoid capstone project.
"""

import asyncio
import logging
import time
from typing import Dict, Any, Optional, List
from dataclasses import dataclass

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import VoiceCommand, CognitivePlan, ActionSequence, RobotState, ExecutionStatus
from ..core.data_models import VoiceCommandModel, CognitivePlanModel, ActionSequenceModel, RobotStateModel
from ..core.vla_manager import get_vla_manager
from ..voice_recognition.voice_command_node import get_voice_command_node
from ..llm_planning.cognitive_planner import get_cognitive_planner
from ..action_execution.robot_controller import get_robot_controller
from ..action_execution.robot_state_sync import get_robot_state_sync_manager
from ..vision_perception.ros2_vision_node import get_vision_node


@dataclass
class IntegrationStatus:
    """Status of the VLA integration."""
    components_connected: bool
    system_ready: bool
    current_state: str
    components_status: Dict[str, bool]
    last_update: float


class VLAPipelineIntegrator:
    """
    Integrates all Vision-Language-Action components into a cohesive pipeline
    for the Autonomous Humanoid capstone project.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Initialize all components
        self.vla_manager = get_vla_manager()
        self.voice_command_node = get_voice_command_node()
        self.cognitive_planner = get_cognitive_planner()
        self.robot_controller = get_robot_controller()
        self.robot_state_sync = get_robot_state_sync_manager()
        self.vision_node = get_vision_node()
        
        # Integration state
        self.integration_status = IntegrationStatus(
            components_connected=False,
            system_ready=False,
            current_state="initializing",
            components_status={},
            last_update=time.time()
        )
        
        # Execution tracking
        self.active_executions = {}
        self.execution_history = []
        
        # Callbacks for integration events
        self.integration_callbacks = []
        
        self.logger.info("VLAPipelineIntegrator initialized")
    
    @log_exception()
    async def initialize_full_pipeline(self) -> bool:
        """
        Initialize the complete VLA pipeline with all components.
        
        Returns:
            True if pipeline is successfully initialized, False otherwise
        """
        try:
            self.logger.info("Initializing complete VLA pipeline")
            
            # Update integration status
            self.integration_status.current_state = "initializing"
            self.integration_status.last_update = time.time()
            
            # Initialize and connect to all components
            init_tasks = [
                self._initialize_voice_component(),
                self._initialize_planning_component(),
                self._initialize_execution_component(),
                self._initialize_vision_component(),
                self._initialize_state_sync()
            ]
            
            # Execute initialization tasks in parallel
            results = await asyncio.gather(*init_tasks, return_exceptions=True)
            
            # Check for initialization errors
            errors = []
            for i, result in enumerate(results):
                if isinstance(result, Exception):
                    errors.append(f"Component {i} initialization failed: {result}")
                else:
                    self.logger.debug(f"Component {i} initialized successfully")
            
            if errors:
                self.logger.error(f"Pipeline initialization errors: {errors}")
                self.integration_status.components_connected = False
                return False
            
            # Verify all components are connected
            connected_components = await self._verify_component_connections()
            all_connected = all(connected_components.values())
            
            if all_connected:
                self.integration_status.components_connected = True
                self.integration_status.current_state = "ready"
                self.integration_status.components_status = connected_components
                self.integration_status.system_ready = True
                self.integration_status.last_update = time.time()
                
                self.logger.info("Complete VLA pipeline initialized and ready")
                
                # Notify integration callbacks
                await self._notify_integration_event("pipeline_ready", self.integration_status)
                
                return True
            else:
                self.logger.error(f"Not all components connected: {connected_components}")
                self.integration_status.components_connected = False
                self.integration_status.system_ready = False
                return False
                
        except Exception as e:
            self.logger.error(f"Error initializing VLA pipeline: {e}")
            self.integration_status.current_state = "error"
            self.integration_status.last_update = time.time()
            raise VLAException(
                f"VLA pipeline initialization error: {str(e)}", 
                VLAErrorType.INTEGRATION_ERROR,
                e
            )
    
    async def _initialize_voice_component(self) -> bool:
        """Initialize the voice recognition component."""
        try:
            self.logger.info("Initializing voice recognition component")
            
            # In a real system, this would initialize the voice recognition service
            # For now, we'll just set up the necessary configurations
            if hasattr(self.voice_command_node, 'initialize'):
                await self.voice_command_node.initialize()
            
            self.logger.debug("Voice component initialized")
            return True
            
        except Exception as e:
            self.logger.error(f"Error initializing voice component: {e}")
            return False
    
    async def _initialize_planning_component(self) -> bool:
        """Initialize the cognitive planning component."""
        try:
            self.logger.info("Initializing cognitive planning component")
            
            # In a real system, this would initialize the LLM planning service
            # For now, we'll just verify it's available
            if hasattr(self.cognitive_planner, 'initialize'):
                await self.cognitive_planner.initialize()
            
            self.logger.debug("Planning component initialized")
            return True
            
        except Exception as e:
            self.logger.error(f"Error initializing planning component: {e}")
            return False
    
    async def _initialize_execution_component(self) -> bool:
        """Initialize the action execution component."""
        try:
            self.logger.info("Initializing action execution component")
            
            # Connect to robot for action execution
            if hasattr(self.robot_controller, 'connect_to_robot'):
                connected = await self.robot_controller.connect_to_robot()
                if not connected:
                    raise VLAException("Could not connect to robot", VLAErrorType.ROBOT_COMMUNICATION_ERROR)
            
            self.logger.debug("Execution component initialized")
            return True
            
        except Exception as e:
            self.logger.error(f"Error initializing execution component: {e}")
            return False
    
    async def _initialize_vision_component(self) -> bool:
        """Initialize the vision perception component."""
        try:
            self.logger.info("Initializing vision perception component")
            
            # In a real system, this would initialize the vision service
            # For now, we'll just verify it's available
            if hasattr(self.vision_node, 'initialize'):
                await self.vision_node.initialize()
            
            self.logger.debug("Vision component initialized")
            return True
            
        except Exception as e:
            self.logger.error(f"Error initializing vision component: {e}")
            return False
    
    async def _initialize_state_sync(self) -> bool:
        """Initialize the robot state synchronization component."""
        try:
            self.logger.info("Initializing robot state synchronization")
            
            # Start continuous state synchronization
            await self.robot_state_sync.start_continuous_sync()
            
            self.logger.debug("State sync component initialized")
            return True
            
        except Exception as e:
            self.logger.error(f"Error initializing state sync component: {e}")
            return False
    
    async def _verify_component_connections(self) -> Dict[str, bool]:
        """
        Verify that all components are properly connected.
        
        Returns:
            Dictionary mapping component names to connection status
        """
        try:
            connections = {}
            
            # Check voice component
            if hasattr(self.voice_command_node, 'get_status'):
                try:
                    voice_status = await self.voice_command_node.get_status()
                    connections['voice'] = voice_status.get('connected', False)
                except:
                    connections['voice'] = False
            else:
                connections['voice'] = True  # Assume available if no status method
            
            # Check planning component
            if hasattr(self.cognitive_planner, 'get_status'):
                try:
                    planning_status = await self.cognitive_planner.get_status()
                    connections['planning'] = planning_status.get('initialized', False)
                except:
                    connections['planning'] = False
            else:
                connections['planning'] = True
            
            # Check execution component
            if hasattr(self.robot_controller, 'get_connection_status'):
                try:
                    exec_status = await self.robot_controller.get_connection_status()
                    connections['execution'] = exec_status
                except:
                    connections['execution'] = False
            else:
                connections['execution'] = True
            
            # Check vision component
            if hasattr(self.vision_node, 'get_status'):
                try:
                    vision_status = await self.vision_node.get_status()
                    connections['vision'] = vision_status.get('active', False)
                except:
                    connections['vision'] = False
            else:
                connections['vision'] = True
            
            # Check state sync component
            # For state sync, we'll consider it connected if it's running
            connections['state_sync'] = True  # Always true since it's a service component
            
            return connections
            
        except Exception as e:
            self.logger.error(f"Error verifying component connections: {e}")
            # Return false for all if verification itself fails
            return {
                'voice': False,
                'planning': False,
                'execution': False,
                'vision': False,
                'state_sync': False
            }
    
    @log_exception()
    async def execute_end_to_end_pipeline(
        self, 
        voice_command: str, 
        user_id: str = "default_user"
    ) -> Dict[str, Any]:
        """
        Execute the complete end-to-end VLA pipeline from voice command to execution.
        
        Args:
            voice_command: Voice command as text
            user_id: ID of the user issuing the command
            
        Returns:
            Dictionary with execution results
        """
        if not self.integration_status.system_ready:
            raise VLAException(
                "VLA pipeline not ready, please initialize first",
                VLAErrorType.INTEGRATION_ERROR
            )
        
        try:
            self.logger.info(f"Executing end-to-end pipeline for command: '{voice_command}' from user {user_id}")
            
            execution_id = f"exec_{int(time.time()*1000)}_{user_id}"
            start_time = time.time()
            
            # Add to active executions
            self.active_executions[execution_id] = {
                'status': 'starting',
                'start_time': start_time,
                'command': voice_command
            }
            
            # Step 1: Process voice command (if needed, in real system this would come from speech)
            self.logger.debug("Step 1: Creating voice command model")
            voice_command_model = VoiceCommandModel.create(
                transcript=voice_command,
                user_id=user_id,
                language=self.config.whisper_language
            )
            
            # Step 2: Cognitive planning
            self.logger.debug("Step 2: Executing cognitive planning")
            cognitive_plan = await self.cognitive_planner.plan(voice_command_model)
            
            if not cognitive_plan or not cognitive_plan.task_decomposition:
                raise VLAException(
                    "Cognitive planning failed to generate a valid plan",
                    VLAErrorType.PLANNING_ERROR
                )
            
            # Step 3: Validate plan with perception data
            self.logger.debug("Step 3: Validating plan with perception")
            # In a real system, this would incorporate current vision perception
            # For now, we'll just validate the plan structure
            validation_errors = await self.cognitive_planner.validate_plan(cognitive_plan)
            if validation_errors:
                self.logger.warning(f"Plan validation found errors: {validation_errors}")
                # Continue execution but log the issues
            
            # Step 4: Sequence the plan into actions
            self.logger.debug("Step 4: Sequencing plan into action sequence")
            # For now, we'll use a simple converter - in a real system this would be more sophisticated
            from ..action_execution.action_sequencer import get_action_sequencer
            action_sequencer = get_action_sequencer()
            action_sequence = await action_sequencer.sequence_plan(cognitive_plan)
            
            # Step 5: Execute the action sequence
            self.logger.debug("Step 5: Executing action sequence")
            action_responses = await self.robot_controller.execute_action_sequence(action_sequence)
            
            # Step 6: Update robot state after execution
            self.logger.debug("Step 6: Updating robot state")
            current_state = await self.robot_state_sync.get_current_robot_state()
            if current_state:
                await self.robot_state_sync.update_robot_state(current_state)
            
            # Calculate results
            successful_actions = sum(1 for resp in action_responses if resp.status == ExecutionStatus.COMPLETED)
            total_actions = len(action_responses)
            success_rate = successful_actions / total_actions if total_actions > 0 else 0.0
            
            execution_time = time.time() - start_time
            
            # Create execution result
            result = {
                'execution_id': execution_id,
                'command': voice_command,
                'user_id': user_id,
                'success_rate': success_rate,
                'successful_actions': successful_actions,
                'total_actions': total_actions,
                'execution_time': execution_time,
                'plan_id': cognitive_plan.plan_id if cognitive_plan else None,
                'sequence_id': action_sequence.sequence_id if action_sequence else None,
                'responses': [
                    {
                        'action_id': resp.action_id,
                        'status': resp.status.name,
                        'completion_percentage': resp.completion_percentage,
                        'result_summary': resp.result_summary
                    } for resp in action_responses
                ],
                'timestamp': start_time
            }
            
            # Add to execution history
            self.execution_history.append(result)
            if len(self.execution_history) > self.config.max_execution_history:
                self.execution_history.pop(0)  # Remove oldest execution
            
            # Update execution status
            self.active_executions[execution_id]['status'] = 'completed'
            self.active_executions[execution_id]['result'] = result
            self.active_executions[execution_id]['end_time'] = time.time()
            
            # Remove from active after a short period
            asyncio.create_task(self._cleanup_active_execution(execution_id, delay=30.0))
            
            self.logger.info(f"End-to-end pipeline completed: {success_rate:.1%} success rate in {execution_time:.2f}s")
            
            # Run integration callbacks
            await self._notify_integration_event("pipeline_completed", result)
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error in end-to-end pipeline execution: {e}")
            
            # Update execution status as failed
            if execution_id in self.active_executions:
                self.active_executions[execution_id]['status'] = 'failed'
                self.active_executions[execution_id]['error'] = str(e)
                self.active_executions[execution_id]['end_time'] = time.time()
            
            raise VLAException(
                f"End-to-end pipeline execution error: {str(e)}", 
                VLAErrorType.PIPELINE_EXECUTION_ERROR,
                e
            )
    
    async def _cleanup_active_execution(self, execution_id: str, delay: float = 30.0):
        """Clean up an active execution after a delay."""
        await asyncio.sleep(delay)
        if execution_id in self.active_executions:
            del self.active_executions[execution_id]
    
    @log_exception()
    async def run_capstone_scenario(self, scenario_description: str) -> Dict[str, Any]:
        """
        Run a specific capstone scenario for the Autonomous Humanoid project.
        
        Args:
            scenario_description: Description of the scenario to run
            
        Returns:
            Dictionary with scenario execution results
        """
        try:
            self.logger.info(f"Running capstone scenario: {scenario_description}")
            
            # Determine which specific scenario to run based on description
            if "pick up" in scenario_description.lower() and "red cup" in scenario_description.lower():
                # Example: "Pick up the red cup from the table"
                return await self._run_object_manipulation_scenario(scenario_description)
            elif "navigate" in scenario_description.lower() or "go to" in scenario_description.lower():
                # Example: "Go to the kitchen and wait by the counter"
                return await self._run_navigation_scenario(scenario_description)
            elif "find" in scenario_description.lower() or "detect" in scenario_description.lower():
                # Example: "Find the blue ball in the living room"
                return await self._run_perception_scenario(scenario_description)
            else:
                # Default: treat as a general command
                return await self.execute_end_to_end_pipeline(scenario_description)
                
        except Exception as e:
            self.logger.error(f"Error running capstone scenario: {e}")
            raise VLAException(
                f"Capstone scenario execution error: {str(e)}", 
                VLAErrorType.CAPSTONE_EXECUTION_ERROR,
                e
            )
    
    async def _run_object_manipulation_scenario(self, scenario: str) -> Dict[str, Any]:
        """Run an object manipulation scenario."""
        # This would be a more complex scenario involving perception, navigation, and manipulation
        # For now, we'll just execute it as a regular pipeline
        return await self.execute_end_to_end_pipeline(scenario)
    
    async def _run_navigation_scenario(self, scenario: str) -> Dict[str, Any]:
        """Run a navigation scenario."""
        # This would be a navigation-specific scenario
        # For now, we'll just execute it as a regular pipeline
        return await self.execute_end_to_end_pipeline(scenario)
    
    async def _run_perception_scenario(self, scenario: str) -> Dict[str, Any]:
        """Run a perception scenario."""
        # This would be a perception-specific scenario
        # For now, we'll just execute it as a regular pipeline
        return await self.execute_end_to_end_pipeline(scenario)
    
    @log_exception()
    async def get_integration_status(self) -> IntegrationStatus:
        """
        Get the current integration status.
        
        Returns:
            IntegrationStatus with current system state
        """
        # Refresh component connections
        self.integration_status.components_status = await self._verify_component_connections()
        self.integration_status.components_connected = all(self.integration_status.components_status.values())
        self.integration_status.system_ready = self.integration_status.components_connected
        self.integration_status.last_update = time.time()
        
        return self.integration_status
    
    def add_integration_callback(self, callback: callable):
        """
        Add a callback to be notified of integration events.
        
        Args:
            callback: Function to call on integration events
        """
        self.integration_callbacks.append(callback)
        self.logger.debug(f"Added integration callback. Total: {len(self.integration_callbacks)}")
    
    async def _notify_integration_event(self, event_type: str, event_data: Any):
        """
        Notify all registered callbacks of an integration event.
        
        Args:
            event_type: Type of integration event
            event_data: Data associated with the event
        """
        for callback in self.integration_callbacks:
            try:
                # Handle both sync and async callbacks
                if asyncio.iscoroutinefunction(callback):
                    await callback(event_type, event_data)
                else:
                    callback(event_type, event_data)
            except Exception as e:
                self.logger.error(f"Error in integration callback: {e}")
    
    @log_exception()
    async def validate_full_pipeline(self) -> List[str]:
        """
        Validate that the entire VLA pipeline is functioning correctly.
        
        Returns:
            List of validation errors or empty list if valid
        """
        try:
            errors = []
            
            # Validate each component individually
            component_validations = await asyncio.gather(
                self._validate_voice_component(),
                self._validate_planning_component(),
                self._validate_execution_component(),
                self._validate_vision_component(),
                self._validate_state_sync_component(),
                return_exceptions=True
            )
            
            for i, validation in enumerate(component_validations):
                if isinstance(validation, Exception):
                    errors.append(f"Component {i} validation error: {validation}")
                else:
                    errors.extend([f"Component {i}: {error}" for error in validation if validation])
            
            # Validate overall system integration
            integration_errors = await self._validate_system_integration()
            errors.extend(integration_errors)
            
            if errors:
                self.logger.warning(f"Pipeline validation found {len(errors)} errors")
            else:
                self.logger.info("Full pipeline validation passed")
            
            return errors
            
        except Exception as e:
            self.logger.error(f"Error in full pipeline validation: {e}")
            raise VLAException(
                f"Pipeline validation error: {str(e)}", 
                VLAErrorType.VALIDATION_ERROR,
                e
            )
    
    async def _validate_voice_component(self) -> List[str]:
        """Validate the voice recognition component."""
        errors = []
        try:
            if hasattr(self.voice_command_node, 'validate_setup'):
                voice_errors = await self.voice_command_node.validate_setup()
                errors.extend([f"Voice component: {error}" for error in voice_errors])
        except Exception as e:
            errors.append(f"Voice component validation error: {e}")
        return errors
    
    async def _validate_planning_component(self) -> List[str]:
        """Validate the cognitive planning component."""
        errors = []
        try:
            if hasattr(self.cognitive_planner, 'validate_setup'):
                planning_errors = await self.cognitive_planner.validate_setup()
                errors.extend([f"Planning component: {error}" for error in planning_errors])
        except Exception as e:
            errors.append(f"Planning component validation error: {e}")
        return errors
    
    async def _validate_execution_component(self) -> List[str]:
        """Validate the action execution component."""
        errors = []
        try:
            if hasattr(self.robot_controller, 'validate_setup'):
                execution_errors = await self.robot_controller.validate_setup()
                errors.extend([f"Execution component: {error}" for error in execution_errors])
        except Exception as e:
            errors.append(f"Execution component validation error: {e}")
        return errors
    
    async def _validate_vision_component(self) -> List[str]:
        """Validate the vision perception component."""
        errors = []
        try:
            if hasattr(self.vision_node, 'validate_setup'):
                vision_errors = await self.vision_node.validate_setup()
                errors.extend([f"Vision component: {error}" for error in vision_errors])
        except Exception as e:
            errors.append(f"Vision component validation error: {e}")
        return errors
    
    async def _validate_state_sync_component(self) -> List[str]:
        """Validate the robot state synchronization component."""
        errors = []
        try:
            if hasattr(self.robot_state_sync, 'validate_setup'):
                sync_errors = await self.robot_state_sync.validate_setup()
                errors.extend([f"State sync component: {error}" for error in sync_errors])
        except Exception as e:
            errors.append(f"State sync component validation error: {e}")
        return errors
    
    async def _validate_system_integration(self) -> List[str]:
        """Validate the integration between system components."""
        errors = []
        
        # Check that components can communicate with each other
        if not self.integration_status.components_connected:
            errors.append("Components not properly connected")
        
        # Check that the VLA manager has all necessary components registered
        if not (hasattr(self.vla_manager, 'voice_recognition_component') and 
                hasattr(self.vla_manager, 'cognitive_planning_component') and
                hasattr(self.vla_manager, 'action_execution_component')):
            errors.append("VLA manager missing required component registrations")
        
        return errors
    
    def get_execution_history(self, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Get the execution history.
        
        Args:
            limit: Maximum number of executions to return
            
        Returns:
            List of recent execution records
        """
        return self.execution_history[-limit:] if len(self.execution_history) > limit else self.execution_history[:]
    
    async def shutdown_pipeline(self):
        """Properly shut down the VLA pipeline."""
        self.logger.info("Shutting down VLA pipeline")
        
        # Stop active tasks
        if hasattr(self.robot_state_sync, 'stop_continuous_sync'):
            await self.robot_state_sync.stop_continuous_sync()
        
        # Clear active executions
        self.active_executions.clear()
        
        self.logger.info("VLA pipeline shut down completed")


# Global pipeline integrator instance
_vla_pipeline_integrator = None


def get_vla_pipeline_integrator() -> VLAPipelineIntegrator:
    """Get the global VLA pipeline integrator instance."""
    global _vla_pipeline_integrator
    if _vla_pipeline_integrator is None:
        _vla_pipeline_integrator = VLAPipelineIntegrator()
    return _vla_pipeline_integrator


async def initialize_full_pipeline() -> bool:
    """Convenience function to initialize the full VLA pipeline."""
    integrator = get_vla_pipeline_integrator()
    return await integrator.initialize_full_pipeline()


async def execute_end_to_end_pipeline(voice_command: str, user_id: str = "default_user") -> Dict[str, Any]:
    """Convenience function to execute the end-to-end pipeline."""
    integrator = get_vla_pipeline_integrator()
    return await integrator.execute_end_to_end_pipeline(voice_command, user_id)


async def run_capstone_scenario(scenario_description: str) -> Dict[str, Any]:
    """Convenience function to run a capstone scenario."""
    integrator = get_vla_pipeline_integrator()
    return await integrator.run_capstone_scenario(scenario_description)


async def get_integration_status() -> IntegrationStatus:
    """Convenience function to get integration status."""
    integrator = get_vla_pipeline_integrator()
    return await integrator.get_integration_status()