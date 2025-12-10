"""
VLA Manager for the Vision-Language-Action (VLA) module.

This class coordinates the different components of the VLA system:
- Voice recognition
- Cognitive planning 
- Action execution
- Vision perception
"""

import asyncio
import logging
from typing import Optional, Dict, Any, List
from dataclasses import dataclass

from .message_types import (
    VoiceCommand, CognitivePlan, ActionSequence, ActionResponse, 
    RobotState, VisionObservation, CommandType
)
from .config import get_config


class VLAManager:
    """Main manager class that coordinates the VLA system components"""
    
    def __init__(self):
        self.config = get_config()
        self.logger = logging.getLogger(self.__class__.__name__)
        
        # Components will be injected later
        self.voice_recognition_component = None
        self.cognitive_planning_component = None
        self.action_execution_component = None
        self.vision_perception_component = None
        
        # Internal state
        self.current_robot_state: Optional[RobotState] = None
        self.command_history: List[VoiceCommand] = []
        self.plan_history: List[CognitivePlan] = []
        self.action_sequence_history: List[ActionSequence] = []
        
        self.logger.info("VLA Manager initialized")
    
    def register_voice_recognition_component(self, component):
        """Register the voice recognition component"""
        self.voice_recognition_component = component
        self.logger.info("Voice recognition component registered")
    
    def register_cognitive_planning_component(self, component):
        """Register the cognitive planning component"""
        self.cognitive_planning_component = component
        self.logger.info("Cognitive planning component registered")
    
    def register_action_execution_component(self, component):
        """Register the action execution component"""
        self.action_execution_component = component
        self.logger.info("Action execution component registered")
    
    def register_vision_perception_component(self, component):
        """Register the vision perception component"""
        self.vision_perception_component = component
        self.logger.info("Vision perception component registered")
    
    async def process_voice_command(self, audio_data: bytes) -> Optional[CognitivePlan]:
        """
        Process a voice command through the full VLA pipeline:
        Voice Recognition -> Cognitive Planning -> Action Execution
        """
        try:
            self.logger.info("Processing voice command...")
            
            # Step 1: Voice recognition
            if not self.voice_recognition_component:
                raise Exception("Voice recognition component not registered")
            
            transcript = await self.voice_recognition_component.recognize(audio_data)
            self.logger.info(f"Recognized: {transcript}")
            
            # Create voice command object
            voice_command = VoiceCommand(
                command_id=f"cmd_{len(self.command_history)}",
                transcript=transcript,
                timestamp=0,  # Will be set by the component
                confidence=0.9,  # Will be set by the component
                user_id="default_user",  # Will be passed from context
                language=self.config.whisper_language
            )
            self.command_history.append(voice_command)
            
            # Step 2: Cognitive planning
            if not self.cognitive_planning_component:
                raise Exception("Cognitive planning component not registered")
            
            cognitive_plan = await self.cognitive_planning_component.plan(voice_command)
            self.plan_history.append(cognitive_plan)
            self.logger.info(f"Generated plan with {len(cognitive_plan.task_decomposition)} tasks")
            
            # Step 3: Action execution (asynchronous)
            if not self.action_execution_component:
                raise Exception("Action execution component not registered")
            
            action_sequence = self._convert_plan_to_action_sequence(cognitive_plan)
            self.action_sequence_history.append(action_sequence)
            
            # Execute the action sequence
            response = await self.action_execution_component.execute(action_sequence)
            
            self.logger.info("Voice command processing completed")
            return cognitive_plan
            
        except Exception as e:
            self.logger.error(f"Error processing voice command: {e}")
            raise
    
    async def process_text_command(self, text: str, user_id: str = "default_user") -> Optional[CognitivePlan]:
        """
        Process a text command through the VLA pipeline (without voice recognition)
        """
        try:
            self.logger.info(f"Processing text command: {text}")
            
            # Create voice command object directly from text
            voice_command = VoiceCommand(
                command_id=f"cmd_{len(self.command_history)}",
                transcript=text,
                timestamp=0,  # Will be set by the component
                confidence=1.0,  # Text has perfect confidence
                user_id=user_id,
                language=self.config.whisper_language
            )
            self.command_history.append(voice_command)
            
            # Step 1: Cognitive planning
            if not self.cognitive_planning_component:
                raise Exception("Cognitive planning component not registered")
            
            cognitive_plan = await self.cognitive_planning_component.plan(voice_command)
            self.plan_history.append(cognitive_plan)
            self.logger.info(f"Generated plan with {len(cognitive_plan.task_decomposition)} tasks")
            
            # Step 2: Action execution (asynchronous)
            if not self.action_execution_component:
                raise Exception("Action execution component not registered")
            
            action_sequence = self._convert_plan_to_action_sequence(cognitive_plan)
            self.action_sequence_history.append(action_sequence)
            
            # Execute the action sequence
            response = await self.action_execution_component.execute(action_sequence)
            
            self.logger.info("Text command processing completed")
            return cognitive_plan
            
        except Exception as e:
            self.logger.error(f"Error processing text command: {e}")
            raise
    
    def _convert_plan_to_action_sequence(self, plan: CognitivePlan) -> ActionSequence:
        """Convert a cognitive plan to an executable action sequence"""
        from .message_types import Action, ActionSequence
        from .data_models import ActionModel, ActionSequenceModel, TaskModel

        # Convert tasks to actions
        actions = []
        for i, task in enumerate(plan.task_decomposition):
            # Create an action based on the task
            action = Action(
                action_id=f"action_{plan.plan_id}_{i}",
                action_type=task.task_type,
                parameters=task.parameters or {}
            )
            actions.append(action)

        # Create action sequence
        action_sequence = ActionSequence(
            sequence_id=f"seq_{plan.plan_id}",
            plan_id=plan.plan_id,
            actions=actions
        )

        return action_sequence
    
    async def get_robot_state(self) -> Optional[RobotState]:
        """Get the current robot state"""
        # This would normally come from the robot's state publisher
        # For now, return the internally tracked state
        return self.current_robot_state
    
    async def update_robot_state(self, robot_state: RobotState):
        """Update the current robot state"""
        self.current_robot_state = robot_state
        self.logger.debug(f"Robot state updated: {robot_state.state_id}")
    
    async def request_vision_observation(self) -> Optional[VisionObservation]:
        """Request a vision observation from the vision perception component"""
        if not self.vision_perception_component:
            self.logger.warning("Vision perception component not registered")
            return None
        
        return await self.vision_perception_component.observe()
    
    async def get_command_history(self) -> List[VoiceCommand]:
        """Get the command history"""
        return self.command_history
    
    async def get_plan_history(self) -> List[CognitivePlan]:
        """Get the plan history"""
        return self.plan_history
    
    async def get_action_sequence_history(self) -> List[ActionSequence]:
        """Get the action sequence history"""
        return self.action_sequence_history
    
    async def process_text_command(self, text_command: str, user_id: str = "default_user"):
        """
        Process a text command through the full VLA pipeline.
        This is the entry point for the cognitive planning component.

        Args:
            text_command: Text command to process
            user_id: ID of the user issuing the command
        """
        try:
            self.logger.info(f"Processing text command via VLA manager: {text_command}")

            # Validate inputs
            if not text_command.strip():
                raise ValueError("Text command cannot be empty")

            # Step 1: Create voice command representation (since we're bypassing voice recognition)
            from .data_models import VoiceCommandModel
            voice_command = VoiceCommandModel.create(
                transcript=text_command,
                user_id=user_id,
                language=self.config.whisper_language
            )
            self.command_history.append(voice_command)

            # Step 2: Cognitive planning
            if not self.cognitive_planning_component:
                raise Exception("Cognitive planning component not registered")

            cognitive_plan = await self.cognitive_planning_component.plan(voice_command)
            self.plan_history.append(cognitive_plan)
            self.logger.info(f"Generated plan with {len(cognitive_plan.task_decomposition)} tasks")

            # Step 3: Action execution (asynchronous)
            if not self.action_execution_component:
                raise Exception("Action execution component not registered")

            action_sequence = self._convert_plan_to_action_sequence(cognitive_plan)
            self.action_sequence_history.append(action_sequence)

            # Execute the action sequence
            response = await self.action_execution_component.execute(action_sequence)

            self.logger.info("Text command processing completed")
            return cognitive_plan

        except Exception as e:
            self.logger.error(f"Error processing text command: {e}")
            raise

    async def process_cognitive_plan(self, cognitive_plan: 'CognitivePlanModel') -> 'ActionSequenceModel':
        """
        Process a cognitive plan through the action execution component.

        Args:
            cognitive_plan: CognitivePlanModel to execute

        Returns:
            ActionSequenceModel that was executed
        """
        try:
            self.logger.info(f"Processing cognitive plan via VLA manager: {cognitive_plan.plan_id}")

            # Validate cognitive plan
            if not cognitive_plan.plan_id:
                raise ValueError("Cognitive plan must have a plan_id")

            if not cognitive_plan.task_decomposition:
                raise ValueError("Cognitive plan must have tasks to execute")

            # Validate plan against robot capabilities
            if self.cognitive_planning_component and hasattr(self.cognitive_planning_component, 'validate_plan'):
                validation_errors = await self.cognitive_planning_component.validate_plan(cognitive_plan)
                if validation_errors:
                    raise ValueError(f"Plan validation failed: {validation_errors}")

            # Register the plan in history
            self.plan_history.append(cognitive_plan)

            # Convert plan to action sequence
            action_sequence = self._convert_plan_to_action_sequence(cognitive_plan)
            self.action_sequence_history.append(action_sequence)

            # Validate action sequence
            if self.action_execution_component and hasattr(self.action_execution_component, 'validate_sequence'):
                validation_errors = await self.action_execution_component.validate_sequence(action_sequence)
                if validation_errors:
                    raise ValueError(f"Action sequence validation failed: {validation_errors}")

            # Execute the action sequence
            if not self.action_execution_component:
                raise Exception("Action execution component not registered")

            response = await self.action_execution_component.execute(action_sequence)

            self.logger.info(f"Plan {cognitive_plan.plan_id} executed, generating action sequence {action_sequence.sequence_id}")
            return action_sequence

        except Exception as e:
            self.logger.error(f"Error processing cognitive plan: {e}")
            raise

    async def register_components(self,
                                  voice_recognition_component=None,
                                  cognitive_planning_component=None,
                                  action_execution_component=None,
                                  vision_perception_component=None):
        """
        Register all the components at once.

        Args:
            voice_recognition_component: Voice recognition component
            cognitive_planning_component: Cognitive planning component
            action_execution_component: Action execution component
            vision_perception_component: Vision perception component
        """
        if voice_recognition_component:
            self.register_voice_recognition_component(voice_recognition_component)

        if cognitive_planning_component:
            self.register_cognitive_planning_component(cognitive_planning_component)

        if action_execution_component:
            self.register_action_execution_component(action_execution_component)

        if vision_perception_component:
            self.register_vision_perception_component(vision_perception_component)

        self.logger.info("All VLA components registered")

    def reset_history(self):
        """Reset all history"""
        self.command_history = []
        self.plan_history = []
        self.action_sequence_history = []
        self.logger.info("History reset")


# Global VLA manager instance
_vla_manager = None


def get_vla_manager() -> 'VLAManager':
    """Get the global VLA manager instance"""
    global _vla_manager
    if _vla_manager is None:
        _vla_manager = VLAManager()
    return _vla_manager