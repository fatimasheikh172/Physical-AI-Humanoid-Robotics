"""
Action execution state management for the Vision-Language-Action (VLA) module.

This module manages the state of ongoing action executions, allowing for
proper tracking, monitoring, and control of actions as they execute.
"""

import asyncio
import logging
import time
import uuid
from typing import Dict, List, Any, Optional, Callable
from enum import Enum
from dataclasses import dataclass

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import Action, ActionSequence, ActionResponse, ExecutionStatus
from ..core.data_models import ActionModel, ActionSequenceModel


class ExecutionState(str, Enum):
    """Enumeration of possible execution states."""
    INITIALIZED = "initialized"
    PENDING = "pending"
    VALIDATING = "validating"
    APPROVED = "approved"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"
    PAUSED = "paused"
    CANCELLED = "cancelled"
    SUSPENDED = "suspended"


@dataclass
class ActionExecutionState:
    """State record for a single action execution."""
    action_id: str
    sequence_id: str
    status: ExecutionStatus
    state: ExecutionState
    start_time: float
    end_time: Optional[float] = None
    error_message: Optional[str] = None
    progress: float = 0.0
    current_step: str = "initialized"
    estimated_duration: float = 0.0
    remaining_attempts: int = 1
    last_updated: float = 0.0


@dataclass
class ActionSequenceExecutionState:
    """State record for an action sequence execution."""
    sequence_id: str
    status: ExecutionStatus
    state: ExecutionState
    start_time: float
    end_time: Optional[float] = None
    error_message: Optional[str] = None
    progress: float = 0.0
    current_action_index: int = 0
    total_actions: int = 0
    completed_actions: int = 0
    failed_actions: int = 0
    last_updated: float = 0.0
    active_action_id: Optional[str] = None


class ActionExecutionStateManager:
    """
    Manages the state of action executions, providing tracking, monitoring,
    and control capabilities for ongoing operations.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Storage for execution states
        self.action_states: Dict[str, ActionExecutionState] = {}
        self.sequence_states: Dict[str, ActionSequenceExecutionState] = {}
        
        # Callback functions for state changes
        self.state_change_callbacks: Dict[str, List[Callable]] = {
            'action': [],
            'sequence': []
        }
        
        # Cleanup settings
        self.completed_state_retention_time = 300.0  # 5 minutes
        self.failed_state_retention_time = 3600.0   # 1 hour
        
        self.logger.info("ActionExecutionStateManager initialized")
    
    @log_exception()
    async def register_action_execution(self, action: ActionModel, sequence_id: str = None) -> str:
        """
        Register a new action for execution tracking.
        
        Args:
            action: ActionModel to track
            sequence_id: Optional sequence ID if action is part of a sequence
            
        Returns:
            Action ID for the registered execution
        """
        try:
            action_id = action.action_id or f"action_{uuid.uuid4().hex[:8]}"
            
            # Create execution state
            state = ActionExecutionState(
                action_id=action_id,
                sequence_id=sequence_id,
                status=ExecutionStatus.PENDING,
                state=ExecutionState.INITIALIZED,
                start_time=time.time(),
                remaining_attempts=action.retry_count + 1,  # Include original attempt
                estimated_duration=action.timeout
            )
            
            # Store state
            self.action_states[action_id] = state
            
            # Notify callbacks
            await self._notify_state_change('action', state)
            
            self.logger.info(f"Registered action execution: {action_id}")
            return action_id
            
        except Exception as e:
            self.logger.error(f"Error registering action execution: {e}")
            raise VLAException(
                f"Error registering action execution: {str(e)}", 
                VLAErrorType.ACTION_EXECUTION_ERROR,
                e
            )
    
    @log_exception()
    async def register_sequence_execution(self, action_sequence: ActionSequenceModel) -> str:
        """
        Register a new action sequence for execution tracking.
        
        Args:
            action_sequence: ActionSequenceModel to track
            
        Returns:
            Sequence ID for the registered execution
        """
        try:
            sequence_id = action_sequence.sequence_id or f"seq_{uuid.uuid4().hex[:8]}"
            
            # Create execution state
            state = ActionSequenceExecutionState(
                sequence_id=sequence_id,
                status=action_sequence.execution_status if hasattr(action_sequence, 'execution_status') else ExecutionStatus.PENDING,
                state=ExecutionState.INITIALIZED,
                start_time=time.time(),
                total_actions=len(action_sequence.actions),
                current_action_index=0,
                completed_actions=0,
                failed_actions=0
            )
            
            # Calculate estimated duration based on individual actions
            estimated_duration = 0.0
            for action in action_sequence.actions:
                estimated_duration += action.timeout
                if hasattr(action, 'retry_count'):
                    estimated_duration += (action.retry_count * action.timeout * 0.1)  # Additional time for retries
            state.estimated_duration = estimated_duration
            
            # Store state
            self.sequence_states[sequence_id] = state
            
            # Notify callbacks
            await self._notify_state_change('sequence', state)
            
            self.logger.info(f"Registered sequence execution: {sequence_id} with {len(action_sequence.actions)} actions")
            return sequence_id
            
        except Exception as e:
            self.logger.error(f"Error registering sequence execution: {e}")
            raise VLAException(
                f"Error registering sequence execution: {str(e)}", 
                VLAErrorType.ACTION_EXECUTION_ERROR,
                e
            )
    
    @log_exception()
    async def update_action_state(
        self, 
        action_id: str, 
        new_status: ExecutionStatus = None, 
        new_state: ExecutionState = None,
        progress: float = None,
        current_step: str = None,
        error_message: str = None,
        **kwargs
    ) -> bool:
        """
        Update the state of an executing action.
        
        Args:
            action_id: ID of the action to update
            new_status: New execution status
            new_state: New execution state
            progress: Progress percentage (0.0 to 1.0)
            current_step: Current step in the action
            error_message: Error message if applicable
            **kwargs: Additional state attributes to update
            
        Returns:
            True if update successful, False otherwise
        """
        try:
            if action_id not in self.action_states:
                self.logger.warning(f"Trying to update unknown action state: {action_id}")
                return False
            
            state = self.action_states[action_id]
            
            # Update provided attributes
            updates_made = False
            if new_status is not None:
                state.status = new_status
                updates_made = True
            
            if new_state is not None:
                state.state = new_state
                updates_made = True
                
                # Set end_time if entering a terminal state
                if new_state in [ExecutionState.COMPLETED, ExecutionState.FAILED, ExecutionState.CANCELLED]:
                    state.end_time = time.time()
            
            if progress is not None:
                state.progress = max(0.0, min(1.0, progress))  # Clamp to [0, 1]
                updates_made = True
            
            if current_step is not None:
                state.current_step = current_step
                updates_made = True
            
            if error_message is not None:
                state.error_message = error_message
                updates_made = True
            
            # Update additional attributes from kwargs
            for key, value in kwargs.items():
                if hasattr(state, key):
                    setattr(state, key, value)
                    updates_made = True
                else:
                    self.logger.warning(f"Unknown attribute for ActionExecutionState: {key}")
            
            if updates_made:
                state.last_updated = time.time()
                
                # Notify callbacks
                await self._notify_state_change('action', state)
            
            self.logger.debug(f"Updated action state {action_id}: {state.state}")
            return True
            
        except Exception as e:
            self.logger.error(f"Error updating action state {action_id}: {e}")
            return False
    
    @log_exception()
    async def update_sequence_state(
        self, 
        sequence_id: str, 
        new_status: ExecutionStatus = None, 
        new_state: ExecutionState = None,
        progress: float = None,
        current_action_index: int = None,
        completed_actions: int = None,
        failed_actions: int = None,
        active_action_id: str = None,
        error_message: str = None,
        **kwargs
    ) -> bool:
        """
        Update the state of an executing sequence.
        
        Args:
            sequence_id: ID of the sequence to update
            new_status: New execution status
            new_state: New execution state
            progress: Progress percentage (0.0 to 1.0)
            current_action_index: Index of current action
            completed_actions: Number of completed actions
            failed_actions: Number of failed actions
            active_action_id: ID of currently active action
            error_message: Error message if applicable
            **kwargs: Additional state attributes to update
            
        Returns:
            True if update successful, False otherwise
        """
        try:
            if sequence_id not in self.sequence_states:
                self.logger.warning(f"Trying to update unknown sequence state: {sequence_id}")
                return False
            
            state = self.sequence_states[sequence_id]
            
            # Update provided attributes
            updates_made = False
            if new_status is not None:
                state.status = new_status
                updates_made = True
            
            if new_state is not None:
                state.state = new_state
                updates_made = True
                
                # Set end_time if entering a terminal state
                if new_state in [ExecutionState.COMPLETED, ExecutionState.FAILED, ExecutionState.CANCELLED]:
                    state.end_time = time.time()
            
            if progress is not None:
                state.progress = max(0.0, min(1.0, progress))  # Clamp to [0, 1]
                updates_made = True
            
            if current_action_index is not None:
                state.current_action_index = current_action_index
                updates_made = True
            
            if completed_actions is not None:
                state.completed_actions = completed_actions
                updates_made = True
            
            if failed_actions is not None:
                state.failed_actions = failed_actions
                updates_made = True
                
            if active_action_id is not None:
                state.active_action_id = active_action_id
                updates_made = True
            
            if error_message is not None:
                state.error_message = error_message
                updates_made = True
            
            # Update additional attributes from kwargs
            for key, value in kwargs.items():
                if hasattr(state, key):
                    setattr(state, key, value)
                    updates_made = True
                else:
                    self.logger.warning(f"Unknown attribute for ActionSequenceExecutionState: {key}")
            
            if updates_made:
                state.last_updated = time.time()
                
                # Notify callbacks
                await self._notify_state_change('sequence', state)
            
            self.logger.debug(f"Updated sequence state {sequence_id}: {state.state}")
            return True
            
        except Exception as e:
            self.logger.error(f"Error updating sequence state {sequence_id}: {e}")
            return False
    
    async def get_action_state(self, action_id: str) -> Optional[ActionExecutionState]:
        """
        Get the state of a specific action execution.
        
        Args:
            action_id: ID of the action to get state for
            
        Returns:
            ActionExecutionState or None if not found
        """
        return self.action_states.get(action_id)
    
    async def get_sequence_state(self, sequence_id: str) -> Optional[ActionSequenceExecutionState]:
        """
        Get the state of a specific action sequence execution.
        
        Args:
            sequence_id: ID of the sequence to get state for
            
        Returns:
            ActionSequenceExecutionState or None if not found
        """
        return self.sequence_states.get(sequence_id)
    
    async def get_sequence_actions_states(self, sequence_id: str) -> List[ActionExecutionState]:
        """
        Get all action states for a specific sequence.
        
        Args:
            sequence_id: ID of the sequence to get action states for
            
        Returns:
            List of ActionExecutionState for actions in the sequence
        """
        sequence_state = self.sequence_states.get(sequence_id)
        if not sequence_state:
            return []
        
        # Find all action states that belong to this sequence
        action_states = [
            state for state in self.action_states.values()
            if state.sequence_id == sequence_id
        ]
        
        # Sort by action ID to maintain order
        action_states.sort(key=lambda s: s.action_id)
        
        return action_states
    
    @log_exception()
    async def cancel_action_execution(self, action_id: str) -> bool:
        """
        Cancel a specific action execution.
        
        Args:
            action_id: ID of the action to cancel
            
        Returns:
            True if cancellation initiated, False otherwise
        """
        try:
            if action_id not in self.action_states:
                self.logger.warning(f"Trying to cancel unknown action: {action_id}")
                return False
            
            state = self.action_states[action_id]
            old_state = state.state
            old_status = state.status
            
            # Update to cancelled state
            state.state = ExecutionState.CANCELLED
            state.status = ExecutionStatus.FAILED
            state.end_time = time.time()
            state.error_message = "Action cancelled by user request"
            
            # Notify callbacks
            await self._notify_state_change('action', state)
            
            self.logger.info(f"Cancelled action {action_id}, was {old_state} with status {old_status}")
            return True
            
        except Exception as e:
            self.logger.error(f"Error cancelling action {action_id}: {e}")
            return False
    
    @log_exception()
    async def cancel_sequence_execution(self, sequence_id: str) -> bool:
        """
        Cancel a specific sequence execution.
        
        Args:
            sequence_id: ID of the sequence to cancel
            
        Returns:
            True if cancellation initiated, False otherwise
        """
        try:
            if sequence_id not in self.sequence_states:
                self.logger.warning(f"Trying to cancel unknown sequence: {sequence_id}")
                return False
            
            state = self.sequence_states[sequence_id]
            old_state = state.state
            old_status = state.status
            
            # Update to cancelled state
            state.state = ExecutionState.CANCELLED
            state.status = ExecutionStatus.FAILED
            state.end_time = time.time()
            state.error_message = "Action sequence cancelled by user request"
            
            # Also cancel all actions in the sequence that are not already completed
            for action_id, action_state in self.action_states.items():
                if (action_state.sequence_id == sequence_id and 
                    action_state.state not in [ExecutionState.COMPLETED, ExecutionState.FAILED, ExecutionState.CANCELLED]):
                    await self.cancel_action_execution(action_id)
            
            # Notify callbacks
            await self._notify_state_change('sequence', state)
            
            self.logger.info(f"Cancelled sequence {sequence_id}, was {old_state} with status {old_status}")
            return True
            
        except Exception as e:
            self.logger.error(f"Error cancelling sequence {sequence_id}: {e}")
            return False
    
    @log_exception()
    async def pause_action_execution(self, action_id: str) -> bool:
        """
        Pause a specific action execution.
        
        Args:
            action_id: ID of the action to pause
            
        Returns:
            True if pause initiated, False otherwise
        """
        try:
            if action_id not in self.action_states:
                self.logger.warning(f"Trying to pause unknown action: {action_id}")
                return False
            
            state = self.action_states[action_id]
            
            # Only allow pausing if currently executing
            if state.state != ExecutionState.EXECUTING:
                self.logger.warning(f"Cannot pause action {action_id} currently in state {state.state}")
                return False
            
            old_state = state.state
            
            # Update to paused state
            state.state = ExecutionState.PAUSED
            state.last_updated = time.time()
            
            # Notify callbacks
            await self._notify_state_change('action', state)
            
            self.logger.info(f"Paused action {action_id}, was {old_state}")
            return True
            
        except Exception as e:
            self.logger.error(f"Error pausing action {action_id}: {e}")
            return False
    
    @log_exception()
    async def resume_action_execution(self, action_id: str) -> bool:
        """
        Resume a specific action execution.
        
        Args:
            action_id: ID of the action to resume
            
        Returns:
            True if resume initiated, False otherwise
        """
        try:
            if action_id not in self.action_states:
                self.logger.warning(f"Trying to resume unknown action: {action_id}")
                return False
            
            state = self.action_states[action_id]
            
            # Only allow resuming if currently paused
            if state.state != ExecutionState.PAUSED:
                self.logger.warning(f"Cannot resume action {action_id} currently in state {state.state}")
                return False
            
            old_state = state.state
            
            # Update to executing state
            state.state = ExecutionState.EXECUTING
            state.last_updated = time.time()
            
            # Notify callbacks
            await self._notify_state_change('action', state)
            
            self.logger.info(f"Resumed action {action_id}, was {old_state}")
            return True
            
        except Exception as e:
            self.logger.error(f"Error resuming action {action_id}: {e}")
            return False
    
    def add_state_change_callback(self, callback_type: str, callback_func: Callable):
        """
        Add a callback function to be notified when state changes occur.
        
        Args:
            callback_type: Either 'action' or 'sequence'
            callback_func: Async callback function to call on state change
        """
        if callback_type in self.state_change_callbacks:
            self.state_change_callbacks[callback_type].append(callback_func)
        else:
            self.logger.error(f"Invalid callback type: {callback_type}")
    
    async def _notify_state_change(self, state_type: str, state_object: Any):
        """
        Internal method to notify registered callbacks of state changes.
        
        Args:
            state_type: Either 'action' or 'sequence'
            state_object: The state object that changed
        """
        callbacks = self.state_change_callbacks.get(state_type, [])
        
        for callback in callbacks:
            try:
                # Call the callback (allow for both sync and async callbacks)
                if asyncio.iscoroutinefunction(callback):
                    await callback(state_type, state_object)
                else:
                    callback(state_type, state_object)
            except Exception as e:
                self.logger.error(f"Error calling state change callback: {e}")
    
    @log_exception()
    async def cleanup_expired_states(self):
        """
        Clean up expired execution states to prevent memory accumulation.
        """
        try:
            current_time = time.time()
            cleaned_count = 0
            
            # Clean up action states
            action_to_remove = []
            for action_id, state in self.action_states.items():
                retention_time = self.failed_state_retention_time if state.status == ExecutionStatus.FAILED else self.completed_state_retention_time
                elapsed_time = current_time - state.last_updated
                
                if elapsed_time > retention_time:
                    action_to_remove.append(action_id)
            
            for action_id in action_to_remove:
                del self.action_states[action_id]
                cleaned_count += 1
                self.logger.debug(f"Removed expired action state: {action_id}")
            
            # Clean up sequence states
            sequence_to_remove = []
            for sequence_id, state in self.sequence_states.items():
                retention_time = self.failed_state_retention_time if state.status == ExecutionStatus.FAILED else self.completed_state_retention_time
                elapsed_time = current_time - state.last_updated
                
                if elapsed_time > retention_time:
                    sequence_to_remove.append(sequence_id)
            
            for sequence_id in sequence_to_remove:
                del self.sequence_states[sequence_id]
                cleaned_count += 1
                self.logger.debug(f"Removed expired sequence state: {sequence_id}")
            
            if cleaned_count > 0:
                self.logger.info(f"Cleaned up {cleaned_count} expired execution states")
            
        except Exception as e:
            self.logger.error(f"Error cleaning up expired states: {e}")
    
    async def get_active_executions(self) -> Dict[str, Any]:
        """
        Get information about all currently active executions.
        
        Returns:
            Dictionary with active execution information
        """
        active_actions = {}
        active_sequences = {}
        
        current_time = time.time()
        
        # Collect active action executions
        for action_id, state in self.action_states.items():
            if state.state in [ExecutionState.EXECUTING, ExecutionState.VALIDATING, ExecutionState.APPROVED]:
                active_actions[action_id] = {
                    'status': state.status.name,
                    'state': state.state.name,
                    'progress': state.progress,
                    'current_step': state.current_step,
                    'elapsed_time': current_time - state.start_time,
                    'remaining_attempts': state.remaining_attempts
                }
        
        # Collect active sequence executions
        for sequence_id, state in self.sequence_states.items():
            if state.state in [ExecutionState.EXECUTING, ExecutionState.VALIDATING, ExecutionState.APPROVED]:
                active_sequences[sequence_id] = {
                    'status': state.status.name,
                    'state': state.state.name,
                    'progress': state.progress,
                    'current_action_index': state.current_action_index,
                    'total_actions': state.total_actions,
                    'completed_actions': state.completed_actions,
                    'failed_actions': state.failed_actions,
                    'elapsed_time': current_time - state.start_time
                }
        
        return {
            'active_actions': active_actions,
            'active_sequences': active_sequences,
            'summary': {
                'active_actions_count': len(active_actions),
                'active_sequences_count': len(active_sequences)
            }
        }
    
    async def get_execution_history(self, max_entries: int = 50) -> Dict[str, Any]:
        """
        Get historical information about recent executions.
        
        Args:
            max_entries: Maximum number of entries to return
            
        Returns:
            Dictionary with recent execution history
        """
        import heapq
        
        # Collect completed states
        completed_actions = []
        completed_sequences = []
        
        current_time = time.time()
        
        # Gather completed action states
        for action_id, state in self.action_states.items():
            if state.state in [ExecutionState.COMPLETED, ExecutionState.FAILED, ExecutionState.CANCELLED]:
                completed_actions.append({
                    'id': action_id,
                    'type': 'action',
                    'status': state.status.name,
                    'state': state.state.name,
                    'sequence_id': state.sequence_id,
                    'duration': (state.end_time or current_time) - state.start_time,
                    'error_message': state.error_message
                })
        
        # Gather completed sequence states
        for sequence_id, state in self.sequence_states.items():
            if state.state in [ExecutionState.COMPLETED, ExecutionState.FAILED, ExecutionState.CANCELLED]:
                completed_sequences.append({
                    'id': sequence_id,
                    'type': 'sequence',
                    'status': state.status.name,
                    'state': state.state.name,
                    'total_actions': state.total_actions,
                    'completed_actions': state.completed_actions,
                    'failed_actions': state.failed_actions,
                    'duration': (state.end_time or current_time) - state.start_time,
                    'error_message': state.error_message
                })
        
        # Sort by end time (most recent first)
        completed_actions.sort(key=lambda x: x['duration'], reverse=True)  # For simplicity, sort by duration
        completed_sequences.sort(key=lambda x: x['duration'], reverse=True)
        
        # Limit results
        completed_actions = completed_actions[:max_entries]
        completed_sequences = completed_sequences[:max_entries]
        
        return {
            'recent_actions': completed_actions,
            'recent_sequences': completed_sequences,
            'summary': {
                'recent_actions_count': len(completed_actions),
                'recent_sequences_count': len(completed_sequences)
            }
        }


# Global action execution state manager instance
_action_execution_state_manager = None


def get_action_execution_state_manager() -> ActionExecutionStateManager:
    """Get the global action execution state manager instance."""
    global _action_execution_state_manager
    if _action_execution_state_manager is None:
        _action_execution_state_manager = ActionExecutionStateManager()
    return _action_execution_state_manager


async def register_action_execution(action: ActionModel, sequence_id: str = None) -> str:
    """Convenience function to register an action for execution tracking."""
    manager = get_action_execution_state_manager()
    return await manager.register_action_execution(action, sequence_id)


async def register_sequence_execution(action_sequence: ActionSequenceModel) -> str:
    """Convenience function to register an action sequence for execution tracking."""
    manager = get_action_execution_state_manager()
    return await manager.register_sequence_execution(action_sequence)


async def update_action_state(
    action_id: str, 
    new_status: ExecutionStatus = None, 
    new_state: ExecutionState = None,
    progress: float = None,
    current_step: str = None,
    error_message: str = None,
    **kwargs
) -> bool:
    """Convenience function to update an action's execution state."""
    manager = get_action_execution_state_manager()
    return await manager.update_action_state(
        action_id, new_status, new_state, progress, current_step, error_message, **kwargs
    )


async def update_sequence_state(
    sequence_id: str, 
    new_status: ExecutionStatus = None, 
    new_state: ExecutionState = None,
    progress: float = None,
    current_action_index: int = None,
    completed_actions: int = None,
    failed_actions: int = None,
    active_action_id: str = None,
    error_message: str = None,
    **kwargs
) -> bool:
    """Convenience function to update a sequence's execution state."""
    manager = get_action_execution_state_manager()
    return await manager.update_sequence_state(
        sequence_id, new_status, new_state, progress, current_action_index,
        completed_actions, failed_actions, active_action_id, error_message, **kwargs
    )