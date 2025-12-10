"""
Robot state synchronization for the Vision-Language-Action (VLA) module.

This module ensures consistent synchronization of robot state across all VLA components,
including position, orientation, joint states, gripper state, and other relevant robot data.
"""

import asyncio
import logging
import time
from typing import Dict, Any, Optional, List, Callable
from dataclasses import dataclass
from enum import Enum
import threading

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import RobotState, RobotStateUpdate
from ..core.data_models import RobotStateModel, RobotStateUpdateModel
from ..voice_recognition.voice_command_node import get_voice_command_node
from ..llm_planning.cognitive_planner import get_cognitive_planner
from ..action_execution.robot_controller import get_robot_controller
from ..vision_perception.ros2_vision_node import get_vision_node


@dataclass
class SyncMetrics:
    """Metrics for state synchronization performance."""
    update_frequency: float  # Updates per second
    average_latency: float   # Average time from sensor to state update (seconds)
    sync_success_rate: float # Successful sync operations / total attempts
    pending_updates: int     # Number of pending state updates
    total_sync_operations: int


class SyncStatus(Enum):
    """Status of the synchronization process."""
    IDLE = "idle"
    SYNCING = "syncing"
    DELAYED = "delayed"  # When updates are coming in faster than they can be processed
    ERROR = "error"


class RobotStateSyncManager:
    """
    Manages synchronization of robot state across all VLA components.
    Provides consistent robot state information to voice recognition, cognitive planning,
    action execution, and vision perception components.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Robot state tracking
        self.current_robot_state: Optional[RobotStateModel] = None
        self.state_history: List[RobotStateModel] = []
        self.max_history_size = getattr(self.config, 'robot_state_max_history', 100)
        
        # Component references
        self.voice_command_node = None
        self.cognitive_planner = None
        self.robot_controller = None
        self.vision_node = None
        
        # State update management
        self.pending_updates: List[RobotStateUpdateModel] = []
        self.sync_callbacks: List[Callable[[RobotStateModel], None]] = []
        self.last_sync_time = 0.0
        self.sync_interval = getattr(self.config, 'robot_state_sync_interval', 0.1)  # 100ms default
        
        # Performance metrics
        self.sync_metrics = SyncMetrics(
            update_frequency=0.0,
            average_latency=0.0,
            sync_success_rate=0.0,
            pending_updates=0,
            total_sync_operations=0
        )
        
        # Synchronization status
        self.sync_status = SyncStatus.IDLE
        self.update_lock = threading.Lock()
        
        # Initialize component references if available
        self._initialize_component_references()
        
        self.logger.info("RobotStateSyncManager initialized")
    
    def _initialize_component_references(self):
        """Initialize references to other VLA components."""
        try:
            self.voice_command_node = get_voice_command_node()
            self.cognitive_planner = get_cognitive_planner()
            self.robot_controller = get_robot_controller()
            self.vision_node = get_vision_node()
            
            self.logger.info("Component references initialized")
        except Exception as e:
            self.logger.warning(f"Could not initialize all component references: {e}")
    
    @log_exception()
    async def update_robot_state(self, new_state: RobotStateModel) -> bool:
        """
        Update the robot state with new information and propagate to all components.
        
        Args:
            new_state: RobotStateModel with new state information
            
        Returns:
            True if update was successful, False otherwise
        """
        try:
            start_time = time.time()
            
            # Validate the new state
            if not self._validate_robot_state(new_state):
                self.logger.error("Invalid robot state received, rejecting update")
                return False
            
            with self.update_lock:
                # Update current state
                self.current_robot_state = new_state
                
                # Add to history
                self.state_history.append(new_state)
                if len(self.state_history) > self.max_history_size:
                    self.state_history.pop(0)
                
                # Add to pending updates
                self.pending_updates.append(RobotStateUpdateModel.create(
                    state_id=f"update_{int(time.time()*1000)}",
                    previous_state_id=self.current_robot_state.state_id if self.current_robot_state else "initial",
                    new_state_id=new_state.state_id,
                    timestamp=time.time(),
                    changes=self._calculate_state_changes(self.current_robot_state, new_state) if self.current_robot_state else {}
                ))
                
            # Propagate state to all components asynchronously
            await self._propagate_state_to_components(new_state)
            
            # Run callbacks
            self._run_sync_callbacks(new_state)
            
            # Update metrics
            sync_time = time.time() - start_time
            self._update_sync_metrics(sync_time, True)
            
            self.logger.debug(f"Robot state updated to ID: {new_state.state_id}, sync time: {sync_time:.3f}s")
            return True
            
        except Exception as e:
            self.logger.error(f"Error updating robot state: {e}")
            self._update_sync_metrics(0.0, False)
            raise VLAException(
                f"Robot state update error: {str(e)}", 
                VLAErrorType.STATE_SYNC_ERROR,
                e
            )
    
    def _validate_robot_state(self, state: RobotStateModel) -> bool:
        """
        Validate a robot state for correctness.
        
        Args:
            state: RobotStateModel to validate
            
        Returns:
            True if valid, False otherwise
        """
        if not state.state_id:
            self.logger.error("State validation failed: missing state_id")
            return False
        
        if not state.robot_id:
            self.logger.error("State validation failed: missing robot_id")
            return False
        
        # Validate position is reasonable (not extreme values)
        position = state.position
        if position:
            for coord in ['x', 'y', 'z']:
                if coord in position:
                    val = position[coord]
                    # Check for valid numeric values (not infinite or NaN)
                    if not isinstance(val, (int, float)) or val != val or abs(val) > 1e6:  # Check for inf/NaN and extremely large values
                        self.logger.error(f"State validation failed: invalid {coord} position value: {val}")
                        return False
        
        # Validate orientation is normalized quaternion
        orientation = state.orientation
        if orientation and 'x' in orientation and 'y' in orientation and 'z' in orientation and 'w' in orientation:
            length = (
                orientation['x']**2 + 
                orientation['y']**2 + 
                orientation['z']**2 + 
                orientation['w']**2
            )**0.5
            
            if abs(length - 1.0) > 0.1:  # Allow some tolerance for normalization errors
                self.logger.warning(f"Orientation quaternion not normalized (length: {length}), adjusting")
                # Optionally normalize here if needed
        
        return True
    
    def _calculate_state_changes(self, old_state: RobotStateModel, new_state: RobotStateModel) -> Dict[str, Any]:
        """
        Calculate changes between two robot states.
        
        Args:
            old_state: Previous RobotStateModel
            new_state: New RobotStateModel
            
        Returns:
            Dictionary with state changes
        """
        changes = {}
        
        # Check position changes
        if old_state.position != new_state.position:
            changes['position'] = {
                'from': old_state.position,
                'to': new_state.position
            }
        
        # Check orientation changes
        if old_state.orientation != new_state.orientation:
            changes['orientation'] = {
                'from': old_state.orientation,
                'to': new_state.orientation
            }
        
        # Check gripper changes
        if old_state.gripper_state != new_state.gripper_state:
            changes['gripper_state'] = {
                'from': old_state.gripper_state,
                'to': new_state.gripper_state
            }
        
        # Check battery changes
        if old_state.battery_level != new_state.battery_level:
            changes['battery_level'] = {
                'from': old_state.battery_level,
                'to': new_state.battery_level
            }
        
        # Check other state changes
        if old_state.mode != new_state.mode:
            changes['mode'] = {
                'from': old_state.mode,
                'to': new_state.mode
            }
        
        return changes
    
    async def _propagate_state_to_components(self, new_state: RobotStateModel):
        """
        Propagate the new robot state to all registered VLA components.
        
        Args:
            new_state: New RobotStateModel to propagate
        """
        propagation_tasks = []
        
        # Propagate to voice command node
        if self.voice_command_node:
            task = asyncio.create_task(
                self._propagate_to_voice_node(new_state)
            )
            propagation_tasks.append(task)
        
        # Propagate to cognitive planner
        if self.cognitive_planner:
            task = asyncio.create_task(
                self._propagate_to_cognitive_planner(new_state)
            )
            propagation_tasks.append(task)
        
        # Propagate to robot controller
        if self.robot_controller:
            task = asyncio.create_task(
                self._propagate_to_robot_controller(new_state)
            )
            propagation_tasks.append(task)
        
        # Propagate to vision node
        if self.vision_node:
            task = asyncio.create_task(
                self._propagate_to_vision_node(new_state)
            )
            propagation_tasks.append(task)
        
        # Wait for all propagation tasks to complete
        if propagation_tasks:
            results = await asyncio.gather(*propagation_tasks, return_exceptions=True)
            
            # Check for errors in propagation
            for i, result in enumerate(results):
                if isinstance(result, Exception):
                    self.logger.error(f"Error propagating state to component {i}: {result}")
    
    async def _propagate_to_voice_node(self, new_state: RobotStateModel):
        """Propagate state to voice command node."""
        try:
            # In a real implementation, this would call the voice node's state update method
            # For now, we'll just log the propagation
            self.voice_command_node.update_robot_state(new_state)
        except Exception as e:
            self.logger.error(f"Error propagating state to voice node: {e}")
    
    async def _propagate_to_cognitive_planner(self, new_state: RobotStateModel):
        """Propagate state to cognitive planner."""
        try:
            # Update the cognitive planner's context with new state
            if hasattr(self.cognitive_planner, 'update_robot_context'):
                await self.cognitive_planner.update_robot_context(new_state)
        except Exception as e:
            self.logger.error(f"Error propagating state to cognitive planner: {e}")
    
    async def _propagate_to_robot_controller(self, new_state: RobotStateModel):
        """Propagate state to robot controller."""
        try:
            # Update the robot controller's internal state
            self.robot_controller.update_robot_state(new_state)
        except Exception as e:
            self.logger.error(f"Error propagating state to robot controller: {e}")
    
    async def _propagate_to_vision_node(self, new_state: RobotStateModel):
        """Propagate state to vision node."""
        try:
            # Update the vision node with new robot state
            if hasattr(self.vision_node, 'update_robot_state'):
                self.vision_node.update_robot_state(new_state)
        except Exception as e:
            self.logger.error(f"Error propagating state to vision node: {e}")
    
    def _run_sync_callbacks(self, state: RobotStateModel):
        """
        Run all registered synchronization callbacks.
        
        Args:
            state: RobotStateModel to pass to callbacks
        """
        for callback in self.sync_callbacks:
            try:
                callback(state)
            except Exception as e:
                self.logger.error(f"Error in sync callback: {e}")
    
    def add_sync_callback(self, callback: Callable[[RobotStateModel], None]):
        """
        Add a callback to be called when robot state is synchronized.
        
        Args:
            callback: Function to call with RobotStateModel when sync occurs
        """
        self.sync_callbacks.append(callback)
        self.logger.debug(f"Added sync callback, now have {len(self.sync_callbacks)} callbacks")
    
    def remove_sync_callback(self, callback: Callable[[RobotStateModel], None]):
        """
        Remove a previously registered callback.
        
        Args:
            callback: Previously registered callback function
        """
        if callback in self.sync_callbacks:
            self.sync_callbacks.remove(callback)
            self.logger.debug(f"Removed sync callback, now have {len(self.sync_callbacks)} callbacks")
    
    @log_exception()
    async def get_current_robot_state(self) -> Optional[RobotStateModel]:
        """
        Get the current robot state.
        
        Returns:
            Current RobotStateModel or None if not available
        """
        # In a real implementation, this might query the robot directly
        # For now, return the cached state
        return self.current_robot_state
    
    @log_exception()
    async def synchronize_with_robot(self) -> bool:
        """
        Actively synchronize with the robot to get the most current state.
        
        Returns:
            True if successful sync, False otherwise
        """
        try:
            self.logger.info("Actively synchronizing with robot state")
            
            # In a real implementation, this would query the robot for its current state
            # For now, we'll just update the timestamp on the current state
            if self.current_robot_state:
                self.current_robot_state.last_updated = time.time()
                return True
            else:
                # If we don't have an initial state, we can't sync
                self.logger.warning("Cannot synchronize - no initial robot state available")
                return False
                
        except Exception as e:
            self.logger.error(f"Error in active robot synchronization: {e}")
            raise VLAException(
                f"Active synchronization error: {str(e)}", 
                VLAErrorType.STATE_SYNC_ERROR,
                e
            )
    
    def _update_sync_metrics(self, sync_time: float, success: bool):
        """
        Update synchronization performance metrics.
        
        Args:
            sync_time: Time taken for the sync operation
            success: Whether the sync was successful
        """
        with self.update_lock:
            # Update total operations count
            self.sync_metrics.total_sync_operations += 1
            
            # Update success rate
            successful_ops = sum(1 for _ in range(self.sync_metrics.total_sync_operations) if success or _ < self.sync_metrics.total_sync_operations - 1)
            self.sync_metrics.sync_success_rate = successful_ops / self.sync_metrics.total_sync_operations if self.sync_metrics.total_sync_operations > 0 else 0.0
            
            # Update average latency
            self.sync_metrics.average_latency = (
                (self.sync_metrics.average_latency * (self.sync_metrics.total_sync_operations - 1) + sync_time) /
                self.sync_metrics.total_sync_operations
            )
            
            # Update pending updates count
            self.sync_metrics.pending_updates = len(self.pending_updates)
    
    async def get_sync_metrics(self) -> SyncMetrics:
        """
        Get current synchronization metrics.
        
        Returns:
            SyncMetrics with current performance data
        """
        # Calculate current update frequency based on recent updates
        now = time.time()
        recent_updates = [update for update in self.pending_updates if now - update.timestamp < 1.0]  # Last 1 second
        self.sync_metrics.update_frequency = len(recent_updates)
        
        return self.sync_metrics
    
    def get_state_history(self, limit: int = 10) -> List[RobotStateModel]:
        """
        Get recent state history.
        
        Args:
            limit: Maximum number of states to return
            
        Returns:
            List of recent RobotStateModel objects
        """
        return self.state_history[-limit:] if len(self.state_history) > limit else self.state_history[:]
    
    async def start_continuous_sync(self):
        """
        Start continuous synchronization in the background.
        """
        self.logger.info("Starting continuous robot state synchronization")
        
        async def sync_loop():
            while True:
                try:
                    # Perform synchronization
                    await self.synchronize_with_robot()
                    
                    # Wait for the sync interval
                    await asyncio.sleep(self.sync_interval)
                    
                except asyncio.CancelledError:
                    self.logger.info("Continuous synchronization loop cancelled")
                    break
                except Exception as e:
                    self.logger.error(f"Error in continuous sync loop: {e}")
                    # Wait before retrying
                    await asyncio.sleep(self.sync_interval)
        
        # Run the sync loop in the background
        self.sync_task = asyncio.create_task(sync_loop())
    
    async def stop_continuous_sync(self):
        """
        Stop continuous synchronization.
        """
        if hasattr(self, 'sync_task'):
            self.sync_task.cancel()
            try:
                await self.sync_task
            except asyncio.CancelledError:
                pass
        self.logger.info("Continuous robot state synchronization stopped")


# Global robot state sync manager instance
_robot_state_sync_manager = None


def get_robot_state_sync_manager() -> RobotStateSyncManager:
    """Get the global robot state sync manager instance."""
    global _robot_state_sync_manager
    if _robot_state_sync_manager is None:
        _robot_state_sync_manager = RobotStateSyncManager()
    return _robot_state_sync_manager


async def update_robot_state(new_state: RobotStateModel) -> bool:
    """Convenience function to update robot state."""
    sync_manager = get_robot_state_sync_manager()
    return await sync_manager.update_robot_state(new_state)


async def get_current_robot_state() -> Optional[RobotStateModel]:
    """Convenience function to get current robot state."""
    sync_manager = get_robot_state_sync_manager()
    return await sync_manager.get_current_robot_state()


async def synchronize_with_robot() -> bool:
    """Convenience function to actively synchronize with robot."""
    sync_manager = get_robot_state_sync_manager()
    return await sync_manager.synchronize_with_robot()


async def get_state_sync_metrics() -> SyncMetrics:
    """Convenience function to get state synchronization metrics."""
    sync_manager = get_robot_state_sync_manager()
    return await sync_manager.get_sync_metrics()