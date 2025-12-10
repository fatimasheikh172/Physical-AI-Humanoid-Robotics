"""
Safety monitor for the Vision-Language-Action (VLA) module.

This module ensures all robotic actions are executed safely by validating 
actions against safety constraints, monitoring the environment, and 
implementing safety responses when needed.
"""

import asyncio
import logging
from typing import Dict, List, Any, Optional
from enum import Enum
import time
import math

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import Action, SafetyViolation, SafetyLevel
from ..core.data_models import ActionModel, SafetyViolationModel, RobotStateModel
from .robot_controller import get_robot_controller


class SafetyCheckType(Enum):
    """Types of safety checks."""
    COLLISION_DETECTION = "collision_detection"
    ENVIRONMENT_MONITORING = "environment_monitoring"
    FORCE_LIMITING = "force_limiting"
    BOUNDARY_CHECK = "boundary_check"
    MANIPULATION_SAFETY = "manipulation_safety"
    NAVIGATION_SAFETY = "navigation_safety"
    COMMUNICATION_INTEGRITY = "communication_integrity"


class SafetySystemState(Enum):
    """States of the safety system."""
    OPERATIONAL = "operational"
    WARNING = "warning"
    DEGRADED = "degraded"
    SAFETY_LOCKOUT = "safety_lockout"
    EMERGENCY_STOP = "emergency_stop"


class SafetyMonitor:
    """
    Monitors safety conditions and validates actions before execution.
    Implements safety checks and responses when violations are detected.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Initialize robot controller to get state information
        self.robot_controller = get_robot_controller()
        
        # Safety thresholds from configuration
        self.collision_threshold = getattr(self.config, 'safety_collision_threshold', 0.3)  # meters
        self.force_threshold = getattr(self.config, 'safety_force_threshold', 50.0)  # Newtons
        self.boundary_padding = getattr(self.config, 'safety_boundary_padding', 0.1)  # meters
        self.manipulation_height_min = getattr(self.config, 'safety_manipulation_min_height', 0.1)  # meters
        self.manipulation_height_max = getattr(self.config, 'safety_manipulation_max_height', 2.0)  # meters
        
        # Safety state tracking
        self.safety_system_state = SafetySystemState.OPERATIONAL
        self.active_violations = []
        self.safety_log = []
        
        # Environment monitoring
        self.environment_map = {}  # Updated from perception system
        self.human_tracking = {}   # Track human positions in environment
        
        self.logger.info("SafetyMonitor initialized")
    
    @log_exception()
    async def check_action_safety(self, action: ActionModel, robot_state: RobotStateModel) -> List[SafetyViolationModel]:
        """
        Check if an action is safe to execute based on current state and environment.
        
        Args:
            action: ActionModel to check
            robot_state: Current RobotStateModel
            
        Returns:
            List of SafetyViolationModel if unsafe, else empty list
        """
        try:
            self.logger.info(f"Checking safety for action: {action.action_id} ({action.action_type})")
            
            violations = []
            
            # Perform different safety checks based on action type
            if action.action_type in ['move_to', 'navigate', 'go_to']:
                violations.extend(await self._check_navigation_safety(action, robot_state))
            elif action.action_type in ['pick_up', 'place', 'grasp', 'release']:
                violations.extend(await self._check_manipulation_safety(action, robot_state))
            elif action.action_type in ['detect_object', 'perceive']:
                violations.extend(await self._check_perception_safety(action, robot_state))
            
            # Check general safety constraints regardless of action type
            violations.extend(self._check_general_safety_constraints(action, robot_state))
            
            # Log violations if found
            if violations:
                for violation in violations:
                    self.logger.warning(f"Safety violation detected: {violation.description}")
                    self.safety_log.append({
                        'timestamp': time.time(),
                        'violation': violation,
                        'action_id': action.action_id
                    })
            
            # Update safety system state based on violations
            if violations:
                self._update_safety_system_state(violations)
            else:
                self.safety_system_state = SafetySystemState.OPERATIONAL
            
            return violations
            
        except Exception as e:
            self.logger.error(f"Error in safety check: {e}")
            raise VLAException(
                f"Safety check error: {str(e)}", 
                VLAErrorType.SAFETY_ERROR,
                e
            )
    
    async def _check_navigation_safety(self, action: ActionModel, robot_state: RobotStateModel) -> List[SafetyViolationModel]:
        """
        Check safety for navigation actions.
        
        Args:
            action: Navigation ActionModel to check
            robot_state: Current RobotStateModel
            
        Returns:
            List of SafetyViolationModel if unsafe, else empty list
        """
        violations = []
        
        # Get target location from action parameters
        target_location = action.parameters.get('target_location')
        if not target_location:
            violations.append(
                SafetyViolationModel.create(
                    violation_id=f"nav_no_target_{int(time.time())}",
                    violation_type=SafetyCheckType.NAVIGATION_SAFETY,
                    severity=VLAErrorType.SAFETY_ERROR,
                    description="Navigation action has no target location specified",
                    details={
                        'action_id': action.action_id,
                        'action_type': action.action_type
                    }
                )
            )
            return violations
        
        # Check if target is within operational boundaries
        boundary_violations = self._check_boundary_violation(target_location)
        violations.extend(boundary_violations)
        
        # Check path for collisions
        if robot_state.position:
            path_violations = await self._check_path_safety(
                robot_state.position, 
                target_location
            )
            violations.extend(path_violations)
        
        # Check for human presence in path
        human_violations = self._check_human_safety_in_path(
            robot_state.position, 
            target_location
        )
        violations.extend(human_violations)
        
        return violations
    
    def _check_boundary_violation(self, location: Dict[str, float]) -> List[SafetyViolationModel]:
        """
        Check if a location is within operational boundaries.
        
        Args:
            location: Location dictionary with x, y, z coordinates
            
        Returns:
            List of boundary violation violations
        """
        violations = []
        
        # Get operational boundaries from config
        boundaries = getattr(self.config, 'operational_boundaries', {})
        
        if 'min_x' in boundaries and location['x'] < boundaries['min_x']:
            violations.append(
                SafetyViolationModel.create(
                    violation_id=f"boundary_violation_x_{int(time.time())}",
                    violation_type=SafetyCheckType.BOUNDARY_CHECK,
                    severity=VLAErrorType.SAFETY_ERROR,
                    description=f"Target x-coordinate {location['x']} is below minimum {boundaries['min_x']}",
                    details={
                        'coordinate': 'x',
                        'value': location['x'],
                        'minimum': boundaries['min_x'],
                        'maximum': boundaries.get('max_x', 'no limit')
                    }
                )
            )
        
        if 'max_x' in boundaries and location['x'] > boundaries['max_x']:
            violations.append(
                SafetyViolationModel.create(
                    violation_id=f"boundary_violation_x_{int(time.time())}",
                    violation_type=SafetyCheckType.BOUNDARY_CHECK,
                    severity=VLAErrorType.SAFETY_ERROR,
                    description=f"Target x-coordinate {location['x']} exceeds maximum {boundaries['max_x']}",
                    details={
                        'coordinate': 'x',
                        'value': location['x'],
                        'minimum': boundaries.get('min_x', 'no limit'),
                        'maximum': boundaries['max_x']
                    }
                )
            )
            
        if 'min_y' in boundaries and location['y'] < boundaries['min_y']:
            violations.append(
                SafetyViolationModel.create(
                    violation_id=f"boundary_violation_y_{int(time.time())}",
                    violation_type=SafetyCheckType.BOUNDARY_CHECK,
                    severity=VLAErrorType.SAFETY_ERROR,
                    description=f"Target y-coordinate {location['y']} is below minimum {boundaries['min_y']}",
                    details={
                        'coordinate': 'y',
                        'value': location['y'],
                        'minimum': boundaries['min_y'],
                        'maximum': boundaries.get('max_y', 'no limit')
                    }
                )
            )
        
        if 'max_y' in boundaries and location['y'] > boundaries['max_y']:
            violations.append(
                SafetyViolationModel.create(
                    violation_id=f"boundary_violation_y_{int(time.time())}",
                    violation_type=SafetyCheckType.BOUNDARY_CHECK,
                    severity=VLAErrorType.SAFETY_ERROR,
                    description=f"Target y-coordinate {location['y']} exceeds maximum {boundaries['max_y']}",
                    details={
                        'coordinate': 'y',
                        'value': location['y'],
                        'minimum': boundaries.get('min_y', 'no limit'),
                        'maximum': boundaries['max_y']
                    }
                )
            )
        
        return violations
    
    async def _check_path_safety(self, start_pos: Dict[str, float], end_pos: Dict[str, float]) -> List[SafetyViolationModel]:
        """
        Check if a path is clear of obstacles.
        
        Args:
            start_pos: Starting position
            end_pos: Ending position
            
        Returns:
            List of collision violations
        """
        violations = []
        
        # Calculate distance between start and end positions
        dx = end_pos['x'] - start_pos['x']
        dy = end_pos['y'] - start_pos['y']
        dz = end_pos['z'] - start_pos['z']
        total_distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # For now, we'll do a simplified check with a straight-line path
        # In a real implementation, this would check against the environment map
        # and potentially use path planning algorithms like A* or RRT
        
        # Check if path goes through known obstacles
        # This is a simplified implementation - in reality, this would be much more sophisticated
        path_resolution = 0.1  # meter resolution for path checking
        num_steps = max(1, int(total_distance / path_resolution))
        
        for step in range(num_steps + 1):
            fraction = step / num_steps if num_steps > 0 else 0
            current_pos = {
                'x': start_pos['x'] + dx * fraction,
                'y': start_pos['y'] + dy * fraction,
                'z': start_pos['z'] + dz * fraction
            }
            
            # Check if this position is near an obstacle
            obstacle_distance = self._get_closest_obstacle_distance(current_pos)
            if obstacle_distance < self.collision_threshold:
                violations.append(
                    SafetyViolationModel.create(
                        violation_id=f"path_collision_{int(time.time())}",
                        violation_type=SafetyCheckType.COLLISION_DETECTION,
                        severity=VLAErrorType.SAFETY_ERROR,
                        description=f"Path collision detected at position ({current_pos['x']:.2f}, {current_pos['y']:.2f}, {current_pos['z']:.2f}), distance to obstacle: {obstacle_distance:.2f}m",
                        details={
                            'position': current_pos,
                            'obstacle_distance': obstacle_distance,
                            'threshold': self.collision_threshold
                        }
                    )
                )
        
        return violations
    
    def _get_closest_obstacle_distance(self, position: Dict[str, float]) -> float:
        """
        Get the distance to the closest known obstacle from a position.
        
        Args:
            position: Position to check
            
        Returns:
            Distance to closest obstacle in meters
        """
        # In a real implementation, this would check against the environment map
        # For now, we'll return a large value to indicate no obstacles
        return 10.0  # Assume far from obstacles in simulation
    
    def _check_human_safety_in_path(self, start_pos: Dict[str, float], end_pos: Dict[str, float]) -> List[SafetyViolationModel]:
        """
        Check if the path goes near humans.
        
        Args:
            start_pos: Starting position
            end_pos: Ending position
            
        Returns:
            List of human safety violations
        """
        violations = []
        
        # Check against known human positions
        for human_id, human_pos in self.human_tracking.items():
            # Calculate distance from path to human
            human_distance = self._distance_to_line_segment(
                human_pos, start_pos, end_pos
            )
            
            if human_distance < self.config.get('safety_human_proximity_threshold', 1.0):
                violations.append(
                    SafetyViolationModel.create(
                        violation_id=f"human_proximity_{human_id}_{int(time.time())}",
                        violation_type=SafetyCheckType.ENVIRONMENT_MONITORING,
                        severity=VLAErrorType.SAFETY_ERROR,
                        description=f"Path dangerously close to human {human_id} at distance {human_distance:.2f}m",
                        details={
                            'human_id': human_id,
                            'human_position': human_pos,
                            'path_start': start_pos,
                            'path_end': end_pos,
                            'distance': human_distance,
                            'threshold': self.config.get('safety_human_proximity_threshold', 1.0)
                        }
                    )
                )
        
        return violations
    
    def _distance_to_line_segment(self, point: Dict[str, float], line_start: Dict[str, float], line_end: Dict[str, float]) -> float:
        """
        Calculate the distance from a point to a line segment.
        
        Args:
            point: The point to measure distance from
            line_start: Start of the line segment
            line_end: End of the line segment
            
        Returns:
            Distance to the line segment
        """
        # Vector from line_start to line_end
        line_vec = {
            'x': line_end['x'] - line_start['x'],
            'y': line_end['y'] - line_start['y'],
            'z': line_end['z'] - line_start['z']
        }
        
        # Vector from line_start to point
        point_vec = {
            'x': point['x'] - line_start['x'],
            'y': point['y'] - line_start['y'],
            'z': point['z'] - line_start['z']
        }
        
        # Length squared of line
        line_len_sq = line_vec['x']**2 + line_vec['y']**2 + line_vec['z']**2
        
        if line_len_sq == 0:
            # Line segment is actually a point
            return math.sqrt(
                (point['x'] - line_start['x'])**2 + 
                (point['y'] - line_start['y'])**2 + 
                (point['z'] - line_start['z'])**2
            )
        
        # Calculate projection of point_vec onto line_vec
        t = max(0, min(1, (
            point_vec['x']*line_vec['x'] + 
            point_vec['y']*line_vec['y'] + 
            point_vec['z']*line_vec['z']
        ) / line_len_sq))
        
        # Calculate closest point on line segment
        closest_point = {
            'x': line_start['x'] + t * line_vec['x'],
            'y': line_start['y'] + t * line_vec['y'],
            'z': line_start['z'] + t * line_vec['z']
        }
        
        # Calculate distance to closest point
        distance = math.sqrt(
            (point['x'] - closest_point['x'])**2 + 
            (point['y'] - closest_point['y'])**2 + 
            (point['z'] - closest_point['z'])**2
        )
        
        return distance
    
    async def _check_manipulation_safety(self, action: ActionModel, robot_state: RobotStateModel) -> List[SafetyViolationModel]:
        """
        Check safety for manipulation actions.
        
        Args:
            action: Manipulation ActionModel to check
            robot_state: Current RobotStateModel
            
        Returns:
            List of SafetyViolationModel if unsafe, else empty list
        """
        violations = []
        
        # Check if target position is within safe manipulation limits
        target_position = action.parameters.get('target_position') or \
                         action.parameters.get('target_location')
        
        if target_position:
            # Check height limits
            if target_position.get('z', 0) < self.manipulation_height_min:
                violations.append(
                    SafetyViolationModel.create(
                        violation_id=f"manipulation_low_{int(time.time())}",
                        violation_type=SafetyCheckType.MANIPULATION_SAFETY,
                        severity=VLAErrorType.SAFETY_ERROR,
                        description=f"Target position too low: {target_position['z']:.2f}m < min {self.manipulation_height_min:.2f}m",
                        details={
                            'target_position': target_position,
                            'minimum_height': self.manipulation_height_min,
                            'actual_height': target_position['z']
                        }
                    )
                )
            
            if target_position.get('z', 0) > self.manipulation_height_max:
                violations.append(
                    SafetyViolationModel.create(
                        violation_id=f"manipulation_high_{int(time.time())}",
                        violation_type=SafetyCheckType.MANIPULATION_SAFETY,
                        severity=VLAErrorType.SAFETY_ERROR,
                        description=f"Target position too high: {target_position['z']:.2f}m > max {self.manipulation_height_max:.2f}m",
                        details={
                            'target_position': target_position,
                            'maximum_height': self.manipulation_height_max,
                            'actual_height': target_position['z']
                        }
                    )
                )
        
        # Check for excessive force requests
        requested_force = action.parameters.get('force_limit')
        if requested_force and requested_force > self.force_threshold:
            violations.append(
                SafetyViolationModel.create(
                    violation_id=f"force_exceeded_{int(time.time())}",
                    violation_type=SafetyCheckType.FORCE_LIMITING,
                    severity=VLAErrorType.SAFETY_ERROR,
                    description=f"Requested force {requested_force}N exceeds safety limit {self.force_threshold}N",
                    details={
                        'requested_force': requested_force,
                        'threshold': self.force_threshold
                    }
                )
            )
        
        # Check if in a safe state for manipulation
        if robot_state.mode in ['navigation', 'moving']:
            violations.append(
                SafetyViolationModel.create(
                    violation_id=f"unsafe_manipulation_{int(time.time())}",
                    violation_type=SafetyCheckType.MANIPULATION_SAFETY,
                    severity=VLAErrorType.SAFETY_ERROR,
                    description="Cannot manipulate while robot is in motion",
                    details={
                        'robot_mode': robot_state.mode,
                        'action_type': action.action_type
                    }
                )
            )
        
        return violations
    
    def _check_general_safety_constraints(self, action: ActionModel, robot_state: RobotStateModel) -> List[SafetyViolationModel]:
        """
        Check general safety constraints applicable to all actions.
        
        Args:
            action: ActionModel to check
            robot_state: Current RobotStateModel
            
        Returns:
            List of SafetyViolationModel if unsafe, else empty list
        """
        violations = []
        
        # Check battery level
        if robot_state.battery_level < self.config.get('safety_min_battery_threshold', 0.15):
            violations.append(
                SafetyViolationModel.create(
                    violation_id=f"low_battery_{int(time.time())}",
                    violation_type=SafetyCheckType.ENVIRONMENT_MONITORING,
                    severity=VLAErrorType.SAFETY_WARNING,
                    description=f"Robot battery level too low: {robot_state.battery_level:.2%} < threshold {self.config.get('safety_min_battery_threshold', 0.15):.2%}",
                    details={
                        'battery_level': robot_state.battery_level,
                        'threshold': self.config.get('safety_min_battery_threshold', 0.15)
                    }
                )
            )
        
        # Check if robot is in error state
        if robot_state.mode in ['error', 'emergency_stop', 'safety_lockout']:
            violations.append(
                SafetyViolationModel.create(
                    violation_id=f"robot_mode_invalid_{int(time.time())}",
                    violation_type=SafetyCheckType.COMMUNICATION_INTEGRITY,
                    severity=VLAErrorType.SAFETY_ERROR,
                    description=f"Cannot execute action while robot is in {robot_state.mode} mode",
                    details={
                        'robot_mode': robot_state.mode,
                        'action_type': action.action_type
                    }
                )
            )
        
        # Check for dangerous action types
        if action.action_type.lower() in ['destroy', 'damage', 'harm', 'injure']:
            violations.append(
                SafetyViolationModel.create(
                    violation_id=f"dangerous_action_{int(time.time())}",
                    violation_type=SafetyCheckType.ENVIRONMENT_MONITORING,
                    severity=VLAErrorType.SAFETY_CRITICAL,
                    description=f"Dangerous action type detected: {action.action_type}",
                    details={
                        'action_type': action.action_type,
                        'action_parameters': action.parameters
                    }
                )
            )
        
        return violations
    
    @log_exception()
    async def handle_safety_violation(self, violation: SafetyViolationModel):
        """
        Handle a detected safety violation according to its severity.
        
        Args:
            violation: SafetyViolationModel to handle
        """
        try:
            self.logger.warning(f"Handling safety violation: {violation.description}")
            
            # Add to active violations if not already there
            if violation.violation_id not in [v.violation_id for v in self.active_violations]:
                self.active_violations.append(violation)
            
            # Handle based on severity
            if violation.severity == VLAErrorType.SAFETY_CRITICAL:
                # Critical violation - need immediate action
                self._handle_critical_violation(violation)
            elif violation.severity == VLAErrorType.SAFETY_ERROR:
                # Error violation - may need to stop operation
                self._handle_error_violation(violation)
            elif violation.severity == VLAErrorType.SAFETY_WARNING:
                # Warning - continue but log
                self._handle_warning_violation(violation)
            
            # Update safety system state
            self._update_safety_system_state([violation])
            
        except Exception as e:
            self.logger.error(f"Error handling safety violation: {e}")
    
    def _handle_critical_violation(self, violation: SafetyViolationModel):
        """Handle critical safety violations requiring immediate response."""
        self.logger.critical(f"CRITICAL SAFETY VIOLATION: {violation.description}")
        
        # Enter emergency stop state
        self.safety_system_state = SafetySystemState.EMERGENCY_STOP
        
        # Attempt emergency stop of robot
        asyncio.create_task(self._issue_robot_emergency_stop())
    
    def _handle_error_violation(self, violation: SafetyViolationModel):
        """Handle error-level safety violations."""
        self.logger.error(f"SAFETY ERROR: {violation.description}")
        
        # May need to pause operations based on the violation
        if self.safety_system_state == SafetySystemState.OPERATIONAL:
            self.safety_system_state = SafetySystemState.SAFETY_LOCKOUT
    
    def _handle_warning_violation(self, violation: SafetyViolationModel):
        """Handle warning-level safety violations."""
        self.logger.warning(f"SAFETY WARNING: {violation.description}")
        
        # Log but continue operations
        if self.safety_system_state == SafetySystemState.OPERATIONAL:
            self.safety_system_state = SafetySystemState.WARNING
    
    async def _issue_robot_emergency_stop(self):
        """Issue emergency stop command to the robot."""
        try:
            # In a real implementation, this would send an emergency stop command to the robot
            # For now, we'll simulate this by calling the robot controller's emergency stop
            await self.robot_controller.emergency_stop()
            self.logger.info("Emergency stop command issued to robot")
        except Exception as e:
            self.logger.error(f"Failed to issue emergency stop: {e}")
    
    def _update_safety_system_state(self, violations: List[SafetyViolationModel]):
        """
        Update the safety system state based on violations.
        
        Args:
            violations: List of current violations
        """
        if not violations:
            self.safety_system_state = SafetySystemState.OPERATIONAL
            return
        
        # Determine the most severe violation
        severity_order = [
            VLAErrorType.SAFETY_CRITICAL,
            VLAErrorType.SAFETY_ERROR,
            VLAErrorType.SAFETY_WARNING
        ]
        
        most_severe = min(violations, key=lambda v: severity_order.index(v.severity)).severity
        
        if most_severe == VLAErrorType.SAFETY_CRITICAL:
            self.safety_system_state = SafetySystemState.EMERGENCY_STOP
        elif most_severe == VLAErrorType.SAFETY_ERROR:
            self.safety_system_state = SafetySystemState.SAFETY_LOCKOUT
        elif most_severe == VLAErrorType.SAFETY_WARNING:
            if self.safety_system_state != SafetySystemState.EMERGENCY_STOP:
                self.safety_system_state = SafetySystemState.WARNING
    
    @log_exception()
    async def update_environment_map(self, new_map_data: Dict[str, Any]):
        """
        Update the environment map with new perception data.
        
        Args:
            new_map_data: New environment map data
        """
        try:
            self.logger.info(f"Updating environment map with {len(new_map_data.get('obstacles', []))} obstacles")
            self.environment_map.update(new_map_data)
        except Exception as e:
            self.logger.error(f"Error updating environment map: {e}")
    
    @log_exception()
    async def update_human_tracking(self, human_positions: Dict[str, Dict[str, float]]):
        """
        Update human tracking information.
        
        Args:
            human_positions: Dictionary mapping human IDs to positions
        """
        try:
            self.logger.info(f"Updating human tracking with {len(human_positions)} humans")
            self.human_tracking.update(human_positions)
        except Exception as e:
            self.logger.error(f"Error updating human tracking: {e}")
    
    def get_safety_status(self) -> Dict[str, Any]:
        """
        Get the current safety status.
        
        Returns:
            Dictionary with safety status information
        """
        return {
            'system_state': self.safety_system_state.value,
            'active_violations_count': len(self.active_violations),
            'active_violations': [
                {
                    'violation_id': v.violation_id,
                    'type': v.violation_type.value,
                    'severity': v.severity.value,
                    'description': v.description
                } for v in self.active_violations
            ],
            'last_update': time.time()
        }
    
    def get_safety_log(self, limit: int = 100) -> List[Dict[str, Any]]:
        """
        Get the recent safety log entries.
        
        Args:
            limit: Maximum number of entries to return
            
        Returns:
            List of safety log entries
        """
        # Return the last 'limit' entries
        log_subset = self.safety_log[-limit:] if len(self.safety_log) > limit else self.safety_log
        
        return [
            {
                'timestamp': entry['timestamp'],
                'violation_id': entry['violation'].violation_id,
                'violation_type': entry['violation'].violation_type.value,
                'severity': entry['violation'].severity.value,
                'description': entry['violation'].description,
                'action_id': entry.get('action_id', 'unknown')
            } for entry in log_subset
        ]
    
    async def reset_safety_system(self) -> bool:
        """
        Reset the safety system after a safety event.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            self.logger.info("Resetting safety system")
            
            # Clear active violations
            self.active_violations.clear()
            
            # Reset safety state to operational
            self.safety_system_state = SafetySystemState.OPERATIONAL
            
            self.logger.info("Safety system reset completed")
            return True
            
        except Exception as e:
            self.logger.error(f"Error resetting safety system: {e}")
            return False


# Global safety monitor instance
_safety_monitor = None


def get_safety_monitor() -> SafetyMonitor:
    """Get the global safety monitor instance."""
    global _safety_monitor
    if _safety_monitor is None:
        _safety_monitor = SafetyMonitor()
    return _safety_monitor


async def check_action_safety(action: ActionModel, robot_state: RobotStateModel) -> List[SafetyViolationModel]:
    """Convenience function to check action safety."""
    safety_monitor = get_safety_monitor()
    return await safety_monitor.check_action_safety(action, robot_state)


async def handle_safety_violation(violation: SafetyViolationModel):
    """Convenience function to handle a safety violation."""
    safety_monitor = get_safety_monitor()
    await safety_monitor.handle_safety_violation(violation)


def get_safety_status() -> Dict[str, Any]:
    """Convenience function to get safety status."""
    safety_monitor = get_safety_monitor()
    return safety_monitor.get_safety_status()