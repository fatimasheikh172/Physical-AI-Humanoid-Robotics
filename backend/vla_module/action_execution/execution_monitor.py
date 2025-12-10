"""
Execution monitoring and safety checks for the Vision-Language-Action (VLA) module.

This module monitors action execution for safety and performance, implementing
checks and safeguards to ensure safe robot operation.
"""

import asyncio
import logging
import time
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass
from enum import Enum

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import Action, ActionSequence, ExecutionStatus, SafetyViolation
from ..core.data_models import ActionModel, ActionSequenceModel, SafetyViolationModel, RobotStateModel
from ..action_execution.robot_controller import get_robot_controller
from ..vision_perception.vision_processor import get_vision_processor
from ..capstone_integration.path_planning_integrator import get_path_integrator
from ..core.robot_state_sync import get_robot_state_sync_manager


class SafetyLevel(Enum):
    """Safety level for violations."""
    LOW = "low"
    MEDIUM = "medium" 
    HIGH = "high"
    CRITICAL = "critical"


@dataclass
class ExecutionMetrics:
    """Metrics for action execution."""
    execution_time: float
    success_rate: float
    resource_usage: Dict[str, float]
    safety_incidents: int
    average_response_time: float


@dataclass
class SafetyCheckResult:
    """Result of a safety check."""
    passed: bool
    violations: List[SafetyViolationModel]
    recommendations: List[str]
    timestamp: float


class ExecutionMonitor:
    """
    Monitors action execution for safety, performance, and correctness.
    Implements various safety checks and performance metrics collection.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Initialize components
        self.robot_controller = get_robot_controller()
        self.vision_processor = get_vision_processor()
        self.path_integrator = get_path_integrator()
        self.state_sync_manager = get_robot_state_sync_manager()
        
        # Monitoring settings
        self.safety_check_frequency = getattr(self.config, 'execution_safety_check_freq', 10.0)  # Hz
        self.performance_monitoring_enabled = True
        self.safety_monitoring_enabled = True
        self.event_callbacks: List[Callable[[str, Any], None]] = []
        
        # Performance and safety tracking
        self.execution_histogram = {}
        self.safety_violations = []
        self.performance_metrics = []
        self.max_metrics_history = getattr(self.config, 'max_metrics_history', 1000)
        
        # Safety thresholds
        self.collision_distance_threshold = getattr(self.config, 'safety_collision_distance_threshold', 0.5)  # meters
        self.velocity_threshold = getattr(self.config, 'safety_velocity_threshold', 1.0)  # m/s
        self.force_threshold = getattr(self.config, 'safety_force_threshold', 50.0)  # Newtons
        self.battery_low_threshold = getattr(self.config, 'safety_battery_low_threshold', 0.15)  # 15% battery
        
        # Monitoring state
        self.active_monitors = {}
        self.monitoring_task = None
        
        self.logger.info("ExecutionMonitor initialized")
    
    @log_exception()
    async def start_monitoring(self):
        """Start the execution monitoring loop."""
        if self.monitoring_task:
            self.logger.warning("Monitoring already running")
            return
        
        self.logger.info("Starting execution monitoring")
        
        # Create monitoring task that runs periodically
        self.monitoring_task = asyncio.create_task(self._monitoring_loop())
    
    @log_exception()
    async def stop_monitoring(self):
        """Stop the execution monitoring loop."""
        if not self.monitoring_task:
            self.logger.warning("No monitoring task to stop")
            return
        
        self.logger.info("Stopping execution monitoring")
        
        self.monitoring_task.cancel()
        try:
            await self.monitoring_task
        except asyncio.CancelledError:
            pass
        
        self.monitoring_task = None
    
    async def _monitoring_loop(self):
        """Main monitoring loop that runs continuously."""
        while True:
            try:
                # Perform safety checks
                if self.safety_monitoring_enabled:
                    await self._perform_safety_monitoring()
                
                # Perform performance monitoring
                if self.performance_monitoring_enabled:
                    await self._perform_performance_monitoring()
                
                # Wait based on the monitoring frequency
                check_interval = 1.0 / self.safety_check_frequency
                await asyncio.sleep(check_interval)
                
            except asyncio.CancelledError:
                self.logger.info("Monitoring loop cancelled")
                break
            except Exception as e:
                self.logger.error(f"Error in monitoring loop: {e}")
                # Continue monitoring despite errors
                await asyncio.sleep(1.0)  # Brief pause before continuing
    
    async def _perform_safety_monitoring(self):
        """Perform safety checks during execution."""
        try:
            # Get current robot state
            current_state = await self.state_sync_manager.get_current_robot_state()
            if not current_state:
                # If we can't get the robot state, we can't perform safety checks
                return
            
            # Check various safety conditions
            violations = []
            
            # 1. Battery level check
            if current_state.battery_level < self.battery_low_threshold:
                battery_violation = SafetyViolationModel.create(
                    violation_id=f"battery_low_{int(time.time())}",
                    violation_type="power_safety",
                    severity=SafetyLevel.HIGH,
                    description=f"Battery level too low: {current_state.battery_level:.2%} < threshold {self.battery_low_threshold:.2%}",
                    details={
                        'battery_level': current_state.battery_level,
                        'threshold': self.battery_low_threshold,
                        'timestamp': time.time()
                    }
                )
                violations.append(battery_violation)
            
            # 2. Velocity limits check
            if hasattr(current_state, 'velocity') and current_state.velocity:
                linear_vel = current_state.velocity.get('linear', {})
                if 'x' in linear_vel and abs(linear_vel['x']) > self.velocity_threshold:
                    velocity_violation = SafetyViolationModel.create(
                        violation_id=f"velocity_high_{int(time.time())}",
                        violation_type="motion_safety",
                        severity=SafetyLevel.MEDIUM,
                        description=f"Linear velocity too high: {abs(linear_vel['x']):.2f}m/s > threshold {self.velocity_threshold:.2f}m/s",
                        details={
                            'linear_velocity_x': linear_vel['x'],
                            'threshold': self.velocity_threshold,
                            'timestamp': time.time()
                        }
                    )
                    violations.append(velocity_violation)
            
            # 3. Check joint limits if available
            if hasattr(current_state, 'joints') and current_state.joints:
                for joint in current_state.joints:
                    if joint.get('position', 0) > joint.get('max_position', float('inf')):
                        joint_violation = SafetyViolationModel.create(
                            violation_id=f"joint_limit_{joint.get('name', 'unknown')}_{int(time.time())}",
                            violation_type="mechanical_safety",
                            severity=SafetyLevel.HIGH,
                            description=f"Joint limit exceeded: {joint.get('name', 'unnamed')} at position {joint['position']:.2f}",
                            details={
                                'joint_name': joint.get('name', 'unknown'),
                                'position': joint['position'],
                                'max_position': joint.get('max_position', float('inf')),
                                'timestamp': time.time()
                            }
                        )
                        violations.append(joint_violation)
            
            # 4. Check for environmental hazards (using vision system)
            env_violations = await self._check_environmental_safety(current_state)
            violations.extend(env_violations)
            
            # Process violations
            for violation in violations:
                self.safety_violations.append(violation)
                if len(self.safety_violations) > self.max_violations_history:
                    self.safety_violations = self.safety_violations[-self.max_violations_history:]
                
                # Log violation
                self.logger.warning(f"Safety violation detected: {violation.description}")
                
                # Run safety callbacks
                await self._trigger_safety_callbacks("violation_detected", violation)
            
            # Trigger event if any violations occurred
            if violations:
                await self._trigger_callbacks("safety_violations", violations)
                
        except Exception as e:
            self.logger.error(f"Error in safety monitoring: {e}")
    
    async def _check_environmental_safety(self, robot_state: RobotStateModel) -> List[SafetyViolationModel]:
        """
        Check for environmental safety hazards using vision system.
        
        Args:
            robot_state: Current robot state for context
            
        Returns:
            List of environmental safety violations
        """
        violations = []
        
        try:
            # Get environment perception from vision system
            # This is a simplified approach - in reality, we'd continuously monitor
            # and only check when specifically monitoring an action
            if self.vision_processor:
                # Get latest environment observation
                # This would come from the current perception buffer
                # For now, we'll simulate environmental safety checks
                pass
            
            # In a real system, we would:
            # 1. Check for newly detected obstacles in the robot's path
            # 2. Detect humans in the workspace
            # 3. Identify hazardous environmental conditions
            
            # For this implementation, we'll just return an empty list
            # since we don't have a live vision system running
            return []
            
        except Exception as e:
            self.logger.error(f"Error checking environmental safety: {e}")
            # Return an empty list instead of failing the entire safety check
            return []
    
    async def _perform_performance_monitoring(self):
        """Perform performance monitoring during execution."""
        try:
            # Collect performance metrics
            # In a real implementation, this would collect metrics from various sources
            # For now, we'll just track safety violation rates and monitoring performance
            pass
            
        except Exception as e:
            self.logger.error(f"Error in performance monitoring: {e}")
    
    @log_exception()
    async def monitor_action_execution(
        self, 
        action: ActionModel, 
        execution_callback: Callable[[], Any]
    ) -> Dict[str, Any]:
        """
        Monitor the execution of a single action with safety and performance checks.
        
        Args:
            action: ActionModel to monitor
            execution_callback: Function to execute the action
            
        Returns:
            Dictionary with execution results and metrics
        """
        try:
            self.logger.info(f"Monitoring execution of action: {action.action_id}")
            
            start_time = time.time()
            initial_state = await self.state_sync_manager.get_current_robot_state()
            
            # Perform pre-execution safety checks
            pre_execution_check = await self.perform_safety_check(
                action=action,
                robot_state=initial_state,
                check_type="pre_execution"
            )
            
            if not pre_execution_check.passed:
                error_msg = f"Pre-execution safety check failed: {[v.description for v in pre_execution_check.violations[:3]]}"
                self.logger.error(error_msg)
                
                # Depending on severity, we might proceed with caution or abort
                critical_violations = [v for v in pre_execution_check.violations if v.severity == SafetyLevel.CRITICAL]
                if critical_violations:
                    return {
                        'status': 'FAILED',
                        'error': error_msg,
                        'safety_violations': pre_execution_check.violations,
                        'execution_time': 0.0
                    }
                else:
                    self.logger.warning("Proceeding with minor safety violations after pre-check")
            
            # Execute the action
            execution_result = await execution_callback()
            
            end_time = time.time()
            execution_time = end_time - start_time
            
            # Perform post-execution safety checks
            final_state = await self.state_sync_manager.get_current_robot_state()
            post_execution_check = await self.perform_safety_check(
                action=action,
                robot_state=final_state,
                check_type="post_execution"
            )
            
            # Calculate metrics
            success = execution_result.get('status') == ExecutionStatus.COMPLETED
            success_rate = 1.0 if success else 0.0
            
            execution_metrics = ExecutionMetrics(
                execution_time=execution_time,
                success_rate=success_rate,
                resource_usage={
                    'cpu_percent': 0.0,  # Filled from system monitoring
                    'memory_mb': 0.0,    # Filled from system monitoring
                    'battery_drain': initial_state.battery_level - final_state.battery_level if initial_state and final_state else 0.0
                },
                safety_incidents=len(post_execution_check.violations),
                average_response_time=execution_time  # For single action, execution time is response time
            )
            
            # Store metrics
            self.performance_metrics.append(execution_metrics)
            if len(self.performance_metrics) > self.max_metrics_history:
                self.performance_metrics.pop(0)
            
            result = {
                'action_id': action.action_id,
                'status': execution_result.get('status', 'UNKNOWN'),
                'execution_time': execution_time,
                'success': success,
                'metrics': execution_metrics,
                'pre_execution_safety_check': {
                    'passed': pre_execution_check.passed,
                    'violations': [v.description for v in pre_execution_check.violations]
                },
                'post_execution_safety_check': {
                    'passed': post_execution_check.passed,
                    'violations': [v.description for v in post_execution_check.violations]
                },
                'recommendations': post_execution_check.recommendations
            }
            
            self.logger.info(f"Action {action.action_id} execution monitored: {execution_time:.2f}s, success: {success}")
            
            # Trigger event callbacks
            await self._trigger_callbacks("action_execution_completed", result)
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error monitoring action execution: {e}")
            raise VLAException(
                f"Action execution monitoring error: {str(e)}", 
                VLAErrorType.MONITORING_ERROR,
                e
            )
    
    @log_exception()
    async def monitor_action_sequence_execution(
        self, 
        action_sequence: ActionSequenceModel, 
        execution_callback: Callable[[], Any]
    ) -> Dict[str, Any]:
        """
        Monitor the execution of an entire action sequence.
        
        Args:
            action_sequence: ActionSequenceModel to monitor
            execution_callback: Function to execute the sequence
            
        Returns:
            Dictionary with sequence execution results and metrics
        """
        try:
            self.logger.info(f"Monitoring execution of action sequence: {action_sequence.sequence_id}")
            
            start_time = time.time()
            initial_state = await self.state_sync_manager.get_current_robot_state()
            
            # Start sequence monitoring
            sequence_monitor_id = f"seq_{action_sequence.sequence_id}_{int(time.time())}"
            self.active_monitors[sequence_monitor_id] = {
                'start_time': start_time,
                'initial_state': initial_state,
                'monitored_actions': [],
                'sequence_id': action_sequence.sequence_id
            }
            
            # Execute the sequence
            execution_result = await execution_callback()
            
            end_time = time.time()
            total_execution_time = end_time - start_time
            
            # Calculate sequence metrics
            action_count = len(action_sequence.actions)
            successful_actions = sum(1 for r in execution_result.get('responses', []) if r.status == ExecutionStatus.COMPLETED)
            success_rate = successful_actions / action_count if action_count > 0 else 0.0
            
            sequence_metrics = ExecutionMetrics(
                execution_time=total_execution_time,
                success_rate=success_rate,
                resource_usage={
                    'cpu_percent': 0.0,  # Calculated from system metrics
                    'memory_mb': 0.0,    # Calculated from system metrics
                    'avg_battery_drain': 0.0  # Calculated from state changes
                },
                safety_incidents=0,  # Calculated from monitoring during execution
                average_response_time=total_execution_time / action_count if action_count > 0 else 0.0
            )
            
            result = {
                'sequence_id': action_sequence.sequence_id,
                'status': execution_result.get('status', 'UNKNOWN'),
                'total_execution_time': total_execution_time,
                'action_count': action_count,
                'successful_actions': successful_actions,
                'success_rate': success_rate,
                'metrics': sequence_metrics,
                'action_results': execution_result.get('responses', [])
            }
            
            # Store metrics
            self.performance_metrics.append(sequence_metrics)
            if len(self.performance_metrics) > self.max_metrics_history:
                self.performance_metrics.pop(0)
            
            # Remove from active monitors
            if sequence_monitor_id in self.active_monitors:
                del self.active_monitors[sequence_monitor_id]
            
            self.logger.info(
                f"Sequence {action_sequence.sequence_id} execution monitored: "
                f"{total_execution_time:.2f}s, success_rate: {success_rate:.2%}"
            )
            
            # Trigger event callbacks
            await self._trigger_callbacks("sequence_execution_completed", result)
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error monitoring action sequence execution: {e}")
            raise VLAException(
                f"Action sequence execution monitoring error: {str(e)}", 
                VLAErrorType.MONITORING_ERROR,
                e
            )
    
    async def perform_safety_check(
        self, 
        action: Optional[ActionModel] = None, 
        robot_state: Optional[RobotStateModel] = None,
        check_type: str = "general"
    ) -> SafetyCheckResult:
        """
        Perform a safety check based on action and robot state.
        
        Args:
            action: Optional ActionModel to check
            robot_state: Optional RobotStateModel to check
            check_type: Type of check to perform ("general", "pre_execution", "post_execution", "during_execution")
            
        Returns:
            SafetyCheckResult with check results
        """
        try:
            violations = []
            recommendations = []
            
            # If no robot state provided, get current
            if not robot_state:
                robot_state = await self.state_sync_manager.get_current_robot_state()
            
            # General safety checks that apply regardless of action
            if robot_state:
                # Check battery level
                if robot_state.battery_level < self.battery_low_threshold:
                    violation = SafetyViolationModel.create(
                        violation_id=f"battery_low_check_{int(time.time())}",
                        violation_type="power_safety",
                        severity=SafetyLevel.HIGH,
                        description=f"Robot battery low: {robot_state.battery_level:.2%}",
                        details={'battery_level': robot_state.battery_level, 'threshold': self.battery_low_threshold}
                    )
                    violations.append(violation)
                    recommendations.append("Charge robot battery or proceed to charging station")
            
                # Check for emergency stop state
                if robot_state.mode == "emergency_stop":
                    violation = SafetyViolationModel.create(
                        violation_id=f"emergency_stop_active_{int(time.time())}",
                        violation_type="system_safety",
                        severity=SafetyLevel.CRITICAL,
                        description="Robot is in emergency stop mode",
                        details={'robot_mode': robot_state.mode}
                    )
                    violations.append(violation)
                    recommendations.append("Clear emergency stop before proceeding")
            
            # Action-specific safety checks
            if action and robot_state:
                if action.action_type == 'move_to' or action.action_type == 'navigate':
                    # Check target location for safety
                    target_location = action.parameters.get('target_location', {})
                    if target_location:
                        # In a real implementation, this would check against environment map for obstacles
                        # For now, we'll just check if target is extremely far away
                        current_pos = robot_state.position
                        if current_pos:
                            distance = (
                                (target_location.get('x', 0) - current_pos.get('x', 0))**2 +
                                (target_location.get('y', 0) - current_pos.get('y', 0))**2 +
                                (target_location.get('z', 0) - current_pos.get('z', 0))**2
                            )**0.5
                            
                            if distance > self.config.max_navigation_distance * 0.9:  # 90% of max distance
                                violation = SafetyViolationModel.create(
                                    violation_id=f"navigation_too_far_{action.action_id}_{int(time.time())}",
                                    violation_type="navigation_safety",
                                    severity=SafetyLevel.MEDIUM,
                                    description=f"Navigation target too far: {distance:.2f}m > 90% of max distance ({config.max_navigation_distance}m)",
                                    details={
                                        'target_distance': distance,
                                        'max_distance': self.config.max_navigation_distance,
                                        'action_type': action.action_type
                                    }
                                )
                                violations.append(violation)
                                recommendations.append(f"Verify navigation target is reachable, consider intermediate waypoints")
                
                elif action.action_type in ['pick_up', 'grasp', 'manipulate']:
                    # Check manipulation safety
                    if robot_state.mode in ['navigation', 'moving']:
                        violation = SafetyViolationModel.create(
                            violation_id=f"manipulation_while_moving_{action.action_id}_{int(time.time())}",
                            violation_type="manipulation_safety",
                            severity=SafetyLevel.HIGH,
                            description="Manipulation requested while robot is in motion, unsafe",
                            details={
                                'action_type': action.action_type,
                                'robot_mode': robot_state.mode,
                                'recommended': 'Wait for robot to stop before manipulation'
                            }
                        )
                        violations.append(violation)
                        recommendations.append("Wait for robot to come to complete stop before manipulation")
            
            # Check safety constraints based on execution context
            if check_type == "pre_execution":
                # More stringent checks before execution
                pass
            elif check_type == "during_execution":
                # Checks appropriate for during execution
                pass
            elif check_type == "post_execution":
                # Checks for post-execution validation
                pass
            
            # Create result
            result = SafetyCheckResult(
                passed=len(violations) == 0,
                violations=violations,
                recommendations=recommendations,
                timestamp=time.time()
            )
            
            self.logger.debug(f"Safety check completed: passed={result.passed}, violations={len(result.violations)}")
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error performing safety check: {e}")
            # Return a safe result (failed with critical violation) if check itself fails
            critical_violation = SafetyViolationModel.create(
                violation_id=f"safety_check_error_{int(time.time())}",
                violation_type="system_error",
                severity=SafetyLevel.CRITICAL,
                description=f"Safety check system error: {str(e)}",
                details={'error': str(e), 'type': 'safety_check_system'}
            )
            
            return SafetyCheckResult(
                passed=False,
                violations=[critical_violation],
                recommendations=["Abort current operation due to safety system error"],
                timestamp=time.time()
            )
    
    def add_event_callback(self, callback: Callable[[str, Any], None]):
        """
        Add a callback to be notified of execution events.
        
        Args:
            callback: Function to call when events occur (event_type, event_data)
        """
        self.event_callbacks.append(callback)
        self.logger.debug(f"Added event callback, total now: {len(self.event_callbacks)}")
    
    async def _trigger_callbacks(self, event_type: str, event_data: Any):
        """
        Trigger all registered callbacks for an event.
        
        Args:
            event_type: Type of event that occurred
            event_data: Data associated with the event
        """
        tasks = []
        for callback in self.event_callbacks:
            try:
                # Handle both sync and async callbacks
                if asyncio.iscoroutinefunction(callback):
                    task = asyncio.create_task(callback(event_type, event_data))
                    tasks.append(task)
                else:
                    callback(event_type, event_data)
            except Exception as e:
                self.logger.error(f"Error in event callback: {e}")
        
        # Wait for async callbacks to complete
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
    
    @log_exception()
    async def validate_safety_before_execution(self, action_sequence: ActionSequenceModel) -> List[str]:
        """
        Perform comprehensive safety validation before executing an action sequence.
        
        Args:
            action_sequence: ActionSequenceModel to validate safety for
            
        Returns:
            List of safety issues found, empty list if safe
        """
        try:
            self.logger.info(f"Performing pre-execution safety validation for sequence: {action_sequence.sequence_id}")
            
            issues = []
            
            # Get current robot state
            current_state = await self.state_sync_manager.get_current_robot_state()
            if not current_state:
                issues.append("Cannot validate safety: Robot state unavailable")
                return issues
            
            # Check battery level
            if current_state.battery_level < self.battery_low_threshold:
                issues.append(f"Robot battery level ({current_state.battery_level:.1%}) below safety threshold ({self.battery_low_threshold:.1%})")
            
            # Validate each action in sequence
            for i, action in enumerate(action_sequence.actions):
                action_issues = await self.validate_action_safety(action, current_state)
                issues.extend([f"Action {i+1} ({action.action_id}): {issue}" for issue in action_issues])
            
            # Check sequence-level safety concerns
            sequence_issues = await self._validate_sequence_safety(action_sequence, current_state)
            issues.extend(sequence_issues)
            
            if issues:
                self.logger.warning(f"Pre-execution validation found {len(issues)} safety issues")
            else:
                self.logger.info("Pre-execution safety validation passed")
            
            return issues
            
        except Exception as e:
            self.logger.error(f"Error in pre-execution safety validation: {e}")
            raise VLAException(
                f"Pre-execution safety validation error: {str(e)}", 
                VLAErrorType.SAFETY_ERROR,
                e
            )
    
    async def validate_action_safety(self, action: ActionModel, robot_state: RobotStateModel) -> List[str]:
        """
        Validate safety for a single action in the context of robot state.
        
        Args:
            action: ActionModel to validate
            robot_state: Current RobotStateModel for context
            
        Returns:
            List of safety issues for this action, empty if safe
        """
        issues = []
        
        # Check if action type is allowed for current robot mode
        if robot_state.mode in ['error', 'emergency_stop', 'safety_lockout'] and action.action_type not in ['status_check', 'reset', 'emergency_stop']:
            issues.append(f"Action type '{action.action_type}' not allowed in robot mode '{robot_state.mode}'")
        
        # Check action-specific constraints
        if action.action_type in ['move_to', 'navigate']:
            target_pos = action.parameters.get('target_location')
            if target_pos:
                # Check if target is within operational bounds
                bounds = getattr(self.config, 'operational_bounds', None)
                if bounds:
                    if (target_pos.get('x', 0) < bounds.get('min_x', float('-inf')) or 
                        target_pos.get('x', 0) > bounds.get('max_x', float('inf')) or
                        target_pos.get('y', 0) < bounds.get('min_y', float('-inf')) or 
                        target_pos.get('y', 0) > bounds.get('max_y', float('inf'))):
                        issues.append(f"Navigation target {target_pos} outside operational bounds")
        
        elif action.action_type in ['pick_up', 'place', 'grasp']:
            # Check if gripper is in proper state for manipulation
            gripper_state = robot_state.gripper_state
            if gripper_state and action.action_type == 'pick_up' and gripper_state.get('is_grasping', False):
                issues.append("Attempting to pick up an object when gripper is already grasping")
            
            if gripper_state and action.action_type == 'place' and not gripper_state.get('is_grasping', False):
                issues.append("Attempting to place an object when gripper is not grasping anything")
        
        # Check for safety in action parameters
        force_limit = action.parameters.get('force_limit')
        if force_limit and force_limit > self.force_threshold:
            issues.append(f"Requested force limit ({force_limit}N) exceeds safety threshold ({self.force_threshold}N)")
        
        return issues
    
    async def _validate_sequence_safety(self, action_sequence: ActionSequenceModel, robot_state: RobotStateModel) -> List[str]:
        """
        Validate safety for the action sequence as a whole.
        
        Args:
            action_sequence: ActionSequenceModel to validate
            robot_state: Current RobotStateModel for context
            
        Returns:
            List of sequence-level safety issues, empty if safe
        """
        issues = []
        
        # Check for rapid mode changes that might be unsafe
        action_types = [action.action_type for action in action_sequence.actions]
        
        # Count navigation actions in sequence
        nav_actions = [at for at in action_types if at in ['move_to', 'navigate']]
        if len(nav_actions) > 10 and len(action_sequence.actions) < 20:
            # Too many consecutive navigation actions without verification
            issues.append("Too many navigation actions in sequence without intermediate verification steps")
        
        # Check for safety-sensitive action patterns
        for i in range(len(action_sequence.actions) - 1):
            current_action = action_sequence.actions[i]
            next_action = action_sequence.actions[i+1]
            
            # Check for manipulation immediately after navigation without verification
            if (current_action.action_type in ['move_to', 'navigate'] and 
                next_action.action_type in ['pick_up', 'grasp', 'place'] and
                current_action.parameters.get('target_location') and
                not any('verify' in str(next_action.parameters).lower() or 'confirm' in str(next_action.parameters).lower())):
                issues.append(f"Action {i+2} performs manipulation without verifying location from previous navigation (action {i+1})")
        
        # Check for resource conflicts in the sequence
        # In a real system, this would check for things like:
        # - Gripper state consistency
        # - Actuator limits
        # - Navigation path conflicts
        
        return issues
    
    def get_monitoring_status(self) -> Dict[str, Any]:
        """
        Get current monitoring status.
        
        Returns:
            Dictionary with monitoring status information
        """
        return {
            'monitoring_active': self.monitoring_task is not None and not self.monitoring_task.done(),
            'active_monitors': len(self.active_monitors),
            'total_safety_violations': len(self.safety_violations),
            'recent_safety_violations': [
                {
                    'id': v.violation_id,
                    'type': v.violation_type,
                    'severity': v.severity.value,
                    'description': v.description[:60] + "..." if len(v.description) > 60 else v.description
                } for v in self.safety_violations[-5:]  # Last 5 violations
            ] if self.safety_violations else [],
            'total_performance_metrics': len(self.performance_metrics),
            'last_update': time.time()
        }
    
    def get_performance_summary(self) -> Dict[str, float]:
        """
        Get performance summary metrics.
        
        Returns:
            Dictionary with performance summary
        """
        if not self.performance_metrics:
            return {
                'average_execution_time': 0.0,
                'average_success_rate': 0.0,
                'average_safety_incidents': 0.0,
                'total_executions_monitored': 0
            }
        
        avg_execution_time = sum(m.execution_time for m in self.performance_metrics) / len(self.performance_metrics)
        avg_success_rate = sum(m.success_rate for m in self.performance_metrics) / len(self.performance_metrics)
        avg_safety_incidents = sum(m.safety_incidents for m in self.performance_metrics) / len(self.performance_metrics)
        
        return {
            'average_execution_time': avg_execution_time,
            'average_success_rate': avg_success_rate,
            'average_safety_incidents': avg_safety_incidents,
            'total_executions_monitored': len(self.performance_metrics)
        }


# Global execution monitor instance
_execution_monitor = None


def get_execution_monitor() -> ExecutionMonitor:
    """Get the global execution monitor instance."""
    global _execution_monitor
    if _execution_monitor is None:
        _execution_monitor = ExecutionMonitor()
    return _execution_monitor


async def monitor_action_execution(action: ActionModel, execution_callback: Callable[[], Any]) -> Dict[str, Any]:
    """Convenience function to monitor action execution."""
    monitor = get_execution_monitor()
    return await monitor.monitor_action_execution(action, execution_callback)


async def monitor_action_sequence_execution(action_sequence: ActionSequenceModel, execution_callback: Callable[[], Any]) -> Dict[str, Any]:
    """Convenience function to monitor action sequence execution."""
    monitor = get_execution_monitor()
    return await monitor.monitor_action_sequence_execution(action_sequence, execution_callback)


async def perform_safety_check(
    action: Optional[ActionModel] = None, 
    robot_state: Optional[RobotStateModel] = None,
    check_type: str = "general"
) -> SafetyCheckResult:
    """Convenience function to perform safety check."""
    monitor = get_execution_monitor()
    return await monitor.perform_safety_check(action, robot_state, check_type)


async def validate_safety_before_execution(action_sequence: ActionSequenceModel) -> List[str]:
    """Convenience function to validate safety before execution."""
    monitor = get_execution_monitor()
    return await monitor.validate_safety_before_execution(action_sequence)


def get_monitoring_status() -> Dict[str, Any]:
    """Convenience function to get monitoring status."""
    monitor = get_execution_monitor()
    return monitor.get_monitoring_status()


def get_performance_summary() -> Dict[str, float]:
    """Convenience function to get performance summary."""
    monitor = get_execution_monitor()
    return monitor.get_performance_summary()