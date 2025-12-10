"""
Action orchestration with dependencies for the Vision-Language-Action (VLA) module.

This module handles complex action sequences with dependencies, parallel execution,
and error handling for multi-step robotic tasks.
"""

import asyncio
import logging
from typing import Dict, List, Any, Optional, Set, Tuple
from dataclasses import dataclass
from enum import Enum
import time
import uuid

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import Action, ActionResponse, ExecutionStatus
from ..core.data_models import ActionModel, ActionResponseModel, ActionSequenceModel
from .action_sequencer import get_action_sequencer
from .robot_controller import get_robot_controller
from .safety_monitor import get_safety_monitor


@dataclass
class ActionDependency:
    """Represents a dependency between actions."""
    dependent_action_id: str
    prerequisite_action_id: str
    dependency_type: str  # 'before', 'after', 'during', 'condition', 'resource'


@dataclass
class ActionOrchestrationState:
    """State information for an orchestration."""
    orchestration_id: str
    action_status: Dict[str, ExecutionStatus]
    execution_order: List[str]
    active_actions: Set[str]
    completed_actions: Set[str]
    failed_actions: Set[str]
    start_time: float
    end_time: Optional[float] = None
    total_actions: int = 0


class ActionOrchestrator:
    """
    Orchestrates complex action sequences with dependencies and parallel execution.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Action sequencer and robot controller
        self.action_sequencer = get_action_sequencer()
        self.robot_controller = get_robot_controller()
        self.safety_monitor = get_safety_monitor()
        
        # Track orchestration states
        self.orchestration_states: Dict[str, ActionOrchestrationState] = {}
        
        # Resource managers for different robot components
        self.resource_managers = {
            'navigation': RobotResourceManager(max_concurrent=1),  # Only one navigation at a time
            'manipulation': RobotResourceManager(max_concurrent=1),  # Only one manipulation at a time
            'perception': RobotResourceManager(max_concurrent=2),   # Multiple perception tasks possible
            'communication': RobotResourceManager(max_concurrent=3) # Multiple communication tasks
        }
        
        self.logger.info("ActionOrchestrator initialized")
    
    @log_exception()
    async def execute_orchestrated_sequence(self, action_sequence: ActionSequenceModel) -> List[ActionResponseModel]:
        """
        Execute an orchestrated action sequence with dependencies and parallel execution.
        
        Args:
            action_sequence: ActionSequenceModel with dependencies to execute
            
        Returns:
            List of ActionResponseModel for each executed action
        """
        try:
            orchestration_id = f"orchestration_{uuid.uuid4().hex[:8]}"
            self.logger.info(f"Starting orchestrated sequence execution: {orchestration_id}")
            
            # Analyze dependencies and create execution plan
            execution_plan = await self._analyze_dependencies(action_sequence)
            
            # Initialize orchestration state
            orchestration_state = ActionOrchestrationState(
                orchestration_id=orchestration_id,
                action_status={action.action_id: ExecutionStatus.PENDING for action in action_sequence.actions},
                execution_order=[],
                active_actions=set(),
                completed_actions=set(),
                failed_actions=set(),
                start_time=time.time(),
                total_actions=len(action_sequence.actions)
            )
            self.orchestration_states[orchestration_id] = orchestration_state
            
            # Execute the orchestrated sequence
            responses = await self._execute_with_dependencies(
                action_sequence, 
                execution_plan, 
                orchestration_state
            )
            
            # Update orchestration state
            orchestration_state.end_time = time.time()
            
            # Calculate completion percentage
            completed_count = len(orchestration_state.completed_actions)
            completion_percentage = completed_count / orchestration_state.total_actions if orchestration_state.total_actions > 0 else 0.0
            
            # Determine overall sequence status
            if len(orchestration_state.failed_actions) == 0:
                sequence_status = ExecutionStatus.COMPLETED
            elif completed_count == 0:
                sequence_status = ExecutionStatus.FAILED
            else:
                sequence_status = ExecutionStatus.PARTIAL_SUCCESS
            
            # Update action sequence status
            action_sequence.execution_status = sequence_status
            
            self.logger.info(f"Orchestrated sequence {orchestration_id} completed: {sequence_status.name} ({completion_percentage:.2%})")
            
            # Remove from active orchestrations
            del self.orchestration_states[orchestration_id]
            
            return responses
            
        except Exception as e:
            self.logger.error(f"Error in orchestrated execution: {e}")
            raise VLAException(
                f"Orchestrated execution error: {str(e)}", 
                VLAErrorType.ACTION_EXECUTION_ERROR,
                e
            )
    
    @log_exception()
    async def _analyze_dependencies(self, action_sequence: ActionSequenceModel) -> Dict[str, Any]:
        """
        Analyze action dependencies to create an execution plan.
        
        Args:
            action_sequence: ActionSequenceModel with potential dependencies
            
        Returns:
            Dictionary with execution plan information
        """
        # In a real implementation, this would parse dependencies from the action sequence
        # For now, we'll create a simple dependency graph based on action parameters
        # This is a simplified implementation - a real system would have more sophisticated dependency handling
        
        dependency_graph = {action.action_id: [] for action in action_sequence.actions}
        independent_actions = []
        
        # Simulate extracting dependencies from action parameters
        # In a real system, dependencies would be defined in the action objects themselves
        for i, action in enumerate(action_sequence.actions):
            # For this example, we'll define some simple dependency rules
            if action.action_type == 'place' and i > 0:
                # Place action depends on previous pick_up action
                for j in range(i-1, -1, -1):
                    if action_sequence.actions[j].action_type == 'pick_up':
                        dependency_graph[action.action_id].append(action_sequence.actions[j].action_id)
                        break
            
            elif action.action_type == 'navigate' and i > 0:
                # Navigation might depend on detection results
                for j in range(i-1, -1, -1):
                    if action_sequence.actions[j].action_type == 'detect_object':
                        dependency_graph[action.action_id].append(action_sequence.actions[j].action_id)
                        break
        
        # Identify independent actions (those with no dependencies)
        for action in action_sequence.actions:
            if not dependency_graph[action.action_id]:
                independent_actions.append(action.action_id)
        
        # Create execution plan
        execution_plan = {
            'dependency_graph': dependency_graph,
            'independent_actions': independent_actions,
            'execution_order': self._topological_sort(dependency_graph, action_sequence.actions)
        }
        
        return execution_plan
    
    def _topological_sort(self, dependency_graph: Dict[str, List[str]], actions: List[ActionModel]) -> List[str]:
        """
        Perform topological sort on the dependency graph to determine execution order.
        
        Args:
            dependency_graph: Graph mapping action IDs to their prerequisites
            actions: List of actions in the sequence
            
        Returns:
            List of action IDs in execution order
        """
        # Build dependency graph for topological sort
        in_degree = {action.action_id: 0 for action in actions}
        adjacency_list = {action.action_id: [] for action in actions}
        
        # Calculate in-degrees and adjacency list
        for action_id, dependencies in dependency_graph.items():
            for dep_id in dependencies:
                if dep_id in in_degree:  # Make sure dependency exists
                    in_degree[action_id] += 1
                    adjacency_list[dep_id].append(action_id)
        
        # Kahn's algorithm for topological sorting
        queue = []
        for action_id, degree in in_degree.items():
            if degree == 0:
                queue.append(action_id)
        
        execution_order = []
        while queue:
            current = queue.pop(0)
            execution_order.append(current)
            
            for neighbor in adjacency_list[current]:
                in_degree[neighbor] -= 1
                if in_degree[neighbor] == 0:
                    queue.append(neighbor)
        
        # Check for cycles by seeing if all nodes were processed
        if len(execution_order) != len(actions):
            # There's a cycle in the dependency graph
            raise VLAException(
                "Circular dependency detected in action sequence", 
                VLAErrorType.ACTION_EXECUTION_ERROR
            )
        
        return execution_order
    
    async def _execute_with_dependencies(
        self, 
        action_sequence: ActionSequenceModel, 
        execution_plan: Dict[str, Any], 
        orchestration_state: ActionOrchestrationState
    ) -> List[ActionResponseModel]:
        """
        Execute actions respecting dependencies and resource constraints.
        
        Args:
            action_sequence: The action sequence to execute
            execution_plan: Plan with dependencies and execution order
            orchestration_state: Current orchestration state
            
        Returns:
            List of ActionResponseModel for each executed action
        """
        responses = []
        failed_actions = set()
        completed_actions = set()
        
        # Map from action_id to ActionModel for easy lookup
        action_lookup = {action.action_id: action for action in action_sequence.actions}
        
        # Execute actions respecting dependencies
        pending_actions = set(action.action_id for action in action_sequence.actions)
        executed_actions = set()
        
        while pending_actions:
            # Find actions whose dependencies are satisfied
            ready_actions = []
            
            for action_id in pending_actions:
                action = action_lookup[action_id]
                
                # Check if all dependencies are completed
                dependencies = execution_plan['dependency_graph'][action_id]
                satisfied_deps = all(dep_id in executed_actions for dep_id in dependencies)
                
                if satisfied_deps:
                    # Check if action requires resources that are available
                    resource_type = self._get_resource_type(action.action_type)
                    if self.resource_managers[resource_type].can_acquire(action_id):
                        ready_actions.append(action)
            
            if not ready_actions:
                # If no actions are ready, check if there are any circular dependencies
                remaining_actions = list(pending_actions)
                self.logger.warning(f"Possible circular dependencies, unable to execute: {remaining_actions}")
                
                # For now, we'll break the loop to prevent infinite waiting
                # In a real system, you'd want more sophisticated handling
                break
            
            # Execute ready actions in parallel
            tasks = []
            for action in ready_actions:
                resource_type = self._get_resource_type(action.action_type)
                acquired = self.resource_managers[resource_type].acquire(action.action_id)
                
                if acquired:
                    # Mark action as active
                    orchestration_state.active_actions.add(action.action_id)
                    
                    # Create task to execute action
                    task = asyncio.create_task(self._execute_single_action_with_error_handling(action))
                    tasks.append((task, action))
                    
                    # Remove from pending actions
                    pending_actions.remove(action.action_id)
            
            # Execute all ready actions in parallel
            if tasks:
                # Wait for all tasks to complete
                results = await asyncio.gather(*[task[0] for task in tasks], return_exceptions=True)
                
                # Process results
                for i, (task, action) in enumerate(tasks):
                    result = results[i]
                    
                    # Release resource regardless of success/failure
                    resource_type = self._get_resource_type(action.action_type)
                    self.resource_managers[resource_type].release(action.action_id)
                    
                    # Remove from active actions
                    orchestration_state.active_actions.remove(action.action_id)
                    
                    if isinstance(result, Exception):
                        # Handle execution error
                        self.logger.error(f"Error executing action {action.action_id}: {result}")
                        failed_actions.add(action.action_id)
                        
                        # Create error response
                        error_response = ActionResponseModel.create(
                            sequence_id=action_sequence.sequence_id,
                            status=ExecutionStatus.FAILED,
                            completion_percentage=0.0,
                            result_summary=f"Action execution failed: {str(result)}",
                            action_logs=[],
                            timestamp=time.time()
                        )
                        responses.append(error_response)
                        
                        # Update orchestration state
                        orchestration_state.action_status[action.action_id] = ExecutionStatus.FAILED
                        orchestration_state.failed_actions.add(action.action_id)
                    else:
                        response = result
                        responses.append(response)
                        completed_actions.add(action.action_id)
                        
                        # Update orchestration state
                        orchestration_state.action_status[action.action_id] = response.status
                        orchestration_state.completed_actions.add(action.action_id)
                        
                        self.logger.debug(f"Action {action.action_id} completed with status: {response.status.name}")
                
                # Check if we should stop on failure
                continue_on_failure = self.config.get('continue_on_action_failure', False)
                if not continue_on_failure and failed_actions:
                    self.logger.info("Stopping execution due to action failure")
                    break
        
        # Handle any remaining unexecuted actions
        for action_id in pending_actions:
            self.logger.warning(f"Action {action_id} not executed due to unresolved dependencies")
            failed_actions.add(action_id)
            
            # Create failure response for unexecuted action
            action = action_lookup[action_id]
            error_response = ActionResponseModel.create(
                sequence_id=action_sequence.sequence_id,
                status=ExecutionStatus.FAILED,
                completion_percentage=0.0,
                result_summary=f"Action not executed - dependencies not satisfied: {execution_plan['dependency_graph'][action_id]}",
                action_logs=[],
                timestamp=time.time()
            )
            responses.append(error_response)
            
            # Update orchestration state
            orchestration_state.action_status[action_id] = ExecutionStatus.FAILED
            orchestration_state.failed_actions.add(action_id)
        
        return responses
    
    async def _execute_single_action_with_error_handling(self, action: ActionModel) -> ActionResponseModel:
        """
        Execute a single action with error handling and safety validation.
        
        Args:
            action: ActionModel to execute
            
        Returns:
            ActionResponseModel with execution results
        """
        try:
            # Validate action safety before execution
            safety_violations = await self.safety_monitor.check_safety_before_action(action)
            if safety_violations:
                # If there are safety violations, handle them appropriately
                error_msg = f"Safety violations detected: {[v.description for v in safety_violations]}"
                self.logger.error(error_msg)
                
                # Handle safety violations
                for violation in safety_violations:
                    await self.safety_monitor.handle_safety_violation(violation)
                
                # Return a failure response
                return ActionResponseModel.create(
                    sequence_id=action.sequence_id or "unknown_sequence",
                    status=ExecutionStatus.FAILED,
                    completion_percentage=0.0,
                    result_summary=error_msg,
                    action_logs=[],
                    timestamp=time.time()
                )
            
            # Execute the action
            response = await self.robot_controller.execute_action(action)
            
            return response
            
        except Exception as e:
            self.logger.error(f"Error executing action {action.action_id}: {e}")
            raise VLAException(
                f"Action execution error: {str(e)}", 
                VLAErrorType.ACTION_EXECUTION_ERROR,
                e
            )
    
    def _get_resource_type(self, action_type: str) -> str:
        """
        Map an action type to the appropriate resource type for resource management.
        
        Args:
            action_type: Type of action
            
        Returns:
            Resource type string
        """
        action_to_resource = {
            'move_to': 'navigation',
            'navigate': 'navigation',
            'go_to': 'navigation',
            'pick_up': 'manipulation',
            'place': 'manipulation',
            'grasp': 'manipulation',
            'release': 'manipulation',
            'detect_object': 'perception',
            'recognize_object': 'perception',
            'localize': 'navigation'
        }
        
        return action_to_resource.get(action_type, 'communication')
    
    @log_exception()
    async def execute_parallelizable_actions(
        self, 
        actions: List[ActionModel], 
        max_concurrent: int = 5
    ) -> List[ActionResponseModel]:
        """
        Execute a list of actions that can be run in parallel.
        
        Args:
            actions: List of ActionModel to execute in parallel
            max_concurrent: Maximum number of parallel executions
            
        Returns:
            List of ActionResponseModel for each executed action
        """
        responses = []
        
        if not actions:
            return responses
        
        # Create batches of actions to respect concurrent limits
        for i in range(0, len(actions), max_concurrent):
            batch = actions[i:i + max_concurrent]
            batch_tasks = []
            
            for action in batch:
                task = asyncio.create_task(self._execute_single_action_with_error_handling(action))
                batch_tasks.append(task)
            
            # Execute batch in parallel
            batch_results = await asyncio.gather(*batch_tasks, return_exceptions=True)
            
            # Process results
            for j, result in enumerate(batch_results):
                if isinstance(result, Exception):
                    self.logger.error(f"Error in parallel action execution: {result}")
                    
                    # Create failure response
                    error_response = ActionResponseModel.create(
                        sequence_id=actions[i+j].sequence_id or "unknown_sequence",
                        status=ExecutionStatus.FAILED,
                        completion_percentage=0.0,
                        result_summary=f"Parallel execution failed: {str(result)}",
                        action_logs=[],
                        timestamp=time.time()
                    )
                    responses.append(error_response)
                else:
                    responses.append(result)
        
        return responses
    
    @log_exception()
    async def schedule_conditional_execution(
        self,
        condition_action: ActionModel,
        success_actions: List[ActionModel],
        failure_actions: List[ActionModel]
    ) -> List[ActionResponseModel]:
        """
        Execute actions based on the result of a condition action.
        
        Args:
            condition_action: ActionModel that determines the conditional branch
            success_actions: Actions to execute if condition succeeds
            failure_actions: Actions to execute if condition fails
            
        Returns:
            List of ActionResponseModel for all executed actions
        """
        responses = []
        
        # Execute the condition action first
        condition_response = await self._execute_single_action_with_error_handling(condition_action)
        responses.append(condition_response)
        
        # Based on the result, execute the appropriate branch
        if condition_response.status == ExecutionStatus.COMPLETED:
            self.logger.info("Condition passed, executing success branch")
            branch_responses = await self.execute_parallelizable_actions(success_actions)
            responses.extend(branch_responses)
        else:
            self.logger.info("Condition failed, executing failure branch")
            branch_responses = await self.execute_parallelizable_actions(failure_actions)
            responses.extend(branch_responses)
        
        return responses
    
    def get_orchestration_status(self, orchestration_id: str) -> Optional[ActionOrchestrationState]:
        """
        Get the status of a running orchestration.
        
        Args:
            orchestration_id: ID of the orchestration to check
            
        Returns:
            ActionOrchestrationState or None if not found
        """
        return self.orchestration_states.get(orchestration_id)
    
    def get_resource_utilization(self) -> Dict[str, Dict[str, Any]]:
        """
        Get current resource utilization across all resource managers.
        
        Returns:
            Dictionary with resource utilization information
        """
        utilization = {}
        
        for resource_type, manager in self.resource_managers.items():
            utilization[resource_type] = {
                'max_concurrent': manager.max_concurrent,
                'current_usage': len(manager.active_resources),
                'utilization_percentage': len(manager.active_resources) / manager.max_concurrent if manager.max_concurrent > 0 else 0,
                'active_actions': list(manager.active_resources.keys())
            }
        
        return utilization


class RobotResourceManager:
    """
    Manages robot resources to prevent conflicts when multiple actions require the same resources.
    """
    
    def __init__(self, max_concurrent: int = 1):
        self.max_concurrent = max_concurrent
        self.active_resources: Dict[str, float] = {}  # action_id -> start_time
        self.request_queue: List[Tuple[str, asyncio.Event]] = []  # (action_id, ready_event)
        self.lock = asyncio.Lock()
    
    async def acquire(self, action_id: str) -> bool:
        """
        Attempt to acquire a resource for an action.
        
        Args:
            action_id: ID of the action requesting the resource
            
        Returns:
            True if acquired, False if denied
        """
        async with self.lock:
            if len(self.active_resources) < self.max_concurrent:
                # Resource available, acquire it
                self.active_resources[action_id] = time.time()
                return True
            else:
                # Resource not available, add to queue
                event = asyncio.Event()
                self.request_queue.append((action_id, event))
                return False
    
    def release(self, action_id: str):
        """
        Release a resource.
        
        Args:
            action_id: ID of the action releasing the resource
        """
        async with self.lock:
            if action_id in self.active_resources:
                del self.active_resources[action_id]
                
                # Check if queued requests can now proceed
                if self.request_queue and len(self.active_resources) < self.max_concurrent:
                    next_action_id, event = self.request_queue.pop(0)
                    self.active_resources[next_action_id] = time.time()
                    event.set()
    
    def can_acquire(self, action_id: str) -> bool:
        """
        Check if a resource can be acquired without blocking.
        
        Args:
            action_id: ID of the action checking for resource availability
            
        Returns:
            True if resource can be acquired immediately, False otherwise
        """
        return len(self.active_resources) < self.max_concurrent or action_id in self.active_resources
    
    async def wait_for_resource(self, action_id: str, timeout: float = 30.0) -> bool:
        """
        Wait for a resource to become available.
        
        Args:
            action_id: ID of the action waiting for the resource
            timeout: Maximum time to wait in seconds
            
        Returns:
            True if resource acquired, False if timeout reached
        """
        async with self.lock:
            if len(self.active_resources) < self.max_concurrent:
                # Resource is available now
                self.active_resources[action_id] = time.time()
                return True
            
            # Add to queue and wait
            event = asyncio.Event()
            self.request_queue.append((action_id, event))
            
        try:
            await asyncio.wait_for(event.wait(), timeout=timeout)
            # When the event is set, the resource has been acquired
            # The caller should call acquire() to update the manager state
            return True
        except asyncio.TimeoutError:
            # Remove from queue if timeout
            async with self.lock:
                self.request_queue = [(aid, evt) for aid, evt in self.request_queue if aid != action_id]
            return False


# Global orchestrator instance
_action_orchestrator = None


def get_action_orchestrator() -> ActionOrchestrator:
    """
    Get the global action orchestrator instance.
    
    Returns:
        ActionOrchestrator instance
    """
    global _action_orchestrator
    if _action_orchestrator is None:
        _action_orchestrator = ActionOrchestrator()
    return _action_orchestrator


async def execute_orchestrated_sequence(action_sequence: ActionSequenceModel) -> List[ActionResponseModel]:
    """
    Convenience function to execute an orchestrated action sequence.
    
    Args:
        action_sequence: ActionSequenceModel to execute with dependencies
        
    Returns:
        List of ActionResponseModel for each action
    """
    orchestrator = get_action_orchestrator()
    return await orchestrator.execute_orchestrated_sequence(action_sequence)


async def execute_parallelizable_actions(actions: List[ActionModel], max_concurrent: int = 5) -> List[ActionResponseModel]:
    """
    Convenience function to execute actions that can run in parallel.
    
    Args:
        actions: List of ActionModel to execute in parallel
        max_concurrent: Maximum number of parallel executions
        
    Returns:
        List of ActionResponseModel for each action
    """
    orchestrator = get_action_orchestrator()
    return await orchestrator.execute_parallelizable_actions(actions, max_concurrent)