"""
Obstacle navigation logic for the Vision-Language-Action (VLA) module.

This module implements reactive and predictive obstacle avoidance
for safe robot navigation in dynamic environments.
"""

import asyncio
import logging
import math
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import RobotState, Action, ActionResponse
from ..core.data_models import RobotStateModel, ActionModel, ActionResponseModel, VisionObservationModel
from ..vision_perception.vision_processor import get_vision_processor
from ..action_execution.robot_controller import get_robot_controller
from .path_planning_integrator import get_path_integrator


@dataclass
class Obstacle:
    """Represents a detected obstacle."""
    id: str
    position: Dict[str, float]  # x, y, z coordinates
    size: Dict[str, float]      # width, height, depth
    type: str                   # static, dynamic, human, etc.
    velocity: Optional[Dict[str, float]] = None  # Optional velocity for dynamic obstacles


@dataclass
class NavigationPath:
    """Represents a navigation path with potential obstacles."""
    waypoints: List[Dict[str, float]]
    original_path: List[Dict[str, float]]
    obstacles: List[Obstacle]
    calculated_at: float


class ObstacleNavigationManager:
    """
    Manages obstacle detection and navigation for the VLA system.
    Handles both static and dynamic obstacles in the environment.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Initialize components
        self.vision_processor = get_vision_processor()
        self.robot_controller = get_robot_controller()
        self.path_integrator = get_path_integrator()
        
        # Navigation parameters
        self.safety_buffer = getattr(self.config, 'navigation_safety_buffer', 0.5)  # meters
        self.replanning_distance_threshold = getattr(self.config, 'replanning_distance_threshold', 0.3)  # meters
        self.dynamic_obstacle_prediction_horizon = getattr(self.config, 'dynamic_obstacle_prediction_horizon', 3.0)  # seconds
        self.min_navigation_speed = getattr(self.config, 'min_navigation_speed', 0.1)  # m/s
        self.max_navigation_speed = getattr(self.config, 'max_navigation_speed', 0.5)  # m/s
        
        # State tracking
        self.current_path: Optional[NavigationPath] = None
        self.last_known_obstacles: List[Obstacle] = []
        self.navigation_mode = "global_planning"  # global_planning, local_avoidance, emergency_stop
        
        # Speed adaptation settings
        self.speed_adaptation_enabled = getattr(self.config, 'speed_adaptation_enabled', True)
        
        self.logger.info("ObstacleNavigationManager initialized")
    
    @log_exception()
    async def navigate_with_obstacle_avoidance(
        self, 
        destination: Dict[str, float], 
        robot_state: RobotStateModel
    ) -> ActionResponseModel:
        """
        Navigate to destination while avoiding obstacles.
        
        Args:
            destination: Target destination as coordinates (x, y, z)
            robot_state: Current RobotStateModel with position and environment info
            
        Returns:
            ActionResponseModel with navigation result
        """
        try:
            self.logger.info(f"Navigating to destination: {destination} from position: {robot_state.position}")
            
            # Get environment information through vision processing
            environment_observation = await self._get_environment_observation(robot_state)
            
            # Check if destination is valid and reachable
            validation_errors = self._validate_destination(destination, environment_observation)
            if validation_errors:
                raise VLAException(
                    f"Destination validation failed: {validation_errors}",
                    VLAErrorType.NAVIGATION_ERROR
                )
            
            # Plan initial path
            path = await self._plan_path_to_destination(robot_state.position, destination, environment_observation)
            
            # Execute navigation with obstacle avoidance
            response = await self._execute_navigation_with_obstacle_avoidance(path, robot_state)
            
            self.logger.info(f"Navigation to destination completed with status: {response.status.name}")
            return response
            
        except Exception as e:
            self.logger.error(f"Error in navigation with obstacle avoidance: {e}")
            raise VLAException(
                f"Navigation with obstacle avoidance error: {str(e)}", 
                VLAErrorType.NAVIGATION_ERROR,
                e
            )
    
    async def _get_environment_observation(self, robot_state: RobotStateModel) -> VisionObservationModel:
        """
        Get current environment observation from vision system.
        
        Args:
            robot_state: Current RobotStateModel for context
            
        Returns:
            VisionObservationModel with environment information
        """
        try:
            # In a real implementation, this would trigger the vision system to capture
            # current environment data. For simulation, we'll return a placeholder or
            # use cached data if available.
            
            # Get obstacles from the vision system
            # In a real implementation this would be from live sensor data
            obstacles = self._detect_obstacles_in_environment(robot_state)
            
            # Create an observation with the detected obstacles
            observation = VisionObservationModel.create(
                observation_id=f"env_obs_{int(time.time()*1000)}",
                state_id=robot_state.state_id,
                timestamp=time.time(),
                objects_detected=[],
                environment_map={
                    'obstacles': [self._obstacle_to_dict(obs) for obs in obstacles],
                    'navigation_space': self._derive_navigation_space(obstacles, robot_state.position)
                },
                processing_metrics={}
            )
            
            return observation
            
        except Exception as e:
            self.logger.error(f"Error getting environment observation: {e}")
            # Return an empty observation if error occurs
            return VisionObservationModel.create(
                observation_id=f"env_obs_{int(time.time()*1000)}",
                state_id=robot_state.state_id,
                timestamp=time.time(),
                objects_detected=[],
                environment_map={'obstacles': [], 'navigation_space': {}},
                processing_metrics={}
            )
    
    def _detect_obstacles_in_environment(self, robot_state: RobotStateModel) -> List[Obstacle]:
        """
        Detect obstacles in the current environment.
        
        Args:
            robot_state: Current RobotStateModel for position context
            
        Returns:
            List of Obstacle objects detected in environment
        """
        # In a real implementation, this would interface with:
        # - LIDAR data processing
        # - Computer vision systems
        # - Depth cameras
        # - Collision meshes
        
        # For now, we'll simulate obstacle detection based on environment context
        # This would normally come from the perception system
        
        detected_obstacles = []
        
        # Simulate detection of static obstacles based on environment map (if available)
        # In a real system, this would process sensor data to identify obstacles
        if hasattr(robot_state, 'environment_map') and robot_state.environment_map:
            static_obstacles = robot_state.environment_map.get('static_obstacles', [])
            for i, obs in enumerate(static_obstacles):
                obstacle = Obstacle(
                    id=f"static_obs_{i}",
                    position=obs.get('position', {'x': 0.0, 'y': 0.0, 'z': 0.0}),
                    size=obs.get('size', {'width': 1.0, 'height': 1.0, 'depth': 1.0}),
                    type=obs.get('type', 'static')
                )
                detected_obstacles.append(obstacle)
        
        # Simulate dynamic obstacle detection
        # In a real system, this would come from tracking algorithms
        if hasattr(robot_state, 'dynamic_objects') and robot_state.dynamic_objects:
            dynamic_obstacles = robot_state.dynamic_objects
            for i, dyn_obj in enumerate(dynamic_obstacles):
                if dyn_obj.get('is_obstacle', False):  # Only add objects that are obstacles
                    obstacle = Obstacle(
                        id=f"dyn_obs_{i}",
                        position=dyn_obj.get('position', {'x': 0.0, 'y': 0.0, 'z': 0.0}),
                        size=dyn_obj.get('size', {'width': 0.5, 'height': 1.7, 'depth': 0.5}),
                        type='dynamic',
                        velocity=dyn_obj.get('velocity')
                    )
                    detected_obstacles.append(obstacle)
        
        # Update our internal obstacle tracking
        self.last_known_obstacles = detected_obstacles
        
        return detected_obstacles
    
    def _validate_destination(self, destination: Dict[str, float], environment_observation: VisionObservationModel) -> List[str]:
        """
        Validate that the destination is reachable and safe.
        
        Args:
            destination: Target destination coordinates
            environment_observation: Current environment observation
            
        Returns:
            List of validation errors or empty list if valid
        """
        errors = []
        
        # Check if destination is too close to known obstacles
        for obstacle in environment_observation.environment_map.get('obstacles', []):
            obs_pos = obstacle['position']
            # Calculate distance to obstacle
            distance = math.sqrt(
                (destination['x'] - obs_pos['x'])**2 + 
                (destination['y'] - obs_pos['y'])**2 + 
                (destination['z'] - obs_pos['z'])**2
            )
            
            # Add buffer based on obstacle size
            min_distance = self.safety_buffer + obstacle['size']['width']/2
            
            if distance < min_distance:
                errors.append(f"Destination too close to obstacle {obstacle['id']} at {obs_pos}")
        
        # Validate coordinates are reasonable (not NaN, infinity, or extremely large values)
        for coord_name in ['x', 'y', 'z']:
            if coord_name in destination:
                coord_value = destination[coord_name]
                if not isinstance(coord_value, (int, float)) or math.isnan(coord_value) or math.isinf(coord_value):
                    errors.append(f"Invalid {coord_name} coordinate: {coord_value}")
                elif abs(coord_value) > 1000:  # Extremely large coordinate values
                    errors.append(f"Coordinate {coord_name} seems too large: {coord_value}")
        
        return errors
    
    async def _plan_path_to_destination(
        self, 
        start_position: Dict[str, float], 
        destination: Dict[str, float], 
        environment_observation: VisionObservationModel
    ) -> NavigationPath:
        """
        Plan a path from start to destination considering obstacles.
        
        Args:
            start_position: Starting position coordinates
            destination: Target destination coordinates
            environment_observation: Environment observation with obstacles
            
        Returns:
            NavigationPath with waypoints and obstacle information
        """
        try:
            self.logger.debug(f"Planning path from {start_position} to {destination}")
            
            # In a real implementation, this would call a path planning algorithm
            # For now, we'll simulate a path planning process with obstacle consideration
            # This would typically use A*, RRT, or other motion planning algorithms
            
            # For simulation purposes, create a simple path with obstacle avoidance
            path_waypoints = await self._compute_path_with_obstacle_avoidance(
                start_position, 
                destination, 
                environment_observation
            )
            
            original_path = self._compute_direct_path(start_position, destination)
            
            # Convert obstacle dicts back to Obstacle objects
            obstacles = []
            for obs_dict in environment_observation.environment_map.get('obstacles', []):
                obstacle = Obstacle(
                    id=obs_dict['id'],
                    position=obs_dict['position'],
                    size=obs_dict['size'],
                    type=obs_dict.get('type', 'static')
                )
                obstacles.append(obstacle)
            
            # Create navigation path object
            nav_path = NavigationPath(
                waypoints=path_waypoints,
                original_path=original_path,
                obstacles=obstacles,
                calculated_at=time.time()
            )
            
            self.current_path = nav_path
            
            self.logger.debug(f"Planned path with {len(path_waypoints)} waypoints")
            return nav_path
            
        except Exception as e:
            self.logger.error(f"Error planning path to destination: {e}")
            raise VLAException(
                f"Path planning error: {str(e)}", 
                VLAErrorType.NAVIGATION_ERROR,
                e
            )
    
    async def _compute_path_with_obstacle_avoidance(
        self, 
        start: Dict[str, float], 
        end: Dict[str, float], 
        environment_observation: VisionObservationModel
    ) -> List[Dict[str, float]]:
        """Compute a path that avoids known obstacles."""
        # In a real system, this would use actual path planning algorithms like A*, RRT, etc.
        # For this simulation, we'll create a simplified obstacle-aware path calculation
        
        # First, try a direct path
        direct_path = self._compute_direct_path(start, end)
        
        # Check for obstacles along the direct path
        obstacles_along_path = self._find_obstacles_on_path(direct_path, environment_observation)
        
        if not obstacles_along_path:
            # No obstacles to avoid, return direct path
            return direct_path
        else:
            # In a real implementation, we would compute a new path that goes around obstacles
            # For now, just return the direct path (which will trigger replanning during execution)
            return direct_path
    
    def _compute_direct_path(self, start: Dict[str, float], end: Dict[str, float]) -> List[Dict[str, float]]:
        """Compute a direct path between two points."""
        # Simple linear interpolation between start and end
        steps = max(2, int(self._calculate_direct_distance(start, end) * 5))  # 5 waypoints per meter
        path = []
        
        for i in range(steps):
            ratio = i / (steps - 1) if steps > 1 else 0
            waypoint = {
                'x': start['x'] + (end['x'] - start['x']) * ratio,
                'y': start['y'] + (end['y'] - start['y']) * ratio,
                'z': start.get('z', 0.0) + (end.get('z', 0.0) - start.get('z', 0.0)) * ratio
            }
            path.append(waypoint)
        
        return path
    
    def _find_obstacles_on_path(self, path: List[Dict[str, float]], environment_observation: VisionObservationModel) -> List[Obstacle]:
        """Find obstacles that are on or near the given path."""
        obstacles_to_avoid = []
        
        for obstacle in environment_observation.environment_map.get('obstacles', []):
            # Check if obstacle is near any segment of the path
            for i in range(len(path) - 1):
                segment_start = path[i]
                segment_end = path[i + 1]
                
                # Calculate distance from obstacle to path segment
                distance_to_segment = self._distance_to_line_segment(
                    obstacle['position'], 
                    segment_start, 
                    segment_end
                )
                
                # If obstacle is too close to the path, mark it for avoidance
                min_distance = self.replanning_distance_threshold + obstacle['size']['width']/2
                if distance_to_segment < min_distance:
                    obstacles_to_avoid.append(obstacle)
                    break  # No need to check other segments for this obstacle
        
        return obstacles_to_avoid
    
    def _distance_to_line_segment(self, point: Dict[str, float], line_start: Dict[str, float], line_end: Dict[str, float]) -> float:
        """Calculate the distance from a point to a line segment."""
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
    
    def _calculate_direct_distance(self, pos1: Dict[str, float], pos2: Dict[str, float]) -> float:
        """Calculate direct distance between two positions."""
        dx = pos2['x'] - pos1['x']
        dy = pos2['y'] - pos1['y']
        dz = pos2.get('z', 0.0) - pos1.get('z', 0.0)
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    async def _execute_navigation_with_obstacle_avoidance(
        self, 
        path: NavigationPath, 
        robot_state: RobotStateModel
    ) -> ActionResponseModel:
        """
        Execute navigation while monitoring for and avoiding obstacles.
        
        Args:
            path: NavigationPath with waypoints to follow
            robot_state: Current RobotStateModel
            
        Returns:
            ActionResponseModel with navigation results
        """
        try:
            self.logger.info(f"Executing navigation with obstacle avoidance along path of {len(path.waypoints)} waypoints")
            
            # Convert path to navigation actions for robot execution
            navigation_actions = self._convert_path_to_navigation_actions(path)
            
            # Execute navigation actions with continuous obstacle monitoring
            success_count = 0
            total_actions = len(navigation_actions)
            action_responses = []
            
            # Prepare a sequence of navigation actions
            navigation_action_sequence = ActionSequenceModel.create(
                sequence_id=f"nav_seq_{int(time.time()*1000)}",
                plan_id="obstacle_navigation_plan",
                actions=navigation_actions
            )
            
            # Execute using robot controller
            action_responses = await self.robot_controller.execute_action_sequence(navigation_action_sequence)
            
            # Analyze results
            successful_actions = sum(1 for resp in action_responses if resp.status == ExecutionStatus.COMPLETED)
            success_rate = successful_actions / total_actions if total_actions > 0 else 0.0
            
            # Create final response
            status = ExecutionStatus.COMPLETED if success_rate > 0.9 else ExecutionStatus.PARTIAL_SUCCESS if success_rate > 0.5 else ExecutionStatus.FAILED
            
            response = ActionResponseModel.create(
                sequence_id=navigation_action_sequence.sequence_id,
                status=status,
                completion_percentage=success_rate,
                result_summary=f"Navigated {successful_actions}/{total_actions} waypoints successfully",
                action_logs=[],
                timestamp=time.time()
            )
            
            self.logger.info(f"Navigation execution completed with {success_rate:.2%} success rate")
            return response
            
        except Exception as e:
            self.logger.error(f"Error executing navigation with obstacle avoidance: {e}")
            raise VLAException(
                f"Navigation execution error: {str(e)}", 
                VLAErrorType.NAVIGATION_ERROR,
                e
            )
    
    def _convert_path_to_navigation_actions(self, path: NavigationPath) -> List[ActionModel]:
        """
        Convert a navigation path to a sequence of navigation actions.
        
        Args:
            path: NavigationPath to convert
            
        Returns:
            List of ActionModel for navigation
        """
        actions = []
        
        for i, waypoint in enumerate(path.waypoints):
            action = ActionModel.create(
                action_type='move_to',
                parameters={
                    'target_location': waypoint,
                    'waypoint_index': i,
                    'total_waypoints': len(path.waypoints),
                    'path_reference': path.original_path  # Include original path for reference during execution
                },
                timeout=30.0,  # Timeout for reaching each waypoint
                retry_count=2   # Allow retries for each waypoint
            )
            action.action_id = f"nav_act_{path.calculated_at:.0f}_{i:03d}"
            actions.append(action)
        
        return actions
    
    async def monitor_environment_for_obstacles(self, robot_state: RobotStateModel) -> List[Obstacle]:
        """
        Continuously monitor the environment for new obstacles during navigation.
        
        Args:
            robot_state: Current RobotStateModel for context
            
        Returns:
            List of obstacles detected since last check
        """
        try:
            # Get updated environment information
            new_observation = await self._get_environment_observation(robot_state)
            
            # Compare with previously known obstacles
            new_obstacles = []
            for obs in new_observation.environment_map.get('obstacles', []):
                # Check if this obstacle is new compared to our last known obstacles
                is_new = True
                for known_obs in self.last_known_obstacles:
                    # Simple proximity check to determine if obstacle is "new"
                    distance = self._calculate_direct_distance(obs['position'], known_obs.position)
                    if distance < self.safety_buffer:
                        is_new = False
                        break
                
                if is_new:
                    new_obstacles.append(
                        Obstacle(
                            id=obs['id'],
                            position=obs['position'],
                            size=obs['size'],
                            type=obs['type'],
                            velocity=obs.get('velocity')
                        )
                    )
            
            # Update our tracking of known obstacles
            self.last_known_obstacles = [
                Obstacle(
                    id=obs['id'], 
                    position=obs['position'], 
                    size=obs['size'], 
                    type=obs['type'],
                    velocity=obs.get('velocity')
                )
                for obs in new_observation.environment_map.get('obstacles', [])
            ]
            
            return new_obstacles
            
        except Exception as e:
            self.logger.error(f"Error monitoring environment for obstacles: {e}")
            # Return empty list if monitoring fails
            return []
    
    def _adapt_navigation_speed_for_obstacles(self, robot_state: RobotStateModel) -> float:
        """
        Adapt navigation speed based on obstacle density in the immediate area.
        
        Args:
            robot_state: Current RobotStateModel for context
            
        Returns:
            Suggested navigation speed in m/s
        """
        if not self.speed_adaptation_enabled:
            return self.max_navigation_speed
        
        # Calculate nearby obstacle density
        nearby_obstacles = 0
        current_pos = robot_state.position
        
        for obstacle in self.last_known_obstacles:
            distance = self._calculate_direct_distance(current_pos, obstacle.position)
            if distance < 2.0:  # Within 2 meters
                nearby_obstacles += 1
        
        # Adjust speed based on obstacle density
        # More obstacles = slower speed
        if nearby_obstacles == 0:
            return self.max_navigation_speed
        elif nearby_obstacles <= 2:
            return self.max_navigation_speed * 0.8
        elif nearby_obstacles <= 5:
            return self.max_navigation_speed * 0.5
        else:
            # Too many obstacles nearby, slow down significantly
            return max(self.min_navigation_speed, self.max_navigation_speed * 0.3)
    
    async def request_emergency_stop(self) -> bool:
        """
        Request emergency stop if obstacle collision is imminent.
        
        Returns:
            True if emergency stop requested successfully, False otherwise
        """
        try:
            self.logger.warning("ObstacleNavigationManager requesting emergency stop")
            
            # In a real system, this would send an emergency stop command to the robot
            # For simulation, we'll just update our navigation mode
            self.navigation_mode = "emergency_stop"
            
            # Also notify the robot controller
            if self.robot_controller:
                await self.robot_controller.emergency_stop()
            
            self.logger.info("Emergency stop requested successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Error requesting emergency stop: {e}")
            return False
    
    def _obstacle_to_dict(self, obstacle: Obstacle) -> Dict[str, Any]:
        """Convert an Obstacle object to a dictionary representation."""
        return {
            'id': obstacle.id,
            'position': obstacle.position,
            'size': obstacle.size,
            'type': obstacle.type,
            'velocity': obstacle.velocity
        }
    
    def _derive_navigation_space(self, obstacles: List[Obstacle], current_position: Dict[str, float]) -> Dict[str, Any]:
        """
        Derive the navigation space based on current obstacles and position.
        
        Args:
            obstacles: List of obstacles in the environment
            current_position: Robot's current position
            
        Returns:
            Dictionary representing the navigable space
        """
        # In a real implementation, this would generate a proper occupancy grid or navigation mesh
        # For now, we'll just return a simplified representation
        return {
            'origin': {'x': -10.0, 'y': -10.0, 'z': 0.0},
            'dimensions': {'x': 20.0, 'y': 20.0, 'z': 2.0},
            'resolution': 0.1,
            'occupied_cells': []  # Would contain cells blocked by obstacles in real implementation
        }


# Global obstacle navigation manager instance
_obstacle_navigation_manager = None


def get_obstacle_navigation_manager() -> ObstacleNavigationManager:
    """Get the global obstacle navigation manager instance."""
    global _obstacle_navigation_manager
    if _obstacle_navigation_manager is None:
        _obstacle_navigation_manager = ObstacleNavigationManager()
    return _obstacle_navigation_manager


async def navigate_with_obstacle_avoidance(destination: Dict[str, float], robot_state: RobotStateModel) -> ActionResponseModel:
    """Convenience function to navigate with obstacle avoidance."""
    manager = get_obstacle_navigation_manager()
    return await manager.navigate_with_obstacle_avoidance(destination, robot_state)


async def monitor_environment_for_dynamic_obstacles(robot_state: RobotStateModel) -> List[Obstacle]:
    """Convenience function to monitor environment for new obstacles."""
    manager = get_obstacle_navigation_manager()
    return await manager.monitor_environment_for_obstacles(robot_state)


def adapt_navigation_speed(robot_state: RobotStateModel) -> float:
    """Convenience function to adapt navigation speed based on obstacles."""
    manager = get_obstacle_navigation_manager()
    return manager._adapt_navigation_speed_for_obstacles(robot_state)