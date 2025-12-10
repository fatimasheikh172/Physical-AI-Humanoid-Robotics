# Week 3: Bipedal Navigation & Model Training

## Learning Objectives

By the end of this week, you will be able to:
- Adapt Nav2 for bipedal humanoid movement with custom path planners
- Implement perception model training with synthetic datasets
- Integrate perception models with navigation system
- Evaluate sim-to-real transfer performance
- Complete a capstone project combining all components

## Overview

Week 3 focuses on combining perception and navigation capabilities for humanoid robots. You'll adapt the Nav2 framework for bipedal locomotion and train perception models using the synthetic datasets generated in Week 1. This week culminates in a complete AI-Robot Brain system that integrates perception, navigation, and control.

## 1. Bipedal Navigation with Nav2

### Understanding Bipedal Navigation Challenges

Humanoid robots present unique challenges for navigation compared to wheeled robots:
- Balance and stability requirements
- Kinematic constraints of legged locomotion
- Footstep planning for stable walking
- Different obstacle clearance requirements

### Adapting Nav2 for Bipedal Robots

To adapt Nav2 for bipedal locomotion, we need to customize several components:

```python
# bipedal_nav2.py
#!/usr/bin/env python3

"""
Bipedal Navigation with Nav2 for Humanoid Robots

This node adapts Nav2 for bipedal humanoid movement patterns
with custom path planners and local controllers that account
for humanoid kinematic constraints.
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import LaserScan, Image, Imu
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose
from builtin_interfaces.msg import Duration


class BipedalNavigatorNode(Node):
    def __init__(self):
        super().__init__('bipedal_navigator_node')
        
        # Declare parameters
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('foot_separation', 0.25)  # meters between feet
        self.declare_parameter('leg_length', 0.8)  # length of humanoid leg
        self.declare_parameter('max_step_length', 0.3)  # max step length for stability
        self.declare_parameter('min_step_length', 0.05)  # minimum step length
        self.declare_parameter('max_step_height', 0.1)  # max step height for obstacles
        self.declare_parameter('max_yaw_rate', 0.5)  # max turning rate in rad/s
        self.declare_parameter('step_time', 0.5)  # time per step in seconds
        self.declare_parameter('balance_margin', 0.05)  # extra margin for balance
        
        # Get parameters
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.foot_separation = self.get_parameter('foot_separation').value
        self.leg_length = self.get_parameter('leg_length').value
        self.max_step_length = self.get_parameter('max_step_length').value
        self.min_step_length = self.get_parameter('min_step_length').value
        self.max_step_height = self.get_parameter('max_step_height').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.step_time = self.get_parameter('step_time').value
        self.balance_margin = self.get_parameter('balance_margin').value
        
        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.footstep_publisher = self.create_publisher(MarkerArray, '/bipedal_footsteps', 10)
        self.balance_publisher = self.create_publisher(Marker, '/bipedal_balance_point', 10)
        
        # Create subscribers for sensor data
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        # Navigation goal subscriber
        self.nav_goal_subscription = self.create_subscription(
            PoseStamped,
            '/bipedal_navigation/goal',
            self.navigation_goal_callback,
            10
        )
        
        # Initialize navigation state
        self.is_navigating = False
        self.current_goal = None
        self.current_pose = None
        self.navigation_path = []
        self.current_step_index = 0
        
        # Initialize footstep planning
        self.left_foot_position = np.array([0.0, self.foot_separation/2, 0.0])
        self.right_foot_position = np.array([0.0, -self.foot_separation/2, 0.0])
        self.support_foot = 'left'  # Which foot is currently supporting weight
        
        # Timer for navigation control
        self.nav_timer = self.create_timer(0.1, self.navigation_control_loop)
        
        self.get_logger().info('Bipedal Navigator initialized')

    def laser_callback(self, msg):
        """Process laser scan data for navigation"""
        # Store laser data for path planning
        self.laser_data = msg

    def navigation_goal_callback(self, msg):
        """Handle navigation goals for bipedal robot"""
        self.get_logger().info(f'Received navigation goal: ({msg.pose.position.x}, {msg.pose.position.y})')
        
        # Plan bipedal-specific path
        path = self.plan_bipedal_path(self.get_current_pose(), msg.pose)
        
        if path:
            self.navigation_path = path
            self.current_step_index = 0
            self.is_navigating = True
            self.current_goal = msg.pose
            self.get_logger().info(f'Bipedal path planned with {len(path)} steps')
        else:
            self.get_logger().error('Failed to plan bipedal path to goal')

    def get_current_pose(self):
        """Get current robot pose from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_base_frame,
                rclpy.time.Time()
            )
            
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation.x = transform.transform.rotation.x
            pose.pose.orientation.y = transform.transform.rotation.y
            pose.pose.orientation.z = transform.transform.rotation.z
            pose.pose.orientation.w = transform.transform.rotation.w
            
            return pose
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform {self.robot_base_frame} to {self.map_frame}: {ex}')
            return None

    def plan_bipedal_path(self, start_pose, goal_pose):
        """Plan a path considering bipedal locomotion constraints"""
        if start_pose is None:
            return []
        
        # Extract positions
        start_pos = np.array([start_pose.pose.position.x, start_pose.pose.position.y])
        goal_pos = np.array([goal_pose.position.x, goal_pose.position.y])
        
        # Calculate path waypoints
        path = []
        direction = goal_pos - start_pos
        distance = np.linalg.norm(direction)
        
        if distance < 0.1:  # Already at goal
            return []
        
        # Create waypoints at step-sized intervals
        num_steps = int(np.ceil(distance / self.max_step_length))
        step_vector = direction / num_steps
        
        for i in range(num_steps + 1):
            waypoint = start_pos + i * step_vector
            path.append({'x': float(waypoint[0]), 'y': float(waypoint[1])})
        
        # Validate path for bipedal constraints using laser data
        validated_path = self.validate_bipedal_path(path)
        
        return validated_path

    def validate_bipedal_path(self, path):
        """Validate path for bipedal-specific constraints (obstacle clearance, etc.)"""
        if not hasattr(self, 'laser_data') or self.laser_data is None:
            return path  # Can't validate without sensor data
        
        validated_path = []
        
        # Check each segment of the path for obstacles
        for i in range(len(path) - 1):
            start_pt = np.array([path[i]['x'], path[i]['y']])
            end_pt = np.array([path[i + 1]['x'], path[i + 1]['y']])
            
            # Check for obstacles along this segment
            if self.is_path_clear(start_pt, end_pt):
                validated_path.append(path[i])
            else:
                # Try to find an alternative path around the obstacle
                detour_path = self.plan_detour(start_pt, end_pt)
                if detour_path:
                    validated_path.extend(detour_path)
                else:
                    self.get_logger().warn(f'Could not find detour around obstacle at segment {i}')
                    return []  # Could not find valid path
        
        # Add the final goal point
        if path:
            validated_path.append(path[-1])
        
        return validated_path

    def is_path_clear(self, start_pt, end_pt):
        """Check if path is clear of obstacles using laser data"""
        if not hasattr(self, 'laser_data'):
            return True  # Assume clear if no sensor data
        
        # Convert start and end points to robot frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                self.map_frame,
                rclpy.time.Time()
            )
        except TransformException:
            return True  # Can't transform, assume clear
        
        # For simplicity, just check if any laser distance is less than step size
        min_range = min([r for r in self.laser_data.ranges if not math.isnan(r)])
        return min_range > self.max_step_length  # Conservative check

    def plan_detour(self, start_pt, end_pt):
        """Plan a detour around obstacles for bipedal locomotion"""
        # Simplified detour planning - in a real implementation, 
        # this would use more sophisticated path planning
        direction = end_pt - start_pt
        perpendicular = np.array([-direction[1], direction[0]])  # Perpendicular vector
        perpendicular = perpendicular / np.linalg.norm(perpendicular)  # Normalize
        
        # Plan a path that goes around the obstacle
        detour_point1 = start_pt + perpendicular * 0.5  # 0.5m detour
        detour_point2 = end_pt + perpendicular * 0.5   # Same detour
        
        return [
            {'x': float(detour_point1[0]), 'y': float(detour_point1[1])},
            {'x': float(detour_point2[0]), 'y': float(detour_point2[1])}
        ]

    def navigation_control_loop(self):
        """Main navigation control loop for bipedal locomotion"""
        if not self.is_navigating or not self.navigation_path or self.current_step_index >= len(self.navigation_path):
            return
        
        # Get current position
        current_pose = self.get_current_pose()
        if current_pose is None:
            return
        
        # Get next waypoint in path
        target = self.navigation_path[self.current_step_index]
        target_pos = np.array([target['x'], target['y']])
        current_pos = np.array([current_pose.pose.position.x, current_pose.pose.position.y])
        
        # Calculate distance to next waypoint
        distance_to_waypoint = np.linalg.norm(target_pos - current_pos)
        
        if distance_to_waypoint < 0.1:  # Reached waypoint
            self.current_step_index += 1
            if self.current_step_index >= len(self.navigation_path):
                self.get_logger().info('Navigation completed')
                self.is_navigating = False
                return
        else:
            # Generate command to move toward waypoint
            direction = target_pos - current_pos
            distance = np.linalg.norm(direction)
            unit_direction = direction / distance if distance > 0 else np.array([0, 0])
            
            # Calculate desired velocity
            linear_speed = min(0.3, distance * 2.0)  # Approach smoothly
            angular_speed = self.calculate_angular_speed(unit_direction, current_pose.pose.orientation)
            
            # Publish velocity command
            cmd = Twist()
            cmd.linear.x = float(linear_speed)
            cmd.angular.z = float(angular_speed)
            
            self.cmd_vel_publisher.publish(cmd)
            
            # Log navigation status
            self.get_logger().debug(f'Navigating: {linear_speed:.2f} m/s, {angular_speed:.2f} rad/s')

    def calculate_angular_speed(self, direction_to_target, current_orientation):
        """Calculate angular speed to turn toward target direction"""
        # Convert quaternion to yaw angle
        q = current_orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Calculate target angle
        target_angle = math.atan2(direction_to_target[1], direction_to_target[0])
        
        # Calculate angle difference
        angle_diff = target_angle - current_yaw
        # Normalize to [-π, π]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Calculate angular speed proportional to error
        kp = 1.0  # Proportional gain
        angular_speed = max(-self.max_yaw_rate, min(self.max_yaw_rate, kp * angle_diff))
        
        return angular_speed

    def visualize_footsteps(self):
        """Publish footstep markers for visualization"""
        if not self.navigation_path:
            return
        
        marker_array = MarkerArray()
        
        # Create markers for planned footsteps
        for i, step in enumerate(self.navigation_path):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            
            marker.ns = "footsteps"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = step['x']
            marker.pose.position.y = step['y']
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0  # Blue for footsteps
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        self.footstep_publisher.publish(marker_array)

    def calculate_support_polygon(self, left_foot, right_foot):
        """Calculate the support polygon for bipedal balance"""
        # The support polygon for a biped is the convex hull of both feet
        # For simplicity, just return the center point between feet
        center = (left_foot + right_foot) / 2.0
        return center


def main(args=None):
    rclpy.init(args=args)
    
    navigator = BipedalNavigatorNode()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Bipedal navigator shutting down')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()