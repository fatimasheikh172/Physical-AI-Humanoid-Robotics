# Week 3: Capstone Project

## Overview

The capstone project brings together all the concepts learned throughout the digital twin module. You'll create a complete system that integrates Gazebo physics simulation, Unity visualization, and perception processing to solve a complex robotics task.

## Learning Objectives

By completing this capstone project, you will demonstrate your ability to:

- Integrate all components of the digital twin system
- Apply perception algorithms to solve navigation challenges
- Evaluate system performance using simulation fidelity measures
- Document and present your complete solution

## Capstone Project: Autonomous Navigation in Dynamic Environment

### Project Description

Design and implement an autonomous navigation system for a mobile robot that must navigate through a dynamic environment filled with moving obstacles to reach a specified goal location. The system must:

1. Sense the environment using simulated sensors (LiDAR, camera, IMU)
2. Process sensor data to identify static and dynamic obstacles
3. Plan a safe path to the goal considering moving obstacles
4. Execute navigation while adapting to changes in the environment
5. Visualize the robot's state and decision-making process in real-time

### Environment Setup

The simulation environment consists of:

- A room approximately 10m x 10m with walls and static obstacles
- 3-5 moving obstacles (simulated pedestrians or other robots)
- A goal location that changes periodically to test replanning
- Various sensor landmarks for localization

### System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Unity        │    │     ROS 2        │    │   Gazebo       │
│ Visualization  │◄──►│   Communication  │◄──►│   Physics      │
│                │    │     Bridge       │    │   Simulation   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                              │
                    ┌──────────────────┐
                    │  Perception      │
                    │    Pipeline      │
                    └──────────────────┘
                              │
                    ┌──────────────────┐
                    │  Navigation      │
                    │    Planner       │
                    └──────────────────┘
```

## Implementation Steps

### Step 1: Environment Configuration

Create the Gazebo environment with:

1. A 10m x 10m room with walls
2. Static obstacles (pillars, furniture)
3. Moving obstacles with predefined paths
4. Your robot model with all sensors (LiDAR, camera, IMU)

### Step 2: Perception System

Implement perception modules to:

1. Process LiDAR data to detect obstacles and build a partial map
2. Process camera data to identify specific landmarks and colored objects
3. Fuse sensor data to improve obstacle detection and tracking
4. Implement a simple object tracker to predict moving obstacle paths

```python
#!/usr/bin/env python3
"""
Capstone Perceptron System
Integrates all perception components for autonomous navigation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
from cv_bridge import CvBridge
from collections import deque
import math


class CapstonePerceptionNode(Node):
    def __init__(self):
        super().__init__('capstone_perception_node')
        
        # ROS2 subscriptions
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.camera_callback,
            10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.obstacle_publisher = self.create_publisher(PoseStamped, '/obstacles', 10)
        self.path_publisher = self.create_publisher(PoseStamped, '/path', 10)
        
        # Internal state
        self.robot_pose = None
        self.lidar_data = None
        self.camera_data = None
        self.cv_bridge = CvBridge()
        
        # Track moving obstacles
        self.moving_obstacles = deque(maxlen=100)  # Track last 100 positions
        
        # Timer for processing
        self.timer = self.create_timer(0.1, self.process_data)  # 10 Hz
        
    def lidar_callback(self, msg):
        """Process LiDAR data to detect obstacles"""
        self.lidar_data = msg
        
        # Process the scan data
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter out invalid ranges
        valid_indices = (ranges >= msg.range_min) & (ranges <= msg.range_max) & (ranges > 0)
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        # Convert to Cartesian coordinates relative to robot
        x_coords = valid_ranges * np.cos(valid_angles)
        y_coords = valid_ranges * np.sin(valid_angles)
        
        # Cluster points to identify obstacles
        obstacles = self.cluster_obstacles(x_coords, y_coords)
        
        # Update obstacle tracking
        self.update_moving_obstacles(obstacles)
    
    def camera_callback(self, msg):
        """Process camera data"""
        # Convert ROS Image to OpenCV
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Simple color-based detection (e.g., red traffic signs)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        red_lower = np.array([0, 50, 50])
        red_upper = np.array([10, 255, 255])
        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        
        # Find contours of red objects
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # For each red object, estimate distance and position
        for contour in contours:
            if cv2.contourArea(contour) > 1000:  # Filter small areas
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                
                # Convert to polar coordinates (approximate distance based on size)
                # This is simplified - real implementation would use depth or stereo
                distance = 1000 / (w * h)  # Approximation
                
                # Convert to world coordinates if robot pose is known
                if self.robot_pose:
                    # Transform from image coordinates to world coordinates
                    # This requires camera calibration and position
                    pass
    
    def odom_callback(self, msg):
        """Update robot pose"""
        self.robot_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'orientation': msg.pose.pose.orientation
        }
    
    def cluster_obstacles(self, x_coords, y_coords, max_distance=0.5):
        """Cluster LiDAR points to identify separate obstacles"""
        if len(x_coords) == 0:
            return []
        
        # Simple clustering algorithm
        clusters = []
        used = [False] * len(x_coords)
        
        for i in range(len(x_coords)):
            if not used[i]:
                cluster = [(x_coords[i], y_coords[i])]
                used[i] = True
                
                # Find nearby points
                for j in range(i + 1, len(x_coords)):
                    if not used[j]:
                        distance = math.sqrt(
                            (x_coords[i] - x_coords[j])**2 + 
                            (y_coords[i] - y_coords[j])**2
                        )
                        
                        if distance < max_distance:
                            cluster.append((x_coords[j], y_coords[j]))
                            used[j] = True
                
                # Calculate cluster center
                center_x = sum(p[0] for p in cluster) / len(cluster)
                center_y = sum(p[1] for p in cluster) / len(cluster)
                
                clusters.append((center_x, center_y))
        
        return clusters
    
    def update_moving_obstacles(self, current_obstacles):
        """Track moving obstacles and predict their motion"""
        # Add current obstacles to tracking
        for obs in current_obstacles:
            self.moving_obstacles.append(obs)
    
    def process_data(self):
        """Main processing loop"""
        if self.lidar_data is not None and self.robot_pose is not None:
            # Perform integrated perception tasks
            self.perception_fusion()
    
    def perception_fusion(self):
        """Fuse information from multiple sensors"""
        # This would combine LiDAR obstacle detection with camera landmark identification
        # and IMU motion data to create a comprehensive world model
        pass


def main(args=None):
    rclpy.init(args=args)
    
    perception_node = CapstonePerceptionNode()
    
    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        pass
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3: Navigation System

Create a navigation system that integrates perception data to plan and execute paths:

1. Implement a global path planner (e.g., A* or Dijkstra)
2. Create a local planner for obstacle avoidance
3. Implement trajectory tracking with feedback control
4. Add replanning capabilities when obstacles move

### Step 4: Integration and Testing

Integrate all components and test the complete system:

1. Launch the full simulation environment
2. Run your perception and navigation nodes
3. Test with different goal locations
4. Evaluate performance with moving obstacles
5. Record metrics and system behavior

## Assessment Criteria

### Technical Requirements (70%)

- **Sensor Integration (15%)**: Successfully integrate and process data from all sensors
- **Perception (20%)**: Accurately detect obstacles, landmarks, and dynamic elements
- **Navigation (20%)**: Successfully navigate to goals while avoiding obstacles
- **System Integration (15%)**: All components work together seamlessly

### Documentation and Analysis (20%)

- **Design Documentation (10%)**: Clear explanation of your architectural decisions
- **Performance Analysis (10%)**: Evaluation of system performance with metrics

### Presentation (10%)

- **Demonstration**: Clear demonstration of the working system
- **Explanation**: Ability to explain design choices and challenges

## Evaluation Metrics

Your system will be evaluated based on:

1. **Success Rate**: Percentage of successful goal-reaching attempts
2. **Efficiency**: Average time to reach goal
3. **Safety**: Number of collisions or near-misses
4. **Robustness**: Performance under different environmental conditions
5. **Adaptability**: How well the system handles moving obstacles

## Development Timeline

### Days 1-2: System Design and Setup
- Define your architecture
- Set up the simulation environment
- Create the basic perception pipeline

### Days 3-4: Core Implementation
- Implement perception algorithms
- Create navigation system
- Integrate with Gazebo and Unity

### Days 5-6: Testing and Refinement
- Test with different scenarios
- Optimize performance
- Document the system

### Day 7: Finalization and Presentation
- Prepare demonstration
- Write final report
- Present your solution

## Resources and References

- [ROS 2 Navigation2 Documentation](https://navigation.ros.org/)
- [Gazebo Simulation Documentation](http://gazebosim.org/tutorials)
- [Unity Robotics Documentation](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [Robotics Algorithms Textbook](https://github.com/AtsushiSakai/PythonRobotics)

## Going Further

After completing the capstone project, consider:

1. Adding more complex perception challenges (e.g., object recognition)
2. Implementing more sophisticated planning algorithms
3. Adding human-robot interaction elements
4. Creating a web interface for remote monitoring