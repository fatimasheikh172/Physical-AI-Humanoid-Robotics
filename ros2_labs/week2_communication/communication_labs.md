# Lab Activities: ROS 2 Communication Patterns

## Objective
Implement and experiment with all three major ROS 2 communication patterns: Topics, Services, and Actions, with specific focus on robotics applications.

## Prerequisites
- Completed Week 1 foundation work
- ROS 2 workspace set up
- Basic understanding of ROS 2 nodes and packages

## Lab Structure
This lab is divided into three main activities corresponding to the three communication patterns.

---

## Activity 1: Topic-Based Robot Motion Control

### Task Description
Implement a simple robot motion control system using the publish/subscribe pattern.

### Implementation Steps

1. **Create a publisher node** that simulates sending motion commands to a robot:
   - Topic name: `/robot_motion_commands`
   - Message type: `geometry_msgs/Twist` (linear and angular velocities)
   - Publish at 10 Hz
   - For simulation, publish simple motion patterns (e.g., forward, turn)

2. **Create a subscriber node** that listens to motion commands and "executes" them:
   - Subscribe to `/robot_motion_commands`
   - Print the received command to the console
   - Simulate robot movement in a virtual environment (print simulated position)

3. **Implement a QoS configuration** with different settings:
   - Default QoS for immediate communication
   - Reliability and durability settings for different scenarios

### Code Template for Publisher (`motion_publisher.py`):
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class MotionPublisher(Node):
    def __init__(self):
        super().__init__('motion_publisher')
        self.publisher_ = self.create_publisher(Twist, '/robot_motion_commands', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        # Simple motion pattern: forward motion with slight turning
        msg.linear.x = 0.5  # Move forward at 0.5 m/s
        msg.angular.z = 0.1 * math.sin(self.i * 0.1)  # Gentle turning
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: linear={msg.linear.x}, angular={msg.angular.z}')
        self.i += 1
```

### Code Template for Subscriber (`motion_subscriber.py`):
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotionSubscriber(Node):
    def __init__(self):
        super().__init__('motion_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/robot_motion_commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation = 0.0

    def listener_callback(self, msg):
        self.get_logger().info(f'Received command: linear={msg.linear.x}, angular={msg.angular.z}')
        
        # Simulate robot movement based on command
        # This is a simplified model
        dt = 0.1  # Time step (matches publisher rate)
        self.position_x += msg.linear.x * dt * math.cos(self.orientation)
        self.position_y += msg.linear.x * dt * math.sin(self.orientation)
        self.orientation += msg.angular.z * dt
        
        self.get_logger().info(f'Simulated position: x={self.position_x:.2f}, y={self.position_y:.2f}, theta={self.orientation:.2f}')
```

### Testing
- Run the publisher: `ros2 run week2_communication motion_publisher`
- In another terminal, run the subscriber: `ros2 run week2_communication motion_subscriber`
- Observe the communication between nodes

---

## Activity 2: Service-Based Robot State Change

### Task Description
Implement a service server that allows clients to change robot states (e.g., enable/disable motion, change operational mode).

### Implementation Steps

1. **Create a service definition** file (`RobotState.srv`) in the `srv` directory:
   ```
   # Request
   string state_request  # "enable", "disable", "idle", etc.
   ---
   # Response
   bool success
   string message
   ```

2. **Create a service server** that handles state change requests:
   - Service name: `/change_robot_state`
   - Process state change requests
   - Return success/failure status

3. **Create a service client** that sends state change requests:
   - Send different state requests
   - Print responses

### Code Template for Service Server (`state_server.py`):
```python
import rclpy
from rclpy.node import Node
from your_package.srv import RobotState  # Replace with your package name

class StateServer(Node):
    def __init__(self):
        super().__init__('state_server')
        self.srv = self.create_service(RobotState, '/change_robot_state', self.change_state_callback)
        self.current_state = "idle"
        self.get_logger().info(f'Robot state server initialized. Current state: {self.current_state}')

    def change_state_callback(self, request, response):
        self.get_logger().info(f'Received request to change state to: {request.state_request}')
        
        # Validate and change the state
        if request.state_request in ["enable", "disable", "idle", "manual"]:
            self.current_state = request.state_request
            response.success = True
            response.message = f'State changed to {self.current_state}'
        else:
            response.success = False
            response.message = f'Invalid state requested: {request.state_request}'
        
        self.get_logger().info(f'Response: {response.message}')
        return response
```

### Code Template for Service Client (`state_client.py`):
```python
import sys
import rclpy
from rclpy.node import Node
from your_package.srv import RobotState  # Replace with your package name

class StateClient(Node):
    def __init__(self):
        super().__init__('state_client')
        self.cli = self.create_client(RobotState, '/change_robot_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = RobotState.Request()

    def send_request(self, state_request):
        self.req.state_request = state_request
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    
    client = StateClient()
    
    if len(sys.argv) > 1:
        state_request = sys.argv[1]
    else:
        state_request = "enable"  # default
        
    response = client.send_request(state_request)
    client.get_logger().info(f'Result: {response.success}, {response.message}')
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Testing
- Run the server: `ros2 run week2_communication state_server`
- In another terminal, run the client: `ros2 run week2_communication state_client enable`
- Try different state requests to see how the service handles them

---

## Activity 3: Action-Based Navigation Command

### Task Description
Implement an action server that handles navigation commands with feedback and the ability to cancel goals.

### Implementation Steps

1. **Create an action definition** file (`NavigateToPose.action`) in the `action` directory:
   ```
   # Goal
   geometry_msgs/PoseStamped target_pose
   ---
   # Result
   bool reached_goal
   geometry_msgs/PoseStamped final_pose
   ---
   # Feedback
   float32 distance_remaining
   geometry_msgs/PoseStamped current_pose
   ```

2. **Create an action server** that simulates navigation to a target:
   - Accept navigation goals
   - Provide feedback on progress
   - Handle goal cancellation
   - Return final result

3. **Create an action client** that sends navigation goals and monitors progress:
   - Send a target pose
   - Monitor feedback
   - Optionally cancel the goal

### Code Template for Action Server (`nav_action_server.py`):
```python
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from your_package.action import NavigateToPose  # Replace with your package name
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class NavActionServer(Node):
    def __init__(self):
        super().__init__('nav_action_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received navigation goal')
        
        # Simulate navigation
        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()
        
        # For this simple simulation, we'll assume a straight-line path
        # with 10 intermediate steps
        for i in range(1, 11):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.reached_goal = False
                result.final_pose = self.get_current_pose()
                self.get_logger().info('Goal canceled')
                return result
                
            # Update feedback
            feedback_msg.distance_remaining = 10.0 - (i * 1.0)
            feedback_msg.current_pose = self.get_current_pose()
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info(f'Feedback: {feedback_msg.distance_remaining}m remaining')
            
            # Simulate movement time
            time.sleep(1)
        
        # If we've reached this point, navigation is complete
        goal_handle.succeed()
        result.reached_goal = True
        result.final_pose = self.get_current_pose()
        
        self.get_logger().info('Navigation completed successfully')
        return result
    
    def get_current_pose(self):
        # Simple simulation of current robot pose
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        return pose

def main(args=None):
    rclpy.init(args=args)
    nav_action_server = NavActionServer()
    rclpy.spin(nav_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Template for Action Client (`nav_action_client.py`):
```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from your_package.action import NavigateToPose  # Replace with your package name
from geometry_msgs.msg import PoseStamped

class NavActionClient(Node):
    def __init__(self):
        super().__init__('nav_action_client')
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose')

    def send_goal(self, target_pose):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.target_pose = target_pose

        self.get_logger().info('Sending navigation goal...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.distance_remaining}m remaining')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.reached_goal}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    action_client = NavActionClient()
    
    # Create a simple target pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'map'
    
    # Send the goal
    action_client.send_goal(target_pose)
    
    # Spin to allow callbacks to be processed
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

### Testing
- Run the action server: `ros2 run week2_communication nav_action_server`
- In another terminal, run the action client: `ros2 run week2_communication nav_action_client`
- Observe the feedback messages during navigation
- Try cancelling the goal during execution

---

## QoS Tuning Lab

### Task Description
Experiment with different Quality of Service settings to understand how they affect communication.

### Exercises
1. Set up a publisher with different reliability settings (reliable vs. best effort)
2. Test with a subscriber using matching and mismatched QoS
3. Observe how message delivery changes based on QoS settings

### Example Code for QoS Testing:
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Create a QoS profile with specific settings
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)
```

## Expected Outcomes

After completing these labs, you should be able to:
1. Implement all three ROS 2 communication patterns
2. Choose the appropriate communication pattern for different use cases
3. Configure Quality of Service settings for your application
4. Understand the trade-offs between different communication patterns
5. Debug communication issues in ROS 2

## Troubleshooting Tips

- Check that the correct message types are imported
- Verify topic/service/action names match between publisher and subscriber
- Ensure the ROS 2 environment is sourced (`source /opt/ros/humble/setup.bash`)
- Use `ros2 topic list`, `ros2 service list`, and `ros2 action list` to verify available endpoints