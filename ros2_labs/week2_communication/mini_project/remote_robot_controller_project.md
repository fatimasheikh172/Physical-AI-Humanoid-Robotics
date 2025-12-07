# Week 2 Mini Project: ROS 2 Remote Robot Controller

## Objective
Design and implement a complete ROS 2-based remote robot controller that effectively uses all three communication patterns (topics, services, and actions) with appropriate Quality of Service (QoS) configurations.

## Project Overview
Create a system that consists of:
- A remote controller interface
- A robot simulator that executes commands
- Proper communication setup using topics, services, and actions

## System Requirements

### Core Components
Your system must include:

#### 1. Controller Node
- Provides user interface for sending commands to the robot
- Uses topics for continuous commands (movement)
- Uses services for immediate actions (stop, start, mode change)
- Uses actions for long-running tasks (navigation, manipulation)

#### 2. Robot Simulator Node
- Receives commands from the controller
- Executes simulated robot behaviors
- Reports status back to controller
- Implements all three communication patterns

#### 3. Communication Setup
- Topic for movement commands (geometry_msgs/Twist)
- Service for immediate robot state changes
- Action for navigation tasks

## Detailed Specifications

### Topic-based Movement Control
- Topic name: `/cmd_vel`
- Message type: `geometry_msgs/Twist`
- Purpose: Send continuous velocity commands to control robot movement
- QoS settings: Reliability - RELIABLE, Depth - 10
- The topic should handle continuous command streaming with proper frequency

### Service for Robot State Control
- Service name: `/robot_control`
- Request: Command type (start, stop, pause, resume, mode change)
- Response: Success/failure status with details
- Purpose: Immediate, critical state changes that require acknowledgment
- QoS settings: Default for services

### Action for Navigation Tasks
- Action name: `/navigate_to_pose`
- Goal: Target pose (position and orientation)
- Feedback: Progress toward goal, distance remaining
- Result: Success/failure with final position
- Purpose: Long-running navigation tasks that provide ongoing feedback
- QoS settings: Default for actions

## Implementation Tasks

### 1. Create Controller Interface (40 points)
Develop a controller that can:
- Send continuous velocity commands via topic
- Send immediate state changes via service
- Send navigation goals via action
- Monitor robot status and feedback
- Handle graceful shutdown and error conditions

#### Controller Features Required:
- Continuous movement control (forward, backward, rotation)
- Immediate stop capability
- Mode switching (manual/auto/programmed)
- Navigation goal setting
- Real-time status display

### 2. Create Robot Simulator (35 points)
Develop a robot simulator that:
- Receives and processes velocity commands
- Responds to service requests
- Executes navigation actions
- Provides feedback during long operations
- Tracks and reports simulation state

#### Robot Simulator Features:
- Velocity command processing
- Immediate state change execution
- Navigation simulation with feedback
- State reporting
- Error handling

### 3. Quality of Service Configuration (15 points)
Configure appropriate QoS settings for each communication pattern:
- Movement control: Reliability and history settings appropriate for continuous commands
- Service calls: Appropriate for immediate responses
- Action feedback: Settings that support ongoing communication

### 4. System Integration and Testing (25 points)
- Integrate all components
- Test communication patterns work together
- Validate error handling
- Verify QoS settings function as expected
- Demonstrate all functionality together

### 5. Documentation and Commentary (10 points)
- Comment your code appropriately
- Document your QoS choices and rationale
- Explain how the three communication patterns work together in your system
- Identify potential improvements or optimizations

## Code Structure Requirements

Your implementation should be organized as follows:

```
week2_communication/
├── src/
│   ├── controller_node.py
│   ├── robot_simulator.py
│   ├── vel_publisher.py
│   ├── state_service.py
│   └── navigation_action.py
├── srv/
│   └── RobotState.srv
├── action/
│   └── NavigateToPose.action
└── launch/
    └── remote_robot_system.launch.py
```

## QoS Considerations

For each communication type, consider:
- **Reliability**: When do you need reliable delivery (control commands) vs. best-effort (status updates)?
- **Durability**: Do new subscribers need historical data (like maps) or is current data sufficient?
- **History**: How many messages should be retained for late joiners or for handling bursts?
- **Deadline**: Are there timing constraints on message delivery?

## Testing Strategy

Your system should handle:
- Normal operation with all three communication patterns
- Network interruptions or slow communication
- Invalid commands or inputs
- Graceful handling of component failures
- Proper cleanup on shutdown

## Submission Requirements

1. **Complete source code** for all required components
2. **Service and action definition files** (srv/ and action/ directories)
3. **Launch file** to start the complete system
4. **Documentation file** with your QoS analysis and rationale
5. **Demonstration script** showing all functionality working together

### Required Documentation
- Explain your QoS choices with specific rationale
- Describe how the three communication patterns work together
- Identify potential limitations of your implementation
- Suggest possible improvements or extensions

## Grading Rubric

- **Functionality (50%)**: All components work correctly individually and together
- **Correctness (20%)**: Proper use of topics, services, and actions
- **QoS Configuration (15%)**: Appropriate settings with clear rationale
- **Documentation (10%)**: Clear explanation of design choices
- **Code Quality (5%)**: Clean, well-commented, and maintainable code

## Additional Challenges (Bonus)

For extra credit, implement:
- A GUI interface for the controller (instead of command-line)
- Multiple simultaneous navigation goals with priority handling
- Advanced error recovery mechanisms
- Performance monitoring and reporting

## Due Date
Submit your completed project by the end of Week 2. Include all source files, documentation, and a README with setup and execution instructions.