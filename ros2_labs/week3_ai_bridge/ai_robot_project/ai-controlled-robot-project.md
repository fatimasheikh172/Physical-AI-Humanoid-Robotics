# Week 3 Mini Project: AI-Controlled Virtual Robot

## Objective
Design and implement an AI-controlled virtual robot that demonstrates integration of AI decision-making with ROS 2 robot control systems and visualization in RViz/Gazebo. The robot should take high-level commands from an AI agent, translate them into robot actions, and operate in a simulated environment with sensors.

## Project Overview
You will create a complete system that includes an AI decision-maker, robot control interfaces, a URDF model of a humanoid robot, and a simulated environment where the AI-controlled robot can operate based on sensor feedback and user commands.

## System Requirements

### Core Components
Your system must include:

#### 1. AI Decision Maker Node
- Receives high-level commands or objectives
- Processes sensor data from the robot
- Makes decisions about robot actions
- Sends commands to the robot controller

#### 2. Robot Controller Node
- Receives commands from the AI
- Interfaces with the robot's joints and actuators
- Publishes sensor feedback to the AI

#### 3. URDF Robot Model
- Humanoid-like structure with appropriate joints
- At least 12 degrees of freedom
- Sensors: at least a camera and lidar
- Proper physical properties for simulation

#### 4. Simulation Environment
- Gazebo simulation with obstacles
- RViz visualization showing robot state
- Proper lighting and environmental settings

## Detailed Specifications

### AI Decision Maker Architecture
- **Input**: High-level commands (natural language or structured goals), sensor data from robot
- **Processing**: Decision making based on sensor data and objectives
- **Output**: Low-level robot commands (velocities, joint positions, etc.)
- **Implementation**: Use OpenAI API or similar to process high-level commands and return executable actions

### Communication Interfaces
- **AI Command Topic**: `/ai_commands` (String messages with commands like "go to red box", "avoid obstacle", etc.)
- **Robot States Topic**: `/robot_states` (sensor data and current robot state)
- **Command Output Topic**: `/cmd_vel` for differential drive or `/joint_commands` for articulated robot
- **Sensor Data Topics**: `/camera/image_raw`, `/scan`, `/imu` (based on your URDF)

### Robot Model Requirements
- **Minimal 12 DOF**: Distributed across body in a humanoid structure
- **Links**: At least 10 links (head, torso, arms, legs, etc.)
- **Sensors**: At least 2 types (camera and lidar recommended)
- **Physical Properties**: Proper masses and inertias for stable simulation

### Simulation Environment
- **Obstacles**: Static and/or dynamic obstacles to navigate around
- **Goals**: Specific locations or objects for the AI-controlled robot to reach/interact with
- **Physics**: Realistic physics parameters for stable simulation
- **Visualization**: Clear visualization of robot state and AI decisions

## Implementation Tasks

### 1. Design the URDF Robot Model (25 points)
Create a humanoid robot model with appropriate joints and sensors:
- Complete URDF file with proper kinematic chain
- Realistic dimensions and physical properties
- Integration of at least 2 sensor types
- Proper material definitions for visualization

#### Key Requirements:
- At least 12 controllable joints
- Appropriate sensor placement (camera for vision, lidar for navigation)
- Realistic physical properties (masses, inertias)
- Proper joint limits and safety constraints

### 2. Implement the AI Decision Maker (35 points)
Create a node that can interpret high-level commands and generate appropriate robot actions:

#### Key Features Required:
- Connect to an AI service (OpenAI API, local model, or simulated AI)
- Process natural language commands or structured goals
- Use sensor data to inform decisions
- Generate appropriate robot commands based on goals and sensor input

#### Example AI Flow:
1. User sends command: "Move to the blue object"
2. AI processes command and identifies relevant object in camera feed
3. AI calculates navigation path avoiding obstacles from lidar data
4. AI sends velocity commands to move robot toward target

### 3. Create Robot Controller (20 points)
Develop the robot control system that executes AI-generated commands:
- Interface with robot joints in simulation
- Handle low-level control and safety constraints
- Provide feedback to the AI system

### 4. Gazebo & RViz Integration (15 points)
Integrate the robot model into simulation with proper visualization:
- Robot spawns correctly in Gazebo environment
- Proper visualization in RViz with robot model
- Correct TF tree visualization
- Sensor data displayed properly

### 5. System Integration & Testing (25 points)
- All components work together seamlessly
- AI can successfully control the robot in simulation
- Robot responds appropriately to various commands
- Sensor feedback is properly used in decision-making
- Error handling for AI failures or communication issues

### 6. Documentation and Commentary (10 points)
- Explain your robot design and rationale
- Describe the AI decision-making process
- Identify potential improvements or extensions
- Document how all components work together

## Project Structure

Your implementation should be organized as follows:

```
week3_ai_bridge/
├── src/
│   ├── ai_decision_maker.py
│   ├── robot_controller.py
│   └── sensor_processor.py
├── urdf/
│   └── ai_controlled_robot.urdf
├── worlds/
│   └── test_environment.world
├── launch/
│   ├── ai_robot_system.launch.py
│   └── simulation.launch.py
├── rviz/
│   └── robot_viz.rviz
├── meshes/ (if using custom 3D models)
└── config/
    └── parameters.yaml
```

## Implementation Approach

### Phase 1: Robot Model (Day 1)
- Design and implement the URDF robot model
- Test the model in RViz for visualization
- Verify the kinematic chain

### Phase 2: Simulation Setup (Day 1)
- Set up Gazebo environment
- Integrate robot model into simulation
- Test basic movement capabilities

### Phase 3: AI Decision System (Day 2)
- Implement AI command interpretation
- Connect to AI service
- Test AI decision-making with simple commands

### Phase 4: Integration (Day 3)
- Integrate all components
- Implement sensor feedback loop
- Test complete system

### Phase 5: Testing & Documentation (Day 3)
- Conduct comprehensive testing
- Document the system
- Prepare project report

## Testing Scenarios

Your system should handle:
1. **Navigation**: AI directs robot to move to specific locations
2. **Obstacle Avoidance**: Robot uses sensor data to avoid obstacles autonomously
3. **Object Interaction**: Robot identifies and moves toward specific objects
4. **Error Recovery**: Robot handles AI command failures gracefully
5. **Sensor Integration**: Robot uses both camera and lidar data in decision-making

## Submission Requirements

1. **Complete source code** for all required components
2. **URDF model** of your humanoid robot with proper sensors
3. **Launch files** to start the complete system
4. **RViz configuration** for visualization
5. **Gazebo world file** for testing environment
6. **Documentation file** explaining your design choices and AI integration
7. **Demo video** showing the system in operation (3-5 minutes)

### Required Documentation
- URDF design rationale with joint placement and sensors
- AI decision-making process and implementation details
- System architecture diagram showing component interactions
- Testing results and performance evaluation
- Known limitations and future improvements

## Grading Rubric

- **Robot Model (20%)**: Proper URDF with appropriate joints and sensors
- **AI Integration (30%)**: Effective AI decision-making and command interpretation
- **Control System (20%)**: Proper robot control and safety considerations
- **Simulation Integration (15%)**: Working simulation with proper visualization
- **System Performance (10%)**: Successful robot operation based on AI commands
- **Documentation (5%)**: Clear explanation of design and implementation

## Extra Credit Opportunities

For additional points (up to 10% bonus):
- Implement more sophisticated AI behaviors (path planning, object recognition, etc.)
- Add reinforcement learning for robot behavior adaptation
- Create a more complex simulation environment
- Develop a custom AI service for specialized robot tasks
- Add haptic feedback or advanced sensors to the robot model

## Resources and References

- ROS 2 Navigation Stack documentation
- URDF tutorials for robot modeling
- Gazebo simulation best practices
- OpenAI API documentation for AI integration
- RViz visualization tutorials

## Due Date
Submit your completed project by the end of Week 3. Include all source files, documentation, configuration files, and a README with setup and execution instructions.

Your project will be evaluated based on functionality, implementation quality, and the effectiveness of AI controlling the robot in simulation. Be prepared to demonstrate your system during the assessment period.