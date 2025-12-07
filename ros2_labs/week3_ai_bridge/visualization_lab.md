# RViz & Gazebo Visualization Lab

## Objective
This lab covers how to visualize robot models in both RViz (for visualization) and Gazebo (for simulation) using URDF models and sensor data.

## Prerequisites
- Understanding of URDF robot modeling
- Basic ROS 2 node concepts
- Completed previous week's content on URDF

## Lab Overview

### Part 1: Loading URDF Models in RViz (30 minutes)
### Part 2: Basic RViz Displays (45 minutes)
### Part 3: Gazebo Simulation Setup (60 minutes)
### Part 4: Sensor Visualization (45 minutes)
### Part 5: Joint Simulation and Control (30 minutes)

---

## Part 1: Loading URDF Models in RViz

### Learning Goals
- Load a URDF robot model into RViz
- Configure the robot model to display properly
- Understand the relationship between TF frames and robot visualization

### Exercises

1. **Create a Simple URDF Model**
   - Create a file `simple_robot.urdf` in your `week3_ai_bridge/urdf` directory
   - Define a robot with a base link and an arm with joints

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Robot base -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.000125"/>
    </inertial>
  </link>

  <!-- Joint connecting base and arm -->
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0.0 0.0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

2. **Launch RViz with Robot Model**
   - Create a launch file `view_robot.launch.py`:
   ```python
   from launch import LaunchDescription
   from launch.substitutions import Command
   from launch_ros.actions import Node
   from launch_ros.substitutions import FindPackageShare
   
   def generate_launch_description():
       # Get URDF via xacro
       robot_description_content = Command([
           'xacro ',
            FindPackageShare('week3_ai_bridge'),
            '/urdf/simple_robot.urdf'
       ])
       robot_description = {'robot_description': robot_description_content}
       
       # Define nodes
       robot_state_publisher = Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           output='both',
           parameters=[robot_description],
       )
       
       joint_state_publisher = Node(
           package='joint_state_publisher',
           executable='joint_state_publisher',
           parameters=[{'source_list': ['joint_states']}]
       )
       
       rviz_node = Node(
           package='rviz2',
           executable='rviz2',
           arguments=['-d', FindPackageShare('week3_ai_bridge'), '/rviz/config.rviz'],
           output='screen'
       )
       
       return LaunchDescription([
           robot_state_publisher,
           joint_state_publisher,
           rviz_node
       ])
   ```

3. **Configure RViz Display**
   - Run the launch file: `ros2 launch week3_ai_bridge view_robot.launch.py`
   - In RViz, add the RobotModel display
   - Set the robot description to `/robot_description`
   - Make sure the Fixed Frame is set to `base_link`
   - Observe your robot model

### Questions to Answer:
1. What happens when you change the joint values in the Joint State Publisher GUI?
2. How is the robot model related to TF transforms?
3. What would happen if you changed the Fixed Frame to `arm_link`?

---

## Part 2: Basic RViz Displays

### Learning Goals
- Configure different RViz displays
- Visualize sensor data streams
- Set up coordinate frame visualization

### Exercises

1. **Create an RViz Configuration File**
   - Create directory: `week3_ai_bridge/rviz/`
   - Create file: `config.rviz` with the following configuration:
   ```yaml
   Panels:
     - Class: rviz_common/Displays
     - Class: rviz_common/Views
     - Class: rviz_common/Time
   Visualization Manager:
     Displays:
       - Class: rviz_default_plugins/RobotModel
         Name: RobotModel
         Enabled: true
         Robot Description: robot_description
       - Class: rviz_default_plugins/TF
         Name: TF
         Enabled: true
         Show Arrows: true
         Show Names: true
         Frame Timeout: 15
       - Class: rviz_default_plugins/Grid
         Name: Grid
         Enabled: true
         Plane: XY
         Cell Size: 1
         Color: 160; 160; 164
       - Class: rviz_default_plugins/LaserScan
         Name: LaserScan
         Enabled: false
         Topic: /scan
       - Class: rviz_default_plugins/PointCloud2
         Name: PointCloud2
         Enabled: false
         Topic: /pointcloud
     Views:
       Current:
         Class: rviz_default_plugins/Orbit
   ```

2. **Add TF Visualization**
   - In your RViz configuration, make sure TF (Transform) display is enabled
   - Adjust the length of transform arrows to make them more visible
   - Change the color scheme for different frames

3. **Display Coordinate Axes**
   - Add a dedicated "Axes" display
   - Set different sizes for different coordinate frames
   - Use this to understand the orientation of different robot links

### Testing Exercises:
1. Add a grid display and align your robot on the grid
2. Make the TF arrows longer and color-coded per frame
3. Add a laser scan display (even though it's disabled for now)

---

## Part 3: Gazebo Simulation Setup

### Learning Goals
- Set up basic Gazebo simulation
- Configure physics properties for robot model
- Understand the difference between visualization (RViz) and simulation (Gazebo)

### Exercises

1. **Extend URDF with Gazebo-specific tags**
   - Modify `simple_robot.urdf` to add Gazebo-specific elements:
   ```xml
   <?xml version="1.0"?>
   <robot name="simple_robot_gazebo">
     <!-- Include the existing links and joints here -->
     
     <!-- Gazebo plugin for ros_control -->
     <gazebo>
       <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
         <robotNamespace>/simple_robot</robotNamespace>
       </plugin>
     </gazebo>
     
     <!-- Gazebo material definition -->
     <gazebo reference="base_link">
       <material>Gazebo/Blue</material>
       <mu1>0.2</mu1>
       <mu2>0.2</mu2>
     </gazebo>
     
     <gazebo reference="arm_link">
       <material>Gazebo/Red</material>
       <mu1>0.2</mu1>
       <mu2>0.2</mu2>
     </gazebo>
   </robot>
   ```

2. **Create World File for Gazebo**
   - Create `week3_ai_bridge/worlds/simple_world.world`:
   ```xml
   <?xml version="1.0"?>
   <sdf version="1.6">
     <world name="simple_world">
       <physics type="ode">
         <gravity>0 0 -9.8</gravity>
         <max_step_size>0.001</max_step_size>
         <real_time_factor>1.0</real_time_factor>
       </physics>
       
       <include>
         <uri>model://ground_plane</uri>
       </include>
       
       <include>
         <uri>model://sun</uri>
       </include>
     </world>
   </sdf>
   ```

3. **Create Gazebo Launch File**
   - Create `gazebo_simulation.launch.py`:
   ```python
   from launch import LaunchDescription
   from launch.substitutions import Command, PathJoinSubstitution
   from launch_ros.actions import Node
   from launch_ros.substitutions import FindPackageShare
   from launch.actions import ExecuteProcess
   
   def generate_launch_description():
       # Get URDF via xacro
       robot_description_content = Command([
           'xacro ',
           FindPackageShare('week3_ai_bridge'),
           '/urdf/simple_robot_gazebo.urdf'
       ])
       robot_description = {'robot_description': robot_description_content}

       # Robot state publisher
       robot_state_publisher = Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           output='both',
           parameters=[robot_description],
       )

       # Spawn robot in Gazebo
       spawn_entity = Node(
           package='gazebo_ros',
           executable='spawn_entity.py',
           arguments=[
               '-topic', 'robot_description',
               '-entity', 'simple_robot'
           ],
           output='screen'
       )

       # Launch Gazebo
       gazebo = ExecuteProcess(
           cmd=['gazebo', '--verbose', '-u',
                PathJoinSubstitution([
                    FindPackageShare('week3_ai_bridge'),
                    'worlds',
                    'simple_world.world'
                ])],
           output='screen'
       )

       return LaunchDescription([
           robot_state_publisher,
           gazebo,
           spawn_entity
       ])
   ```

4. **Run Gazebo Simulation**
   - Execute: `ros2 launch week3_ai_bridge gazebo_simulation.launch.py`
   - Observe your robot in the Gazebo physics simulation
   - Note the differences from RViz visualization

### Questions:
1. How does the robot behave differently in Gazebo vs. RViz?
2. What happens when you apply forces to the robot in Gazebo?
3. How does the physics simulation affect joint movement?

---

## Part 4: Sensor Visualization

### Learning Goals
- Add sensors to your URDF model
- Visualize sensor data in both RViz and Gazebo
- Understand how sensor data flows through the system

### Exercises

1. **Add a Camera to URDF**
   - Extend your URDF with a camera sensor:
   ```xml
   <!-- Camera link mounted on the robot -->
   <link name="camera_link">
     <visual>
       <geometry>
         <box size="0.02 0.05 0.02"/>
       </geometry>
       <material name="black">
         <color rgba="0 0 0 1"/>
       </material>
     </visual>
     <collision>
       <geometry>
         <box size="0.02 0.05 0.02"/>
       </geometry>
     </collision>
     <inertial>
       <mass value="0.01"/>
       <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
     </inertial>
   </link>

   <joint name="camera_joint" type="fixed">
     <parent link="base_link"/>
     <child link="camera_link"/>
     <origin xyz="0.1 0.0 0.1" rpy="0 0 0"/>
   </joint>

   <!-- Gazebo camera definition -->
   <gazebo reference="camera_link">
     <sensor type="camera" name="camera1">
       <update_rate>30.0</update_rate>
       <camera name="head">
         <horizontal_fov>1.3962634</horizontal_fov>
         <image>
           <width>800</width>
           <height>600</height>
           <format>R8G8B8</format>
         </image>
         <clip>
           <near>0.02</near>
           <far>300</far>
         </clip>
       </camera>
       <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
         <frame_name>camera_link</frame_name>
         <topic_name>camera/image_raw</topic_name>
       </plugin>
     </sensor>
   </gazebo>
   ```

2. **Add a LiDAR to URDF**
   - Add a 3D lidar to your URDF:
   ```xml
   <!-- LiDAR link -->
   <link name="lidar_link">
     <visual>
       <geometry>
         <cylinder length="0.05" radius="0.03"/>
       </geometry>
       <material name="gray">
         <color rgba="0.5 0.5 0.5 1"/>
       </material>
     </visual>
     <collision>
       <geometry>
         <cylinder length="0.05" radius="0.03"/>
       </geometry>
     </collision>
     <inertial>
       <mass value="0.1"/>
       <inertia ixx="0.0000125" ixy="0" ixz="0" iyy="0.0000125" iyz="0" izz="0.000025"/>
     </inertial>
   </link>

   <joint name="lidar_joint" type="fixed">
     <parent link="base_link"/>
     <child link="lidar_link"/>
     <origin xyz="0.0 0.0 0.25" rpy="0 0 0"/>
   </joint>

   <!-- Gazebo LiDAR definition -->
   <gazebo reference="lidar_link">
     <sensor type="ray" name="lidar">
       <always_on>true</always_on>
       <update_rate>10</update_rate>
       <ray>
         <scan>
           <horizontal>
             <samples>360</samples>
             <resolution>1.0</resolution>
             <min_angle>-3.14159</min_angle>
             <max_angle>3.14159</max_angle>
           </horizontal>
         </scan>
         <range>
           <min>0.1</min>
           <max>30.0</max>
           <resolution>0.01</resolution>
         </range>
       </ray>
       <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
         <ros_topic>/scan</ros_topic>
         <frame_name>lidar_link</frame_name>
       </plugin>
     </sensor>
   </gazebo>
   ```

3. **Create Sensor Visualization Launch**
   - Create `sensor_viz.launch.py`:
   ```python
   from launch import LaunchDescription
   from launch.substitutions import Command
   from launch_ros.actions import Node
   from launch_ros.substitutions import FindPackageShare
   from launch.actions import ExecuteProcess
   
   def generate_launch_description():
       # Get URDF via xacro
       robot_description_content = Command([
           'xacro ',
           FindPackageShare('week3_ai_bridge'),
           '/urdf/simple_robot_gazebo.urdf'
       ])
       robot_description = {'robot_description': robot_description_content}

       # Robot state publisher
       robot_state_publisher = Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           output='both',
           parameters=[robot_description],
       )

       # Joint state broadcaster
       joint_state_broadcaster = Node(
           package='controller_manager',
           executable='spawner.py',
           arguments=['joint_state_broadcaster'],
       )

       # RViz node for sensor visualization
       rviz_node = Node(
           package='rviz2',
           executable='rviz2',
           arguments=['-d', FindPackageShare('week3_ai_bridge'), '/rviz/sensor_config.rviz'],
           output='screen'
       )

       return LaunchDescription([
           robot_state_publisher,
           joint_state_broadcaster,
           rviz_node
       ])
   ```

4. **Configure RViz for Sensor Data**
   - Create a new RViz config `sensor_config.rviz` that includes displays for camera and lidar data
   - Add Image display for camera feed
   - Add LaserScan display for lidar data
   - Add PointCloud display for 3D sensor data

### Testing the Sensor Visualization:
1. Launch simulation with sensors: `ros2 launch week3_ai_bridge gazebo_simulation.launch.py`
2. In another terminal, launch visualization: `ros2 launch week3_ai_bridge sensor_viz.launch.py`
3. Observe sensor data in RViz as the robot moves in Gazebo

---

## Part 5: Joint Simulation and Control

### Learning Goals
- Control robot joints in simulation
- Understand joint state feedback
- Visualize joint movements in both environments

### Exercises

1. **Create Joint Control Node**
   - Create `joint_control_demo.py`:
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import Float64MultiArray
   import math
   import time
   
   class JointControllerDemo(Node):
       def __init__(self):
           super().__init__('joint_controller_demo')
           
           # Publisher for joint commands
           self.joint_publisher = self.create_publisher(
               Float64MultiArray,
               '/simple_robot/position_commands',
               10
           )
           
           # Timer for sending commands
           self.timer = self.create_timer(0.1, self.timer_callback)
           self.time_counter = 0.0
           
       def timer_callback(self):
           # Create joint command message
           msg = Float64MultiArray()
           
           # Oscillate the arm joint
           arm_position = 0.5 * math.sin(self.time_counter)
           msg.data = [arm_position]  # For arm_joint
           
           self.joint_publisher.publish(msg)
           self.get_logger().info(f'Published joint command: {msg.data}')
           
           self.time_counter += 0.1
   
   def main(args=None):
       rclpy.init(args=args)
       
       controller = JointControllerDemo()
       
       try:
           rclpy.spin(controller)
       except KeyboardInterrupt:
           pass
       finally:
           controller.destroy_node()
           rclpy.shutdown()
   
   if __name__ == '__main__':
       main()
   ```

2. **Run Joint Control Demo**
   - Start the Gazebo simulation
   - Run the joint controller: `ros2 run week3_ai_bridge joint_control_demo.py`
   - Observe the robot's arm moving in both Gazebo and RViz

### Final Testing Exercise:
1. Combine all components: Run Gazebo with your robot model
2. Visualize in RViz with all displays active
3. Use the joint controller to move parts of your robot
4. Observe sensor data as the robot moves

---

## Lab Report Questions

Answer the following questions in your lab report:

1. What are the key differences between RViz visualization and Gazebo simulation?
2. How does the URDF model relate to TF transforms in ROS 2?
3. What are the main considerations when designing URDF models for simulation vs. visualization?
4. How can you verify that sensor data is flowing correctly from Gazebo to RViz?
5. What challenges did you encounter when adding sensors to your robot model?

## Troubleshooting Tips

- If models don't appear in RViz: Check that the robot_description parameter is set correctly
- If joints don't move: Verify TF tree with `ros2 run tf2_tools view_frames`
- If sensors don't publish: Check that Gazebo plugins are correctly loaded
- If simulation is unstable: Check inertial properties in your URDF

## Extensions (Optional)

For additional learning:
- Add more complex sensors (IMU, force-torque sensors)
- Create a more humanoid robot model
- Implement a simple controller to follow trajectories
- Experiment with different physics parameters