# Introduction to URDF for Humanoid Robot Design

## What is URDF?

URDF stands for Unified Robot Description Format. It's an XML-based format used to describe robot models in ROS. URDF files specify all the physical properties of a robot including:

- **Links**: Rigid components of the robot (like arms, legs, torso)
- **Joints**: Connections between links (like hinges, rotational joints)
- **Visual elements**: How the robot appears in simulation and visualization
- **Collision elements**: How the robot interacts physically in simulation
- **Inertial properties**: Mass, center of mass, and inertia for physics simulation

## URDF Structure

A basic URDF file follows this structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <!-- Visual elements -->
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
    <!-- Collision elements -->
    <collision>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
    <!-- Inertial properties -->
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>
  
  <!-- Joints define connections between links -->
  <joint name="joint_name" type="revolute">
    <parent link="base_link"/>
    <child link="another_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
  </joint>
  
  <!-- Additional links and joints -->
  <link name="another_link">
    <!-- ... -->
  </link>
</robot>
```

## Links in URDF

Links represent rigid bodies in the robot. Each link can have:

1. **Visual properties**: How the link appears in simulation/GUI
2. **Collision properties**: How the link interacts in physical simulation
3. **Inertial properties**: Mass and inertial tensor for physics simulation

### Visual Elements
Visual elements define how the robot appears to users:

- **Geometry**: Shape of the visual element (box, cylinder, sphere, mesh)
- **Material**: Color and appearance properties
- **Origin**: Position and orientation offset from the link's origin

### Collision Elements
Collision elements define how the robot interacts in physical simulation:

- **Geometry**: Shape for collision detection (similar to visual)
- **Origin**: Offset from the link's origin
- Generally simpler geometry than visual to improve performance

### Inertial Properties
Inertial properties define the physical behavior during simulation:

- **Mass**: Mass of the link in kg
- **Inertia tensor**: Describes how mass is distributed (ixx, ixy, ixz, iyy, iyz, izz)

## Joints in URDF

Joints define how links connect and move relative to each other. The main joint types are:

### 1. Revolute Joint
- Rotates around an axis
- Has upper and lower angle limits
- Like a wheel rotating on an axle

### 2. Continuous Joint
- Like revolute but unlimited rotation
- Suitable for wheel joints where full rotation is needed

### 3. Prismatic Joint
- Linear sliding motion along an axis
- Like a drawer sliding in and out

### 4. Fixed Joint
- No movement between links
- Used to rigidly connect two links

### 5. Planar Joint
- Movement in a plane (rarely used)

### 6. Floating Joint
- 6-DOF motion (rarely used)

## Example: Simple Humanoid Leg

Here's a basic example of a humanoid leg with hip, knee, and ankle joints:

```xml
<?xml version="1.0"?>
<robot name="humanoid_leg">
  <!-- Hip Link -->
  <link name="hip">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.0417" ixy="0" ixz="0" iyy="0.0417" iyz="0" izz="0.0417"/>
    </inertial>
  </link>

  <!-- Hip Joint -->
  <joint name="hip_joint" type="revolute">
    <parent link="hip"/>
    <child link="thigh"/>
    <origin xyz="0 0 -0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Thigh Link -->
  <link name="thigh">
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
      <mass value="3.0"/>
      <inertia ixx="0.3125" ixy="0" ixz="0" iyy="0.3125" iyz="0" izz="0.015"/>
    </inertial>
  </link>

  <!-- Knee Joint -->
  <joint name="knee_joint" type="revolute">
    <parent link="thigh"/>
    <child link="shin"/>
    <origin xyz="0 0 -0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="3.14" effort="100" velocity="1"/>
  </joint>

  <!-- Shin Link -->
  <link name="shin">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.2604" ixy="0" ixz="0" iyy="0.2604" iyz="0" izz="0.0125"/>
    </inertial>
  </link>
</robot>
```

## Humanoid Robot Skeleton Design

### Design Principles
When designing a humanoid robot skeleton:

1. **Anthropomorphic proportions**: Base link sizes and joint positions on human proportions
2. **Degrees of Freedom**: Sufficient DOF for desired motions
3. **Center of Gravity**: Consider COG position for balance and stability
4. **Actuator placement**: Ensure joints are drivable by planned actuators

### Typical Humanoid Joint Configuration

#### Head (2-3 DOF)
- Neck joint for head orientation (pitch/yaw)

#### Torso (3-6 DOF)
- Waist for bending and twisting (pitch/yaw/roll)
- Upper body movement

#### Arms (6-7 DOF per arm)
- Shoulder (3 DOF: pitch, roll, yaw)
- Elbow (1 DOF: pitch)
- Wrist (2-3 DOF: pitch, yaw, sometimes roll)
- Grippers/hands (varies greatly)

#### Legs (6 DOF per leg)
- Hip (3 DOF: pitch, roll, yaw)
- Knee (1 DOF: pitch)
- Ankle (2 DOF: pitch, roll)

## Sensors in URDF

URDF also supports defining sensor placement in the robot:

```xml
<!-- Example of sensor definition -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.1" rpy="0 0 0"/>
</joint>
```

## Best Practices for URDF Design

### 1. Organization
- Use consistent naming conventions
- Group related links and joints logically
- Use Xacro macros for repetitive structures (like fingers or toes)

### 2. Validation
- Check for proper tree structure (no loops)
- Validate joint limits and effort values
- Ensure all links are connected properly

### 3. Physics Simulation
- Use realistic mass and inertia values
- Simplify collision geometry for performance
- Consider using separate high-detail visual models

### 4. Documentation
- Comment complex sections
- Document the kinematic chain clearly
- Include design notes and assumptions

## Tools for URDF

Several tools assist with URDF development:

1. **RViz**: For visualizing the robot model
2. **Gazebo**: For physics simulation
3. **URDF Checkers**: Tools to validate URDF files
4. **Mesh Processing Tools**: To optimize 3D models for visual and collision

## Common Pitfalls

1. **Incorrect joint limits**: Can make simulation unrealistic
2. **Missing inertial properties**: Can cause simulation instability
3. **Tree violations**: URDF must be a single tree structure
4. **Improper origins**: Can cause visual/physical mismatches
5. **Unit mismatches**: All units should be SI (meters, kilograms, seconds)

## Next Steps

After understanding basic URDF concepts, explore:
- Xacro for parameterized and macro-based descriptions
- Transmission definitions for actuator integration
- Gazebo-specific extensions for simulation
- ROS Control integration for hardware interfaces

This introduction covers the fundamentals of URDF for humanoid robotics. With this foundation, you can build more complex humanoid robot models with proper kinematic chains, physical properties, and sensor placements.