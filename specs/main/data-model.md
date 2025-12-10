# Data Model: Module 02 - Digital Twin for Robotic Systems

## Entities Overview

The digital twin system involves several key entities that represent different aspects of the robot simulation and visualization pipeline:

1. **RobotModel**: Represents the physical characteristics of a robot
2. **PhysicsState**: Represents the dynamic state of a robot in the physics simulation
3. **SensorData**: Represents data from various sensors attached to the robot
4. **VisualizationState**: Represents the state of the robot in the Unity visualization
5. **SimulationParameters**: Configuration parameters for physics simulation

## Entity 1: RobotModel

### Fields
- `robot_id` (string): Unique identifier for the robot
- `model_name` (string): Name of the URDF/SDF model file
- `links` (array of objects): Physical links of the robot with properties:
  - `name` (string): Link name
  - `mass` (float): Mass in kg
  - `inertia` (object): Inertia tensor with `ixx`, `ixy`, `ixz`, `iyy`, `iyz`, `izz`
  - `geometry` (object): Collision and visual geometry
- `joints` (array of objects): Joint definitions with properties:
  - `name` (string): Joint name
  - `type` (string): Joint type (revolute, prismatic, fixed, etc.)
  - `parent` (string): Parent link name
  - `child` (string): Child link name
  - `axis` (object): Joint axis with `x`, `y`, `z`
  - `limits` (object): Joint limits with `lower`, `upper`, `effort`, `velocity`
- `materials` (array of objects): Visual materials with properties:
  - `name` (string): Material name
  - `color` (object): RGB color with `r`, `g`, `b`, `a` values

### Relationships
- One-to-many: RobotModel to PhysicsState (robot has many states over time)
- One-to-many: RobotModel to SensorData (robot has many sensor readings)

### Validation Rules
- RobotModel must have at least one link
- RobotModel must have valid URDF/SDF file reference
- Joint parent and child links must exist in the same robot
- Mass values must be positive
- Inertia values must form a positive definite matrix

## Entity 2: PhysicsState

### Fields
- `timestamp` (float): Simulation time in seconds
- `robot_id` (string): Reference to the RobotModel
- `poses` (array of objects): Pose of each link with properties:
  - `link_name` (string): Name of the link
  - `position` (object): Position with `x`, `y`, `z` coordinates
  - `orientation` (object): Orientation with `x`, `y`, `z`, `w` quaternion
- `velocities` (array of objects): Velocity of each link with properties:
  - `link_name` (string): Name of the link
  - `linear` (object): Linear velocity with `x`, `y`, `z` components
  - `angular` (object): Angular velocity with `x`, `y`, `z` components
- `joint_states` (array of objects): Joint positions, velocities, efforts:
  - `name` (string): Joint name
  - `position` (float): Position in radians or meters
  - `velocity` (float): Velocity in rad/s or m/s
  - `effort` (float): Applied effort in N or Nm

### Relationships
- Many-to-one: PhysicsState to RobotModel (many states for one robot)
- One-to-many: PhysicsState to SensorData (state generates sensor data)

### Validation Rules
- Timestamp must be non-negative and monotonically increasing
- Position and orientation values must be finite numbers
- Quaternion must be normalized (unit length)
- Joint state names must match those defined in RobotModel

### State Transitions
- PhysicsState updates continuously based on physics simulation and applied forces
- Updates are driven by the Gazebo physics engine at simulation time steps

## Entity 3: SensorData

### Fields
- `sensor_id` (string): Unique identifier for the sensor
- `robot_id` (string): Reference to the RobotModel
- `sensor_type` (string): Type of sensor (lidar, depth_camera, imu, etc.)
- `timestamp` (float): Timestamp of sensor reading
- `frame_id` (string): Reference frame for the sensor data
- `data_payload` (object): Sensor-specific data:
  - For lidar: `ranges` (array of floats), `intensities` (array of floats), `angle_min`, `angle_max`, `angle_increment`, `time_increment`, `scan_time`, `range_min`, `range_max`
  - For depth_camera: `image` (base64 encoded), `depth` (base64 encoded), `camera_info` (object with intrinsic params)
  - For imu: `orientation` (quaternion), `orientation_covariance` (array of 9 floats), `angular_velocity` (vector), `angular_velocity_covariance` (array of 9 floats), `linear_acceleration` (vector), `linear_acceleration_covariance` (array of 9 floats)
- `noise_parameters` (object): Parameters characterizing sensor noise:
  - `mean` (float): Noise mean
  - `std_dev` (float): Noise standard deviation
  - `dropout_rate` (float): Probability of data dropout

### Relationships
- Many-to-one: SensorData to PhysicsState (sensor data originates from a state)
- Many-to-one: SensorData to RobotModel (sensor belongs to a robot)

### Validation Rules
- Timestamp must be non-negative
- Sensor type must be one of the supported types
- Data payload must match the sensor type specification
- Frame ID must correspond to a valid transform in the robot's frame tree

## Entity 4: VisualizationState

### Fields
- `timestamp` (float): Time of the visualization update
- `robot_id` (string): Reference to the RobotModel
- `unity_scene_id` (string): ID of the Unity scene
- `robot_transforms` (array of objects): Transforms for each robot component:
  - `component_name` (string): Name of the component (link/joint)
  - `position` (object): Position in Unity coordinates (x, y, z)
  - `rotation` (object): Rotation in Unity coordinates (x, y, z, w quaternion)
  - `scale` (object): Scale factors (x, y, z)
- `sensor_visualizations` (array of objects): Visual representations of sensor data:
  - `sensor_id` (string): ID of the sensor being visualized
  - `visualization_data` (object): Visualization-specific data (point cloud vertices, depth image, etc.)
- `material_properties` (array of objects): Runtime material changes:
  - `component_name` (string): Component to apply material to
  - `material_params` (object): Material parameters (color, shininess, etc.)

### Relationships
- Many-to-one: VisualizationState to RobotModel (visualization of a robot)
- One-to-many: VisualizationState to UnityScene elements

### Validation Rules
- Position and rotation values must be finite numbers
- Scale factors must be positive numbers
- Component names must match those in the RobotModel

## Entity 5: SimulationParameters

### Fields
- `param_id` (string): Unique identifier for this parameter set
- `robot_id` (string): Reference to the RobotModel these parameters apply to
- `physics_engine` (string): Physics engine used (ode, bullet, simbody)
- `gravity` (object): Gravity vector with `x`, `y`, `z` components
- `solver_type` (string): Type of physics solver
- `max_step_size` (float): Maximum ODE step size
- `real_time_factor` (float): Real time update rate
- `max_contacts` (int): Maximum number of contacts
- `friction_parameters` (array of objects): Friction settings for links:
  - `link_name` (string): Link to apply friction to
  - `mu1`, `mu2` (float): Friction coefficients
  - `fdir1` (object): Friction direction vector
- `damping_parameters` (array of objects): Damping settings:
  - `link_name` (string): Link to apply damping to
  - `linear_damping` (float): Linear damping coefficient
  - `angular_damping` (float): Angular damping coefficient
- `contact_parameters` (array of objects): Contact model parameters:
  - `collision1`, `collision2` (string): Colliding geometries
  - `kp` (float): Spring stiffness
  - `kd` (float): Damping coefficient
  - `max_vel` (float): Maximum contact correction velocity
  - `min_depth` (float): Minimum contact depth

### Relationships
- One-to-one: SimulationParameters to RobotModel (parameters for one robot)

### Validation Rules
- Step size must be positive
- Real time factor must be positive
- Friction coefficients must be non-negative
- Damping coefficients must be non-negative

## Relationships Summary

```
RobotModel (1) ─── (M) PhysicsState (1) ─── (M) SensorData
     │                    │                   │
     │                    └───────────────────┘
     │
     └── (1) SimulationParameters
     │
     └── (M) VisualizationState
```

## State Transition Diagrams

### PhysicsState Lifecycle
```
[Initial State] → [Physics Simulation Running] → [Paused/Stopped]
       ↓              ↓ (updates continuously)        ↓
[Parameter Set] → [State Updates] → [State Frozen]
```

### SensorData Lifecycle
```
[Sensor Initialized] → [Collecting Data] → [Processing Data]
       ↓              ↓ (continuous)         ↓
[Configuration] → [Raw Data Stream] → [Processed Output]
```

## Data Validation and Quality Assurance

### Physics Validation
- Conservation of energy checks
- Joint limit enforcement
- Collision detection accuracy
- Integration accuracy for kinematic equations

### Sensor Validation
- Range/field of view validation
- Noise model adherence
- Temporal consistency checks
- Cross-sensor validation where applicable

### Visualization Validation
- Coordinate system consistency between Gazebo and Unity
- Timing synchronization validation
- Material and appearance fidelity
- Performance metrics (frame rate, resource usage)