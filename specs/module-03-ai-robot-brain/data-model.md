# Data Model: Module 03 - The AI-Robot Brain (NVIDIA Isaac™)

## Entities Overview

The AI-Robot Brain module involves several key entities that represent the various components of the AI perception and navigation pipeline:

1. **SimulationEnvironment**: Represents a virtual environment in Isaac Sim
2. **SyntheticDataset**: Represents a generated dataset with ground truth annotations
3. **PerceptionModel**: Represents an AI model for perception tasks
4. **VSLAMState**: Represents the visual SLAM state and map
5. **BipedalPath**: Represents navigation paths optimized for bipedal locomotion
6. **TrainingConfiguration**: Configuration for model training workflows

## Entity 1: SimulationEnvironment

### Fields
- `id` (string): Unique identifier for the simulation environment
- `name` (string): Human-readable name of the environment
- `description` (string): Brief description of the environment
- `scene_file` (string): Path to the USD file defining the scene
- `lighting_config` (object): Configuration for lighting
  - `time_of_day` (float): Time of day in hours (0-24) affecting lighting
  - `weather` (string): Weather conditions affecting lighting and visibility
- `physics_config` (object): Configuration for physics simulation
  - `gravity` (array of 3 floats): Gravity vector [x, y, z]
  - `solver_type` (string): Physics solver (PhysX)
  - `max_step_size` (float): Maximum physics step size
- `objects` (array of objects): Physical objects in the environment
  - `name` (string): Object identifier
  - `asset_path` (string): Path to the USD asset
  - `position` (array of 3 floats): Position [x, y, z] in meters
  - `rotation` (array of 4 floats): Rotation as quaternion [x, y, z, w]
  - `scale` (array of 3 floats): Scale factors [x, y, z]
- `sensors` (array of objects): Sensor configurations in the environment
  - `name` (string): Sensor identifier
  - `type` (string): Sensor type (RGB, depth, LiDAR, IMU, etc.)
  - `position` (array of 3 floats): Position [x, y, z] in meters
  - `rotation` (array of 4 floats): Rotation as quaternion [x, y, z, w]
  - `properties` (object): Sensor-specific properties like resolution, range, etc.

### Relationships
- One-to-many: SimulationEnvironment to SyntheticDataset (many datasets can be generated from one environment)
- One-to-many: SimulationEnvironment to RobotInstance (many robot instances can be simulated in the environment)

### Validation Rules
- Scene file must be a valid USD or USDA file
- Positions must be within environment bounds
- Physics parameters must be within realistic ranges
- Object scales must be positive values

## Entity 2: SyntheticDataset

### Fields
- `id` (string): Unique identifier for the dataset
- `name` (string): Human-readable name of the dataset
- `source_environment` (string): Reference to the SimulationEnvironment
- `generated_at` (string): ISO timestamp of generation
- `annotations_available` (array of strings): Types of annotations available (bounding_boxes, segmentation, depth, etc.)
- `image_count` (integer): Total number of images in the dataset
- `sensor_config` (object): Configuration of sensors used for generation
  - `camera_resolution` (array of 2 integers): Resolution [width, height] in pixels
  - `camera_fov` (float): Field of view in degrees
  - `depth_range` (array of 2 floats): Depth range [min, max] in meters
- `domain_randomization` (object): Configuration of domain randomization used
  - `lighting_variations` (boolean): Whether lighting was varied during generation
  - `texture_variations` (boolean): Whether textures were varied during generation
  - `object_positions` (boolean): Whether object positions were varied during generation
- `statistics` (object): Statistical properties of the dataset
  - `mean_brightness` (float): Average brightness of images
  - `contrast_range` (array of 2 floats): Range of contrast values
  - `object_counts` (object): Histogram of object instance counts per image

### Relationships
- Many-to-one: SyntheticDataset to SimulationEnvironment (dataset originates from environment)
- One-to-many: SyntheticDataset to PerceptionModel (dataset can be used to train multiple models)

### Validation Rules
- Image count must be positive
- Annotations must match available formats
- Generated timestamp must be in the past
- Statistics must be properly calculated

### State Transitions
- `Generating` → `Ready`: Dataset generation completes successfully
- `Generating` → `Failed`: Dataset generation encounters error
- `Ready` → `Archived`: Dataset moved to long-term storage

## Entity 3: PerceptionModel

### Fields
- `id` (string): Unique identifier for the model
- `name` (string): Human-readable name of the model
- `model_type` (string): Type of perception model (object_detection, segmentation, classification, etc.)
- `architecture` (string): Model architecture (YOLO, Mask-RCNN, etc.)
- `input_format` (object): Expected input format
  - `channels` (integer): Number of input channels (typically 3 for RGB)
  - `height` (integer): Input height in pixels
  - `width` (integer): Input width in pixels
- `output_format` (object): Model output format and specifications
  - `detection_classes` (array of strings): Classes the model can detect
  - `confidence_threshold` (float): Minimum confidence for detections
- `training_dataset` (string): Reference to SyntheticDataset used for training
- `tensorrt_config` (object): Configuration for TensorRT optimization
  - `precision_mode` (string): FP32, FP16, or INT8
  - `batch_size` (integer): Optimized batch size
  - `dynamic_shapes` (object): Min/opt/max shapes for optimization
- `performance_metrics` (object): Performance metrics
  - `inference_time_ms` (float): Average inference time in milliseconds
  - `accuracy` (float): Model accuracy on validation set
  - `memory_usage_mb` (float): GPU memory usage in MB

### Relationships
- Many-to-one: PerceptionModel to SyntheticDataset (model trained on dataset)
- One-to-many: PerceptionModel to VSLAMState (model used in various SLAM states)

### Validation Rules
- Architecture must be supported by TensorRT
- Input format must match dataset specifications
- Performance metrics must be non-negative
- Model artifacts must exist at specified paths

## Entity 4: VSLAMState

### Fields
- `timestamp` (float): Simulation time in seconds
- `robot_id` (string): Reference to the robot instance
- `camera_pose` (object): Pose of the camera in world coordinates
  - `position` (array of 3 floats): Position [x, y, z] in meters
  - `orientation` (array of 4 floats): Orientation as quaternion [x, y, z, w]
- `tracked_features` (array of objects): Currently tracked visual features
  - `id` (integer): Feature identifier
  - `location_3d` (array of 3 floats): 3D location [x, y, z]
  - `pixel_location` (array of 2 floats): 2D pixel location [u, v]
  - `confidence` (float): Confidence of feature tracking
- `map_points` (array of objects): 3D points in the reconstructed map
  - `id` (integer): Map point identifier
  - `position` (array of 3 floats): 3D position [x, y, z]
  - `descriptor` (array of floats): Feature descriptor vector
  - `observations` (integer): Number of observations of this point
- `tracking_status` (string): Tracking status (Good, Degraded, Lost)
- `optimization_needed` (boolean): Whether map optimization should be performed

### Relationships
- Many-to-one: VSLAMState to RobotInstance (state belongs to robot)
- One-to-many: VSLAMState to PerceptionModel (perception model integrated with SLAM)

### Validation Rules
- Timestamp must be non-negative and monotonic
- Camera pose components must be finite numbers
- Quaternions must be normalized (unit length)
- Map point IDs must be unique

### State Transitions
- `Initializing` → `Tracking`: Successful initialization with sufficient features
- `Tracking` → `Lost`: Tracking fails due to lack of features or motion blur
- `Lost` → `Recovering`: Attempting to recover tracking
- `Recovering` → `Tracking`: Tracking recovered successfully

## Entity 5: BipedalPath

### Fields
- `id` (string): Unique identifier for the path
- `source` (string): Reference to the navigation request source
- `destination` (array of 3 floats): Destination coordinates [x, y, z] in meters
- `waypoints` (array of objects): Waypoints in the planned path
  - `position` (array of 3 floats): Position [x, y, z] in meters
  - `heading` (float): Yaw heading in radians
  - `footstep_constraints` (object): Bipedal-specific constraints
    - `left_foot_position` (array of 3 floats): Position for left foot placement [x, y, z]
    - `right_foot_position` (array of 3 floats): Position for right foot placement [x, y, z]
    - `stance_width` (float): Distance between feet for stability
- `planning_time_ms` (float): Time taken to plan the path in milliseconds
- `path_length_m` (float): Total length of path in meters
- `footstep_plan` (array of objects): Detailed footstep plan for bipedal locomotion
  - `step_type` (string): Left, Right, or Standing
  - `position` (array of 3 floats): Planned foot position [x, y, z]
  - `orientation` (float): Yaw orientation in radians
  - `step_time` (float): Expected time when this footstep should be executed
- `obstacles_avoided` (integer): Count of obstacles avoided during planning
- `kinematic_feasibility` (float): Score representing how kinematically feasible the path is (0.0-1.0)

### Relationships
- Many-to-one: BipedalPath to RobotInstance (path is for specific robot)
- One-to-one: BipedalPath to NavigationRequest (path is response to request)

### Validation Rules
- Destination coordinates must be within map bounds
- Waypoints must form a connected path
- Footstep positions must ensure bipedal stability
- Kinematic feasibility scores must be between 0.0 and 1.0

## Entity 6: TrainingConfiguration

### Fields
- `id` (string): Unique identifier for the configuration
- `model_type` (string): Type of model to train (object_detection, segmentation, etc.)
- `dataset_id` (string): Reference to the dataset for training
- `hyperparameters` (object): Training hyperparameters
  - `learning_rate` (float): Learning rate for optimization
  - `batch_size` (integer): Batch size for training
  - `epochs` (integer): Number of epochs to train
  - `optimizer` (string): Optimizer type (Adam, SGD, etc.)
  - `weight_decay` (float): L2 regularization strength
- `augmentation_config` (object): Data augmentation configuration
  - `brightness_range` (array of 2 floats): Range of brightness adjustments
  - `rotation_range_deg` (float): Maximum rotation in degrees
  - `noise_sigma` (float): Sigma value for noise augmentation
- `validation_split` (float): Fraction of data for validation (0.0-1.0)
- `loss_functions` (array of strings): Loss functions to use
- `metrics` (array of strings): Evaluation metrics to monitor
- `early_stopping_patience` (integer): Epochs to wait before stopping if no improvement
- `results` (object): Training results (populated after training)
  - `final_loss` (float): Final training loss
  - `validation_loss` (float): Validation loss at the end
  - `training_accuracy` (float): Training accuracy
  - `validation_accuracy` (float): Validation accuracy
  - `best_epoch` (integer): Epoch with best validation metric

### Relationships
- One-to-many: TrainingConfiguration to PerceptionModel (configuration can produce multiple model variants)
- Many-to-one: TrainingConfiguration to SyntheticDataset (configuration uses dataset)

### Validation Rules
- Hyperparameter values must be within reasonable ranges
- Dataset reference must point to a valid SyntheticDataset
- Validation split must be between 0.0 and 1.0
- Batch size must be positive

## Relationships Summary

```
SimulationEnvironment (1) ─── (M) SyntheticDataset (M) ─── (1) PerceptionModel
        │                           │                              │
        └───────────────────────────┼──────────────────────────────┘
                                    │
RobotInstance (1) ─── (M) VSLAMState (1) ─── (M) PerceptionModel
        │                           │
        │                           └── (1) BipedalPath (M) ─── (1) NavigationRequest
        │
        └── (1) TrainingConfiguration (M) ─── (M) SyntheticDataset
```

## State Transition Diagrams

### VSLAMState Lifecycle
```
[Initialization] → [Tracking] → [Degraded] → [Lost]
       ↓             ↑   ↓        ↑   ↓       ↑
   (valid features)   (motion)  (recovery)  (failure)
       ↓             ↓   ↓        ↓   ↓       ↓
   [Recovery] ←───────────────────────────────┘
```

### BipedalPath Lifecycle
```
[Path Request] → [Planning] → [Verified] → [Executing] → [Completed]
                      ↓           ↓           ↓            ↓
                [Replanning] ← [Adjusting] ← [Active] ← [Monitoring]
```

## Data Validation and Quality Assurance

### Simulation Quality
- Photorealistic rendering validation against real-world images
- Physics accuracy verification with ground truth data
- Sensor model validation against real sensor specifications

### Dataset Quality
- Annotation accuracy verification through sampling and validation
- Distribution assessment to ensure adequate coverage of scenarios
- Domain randomization effectiveness validation

### Model Quality
- Benchmark accuracy validation against standard datasets
- Robustness testing under various conditions
- TensorRT optimization preservation of accuracy validation

### Navigation Quality
- Path optimality assessment against baseline algorithms
- Kinematic feasibility verification for planned trajectories
- Obstacle avoidance effectiveness validation