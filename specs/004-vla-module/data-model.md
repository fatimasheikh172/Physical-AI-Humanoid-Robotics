# Data Model: Module 4 - Vision-Language-Action (VLA)

## Entities Overview

The Vision-Language-Action (VLA) system involves several key entities that represent different aspects of the voice processing, cognitive planning, and action execution pipeline:

1. **VoiceCommand**: Represents a voice command captured from the user
2. **CognitivePlan**: Represents an AI-generated plan based on a voice command
3. **ActionSequence**: Represents a sequence of executable actions derived from a cognitive plan
4. **RobotState**: Represents the current state of the robot for planning and execution
5. **VisionObservation**: Represents object detection and visual understanding data
6. **ActionResponse**: Represents the result of executing an action sequence

## Entity 1: VoiceCommand

### Fields
- `command_id` (string): Unique identifier for the command
- `audio_data` (binary): Raw audio data or URI to audio file
- `transcript` (string): Transcribed text from the voice command
- `timestamp` (float): Time when the command was received
- `confidence` (float): Confidence score of the transcription (0.0-1.0)
- `user_id` (string): Identifier for the user who issued the command
- `language` (string): Language code of the voice command (e.g., 'en', 'es')
- `session_id` (string): Session identifier to group related commands

### Relationships
- One-to-many: VoiceCommand to CognitivePlan (one command results in one plan)
- One-to-many: VoiceCommand to ActionSequence (one command may result in multiple action sequences)

### Validation Rules
- VoiceCommand must have valid audio data or transcript
- Confidence score must be between 0.0 and 1.0
- Timestamp must be non-negative
- Language must be a supported language code

## Entity 2: CognitivePlan

### Fields
- `plan_id` (string): Unique identifier for the plan
- `command_id` (string): Reference to the original voice command
- `llm_model` (string): Name of the LLM used to generate the plan
- `llm_response` (string): Raw response from the LLM
- `task_decomposition` (array of objects): Breakdown of the high-level command into subtasks
  - `task_id` (string): Unique identifier for the subtask
  - `task_description` (string): Human-readable description of the task
  - `task_type` (string): Type of task (navigation, manipulation, perception, etc.)
  - `priority` (int): Priority level for task execution
  - `parameters` (object): Task-specific parameters
- `execution_context` (object): Context information for plan execution
  - `environment_map` (string): URI to environment map
  - `robot_capabilities` (array of strings): List of robot capabilities
  - `safety_constraints` (object): Safety constraints for execution
- `timestamp` (float): Time when the plan was generated
- `confidence` (float): Confidence score of the plan (0.0-1.0)

### Relationships
- Many-to-one: CognitivePlan to VoiceCommand (many plans for one command - alternative plans)
- One-to-many: CognitivePlan to ActionSequence (one plan results in action sequence)
- Many-to-many: CognitivePlan to RobotState (plan may reference multiple robot states)

### Validation Rules
- CognitivePlan must reference a valid VoiceCommand
- All subtasks in task_decomposition must be valid action types
- Execution context must contain all required information for plan execution
- Confidence score must be between 0.0 and 1.0

### State Transitions
- [Generated] → [Validated] → [Approved] → [In Execution] → [Completed] / [Failed]

## Entity 3: ActionSequence

### Fields
- `sequence_id` (string): Unique identifier for the action sequence
- `plan_id` (string): Reference to the cognitive plan
- `actions` (array of objects): Ordered list of actions to execute
  - `action_id` (string): Unique identifier for the action
  - `action_type` (string): Type of action (move_to, pick_up, place, detect_object, etc.)
  - `parameters` (object): Action-specific parameters
    - `target_location` (object, optional): Coordinates for navigation
      - `x` (float): X coordinate
      - `y` (float): Y coordinate
      - `z` (float): Z coordinate
      - `orientation` (object): Orientation quaternion
        - `x` (float)
        - `y` (float)
        - `z` (float)
        - `w` (float)
    - `target_object` (object, optional): Object to interact with
      - `name` (string): Object name
      - `type` (string): Object type
      - `properties` (object): Object properties
    - `gripper_position` (float, optional): Gripper position (0-1)
    - `force_limit` (float, optional): Maximum force to apply
  - `timeout` (float): Maximum time to complete the action (seconds)
  - `retry_count` (int): Number of allowed retries
- `execution_status` (string): Current status of the sequence (pending, executing, completed, failed, paused)
- `execution_log` (array of objects): Log of executed actions
  - `action_id` (string): Action identifier
  - `start_time` (float): Time when action started
  - `end_time` (float): Time when action ended
  - `status` (string): Result status (success, failed, skipped)
  - `error_message` (string, optional): Error message if action failed
- `estimated_duration` (float): Estimated time to complete the sequence
- `timestamp` (float): Time when the sequence was created

### Relationships
- Many-to-one: ActionSequence to CognitivePlan (many sequences for one plan - parallel execution)
- One-to-many: ActionSequence to ActionResponse (each sequence produces a response)

### Validation Rules
- Each action in the sequence must have a valid action_type
- Action parameters must match the requirements of the action_type
- Execution status must be one of the allowed values
- No circular dependencies in action sequence

### State Transitions
- [Created] → [Validated] → [Executing] → [Completed] / [Failed] / [Paused] / [Cancelled]

## Entity 4: RobotState

### Fields
- `state_id` (string): Unique identifier for the robot state
- `robot_id` (string): Identifier for the robot
- `timestamp` (float): Time when the state was captured
- `position` (object): Current position of the robot
  - `x` (float): X coordinate
  - `y` (float): Y coordinate
  - `z` (float): Z coordinate
- `orientation` (object): Current orientation as quaternion
  - `x` (float)
  - `y` (float)
  - `z` (float)
  - `w` (float)
- `joints` (array of objects): Joint positions (for manipulator robots)
  - `name` (string): Joint name
  - `position` (float): Joint position in radians or meters
  - `velocity` (float): Joint velocity
  - `effort` (float): Joint effort
- `gripper_state` (object): Gripper state
  - `position` (float): Gripper position (0-1)
  - `max_effort` (float): Maximum gripper effort
  - `is_grasping` (boolean): Whether object is grasped
- `battery_level` (float): Battery level as percentage (0.0-1.0)
- `current_action` (string): ID of currently executing action
- `capabilities` (array of strings): List of robot capabilities
- `sensors` (array of objects): Sensor readings
  - `sensor_name` (string): Name of the sensor
  - `sensor_type` (string): Type of sensor
  - `value` (object): Sensor reading value
  - `timestamp` (float): Time of reading

### Relationships
- Many-to-one: RobotState to ActionSequence (many states for one sequence - during execution)
- One-to-many: RobotState to VisionObservation (state provides context for vision)

### Validation Rules
- Position coordinates must be finite numbers
- Orientation quaternion must be normalized (unit length)
- Joint positions must be within robot limits
- Battery level must be between 0.0 and 1.0

### State Transitions
- RobotState is continuously updated as the robot moves and changes configuration

## Entity 5: VisionObservation

### Fields
- `observation_id` (string): Unique identifier for the observation
- `state_id` (string): Reference to the robot state at time of observation
- `image_data` (binary): Raw image data or URI to image file
- `timestamp` (float): Time when the observation was made
- `objects_detected` (array of objects): List of detected objects
  - `object_id` (string): Unique identifier for the object
  - `name` (string): Object name or classification
  - `confidence` (float): Detection confidence (0.0-1.0)
  - `bbox` (object): Bounding box coordinates
    - `x_min` (float): Minimum X coordinate
    - `y_min` (float): Minimum Y coordinate
    - `x_max` (float): Maximum X coordinate
    - `y_max` (float): Maximum Y coordinate
  - `position_3d` (object): 3D position in world coordinates
    - `x` (float): X coordinate
    - `y` (float): Y coordinate
    - `z` (float): Z coordinate
  - `properties` (object): Additional object properties
- `depth_data` (binary, optional): Depth information if available
- `camera_info` (object): Camera intrinsic parameters
  - `width` (int): Image width in pixels
  - `height` (int): Image height in pixels
  - `fx` (float): Focal length in x
  - `fy` (float): Focal length in y
  - `cx` (float): Principal point x
  - `cy` (float): Principal point y
  - `distortion_model` (string): Distortion model type
  - `distortion_coefficients` (array of floats): Distortion coefficients

### Relationships
- Many-to-one: VisionObservation to RobotState (many observations for one state timeline)
- One-to-many: VisionObservation to ActionSequence (observation may trigger actions)

### Validation Rules
- Objects detected must have confidence scores between 0.0 and 1.0
- Bounding box coordinates must be within image dimensions
- 3D positions must be valid coordinates in the robot's reference frame
- Camera info must have valid intrinsic parameters

## Entity 6: ActionResponse

### Fields
- `response_id` (string): Unique identifier for the response
- `sequence_id` (string): Reference to the executed action sequence
- `status` (string): Overall execution status (success, failed, partial_success)
- `completion_percentage` (float): Percentage of actions completed (0.0-1.0)
- `result_summary` (string): Human-readable summary of results
- `action_logs` (array of objects): Detailed logs of all executed actions
  - `action_id` (string): Action identifier
  - `status` (string): Action status (success, failed, skipped)
  - `start_time` (float): Time when action started
  - `end_time` (float): Time when action ended
  - `execution_time` (float): Duration of action execution
  - `error_message` (string, optional): Error details if action failed
  - `feedback` (object, optional): Feedback from action execution
- `timestamp` (float): Time when the response was generated
- `user_feedback` (object, optional): Feedback from the user
  - `satisfaction_rating` (int): Rating from 1-5
  - `comments` (string): Additional feedback comments

### Relationships
- Many-to-one: ActionResponse to ActionSequence (one response per sequence)
- One-to-many: ActionResponse to VoiceCommand (response may influence next commands)

### Validation Rules
- Status must be one of the allowed values
- Completion percentage must be between 0.0 and 1.0
- Action logs must reference valid actions from the sequence
- Completion percentage should match the actual status of actions

## Relationships Summary

```
VoiceCommand (1) ─── (1) CognitivePlan (1) ─── (M) ActionSequence (1) ─── (1) ActionResponse
     │                    │                      │                        │
     │                    │                      │                        └── (M) VoiceCommand
     │                    │                      └── (M) RobotState
     │                    └── (M) ActionSequence
     └── (M) CognitivePlan

RobotState (M) ─── (1) VisionObservation (1) ─── (M) ActionSequence
```

## State Transition Diagrams

### ActionSequence Lifecycle
```
[Created] → [Validated] → [Scheduled] → [Executing] → [Completed/Failed]
     ↓         ↓           ↓           ↓           ↓
[Validation] [Approval]  [Queue]   [In Progress] [Logging]
```

### CognitivePlan Lifecycle
```
[Generated] → [Validated] → [Approved] → [In Execution] → [Completed]
     ↓           ↓           ↓             ↓             ↓
[LLM Query]  [Safety Check][User Approval][Action Seq] [Feedback]
```

## Data Validation and Quality Assurance

### Voice Command Validation
- Audio quality checks (minimum duration, signal-to-noise ratio)
- Transcript confidence thresholds
- Language identification validation
- Duplicate command prevention

### Action Plan Validation
- Robot capability checks
- Environment constraint validation
- Safety constraint enforcement
- Action sequence feasibility verification

### Visual Observation Validation
- Object detection confidence thresholds
- Spatial consistency checks
- Temporal consistency with robot movement
- Sensor data quality assessment