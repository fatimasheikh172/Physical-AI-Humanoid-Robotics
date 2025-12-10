"""
Data models as Python classes for the Vision-Language-Action (VLA) module.

This module implements the data entities defined in the data model specification
as Python classes with validation and helper methods.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Optional, Any
from datetime import datetime
from enum import Enum
import uuid
import math

from .message_types import (
    VoiceCommand, CognitivePlan, Task, Action, ActionSequence, 
    RobotState, DetectedObject, VisionObservation, ActionResponse, 
    ActionLog, ExecutionStatus
)


@dataclass
class VoiceCommandModel:
    """
    VoiceCommand entity implementation with validation and helper methods.
    
    Represents a voice command captured from the user.
    """
    command_id: str
    transcript: str
    timestamp: float
    confidence: float
    user_id: str
    language: str = "en"
    session_id: Optional[str] = None
    audio_data: Optional[bytes] = None
    
    def __post_init__(self):
        """Validate fields after initialization."""
        self.validate()
    
    def validate(self):
        """Validate the VoiceCommand fields."""
        if not self.command_id:
            raise ValueError("command_id cannot be empty")
        
        if not self.transcript.strip():
            raise ValueError("transcript cannot be empty")
        
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be between 0.0 and 1.0")
        
        if not self.user_id:
            raise ValueError("user_id cannot be empty")
        
        if self.timestamp < 0:
            raise ValueError("timestamp cannot be negative")
    
    @classmethod
    def create(cls, transcript: str, user_id: str, language: str = "en") -> 'VoiceCommandModel':
        """Create a new VoiceCommand with auto-generated fields."""
        return cls(
            command_id=f"cmd_{uuid.uuid4().hex[:8]}",
            transcript=transcript,
            timestamp=datetime.now().timestamp(),
            confidence=0.9,  # Default confidence
            user_id=user_id,
            language=language
        )
    
    def to_message_types(self) -> VoiceCommand:
        """Convert to the message types format."""
        return VoiceCommand(
            command_id=self.command_id,
            transcript=self.transcript,
            timestamp=self.timestamp,
            confidence=self.confidence,
            user_id=self.user_id,
            language=self.language,
            session_id=self.session_id,
            audio_data=self.audio_data
        )


@dataclass
class TaskModel:
    """
    Task entity implementation with validation and helper methods.
    
    Represents a single task in an action sequence.
    """
    task_id: str
    task_description: str
    task_type: str  # navigation, manipulation, perception, etc.
    priority: int = 1
    parameters: Optional[Dict[str, Any]] = field(default_factory=dict)
    
    def __post_init__(self):
        """Validate fields after initialization."""
        self.validate()
    
    def validate(self):
        """Validate the Task fields."""
        if not self.task_id:
            raise ValueError("task_id cannot be empty")
        
        if not self.task_description.strip():
            raise ValueError("task_description cannot be empty")
        
        if not self.task_type.strip():
            raise ValueError("task_type cannot be empty")
        
        if self.priority < 1:
            raise ValueError("priority must be at least 1")
    
    @classmethod
    def create(cls, task_description: str, task_type: str, 
               parameters: Optional[Dict[str, Any]] = None) -> 'TaskModel':
        """Create a new Task with auto-generated fields."""
        return cls(
            task_id=f"task_{uuid.uuid4().hex[:8]}",
            task_description=task_description,
            task_type=task_type,
            priority=1,
            parameters=parameters or {}
        )
    
    def to_message_types(self) -> Task:
        """Convert to the message types format."""
        return Task(
            task_id=self.task_id,
            task_description=self.task_description,
            task_type=self.task_type,
            priority=self.priority,
            parameters=self.parameters
        )


@dataclass
class CognitivePlanModel:
    """
    CognitivePlan entity implementation with validation and helper methods.
    
    Represents an AI-generated plan based on a voice command.
    """
    plan_id: str
    command_id: str
    llm_model: str
    llm_response: str
    task_decomposition: List[TaskModel]
    execution_context: Dict[str, Any]  # environment_map, robot_capabilities, safety_constraints
    timestamp: float
    confidence: float
    
    def __post_init__(self):
        """Validate fields after initialization."""
        self.validate()
    
    def validate(self):
        """Validate the CognitivePlan fields."""
        if not self.plan_id:
            raise ValueError("plan_id cannot be empty")
        
        if not self.command_id:
            raise ValueError("command_id cannot be empty")
        
        if not self.llm_model:
            raise ValueError("llm_model cannot be empty")
        
        if not self.llm_response.strip():
            raise ValueError("llm_response cannot be empty")
        
        if not self.task_decomposition:
            raise ValueError("task_decomposition cannot be empty")
        
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be between 0.0 and 1.0")
        
        if self.timestamp < 0:
            raise ValueError("timestamp cannot be negative")
    
    @classmethod
    def create(cls, command_id: str, llm_model: str, llm_response: str,
               task_decomposition: List[TaskModel], execution_context: Dict[str, Any]) -> 'CognitivePlanModel':
        """Create a new CognitivePlan with auto-generated fields."""
        return cls(
            plan_id=f"plan_{uuid.uuid4().hex[:8]}",
            command_id=command_id,
            llm_model=llm_model,
            llm_response=llm_response,
            task_decomposition=task_decomposition,
            execution_context=execution_context,
            timestamp=datetime.now().timestamp(),
            confidence=0.8  # Default confidence
        )
    
    def to_message_types(self) -> CognitivePlan:
        """Convert to the message types format."""
        return CognitivePlan(
            plan_id=self.plan_id,
            command_id=self.command_id,
            llm_model=self.llm_model,
            llm_response=self.llm_response,
            task_decomposition=[task.to_message_types() for task in self.task_decomposition],
            execution_context=self.execution_context,
            timestamp=self.timestamp,
            confidence=self.confidence
        )


@dataclass
class ActionModel:
    """
    Action entity implementation with validation and helper methods.
    
    Represents a single action to be executed.
    """
    action_id: str
    action_type: str  # move_to, pick_up, place, detect_object, etc.
    parameters: Dict[str, Any]  # target_location, target_object, gripper_position, etc.
    timeout: float = 30.0  # seconds
    retry_count: int = 1
    
    def __post_init__(self):
        """Validate fields after initialization."""
        self.validate()
    
    def validate(self):
        """Validate the Action fields."""
        if not self.action_id:
            raise ValueError("action_id cannot be empty")
        
        if not self.action_type.strip():
            raise ValueError("action_type cannot be empty")
        
        if self.timeout <= 0:
            raise ValueError("timeout must be positive")
        
        if self.retry_count < 0:
            raise ValueError("retry_count cannot be negative")
    
    @classmethod
    def create(cls, action_type: str, parameters: Dict[str, Any],
               timeout: float = 30.0, retry_count: int = 1) -> 'ActionModel':
        """Create a new Action with auto-generated fields."""
        return cls(
            action_id=f"act_{uuid.uuid4().hex[:8]}",
            action_type=action_type,
            parameters=parameters,
            timeout=timeout,
            retry_count=retry_count
        )
    
    def to_message_types(self) -> Action:
        """Convert to the message types format."""
        return Action(
            action_id=self.action_id,
            action_type=self.action_type,
            parameters=self.parameters,
            timeout=self.timeout,
            retry_count=self.retry_count
        )


@dataclass
class ActionSequenceModel:
    """
    ActionSequence entity implementation with validation and helper methods.
    
    Represents a sequence of executable actions derived from a cognitive plan.
    """
    sequence_id: str
    plan_id: str
    actions: List[ActionModel]
    estimated_duration: float = 0.0
    timestamp: float = 0.0
    execution_status: ExecutionStatus = ExecutionStatus.PENDING
    execution_log: Optional[List[Dict[str, Any]]] = field(default_factory=list)
    
    def __post_init__(self):
        """Validate fields after initialization."""
        self.validate()
    
    def validate(self):
        """Validate the ActionSequence fields."""
        if not self.sequence_id:
            raise ValueError("sequence_id cannot be empty")
        
        if not self.plan_id:
            raise ValueError("plan_id cannot be empty")
        
        if not self.actions:
            raise ValueError("actions cannot be empty")
        
        if self.estimated_duration < 0:
            raise ValueError("estimated_duration cannot be negative")
    
    @classmethod
    def create(cls, plan_id: str, actions: List[ActionModel]) -> 'ActionSequenceModel':
        """Create a new ActionSequence with auto-generated fields."""
        return cls(
            sequence_id=f"seq_{uuid.uuid4().hex[:8]}",
            plan_id=plan_id,
            actions=actions,
            timestamp=datetime.now().timestamp()
        )
    
    def to_message_types(self) -> ActionSequence:
        """Convert to the message types format."""
        return ActionSequence(
            sequence_id=self.sequence_id,
            plan_id=self.plan_id,
            actions=[action.to_message_types() for action in self.actions],
            estimated_duration=self.estimated_duration,
            timestamp=self.timestamp,
            execution_status=self.execution_status,
            execution_log=self.execution_log
        )
    
    def add_log_entry(self, action_id: str, status: str, start_time: float, 
                     end_time: float, error_message: Optional[str] = None):
        """Add an execution log entry."""
        log_entry = {
            'action_id': action_id,
            'status': status,
            'start_time': start_time,
            'end_time': end_time,
            'execution_time': end_time - start_time,
            'error_message': error_message
        }
        self.execution_log.append(log_entry)


@dataclass
class RobotStateModel:
    """
    RobotState entity implementation with validation and helper methods.
    
    Represents the current state of the robot.
    """
    state_id: str
    robot_id: str
    timestamp: float
    position: Dict[str, float]  # x, y, z
    orientation: Dict[str, float]  # x, y, z, w (quaternion)
    joints: Optional[List[Dict[str, float]]] = field(default_factory=list)  # name, position, velocity, effort
    gripper_state: Optional[Dict[str, Any]] = field(default_factory=dict)  # position, max_effort, is_grasping
    battery_level: float = 1.0  # percentage (0.0-1.0)
    current_action: Optional[str] = None
    capabilities: Optional[List[str]] = field(default_factory=list)
    sensors: Optional[List[Dict[str, Any]]] = field(default_factory=list)  # sensor_name, sensor_type, value, timestamp
    
    def __post_init__(self):
        """Validate fields after initialization."""
        self.validate()
    
    def validate(self):
        """Validate the RobotState fields."""
        if not self.state_id:
            raise ValueError("state_id cannot be empty")
        
        if not self.robot_id:
            raise ValueError("robot_id cannot be empty")
        
        if self.timestamp < 0:
            raise ValueError("timestamp cannot be negative")
        
        # Validate position
        if not all(k in self.position for k in ['x', 'y', 'z']):
            raise ValueError("position must contain x, y, z coordinates")
        
        # Validate orientation (quaternion should be normalized)
        if not all(k in self.orientation for k in ['x', 'y', 'z', 'w']):
            raise ValueError("orientation must contain x, y, z, w components")
        
        norm = math.sqrt(
            self.orientation['x']**2 + 
            self.orientation['y']**2 + 
            self.orientation['z']**2 + 
            self.orientation['w']**2
        )
        if abs(norm - 1.0) > 0.1:  # Allow some tolerance for normalization errors
            raise ValueError(f"orientation quaternion should be normalized (norm={norm})")
        
        if not 0.0 <= self.battery_level <= 1.0:
            raise ValueError("battery_level must be between 0.0 and 1.0")
    
    @classmethod
    def create(cls, robot_id: str, position: Dict[str, float], 
               orientation: Dict[str, float]) -> 'RobotStateModel':
        """Create a new RobotState with auto-generated fields."""
        return cls(
            state_id=f"state_{uuid.uuid4().hex[:8]}",
            robot_id=robot_id,
            timestamp=datetime.now().timestamp(),
            position=position,
            orientation=orientation
        )
    
    def to_message_types(self) -> RobotState:
        """Convert to the message types format."""
        return RobotState(
            state_id=self.state_id,
            robot_id=self.robot_id,
            timestamp=self.timestamp,
            position=self.position,
            orientation=self.orientation,
            joints=self.joints,
            gripper_state=self.gripper_state,
            battery_level=self.battery_level,
            current_action=self.current_action,
            capabilities=self.capabilities,
            sensors=self.sensors
        )


@dataclass
class DetectedObjectModel:
    """
    DetectedObject entity implementation with validation and helper methods.
    
    Represents a detected object from vision processing.
    """
    object_id: str
    name: str  # object name or classification
    confidence: float  # detection confidence (0.0-1.0)
    bbox: Dict[str, float]  # x_min, y_min, x_max, y_max
    position_3d: Optional[Dict[str, float]] = field(default_factory=dict)  # x, y, z in world coordinates
    properties: Optional[Dict[str, Any]] = field(default_factory=dict)
    
    def __post_init__(self):
        """Validate fields after initialization."""
        self.validate()
    
    def validate(self):
        """Validate the DetectedObject fields."""
        if not self.object_id:
            raise ValueError("object_id cannot be empty")
        
        if not self.name.strip():
            raise ValueError("name cannot be empty")
        
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be between 0.0 and 1.0")
        
        # Validate bbox fields
        for coord in ['x_min', 'y_min', 'x_max', 'y_max']:
            if coord not in self.bbox:
                raise ValueError(f"bbox must contain {coord}")
            if self.bbox[coord] < 0:
                raise ValueError(f"bbox {coord} cannot be negative")
        
        # Validate position_3d if provided
        if self.position_3d:
            for coord in ['x', 'y', 'z']:
                if coord not in self.position_3d:
                    raise ValueError(f"position_3d must contain {coord} if specified")
    
    @classmethod
    def create(cls, name: str, confidence: float, bbox: Dict[str, float],
               position_3d: Optional[Dict[str, float]] = None) -> 'DetectedObjectModel':
        """Create a new DetectedObject with auto-generated fields."""
        return cls(
            object_id=f"obj_{uuid.uuid4().hex[:8]}",
            name=name,
            confidence=confidence,
            bbox=bbox,
            position_3d=position_3d or {}
        )
    
    def to_message_types(self) -> DetectedObject:
        """Convert to the message types format."""
        return DetectedObject(
            object_id=self.object_id,
            name=self.name,
            confidence=self.confidence,
            bbox=self.bbox,
            position_3d=self.position_3d,
            properties=self.properties
        )


@dataclass
class VisionObservationModel:
    """
    VisionObservation entity implementation with validation and helper methods.
    
    Represents object detection and visual understanding data.
    """
    observation_id: str
    state_id: str  # Reference to robot state at time of observation
    timestamp: float
    objects_detected: List[DetectedObjectModel]
    image_timestamp: Optional[float] = None
    camera_info: Optional[Dict[str, Any]] = field(default_factory=dict)  # width, height, fx, fy, cx, cy, etc.
    image_data: Optional[bytes] = None  # For potential use
    
    def __post_init__(self):
        """Validate fields after initialization."""
        self.validate()
    
    def validate(self):
        """Validate the VisionObservation fields."""
        if not self.observation_id:
            raise ValueError("observation_id cannot be empty")
        
        if not self.state_id:
            raise ValueError("state_id cannot be empty")
        
        if self.timestamp < 0:
            raise ValueError("timestamp cannot be negative")
        
        if self.image_timestamp is not None and self.image_timestamp < 0:
            raise ValueError("image_timestamp cannot be negative")
    
    @classmethod
    def create(cls, state_id: str, objects_detected: List[DetectedObjectModel]) -> 'VisionObservationModel':
        """Create a new VisionObservation with auto-generated fields."""
        return cls(
            observation_id=f"obs_{uuid.uuid4().hex[:8]}",
            state_id=state_id,
            timestamp=datetime.now().timestamp(),
            objects_detected=objects_detected
        )
    
    def to_message_types(self) -> VisionObservation:
        """Convert to the message types format."""
        return VisionObservation(
            observation_id=self.observation_id,
            state_id=self.state_id,
            timestamp=self.timestamp,
            objects_detected=[obj.to_message_types() for obj in self.objects_detected],
            image_timestamp=self.image_timestamp,
            camera_info=self.camera_info,
            image_data=self.image_data
        )


@dataclass
class ActionLogModel:
    """
    ActionLog entity implementation with validation and helper methods.
    
    Represents a log entry for an executed action.
    """
    action_id: str
    status: ExecutionStatus
    start_time: float
    end_time: float
    execution_time: float
    error_message: Optional[str] = None
    feedback: Optional[Dict[str, Any]] = field(default_factory=dict)
    
    def __post_init__(self):
        """Validate fields after initialization."""
        self.validate()
    
    def validate(self):
        """Validate the ActionLog fields."""
        if not self.action_id:
            raise ValueError("action_id cannot be empty")
        
        if self.start_time < 0 or self.end_time < 0:
            raise ValueError("start_time and end_time cannot be negative")
        
        if self.end_time < self.start_time:
            raise ValueError("end_time cannot be less than start_time")
        
        if self.execution_time != (self.end_time - self.start_time):
            raise ValueError("execution_time must equal (end_time - start_time)")
    
    @classmethod
    def create(cls, action_id: str, status: ExecutionStatus, 
               start_time: float, end_time: float) -> 'ActionLogModel':
        """Create a new ActionLog with auto-generated fields."""
        execution_time = end_time - start_time
        return cls(
            action_id=action_id,
            status=status,
            start_time=start_time,
            end_time=end_time,
            execution_time=execution_time
        )
    
    def to_message_types(self) -> ActionLog:
        """Convert to the message types format."""
        return ActionLog(
            action_id=self.action_id,
            status=self.status,
            start_time=self.start_time,
            end_time=self.end_time,
            execution_time=self.execution_time,
            error_message=self.error_message,
            feedback=self.feedback
        )


@dataclass
class ActionResponseModel:
    """
    ActionResponse entity implementation with validation and helper methods.
    
    Represents the result of executing an action sequence.
    """
    response_id: str
    sequence_id: str
    status: ExecutionStatus
    completion_percentage: float
    result_summary: str
    action_logs: List[ActionLogModel]
    timestamp: float
    user_feedback: Optional[Dict[str, Any]] = field(default_factory=dict)  # satisfaction_rating, comments
    
    def __post_init__(self):
        """Validate fields after initialization."""
        self.validate()
    
    def validate(self):
        """Validate the ActionResponse fields."""
        if not self.response_id:
            raise ValueError("response_id cannot be empty")
        
        if not self.sequence_id:
            raise ValueError("sequence_id cannot be empty")
        
        if not 0.0 <= self.completion_percentage <= 1.0:
            raise ValueError("completion_percentage must be between 0.0 and 1.0")
        
        if self.timestamp < 0:
            raise ValueError("timestamp cannot be negative")
    
    @classmethod
    def create(cls, sequence_id: str, status: ExecutionStatus, 
               completion_percentage: float, result_summary: str,
               action_logs: List[ActionLogModel]) -> 'ActionResponseModel':
        """Create a new ActionResponse with auto-generated fields."""
        return cls(
            response_id=f"resp_{uuid.uuid4().hex[:8]}",
            sequence_id=sequence_id,
            status=status,
            completion_percentage=completion_percentage,
            result_summary=result_summary,
            action_logs=action_logs,
            timestamp=datetime.now().timestamp()
        )
    
    def to_message_types(self) -> ActionResponse:
        """Convert to the message types format."""
        return ActionResponse(
            response_id=self.response_id,
            sequence_id=self.sequence_id,
            status=self.status,
            completion_percentage=self.completion_percentage,
            result_summary=self.result_summary,
            action_logs=[log.to_message_types() for log in self.action_logs],
            timestamp=self.timestamp,
            user_feedback=self.user_feedback
        )