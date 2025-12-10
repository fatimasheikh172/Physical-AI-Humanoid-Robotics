"""
Message types for the Vision-Language-Action (VLA) module.

This module defines the data structures used for communication
between different components of the VLA system.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Any
from enum import Enum


class CommandType(Enum):
    """Enumeration of possible command types"""
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    PERCEPTION = "perception"
    COMPOSITE = "composite"


class ExecutionStatus(Enum):
    """Enumeration of possible execution statuses"""
    PENDING = "pending"
    VALIDATED = "validated"
    APPROVED = "approved"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"
    PAUSED = "paused"
    CANCELLED = "cancelled"


@dataclass
class VoiceCommand:
    """Represents a voice command captured from the user"""
    command_id: str
    transcript: str
    timestamp: float
    confidence: float
    user_id: str
    language: str = "en"
    session_id: Optional[str] = None
    audio_data: Optional[bytes] = None  # For potential use


@dataclass
class Task:
    """Represents a single task in an action sequence"""
    task_id: str
    task_description: str
    task_type: str  # navigation, manipulation, perception, etc.
    priority: int = 1
    parameters: Optional[Dict[str, Any]] = None


@dataclass
class CognitivePlan:
    """Represents an AI-generated plan based on a voice command"""
    plan_id: str
    command_id: str
    llm_model: str
    llm_response: str
    task_decomposition: List[Task]
    execution_context: Dict[str, Any]  # environment_map, robot_capabilities, safety_constraints
    timestamp: float
    confidence: float


@dataclass
class ActionTarget:
    """Represents a target for an action (location, object, etc.)"""
    name: str = ""
    type: str = ""  # "location", "object", etc.
    position: Optional[Dict[str, float]] = None  # x, y, z coordinates
    orientation: Optional[Dict[str, float]] = None  # quaternion: x, y, z, w
    properties: Optional[Dict[str, Any]] = None


@dataclass
class Action:
    """Represents a single action to be executed"""
    action_id: str
    action_type: str  # move_to, pick_up, place, detect_object, etc.
    parameters: Dict[str, Any]  # target_location, target_object, gripper_position, etc.
    timeout: float = 30.0  # seconds
    retry_count: int = 1


@dataclass
class ActionSequence:
    """Represents a sequence of executable actions derived from a cognitive plan"""
    sequence_id: str
    plan_id: str
    actions: List[Action]
    estimated_duration: float = 0.0
    timestamp: float = 0.0
    execution_status: ExecutionStatus = ExecutionStatus.PENDING
    execution_log: Optional[List[Dict[str, Any]]] = None  # action_id, start_time, end_time, status, error_message


@dataclass
class RobotState:
    """Represents the current state of the robot"""
    state_id: str
    robot_id: str
    timestamp: float
    position: Dict[str, float]  # x, y, z
    orientation: Dict[str, float]  # x, y, z, w (quaternion)
    joints: Optional[List[Dict[str, float]]] = None  # name, position, velocity, effort
    gripper_state: Optional[Dict[str, Any]] = None  # position, max_effort, is_grasping
    battery_level: float = 1.0  # percentage (0.0-1.0)
    current_action: Optional[str] = None
    capabilities: Optional[List[str]] = None
    sensors: Optional[List[Dict[str, Any]]] = None  # sensor_name, sensor_type, value, timestamp


@dataclass
class DetectedObject:
    """Represents a detected object from vision processing"""
    object_id: str
    name: str  # object name or classification
    confidence: float  # detection confidence (0.0-1.0)
    bbox: Dict[str, float]  # x_min, y_min, x_max, y_max
    position_3d: Optional[Dict[str, float]] = None  # x, y, z in world coordinates
    properties: Optional[Dict[str, Any]] = None


@dataclass
class VisionObservation:
    """Represents object detection and visual understanding data"""
    observation_id: str
    state_id: str  # Reference to robot state at time of observation
    timestamp: float
    objects_detected: List[DetectedObject]
    image_timestamp: Optional[float] = None
    camera_info: Optional[Dict[str, Any]] = None  # width, height, fx, fy, cx, cy, etc.
    image_data: Optional[bytes] = None  # For potential use


@dataclass
class ActionLog:
    """Represents a log entry for an executed action"""
    action_id: str
    status: ExecutionStatus
    start_time: float
    end_time: float
    execution_time: float
    error_message: Optional[str] = None
    feedback: Optional[Dict[str, Any]] = None


@dataclass
class ActionResponse:
    """Represents the result of executing an action sequence"""
    response_id: str
    sequence_id: str
    status: ExecutionStatus
    completion_percentage: float
    result_summary: str
    action_logs: List[ActionLog]
    timestamp: float
    user_feedback: Optional[Dict[str, Any]] = None  # satisfaction_rating, comments