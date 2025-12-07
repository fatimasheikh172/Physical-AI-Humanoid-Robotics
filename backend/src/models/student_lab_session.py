"""
Student Lab Session Model
Records of student's lab sessions with ROS 2
"""
from datetime import datetime
from typing import List, Optional
from pydantic import BaseModel, UUID4
from enum import Enum


class LabSessionStatus(Enum):
    ACTIVE = "active"
    COMPLETED = "completed"
    TERMINATED = "terminated"


class StudentLabSession(BaseModel):
    """
    Records of student's lab sessions with ROS 2
    Fields:
    - id (UUID, Primary Key)
    - student_id (UUID, Foreign Key to User, Required)
    - lab_env_id (UUID, Foreign Key to ROS2LabEnvironment, Required)
    - session_start (DateTime, Required)
    - session_end (DateTime, Optional)
    - duration_seconds (Integer, Optional)
    - ros_commands_used (Text Array, Optional)
    - errors_encountered (Text Array, Optional)
    - ai_tutor_requests (Integer, Default: 0)
    - status (Enum: 'active', 'completed', 'terminated', Required)
    - session_notes (Text, Optional)
    """
    id: UUID4
    student_id: UUID4
    lab_env_id: UUID4
    session_start: datetime
    session_end: Optional[datetime] = None
    duration_seconds: Optional[int] = None
    ros_commands_used: Optional[List[str]] = []
    errors_encountered: Optional[List[str]] = []
    ai_tutor_requests: int = 0
    status: LabSessionStatus
    session_notes: Optional[str] = None
    
    class Config:
        arbitrary_types_allowed = True