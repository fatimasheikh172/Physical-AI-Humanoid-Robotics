"""
Student Section Progress Model
Tracks student's progress through individual content sections
"""
from datetime import datetime
from typing import Optional
from pydantic import BaseModel, UUID4
from enum import Enum


class CompletionStatus(Enum):
    NOT_STARTED = "not_started"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"


class StudentSectionProgress(BaseModel):
    """
    Tracks student's progress through individual content sections
    Fields:
    - id (UUID, Primary Key)
    - student_id (UUID, Foreign Key to User, Required)
    - section_id (UUID, Foreign Key to ROS2ContentSection, Required)
    - started_at (DateTime)
    - completed_at (DateTime, Optional)
    - time_spent_seconds (Integer, Default: 0)
    - completion_status (Enum: 'not_started', 'in_progress', 'completed', Required, Default: 'not_started')
    - notes (Text, Optional)
    """
    id: UUID4
    student_id: UUID4
    section_id: UUID4
    started_at: datetime
    completed_at: Optional[datetime] = None
    time_spent_seconds: int = 0
    completion_status: CompletionStatus = CompletionStatus.NOT_STARTED
    notes: Optional[str] = None
    
    class Config:
        arbitrary_types_allowed = True