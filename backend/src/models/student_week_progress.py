"""
Student Week Progress Model
Tracks student's progress through individual weeks
"""
from datetime import datetime
from typing import Optional, Dict
from pydantic import BaseModel, UUID4


class StudentWeekProgress(BaseModel):
    """
    Tracks student's progress through individual weeks
    Fields:
    - id (UUID, Primary Key)
    - student_id (UUID, Foreign Key to User, Required)
    - week_id (UUID, Foreign Key to ROS2Week, Required)
    - progress_percentage (Integer, 0-100, Required)
    - started_at (DateTime)
    - completed_at (DateTime, Optional)
    - last_accessed_at (DateTime)
    - time_spent_seconds (Integer, Default: 0)
    - section_progress (JSON, Optional - {section_id: completion_status})
    - lab_completion_status (JSON, Optional - {lab_env_id: status})
    - assessment_scores (JSON, Optional - {assessment_id: score})
    - is_completed (Boolean, Default: false)
    """
    id: UUID4
    student_id: UUID4
    week_id: UUID4
    progress_percentage: int
    started_at: datetime
    completed_at: Optional[datetime] = None
    last_accessed_at: datetime = datetime.now()
    time_spent_seconds: int = 0
    section_progress: Optional[Dict] = {}
    lab_completion_status: Optional[Dict] = {}
    assessment_scores: Optional[Dict] = {}
    is_completed: bool = False
    
    class Config:
        arbitrary_types_allowed = True