"""
Student Module Progress Model
Tracks student's progress through the ROS 2 module
"""
from datetime import datetime
from typing import List, Optional, Dict
from pydantic import BaseModel, UUID4


class StudentModuleProgress(BaseModel):
    """
    Tracks student's progress through the ROS 2 module
    Fields:
    - id (UUID, Primary Key)
    - student_id (UUID, Foreign Key to User, Required)
    - module_id (UUID, Foreign Key to ROS2Module, Required)
    - progress_percentage (Integer, 0-100, Required)
    - started_at (DateTime)
    - completed_at (DateTime, Optional)
    - last_accessed_at (DateTime)
    - time_spent_seconds (Integer, Default: 0)
    - week_progress (JSON, Optional - {week_id: completion_percentage})
    - assessment_scores (JSON, Optional - {assessment_id: score})
    - project_submissions_status (JSON, Optional - {project_id: status})
    - is_completed (Boolean, Default: false)
    """
    id: UUID4
    student_id: UUID4
    module_id: UUID4
    progress_percentage: int
    started_at: datetime
    completed_at: Optional[datetime] = None
    last_accessed_at: datetime = datetime.now()
    time_spent_seconds: int = 0
    week_progress: Optional[Dict] = {}
    assessment_scores: Optional[Dict] = {}
    project_submissions_status: Optional[Dict] = {}
    is_completed: bool = False
    
    class Config:
        arbitrary_types_allowed = True