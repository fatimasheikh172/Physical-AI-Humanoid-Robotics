"""
Student Assessment Result Model
Student's results for ROS 2 assessments
"""
from datetime import datetime
from typing import Optional
from pydantic import BaseModel, UUID4


class StudentAssessmentResult(BaseModel):
    """
    Student's results for ROS 2 assessments
    Fields:
    - id (UUID, Primary Key)
    - student_id (UUID, Foreign Key to User, Required)
    - assessment_id (UUID, Foreign Key to ROS2Assessment, Required)
    - total_score (Integer, 0-100, Required)
    - max_score (Integer, Required)
    - started_at (DateTime)
    - completed_at (DateTime, Required)
    - time_taken_seconds (Integer, Required)
    - passed (Boolean, Required)
    - feedback (Text, Optional)
    """
    id: UUID4
    student_id: UUID4
    assessment_id: UUID4
    total_score: int
    max_score: int
    started_at: datetime
    completed_at: datetime
    time_taken_seconds: int
    passed: bool
    feedback: Optional[str] = None
    
    class Config:
        arbitrary_types_allowed = True