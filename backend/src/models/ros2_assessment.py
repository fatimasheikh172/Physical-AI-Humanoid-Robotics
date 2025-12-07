"""
ROS2 Assessment Model
Assessment components for the ROS 2 module
"""
from datetime import datetime
from typing import List, Optional
from pydantic import BaseModel, UUID4
from enum import Enum


class AssessmentType(Enum):
    QUIZ = "quiz"
    PRACTICAL = "practical"
    THEORY_TEST = "theory_test"
    DEBUGGING_CHALLENGE = "debugging_challenge"
    FINAL_PROJECT = "final_project"


class ROS2Assessment(BaseModel):
    """
    Assessment components for the ROS 2 module
    Fields:
    - id (UUID, Primary Key)
    - module_id (UUID, Foreign Key to ROS2Module, Required)
    - week_id (UUID, Foreign Key to ROS2Week, Optional - null for final assessment)
    - title (String, Required)
    - description (Text, Required)
    - assessment_type (Enum: 'quiz', 'practical', 'theory_test', 'debugging_challenge', 'final_project', Required)
    - questions_count (Integer, Optional for practical assessments)
    - time_limit_minutes (Integer, Optional)
    - passing_score_percentage (Integer, 0-100, Required, Default: 70)
    - created_at (DateTime)
    - updated_at (DateTime)
    """
    id: UUID4
    module_id: UUID4
    week_id: Optional[UUID4] = None
    title: str
    description: str
    assessment_type: AssessmentType
    questions_count: Optional[int] = None
    time_limit_minutes: Optional[int] = None
    passing_score_percentage: int = 70
    created_at: datetime = datetime.now()
    updated_at: datetime = datetime.now()
    
    class Config:
        arbitrary_types_allowed = True