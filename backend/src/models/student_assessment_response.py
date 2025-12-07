"""
Student Assessment Response Model
Student's specific responses to assessment questions
"""
from datetime import datetime
from typing import Optional
from pydantic import BaseModel, UUID4


class StudentAssessmentResponse(BaseModel):
    """
    Student's specific responses to assessment questions
    Fields:
    - id (UUID, Primary Key)
    - result_id (UUID, Foreign Key to StudentAssessmentResult, Required)
    - question_id (UUID, Foreign Key to ROS2AssessmentQuestion, Required)
    - response_text (Text, Required)
    - is_correct (Boolean, Required)
    - points_awarded (Integer, Required)
    - feedback (Text, Optional)
    """
    id: UUID4
    result_id: UUID4
    question_id: UUID4
    response_text: str
    is_correct: bool
    points_awarded: int
    feedback: Optional[str] = None
    
    class Config:
        arbitrary_types_allowed = True