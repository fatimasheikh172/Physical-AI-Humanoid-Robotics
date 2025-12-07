"""
ROS2 AI Prompt Model
Prompts and queries sent to the AI tutor for ROS 2 assistance
"""
from datetime import datetime
from typing import Optional
from pydantic import BaseModel, UUID4


class ROS2AIPrompt(BaseModel):
    """
    Prompts and queries sent to the AI tutor for ROS 2 assistance
    Fields:
    - id (UUID, Primary Key)
    - student_id (UUID, Foreign Key to User, Required)
    - module_id (UUID, Foreign Key to ROS2Module, Required)
    - week_id (UUID, Foreign Key to ROS2Week, Optional)
    - query_text (Text, Required)
    - query_context (JSON, Optional - {lab_env: string, error_message: string, code_snippet: string})
    - response_text (Text, Required)
    - response_timestamp (DateTime, Required)
    - response_confidence (Float, 0.0-1.0, Optional)
    - was_helpful (Boolean, Optional)
    - feedback_rating (Integer, 1-5, Optional)
    - feedback_text (Text, Optional)
    """
    id: UUID4
    student_id: UUID4
    module_id: UUID4
    week_id: Optional[UUID4] = None
    query_text: str
    query_context: Optional[dict] = {}
    response_text: str
    response_timestamp: datetime
    response_confidence: Optional[float] = None
    was_helpful: Optional[bool] = None
    feedback_rating: Optional[int] = None
    feedback_text: Optional[str] = None
    
    class Config:
        arbitrary_types_allowed = True