"""
ROS2 Assessment Question Model
Individual question within an ROS 2 assessment
"""
from datetime import datetime
from typing import List, Optional
from pydantic import BaseModel, UUID4
from enum import Enum


class QuestionType(Enum):
    MULTIPLE_CHOICE = "multiple_choice"
    TRUE_FALSE = "true_false"
    SHORT_ANSWER = "short_answer"
    CODE = "code"
    PRACTICAL = "practical"


class DifficultyLevel(Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class ROS2AssessmentQuestion(BaseModel):
    """
    Individual question within an ROS 2 assessment
    Fields:
    - id (UUID, Primary Key)
    - assessment_id (UUID, Foreign Key to ROS2Assessment, Required)
    - question_text (Text, Required)
    - question_type (Enum: 'multiple_choice', 'true_false', 'short_answer', 'code', 'practical', Required)
    - options (JSON Array, Optional - for multiple choice)
    - correct_answer (Text, Required)
    - explanation (Text, Optional)
    - difficulty_level (Enum: 'beginner', 'intermediate', 'advanced', Required)
    - points (Integer, Required, Default: 1)
    - order_index (Integer, Required within assessment)
    """
    id: UUID4
    assessment_id: UUID4
    question_text: str
    question_type: QuestionType
    options: Optional[List[str]] = []
    correct_answer: str
    explanation: Optional[str] = None
    difficulty_level: DifficultyLevel
    points: int = 1
    order_index: int
    
    class Config:
        arbitrary_types_allowed = True