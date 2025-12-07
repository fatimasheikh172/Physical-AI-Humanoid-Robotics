"""
ROS2 Content Section Model
Represents a section within a week's content (theory, lab instructions, etc.)
"""
from datetime import datetime
from typing import List, Optional
from pydantic import BaseModel, UUID4
from enum import Enum


class ContentType(Enum):
    THEORY = "theory"
    LAB_INSTRUCTION = "lab_instruction"
    CODE_EXAMPLE = "code_example"
    TUTORIAL = "tutorial"
    QUIZ = "quiz"
    PROJECT = "project"


class DifficultyLevel(Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class ROS2ContentSection(BaseModel):
    """
    A section within a week's content (theory, lab instructions, etc.)
    Fields:
    - id (UUID, Primary Key)
    - week_id (UUID, Foreign Key to ROS2Week, Required)
    - title (String, Required)
    - content_type (Enum: 'theory', 'lab_instruction', 'code_example', 'tutorial', 'quiz', 'project', Required)
    - content (Text, Required)
    - order_index (Integer, Required within week)
    - estimated_completion_time_minutes (Integer, Required)
    - difficulty_level (Enum: 'beginner', 'intermediate', 'advanced', Required)
    - prerequisite_section_ids (UUID Array, Optional)
    - created_at (DateTime)
    - updated_at (DateTime)
    """
    id: UUID4
    week_id: UUID4
    title: str
    content_type: ContentType
    content: str
    order_index: int
    estimated_completion_time_minutes: int
    difficulty_level: DifficultyLevel
    prerequisite_section_ids: Optional[List[UUID4]] = []
    created_at: datetime = datetime.now()
    updated_at: datetime = datetime.now()
    
    class Config:
        arbitrary_types_allowed = True