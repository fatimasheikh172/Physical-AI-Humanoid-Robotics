"""
ROS2 Mini Project Model
Represents the mini-project for each week of the module
"""
from datetime import datetime
from typing import List
from pydantic import BaseModel, UUID4


class ROS2MiniProject(BaseModel):
    """
    Represents the mini-project for each week of the module
    Fields:
    - id (UUID, Primary Key)
    - week_id (UUID, Foreign Key to ROS2Week, Required)
    - title (String, Required)
    - description (Text, Required)
    - requirements (Text Array, Required)
    - evaluation_criteria (JSON, Required - {criteria: array, weights: object})
    - submission_instructions (Text, Required)
    - created_at (DateTime)
    - updated_at (DateTime)
    """
    id: UUID4
    week_id: UUID4
    title: str
    description: str
    requirements: List[str]
    evaluation_criteria: dict
    submission_instructions: str
    created_at: datetime = datetime.now()
    updated_at: datetime = datetime.now()
    
    class Config:
        arbitrary_types_allowed = True