"""
ROS2 Module Model
Represents the complete Module 1 content focused on ROS 2 fundamentals
"""
from datetime import datetime
from typing import List, Optional
from pydantic import BaseModel, UUID4
from enum import Enum


class ROS2Module(BaseModel):
    """
    Represents the complete Module 1 content focused on ROS 2 fundamentals
    Fields:
    - id (UUID, Primary Key)
    - title (String, Required)
    - description (Text, Required)
    - module_number (Integer, Required, Value: 1)
    - duration_weeks (Integer, Required, Value: 3)
    - learning_objectives (Text Array, Required)
    - prerequisites (Text Array, Optional)
    - created_at (DateTime)
    - updated_at (DateTime)
    - published (Boolean, Default: false)
    """
    id: UUID4
    title: str
    description: str
    module_number: int = 1
    duration_weeks: int = 3
    learning_objectives: List[str]
    prerequisites: Optional[List[str]] = []
    created_at: datetime = datetime.now()
    updated_at: datetime = datetime.now()
    published: bool = False
    
    class Config:
        # This allows for the model to be serialized with UUIDs
        arbitrary_types_allowed = True