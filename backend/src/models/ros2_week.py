"""
ROS2 Week Model
Represents a week of content within the ROS 2 module
"""
from datetime import datetime
from typing import List, Optional, Dict
from pydantic import BaseModel, UUID4


class ROS2Week(BaseModel):
    """
    Represents a week of content within the ROS 2 module
    Fields:
    - id (UUID, Primary Key)
    - module_id (UUID, Foreign Key to ROS2Module, Required)
    - week_number (Integer, Required, 1-3)
    - title (String, Required)
    - description (Text, Required)
    - learning_objectives (Text Array, Required)
    - theory_content (Text, Required)
    - lab_tasks (JSON, Required - {task_name: description, setup: string, steps: array})
    - mini_project (JSON, Required - {title: string, description: text, requirements: array})
    - ai_tutor_integration (JSON, Required - {features: array, focus_areas: array})
    - estimated_duration_hours (Integer, Required)
    - order_index (Integer, Required within module)
    """
    id: UUID4
    module_id: UUID4
    week_number: int
    title: str
    description: str
    learning_objectives: List[str]
    theory_content: str
    lab_tasks: Dict
    mini_project: Dict
    ai_tutor_integration: Dict
    estimated_duration_hours: int
    order_index: int
    
    class Config:
        arbitrary_types_allowed = True