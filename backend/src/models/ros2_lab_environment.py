"""
ROS2 Lab Environment Model
Represents the ROS 2 lab environment for a specific week
"""
from datetime import datetime
from typing import List
from pydantic import BaseModel, UUID4
from enum import Enum


class ROSDistro(Enum):
    HUMBLE = "humble"
    IRON = "iron"


class ROS2LabEnvironment(BaseModel):
    """
    Represents the ROS 2 lab environment for a specific week
    Fields:
    - id (UUID, Primary Key)
    - week_id (UUID, Foreign Key to ROS2Week, Required)
    - name (String, Required)
    - description (Text, Required)
    - ros_distro (Enum: 'humble', 'iron', Required, Default: 'humble')
    - package_name (String, Required)
    - launch_file (String, Required)
    - dependencies (Text Array, Required)
    - setup_instructions (Text, Required)
    - evaluation_criteria (JSON, Required - {criteria: array, weights: object})
    - created_at (DateTime)
    - updated_at (DateTime)
    """
    id: UUID4
    week_id: UUID4
    name: str
    description: str
    ros_distro: ROSDistro = ROSDistro.HUMBLE
    package_name: str
    launch_file: str
    dependencies: List[str]
    setup_instructions: str
    evaluation_criteria: dict
    created_at: datetime = datetime.now()
    updated_at: datetime = datetime.now()
    
    class Config:
        arbitrary_types_allowed = True