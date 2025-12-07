"""
AI Bridge Model
Represents the integration between AI agents and ROS 2 systems
"""
from datetime import datetime
from typing import Optional
from pydantic import BaseModel, UUID4


class AIBridge(BaseModel):
    """
    Represents the integration between AI agents and ROS 2 systems
    Fields:
    - id (UUID, Primary Key)
    - student_id (UUID, Required)
    - ai_model_type (String, Required) - e.g., 'gpt-4', 'custom-ml-model'
    - bridge_config (JSON, Required) - configuration for connecting AI to ROS
    - created_at (DateTime)
    - updated_at (DateTime)
    """
    id: UUID4
    student_id: UUID4
    ai_model_type: str
    bridge_config: dict
    created_at: datetime = datetime.now()
    updated_at: datetime = datetime.now()
    
    class Config:
        arbitrary_types_allowed = True