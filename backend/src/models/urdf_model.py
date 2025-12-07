"""
URDF Model
Represents URDF models created for humanoid robots in the module
"""
from datetime import datetime
from typing import Optional
from pydantic import BaseModel, UUID4


class URDFModel(BaseModel):
    """
    Represents URDF models created for humanoid robots in the module
    Fields:
    - id (UUID, Primary Key)
    - student_id (UUID, Foreign Key to User, Required)
    - name (String, Required)
    - description (Text, Optional)
    - urdf_content (Text, Required)
    - created_at (DateTime)
    - updated_at (DateTime)
    - is_shared (Boolean, Default: false)
    - evaluation_score (Integer, 0-100, Optional)
    - evaluation_feedback (Text, Optional)
    """
    id: UUID4
    student_id: UUID4
    name: str
    description: Optional[str] = None
    urdf_content: str
    created_at: datetime = datetime.now()
    updated_at: datetime = datetime.now()
    is_shared: bool = False
    evaluation_score: Optional[int] = None
    evaluation_feedback: Optional[str] = None
    
    class Config:
        arbitrary_types_allowed = True