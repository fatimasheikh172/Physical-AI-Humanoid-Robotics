"""
Lab Session Artifact Model
Artifacts generated during lab sessions (code, logs, etc.)
"""
from datetime import datetime
from typing import Optional
from pydantic import BaseModel, UUID4
from enum import Enum


class ArtifactType(Enum):
    CODE_OUTPUT = "code_output"
    CONSOLE_LOG = "console_log"
    IMAGE = "image"
    VIDEO = "video"
    GENERATED_FILE = "generated_file"


class LabSessionArtifact(BaseModel):
    """
    Artifacts generated during lab sessions (code, logs, etc.)
    Fields:
    - id (UUID, Primary Key)
    - lab_session_id (UUID, Foreign Key to StudentLabSession, Required)
    - artifact_type (Enum: 'code_output', 'console_log', 'image', 'video', 'generated_file', Required)
    - filename (String, Required)
    - file_path (String, Required)
    - file_size_bytes (Integer, Required)
    - mime_type (String, Required)
    - description (Text, Optional)
    - created_at (DateTime)
    """
    id: UUID4
    lab_session_id: UUID4
    artifact_type: ArtifactType
    filename: str
    file_path: str
    file_size_bytes: int
    mime_type: str
    description: Optional[str] = None
    created_at: datetime = datetime.now()
    
    class Config:
        arbitrary_types_allowed = True