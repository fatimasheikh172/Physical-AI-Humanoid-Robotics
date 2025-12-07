"""
Lab Asset Model
Assets associated with lab environments (code files, configuration, etc.)
"""
from datetime import datetime
from typing import Optional
from pydantic import BaseModel, UUID4
from enum import Enum


class AssetType(Enum):
    PYTHON_CODE = "python_code"
    LAUNCH_FILE = "launch_file"
    CONFIG = "config"
    URDF = "urdf"
    WORLD_FILE = "world_file"
    DOCUMENTATION = "documentation"


class LabAsset(BaseModel):
    """
    Assets associated with lab environments (code files, configuration, etc.)
    Fields:
    - id (UUID, Primary Key)
    - lab_env_id (UUID, Foreign Key to ROS2LabEnvironment, Required)
    - asset_type (Enum: 'python_code', 'launch_file', 'config', 'urdf', 'world_file', 'documentation', Required)
    - filename (String, Required)
    - original_filename (String, Required)
    - file_path (String, Required)
    - file_size_bytes (Integer, Required)
    - mime_type (String, Required)
    - checksum (String, Required for integrity verification)
    - uploaded_at (DateTime)
    - is_required (Boolean, Default: true)
    """
    id: UUID4
    lab_env_id: UUID4
    asset_type: AssetType
    filename: str
    original_filename: str
    file_path: str
    file_size_bytes: int
    mime_type: str
    checksum: str
    uploaded_at: datetime = datetime.now()
    is_required: bool = True
    
    class Config:
        arbitrary_types_allowed = True