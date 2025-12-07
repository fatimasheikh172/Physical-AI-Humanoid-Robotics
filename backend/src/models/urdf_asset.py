"""
URDF Model Asset
Assets associated with URDF models (mesh files, textures, etc.)
"""
from datetime import datetime
from typing import Optional
from pydantic import BaseModel, UUID4
from enum import Enum


class AssetType(Enum):
    MESH = "mesh"
    TEXTURE = "texture"
    MATERIAL = "material"
    CONFIGURATION = "configuration"


class URDFModelAsset(BaseModel):
    """
    Assets associated with URDF models (mesh files, textures, etc.)
    Fields:
    - id (UUID, Primary Key)
    - urdf_model_id (UUID, Foreign Key to URDFModel, Required)
    - asset_type (Enum: 'mesh', 'texture', 'material', 'configuration', Required)
    - filename (String, Required)
    - file_path (String, Required)
    - file_size_bytes (Integer, Required)
    - mime_type (String, Required)
    - checksum (String, Required for integrity verification)
    - uploaded_at (DateTime)
    """
    id: UUID4
    urdf_model_id: UUID4
    asset_type: AssetType
    filename: str
    file_path: str
    file_size_bytes: int
    mime_type: str
    checksum: str
    uploaded_at: datetime = datetime.now()
    
    class Config:
        arbitrary_types_allowed = True