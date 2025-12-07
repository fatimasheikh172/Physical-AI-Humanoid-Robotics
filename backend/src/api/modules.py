"""
API endpoints for handling ROS2 modules
"""
from fastapi import APIRouter, HTTPException
from typing import List
from ..models.ros2_module import ROS2Module
from ..services.ros2_module_service import ROS2ModuleService

router = APIRouter()
module_service = ROS2ModuleService()


@router.get("/", response_model=List[ROS2Module])
async def get_all_modules():
    """
    Get all ROS2 modules
    """
    return await module_service.get_all_modules()


@router.get("/{module_id}", response_model=ROS2Module)
async def get_module_by_id(module_id: str):
    """
    Get a specific ROS2 module by ID
    """
    module = await module_service.get_module_by_id(module_id)
    if not module:
        raise HTTPException(status_code=404, detail="Module not found")
    return module


@router.post("/", response_model=ROS2Module)
async def create_module(module: ROS2Module):
    """
    Create a new ROS2 module
    """
    return await module_service.create_module(module.dict())


@router.put("/{module_id}", response_model=ROS2Module)
async def update_module(module_id: str, module_update: dict):
    """
    Update an existing ROS2 module
    """
    updated_module = await module_service.update_module(module_id, module_update)
    if not updated_module:
        raise HTTPException(status_code=404, detail="Module not found")
    return updated_module


@router.delete("/{module_id}")
async def delete_module(module_id: str):
    """
    Delete a ROS2 module
    """
    deleted = await module_service.delete_module(module_id)
    if not deleted:
        raise HTTPException(status_code=404, detail="Module not found")
    return {"message": "Module deleted successfully"}