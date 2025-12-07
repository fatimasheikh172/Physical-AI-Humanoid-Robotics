"""
API endpoints for handling ROS2 lab environments
"""
from fastapi import APIRouter, HTTPException
from typing import List
from ..models.ros2_lab_environment import ROS2LabEnvironment
from ..services.ros2_lab_service import ROS2LabEnvironmentService

router = APIRouter()
lab_service = ROS2LabEnvironmentService()


@router.get("/", response_model=List[ROS2LabEnvironment])
async def get_all_lab_environments():
    """
    Get all ROS2 lab environments
    """
    # In a real implementation, you might want to filter by week_id
    return await lab_service.get_all_lab_environments()


@router.get("/{lab_id}", response_model=ROS2LabEnvironment)
async def get_lab_environment_by_id(lab_id: str):
    """
    Get a specific ROS2 lab environment by ID
    """
    lab = await lab_service.get_lab_environment_by_id(lab_id)
    if not lab:
        raise HTTPException(status_code=404, detail="Lab environment not found")
    return lab


@router.post("/", response_model=ROS2LabEnvironment)
async def create_lab_environment(lab: ROS2LabEnvironment):
    """
    Create a new ROS2 lab environment
    """
    return await lab_service.create_lab_environment(lab.dict())


@router.put("/{lab_id}", response_model=ROS2LabEnvironment)
async def update_lab_environment(lab_id: str, lab_update: dict):
    """
    Update an existing ROS2 lab environment
    """
    updated_lab = await lab_service.update_lab_environment(lab_id, lab_update)
    if not updated_lab:
        raise HTTPException(status_code=404, detail="Lab environment not found")
    return updated_lab


@router.delete("/{lab_id}")
async def delete_lab_environment(lab_id: str):
    """
    Delete a ROS2 lab environment
    """
    deleted = await lab_service.delete_lab_environment(lab_id)
    if not deleted:
        raise HTTPException(status_code=404, detail="Lab environment not found")
    return {"message": "Lab environment deleted successfully"}