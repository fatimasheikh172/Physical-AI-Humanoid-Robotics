"""
API endpoints for handling ROS2 weeks
"""
from fastapi import APIRouter, HTTPException
from typing import List
from ..models.ros2_week import ROS2Week
from ..services.ros2_week_service import ROS2WeekService

router = APIRouter()
week_service = ROS2WeekService()


@router.get("/", response_model=List[ROS2Week])
async def get_all_weeks():
    """
    Get all ROS2 weeks
    """
    # In a real implementation, you might want to filter by module_id
    return await week_service.get_all_weeks()


@router.get("/{week_id}", response_model=ROS2Week)
async def get_week_by_id(week_id: str):
    """
    Get a specific ROS2 week by ID
    """
    week = await week_service.get_week_by_id(week_id)
    if not week:
        raise HTTPException(status_code=404, detail="Week not found")
    return week


@router.post("/", response_model=ROS2Week)
async def create_week(week: ROS2Week):
    """
    Create a new ROS2 week
    """
    return await week_service.create_week(week.dict())


@router.put("/{week_id}", response_model=ROS2Week)
async def update_week(week_id: str, week_update: dict):
    """
    Update an existing ROS2 week
    """
    updated_week = await week_service.update_week(week_id, week_update)
    if not updated_week:
        raise HTTPException(status_code=404, detail="Week not found")
    return updated_week


@router.delete("/{week_id}")
async def delete_week(week_id: str):
    """
    Delete a ROS2 week
    """
    deleted = await week_service.delete_week(week_id)
    if not deleted:
        raise HTTPException(status_code=404, detail="Week not found")
    return {"message": "Week deleted successfully"}