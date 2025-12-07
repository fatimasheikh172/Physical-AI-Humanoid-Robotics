"""
API endpoints for handling ROS2 content sections
"""
from fastapi import APIRouter, HTTPException
from typing import List
from ..models.ros2_content_section import ROS2ContentSection
from ..services.ros2_content_service import ROS2ContentSectionService

router = APIRouter()
content_service = ROS2ContentSectionService()


@router.get("/", response_model=List[ROS2ContentSection])
async def get_all_content_sections():
    """
    Get all ROS2 content sections
    """
    # In a real implementation, you might want to filter by week_id
    return await content_service.get_all_content_sections()


@router.get("/{section_id}", response_model=ROS2ContentSection)
async def get_content_section_by_id(section_id: str):
    """
    Get a specific ROS2 content section by ID
    """
    section = await content_service.get_section_by_id(section_id)
    if not section:
        raise HTTPException(status_code=404, detail="Content section not found")
    return section


@router.post("/", response_model=ROS2ContentSection)
async def create_content_section(section: ROS2ContentSection):
    """
    Create a new ROS2 content section
    """
    return await content_service.create_section(section.dict())


@router.put("/{section_id}", response_model=ROS2ContentSection)
async def update_content_section(section_id: str, section_update: dict):
    """
    Update an existing ROS2 content section
    """
    updated_section = await content_service.update_section(section_id, section_update)
    if not updated_section:
        raise HTTPException(status_code=404, detail="Content section not found")
    return updated_section


@router.delete("/{section_id}")
async def delete_content_section(section_id: str):
    """
    Delete a ROS2 content section
    """
    deleted = await content_service.delete_section(section_id)
    if not deleted:
        raise HTTPException(status_code=404, detail="Content section not found")
    return {"message": "Content section deleted successfully"}