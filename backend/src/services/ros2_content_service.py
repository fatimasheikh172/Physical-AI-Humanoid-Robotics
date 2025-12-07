"""
ROS2 Content Section Service
Service layer for handling ROS2 content section operations
"""
from typing import List, Optional
from ..models.ros2_content_section import ROS2ContentSection
from uuid import UUID4


class ROS2ContentSectionService:
    """
    Service class to handle operations related to ROS2 content sections
    """
    
    def __init__(self):
        # In a real implementation, this would connect to a database
        self.sections = []
    
    async def create_section(self, section_data: dict) -> ROS2ContentSection:
        """
        Create a new ROS2 content section
        """
        section = ROS2ContentSection(**section_data)
        self.sections.append(section)
        return section
    
    async def get_section_by_id(self, section_id: UUID4) -> Optional[ROS2ContentSection]:
        """
        Retrieve a section by its ID
        """
        for section in self.sections:
            if section.id == section_id:
                return section
        return None
    
    async def get_sections_by_week_id(self, week_id: UUID4) -> List[ROS2ContentSection]:
        """
        Retrieve all sections for a specific week
        """
        return [section for section in self.sections if section.week_id == week_id]
    
    async def update_section(self, section_id: UUID4, update_data: dict) -> Optional[ROS2ContentSection]:
        """
        Update an existing section
        """
        for i, section in enumerate(self.sections):
            if section.id == section_id:
                # Update the section with new data
                for key, value in update_data.items():
                    if hasattr(section, key):
                        setattr(section, key, value)
                self.sections[i] = section
                return section
        return None
    
    async def delete_section(self, section_id: UUID4) -> bool:
        """
        Delete a section by its ID
        """
        for i, section in enumerate(self.sections):
            if section.id == section_id:
                del self.sections[i]
                return True
        return False