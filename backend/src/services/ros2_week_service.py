"""
ROS2 Week Service
Service layer for handling ROS2 week operations
"""
from typing import List, Optional
from ..models.ros2_week import ROS2Week
from uuid import UUID4


class ROS2WeekService:
    """
    Service class to handle operations related to ROS2 weeks
    """
    
    def __init__(self):
        # In a real implementation, this would connect to a database
        self.weeks = []
    
    async def create_week(self, week_data: dict) -> ROS2Week:
        """
        Create a new ROS2 week
        """
        week = ROS2Week(**week_data)
        self.weeks.append(week)
        return week
    
    async def get_week_by_id(self, week_id: UUID4) -> Optional[ROS2Week]:
        """
        Retrieve a week by its ID
        """
        for week in self.weeks:
            if week.id == week_id:
                return week
        return None
    
    async def get_weeks_by_module_id(self, module_id: UUID4) -> List[ROS2Week]:
        """
        Retrieve all weeks for a specific module
        """
        return [week for week in self.weeks if week.module_id == module_id]
    
    async def update_week(self, week_id: UUID4, update_data: dict) -> Optional[ROS2Week]:
        """
        Update an existing week
        """
        for i, week in enumerate(self.weeks):
            if week.id == week_id:
                # Update the week with new data
                for key, value in update_data.items():
                    if hasattr(week, key):
                        setattr(week, key, value)
                self.weeks[i] = week
                return week
        return None
    
    async def delete_week(self, week_id: UUID4) -> bool:
        """
        Delete a week by its ID
        """
        for i, week in enumerate(self.weeks):
            if week.id == week_id:
                del self.weeks[i]
                return True
        return False