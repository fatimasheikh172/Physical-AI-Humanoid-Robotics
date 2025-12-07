"""
ROS2 Lab Environment Service
Service layer for handling ROS2 lab environment operations
"""
from typing import List, Optional
from ..models.ros2_lab_environment import ROS2LabEnvironment
from uuid import UUID4


class ROS2LabEnvironmentService:
    """
    Service class to handle operations related to ROS2 lab environments
    """
    
    def __init__(self):
        # In a real implementation, this would connect to a database
        self.lab_environments = []
    
    async def create_lab_environment(self, lab_env_data: dict) -> ROS2LabEnvironment:
        """
        Create a new ROS2 lab environment
        """
        lab_env = ROS2LabEnvironment(**lab_env_data)
        self.lab_environments.append(lab_env)
        return lab_env
    
    async def get_lab_environment_by_id(self, lab_env_id: UUID4) -> Optional[ROS2LabEnvironment]:
        """
        Retrieve a lab environment by its ID
        """
        for lab_env in self.lab_environments:
            if lab_env.id == lab_env_id:
                return lab_env
        return None
    
    async def get_lab_environments_by_week_id(self, week_id: UUID4) -> List[ROS2LabEnvironment]:
        """
        Retrieve all lab environments for a specific week
        """
        return [lab_env for lab_env in self.lab_environments if lab_env.week_id == week_id]
    
    async def update_lab_environment(self, lab_env_id: UUID4, update_data: dict) -> Optional[ROS2LabEnvironment]:
        """
        Update an existing lab environment
        """
        for i, lab_env in enumerate(self.lab_environments):
            if lab_env.id == lab_env_id:
                # Update the lab environment with new data
                for key, value in update_data.items():
                    if hasattr(lab_env, key):
                        setattr(lab_env, key, value)
                self.lab_environments[i] = lab_env
                return lab_env
        return None
    
    async def delete_lab_environment(self, lab_env_id: UUID4) -> bool:
        """
        Delete a lab environment by its ID
        """
        for i, lab_env in enumerate(self.lab_environments):
            if lab_env.id == lab_env_id:
                del self.lab_environments[i]
                return True
        return False