"""
ROS2 Module Service
Service layer for handling ROS2 module operations
"""
from typing import List, Optional
from ..models.ros2_module import ROS2Module
from uuid import UUID4


class ROS2ModuleService:
    """
    Service class to handle operations related to ROS2 modules
    """
    
    def __init__(self):
        # In a real implementation, this would connect to a database
        self.modules = []
    
    async def create_module(self, module_data: dict) -> ROS2Module:
        """
        Create a new ROS2 module
        """
        # Create a new module instance with the provided data
        module = ROS2Module(**module_data)
        self.modules.append(module)
        return module
    
    async def get_module_by_id(self, module_id: UUID4) -> Optional[ROS2Module]:
        """
        Retrieve a module by its ID
        """
        for module in self.modules:
            if module.id == module_id:
                return module
        return None
    
    async def get_all_modules(self) -> List[ROS2Module]:
        """
        Retrieve all modules
        """
        return self.modules
    
    async def update_module(self, module_id: UUID4, update_data: dict) -> Optional[ROS2Module]:
        """
        Update an existing module
        """
        for i, module in enumerate(self.modules):
            if module.id == module_id:
                # Update the module with new data
                for key, value in update_data.items():
                    if hasattr(module, key):
                        setattr(module, key, value)
                self.modules[i] = module
                return module
        return None
    
    async def delete_module(self, module_id: UUID4) -> bool:
        """
        Delete a module by its ID
        """
        for i, module in enumerate(self.modules):
            if module.id == module_id:
                del self.modules[i]
                return True
        return False