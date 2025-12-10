"""
API key management and .env handling for the Vision-Language-Action (VLA) module.

This module handles secure storage and retrieval of API keys.
"""

import os
from typing import Optional
from dotenv import load_dotenv
import logging


# Load environment variables from .env file
load_dotenv()


class APIKeyManager:
    """Manages API keys for various services used in the VLA module"""
    
    def __init__(self, env_file_path: Optional[str] = None):
        self.env_file_path = env_file_path or ".env"
        self._api_keys = {}
        
        # Load keys at initialization
        self.load_keys()
        
        # Set up logging
        self.logger = logging.getLogger(self.__class__.__name__)
    
    def load_keys(self):
        """Load API keys from environment variables"""
        # OpenAI API key
        self._api_keys['openai'] = os.getenv('OPENAI_API_KEY')
        if not self._api_keys['openai']:
            self.logger.warning("OPENAI_API_KEY not found in environment variables")
        
        # Anthropic API key
        self._api_keys['anthropic'] = os.getenv('ANTHROPIC_API_KEY')
        if not self._api_keys['anthropic']:
            self.logger.info("ANTHROPIC_API_KEY not found in environment variables (optional)")
        
        # Other potential API keys can be added here
        self.logger.info("API keys loaded from environment variables")
    
    def get_key(self, service_name: str) -> Optional[str]:
        """
        Get an API key for a specific service
        
        Args:
            service_name: Name of the service ('openai', 'anthropic', etc.)
            
        Returns:
            API key string or None if not found
        """
        key = self._api_keys.get(service_name.lower())
        if key is None:
            self.logger.warning(f"No API key found for service: {service_name}")
        return key
    
    def set_key(self, service_name: str, api_key: str):
        """
        Set an API key for a specific service (at runtime, not persisted)
        
        Args:
            service_name: Name of the service
            api_key: The API key string
        """
        self._api_keys[service_name.lower()] = api_key
        self.logger.info(f"Runtime API key set for service: {service_name}")
    
    def validate_key(self, service_name: str) -> bool:
        """
        Validate if an API key for a service is present and appears valid
        
        Args:
            service_name: Name of the service
            
        Returns:
            True if key is valid, False otherwise
        """
        key = self.get_key(service_name)
        if not key:
            return False
        
        # Basic validation: check if key has reasonable length and format
        # This is a simple check - actual validation would depend on the service
        if len(key) < 10:  # Arbitrary minimum length
            self.logger.warning(f"API key for {service_name} appears too short")
            return False
        
        # Check if key looks like it might be a placeholder
        if key.lower() in ['your_openai_api_key_here', 'your_anthropic_api_key_here', '']:
            self.logger.warning(f"API key for {service_name} appears to be a placeholder")
            return False
        
        return True
    
    def is_service_available(self, service_name: str) -> bool:
        """
        Check if a service is available (has a valid API key)
        
        Args:
            service_name: Name of the service
            
        Returns:
            True if service is available, False otherwise
        """
        return self.validate_key(service_name)
    
    def get_all_services(self) -> list:
        """Get a list of all services with API keys"""
        return [service for service, key in self._api_keys.items() if key]
    
    def save_to_env_file(self, service_name: str, api_key: str):
        """
        Save an API key to the .env file
        
        Args:
            service_name: Name of the service
            api_key: The API key string
        """
        service_env_var = service_name.upper() + "_API_KEY"
        
        # Read existing file content
        lines = []
        if os.path.exists(self.env_file_path):
            with open(self.env_file_path, 'r') as file:
                lines = file.readlines()
        
        # Check if key already exists in the file
        key_exists = False
        for i, line in enumerate(lines):
            if line.startswith(f"{service_env_var}="):
                lines[i] = f"{service_env_var}={api_key}\n"
                key_exists = True
                break
        
        # If key doesn't exist, add it
        if not key_exists:
            lines.append(f"{service_env_var}={api_key}\n")
        
        # Write back to file
        with open(self.env_file_path, 'w') as file:
            file.writelines(lines)
        
        self.logger.info(f"API key for {service_name} saved to {self.env_file_path}")
        
        # Reload keys to include the new one
        self.load_keys()


# Global API key manager instance
_api_key_manager = None


def get_api_key_manager(env_file_path: Optional[str] = None) -> APIKeyManager:
    """Get the global API key manager instance"""
    global _api_key_manager
    if _api_key_manager is None:
        _api_key_manager = APIKeyManager(env_file_path)
    return _api_key_manager


def get_api_key(service_name: str) -> Optional[str]:
    """Get an API key for a specific service"""
    return get_api_key_manager().get_key(service_name)


def is_service_available(service_name: str) -> bool:
    """Check if a service is available (has a valid API key)"""
    return get_api_key_manager().is_service_available(service_name)


def validate_api_key(service_name: str) -> bool:
    """Validate if an API key for a service is present and appears valid"""
    return get_api_key_manager().validate_key(service_name)