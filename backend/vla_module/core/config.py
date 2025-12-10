"""
Configuration management for the Vision-Language-Action (VLA) module.

This module handles loading and managing configuration parameters
for the VLA system from various sources.
"""

import os
import yaml
from typing import Dict, Any, Optional
from dataclasses import dataclass, asdict
from dotenv import load_dotenv
import logging

from .api_key_manager import get_api_key


# Load environment variables from .env file
load_dotenv()


@dataclass
class VLAConfig:
    """Configuration class that holds all VLA module settings"""
    # API Configuration
    openai_api_key: str = "your_openai_api_key_here"  # Set dynamically
    anthropic_api_key: str = "your_anthropic_api_key_here"  # Set dynamically

    # Whisper Configuration
    whisper_model: str = os.getenv("WHISPER_MODEL", "whisper-1")
    whisper_language: str = os.getenv("WHISPER_LANGUAGE", "en")

    # LLM Configuration
    llm_provider: str = os.getenv("LLM_PROVIDER", "openai")  # openai, anthropic, local
    llm_model: str = os.getenv("LLM_MODEL", "gpt-4-turbo")
    llm_temperature: float = float(os.getenv("LLM_TEMPERATURE", "0.3"))

    # Robot Configuration
    robot_type: str = os.getenv("ROBOT_TYPE", "default_humanoid")
    robot_capabilities: list = None  # Will be set in __post_init__

    # Performance Configuration
    voice_recognition_timeout: float = float(os.getenv("VOICE_RECOGNITION_TIMEOUT", "5.0"))
    planning_timeout: float = float(os.getenv("PLANNING_TIMEOUT", "10.0"))
    action_execution_timeout: float = float(os.getenv("ACTION_EXECUTION_TIMEOUT", "30.0"))
    max_command_history: int = int(os.getenv("MAX_COMMAND_HISTORY", "50"))

    # Safety Configuration
    max_navigation_distance: float = float(os.getenv("MAX_NAVIGATION_DISTANCE", "10.0"))
    safety_check_interval: float = float(os.getenv("SAFETY_CHECK_INTERVAL", "0.5"))
    allowed_action_types: list = None  # Will be set in __post_init__

    # Vision Configuration
    object_detection_threshold: float = float(os.getenv("OBJECT_DETECTION_THRESHOLD", "0.7"))
    vision_processing_fps: int = int(os.getenv("VISION_PROCESSING_FPS", "10"))

    def __post_init__(self):
        if self.robot_capabilities is None:
            self.robot_capabilities = [
                "navigation",
                "manipulation",
                "object_detection",
                "grasping"
            ]
        if self.allowed_action_types is None:
            self.allowed_action_types = [
                "move_to",
                "pick_up",
                "place",
                "detect_object",
                "grasp",
                "release"
            ]

        # Set API keys dynamically from environment or key manager
        if self.openai_api_key == "your_openai_api_key_here":
            self.openai_api_key = get_api_key("openai") or os.getenv("OPENAI_API_KEY", "your_openai_api_key_here")
        if self.anthropic_api_key == "your_anthropic_api_key_here":
            self.anthropic_api_key = get_api_key("anthropic") or os.getenv("ANTHROPIC_API_KEY", "your_anthropic_api_key_here")


class ConfigManager:
    """Manages configuration for the VLA module"""
    
    def __init__(self, config_path: Optional[str] = None):
        self.config_path = config_path
        self.config = self.load_config()
        
    def load_config(self) -> VLAConfig:
        """Load configuration from YAML file, environment variables, or defaults"""
        config_dict = {}
        
        # Load from YAML file if provided
        if self.config_path and os.path.exists(self.config_path):
            try:
                with open(self.config_path, 'r') as file:
                    yaml_config = yaml.safe_load(file)
                    if yaml_config:
                        config_dict.update(yaml_config)
            except Exception as e:
                logging.warning(f"Failed to load config from {self.config_path}: {e}")
        
        # Override with environment variables
        env_overrides = {}
        for key in [field.name for field in VLAConfig.__dataclass_fields__.values()]:
            env_key = key.upper()
            env_value = os.getenv(env_key)
            if env_value is not None:
                # Convert environment variable to the appropriate type based on the field
                try:
                    # Get the field type from the dataclass default
                    field_type = VLAConfig.__dataclass_fields__[key].type
                    if field_type == str:
                        env_overrides[key] = env_value
                    elif field_type == int:
                        env_overrides[key] = int(env_value)
                    elif field_type == float:
                        env_overrides[key] = float(env_value)
                    elif field_type == bool:
                        env_overrides[key] = env_value.lower() in ['true', '1', 'yes', 'on']
                    elif field_type == list or field_type == list or field_type == Dict[str, Any]:
                        # For complex types, assume it's a JSON string
                        import json
                        env_overrides[key] = json.loads(env_value)
                except (ValueError, TypeError, AttributeError):
                    # If conversion fails, keep as string
                    env_overrides[key] = env_value
        
        config_dict.update(env_overrides)
        
        # Create and return VLAConfig instance
        return VLAConfig(**config_dict)
    
    def get_config(self) -> VLAConfig:
        """Return the current configuration"""
        return self.config
    
    def update_config(self, **kwargs) -> VLAConfig:
        """Update the configuration with new values"""
        for key, value in kwargs.items():
            if hasattr(self.config, key):
                setattr(self.config, key, value)
        return self.config
    
    def save_config(self, path: str) -> None:
        """Save the current configuration to a YAML file"""
        config_dict = asdict(self.config)
        with open(path, 'w') as file:
            yaml.dump(config_dict, file, default_flow_style=False)
        
        logging.info(f"Configuration saved to {path}")


# Global configuration manager instance
_config_manager = None


def get_config_manager(config_path: Optional[str] = None) -> ConfigManager:
    """Get the global configuration manager instance"""
    global _config_manager
    if _config_manager is None:
        _config_manager = ConfigManager(config_path)
    return _config_manager


def get_config() -> VLAConfig:
    """Get the current configuration"""
    return get_config_manager().get_config()


def init_config(config_path: Optional[str] = None) -> ConfigManager:
    """Initialize the configuration manager"""
    global _config_manager
    _config_manager = ConfigManager(config_path)
    return _config_manager