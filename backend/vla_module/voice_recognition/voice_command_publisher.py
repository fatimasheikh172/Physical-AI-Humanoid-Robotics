"""
Voice command message publisher for the Vision-Language-Action (VLA) module.

This module handles publishing voice commands to the appropriate topics
and managing the message format for the VLA system.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from typing import Optional
import logging

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import VoiceCommand
from ..core.data_models import VoiceCommandModel


class VoiceCommandPublisher(Node):
    """
    ROS 2 publisher for voice commands in the VLA system.
    """
    
    def __init__(self, node_name: str = "voice_command_publisher"):
        super().__init__(node_name)
        
        self.config = get_config()
        self.logger = setup_logger(node_name)
        
        # Set up publisher
        self.setup_publisher()
        
        self.get_logger().info(f"{node_name} initialized successfully")
    
    def setup_publisher(self):
        """Set up the ROS 2 publisher for voice commands."""
        self.publisher = self.create_publisher(
            String,
            '/vla/voice_commands',
            qos_profile=qos_profile_sensor_data
        )
        
        self.get_logger().info("Voice command publisher set up on /vla/voice_commands")
    
    @log_exception()
    def publish_voice_command_raw(self, command_text: str, command_id: Optional[str] = None):
        """
        Publish a raw voice command string.
        
        Args:
            command_text: The transcribed command text
            command_id: Optional command ID (auto-generated if not provided)
        """
        if not command_text.strip():
            raise VLAException(
                "Command text cannot be empty",
                VLAErrorType.VALIDATION_ERROR
            )
        
        # Create message ID if not provided
        msg_id = command_id or f"cmd_{rclpy.clock.Clock().now().nanoseconds}"
        
        # Format the message (in a real system, we'd use a proper message type)
        message_content = f"{msg_id}|{command_text}"
        
        # Create and publish the message
        msg = String()
        msg.data = message_content
        self.publisher.publish(msg)
        
        self.get_logger().info(f"Published voice command: {msg_id[:8]}... - '{command_text[:50]}{'...' if len(command_text) > 50 else ''}'")
    
    @log_exception()
    def publish_voice_command_model(self, voice_command: VoiceCommandModel):
        """
        Publish a VoiceCommandModel instance.
        
        Args:
            voice_command: VoiceCommandModel to publish
        """
        if not voice_command.transcript.strip():
            raise VLAException(
                "Voice command transcript cannot be empty",
                VLAErrorType.VALIDATION_ERROR
            )
        
        # Convert the VoiceCommandModel to the message format
        message_content = f"{voice_command.command_id}|{voice_command.transcript}|{voice_command.user_id}|{voice_command.confidence}"
        
        # Create and publish the message
        msg = String()
        msg.data = message_content
        self.publisher.publish(msg)
        
        self.get_logger().info(f"Published voice command: {voice_command.command_id} - '{voice_command.transcript[:50]}{'...' if len(voice_command.transcript) > 50 else ''}'")
    
    @log_exception()
    def publish_voice_command_message_types(self, voice_command: VoiceCommand):
        """
        Publish a voice command using the message_types format.
        
        Args:
            voice_command: VoiceCommand to publish
        """
        if not voice_command.transcript.strip():
            raise VLAException(
                "Voice command transcript cannot be empty",
                VLAErrorType.VALIDATION_ERROR
            )
        
        # Format the message content
        message_content = f"{voice_command.command_id}|{voice_command.transcript}|{voice_command.user_id}|{voice_command.confidence}"
        
        # Create and publish the message
        msg = String()
        msg.data = message_content
        self.publisher.publish(msg)
        
        self.get_logger().info(f"Published voice command: {voice_command.command_id} - '{voice_command.transcript[:50]}{'...' if len(voice_command.transcript) > 50 else ''}'")


# Global publisher instance
_voice_command_publisher = None


def get_voice_command_publisher(node_name: str = "voice_command_publisher") -> VoiceCommandPublisher:
    """
    Get the global voice command publisher instance.
    
    Args:
        node_name: Name for the publisher node
        
    Returns:
        VoiceCommandPublisher instance
    """
    global _voice_command_publisher
    if _voice_command_publisher is None:
        # Initialize within an rclpy context if available
        if not rclpy.ok():
            rclpy.init()
        _voice_command_publisher = VoiceCommandPublisher(node_name)
    return _voice_command_publisher


def publish_voice_command(command_text: str, command_id: Optional[str] = None):
    """
    Convenience function to publish a voice command.
    
    Args:
        command_text: The transcribed command text
        command_id: Optional command ID
    """
    publisher = get_voice_command_publisher()
    publisher.publish_voice_command_raw(command_text, command_id)