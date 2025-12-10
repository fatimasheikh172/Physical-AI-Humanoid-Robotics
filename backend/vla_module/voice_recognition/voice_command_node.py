"""
Voice command ROS 2 node for the Vision-Language-Action (VLA) module.

This module implements a ROS 2 node that handles voice commands,
integrating audio preprocessing, Whisper transcription, and command routing.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String, Header
from audio_common_msgs.msg import AudioData  # Note: This message type might need to be defined
from sensor_msgs.msg import Image  # For potential audio visualization

import asyncio
import logging
from typing import Optional, List
import threading
import time

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger, time_it
from ..core.vla_manager import get_vla_manager
from ..core.message_types import VoiceCommand
from ..core.data_models import VoiceCommandModel
from .whisper_client import WhisperClient, get_whisper_client
from .audio_processor import AudioProcessor, get_audio_processor


class VoiceCommandNode(Node):
    """
    ROS 2 node for processing voice commands.
    """
    
    def __init__(self, node_name: str = "voice_command_node"):
        super().__init__(node_name)
        
        self.config = get_config()
        self.logger = setup_logger(node_name)
        
        # Initialize components
        self.whisper_client = get_whisper_client()
        self.audio_processor = get_audio_processor()
        self.vla_manager = None  # Will be set by the manager
        
        # Set up ROS 2 infrastructure
        self.setup_publishers_subscribers()
        
        # Set up internal state
        self.is_listening = False
        self.command_history = []
        
        # Set up async execution
        self._async_loop = None
        self._async_thread = None
        self._setup_async_loop()
        
        self.get_logger().info(f"{node_name} initialized successfully")
    
    def setup_publishers_subscribers(self):
        """Set up ROS 2 publishers and subscribers."""
        # Publishers
        self.command_publisher = self.create_publisher(
            String,
            '/vla/voice_commands',
            qos_profile=qos_profile_sensor_data
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/vla/voice_status',
            qos_profile=qos_profile_sensor_data
        )
        
        # Subscribers
        # Note: AudioData might not be a standard ROS 2 message, 
        # we might need to define our own or use a different approach
        self.audio_subscriber = self.create_subscription(
            String,  # Using String as placeholder until we define proper audio message type
            '/audio/input',
            self.audio_callback,
            qos_profile=qos_profile_sensor_data
        )
        
        # Service server for manual command processing
        self.process_command_service = self.create_service(
            String,
            '/vla/process_voice_command',
            self.process_command_callback
        )
    
    def _setup_async_loop(self):
        """Set up the asyncio event loop in a separate thread."""
        self._async_loop = asyncio.new_event_loop()
        
        def run_loop():
            asyncio.set_event_loop(self._async_loop)
            self._async_loop.run_forever()
        
        self._async_thread = threading.Thread(target=run_loop, daemon=True)
        self._async_thread.start()
    
    def _teardown_async_loop(self):
        """Tear down the asyncio event loop."""
        if self._async_loop:
            self._async_loop.call_soon_threadsafe(self._async_loop.stop)
            if self._async_thread:
                self._async_thread.join()
    
    @log_exception()
    def audio_callback(self, msg):
        """
        Callback for audio input.
        
        Args:
            msg: Audio message (currently String as placeholder)
        """
        self.get_logger().info("Audio received, processing...")
        
        # In a real implementation, this would receive actual audio data
        # For now, we'll treat the string data as a file path or raw audio bytes
        try:
            # Process the audio in the async loop
            future = asyncio.run_coroutine_threadsafe(
                self.process_audio_async(msg.data.encode() if isinstance(msg.data, str) else msg.data),
                self._async_loop
            )
            # We don't await here to keep the callback non-blocking
            
            # Publish status
            status_msg = String()
            status_msg.data = "PROCESSING"
            self.status_publisher.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in audio callback: {e}")
            status_msg = String()
            status_msg.data = f"ERROR: {str(e)}"
            self.status_publisher.publish(status_msg)
    
    @log_exception()
    def process_command_callback(self, request, response):
        """
        Service callback for manual command processing.
        
        Args:
            request: Request message containing the command
            response: Response message to fill in
            
        Returns:
            Response message
        """
        try:
            # For now, just return a placeholder response
            response.data = f"Command processed: {request.data}"
            self.get_logger().info(f"Manual command processed: {request.data}")
        except Exception as e:
            self.get_logger().error(f"Error in process command callback: {e}")
            response.data = f"Error: {str(e)}"
        
        return response
    
    async def process_audio_async(self, audio_data: bytes) -> Optional[VoiceCommandModel]:
        """
        Asynchronously process audio data through the full pipeline.
        
        Args:
            audio_data: Raw audio data as bytes
            
        Returns:
            VoiceCommandModel if successful, None otherwise
        """
        start_time = time.time()
        
        try:
            self.get_logger().info("Starting audio preprocessing...")
            
            # Preprocess the audio (noise reduction, format conversion, etc.)
            preprocessed_audios = self.audio_processor.preprocess_audio(
                audio_data, 
                apply_noise_reduction=True,
                detect_voice_activity=True
            )
            
            self.get_logger().info(f"Preprocessing complete, got {len(preprocessed_audios)} audio segments")
            
            # Process each voice segment
            for i, segment in enumerate(preprocessed_audios):
                self.get_logger().info(f"Processing voice segment {i+1}/{len(preprocessed_audios)}")
                
                # Transcribe the audio segment
                transcription, confidence = await self.whisper_client.transcribe_with_confidence(segment)
                
                self.get_logger().info(f"Transcription: {transcription} (confidence: {confidence:.2f})")
                
                # Create a voice command model
                voice_command = VoiceCommandModel.create(
                    transcript=transcription,
                    user_id="default_user",  # Should come from context
                    language=self.config.whisper_language
                )
                voice_command.confidence = confidence
                
                # Add to history
                self.command_history.append(voice_command)
                
                # Publish the command
                await self.publish_voice_command(voice_command)
                
                # If we have a VLA manager, send the command to it
                if self.vla_manager:
                    try:
                        # Convert to message type for manager
                        message_types_cmd = voice_command.to_message_types()
                        await self.vla_manager.process_text_command(
                            message_types_cmd.transcript,
                            message_types_cmd.user_id
                        )
                    except Exception as e:
                        self.get_logger().error(f"Error sending command to VLA manager: {e}")
                else:
                    # Use the global VLA manager if available
                    try:
                        from ..core.vla_manager import get_vla_manager
                        global_vla_manager = get_vla_manager()
                        message_types_cmd = voice_command.to_message_types()
                        await global_vla_manager.process_text_command(
                            message_types_cmd.transcript,
                            message_types_cmd.user_id
                        )
                    except Exception as e:
                        self.get_logger().error(f"Error sending command to global VLA manager: {e}")
                
                # For now, just process the first segment
                # In a real implementation, you might want to combine multiple segments
                # or handle multiple commands
                break
            
            end_time = time.time()
            self.get_logger().info(f"Audio processing completed in {end_time - start_time:.2f}s")
            
            # Publish completion status
            status_msg = String()
            status_msg.data = "COMPLETED"
            self.status_publisher.publish(status_msg)
            
            return voice_command if 'voice_command' in locals() else None
            
        except Exception as e:
            self.get_logger().error(f"Error in async audio processing: {e}")
            
            # Publish error status
            status_msg = String()
            status_msg.data = f"ERROR: {str(e)}"
            self.status_publisher.publish(status_msg)
            
            raise VLAException(
                f"Error in audio processing: {str(e)}", 
                VLAErrorType.VOICE_RECOGNITION_ERROR,
                e
            )
    
    async def publish_voice_command(self, voice_command: VoiceCommandModel):
        """
        Publish a voice command to the appropriate ROS topic.
        
        Args:
            voice_command: VoiceCommandModel to publish
        """
        # Convert to message type
        cmd_msg = String()
        cmd_msg.data = f"{voice_command.command_id}:{voice_command.transcript}"
        
        # Publish
        self.command_publisher.publish(cmd_msg)
        self.get_logger().info(f"Published voice command: {voice_command.command_id}")
    
    def set_vla_manager(self, vla_manager: 'VLAManager'):
        """
        Set the VLA manager for this node to communicate with.
        
        Args:
            vla_manager: VLAManager instance
        """
        self.vla_manager = vla_manager
        self.get_logger().info("VLA Manager registered with VoiceCommandNode")
    
    def start_listening(self):
        """Start listening for audio input."""
        self.is_listening = True
        self.get_logger().info("VoiceCommandNode started listening")
    
    def stop_listening(self):
        """Stop listening for audio input."""
        self.is_listening = False
        self.get_logger().info("VoiceCommandNode stopped listening")
    
    def get_command_history(self) -> List[VoiceCommandModel]:
        """Get the command history."""
        return self.command_history
    
    def destroy_node(self):
        """Clean up when the node is destroyed."""
        self.stop_listening()
        self._teardown_async_loop()
        super().destroy_node()


def main(args=None):
    """Main function to run the VoiceCommandNode."""
    rclpy.init(args=args)
    
    node = VoiceCommandNode()
    node.start_listening()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()