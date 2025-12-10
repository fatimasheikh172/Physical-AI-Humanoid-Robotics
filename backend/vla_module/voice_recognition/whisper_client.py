"""
OpenAI Whisper client for the Vision-Language-Action (VLA) module.

This module handles voice recognition using OpenAI's Whisper API.
"""

import asyncio
import logging
from typing import Optional, Union
import aiohttp
import io
from openai import AsyncOpenAI

from ..core.config import get_config
from ..core.api_key_manager import get_api_key, is_service_available
from ..core.error_handling import VLAException, VLAErrorType, log_exception
from ..core.message_types import VoiceCommand
from ..core.data_models import VoiceCommandModel


class WhisperClient:
    """
    Client for interacting with OpenAI's Whisper API for voice recognition.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = logging.getLogger(self.__class__.__name__)
        
        # Check if OpenAI service is available
        if not is_service_available("openai"):
            raise VLAException(
                "OpenAI API key not available for Whisper client", 
                VLAErrorType.CONFIGURATION_ERROR
            )
        
        # Initialize the OpenAI client
        api_key = get_api_key("openai")
        self.client = AsyncOpenAI(api_key=api_key)
        
        self.logger.info("WhisperClient initialized")
    
    @log_exception()
    async def transcribe_audio(
        self, 
        audio_data: Union[bytes, str], 
        language: Optional[str] = None,
        response_format: str = "text"
    ) -> str:
        """
        Transcribe audio data using OpenAI's Whisper API.
        
        Args:
            audio_data: Audio data as bytes or path to audio file
            language: Language of the audio (e.g., 'en', 'es')
            response_format: Format of the response ('text', 'json', etc.)
            
        Returns:
            Transcribed text
        """
        try:
            # Determine if audio_data is a file path or raw bytes
            if isinstance(audio_data, str):
                # It's a file path
                with open(audio_data, 'rb') as audio_file:
                    file_content = audio_file.read()
                    file_like = io.BytesIO(file_content)
                    file_like.name = audio_data.split('/')[-1]  # Set a name for the file
            else:
                # It's raw bytes
                file_like = io.BytesIO(audio_data)
                file_like.name = "audio.wav"  # Set a default name
            
            # Prepare parameters for the API call
            params = {
                "model": self.config.whisper_model,
                "file": file_like,
                "response_format": response_format,
            }
            
            if language:
                params["language"] = language
            elif self.config.whisper_language:
                params["language"] = self.config.whisper_language
            
            # Make the API call
            start_time = asyncio.get_event_loop().time()
            transcription = await self.client.audio.transcriptions.create(**params)
            end_time = asyncio.get_event_loop().time()
            
            self.logger.info(f"Transcription completed in {end_time - start_time:.2f}s")
            
            # Return the transcribed text
            if response_format == "text":
                return transcription
            else:
                # If response_format is not text, assume it has a text field
                return transcription.text if hasattr(transcription, 'text') else str(transcription)
                
        except Exception as e:
            self.logger.error(f"Error transcribing audio: {e}")
            raise VLAException(
                f"Error in Whisper transcription: {str(e)}", 
                VLAErrorType.VOICE_RECOGNITION_ERROR,
                e
            )
    
    @log_exception()
    async def transcribe_with_confidence(self, audio_data: Union[bytes, str]) -> tuple[str, float]:
        """
        Transcribe audio and return both text and a confidence score.
        Note: Whisper API doesn't directly provide confidence scores,
        so we'll return a default high confidence for successful transcriptions.
        
        Args:
            audio_data: Audio data as bytes or path to audio file
            
        Returns:
            Tuple of (transcribed_text, confidence_score)
        """
        transcription = await self.transcribe_audio(audio_data)
        
        # For now, return a default confidence since Whisper doesn't provide it
        # In a real implementation, you might use a different model or approach
        # to estimate confidence
        confidence = 0.9  # Default to high confidence for successful transcriptions
        
        return transcription, confidence
    
    @log_exception()
    async def create_voice_command(
        self, 
        audio_data: Union[bytes, str], 
        user_id: str,
        session_id: Optional[str] = None
    ) -> VoiceCommandModel:
        """
        Create a VoiceCommandModel from audio data.
        
        Args:
            audio_data: Audio data as bytes or path to audio file
            user_id: ID of the user issuing the command
            session_id: Optional session identifier
            
        Returns:
            VoiceCommandModel instance
        """
        try:
            transcription, confidence = await self.transcribe_with_confidence(audio_data)
            
            # Create and return the VoiceCommandModel
            voice_command = VoiceCommandModel.create(
                transcript=transcription,
                user_id=user_id,
                language=self.config.whisper_language
            )
            
            # Set additional fields if available
            voice_command.session_id = session_id
            voice_command.confidence = confidence
            voice_command.audio_data = audio_data if isinstance(audio_data, bytes) else None
            
            return voice_command
            
        except Exception as e:
            self.logger.error(f"Error creating voice command: {e}")
            raise VLAException(
                f"Error creating voice command: {str(e)}", 
                VLAErrorType.VOICE_RECOGNITION_ERROR,
                e
            )


# Global Whisper client instance
_whisper_client = None


def get_whisper_client() -> WhisperClient:
    """Get the global Whisper client instance."""
    global _whisper_client
    if _whisper_client is None:
        _whisper_client = WhisperClient()
    return _whisper_client