"""
Audio stream handling for real-time processing in the Vision-Language-Action (VLA) module.

This module handles the real-time processing of audio streams for voice recognition.
"""

import asyncio
import logging
import threading
import queue
from typing import Optional, Callable, Any
from dataclasses import dataclass
import time
import numpy as np

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger, AsyncRateLimiter
from ..core.message_types import VoiceCommand
from ..core.data_models import VoiceCommandModel
from .whisper_client import WhisperClient, get_whisper_client
from .audio_processor import AudioProcessor, get_audio_processor
from .voice_command_publisher import publish_voice_command


@dataclass
class AudioStreamBuffer:
    """
    Buffer for audio stream chunks.
    """
    chunks: list
    sample_rate: int
    channels: int
    dtype: str
    start_time: float
    end_time: float
    
    def __post_init__(self):
        self.duration = self.end_time - self.start_time


class AudioStreamHandler:
    """
    Handles real-time audio stream processing for voice recognition.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        self.whisper_client = get_whisper_client()
        self.audio_processor = get_audio_processor()
        
        # Audio processing queue
        self.audio_queue = queue.Queue(maxsize=10)  # Limit queue size to prevent memory buildup
        
        # Processing state
        self.is_processing = False
        self.is_streaming = False
        self.stream_buffer = None
        
        # Rate limiter to prevent overloading the Whisper API
        self.rate_limiter = AsyncRateLimiter(
            max_calls=self.config.max_command_history,  # Limit based on configuration
            time_window=60.0  # Per minute
        )
        
        # Wake word detection (simplified)
        self.wake_words = ["hey robot", "hello robot", "robot"]  # Configurable wake words
        
        self.logger.info("AudioStreamHandler initialized")
    
    def start_streaming(self):
        """
        Start accepting audio stream chunks.
        """
        self.is_streaming = True
        self.logger.info("Audio streaming started")
    
    def stop_streaming(self):
        """
        Stop accepting audio stream chunks.
        """
        self.is_streaming = False
        self.logger.info("Audio streaming stopped")
    
    @log_exception()
    def add_audio_chunk(self, chunk: bytes, sample_rate: int = 16000):
        """
        Add an audio chunk to the processing queue.
        
        Args:
            chunk: Audio chunk as bytes
            sample_rate: Sample rate of the audio chunk
        """
        if not self.is_streaming:
            self.logger.warning("Received audio chunk while not streaming, ignoring")
            return
        
        try:
            # Add to queue with timeout to prevent blocking
            self.audio_queue.put_nowait({
                'chunk': chunk,
                'sample_rate': sample_rate,
                'timestamp': time.time()
            })
            self.logger.debug(f"Added audio chunk to queue, queue size: {self.audio_queue.qsize()}")
        except queue.Full:
            self.logger.warning("Audio queue is full, dropping chunk")
    
    async def process_audio_stream(self):
        """
        Process the audio stream asynchronously, handling chunks as they arrive.
        """
        self.logger.info("Starting audio stream processing")
        self.is_processing = True
        
        while self.is_streaming or not self.audio_queue.empty():
            try:
                # Get an audio chunk from the queue (with timeout)
                try:
                    chunk_data = self.audio_queue.get(timeout=1.0)
                    chunk = chunk_data['chunk']
                    sample_rate = chunk_data['sample_rate']
                    timestamp = chunk_data['timestamp']
                    
                    self.logger.debug(f"Processing audio chunk of size {len(chunk)}")
                    
                    # Process the audio chunk
                    await self._handle_single_chunk(chunk, sample_rate, timestamp)
                    
                    # Mark the chunk as processed
                    self.audio_queue.task_done()
                    
                except queue.Empty:
                    # No chunk available, continue the loop
                    continue
                    
            except Exception as e:
                self.logger.error(f"Error processing audio chunk: {e}")
        
        self.is_processing = False
        self.logger.info("Audio stream processing ended")
    
    async def _handle_single_chunk(self, chunk: bytes, sample_rate: int, timestamp: float):
        """
        Handle a single audio chunk, including preprocessing, wake word detection, and transcription.
        
        Args:
            chunk: Audio chunk as bytes
            sample_rate: Sample rate of the chunk
            timestamp: Time when the chunk was received
        """
        try:
            # Apply audio preprocessing (noise reduction, format conversion)
            preprocessed_chunks = self.audio_processor.preprocess_audio(
                chunk,
                apply_noise_reduction=True,
                detect_voice_activity=True
            )
            
            # Process each voice segment
            for segment in preprocessed_chunks:
                # Check if the segment contains a wake word or other trigger
                if self._check_trigger_word(segment, sample_rate):
                    self.logger.info("Trigger word detected, preparing for voice command processing")
                    
                    # Since this is a real-time system, we need to buffer enough audio 
                    # to form a complete command. For simplicity, we'll process immediately.
                    await self._process_voice_command(segment)
                else:
                    # If no trigger word is detected, we might want to buffer it for later
                    # For now, just check if there's enough audio content to warrant processing
                    if len(segment) > 16000 * 0.5:  # More than 0.5 seconds of audio at 16kHz
                        transcription, confidence = await self.whisper_client.transcribe_with_confidence(segment)
                        
                        # Only process if we have a reasonably confident transcription
                        if confidence > 0.7 and len(transcription.strip()) > 3:
                            self.logger.info(f"Non-trigger phrase detected: '{transcription}', confidence: {confidence:.2f}")
                            # Optionally, we could check if this might be a command despite lack of wake word
                            if self._might_be_command(transcription):
                                self.logger.info("Processing potential command without wake word")
                                await self._process_voice_command(segment)
        
        except Exception as e:
            self.logger.error(f"Error handling audio chunk: {e}")
            raise VLAException(
                f"Error handling audio chunk: {str(e)}", 
                VLAErrorType.VOICE_RECOGNITION_ERROR,
                e
            )
    
    def _check_trigger_word(self, audio_data: bytes, sample_rate: int) -> bool:
        """
        Check if the audio contains a wake word or trigger phrase.
        This is a simplified implementation - a real system would use 
        keyword spotting models for better accuracy.
        
        Args:
            audio_data: Audio data as bytes
            sample_rate: Sample rate of the audio
            
        Returns:
            True if a trigger word is detected, False otherwise
        """
        # For now, we'll skip actual wake word detection and just return True to process everything
        # In a real implementation, this would use a keyword spotting model like Picovoice Porcupine
        # or a simple energy-based detection followed by wake word transcription.
        
        # For demonstration purposes, we'll say that if the audio has enough data,
        # there might be a command in it
        return len(audio_data) > 1600  # 100ms at 16kHz, 16-bit
    
    def _might_be_command(self, transcription: str) -> bool:
        """
        Determine if a transcription might be a command even without a trigger word.
        This could be based on command-like phrasing or intent.
        
        Args:
            transcription: The transcribed text
            
        Returns:
            True if the transcription might be a command, False otherwise
        """
        # Simple heuristic: check if transcription starts with common command prefixes
        transcription_lower = transcription.lower().strip()
        
        command_prefixes = [
            "move", "go", "navigate", "turn", "rotate", 
            "pick", "grab", "take", "lift", "place", 
            "find", "look", "see", "detect", "identify",
            "stop", "pause", "start", "resume"
        ]
        
        for prefix in command_prefixes:
            if transcription_lower.startswith(prefix):
                return True
        
        return False
    
    async def _process_voice_command(self, audio_segment: bytes):
        """
        Process a voice command from an audio segment.
        
        Args:
            audio_segment: Audio segment that contains the command
        """
        try:
            # Acquire rate limit permission before calling Whisper API
            await self.rate_limiter.acquire()
            
            # Transcribe the audio segment
            transcription, confidence = await self.whisper_client.transcribe_with_confidence(audio_segment)
            
            if confidence < 0.5:  # Skip if confidence is too low
                self.logger.info(f"Skipping transcription due to low confidence: {confidence:.2f}")
                return
            
            # Create a voice command and publish it
            voice_command = VoiceCommandModel.create(
                transcript=transcription,
                user_id="streaming_user",  # Could be determined from context in a real system
                language=self.config.whisper_language
            )
            voice_command.confidence = confidence
            
            self.logger.info(f"Processed voice command: '{transcription[:50]}{'...' if len(transcription) > 50 else ''}' (confidence: {confidence:.2f})")
            
            # Publish the voice command (in a real system, this would be handled by the VLA manager)
            publish_voice_command(transcription, voice_command.command_id)
            
        except Exception as e:
            self.logger.error(f"Error processing voice command: {e}")
            raise VLAException(
                f"Error processing voice command: {str(e)}", 
                VLAErrorType.VOICE_RECOGNITION_ERROR,
                e
            )
    
    async def start_processing_loop(self):
        """
        Start the continuous processing loop in the background.
        """
        # Run the processing in the background
        asyncio.create_task(self.process_audio_stream())


# Global audio stream handler instance
_audio_stream_handler = None


def get_audio_stream_handler() -> AudioStreamHandler:
    """
    Get the global audio stream handler instance.
    
    Returns:
        AudioStreamHandler instance
    """
    global _audio_stream_handler
    if _audio_stream_handler is None:
        _audio_stream_handler = AudioStreamHandler()
    return _audio_stream_handler


def add_audio_stream_chunk(chunk: bytes, sample_rate: int = 16000):
    """
    Convenience function to add an audio chunk to the stream handler.
    
    Args:
        chunk: Audio chunk as bytes
        sample_rate: Sample rate of the audio chunk
    """
    handler = get_audio_stream_handler()
    handler.add_audio_chunk(chunk, sample_rate)


async def start_voice_streaming():
    """
    Start the voice streaming and processing loop.
    """
    handler = get_audio_stream_handler()
    handler.start_streaming()
    await handler.start_processing_loop()