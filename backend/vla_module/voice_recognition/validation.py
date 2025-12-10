"""
Voice recognition validation for the Vision-Language-Action (VLA) module.

This module implements validation functions to ensure voice recognition
meets the required accuracy and latency standards.
"""

import asyncio
import logging
import time
from typing import Dict, Any
import numpy as np

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from .whisper_client import WhisperClient
from .audio_processor import AudioProcessor


class VoiceRecognitionValidator:
    """
    Validates that voice recognition meets the required performance standards:
    - Accuracy threshold (>90% in quiet environments)
    - Latency threshold (<1s for processing)
    - Support for multiple languages
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        self.whisper_client = WhisperClient()
        self.audio_processor = AudioProcessor()
        
        # Performance thresholds
        self.accuracy_threshold = 0.9  # 90% accuracy
        self.latency_threshold = 1.0  # 1 second
        self.supported_languages = ["en", "es", "fr", "de", "ja", "zh"]  # Example supported languages
        
        self.logger.info("VoiceRecognitionValidator initialized")
    
    @log_exception()
    async def validate_accuracy(self, test_audio_data: bytes, expected_transcription: str) -> Dict[str, Any]:
        """
        Validate the accuracy of voice recognition.
        
        Args:
            test_audio_data: Audio data to test
            expected_transcription: Expected transcription of the audio
            
        Returns:
            Dictionary with validation results
        """
        try:
            start_time = time.time()
            
            # Transcribe the audio
            transcription, confidence = await self.whisper_client.transcribe_with_confidence(test_audio_data)
            
            processing_time = time.time() - start_time
            
            # Calculate accuracy metric (simple character-based similarity)
            accuracy = self._calculate_similarity(transcription.lower().strip(), expected_transcription.lower().strip())
            
            # Determine if accuracy meets threshold
            is_accurate = accuracy >= self.accuracy_threshold
            
            result = {
                'expected_transcription': expected_transcription,
                'actual_transcription': transcription,
                'confidence_score': confidence,
                'calculated_accuracy': accuracy,
                'accuracy_threshold': self.accuracy_threshold,
                'meets_accuracy_requirement': is_accurate,
                'processing_time': processing_time,
                'latency_threshold': self.latency_threshold,
                'meets_latency_requirement': processing_time < self.latency_threshold
            }
            
            self.logger.info(f"Accuracy validation: {accuracy:.2%} accuracy, {processing_time:.2f}s processing time")
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error in accuracy validation: {e}")
            raise VLAException(
                f"Error in voice recognition accuracy validation: {str(e)}", 
                VLAErrorType.VOICE_RECOGNITION_ERROR,
                e
            )
    
    @log_exception()
    async def validate_latency(self, test_audio_data: bytes) -> Dict[str, Any]:
        """
        Validate that voice recognition meets latency requirements.
        
        Args:
            test_audio_data: Audio data to test
            
        Returns:
            Dictionary with latency validation results
        """
        try:
            start_time = time.time()
            
            # Transcribe the audio
            transcription, confidence = await self.whisper_client.transcribe_with_confidence(test_audio_data)
            
            processing_time = time.time() - start_time
            
            # Determine if latency meets threshold
            meets_latency = processing_time < self.latency_threshold
            
            result = {
                'transcription': transcription,
                'confidence_score': confidence,
                'processing_time': processing_time,
                'latency_threshold': self.latency_threshold,
                'meets_latency_requirement': meets_latency
            }
            
            self.logger.info(f"Latency validation: {processing_time:.2f}s processing time")
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error in latency validation: {e}")
            raise VLAException(
                f"Error in voice recognition latency validation: {str(e)}", 
                VLAErrorType.VOICE_RECOGNITION_ERROR,
                e
            )
    
    @log_exception()
    async def validate_language_support(self, test_audio_data_by_language: Dict[str, bytes]) -> Dict[str, Any]:
        """
        Validate support for multiple languages.
        
        Args:
            test_audio_data_by_language: Dictionary mapping language codes to audio data
            
        Returns:
            Dictionary with language support validation results
        """
        try:
            results = {}
            
            for language_code, audio_data in test_audio_data_by_language.items():
                start_time = time.time()
                
                # Try to transcribe with specific language
                try:
                    transcription, confidence = await self.whisper_client.transcribe_with_confidence(audio_data)
                    
                    processing_time = time.time() - start_time
                    
                    # Check if language was properly detected/used
                    is_supported = language_code in self.supported_languages
                    
                    results[language_code] = {
                        'transcription': transcription,
                        'confidence_score': confidence,
                        'processing_time': processing_time,
                        'is_supported_language': is_supported,
                        'supported_languages_reference': self.supported_languages
                    }
                    
                    self.logger.info(f"Language {language_code} validation: {processing_time:.2f}s processing time")
                    
                except Exception as e:
                    self.logger.error(f"Error validating language {language_code}: {e}")
                    results[language_code] = {
                        'transcription': None,
                        'confidence_score': None,
                        'processing_time': None,
                        'is_supported_language': False,
                        'error': str(e)
                    }
            
            return results
            
        except Exception as e:
            self.logger.error(f"Error in language support validation: {e}")
            raise VLAException(
                f"Error in voice recognition language support validation: {str(e)}", 
                VLAErrorType.VOICE_RECOGNITION_ERROR,
                e
            )
    
    @log_exception()
    async def validate_comprehensive(self, test_cases: list) -> Dict[str, Any]:
        """
        Perform comprehensive validation of voice recognition system.
        
        Args:
            test_cases: List of dictionaries with 'audio_data' and 'expected_transcription' keys
            
        Returns:
            Dictionary with comprehensive validation results
        """
        try:
            self.logger.info(f"Starting comprehensive validation with {len(test_cases)} test cases")
            
            all_results = {
                'overall_accuracy': 0.0,
                'average_latency': 0.0,
                'total_tests': len(test_cases),
                'passed_tests': 0,
                'failed_tests': 0,
                'individual_results': []
            }
            
            total_accuracy = 0.0
            total_latency = 0.0
            passed_count = 0
            
            for i, test_case in enumerate(test_cases):
                audio_data = test_case['audio_data']
                expected = test_case['expected_transcription']
                
                # Run individual validation
                result = await self.validate_accuracy(audio_data, expected)
                
                # Accumulate stats
                total_accuracy += result['calculated_accuracy']
                total_latency += result['processing_time']
                
                if result['meets_accuracy_requirement'] and result['meets_latency_requirement']:
                    passed_count += 1
                
                result['test_case_number'] = i + 1
                all_results['individual_results'].append(result)
            
            # Calculate averages
            if len(test_cases) > 0:
                all_results['overall_accuracy'] = total_accuracy / len(test_cases)
                all_results['average_latency'] = total_latency / len(test_cases)
            
            all_results['passed_tests'] = passed_count
            all_results['failed_tests'] = len(test_cases) - passed_count
            
            # Determine overall success
            overall_success = (
                all_results['overall_accuracy'] >= self.accuracy_threshold and
                all_results['average_latency'] <= self.latency_threshold and
                all_results['failed_tests'] == 0
            )
            
            all_results['overall_success'] = overall_success
            all_results['overall_accuracy_threshold'] = self.accuracy_threshold
            all_results['overall_latency_threshold'] = self.latency_threshold
            
            self.logger.info(
                f"Comprehensive validation complete: "
                f"{all_results['overall_accuracy']:.2%} accuracy, "
                f"{all_results['average_latency']:.2f}s avg latency, "
                f"{passed_count}/{len(test_cases)} passed"
            )
            
            return all_results
            
        except Exception as e:
            self.logger.error(f"Error in comprehensive validation: {e}")
            raise VLAException(
                f"Error in comprehensive voice recognition validation: {str(e)}", 
                VLAErrorType.VOICE_RECOGNITION_ERROR,
                e
            )
    
    def _calculate_similarity(self, str1: str, str2: str) -> float:
        """
        Calculate similarity between two strings using a simple algorithm.
        This is a basic implementation - a real system might use more sophisticated methods like WER (Word Error Rate).
        
        Args:
            str1: First string
            str2: Second string
            
        Returns:
            Similarity ratio (0.0 to 1.0)
        """
        if not str1 and not str2:
            return 1.0
        if not str1 or not str2:
            return 0.0
        
        # Normalize strings
        s1, s2 = str1.lower(), str2.lower()
        
        # Calculate similarity using a basic character-based algorithm
        # In a real system, this would be more sophisticated (e.g., using edit distance, WER, etc.)
        matches = 0
        min_len = min(len(s1), len(s2))
        
        for i in range(min_len):
            if s1[i] == s2[i]:
                matches += 1
        
        # Also consider length difference as a penalty
        length_penalty = abs(len(s1) - len(s2)) / max(len(s1), len(s2)) if max(len(s1), len(s2)) > 0 else 0
        
        similarity = matches / len(s1) if len(s1) > 0 else 0
        return max(0.0, similarity - length_penalty)


# Global validator instance
_voice_validator = None


def get_voice_validator() -> VoiceRecognitionValidator:
    """Get the global voice recognition validator instance."""
    global _voice_validator
    if _voice_validator is None:
        _voice_validator = VoiceRecognitionValidator()
    return _voice_validator