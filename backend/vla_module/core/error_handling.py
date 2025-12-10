"""
Logging and error handling infrastructure for the Vision-Language-Action (VLA) module.
"""

import logging
import sys
import traceback
from typing import Dict, Any, Optional
from enum import Enum
import json
import time
from functools import wraps
import asyncio


class VLAErrorType(Enum):
    """Enumeration of different types of errors in the VLA module"""
    VOICE_RECOGNITION_ERROR = "voice_recognition_error"
    LLM_ERROR = "llm_error"
    ACTION_EXECUTION_ERROR = "action_execution_error"
    VISION_ERROR = "vision_error"
    PLANNING_ERROR = "planning_error"
    SAFETY_ERROR = "safety_error"
    CONFIGURATION_ERROR = "configuration_error"
    CONNECTION_ERROR = "connection_error"
    VALIDATION_ERROR = "validation_error"
    UNKNOWN_ERROR = "unknown_error"


class VLAException(Exception):
    """Base exception class for the VLA module"""
    
    def __init__(self, message: str, error_type: VLAErrorType = VLAErrorType.UNKNOWN_ERROR, 
                 original_exception: Optional[Exception] = None):
        super().__init__(message)
        self.message = message
        self.error_type = error_type
        self.original_exception = original_exception
        self.timestamp = time.time()
        self.error_id = f"err_{int(self.timestamp)}_{hash(message) % 10000:04d}"
        
        # Log the error
        logging.error(f"[{self.error_id}] {error_type.value}: {message}")
        if original_exception:
            logging.error(f"[{self.error_id}] Original exception: {original_exception}")
            logging.error(f"[{self.error_id}] Traceback: {traceback.format_tb(original_exception.__traceback__)}")


class SafetyError(VLAException):
    """Exception raised for safety-related issues"""
    def __init__(self, message: str, original_exception: Optional[Exception] = None):
        super().__init__(message, VLAErrorType.SAFETY_ERROR, original_exception)


class ValidationError(VLAException):
    """Exception raised for validation issues"""
    def __init__(self, message: str, original_exception: Optional[Exception] = None):
        super().__init__(message, VLAErrorType.VALIDATION_ERROR, original_exception)


def setup_logging(level: int = logging.INFO, log_file: Optional[str] = None):
    """
    Set up logging for the VLA module.
    
    Args:
        level: Logging level (default: INFO)
        log_file: Optional file to write logs to
    """
    # Create a custom formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Configure the root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(level)
    
    # Clear any existing handlers
    root_logger.handlers.clear()
    
    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)
    console_handler.setFormatter(formatter)
    root_logger.addHandler(console_handler)
    
    # File handler if specified
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        root_logger.addHandler(file_handler)
    
    logging.info("VLA module logging configured")


def log_exception(level: int = logging.ERROR):
    """
    Decorator to log exceptions with additional context.
    
    Args:
        level: Logging level for the exception (default: ERROR)
        
    Returns:
        Decorated function
    """
    def decorator(func):
        @wraps(func)
        async def async_wrapper(*args, **kwargs):
            try:
                return await func(*args, **kwargs)
            except VLAException:
                # Re-raise VLA-specific exceptions without logging them again
                raise
            except Exception as e:
                logging.log(level, f"Exception in {func.__name__}: {str(e)}")
                logging.log(level, f"Args: {args}, Kwargs: {kwargs}")
                logging.log(level, f"Traceback: {traceback.format_exc()}")
                raise
        
        @wraps(func)
        def sync_wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except VLAException:
                # Re-raise VLA-specific exceptions without logging them again
                raise
            except Exception as e:
                logging.log(level, f"Exception in {func.__name__}: {str(e)}")
                logging.log(level, f"Args: {args}, Kwargs: {kwargs}")
                logging.log(level, f"Traceback: {traceback.format_exc()}")
                raise
        
        # Return the appropriate wrapper based on whether the function is async
        if asyncio.iscoroutinefunction(func):
            return async_wrapper
        else:
            return sync_wrapper
    
    return decorator


def validate_api_response(response: Any, expected_keys: Optional[list] = None) -> bool:
    """
    Validate an API response.
    
    Args:
        response: The API response to validate
        expected_keys: Optional list of keys that must be present in the response
        
    Returns:
        True if valid, raises ValidationError if invalid
    """
    if response is None:
        raise ValidationError("API response is None")
    
    if expected_keys:
        if isinstance(response, dict):
            missing_keys = [key for key in expected_keys if key not in response]
            if missing_keys:
                raise ValidationError(f"Missing keys in API response: {missing_keys}")
        else:
            raise ValidationError("API response is not a dictionary when keys are expected")
    
    return True


def handle_error(error_type: VLAErrorType, message: str, 
                original_exception: Optional[Exception] = None) -> VLAException:
    """
    Create and return an appropriate VLA exception based on the error type.
    
    Args:
        error_type: Type of the error
        message: Error message
        original_exception: Original exception that caused this error
        
    Returns:
        Appropriate VLA exception instance
    """
    error_map = {
        VLAErrorType.SAFETY_ERROR: SafetyError,
        VLAErrorType.VALIDATION_ERROR: ValidationError,
    }
    
    error_class = error_map.get(error_type, VLAException)
    return error_class(message, original_exception)


def log_api_call(api_name: str, success: bool, response_time: float, 
                request_details: Optional[Dict[str, Any]] = None):
    """
    Log an API call with details.
    
    Args:
        api_name: Name of the API
        success: Whether the call was successful
        response_time: Time taken for the call in seconds
        request_details: Optional details about the request
    """
    status = "SUCCESS" if success else "FAILED"
    logging.info(f"API CALL: {api_name} - {status} - {response_time:.2f}s")
    
    if request_details:
        logging.debug(f"API CALL DETAILS: {api_name} - {request_details}")


def safe_execute(func, *args, default_return=None, exception_handler=None, **kwargs):
    """
    Safely execute a function, catching exceptions and returning a default value.
    
    Args:
        func: Function to execute
        *args: Arguments to pass to the function
        default_return: Default value to return if an exception occurs
        exception_handler: Optional function to handle exceptions
        **kwargs: Keyword arguments to pass to the function
        
    Returns:
        Result of the function or default_return if exception occurs
    """
    try:
        return func(*args, **kwargs)
    except Exception as e:
        if exception_handler:
            exception_handler(e)
        else:
            logging.warning(f"Exception in {func.__name__}: {e}")
        return default_return


# Initialize logging when this module is imported
setup_logging()