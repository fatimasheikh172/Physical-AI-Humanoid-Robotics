"""
Common utilities and helper functions for the Vision-Language-Action (VLA) module.
"""

import logging
import time
import asyncio
from typing import Any, Dict, Optional, Callable, Awaitable
from functools import wraps
import json
import base64


def setup_logger(name: str, level: int = logging.INFO) -> logging.Logger:
    """
    Set up a logger with the specified name and level.
    
    Args:
        name: Name of the logger
        level: Logging level (default: INFO)
        
    Returns:
        Configured logger instance
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)
    
    # Avoid adding multiple handlers if logger already exists
    if not logger.handlers:
        handler = logging.StreamHandler()
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)
    
    return logger


def time_it(func: Callable) -> Callable:
    """
    Decorator to time function execution.
    
    Args:
        func: Function to time
        
    Returns:
        Timed function
    """
    @wraps(func)
    async def async_wrapper(*args, **kwargs):
        start = time.time()
        try:
            result = await func(*args, **kwargs)
            end = time.time()
            logging.info(f"{func.__name__} took {end - start:.2f} seconds")
            return result
        except Exception as e:
            end = time.time()
            logging.error(f"{func.__name__} took {end - start:.2f} seconds and failed: {e}")
            raise
    
    @wraps(func)
    def sync_wrapper(*args, **kwargs):
        start = time.time()
        try:
            result = func(*args, **kwargs)
            end = time.time()
            logging.info(f"{func.__name__} took {end - start:.2f} seconds")
            return result
        except Exception as e:
            end = time.time()
            logging.error(f"{func.__name__} took {end - start:.2f} seconds and failed: {e}")
            raise
    
    # Return the appropriate wrapper based on whether the function is async
    if asyncio.iscoroutinefunction(func):
        return async_wrapper
    else:
        return sync_wrapper


def validate_json(json_str: str) -> bool:
    """
    Validate if a string is valid JSON.
    
    Args:
        json_str: String to validate
        
    Returns:
        True if valid JSON, False otherwise
    """
    try:
        json.loads(json_str)
        return True
    except ValueError:
        return False


def encode_image_to_base64(image_path: str) -> Optional[str]:
    """
    Encode an image file to base64 string.
    
    Args:
        image_path: Path to the image file
        
    Returns:
        Base64 encoded string or None if failed
    """
    try:
        with open(image_path, "rb") as image_file:
            encoded_string = base64.b64encode(image_file.read()).decode('utf-8')
            return encoded_string
    except Exception as e:
        logging.error(f"Failed to encode image {image_path} to base64: {e}")
        return None


def decode_base64_to_image(base64_str: str, output_path: str) -> bool:
    """
    Decode a base64 string to an image file.
    
    Args:
        base64_str: Base64 encoded string
        output_path: Path to save the decoded image
        
    Returns:
        True if successful, False otherwise
    """
    try:
        image_data = base64.b64decode(base64_str)
        with open(output_path, "wb") as image_file:
            image_file.write(image_data)
        return True
    except Exception as e:
        logging.error(f"Failed to decode base64 to image {output_path}: {e}")
        return False


def sanitize_text(text: str) -> str:
    """
    Sanitize text input by removing potentially harmful characters.
    
    Args:
        text: Input text to sanitize
        
    Returns:
        Sanitized text
    """
    # Remove potentially harmful characters
    sanitized = text.replace('\0', '')  # Null bytes
    sanitized = sanitized.replace('\x00', '')  # Another form of null
    return sanitized


def retry_on_failure(max_retries: int = 3, delay: float = 1.0):
    """
    Decorator to retry a function on failure.
    
    Args:
        max_retries: Maximum number of retries (default: 3)
        delay: Delay between retries in seconds (default: 1.0)
        
    Returns:
        Decorated function
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        async def async_wrapper(*args, **kwargs):
            last_exception = None
            
            for attempt in range(max_retries):
                try:
                    return await func(*args, **kwargs)
                except Exception as e:
                    last_exception = e
                    if attempt < max_retries - 1:
                        logging.warning(f"Attempt {attempt + 1} failed: {e}. Retrying in {delay}s...")
                        await asyncio.sleep(delay)
                    else:
                        logging.error(f"All {max_retries} attempts failed. Last error: {e}")
            
            raise last_exception
        
        @wraps(func)
        def sync_wrapper(*args, **kwargs):
            last_exception = None
            
            for attempt in range(max_retries):
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    last_exception = e
                    if attempt < max_retries - 1:
                        logging.warning(f"Attempt {attempt + 1} failed: {e}. Retrying in {delay}s...")
                        time.sleep(delay)
                    else:
                        logging.error(f"All {max_retries} attempts failed. Last error: {e}")
            
            raise last_exception
        
        # Return the appropriate wrapper based on whether the function is async
        if asyncio.iscoroutinefunction(func):
            return async_wrapper
        else:
            return sync_wrapper
    
    return decorator


def format_timestamp(timestamp: Optional[float] = None) -> str:
    """
    Format a timestamp as an ISO 8601 string.
    
    Args:
        timestamp: Unix timestamp (default: current time)
        
    Returns:
        Formatted timestamp string
    """
    import datetime
    
    if timestamp is None:
        timestamp = time.time()
    
    return datetime.datetime.fromtimestamp(timestamp).isoformat()


def calculate_distance_3d(p1: Dict[str, float], p2: Dict[str, float]) -> float:
    """
    Calculate 3D Euclidean distance between two points.
    
    Args:
        p1: First point with x, y, z coordinates
        p2: Second point with x, y, z coordinates
        
    Returns:
        Distance between the points
    """
    import math
    
    dx = p2['x'] - p1['x']
    dy = p2['y'] - p1['y']
    dz = p2['z'] - p1['z']
    
    return math.sqrt(dx*dx + dy*dy + dz*dz)


class AsyncRateLimiter:
    """
    A simple async rate limiter to control the rate of API calls.
    """
    
    def __init__(self, max_calls: int, time_window: float):
        """
        Initialize the rate limiter.
        
        Args:
            max_calls: Maximum number of calls allowed
            time_window: Time window in seconds
        """
        self.max_calls = max_calls
        self.time_window = time_window
        self.calls = []
    
    async def acquire(self):
        """Acquire permission to make a call, potentially waiting if needed."""
        now = time.time()
        
        # Remove calls that are outside the time window
        self.calls = [call_time for call_time in self.calls if now - call_time < self.time_window]
        
        if len(self.calls) >= self.max_calls:
            # Wait until we can make another call
            sleep_time = self.time_window - (now - self.calls[0])
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
        
        # Add the current call
        self.calls.append(time.time())