"""
Comprehensive logging and error handling for the Vision-Language-Action (VLA) module.

This module implements centralized logging and standardized error handling
across all VLA components to ensure observability and reliability.
"""

import logging
import logging.config
import sys
import traceback
from typing import Dict, Any, Optional, List
from enum import Enum
import json
import time
from datetime import datetime
import asyncio

from ..core.config import get_config
from ..core.error_handling import VLAException, VLAErrorType


class VLAFormatter(logging.Formatter):
    """
    Custom formatter for VLA module logs that includes structured information
    and follows consistent format across all components.
    """
    
    def format(self, record):
        # Create a structured log entry
        log_entry = {
            'timestamp': datetime.utcfromtimestamp(record.created).isoformat() + 'Z',
            'level': record.levelname,
            'logger': record.name,
            'message': record.getMessage(),
            'module': record.module,
            'function': record.funcName,
            'line': record.lineno
        }
        
        # Add exception info if present
        if record.exc_info:
            log_entry['exception'] = self.formatException(record.exc_info)
        
        # Add extra fields if present
        if hasattr(record, 'user_id'):
            log_entry['user_id'] = record.user_id
        if hasattr(record, 'command_id'):
            log_entry['command_id'] = record.command_id
        if hasattr(record, 'plan_id'):
            log_entry['plan_id'] = record.plan_id
        if hasattr(record, 'action_id'):
            log_entry['action_id'] = record.action_id
        if hasattr(record, 'session_id'):
            log_entry['session_id'] = record.session_id
        
        return json.dumps(log_entry)


class VLAErrorHandler:
    """
    Centralized error handler for the VLA module.
    Provides consistent error handling and reporting across all components.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = self._setup_vla_logger()
        self.error_history = []
        self.max_error_history = getattr(self.config, 'max_error_history', 1000)
        
        # Error recovery strategies
        self.recovery_strategies = {
            VLAErrorType.VOICE_RECOGNITION_ERROR: self._recover_voice_error,
            VLAErrorType.LLM_ERROR: self._recover_llm_error,
            VLAErrorType.ACTION_EXECUTION_ERROR: self._recover_action_error,
            VLAErrorType.VISION_ERROR: self._recover_vision_error,
            VLAErrorType.SAFETY_ERROR: self._recover_safety_error,
            VLAErrorType.ROBOT_COMMUNICATION_ERROR: self._recover_robot_comm_error
        }
        
        self.logger.info("VLAErrorHandler initialized")
    
    def _setup_vla_logger(self) -> logging.Logger:
        """
        Set up the VLA module logger with standardized configuration.
        
        Returns:
            Configured logger instance
        """
        # Create a custom logger
        logger = logging.getLogger('vla_module')
        logger.setLevel(getattr(self.config, 'log_level', logging.INFO))
        
        # Prevent adding handlers multiple times
        if logger.handlers:
            return logger
        
        # Create handler with custom formatter
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(VLAFormatter())
        
        # Add the handler to the logger
        logger.addHandler(handler)
        
        # Prevent propagation to root logger to avoid duplicate logs
        logger.propagate = False
        
        return logger
    
    def log_debug(self, message: str, extra: Optional[Dict[str, Any]] = None):
        """Log a debug message."""
        self._log_message(logging.DEBUG, message, extra)
    
    def log_info(self, message: str, extra: Optional[Dict[str, Any]] = None):
        """Log an info message."""
        self._log_message(logging.INFO, message, extra)
    
    def log_warning(self, message: str, extra: Optional[Dict[str, Any]] = None):
        """Log a warning message."""
        self._log_message(logging.WARNING, message, extra)
    
    def log_error(self, message: str, extra: Optional[Dict[str, Any]] = None):
        """Log an error message."""
        self._log_message(logging.ERROR, message, extra)
    
    def log_critical(self, message: str, extra: Optional[Dict[str, Any]] = None):
        """Log a critical message."""
        self._log_message(logging.CRITICAL, message, extra)
    
    def _log_message(self, level: int, message: str, extra: Optional[Dict[str, Any]] = None):
        """
        Internal method to log a message with optional extra context.
        
        Args:
            level: Logging level
            message: Log message
            extra: Optional dictionary with extra context
        """
        if extra is None:
            extra = {}
        
        # Create a custom log record with extra fields
        record = logging.LogRecord(
            name=self.logger.name,
            level=level,
            pathname='',
            lineno=0,
            msg=message,
            args=(),
            exc_info=None
        )
        
        # Add extra fields to the record
        for key, value in extra.items():
            setattr(record, key, value)
        
        self.logger.handle(record)
    
    def handle_exception(self, e: Exception, context: Optional[Dict[str, Any]] = None):
        """
        Handle an exception with standardized processing.
        
        Args:
            e: Exception to handle
            context: Optional context information about where the exception occurred
        """
        if context is None:
            context = {}
        
        # Create a detailed error report
        error_report = {
            'error_type': type(e).__name__,
            'error_message': str(e),
            'traceback': traceback.format_exc(),
            'timestamp': time.time(),
            'context': context
        }
        
        # Add to error history
        self.error_history.append(error_report)
        if len(self.error_history) > self.max_error_history:
            self.error_history.pop(0)  # Remove oldest error
        
        # Log the error
        extra = {**context}
        self.log_error(f"Exception occurred: {type(e).__name__}: {str(e)}", extra=extra)
        
        # If it's a VLAException, handle according to error type
        if isinstance(e, VLAException):
            self._handle_vla_exception(e, context)
    
    def _handle_vla_exception(self, e: VLAException, context: Dict[str, Any]):
        """
        Handle VLA-specific exceptions with tailored responses.
        
        Args:
            e: VLAException to handle
            context: Context information
        """
        self.log_error(
            f"VLA Exception: {e.error_type.value}, Message: {e.message}", 
            extra={**context, 'error_type': e.error_type.value}
        )
        
        # Attempt recovery if a strategy exists
        if e.error_type in self.recovery_strategies:
            try:
                recovery_result = self.recovery_strategies[e.error_type](e, context)
                return recovery_result
            except Exception as recovery_error:
                self.log_error(f"Recovery failed: {recovery_error}")
                # Recovery failure is logged but original exception is still propagated
    
    def _recover_voice_error(self, e: VLAException, context: Dict[str, Any]) -> bool:
        """Recover from voice recognition errors."""
        self.log_warning("Attempting voice recognition recovery", extra={'context': context})
        
        # Recovery might involve restarting voice recognition services
        # or falling back to text-based input
        return True  # Indicate recovery attempt completed
    
    def _recover_llm_error(self, e: VLAException, context: Dict[str, Any]) -> bool:
        """Recover from LLM communication errors."""
        self.log_warning("Attempting LLM communication recovery", extra={'context': context})
        
        # Recovery might involve switching to backup LLM provider
        # or using cached responses
        return True
    
    def _recover_action_error(self, e: VLAException, context: Dict[str, Any]) -> bool:
        """Recover from action execution errors."""
        self.log_warning("Attempting action execution recovery", extra={'context': context})
        
        # Recovery might involve retrying action or planning alternative
        return True
    
    def _recover_vision_error(self, e: VLAException, context: Dict[str, Any]) -> bool:
        """Recover from vision processing errors."""
        self.log_warning("Attempting vision processing recovery", extra={'context': context})
        
        # Recovery might involve restarting vision services
        # or using alternative perception methods
        return True
    
    def _recover_safety_error(self, e: VLAException, context: Dict[str, Any]) -> bool:
        """Recover from safety-related errors."""
        self.log_critical("Processing safety error", extra={'context': context})
        
        # Safety errors require immediate action - likely emergency stop
        # This would trigger safety protocols in a real implementation
        return False  # Cannot recover from safety errors
    
    def _recover_robot_comm_error(self, e: VLAException, context: Dict[str, Any]) -> bool:
        """Recover from robot communication errors."""
        self.log_warning("Attempting robot communication recovery", extra={'context': context})
        
        # Recovery might involve reconnecting to robot
        return True
    
    def get_error_summary(self) -> Dict[str, Any]:
        """
        Get a summary of recent errors.
        
        Returns:
            Dictionary with error summary statistics
        """
        if not self.error_history:
            return {
                'total_errors': 0,
                'error_types': [],
                'recent_errors': []
            }
        
        # Calculate statistics
        total_errors = len(self.error_history)
        
        # Count error types
        error_type_counts = {}
        for error in self.error_history:
            err_type = error['error_type']
            error_type_counts[err_type] = error_type_counts.get(err_type, 0) + 1
        
        # Get most recent errors
        recent_errors = self.error_history[-10:]  # Last 10 errors
        
        return {
            'total_errors': total_errors,
            'error_type_distribution': error_type_counts,
            'recent_errors': [
                {
                    'type': err['error_type'],
                    'message': err['error_message'][:100] + ('...' if len(err['error_message']) > 100 else ''),
                    'timestamp': err['timestamp']
                } for err in recent_errors
            ],
            'error_rate_per_hour': self._calculate_error_rate()
        }
    
    def _calculate_error_rate(self) -> float:
        """Calculate error rate per hour."""
        if not self.error_history:
            return 0.0
        
        time_span = time.time() - self.error_history[0]['timestamp']
        hours_span = time_span / 3600.0
        return len(self.error_history) / hours_span if hours_span > 0 else 0.0
    
    def setup_component_logging(self, component_name: str) -> logging.Logger:
        """
        Set up logging for a specific VLA component.
        
        Args:
            component_name: Name of the component
            
        Returns:
            Logger configured for the component
        """
        logger = logging.getLogger(f"vla_module.{component_name}")
        logger.setLevel(getattr(self.config, 'log_level', logging.INFO))
        
        # Use the same handler and formatter as the main logger
        if not logger.handlers and self.logger.handlers:
            logger.addHandler(self.logger.handlers[0])
            logger.propagate = False
        
        return logger


# Global error handler instance
_vla_error_handler = None


def get_error_handler() -> VLAErrorHandler:
    """Get the global VLA error handler instance."""
    global _vla_error_handler
    if _vla_error_handler is None:
        _vla_error_handler = VLAErrorHandler()
    return _vla_error_handler


def log_debug(message: str, extra: Optional[Dict[str, Any]] = None):
    """Convenience function for debug logging."""
    handler = get_error_handler()
    handler.log_debug(message, extra)


def log_info(message: str, extra: Optional[Dict[str, Any]] = None):
    """Convenience function for info logging."""
    handler = get_error_handler()
    handler.log_info(message, extra)


def log_warning(message: str, extra: Optional[Dict[str, Any]] = None):
    """Convenience function for warning logging."""
    handler = get_error_handler()
    handler.log_warning(message, extra)


def log_error(message: str, extra: Optional[Dict[str, Any]] = None):
    """Convenience function for error logging."""
    handler = get_error_handler()
    handler.log_error(message, extra)


def log_critical(message: str, extra: Optional[Dict[str, Any]] = None):
    """Convenience function for critical logging."""
    handler = get_error_handler()
    handler.log_critical(message, extra)


def handle_exception(e: Exception, context: Optional[Dict[str, Any]] = None):
    """Convenience function to handle an exception."""
    handler = get_error_handler()
    handler.handle_exception(e, context)


def get_error_summary() -> Dict[str, Any]:
    """Convenience function to get error summary."""
    handler = get_error_handler()
    return handler.get_error_summary()


def setup_component_logger(component_name: str) -> logging.Logger:
    """Convenience function to set up component-specific logger."""
    handler = get_error_handler()
    return handler.setup_component_logging(component_name)