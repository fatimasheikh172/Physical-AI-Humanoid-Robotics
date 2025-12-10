"""
LLM (Large Language Model) client interface for the Vision-Language-Action (VLA) module.

This module provides a unified interface to multiple LLM providers (OpenAI, Anthropic, etc.)
for cognitive planning and natural language processing.
"""

import asyncio
import logging
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Any, Union
import json
from enum import Enum

from openai import AsyncOpenAI
from anthropic import AsyncAnthropic
import google.generativeai as genai

from ..core.config import get_config
from ..core.api_key_manager import get_api_key, is_service_available
from ..core.error_handling import VLAException, VLAErrorType, log_exception
from ..core.utils import setup_logger


class LLMProvider(str, Enum):
    """Enumeration of supported LLM providers"""
    OPENAI = "openai"
    ANTHROPIC = "anthropic"
    GOOGLE = "google"
    LOCAL = "local"


class LLMRole(str, Enum):
    """Enumeration of message roles in the conversation"""
    SYSTEM = "system"
    USER = "user"
    ASSISTANT = "assistant"


class BaseLLMClient(ABC):
    """Abstract base class for LLM clients"""
    
    def __init__(self, provider: LLMProvider, model: str):
        self.provider = provider
        self.model = model
        self.config = get_config()
        self.logger = setup_logger(f"{self.__class__.__name__}_{provider.value}")
        
        self.logger.info(f"Initialized {provider.value} LLM client with model: {model}")
    
    @abstractmethod
    async def generate_text(
        self, 
        prompt: str, 
        system_prompt: Optional[str] = None,
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        **kwargs
    ) -> str:
        """Generate text based on the provided prompt."""
        pass
    
    @abstractmethod
    async def generate_structured_output(
        self, 
        prompt: str, 
        response_format: Dict[str, Any],
        system_prompt: Optional[str] = None,
        **kwargs
    ) -> Dict[str, Any]:
        """Generate structured output based on the prompt and response format."""
        pass
    
    @abstractmethod
    async def chat_completion(
        self,
        messages: List[Dict[str, str]],
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        **kwargs
    ) -> str:
        """Perform a chat completion with conversation history."""
        pass


class OpenAILLMClient(BaseLLMClient):
    """Client for OpenAI's LLM APIs"""
    
    def __init__(self, model: str = "gpt-4-turbo"):
        super().__init__(LLMProvider.OPENAI, model)
        
        if not is_service_available("openai"):
            raise VLAException(
                "OpenAI API key not available", 
                VLAErrorType.CONFIGURATION_ERROR
            )
        
        api_key = get_api_key("openai")
        self.client = AsyncOpenAI(api_key=api_key)
        self.logger.info(f"OpenAI client initialized with model: {model}")
    
    @log_exception()
    async def generate_text(
        self, 
        prompt: str, 
        system_prompt: Optional[str] = None,
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        **kwargs
    ) -> str:
        """Generate text using OpenAI's API."""
        try:
            messages = []
            
            if system_prompt:
                messages.append({"role": "system", "content": system_prompt})
            
            messages.append({"role": "user", "content": prompt})
            
            params = {
                "model": self.model,
                "messages": messages,
                "temperature": temperature or self.config.llm_temperature,
                "max_tokens": max_tokens or 1024
            }
            
            # Add any additional parameters passed in kwargs
            params.update(kwargs)
            
            response = await self.client.chat.completions.create(**params)
            
            return response.choices[0].message.content
            
        except Exception as e:
            self.logger.error(f"Error in OpenAI text generation: {e}")
            raise VLAException(
                f"Error in OpenAI text generation: {str(e)}", 
                VLAErrorType.LLM_ERROR,
                e
            )
    
    @log_exception()
    async def generate_structured_output(
        self, 
        prompt: str, 
        response_format: Dict[str, Any],
        system_prompt: Optional[str] = None,
        **kwargs
    ) -> Dict[str, Any]:
        """Generate structured output using OpenAI's API."""
        try:
            # Note: This is a simplified implementation. For true structured output,
            # you would use OpenAI's function calling or response_format parameter
            # when available in the API version
            
            # Create a prompt that guides the LLM to produce JSON output
            structured_prompt = f"""
            {prompt}
            
            Respond in JSON format with the following structure:
            {json.dumps(response_format, indent=2)}
            
            Only return valid JSON, nothing else.
            """
            
            # For now, let's use the text generation method
            # In a real implementation, you'd use response_format or function calling
            result = await self.generate_text(
                structured_prompt,
                system_prompt=system_prompt,
                **kwargs
            )
            
            # Try to parse the result as JSON
            try:
                # Find potential JSON in the response
                start_idx = result.find('{')
                end_idx = result.rfind('}') + 1
                
                if start_idx != -1 and end_idx != -1:
                    json_str = result[start_idx:end_idx]
                    structured_result = json.loads(json_str)
                    
                    # Validate against expected structure
                    # This is a basic validation - real implementation would be more thorough
                    return structured_result
                else:
                    self.logger.warning("Could not extract JSON from response")
                    return {"raw_response": result}
                    
            except json.JSONDecodeError as e:
                self.logger.error(f"Could not parse JSON from response: {e}")
                return {"raw_response": result, "parse_error": str(e)}
                
        except Exception as e:
            self.logger.error(f"Error in OpenAI structured output generation: {e}")
            raise VLAException(
                f"Error in OpenAI structured output generation: {str(e)}", 
                VLAErrorType.LLM_ERROR,
                e
            )
    
    @log_exception()
    async def chat_completion(
        self,
        messages: List[Dict[str, str]],
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        **kwargs
    ) -> str:
        """Perform a chat completion with conversation history."""
        try:
            params = {
                "model": self.model,
                "messages": messages,
                "temperature": temperature or self.config.llm_temperature,
                "max_tokens": max_tokens or 1024
            }
            
            # Add any additional parameters passed in kwargs
            params.update(kwargs)
            
            response = await self.client.chat.completions.create(**params)
            
            return response.choices[0].message.content
            
        except Exception as e:
            self.logger.error(f"Error in OpenAI chat completion: {e}")
            raise VLAException(
                f"Error in OpenAI chat completion: {str(e)}", 
                VLAErrorType.LLM_ERROR,
                e
            )


class AnthropicLLMClient(BaseLLMClient):
    """Client for Anthropic's Claude API"""
    
    def __init__(self, model: str = "claude-3-sonnet-20240229"):
        super().__init__(LLMProvider.ANTHROPIC, model)
        
        if not is_service_available("anthropic"):
            raise VLAException(
                "Anthropic API key not available", 
                VLAErrorType.CONFIGURATION_ERROR
            )
        
        api_key = get_api_key("anthropic")
        self.client = AsyncAnthropic(api_key=api_key)
        self.logger.info(f"Anthropic client initialized with model: {model}")
    
    @log_exception()
    async def generate_text(
        self, 
        prompt: str, 
        system_prompt: Optional[str] = None,
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        **kwargs
    ) -> str:
        """Generate text using Anthropic's API."""
        try:
            # Anthropic API requires a more specific message format
            messages = [{"role": "user", "content": prompt}]
            
            # System prompt is passed separately in Anthropic API
            system_param = system_prompt or ""
            
            params = {
                "model": self.model,
                "max_tokens": max_tokens or 1024,
                "temperature": temperature or self.config.llm_temperature,
                "system": system_param,
                "messages": messages
            }
            
            # Add any additional parameters passed in kwargs
            params.update(kwargs)
            
            response = await self.client.messages.create(**params)
            
            return response.content[0].text
            
        except Exception as e:
            self.logger.error(f"Error in Anthropic text generation: {e}")
            raise VLAException(
                f"Error in Anthropic text generation: {str(e)}", 
                VLAErrorType.LLM_ERROR,
                e
            )
    
    @log_exception()
    async def generate_structured_output(
        self, 
        prompt: str, 
        response_format: Dict[str, Any],
        system_prompt: Optional[str] = None,
        **kwargs
    ) -> Dict[str, Any]:
        """Generate structured output using Anthropic's API."""
        try:
            # Create a prompt that guides the LLM to produce JSON output
            structured_prompt = f"""
            {prompt}
            
            Respond in JSON format with the following structure:
            {json.dumps(response_format, indent=2)}
            
            Only return valid JSON, nothing else.
            """
            
            result = await self.generate_text(
                structured_prompt,
                system_prompt=system_prompt,
                **kwargs
            )
            
            # Try to parse the result as JSON
            try:
                # Find potential JSON in the response
                start_idx = result.find('{')
                end_idx = result.rfind('}') + 1
                
                if start_idx != -1 and end_idx != -1:
                    json_str = result[start_idx:end_idx]
                    structured_result = json.loads(json_str)
                    
                    return structured_result
                else:
                    self.logger.warning("Could not extract JSON from response")
                    return {"raw_response": result}
                    
            except json.JSONDecodeError as e:
                self.logger.error(f"Could not parse JSON from response: {e}")
                return {"raw_response": result, "parse_error": str(e)}
                
        except Exception as e:
            self.logger.error(f"Error in Anthropic structured output generation: {e}")
            raise VLAException(
                f"Error in Anthropic structured output generation: {str(e)}", 
                VLAErrorType.LLM_ERROR,
                e
            )
    
    @log_exception()
    async def chat_completion(
        self,
        messages: List[Dict[str, str]],
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        **kwargs
    ) -> str:
        """Perform a chat completion with conversation history."""
        try:
            # Separate system messages from others
            system_message = ""
            filtered_messages = []
            
            for msg in messages:
                if msg.get("role") == "system":
                    if system_message:
                        system_message += "\n" + msg.get("content", "")
                    else:
                        system_message = msg.get("content", "")
                else:
                    filtered_messages.append(msg)
            
            params = {
                "model": self.model,
                "max_tokens": max_tokens or 1024,
                "temperature": temperature or self.config.llm_temperature,
                "system": system_message,
                "messages": filtered_messages
            }
            
            # Add any additional parameters passed in kwargs
            params.update(kwargs)
            
            response = await self.client.messages.create(**params)
            
            return response.content[0].text
            
        except Exception as e:
            self.logger.error(f"Error in Anthropic chat completion: {e}")
            raise VLAException(
                f"Error in Anthropic chat completion: {str(e)}", 
                VLAErrorType.LLM_ERROR,
                e
            )


class LLMClient:
    """Main LLM client interface that can switch between providers"""
    
    def __init__(self, provider: str = None, model: str = None):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Use provided values or fall back to config
        selected_provider = provider or self.config.llm_provider
        selected_model = model or self.config.llm_model
        
        if selected_provider == LLMProvider.OPENAI:
            self.client = OpenAILLMClient(selected_model)
        elif selected_provider == LLMProvider.ANTHROPIC:
            self.client = AnthropicLLMClient(selected_model)
        else:
            raise VLAException(
                f"Unsupported LLM provider: {selected_provider}", 
                VLAErrorType.CONFIGURATION_ERROR
            )
        
        self.provider = selected_provider
        self.model = selected_model
        self.logger.info(f"LLM client initialized: {selected_provider}/{selected_model}")
    
    @log_exception()
    async def generate_text(
        self, 
        prompt: str, 
        system_prompt: Optional[str] = None,
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        **kwargs
    ) -> str:
        """Generate text using the selected LLM provider."""
        return await self.client.generate_text(
            prompt=prompt,
            system_prompt=system_prompt,
            temperature=temperature,
            max_tokens=max_tokens,
            **kwargs
        )
    
    @log_exception()
    async def generate_structured_output(
        self, 
        prompt: str, 
        response_format: Dict[str, Any],
        system_prompt: Optional[str] = None,
        **kwargs
    ) -> Dict[str, Any]:
        """Generate structured output using the selected LLM provider."""
        return await self.client.generate_structured_output(
            prompt=prompt,
            response_format=response_format,
            system_prompt=system_prompt,
            **kwargs
        )
    
    @log_exception()
    async def chat_completion(
        self,
        messages: List[Dict[str, str]],
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        **kwargs
    ) -> str:
        """Perform a chat completion using the selected LLM provider."""
        return await self.client.chat_completion(
            messages=messages,
            temperature=temperature,
            max_tokens=max_tokens,
            **kwargs
        )
    
    @log_exception()
    async def generate_plan_from_command(
        self, 
        command: str, 
        robot_capabilities: List[str],
        environment_context: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Generate an action plan from a natural language command.
        
        Args:
            command: Natural language command to plan for
            robot_capabilities: List of capabilities the robot has
            environment_context: Context about the current environment
            
        Returns:
            Dictionary containing the action plan
        """
        try:
            system_prompt = f"""
            You are an AI cognitive planner for a robotic system. Your role is to translate natural language commands 
            into detailed, executable action plans for a robot with specific capabilities.
            
            Robot capabilities: {', '.join(robot_capabilities)}
            
            Environmental context: {json.dumps(environment_context, indent=2)}
            
            Provide your response in valid JSON format with the following structure:
            {{
                "command_received": "...",
                "decomposition": [
                    {{
                        "step_id": "...",
                        "action_type": "...", 
                        "description": "...",
                        "parameters": {{}},
                        "estimated_duration": 0,
                        "prerequisites": [...]
                    }}
                ],
                "execution_context": {{
                    "environment_map": "...",
                    "robot_state": {{}},
                    "constraints": {{}}
                }},
                "safety_analysis": {{
                    "potential_risks": [...],
                    "safety_checks": [...]
                }}
            }}
            
            Be specific and practical in your action plan. Consider the physical constraints and 
            safety requirements of robotic systems. Each action should be executable by the robot.
            """
            
            user_prompt = f"Create a detailed action plan for the following command: '{command}'"
            
            plan = await self.generate_structured_output(
                prompt=user_prompt,
                response_format={
                    "command_received": "string",
                    "decomposition": [
                        {
                            "step_id": "string",
                            "action_type": "string",
                            "description": "string",
                            "parameters": "object",
                            "estimated_duration": "number",
                            "prerequisites": ["string"]
                        }
                    ],
                    "execution_context": "object",
                    "safety_analysis": "object"
                },
                system_prompt=system_prompt
            )
            
            self.logger.info(f"Generated plan for command: {command[:50]}{'...' if len(command) > 50 else ''}")
            return plan
            
        except Exception as e:
            self.logger.error(f"Error generating plan from command: {e}")
            raise VLAException(
                f"Error generating plan from command: {str(e)}", 
                VLAErrorType.LLM_ERROR,
                e
            )


# Global LLM client instance
_llm_client = None


def get_llm_client(provider: str = None, model: str = None) -> LLMClient:
    """Get the global LLM client instance."""
    global _llm_client
    if _llm_client is None:
        _llm_client = LLMClient(provider, model)
    return _llm_client


def generate_plan_from_command(
    command: str, 
    robot_capabilities: List[str],
    environment_context: Dict[str, Any]
) -> Dict[str, Any]:
    """Convenience function to generate a plan from a command."""
    llm_client = get_llm_client()
    return asyncio.run(llm_client.generate_plan_from_command(command, robot_capabilities, environment_context))