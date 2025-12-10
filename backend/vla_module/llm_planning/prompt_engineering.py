"""
Prompt engineering for LLM safety and accuracy in the Vision-Language-Action (VLA) module.

This module provides carefully crafted prompts that guide LLMs to generate
safe, accurate, and contextually appropriate action plans for robotic systems.
"""

import logging
from typing import Dict, List, Any, Optional
from enum import Enum


class SafetyConstraint(Enum):
    """Types of safety constraints to enforce."""
    PHYSICAL_SAFETY = "physical_safety"
    ENVIRONMENTAL_SAFETY = "environmental_safety"
    HUMAN_SAFETY = "human_safety"
    PROPERTY_SAFETY = "property_safety"
    DATA_SAFETY = "data_safety"


class AccuracyConstraint(Enum):
    """Types of accuracy requirements."""
    PRECISION = "precision"
    RECALL = "recall"
    COMPLETENESS = "completeness"
    CONSISTENCY = "consistency"


class PromptEngineering:
    """
    Class to manage all prompt templates for the VLA system.
    Provides safe and effective prompts for various tasks.
    """
    
    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        
        # Robot capabilities that should be referenced in prompts
        self.robot_capabilities = [
            "navigation", "manipulation", "object_detection",
            "grasping", "releasing", "rotating", "moving_forward",
            "moving_backward", "lifting", "placing"
        ]
        
        # Safety constraints that should be emphasized in prompts
        self.safety_constraints = [
            "avoid collisions", "respect boundaries", "operate within limits",
            "protect humans", "preserve property", "follow protocols"
        ]
        
        self.logger.info("PromptEngineering initialized")
    
    def create_system_prompt_template(self, robot_type: str = "default") -> str:
        """
        Create a system prompt template that establishes the LLM's role and constraints.
        
        Args:
            robot_type: Type of robot being controlled
            
        Returns:
            Formatted system prompt string
        """
        # Different robot types may need different capabilities or constraints
        if robot_type == "humanoid":
            capabilities_description = """
You are controlling a humanoid robot with the following capabilities:
- Navigate in 3D space with collision avoidance
- Manipulate objects with dexterous hands
- Detect and recognize objects and people in the environment
- Communicate verbally with humans
- Perform complex multi-step tasks involving both mobility and manipulation
"""
        elif robot_type == "wheeled_manipulator":
            capabilities_description = """
You are controlling a wheeled robot with manipulator arm with the following capabilities:
- Navigate in 2D space with collision avoidance
- Manipulate objects with a robotic arm
- Detect and recognize objects in the environment
- Perform tasks involving both mobility and manipulation
"""
        else:
            capabilities_description = """
You are controlling a robot with the following capabilities:
- Navigate in 3D space with collision avoidance
- Manipulate objects
- Detect and recognize objects in the environment
- Perform simple to moderate complexity tasks
"""
        
        return f"""{capabilities_description}

SAFETY CONSTRAINTS AND PROTOCOLS:
1. Always prioritize human safety above task completion
2. Respect physical boundaries and obstacles
3. Operate within robot's physical limits
4. If uncertain about safety, request clarification rather than proceeding
5. Avoid actions that could damage property or environment

Your role is to interpret natural language commands into detailed, executable action plans.
Each action plan should include:
- Sequence of specific actions to perform
- Required parameters for each action
- Safety checks before critical actions
- Error handling and contingency plans
- Estimation of time required for each action

Always ensure the plan is physically achievable by the robot and respects all safety constraints.
"""
    
    def create_command_to_plan_prompt(
        self, 
        command: str, 
        robot_capabilities: List[str] = None,
        environment_context: Dict[str, Any] = None,
        safety_requirements: List[str] = None
    ) -> str:
        """
        Create a prompt to convert a natural language command to an action plan.
        
        Args:
            command: Natural language command to process
            robot_capabilities: List of robot capabilities (defaults to self.robot_capabilities)
            environment_context: Context about the environment
            safety_requirements: Specific safety requirements to emphasize
            
        Returns:
            Formatted prompt for command-to-plan conversion
        """
        capabilities = robot_capabilities or self.robot_capabilities
        safety_reqs = safety_requirements or self.safety_constraints
        env_context = environment_context or {}
        
        environment_info = ""
        if env_context:
            environment_info = f"""
ENVIRONMENTAL CONTEXT:
{self._format_environment_context(env_context)}
"""
        
        safety_info = ""
        if safety_reqs:
            safety_info = f"""
SPECIFIC SAFETY REQUIREMENTS:
{"- " + chr(10) + "- ".join(safety_reqs)}

"""
        
        return f"""{environment_info}

{self._create_safety_emphasis(safety_reqs)}

COMMAND TO PROCESS:
{command}

CAPABILITIES AVAILABLE:
{", ".join(capabilities)}

Generate a detailed action plan for the robot to execute this command safely and effectively.

Respond in the following JSON format:
{{
  "command_received": "...",
  "decomposition": [
    {{
      "step_id": "...",
      "action_type": "...", 
      "description": "...",
      "parameters": {{}},
      "estimated_duration": 0,
      "safety_checks": ["..."],
      "prerequisites": ["..."]
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

Be specific about action types and parameters. Action types should be from this list:
- move_to: Move robot to a specific location
- pick_up: Pick up an object
- place: Place an object at a location
- detect_object: Look for specific objects
- grasp: Grasp an object
- release: Release a grasped object
- rotate: Rotate robot
- navigate: Navigate through space
- stop: Stop current actions

Only respond with valid JSON, nothing else.
"""
    
    def create_validation_prompt(
        self,
        action_plan: Dict[str, Any],
        robot_capabilities: List[str] = None,
        safety_requirements: List[str] = None
    ) -> str:
        """
        Create a prompt to validate an action plan for safety and feasibility.
        
        Args:
            action_plan: Action plan to validate
            robot_capabilities: Robot capabilities to validate against
            safety_requirements: Safety requirements to check for
            
        Returns:
            Formatted prompt for plan validation
        """
        capabilities = robot_capabilities or self.robot_capabilities
        safety_reqs = safety_requirements or self.safety_constraints
        
        return f"""
ACTION PLAN VALIDATION REQUEST:

ACTION PLAN TO VALIDATE:
{self._format_action_plan(action_plan)}

ROBOT CAPABILITIES:
{", ".join(capabilities)}

SAFETY REQUIREMENTS TO CHECK:
{", ".join(safety_reqs)}

For each action in the plan, verify:
1. Is the action type supported by the robot's capabilities?
2. Are the action parameters within the robot's operational limits?
3. Does the action respect environmental constraints?
4. Does the action prioritize safety over task completion?
5. Are there adequate safety checks before risky actions?

Provide your analysis in the following JSON format:
{{
  "valid": boolean,
  "errors": [
    {{
      "action_id": "...",
      "issue_type": "capability|safety|feasibility|constraint",
      "description": "...",
      "severity": "critical|high|medium|low"
    }}
  ],
  "warnings": [
    {{ 
      "action_id": "...",
      "warning_type": "...",
      "description": "..."
    }}
  ],
  "suggestions": [
    {{
      "action_id": "...",
      "suggestion": "..."
    }}
  ]
}}

Only respond with valid JSON, nothing else.
"""
    
    def create_capability_query_prompt(
        self,
        command: str,
        robot_capabilities: List[str],
        environment_context: Dict[str, Any] = None
    ) -> str:
        """
        Create a prompt to determine if the robot can perform a command.
        
        Args:
            command: Command to evaluate
            robot_capabilities: Robot capabilities to check against
            environment_context: Context about the environment
            
        Returns:
            Formatted prompt for capability query
        """
        env_context = environment_context or {}
        
        environment_info = ""
        if env_context:
            environment_info = f"""
ENVIRONMENTAL CONTEXT:
{self._format_environment_context(env_context)}
"""
        
        return f"""{environment_info}

ABILITY ASSESSMENT REQUEST:

COMMAND TO ASSESS:
{command}

ROBOT CAPABILITIES:
{", ".join(robot_capabilities)}

Does the robot have the capability to perform this command in the given environment?
Consider:
1. Required physical capabilities
2. Environmental constraints
3. Safety considerations
4. Available time/resources

Respond in the following JSON format:
{{
  "can_perform": boolean,
  "capabilities_needed": ["..."],
  "capabilities_available": ["..."],
  "missing_capabilities": ["..."],
  "reasoning": "..."
}}

Only respond with valid JSON, nothing else.
"""
    
    def create_error_recovery_prompt(
        self,
        failed_action: Dict[str, Any],
        error_context: Dict[str, Any],
        robot_state: Dict[str, Any]
    ) -> str:
        """
        Create a prompt for generating error recovery strategies.
        
        Args:
            failed_action: The action that failed
            error_context: Context about the error
            robot_state: Current state of the robot
            
        Returns:
            Formatted prompt for error recovery
        """
        return f"""
ERROR RECOVERY REQUEST:

FAILED ACTION:
{self._format_action(failed_action)}

ERROR CONTEXT:
{self._format_error_context(error_context)}

CURRENT ROBOT STATE:
{self._format_robot_state(robot_state)}

Generate recovery strategies to handle this error and continue toward the original goal when possible.

Respond in the following JSON format:
{{
  "recovery_strategies": [
    {{
      "strategy_id": "...",
      "description": "...",
      "next_action": "{{action_object}}",
      "fallback_option": boolean
    }}
  ],
  "goal_adjustment_needed": boolean,
  "alternative_approach": "..."
}}

Only respond with valid JSON, nothing else.
"""
    
    def _format_environment_context(self, context: Dict[str, Any]) -> str:
        """Format environment context for prompts."""
        return str(context)
    
    def _create_safety_emphasis(self, safety_requirements: List[str]) -> str:
        """Create emphasis on safety in prompts."""
        return f"""
IMPORTANT SAFETY INSTRUCTIONS:
You are controlling a real robot in a real environment. The actions you plan will be executed physically.
- Always prioritize safety over task completion
- When in doubt, favor conservative actions
- Explicitly include safety checks for potentially dangerous actions
- Consider all possible failure modes and include mitigations
- {chr(10) + chr(10)}.join([f"- {req}" for req in safety_requirements])
"""
    
    def _format_action_plan(self, plan: Dict[str, Any]) -> str:
        """Format an action plan for display in prompts."""
        return str(plan)
    
    def _format_action(self, action: Dict[str, Any]) -> str:
        """Format an action for display in prompts."""
        return str(action)
    
    def _format_error_context(self, error: Dict[str, Any]) -> str:
        """Format error context for display in prompts."""
        return str(error)
    
    def _format_robot_state(self, state: Dict[str, Any]) -> str:
        """Format robot state for display in prompts."""
        return str(state)


class PromptSafetyValidator:
    """
    Validates that prompts contain appropriate safety considerations.
    """
    
    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        
        # Keywords that should appear in safety-conscious prompts
        self.safety_keywords = [
            "safety", "secure", "protect", "risk", "harm", "danger", 
            "caution", "careful", "carefully", "protocol", "protocol", 
            "boundaries", "limits", "constraints", "avoid", "prevent"
        ]
        
        # Keywords that suggest unsafe behavior to watch for
        self.unsafe_keywords = [
            "ignore", "disregard", "override", "force", "break", 
            "damage", "break", "hurt", "harm", "unsafe", "danger"
        ]
    
    def validate_prompt_safety(self, prompt: str) -> Dict[str, Any]:
        """
        Validate that a prompt contains appropriate safety considerations.
        
        Args:
            prompt: Prompt to validate
            
        Returns:
            Dictionary with validation results
        """
        result = {
            'is_safe': True,
            'safety_keywords_found': [],
            'unsafe_keywords_found': [],
            'issues': [],
            'suggestions': []
        }
        
        lowercase_prompt = prompt.lower()
        
        # Check for safety keywords
        for keyword in self.safety_keywords:
            if keyword in lowercase_prompt:
                result['safety_keywords_found'].append(keyword)
        
        # Check for potentially unsafe keywords
        for keyword in self.unsafe_keywords:
            if keyword in lowercase_prompt:
                result['unsafe_keywords_found'].append(keyword)
                result['issues'].append(f"Potentially unsafe keyword found: {keyword}")
        
        # If no safety keywords are found, suggest adding them
        if not result['safety_keywords_found']:
            result['is_safe'] = False
            result['issues'].append("No safety-related keywords found in prompt")
            result['suggestions'].append(f"Add safety considerations such as: {', '.join(self.safety_keywords[:5])}")
        else:
            result['is_safe'] = len(result['unsafe_keywords_found']) == 0
        
        return result


# Global instances
_prompt_engineering = None
_prompt_validator = None


def get_prompt_engineering() -> PromptEngineering:
    """Get the global prompt engineering instance."""
    global _prompt_engineering
    if _prompt_engineering is None:
        _prompt_engineering = PromptEngineering()
    return _prompt_engineering


def get_prompt_validator() -> PromptSafetyValidator:
    """Get the global prompt safety validator instance."""
    global _prompt_validator
    if _prompt_validator is None:
        _prompt_validator = PromptSafetyValidator()
    return _prompt_validator