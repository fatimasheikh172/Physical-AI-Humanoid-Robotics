"""
Command-Line Interface for the Vision-Language-Action (VLA) module.

This module provides a CLI for interacting with the VLA system, allowing users
to issue commands, run demonstrations, and manage the VLA system.
"""

import argparse
import asyncio
import logging
import sys
import json
import os
from typing import Dict, Any, Optional
from pathlib import Path

from ..core.config import get_config
from ..core.error_handling import log_exception, VLAException, VLAErrorType
from ..core.utils import setup_logger
from ..core.message_types import VoiceCommand, CognitivePlan, ActionSequence
from ..core.data_models import VoiceCommandModel, CognitivePlanModel, ActionSequenceModel
from ..core.vla_manager import get_vla_manager
from ..voice_recognition.whisper_client import get_whisper_client
from ..llm_planning.cognitive_planner import get_cognitive_planner
from ..action_execution.robot_controller import get_robot_controller
from ..vision_perception.vision_processor import get_vision_processor
from ..capstone_integration.capstone_demo import get_capstone_demonstrator


class VLACommandLineInterface:
    """
    Command-line interface for the VLA module.
    Allows users to interact with the system via command line.
    """
    
    def __init__(self):
        self.config = get_config()
        self.logger = setup_logger(self.__class__.__name__)
        
        # Initialize VLA components
        self.vla_manager = get_vla_manager()
        self.whisper_client = get_whisper_client()
        self.cognitive_planner = get_cognitive_planner()
        self.robot_controller = get_robot_controller()
        self.vision_processor = get_vision_processor()
        self.capstone_demonstrator = get_capstone_demonstrator()
        
        self.parser = self._create_parser()
        
        self.logger.info("VLACommandLineInterface initialized")
    
    def _create_parser(self) -> argparse.ArgumentParser:
        """Create the argument parser with all available commands."""
        parser = argparse.ArgumentParser(
            description="Vision-Language-Action (VLA) Module Command-Line Interface",
            formatter_class=argparse.RawDescriptionHelpFormatter,
            epilog="""
Examples:
  %(prog)s command "Move to the kitchen" --user john
  %(prog)s demo run navigation --count 3
  %(prog)s status
  %(prog)s validate --component planning
            """
        )
        
        subparsers = parser.add_subparsers(dest='command', help='Available commands')
        
        # Command subparser - process a single command
        command_parser = subparsers.add_parser(
            'command', 
            help='Process a natural language command'
        )
        command_parser.add_argument(
            'text_command', 
            type=str, 
            help='Natural language command to process'
        )
        command_parser.add_argument(
            '--user', 
            type=str, 
            default='cli_user', 
            help='User ID for the command (default: cli_user)'
        )
        command_parser.add_argument(
            '--robot-state', 
            type=str, 
            help='Path to robot state file (JSON)'  
        )
        command_parser.add_argument(
            '--verbose', 
            action='store_true', 
            help='Show detailed execution information'
        )
        
        # Demo subparser - run demonstration scenarios
        demo_parser = subparsers.add_parser(
            'demo', 
            help='Run demonstration scenarios'
        )
        demo_subparsers = demo_parser.add_subparsers(dest='demo_action', help='Demo actions')
        
        run_demo_parser = demo_subparsers.add_parser(
            'run', 
            help='Run a specific demo scenario'
        )
        run_demo_parser.add_argument(
            'scenario', 
            type=str, 
            nargs='?',
            default='complex_task',
            choices=['navigation', 'object_interaction', 'complex_task', 'autonomous_helper', 'auto_humanoid'],
            help='Demo scenario to run (default: complex_task)'
        )
        run_demo_parser.add_argument(
            '--count', 
            type=int, 
            default=1, 
            help='Number of times to run the demo (default: 1)'
        )
        run_demo_parser.add_argument(
            '--verbose', 
            action='store_true', 
            help='Show detailed demo information'
        )
        
        # Status subparser - get system status
        status_parser = subparsers.add_parser(
            'status', 
            help='Get system status and component information'
        )
        status_parser.add_argument(
            '--component', 
            type=str, 
            choices=['voice', 'planning', 'execution', 'vision', 'all'],
            default='all',
            help='Specific component status to check (default: all)'
        )
        
        # Validate subparser - validate system components
        validate_parser = subparsers.add_parser(
            'validate', 
            help='Validate system components for readiness'
        )
        validate_parser.add_argument(
            '--component', 
            type=str, 
            choices=['voice', 'planning', 'execution', 'vision', 'all'],
            default='all',
            help='Specific component to validate (default: all)'
        )
        validate_parser.add_argument(
            '--verbose', 
            action='store_true', 
            help='Show detailed validation information'
        )
        
        # Config subparser - manage configuration
        config_parser = subparsers.add_parser(
            'config', 
            help='Manage VLA module configuration'
        )
        config_subparsers = config_parser.add_subparsers(dest='config_action', help='Config actions')
        
        show_config_parser = config_subparsers.add_parser(
            'show', 
            help='Show current configuration'
        )
        show_config_parser.add_argument(
            '--json', 
            action='store_true', 
            help='Output configuration as JSON'
        )
        
        # Stats subparser - get system statistics
        stats_parser = subparsers.add_parser(
            'stats', 
            help='Get system statistics'
        )
        stats_parser.add_argument(
            '--type', 
            type=str, 
            choices=['demo', 'execution', 'performance', 'errors'],
            default='demo',
            help='Type of statistics to show (default: demo)'
        )
        
        # Vision subparser - vision-specific operations
        vision_parser = subparsers.add_parser(
            'vision', 
            help='Vision perception commands'
        )
        vision_subparsers = vision_parser.add_subparsers(dest='vision_action', help='Vision actions')
        
        detect_parser = vision_subparsers.add_parser(
            'detect', 
            help='Detect objects in an image'
        )
        detect_parser.add_argument(
            'image_path', 
            type=str, 
            help='Path to image file for detection'
        )
        detect_parser.add_argument(
            '--target', 
            type=str, 
            help='Specific object to detect'
        )
        
        # Plan subparser - planning-specific operations
        plan_parser = subparsers.add_parser(
            'plan', 
            help='Cognitive planning commands'
        )
        plan_subparsers = plan_parser.add_subparsers(dest='plan_action', help='Planning actions')
        
        plan_generate_parser = plan_subparsers.add_parser(
            'generate', 
            help='Generate a cognitive plan'
        )
        plan_generate_parser.add_argument(
            'command', 
            type=str, 
            help='Natural language command to generate plan for'
        )
        
        return parser
    
    @log_exception()
    async def run_command(self, args: argparse.Namespace):
        """
        Run the appropriate command based on parsed arguments.
        
        Args:
            args: Parsed arguments namespace
        """
        try:
            if args.command == 'command':
                await self._process_text_command(args)
            elif args.command == 'demo':
                await self._run_demo_command(args)
            elif args.command == 'status':
                await self._get_status_command(args)
            elif args.command == 'validate':
                await self._validate_component_command(args)
            elif args.command == 'config':
                await self._config_command(args)
            elif args.command == 'stats':
                await self._stats_command(args)
            elif args.command == 'vision':
                await self._vision_command(args)
            elif args.command == 'plan':
                await self._plan_command(args)
            else:
                # No command specified - show help
                self.parser.print_help()
                
        except KeyboardInterrupt:
            self.logger.info("Command interrupted by user")
            return
        except Exception as e:
            self.logger.error(f"Error executing command: {e}")
            print(f"Error: {str(e)}", file=sys.stderr)
            sys.exit(1)
    
    async def _process_text_command(self, args: argparse.Namespace):
        """Process a text-based voice command."""
        try:
            self.logger.info(f"Processing text command: '{args.text_command}' for user {args.user}")
            
            # Create voice command model (simulating voice input with text)
            voice_command = VoiceCommandModel.create(
                transcript=args.text_command,
                user_id=args.user,
                language=self.config.whisper_language
            )
            
            # If robot state is provided, load it
            robot_state = None
            if args.robot_state:
                robot_state = self._load_robot_state_from_file(args.robot_state)
            
            # Execute the command through the VLA manager
            if robot_state:
                cognitive_plan = await self.vla_manager.process_text_command_with_state(
                    args.text_command, 
                    args.user, 
                    robot_state
                )
            else:
                cognitive_plan = await self.vla_manager.process_text_command(
                    args.text_command, 
                    args.user
                )
            
            # Print results
            if args.verbose:
                print(json.dumps(cognitive_plan.to_dict(), indent=2))
            else:
                print(f"Command processed successfully")
                print(f"Plan ID: {cognitive_plan.plan_id}")
                print(f"Task count: {len(cognitive_plan.task_decomposition)}")
                print(f"Confidence: {cognitive_plan.confidence:.2%}")
            
            self.logger.info(f"Command processing completed for: {args.text_command[:50]}...")
            
        except Exception as e:
            self.logger.error(f"Error processing text command: {e}")
            raise
    
    async def _run_demo_command(self, args: argparse.Namespace):
        """Run a demonstration scenario."""
        try:
            if args.demo_action == 'run':
                if args.scenario == 'auto_humanoid':
                    # Run the autonomous humanoid demo specifically
                    result = await self.capstone_demonstrator.run_autonomous_humanoid_demo()
                    print(f"Autonomous Humanoid Demo completed:")
                    print(f"  Success: {result.success}")
                    print(f"  Execution time: {result.execution_time:.2f}s")
                    print(f"  Success Rate: {result.success_rate:.2%}")
                else:
                    # Run a general demo
                    for i in range(args.count):
                        if i > 0:
                            print(f"\n--- Running demo {i+1}/{args.count} ---")
                        
                        result = await self.capstone_demonstrator.run_demo_scenario(args.scenario)
                        
                        print(f"Demo {i+1} completed:")
                        print(f"  Scenario: {result.scenario}")
                        print(f"  Success: {result.success}")
                        print(f"  Execution time: {result.execution_time:.2f}s")
                        print(f"  Success rate: {result.success_rate:.2%}")
                        print(f"  Total actions: {result.total_actions}")
                        print(f"  Errors: {len(result.errors)}")
                        
                        if args.verbose and result.details:
                            print(f"  Details: {json.dumps(result.details, indent=2)[:500]}...")
                
            else:
                # Show available demos if no specific action
                demos = self.capstone_demonstrator.get_demo_scenarios()
                print("Available demonstration scenarios:")
                for name, details in demos.items():
                    print(f"  {name}: {details['description']}")
                    
        except Exception as e:
            self.logger.error(f"Error running demo: {e}")
            raise
    
    async def _get_status_command(self, args: argparse.Namespace):
        """Get system status information."""
        try:
            if args.component == 'all' or args.component == 'voice':
                if hasattr(self.whisper_client, 'get_status'):
                    status = await self.whisper_client.get_status()
                    print(f"Voice Recognition Status: {status}")
                else:
                    print("Voice Recognition: Available")
            
            if args.component == 'all' or args.component == 'planning':
                if hasattr(self.cognitive_planner, 'get_status'):
                    status = await self.cognitive_planner.get_status()
                    print(f"Cognitive Planning Status: {status}")
                else:
                    print("Cognitive Planning: Available")
            
            if args.component == 'all' or args.component == 'execution':
                if hasattr(self.robot_controller, 'get_status'):
                    status = await self.robot_controller.get_status()
                    print(f"Action Execution Status: {status}")
                else:
                    print("Action Execution: Available")
            
            if args.component == 'all' or args.component == 'vision':
                if hasattr(self.vision_processor, 'get_status'):
                    status = await self.vision_processor.get_status()
                    print(f"Vision Perception Status: {status}")
                else:
                    print("Vision Perception: Available")
                
            # Overall system status
            print("\nVLA System Status:")
            active_demos = self.capstone_demonstrator.get_active_demos()
            print(f"  Active demos: {len(active_demos)}")
            
            if active_demos:
                for demo_id, info in active_demos.items():
                    print(f"    {demo_id}: {info['scenario']} for {info['duration']:.1f}s")
        
        except Exception as e:
            self.logger.error(f"Error getting status: {e}")
            raise
    
    async def _validate_component_command(self, args: argparse.Namespace):
        """Validate system components for readiness."""
        try:
            if args.component == 'all' or args.component == 'voice':
                is_ready = await self._validate_voice_component()
                status_icon = "✓" if is_ready else "✗"
                print(f"{status_icon} Voice Recognition Component: {'Ready' if is_ready else 'Not Ready'}")
                
                if args.verbose and not is_ready:
                    print("  Voice component validation failed")
            
            if args.component == 'all' or args.component == 'planning':
                is_ready = await self._validate_planning_component()
                status_icon = "✓" if is_ready else "✗"
                print(f"{status_icon} Cognitive Planning Component: {'Ready' if is_ready else 'Not Ready'}")
                
                if args.verbose and not is_ready:
                    print("  Planning component validation failed")
            
            if args.component == 'all' or args.component == 'execution':
                is_ready = await self._validate_execution_component()
                status_icon = "✓" if is_ready else "✗"
                print(f"{status_icon} Action Execution Component: {'Ready' if is_ready else 'Not Ready'}")
                
                if args.verbose and not is_ready:
                    print("  Execution component validation failed")
            
            if args.component == 'all' or args.component == 'vision':
                is_ready = await self._validate_vision_component()
                status_icon = "✓" if is_ready else "✗"
                print(f"{status_icon} Vision Perception Component: {'Ready' if is_ready else 'Not Ready'}")
                
                if args.verbose and not is_ready:
                    print("  Vision component validation failed")
            
            # Run the comprehensive validation
            ready_issues = await self.capstone_demonstrator.validate_demo_readiness()
            if ready_issues:
                print(f"\nSystem readiness check found {len(ready_issues)} issues:")
                for issue in ready_issues:
                    print(f"  - {issue}")
            else:
                print(f"\n✓ System is ready for operation")
                
        except Exception as e:
            self.logger.error(f"Error validating components: {e}")
            raise
    
    async def _validate_voice_component(self) -> bool:
        """Validate the voice recognition component."""
        try:
            # In a real implementation, this would test actual connection to Whisper API
            # For simulation, we'll just check if the client is initialized
            if hasattr(self.whisper_client, 'test_connection'):
                return await self.whisper_client.test_connection()
            else:
                # Assume ready if no explicit test method
                return True
        except Exception:
            return False
    
    async def _validate_planning_component(self) -> bool:
        """Validate the cognitive planning component."""
        try:
            # In a real implementation, this would test actual connection to LLM API
            if hasattr(self.cognitive_planner, 'test_connection'):
                return await self.cognitive_planner.test_connection()
            else:
                # Assume ready if no explicit test method
                return True
        except Exception:
            return False
    
    async def _validate_execution_component(self) -> bool:
        """Validate the action execution component."""
        try:
            # In a real implementation, this would test actual robot connection
            if hasattr(self.robot_controller, 'test_connection'):
                return await self.robot_controller.test_connection()
            else:
                # Assume ready if no explicit test method
                return True
        except Exception:
            return False
    
    async def _validate_vision_component(self) -> bool:
        """Validate the vision perception component."""
        try:
            # In a real implementation, this would test actual vision system
            if hasattr(self.vision_processor, 'test_connection'):
                return await self.vision_processor.test_connection()
            else:
                # Assume ready if no explicit test method
                return True
        except Exception:
            return False
    
    async def _config_command(self, args: argparse.Namespace):
        """Handle configuration commands."""
        try:
            if args.config_action == 'show':
                config_data = {
                    'robot_id': self.config.get('robot_id', 'default_robot'),
                    'llm_model': self.config.get('llm_model', 'gpt-4'),
                    'whisper_language': self.config.get('whisper_language', 'en'),
                    'max_navigation_distance': self.config.get('max_navigation_distance', 10.0),
                    'safety_collision_threshold': self.config.get('safety_collision_threshold', 0.5),
                    'execution_timeout': self.config.get('execution_timeout', 30.0),
                    'vision_processing_frequency': self.config.get('vision_processing_frequency', 10.0)
                }
                
                if args.json:
                    print(json.dumps(config_data, indent=2))
                else:
                    print("VLA Module Configuration:")
                    for key, value in config_data.items():
                        print(f"  {key}: {value}")
            else:
                print("Available config commands: show")
                
        except Exception as e:
            self.logger.error(f"Error in config command: {e}")
            raise
    
    async def _stats_command(self, args: argparse.Namespace):
        """Handle statistics commands."""
        try:
            if args.type == 'demo':
                stats = self.capstone_demonstrator.get_demo_statistics()
                print("Demo Statistics:")
                print(f"  Total demos: {stats['total_demos']}")
                print(f"  Successful demos: {stats['successful_demos']}")
                print(f"  Success rate: {stats['success_rate']:.2%}")
                print(f"  Average execution time: {stats['average_execution_time']:.2f}s")
                
                if stats['recent_demos']:
                    print(f"\nRecent demos:")
                    for demo in stats['recent_demos']:
                        status_icon = "✓" if demo['success'] else "✗"
                        print(f"  {status_icon} {demo['demo_id'][:12]}...: {demo['scenario']} ({demo['success_rate']:.1%})")
            
            elif args.type == 'execution':
                # In a real implementation, this would return execution statistics
                print("Execution statistics not available in this simulation")
            
            else:
                print(f"Statistics type '{args.type}' not implemented yet")
                
        except Exception as e:
            self.logger.error(f"Error in stats command: {e}")
            raise
    
    async def _vision_command(self, args: argparse.Namespace):
        """Handle vision-specific commands."""
        try:
            if args.vision_action == 'detect':
                if not os.path.exists(args.image_path):
                    raise FileNotFoundError(f"Image file not found: {args.image_path}")
                
                # Load image as bytes for processing
                with open(args.image_path, 'rb') as f:
                    image_bytes = f.read()
                
                # Process image with vision system
                if args.target:
                    # Detect specific object
                    detection_result = await self.vision_processor.detect_specific_object(image_bytes, args.target)
                    if detection_result:
                        print(f"Found {args.target}: {detection_result.name} with confidence {detection_result.confidence:.2f}")
                        print(f"Position: ({detection_result.position_3d['x']:.2f}, {detection_result.position_3d['y']:.2f}, {detection_result.position_3d['z']:.2f})")
                    else:
                        print(f"Did not find {args.target} in the image")
                else:
                    # General object detection
                    perception_result = await self.vision_processor.process_image(image_bytes)
                    print(f"Detected {len(perception_result.objects_detected)} objects:")
                    for obj in perception_result.objects_detected:
                        print(f"  - {obj.name}: confidence {obj.confidence:.2f}")
            
            else:
                print("Available vision commands: detect")
                
        except Exception as e:
            self.logger.error(f"Error in vision command: {e}")
            raise
    
    async def _plan_command(self, args: argparse.Namespace):
        """Handle planning-specific commands."""
        try:
            if args.plan_action == 'generate':
                # Create a voice command model from the text
                voice_command = VoiceCommandModel.create(
                    transcript=args.command,
                    user_id="cli_user",
                    language=self.config.whisper_language
                )
                
                # Generate plan
                cognitive_plan = await self.cognitive_planner.plan(voice_command)
                
                # Print plan details
                print(f"Generated plan '{cognitive_plan.plan_id}' for command: {args.command}")
                print(f"Confidence: {cognitive_plan.confidence:.2%}")
                print(f"Task decomposition ({len(cognitive_plan.task_decomposition)} tasks):")
                
                for i, task in enumerate(cognitive_plan.task_decomposition):
                    print(f"  {i+1}. {task.task_type}: {task.task_description}")
                    if task.parameters:
                        print(f"      Parameters: {json.dumps(task.parameters, indent=6)[6:]}")
            
            else:
                print("Available planning commands: generate")
                
        except Exception as e:
            self.logger.error(f"Error in plan command: {e}")
            raise
    
    def _load_robot_state_from_file(self, file_path: str) -> 'RobotStateModel':
        """
        Load robot state from a JSON file.
        
        Args:
            file_path: Path to the JSON file containing robot state
            
        Returns:
            RobotStateModel loaded from the file
        """
        with open(file_path, 'r') as f:
            state_data = json.load(f)
        
        # Create RobotStateModel from the loaded data
        # This would map JSON fields to the RobotStateModel constructor
        return RobotStateModel(
            state_id=state_data.get('state_id', f"state_{int(time.time())}"),
            robot_id=state_data.get('robot_id', 'loaded_robot'),
            position=state_data.get('position', {'x': 0.0, 'y': 0.0, 'z': 0.0}),
            orientation=state_data.get('orientation', {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}),
            timestamp=state_data.get('timestamp', time.time()),
            battery_level=state_data.get('battery_level', 1.0),
            joints=state_data.get('joints', []),
            gripper_state=state_data.get('gripper_state', {}),
            capabilities=state_data.get('capabilities', []),
            mode=state_data.get('mode', 'idle'),
            current_action=state_data.get('current_action'),
            sensors=state_data.get('sensors', []),
            velocity=state_data.get('velocity')
        )
    
    def run_interactive(self):
        """Run in interactive mode."""
        print("VLA Command-Line Interface")
        print("Type 'help' for available commands, 'quit' to exit\n")
        
        while True:
            try:
                user_input = input("vla> ").strip()
                
                if user_input.lower() in ['quit', 'exit', 'q']:
                    break
                elif user_input.lower() == 'help':
                    self.parser.print_help()
                elif user_input:
                    # Parse and run the command
                    args = self.parser.parse_args(user_input.split())
                    asyncio.run(self.run_command(args))
                
            except EOFError:
                # Handle Ctrl+D
                print("\nQuitting...")
                break
            except SystemExit:
                # argparse calls sys.exit on errors, we don't want to exit the whole program
                continue
            except KeyboardInterrupt:
                print("\nInterrupted")
                continue
            except Exception as e:
                print(f"Error: {e}")


def main():
    """Main entry point for the VLA CLI."""
    cli = VLACommandLineInterface()
    
    # Parse command line arguments
    args = cli.parser.parse_args()
    
    # If no arguments provided, run interactive mode
    if len(sys.argv) == 1:
        cli.run_interactive()
    else:
        # Run the specific command
        try:
            asyncio.run(cli.run_command(args))
        except KeyboardInterrupt:
            print("\nCommand interrupted by user")
            sys.exit(0)


if __name__ == '__main__':
    main()