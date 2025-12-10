# Vision-Language-Action (VLA) Module

The Vision-Language-Action (VLA) module implements a complete AI-native robotic system that integrates voice recognition, large language model cognitive planning, and robotic action execution. This system enables natural human-robot interaction through voice commands that are translated into executable action sequences.

## Features

- **Voice Command Processing**: Uses OpenAI Whisper for voice recognition and natural language understanding
- **Cognitive Planning**: Employs LLMs (OpenAI GPT, Anthropic Claude) to translate natural language commands into executable action plans
- **Action Execution**: Executes action sequences through ROS 2 integration with robotic platforms
- **Vision Perception**: Real-time object detection and scene understanding for environmental awareness
- **Safety Integration**: Comprehensive safety checks and validation layers throughout the pipeline
- **Modular Architecture**: Component-based design allowing independent development and testing

## Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Voice        │    │     LLM-based    │    │   Action        │
│ Recognition    │───▶│   Cognitive      │───▶│   Execution     │
│   (Whisper)    │    │   Planning       │    │   (ROS 2)       │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                        │                        │
         ▼                        ▼                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Vision       │    │   Path Planning  │    │   Robot         │
│   Perception   │───▶│   & Navigation   │───▶│   Interface     │
│   (OpenCV/     │    │   Integration    │    │   (Hardware/    │
│   PyTorch)     │    │                  │    │   Simulation)   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

The VLA module follows a modular architecture where each component can be developed, tested, and deployed independently while maintaining clear interfaces for integration.

## Technologies Used

- **Backend**: Python 3.11 with ROS 2 Humble Hawksbill
- **Voice Processing**: OpenAI Whisper API
- **LLM Integration**: OpenAI GPT API, Anthropic Claude API
- **Vision Processing**: OpenCV, PyTorch
- **Robot Interface**: ROS 2 action clients and services
- **API Documentation**: OpenAPI 3.0 specifications

## Installation

### Prerequisites

- ROS 2 Humble Hawksbill or Rolling (with Gazebo Classic)
- Python 3.10 or higher
- Pip package manager
- OpenAI API key
- (Optional) Anthropic API key for Claude integration

### Setup

1. Clone the repository:
   ```bash
   git clone https://github.com/[your-org]/ai-native-robotic-education.git
   cd ai-native-robotic-education
   ```

2. Navigate to the VLA module:
   ```bash
   cd backend/vla_module
   ```

3. Create a virtual environment:
   ```bash
   python3 -m venv vla_env
   source vla_env/bin/activate  # On Windows: vla_env\Scripts\activate
   ```

4. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   # Or install manually:
   pip install rclpy openai anthropic opencv-python torch torchvision torchaudio numpy
   ```

5. Set up environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your API keys
   ```

## Configuration

The VLA module is configured through environment variables and configuration files:

- **API Keys**: Set `OPENAI_API_KEY` and optionally `ANTHROPIC_API_KEY` in `.env`
- **Performance Parameters**: Configuration in `config/vla_config.yaml`
- **Robot Specifications**: Define robot capabilities and constraints in `config/robot_specs.yaml`

Key configuration parameters:
- `whisper_model`: OpenAI Whisper model to use (default: whisper-1)
- `llm_model`: LLM model to use for planning (default: gpt-4-turbo)
- `max_navigation_distance`: Maximum navigation distance in meters (default: 10.0)
- `action_timeout`: Default timeout for actions in seconds (default: 30.0)
- `safety_collision_threshold`: Distance threshold for collision detection (default: 0.5m)

## Usage

### Running the Complete VLA System

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Activate Python environment
source vla_env/bin/activate

# Launch the complete system
ros2 launch vla_module vla_complete_system.launch.py
```

### CLI Interface

The VLA module includes a command-line interface for testing and direct control:

```bash
# Process a text command (simulating voice input)
python -m vla_module.cli.vla_cli --command "Move to the kitchen and find the red cup"

# Run a specific component in isolation
python -m vla_module.voice_recognition.debug_voice --audio-file test_audio.wav

# Test the cognitive planning with a command
python -m vla_module.llm_planning.debug_planner --command "Clean the table"
```

### Example Commands

The VLA module supports various types of natural language commands:

- **Navigation**: "Go to the kitchen", "Move to the table in the corner"
- **Object Interaction**: "Pick up the red cup", "Grasp the pen on the desk"
- **Complex Tasks**: "Find the keys in the office and bring them to me"
- **Multi-step Operations**: "Navigate to the living room, detect the remote, and pick it up"

## Components

### Voice Recognition
Handles audio input and converts to text. Supports both real-time audio streaming and pre-recorded audio files.

### Cognitive Planning
Translates natural language commands into step-by-step action plans. Includes safety validation and capability verification.

### Action Execution
Executes action sequences on the robot via ROS 2 interfaces. Handles error recovery and execution monitoring.

### Vision Perception
Provides real-time object detection and environmental understanding to enhance action planning.

### State Management
Keeps track of robot state and environmental context across the entire pipeline.

## Development

### Adding New Action Types

To add new action types:

1. Define the action in the data model (`core/data_models.py`)
2. Create an action handler in the action execution module
3. Add to cognitive planner action mappings
4. Update the path planning integrations if needed

### Testing

Run the test suite:

```bash
python -m pytest tests/unit/ -v
python -m pytest tests/integration/ -v
python -m pytest tests/performance/ -v
```

### Project Structure

```
backend/vla_module/
├── voice_recognition/      # Voice processing components
│   ├── whisper_client.py
│   ├── audio_processor.py
│   └── voice_command_node.py
├── llm_planning/           # Cognitive planning components
│   ├── cognitive_planner.py
│   ├── action_sequencer.py
│   └── prompt_engineering.py
├── action_execution/       # Action execution components
│   ├── ros2_action_client.py
│   ├── robot_controller.py
│   └── action_executor.py
├── vision_perception/      # Vision components
│   ├── vision_processor.py
│   ├── object_detector.py
│   └── cv_processor.py
├── capstone_integration/   # Integration components
│   ├── full_pipeline_integrator.py
│   ├── capstone_demo.py
│   └── path_planning_integrator.py
├── core/                   # Core utilities and base components
│   ├── message_types.py
│   ├── config.py
│   ├── error_handling.py
│   └── vla_manager.py
├── cli/                    # Command-line interface
├── launch/                 # ROS 2 launch files
├── config/                 # Configuration files
├── tests/                  # Test files
└── requirements.txt        # Python dependencies
```

## Safety Considerations

The VLA module implements multiple safety layers:

- **Action Validation**: All planned actions are validated against robot capabilities
- **Environmental Awareness**: Continuous monitoring of surroundings for obstacles/changes
- **Force Limiting**: Built-in force limits for manipulation tasks
- **Emergency Stop**: Ability to halt all actions immediately
- **Path Validation**: Verification of navigation paths for safety before execution

## Performance Benchmarks

- **Voice Recognition**: <1s average transcription time
- **Cognitive Planning**: <2s average plan generation time
- **Action Execution**: <5% failure rate for basic actions
- **Overall System**: >80% task completion success rate for complex commands

## Troubleshooting

### Common Issues

1. **API Errors**: Verify your API keys are set correctly in the `.env` file
2. **ROS 2 Issues**: Make sure ROS 2 is sourced and the robot drivers are running
3. **Vision Processing**: Ensure OpenCV and PyTorch are properly installed
4. **Network Connectivity**: The system requires internet for Whisper and LLM APIs

### Verification Commands

1. Test voice recognition: `python -m vla_module.voice_recognition.test_client`
2. Test LLM connection: `python -m vla_module.llm_planning.test_llm_connection`
3. Check ROS 2 setup: `ros2 node list | grep vla`
4. Test action execution: `python -m vla_module.action_execution.test_executor`

## Contributing

We welcome contributions to the VLA module. Please follow these guidelines:

1. Fork the repository and create a feature branch
2. Follow the existing code style and documentation practices
3. Write tests for new functionality
4. Submit a pull request with a clear description of changes

## License

This project is licensed under the Apache 2.0 License - see the LICENSE file for details.

## Support

For questions and issues, please open an issue on the GitHub repository.