# Contributing to Vision-Language-Action (VLA) Module

We welcome contributions to the VLA module! This document provides guidelines for contributing to the project.

## Development Setup

1. Fork and clone the repository
2. Create a virtual environment:
   ```bash
   python -m venv vla_env
   source vla_env/bin/activate  # On Windows: vla_env\Scripts\activate
   ```
3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   pip install -e .
   ```

## Code Standards

- Follow PEP 8 coding standards
- Use type hints for all function parameters and return values
- Write comprehensive docstrings for all classes and functions
- Add unit tests for new functionality
- Ensure code passes all existing tests

## Pull Request Process

1. Create a feature branch for your changes
2. Make your changes following the code standards
3. Add or update tests as needed
4. Ensure all tests pass
5. Submit a pull request with a clear description of your changes

## Testing

Run all tests before submitting changes:
```bash
pytest tests/
```

For integration tests:
```bash
pytest tests/integration/
```

## Architecture Principles

The VLA module follows these architectural principles:
- Each component (voice recognition, LLM interface, action execution) should be modular and testable
- All functionality should have CLI interfaces for testing and debugging
- TDD is enforced with tests written before implementation
- Focus on integration points between voice recognition, planning, and action execution
- All components should have structured logging and appropriate error handling