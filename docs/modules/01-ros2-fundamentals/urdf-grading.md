# URDF Grading Rubric for Humanoid Robot Models

## Purpose
This rubric evaluates URDF models of humanoid robots, assessing the quality and completeness of the robot definition according to established best practices and humanoid robotics requirements.

## Evaluation Categories

### 1. Structural Design (25 points)

#### Link Definition (10 points)
- **Excellent (9-10 points)**: All essential humanoid links defined with realistic proportions (torso, head, arms with shoulders/elbows/wrists, legs with hips/knees/ankles)
- **Good (7-8 points)**: Most essential links defined but minor proportion issues
- **Fair (5-6 points)**: Basic links defined but missing some essential parts or unrealistic proportions
- **Poor (0-4 points)**: Incomplete or incorrect link definitions

#### Joint Design (10 points)
- **Excellent (9-10 points)**: Appropriate joint types connecting links with realistic ranges of motion (at least 12 total DOF)
- **Good (7-8 points)**: Mostly correct joint types and connections with minor limitations
- **Fair (5-6 points)**: Basic joint connections but limited DOF or inappropriate joint types
- **Poor (0-4 points)**: Major issues with joint connections or types

#### Kinematic Chain (5 points)
- **Excellent (5 points)**: Proper parent-child relationships creating a single, valid kinematic tree with a clear root link
- **Good (4 points)**: Mostly correct but minor tree structure issues
- **Fair (3 points)**: Basic tree structure but some connectivity issues
- **Poor (0-2 points)**: Incorrect tree structure or disconnected components

### 2. Physical Properties (20 points)

#### Mass Properties (10 points)
- **Excellent (9-10 points)**: Realistic masses assigned to all links with appropriate distribution
- **Good (7-8 points)**: Mostly realistic masses but some values could be improved
- **Fair (5-6 points)**: Masses assigned but with some unrealistic values
- **Poor (0-4 points)**: Missing or unrealistic mass properties

#### Inertial Properties (10 points)
- **Excellent (9-10 points)**: Properly defined inertia tensors respecting physical laws (positive diagonal elements, satisfying triangle inequality)
- **Good (7-8 points)**: Mostly correct inertias but minor approximation issues
- **Fair (5-6 points)**: Inertias present but some values inconsistent with link geometry
- **Poor (0-4 points)**: Incorrect or missing inertial properties

### 3. Visual and Collision Models (20 points)

#### Visual Elements (10 points)
- **Excellent (9-10 points)**: Appropriate geometric shapes or accurate meshes for all links with proper materials
- **Good (7-8 points)**: Mostly appropriate visuals with minor accuracy issues
- **Fair (5-6 points)**: Basic visuals present but geometry doesn't match concept
- **Poor (0-4 points)**: Missing or inappropriate visual elements

#### Collision Elements (10 points)
- **Excellent (9-10 points)**: Appropriate collision geometry for all links that matches or simplifies visual geometry
- **Good (7-8 points)**: Proper collision geometry but some could be optimized
- **Fair (5-6 points)**: Collision geometry present but may be inaccurate for simulation
- **Poor (0-4 points)**: Missing or inappropriate collision geometry

### 4. Humanoid Design Principles (15 points)

#### Anthropomorphic Proportions (8 points)
- **Excellent (7-8 points)**: Links sized appropriately relative to human proportions (limb lengths, torso height, head size)
- **Good (6 points)**: Mostly proportional but some dimension issues
- **Fair (4-5 points)**: Roughly humanoid but proportions significantly off
- **Poor (0-3 points)**: Non-humanoid proportions

#### Degrees of Freedom (7 points)
- **Excellent (6-7 points)**: Sufficient DOF for intended humanoid behaviors (minimum 12 DOF distributed appropriately)
- **Good (5 points)**: Close to appropriate DOF but minor limitations
- **Fair (3-4 points)**: Basic movement DOF but limited functionality
- **Poor (0-2 points)**: Insufficient DOF for effective movement

### 5. Technical Quality and Standards (20 points)

#### URDF Syntax and Structure (8 points)
- **Excellent (7-8 points)**: Valid XML with proper URDF structure, no syntax errors
- **Good (6 points)**: Proper structure with minor validation warnings
- **Fair (4-5 points)**: Mostly correct structure but with some validation issues
- **Poor (0-3 points)**: Syntax errors or structural issues

#### Documentation and Comments (6 points)
- **Excellent (6 points)**: Clear comments explaining design choices, parameter meanings, and structure
- **Good (5 points)**: Adequate comments with minor omissions
- **Fair (3-4 points)**: Basic comments but missing important explanations
- **Poor (0-2 points)**: Absent or inadequate documentation

#### Compliance with Standards (6 points)
- **Excellent (6 points)**: Follows ROS/URDF best practices, proper naming conventions, appropriate file organization
- **Good (5 points)**: Mostly follows standards with minor deviations
- **Fair (3-4 points)**: Some adherence to standards but notable deviations
- **Poor (0-2 points)**: Does not follow established standards

## Total Score: 100 points

### Grade Boundaries
- **A (90-100 points)**: Excellent humanoid robot model meeting all requirements and best practices
- **B (80-89 points)**: Good humanoid robot model with minor issues
- **C (70-79 points)**: Adequate humanoid robot model meeting basic requirements
- **D (60-69 points)**: Below-average model with significant issues
- **F (0-59 points)**: Unsatisfactory model not meeting core requirements

## Additional Considerations

### Bonus Points (up to 10 points)
- Advanced features: Custom sensors, actuators, transmission definitions
- Simulation enhancements: Gazebo-specific tags, plugins
- Realism: Detailed meshes, accurate reproduction of real humanoid designs
- Creativity: Innovative solutions to common humanoid design challenges

### Critical Deficiencies
The following issues automatically result in major point deductions:
- Inability to load the URDF file
- Kinematic loops
- Negative mass/inertia values
- Incorrect parent-child relationships

## Feedback Guidelines

When providing feedback, address:
1. Strengths of the design
2. Areas for improvement with specific suggestions
3. Resources for understanding humanoid robotics principles
4. Recommendations for enhancing the model