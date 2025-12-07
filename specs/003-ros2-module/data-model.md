# Data Model: Module 1 - The Robotic Nervous System (ROS 2)

**Feature**: Module 1 - The Robotic Nervous System (ROS 2)
**Date**: 2025-12-07
**Modeler**: AI Assistant

## Overview

This document defines the data models for Module 1 of the Physical AI & Humanoid Robotics textbook, focusing specifically on ROS 2 fundamentals. The module includes content for 3 weeks covering ROS 2 architecture, communication patterns, and AI-ROS integration with humanoid robot modeling.

## Entity Models

### ROS2Module
**Description**: Represents the complete Module 1 content focused on ROS 2 fundamentals
**Fields**:
- id (UUID, Primary Key)
- title (String, Required)
- description (Text, Required)
- module_number (Integer, Required, Value: 1)
- duration_weeks (Integer, Required, Value: 3)
- learning_objectives (Text Array, Required)
- prerequisites (Text Array, Optional)
- created_at (DateTime)
- updated_at (DateTime)
- published (Boolean, Default: false)

**Relationships**:
- One-to-Many: ROS2Module → ROS2Week
- One-to-Many: ROS2Module → ROS2Assessment
- One-to-Many: ROS2Module → StudentModuleProgress

**Validation Rules**:
- Title and description required
- Module number must be 1 (this module)
- Duration must be 3 weeks

### ROS2Week
**Description**: Represents a week of content within the ROS 2 module
**Fields**:
- id (UUID, Primary Key)
- module_id (UUID, Foreign Key to ROS2Module, Required)
- week_number (Integer, Required, 1-3)
- title (String, Required)
- description (Text, Required)
- learning_objectives (Text Array, Required)
- theory_content (Text, Required)
- lab_tasks (JSON, Required - {task_name: description, setup: string, steps: array})
- mini_project (JSON, Required - {title: string, description: text, requirements: array})
- ai_tutor_integration (JSON, Required - {features: array, focus_areas: array})
- estimated_duration_hours (Integer, Required)
- order_index (Integer, Required within module)

**Relationships**:
- Many-to-One: ROS2Week → ROS2Module
- One-to-Many: ROS2Week → ROS2ContentSection
- One-to-Many: ROS2Week → ROS2LabEnvironment
- One-to-Many: ROS2Week → StudentWeekProgress

**Validation Rules**:
- Week number must be 1, 2, or 3
- Order index must be unique within module
- All content fields required

### ROS2ContentSection
**Description**: A section within a week's content (theory, lab instructions, etc.)
**Fields**:
- id (UUID, Primary Key)
- week_id (UUID, Foreign Key to ROS2Week, Required)
- title (String, Required)
- content_type (Enum: 'theory', 'lab_instruction', 'code_example', 'tutorial', 'quiz', 'project', Required)
- content (Text, Required)
- order_index (Integer, Required within week)
- estimated_completion_time_minutes (Integer, Required)
- difficulty_level (Enum: 'beginner', 'intermediate', 'advanced', Required)
- prerequisite_section_ids (UUID Array, Optional)
- created_at (DateTime)
- updated_at (DateTime)

**Relationships**:
- Many-to-One: ROS2ContentSection → ROS2Week
- One-to-Many: ROS2ContentSection → ContentAsset
- One-to-Many: ROS2ContentSection → StudentSectionProgress

**Validation Rules**:
- Content type must be one of allowed values
- Order index must be unique within week
- Content required

### ROS2LabEnvironment
**Description**: Represents the ROS 2 lab environment for a specific week
**Fields**:
- id (UUID, Primary Key)
- week_id (UUID, Foreign Key to ROS2Week, Required)
- name (String, Required)
- description (Text, Required)
- ros_distro (Enum: 'humble', 'iron', Required, Default: 'humble')
- package_name (String, Required)
- launch_file (String, Required)
- dependencies (Text Array, Required)
- setup_instructions (Text, Required)
- evaluation_criteria (JSON, Required - {criteria: array, weights: object})
- created_at (DateTime)
- updated_at (DateTime)

**Relationships**:
- Many-to-One: ROS2LabEnvironment → ROS2Week
- One-to-Many: ROS2LabEnvironment → LabAsset
- One-to-Many: ROS2LabEnvironment → StudentLabSession

**Validation Rules**:
- ROS distro must be valid
- Package name must follow ROS naming conventions
- All fields required

### LabAsset
**Description**: Assets associated with lab environments (code files, configuration, etc.)
**Fields**:
- id (UUID, Primary Key)
- lab_env_id (UUID, Foreign Key to ROS2LabEnvironment, Required)
- asset_type (Enum: 'python_code', 'launch_file', 'config', 'urdf', 'world_file', 'documentation', Required)
- filename (String, Required)
- original_filename (String, Required)
- file_path (String, Required)
- file_size_bytes (Integer, Required)
- mime_type (String, Required)
- checksum (String, Required for integrity verification)
- uploaded_at (DateTime)
- is_required (Boolean, Default: true)

**Relationships**:
- Many-to-One: LabAsset → ROS2LabEnvironment

**Validation Rules**:
- File size limits based on asset type
- Valid MIME type for asset type
- Checksum required for integrity

### ROS2MiniProject
**Description**: Represents the mini-project for each week of the module
**Fields**:
- id (UUID, Primary Key)
- week_id (UUID, Foreign Key to ROS2Week, Required)
- title (String, Required)
- description (Text, Required)
- requirements (Text Array, Required)
- evaluation_criteria (JSON, Required - {criteria: array, weights: object})
- submission_instructions (Text, Required)
- created_at (DateTime)
- updated_at (DateTime)

**Relationships**:
- Many-to-One: ROS2MiniProject → ROS2Week
- One-to-Many: ROS2MiniProject → StudentProjectSubmission

**Validation Rules**:
- All fields required
- Requirements must be specific and measurable

### ROS2Assessment
**Description**: Assessment components for the ROS 2 module
**Fields**:
- id (UUID, Primary Key)
- module_id (UUID, Foreign Key to ROS2Module, Required)
- week_id (UUID, Foreign Key to ROS2Week, Optional - null for final assessment)
- title (String, Required)
- description (Text, Required)
- assessment_type (Enum: 'quiz', 'practical', 'theory_test', 'debugging_challenge', 'final_project', Required)
- questions_count (Integer, Optional for practical assessments)
- time_limit_minutes (Integer, Optional)
- passing_score_percentage (Integer, 0-100, Required, Default: 70)
- created_at (DateTime)
- updated_at (DateTime)

**Relationships**:
- Many-to-One: ROS2Assessment → ROS2Module
- Many-to-One: ROS2Assessment → ROS2Week (optional)
- One-to-Many: ROS2Assessment → ROS2AssessmentQuestion
- One-to-Many: ROS2Assessment → StudentAssessmentResult

**Validation Rules**:
- Assessment type must be one of allowed values
- Passing score between 0-100

### ROS2AssessmentQuestion
**Description**: Individual question within an ROS 2 assessment
**Fields**:
- id (UUID, Primary Key)
- assessment_id (UUID, Foreign Key to ROS2Assessment, Required)
- question_text (Text, Required)
- question_type (Enum: 'multiple_choice', 'true_false', 'short_answer', 'code', 'practical', Required)
- options (JSON Array, Optional - for multiple choice)
- correct_answer (Text, Required)
- explanation (Text, Optional)
- difficulty_level (Enum: 'beginner', 'intermediate', 'advanced', Required)
- points (Integer, Required, Default: 1)
- order_index (Integer, Required within assessment)

**Relationships**:
- Many-to-One: ROS2AssessmentQuestion → ROS2Assessment
- One-to-Many: ROS2AssessmentQuestion → StudentAssessmentResponse

**Validation Rules**:
- Question text required
- For multiple choice, options must be provided
- Points must be positive

### StudentModuleProgress
**Description**: Tracks student's progress through the ROS 2 module
**Fields**:
- id (UUID, Primary Key)
- student_id (UUID, Foreign Key to User, Required)
- module_id (UUID, Foreign Key to ROS2Module, Required)
- progress_percentage (Integer, 0-100, Required)
- started_at (DateTime)
- completed_at (DateTime, Optional)
- last_accessed_at (DateTime)
- time_spent_seconds (Integer, Default: 0)
- week_progress (JSON, Optional - {week_id: completion_percentage})
- assessment_scores (JSON, Optional - {assessment_id: score})
- project_submissions_status (JSON, Optional - {project_id: status})
- is_completed (Boolean, Default: false)

**Relationships**:
- Many-to-One: StudentModuleProgress → User
- Many-to-One: StudentModuleProgress → ROS2Module
- One-to-Many: StudentModuleProgress → StudentWeekProgress

**Validation Rules**:
- Progress percentage between 0-100
- Started at required
- Can only have one record per student-module combination

### StudentWeekProgress
**Description**: Tracks student's progress through individual weeks
**Fields**:
- id (UUID, Primary Key)
- student_id (UUID, Foreign Key to User, Required)
- week_id (UUID, Foreign Key to ROS2Week, Required)
- progress_percentage (Integer, 0-100, Required)
- started_at (DateTime)
- completed_at (DateTime, Optional)
- last_accessed_at (DateTime)
- time_spent_seconds (Integer, Default: 0)
- section_progress (JSON, Optional - {section_id: completion_status})
- lab_completion_status (JSON, Optional - {lab_env_id: status})
- assessment_scores (JSON, Optional - {assessment_id: score})
- is_completed (Boolean, Default: false)

**Relationships**:
- Many-to-One: StudentWeekProgress → User
- Many-to-One: StudentWeekProgress → ROS2Week
- One-to-Many: StudentWeekProgress → StudentSectionProgress

**Validation Rules**:
- Progress percentage between 0-100
- Started at required
- Can only have one record per student-week combination

### StudentSectionProgress
**Description**: Tracks student's progress through individual content sections
**Fields**:
- id (UUID, Primary Key)
- student_id (UUID, Foreign Key to User, Required)
- section_id (UUID, Foreign Key to ROS2ContentSection, Required)
- started_at (DateTime)
- completed_at (DateTime, Optional)
- time_spent_seconds (Integer, Default: 0)
- completion_status (Enum: 'not_started', 'in_progress', 'completed', Required, Default: 'not_started')
- notes (Text, Optional)

**Relationships**:
- Many-to-One: StudentSectionProgress → User
- Many-to-One: StudentSectionProgress → ROS2ContentSection

**Validation Rules**:
- Completion status must be one of allowed values

### StudentLabSession
**Description**: Records of student's lab sessions with ROS 2
**Fields**:
- id (UUID, Primary Key)
- student_id (UUID, Foreign Key to User, Required)
- lab_env_id (UUID, Foreign Key to ROS2LabEnvironment, Required)
- session_start (DateTime, Required)
- session_end (DateTime, Optional)
- duration_seconds (Integer, Optional)
- ros_commands_used (Text Array, Optional)
- errors_encountered (Text Array, Optional)
- ai_tutor_requests (Integer, Default: 0)
- status (Enum: 'active', 'completed', 'terminated', Required)
- session_notes (Text, Optional)

**Relationships**:
- Many-to-One: StudentLabSession → User
- Many-to-One: StudentLabSession → ROS2LabEnvironment
- One-to-Many: StudentLabSession → LabSessionArtifact

**Validation Rules**:
- Session start required
- Duration must be positive if session completed

### LabSessionArtifact
**Description**: Artifacts generated during lab sessions (code, logs, etc.)
**Fields**:
- id (UUID, Primary Key)
- lab_session_id (UUID, Foreign Key to StudentLabSession, Required)
- artifact_type (Enum: 'code_output', 'console_log', 'image', 'video', 'generated_file', Required)
- filename (String, Required)
- file_path (String, Required)
- file_size_bytes (Integer, Required)
- mime_type (String, Required)
- description (Text, Optional)
- created_at (DateTime)

**Relationships**:
- Many-to-One: LabSessionArtifact → StudentLabSession

**Validation Rules**:
- File size limits based on artifact type
- Valid MIME type for artifact type

### StudentAssessmentResult
**Description**: Student's results for ROS 2 assessments
**Fields**:
- id (UUID, Primary Key)
- student_id (UUID, Foreign Key to User, Required)
- assessment_id (UUID, Foreign Key to ROS2Assessment, Required)
- total_score (Integer, 0-100, Required)
- max_score (Integer, Required)
- started_at (DateTime)
- completed_at (DateTime, Required)
- time_taken_seconds (Integer, Required)
- passed (Boolean, Required)
- feedback (Text, Optional)

**Relationships**:
- Many-to-One: StudentAssessmentResult → User
- Many-to-One: StudentAssessmentResult → ROS2Assessment
- One-to-Many: StudentAssessmentResult → StudentAssessmentResponse

**Validation Rules**:
- Total score must be between 0 and max score
- Completion time required

### StudentAssessmentResponse
**Description**: Student's specific responses to assessment questions
**Fields**:
- id (UUID, Primary Key)
- result_id (UUID, Foreign Key to StudentAssessmentResult, Required)
- question_id (UUID, Foreign Key to ROS2AssessmentQuestion, Required)
- response_text (Text, Required)
- is_correct (Boolean, Required)
- points_awarded (Integer, Required)
- feedback (Text, Optional)

**Relationships**:
- Many-to-One: StudentAssessmentResponse → StudentAssessmentResult
- Many-to-One: StudentAssessmentResponse → ROS2AssessmentQuestion

**Validation Rules**:
- Points awarded cannot exceed question's point value

### StudentProjectSubmission
**Description**: Student's submission for ROS 2 mini-projects or final project
**Fields**:
- id (UUID, Primary Key)
- student_id (UUID, Foreign Key to User, Required)
- project_id (UUID, Foreign Key to ROS2MiniProject, Required)
- submission_content (Text, Required)
- file_attachments (JSON Array, Optional - {filename: string, path: string})
- submitted_at (DateTime, Required)
- evaluated_at (DateTime, Optional)
- evaluator_id (UUID, Foreign Key to User, Optional)
- score (Integer, 0-100, Optional)
- feedback (Text, Optional)
- status (Enum: 'submitted', 'evaluated', 'revision_requested', Required, Default: 'submitted')

**Relationships**:
- Many-to-One: StudentProjectSubmission → User (student)
- Many-to-One: StudentProjectSubmission → ROS2MiniProject
- Many-to-One: StudentProjectSubmission → User (evaluator, optional)

**Validation Rules**:
- Submission content or file attachments required
- Score between 0-100 if evaluated

### URDFModel
**Description**: Represents URDF models created for humanoid robots in the module
**Fields**:
- id (UUID, Primary Key)
- student_id (UUID, Foreign Key to User, Required)
- name (String, Required)
- description (Text, Optional)
- urdf_content (Text, Required)
- created_at (DateTime)
- updated_at (DateTime)
- is_shared (Boolean, Default: false)
- evaluation_score (Integer, 0-100, Optional)
- evaluation_feedback (Text, Optional)

**Relationships**:
- Many-to-One: URDFModel → User
- One-to-Many: URDFModel → URDFModelAsset

**Validation Rules**:
- URDF content must be valid XML format
- Name required

### URDFModelAsset
**Description**: Assets associated with URDF models (mesh files, textures, etc.)
**Fields**:
- id (UUID, Primary Key)
- urdf_model_id (UUID, Foreign Key to URDFModel, Required)
- asset_type (Enum: 'mesh', 'texture', 'material', 'configuration', Required)
- filename (String, Required)
- file_path (String, Required)
- file_size_bytes (Integer, Required)
- mime_type (String, Required)
- checksum (String, Required for integrity verification)
- uploaded_at (DateTime)

**Relationships**:
- Many-to-One: URDFModelAsset → URDFModel

**Validation Rules**:
- File size limits based on asset type
- Valid MIME type for asset type
- Checksum required for integrity

### ROS2AIPrompt
**Description**: Prompts and queries sent to the AI tutor for ROS 2 assistance
**Fields**:
- id (UUID, Primary Key)
- student_id (UUID, Foreign Key to User, Required)
- module_id (UUID, Foreign Key to ROS2Module, Required)
- week_id (UUID, Foreign Key to ROS2Week, Optional)
- query_text (Text, Required)
- query_context (JSON, Optional - {lab_env: string, error_message: string, code_snippet: string})
- response_text (Text, Required)
- response_timestamp (DateTime, Required)
- response_confidence (Float, 0.0-1.0, Optional)
- was_helpful (Boolean, Optional)
- feedback_rating (Integer, 1-5, Optional)
- feedback_text (Text, Optional)

**Relationships**:
- Many-to-One: ROS2AIPrompt → User
- Many-to-One: ROS2AIPrompt → ROS2Module
- Many-to-One: ROS2AIPrompt → ROS2Week

**Validation Rules**:
- Query and response text required
- Confidence between 0.0-1.0 if provided
- Rating between 1-5 if provided

## State Transitions

### StudentModuleProgress States
- `not_started` → `in_progress` → `completed`

### StudentWeekProgress States  
- `not_started` → `in_progress` → `completed`

### StudentSectionProgress States
- `not_started` → `in_progress` → `completed`

### StudentLabSession States
- `active` → `completed` | `terminated`

### StudentProjectSubmission States
- `submitted` → `evaluated` | `revision_requested`

## Indexes

### Performance Indexes
- StudentModuleProgress.student_id + StudentModuleProgress.module_id (composite)
- StudentWeekProgress.student_id + StudentWeekProgress.week_id (composite)
- StudentLabSession.student_id + StudentLabSession.lab_env_id (composite)
- ROS2AIPrompt.student_id + ROS2AIPrompt.response_timestamp (composite)

### Search Indexes
- ROS2AIPrompt.query_text (full-text for pattern analysis)
- URDFModel.urdf_content (full-text for content search)
- ROS2ContentSection.content (full-text for knowledge base)