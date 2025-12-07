# Data Model: Frontend Interface for Physical AI & Humanoid Robotics Platform

**Feature**: Frontend Interface for Physical AI & Humanoid Robotics Platform
**Date**: 2025-12-07
**Modeler**: AI Assistant

## Overview

This document defines the data models for the frontend interface of the Physical AI & Humanoid Robotics platform. The frontend primarily handles user interface state, content organization, and communication with backend services. The models focus on content structure, user interactions, and integration with AI tutor functionality.

## Entity Models

### Module
**Description**: Represents a major section of the Physical AI & Humanoid Robotics textbook
**Fields**:
- id (String, Primary Key)
- title (String, Required)
- description (Text, Required)
- order_index (Integer, Required, Unique within platform)
- duration_weeks (Integer, Required)
- learning_objectives (Text Array, Required)
- prerequisites (Text Array, Optional)
- is_published (Boolean, Default: false)
- created_at (DateTime)
- updated_at (DateTime)

**Relationships**:
- One-to-Many: Module → Chapter
- One-to-Many: Module → UserProgress (tracking user progress through the module)

**Validation Rules**:
- Title and description required
- Order index must be unique within platform
- Duration must be positive
- Learning objectives must be provided

### Chapter
**Description**: Represents a chapter within a module of the textbook
**Fields**:
- id (String, Primary Key)
- module_id (String, Foreign Key to Module, Required)
- title (String, Required)
- description (Text, Required)
- order_index (Integer, Required, Unique within module)
- duration_minutes (Integer, Required)
- learning_objectives (Text Array, Required)
- content_type (Enum: 'theory', 'lab', 'project', 'quiz', Required)
- markdown_content (Text, Required for theory chapters)
- lab_instructions (JSON, Required for lab chapters - {setup: string, steps: array, goals: array})
- project_requirements (JSON, Required for project chapters - {requirements: array, evaluation: object})
- is_published (Boolean, Default: false)
- created_at (DateTime)
- updated_at (DateTime)

**Relationships**:
- Many-to-One: Chapter → Module
- One-to-Many: Chapter → Lesson
- One-to-Many: Chapter → UserChapterProgress

**Validation Rules**:
- Title, description, and content/structure required based on content_type
- Order index must be unique within module
- Duration must be positive

### Lesson
**Description**: Represents a smaller section within a chapter, particularly used for theory chapters
**Fields**:
- id (String, Primary Key)
- chapter_id (String, Foreign Key to Chapter, Required)
- title (String, Required)
- content (Text, Required)
- order_index (Integer, Required, Unique within chapter)
- estimated_reading_time_minutes (Integer, Required)
- is_interactive (Boolean, Default: false)
- interactive_component (JSON, Optional - {type: string, props: object})
- created_at (DateTime)
- updated_at (DateTime)

**Relationships**:
- Many-to-One: Lesson → Chapter
- One-to-Many: Lesson → UserLessonProgress

**Validation Rules**:
- Title and content required
- Order index must be unique within chapter
- Reading time must be positive

### Quiz
**Description**: Represents a quiz assessment for a chapter
**Fields**:
- id (String, Primary Key)
- chapter_id (String, Foreign Key to Chapter, Required)
- title (String, Required)
- description (Text, Required)
- questions_count (Integer, Required)
- time_limit_minutes (Integer, Optional)
- passing_score_percentage (Integer, 0-100, Required, Default: 70)
- created_at (DateTime)
- updated_at (DateTime)

**Relationships**:
- Many-to-One: Quiz → Chapter
- One-to-Many: Quiz → QuizQuestion
- One-to-Many: Quiz → UserQuizResult

**Validation Rules**:
- Title and description required
- Questions count must be positive
- Passing score between 0-100

### QuizQuestion
**Description**: Individual question within a quiz
**Fields**:
- id (String, Primary Key)
- quiz_id (String, Foreign Key to Quiz, Required)
- question_text (Text, Required)
- question_type (Enum: 'multiple_choice', 'true_false', 'short_answer', 'code', 'practical', Required)
- options (JSON Array, Optional - for multiple choice)
- correct_answer (Text, Required)
- explanation (Text, Optional)
- difficulty_level (Enum: 'beginner', 'intermediate', 'advanced', Required)
- points (Integer, Required, Default: 1)
- order_index (Integer, Required within quiz)

**Relationships**:
- Many-to-One: QuizQuestion → Quiz
- One-to-Many: QuizQuestion → UserQuizResponse

**Validation Rules**:
- Question text required
- For multiple choice, options must be provided
- Points must be positive

### User
**Description**: Represents a student or instructor using the platform
**Fields**:
- id (String, Primary Key)
- email (String, Unique, Required)
- name (String, Required)
- password_hash (String, Required for registered users)
- role (Enum: 'student', 'instructor', 'admin', Default: 'student')
- programming_experience (Enum: 'beginner', 'intermediate', 'advanced', Required)
- ai_ml_background (Enum: 'none', 'basic', 'intermediate', 'advanced', Required)
- hardware_access (JSON, Optional - {rtx_pc: boolean, jetson: boolean, robot: boolean})
- learning_goals (Text Array, Optional)
- created_at (DateTime)
- updated_at (DateTime)
- last_login_at (DateTime)
- is_active (Boolean, Default: true)
- preferences (JSON, Optional - {ui_theme: string, language: string, content_language: string})

**Relationships**:
- One-to-Many: User → UserModuleProgress
- One-to-Many: User → UserChapterProgress
- One-to-Many: User → UserLessonProgress
- One-to-Many: User → UserQuizResult
- One-to-Many: User → UserLabSession
- One-to-Many: User → UserProjectSubmission
- One-to-Many: User → AITutorInteraction

**Validation Rules**:
- Email must be valid format
- Programming experience and AI/ML background required at registration

### UserModuleProgress
**Description**: Tracks student's progress through a module
**Fields**:
- id (String, Primary Key)
- user_id (String, Foreign Key to User, Required)
- module_id (String, Foreign Key to Module, Required)
- progress_percentage (Integer, 0-100, Required)
- started_at (DateTime)
- completed_at (DateTime, Optional)
- last_accessed_at (DateTime)
- time_spent_seconds (Integer, Default: 0)
- chapter_progress (JSON, Optional - {chapter_id: completion_percentage})
- quiz_scores (JSON, Optional - {quiz_id: score})
- project_submissions_status (JSON, Optional - {project_id: status})
- is_completed (Boolean, Default: false)

**Relationships**:
- Many-to-One: UserModuleProgress → User
- Many-to-One: UserModuleProgress → Module
- One-to-Many: UserModuleProgress → UserChapterProgress

**Validation Rules**:
- Progress percentage between 0-100
- Started at required
- Can only have one record per user-module combination

### UserChapterProgress
**Description**: Tracks student's progress through an individual chapter
**Fields**:
- id (String, Primary Key)
- user_id (String, Foreign Key to User, Required)
- chapter_id (String, Foreign Key to Chapter, Required)
- progress_percentage (Integer, 0-100, Required)
- started_at (DateTime)
- completed_at (DateTime, Optional)
- last_accessed_at (DateTime)
- time_spent_seconds (Integer, Default: 0)
- lesson_progress (JSON, Optional - {lesson_id: completion_status})
- quiz_completion_status (JSON, Optional - {quiz_id: status})
- lab_completion_status (JSON, Optional - {lab_id: status})
- is_completed (Boolean, Default: false)

**Relationships**:
- Many-to-One: UserChapterProgress → User
- Many-to-One: UserChapterProgress → Chapter
- One-to-Many: UserChapterProgress → UserLessonProgress

**Validation Rules**:
- Progress percentage between 0-100
- Started at required
- Can only have one record per user-chapter combination

### UserLessonProgress
**Description**: Tracks student's progress through individual lessons
**Fields**:
- id (String, Primary Key)
- user_id (String, Foreign Key to User, Required)
- lesson_id (String, Foreign Key to Lesson, Required)
- started_at (DateTime)
- completed_at (DateTime, Optional)
- time_spent_seconds (Integer, Default: 0)
- completion_status (Enum: 'not_started', 'in_progress', 'completed', Required, Default: 'not_started')
- notes (Text, Optional)

**Relationships**:
- Many-to-One: UserLessonProgress → User
- Many-to-One: UserLessonProgress → Lesson

**Validation Rules**:
- Completion status must be one of allowed values

### UserQuizResult
**Description**: Student's results for chapter quizzes
**Fields**:
- id (String, Primary Key)
- user_id (String, Foreign Key to User, Required)
- quiz_id (String, Foreign Key to Quiz, Required)
- total_score (Integer, 0-100, Required)
- max_score (Integer, Required)
- started_at (DateTime)
- completed_at (DateTime, Required)
- time_taken_seconds (Integer, Required)
- passed (Boolean, Required)
- feedback (Text, Optional)

**Relationships**:
- Many-to-One: UserQuizResult → User
- Many-to-One: UserQuizResult → Quiz
- One-to-Many: UserQuizResult → UserQuizResponse

**Validation Rules**:
- Total score must be between 0 and max score
- Completion time required

### UserQuizResponse
**Description**: Student's specific responses to quiz questions
**Fields**:
- id (String, Primary Key)
- result_id (String, Foreign Key to UserQuizResult, Required)
- question_id (String, Foreign Key to QuizQuestion, Required)
- response_text (Text, Required)
- is_correct (Boolean, Required)
- points_awarded (Integer, Required)
- feedback (Text, Optional)

**Relationships**:
- Many-to-One: UserQuizResponse → UserQuizResult
- Many-to-One: UserQuizResponse → QuizQuestion

**Validation Rules**:
- Points awarded cannot exceed question's point value

### UserLabSession
**Description**: Records of student's lab sessions
**Fields**:
- id (String, Primary Key)
- user_id (String, Foreign Key to User, Required)
- chapter_id (String, Foreign Key to Chapter, Required)
- session_start (DateTime, Required)
- session_end (DateTime, Optional)
- duration_seconds (Integer, Optional)
- lab_exercise_id (String, Optional)
- code_submissions (JSON Array, Optional - {code: string, timestamp: datetime, feedback: string})
- errors_encountered (Text Array, Optional)
- ai_tutor_requests (Integer, Default: 0)
- status (Enum: 'active', 'completed', 'terminated', Required)
- session_notes (Text, Optional)

**Relationships**:
- Many-to-One: UserLabSession → User
- Many-to-One: UserLabSession → Chapter
- One-to-Many: UserLabSession → LabSessionArtifact

**Validation Rules**:
- Session start required
- Duration must be positive if session completed

### LabSessionArtifact
**Description**: Artifacts generated during lab sessions (code, logs, etc.)
**Fields**:
- id (String, Primary Key)
- lab_session_id (String, Foreign Key to UserLabSession, Required)
- artifact_type (Enum: 'code_output', 'console_log', 'image', 'video', 'generated_file', Required)
- filename (String, Required)
- file_path (String, Required)
- file_size_bytes (Integer, Required)
- mime_type (String, Required)
- description (Text, Optional)
- created_at (DateTime)

**Relationships**:
- Many-to-One: LabSessionArtifact → UserLabSession

**Validation Rules**:
- File size limits based on artifact type
- Valid MIME type for artifact type

### UserProjectSubmission
**Description**: Student's submission for chapter projects
**Fields**:
- id (String, Primary Key)
- user_id (String, Foreign Key to User, Required)
- chapter_id (String, Foreign Key to Chapter, Required)
- submission_content (Text, Required)
- file_attachments (JSON Array, Optional - {filename: string, path: string})
- submitted_at (DateTime, Required)
- evaluated_at (DateTime, Optional)
- evaluator_id (String, Foreign Key to User, Optional)
- score (Integer, 0-100, Optional)
- feedback (Text, Optional)
- status (Enum: 'submitted', 'evaluated', 'revision_requested', Required, Default: 'submitted')

**Relationships**:
- Many-to-One: UserProjectSubmission → User (student)
- Many-to-One: UserProjectSubmission → Chapter
- Many-to-One: UserProjectSubmission → User (evaluator, optional)

**Validation Rules**:
- Submission content or file attachments required
- Score between 0-100 if evaluated

### AITutorInteraction
**Description**: Prompts and queries sent to the AI tutor for assistance
**Fields**:
- id (String, Primary Key)
- user_id (String, Foreign Key to User, Required)
- module_id (String, Foreign Key to Module, Optional)
- chapter_id (String, Foreign Key to Chapter, Optional)
- lesson_id (String, Foreign Key to Lesson, Optional)
- query_text (Text, Required)
- query_context (JSON, Optional - {page_section: string, code_snippet: string, error_message: string})
- response_text (Text, Required)
- response_timestamp (DateTime, Required)
- response_confidence (Float, 0.0-1.0, Optional)
- was_helpful (Boolean, Optional)
- feedback_rating (Integer, 1-5, Optional)
- feedback_text (Text, Optional)

**Relationships**:
- Many-to-One: AITutorInteraction → User
- Many-to-One: AITutorInteraction → Module (optional)
- Many-to-One: AITutorInteraction → Chapter (optional)
- Many-to-One: AITutorInteraction → Lesson (optional)

**Validation Rules**:
- Query and response text required
- Confidence between 0.0-1.0 if provided
- Rating between 1-5 if provided

## State Transitions

### UserModuleProgress States
- `not_started` (0%) → `in_progress` (1-99%) → `completed` (100%)

### UserChapterProgress States
- `not_started` (0%) → `in_progress` (1-99%) → `completed` (100%)

### UserLessonProgress States
- `not_started` → `in_progress` → `completed`

### UserLabSession States
- `active` → `completed` | `terminated`

### UserProjectSubmission States
- `submitted` → `evaluated` | `revision_requested`

### AITutorInteraction States
- `new_query` → `response_provided` → `feedback_given` (optional)

## Indexes

### Performance Indexes
- UserModuleProgress.user_id + UserModuleProgress.module_id (composite)
- UserChapterProgress.user_id + UserChapterProgress.chapter_id (composite)
- UserLessonProgress.user_id + UserLessonProgress.lesson_id (composite)
- AITutorInteraction.user_id + AITutorInteraction.response_timestamp (composite)

### Search Indexes
- AITutorInteraction.query_text (full-text for pattern analysis)
- Chapter.markdown_content (full-text for knowledge base)
- Lesson.content (full-text for knowledge base)