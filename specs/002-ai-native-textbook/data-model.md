# Data Model: AI-Native Textbook System

**Feature**: AI-Native Textbook System
**Date**: 2025-12-07
**Modeler**: AI Assistant

## Overview

This document defines the data models for the AI-Native Textbook System. The system includes 7 major sections covering Physical AI Foundations, ROS 2, Gazebo & Unity, NVIDIA Isaac, VLA, Conversational Robotics, and a Capstone Autonomous Humanoid project, with associated user management, personalization, and AI tutoring capabilities.

## Entity Models

### User
**Description**: Represents a student or instructor using the textbook system
**Fields**:
- id (UUID, Primary Key)
- email (String, Unique, Required)
- name (String, Required)
- password_hash (String, Required for registered users)
- auth_provider (String, Optional - for SSO users)
- auth_provider_id (String, Optional - for SSO users)
- role (Enum: 'student', 'instructor', 'admin', Default: 'student')
- programming_experience (Enum: 'beginner', 'intermediate', 'advanced', Required)
- ai_ml_background (Enum: 'none', 'basic', 'intermediate', 'advanced', Required)
- hardware_access (JSON, Optional - {rtx_pc: boolean, jetson: boolean, robot: boolean})
- learning_goals (Text Array, Optional)
- created_at (DateTime)
- updated_at (DateTime)
- last_login_at (DateTime)
- is_active (Boolean, Default: true)
- privacy_consent (Boolean, Required for GDPR compliance)

**Relationships**:
- One-to-Many: User → UserProgress
- One-to-Many: User → UserAssessment
- One-to-Many: User → AIInteraction
- One-to-Many: User → ChapterDownload

**Validation Rules**:
- Email must be valid format
- Programming experience and AI/ML background required at registration
- Privacy consent required for GDPR compliance

### Chapter
**Description**: Represents a textbook chapter with multiple sections of content
**Fields**:
- id (UUID, Primary Key)
- title (String, Required)
- slug (String, Unique, Required for URLs)
- description (Text, Optional)
- module_number (Integer, Required - 1-7)
- order_index (Integer, Required within module)
- difficulty_level (Enum: 'beginner', 'intermediate', 'advanced', Required)
- estimated_reading_time_minutes (Integer, Required)
- content_format (Enum: 'textbook', 'lab', 'project', 'assessment', Required)
- content_metadata (JSON, Optional - {code_examples: integer, images: integer, etc.})
- is_published (Boolean, Default: false)
- created_at (DateTime)
- updated_at (DateTime)
- published_at (DateTime, Optional)
- version (Integer, Default: 1)

**Relationships**:
- One-to-Many: Chapter → Section
- One-to-Many: Chapter → Quiz
- One-to-Many: Chapter → UserProgress
- One-to-Many: Chapter → ChapterDownload
- One-to-Many: Chapter → AIKnowledgeBase (for RAG system)

**Validation Rules**:
- Title and slug required
- Module number must be 1-7
- Order index must be unique within module

### Section
**Description**: A subsection within a chapter (e.g., theory, code example, lab task)
**Fields**:
- id (UUID, Primary Key)
- chapter_id (UUID, Foreign Key to Chapter, Required)
- title (String, Required)
- content_type (Enum: 'theory', 'code_example', 'lab_task', 'mini_project', 'quiz', 'interview_q', 'ai_tutor', Required)
- content (Text, Required)
- order_index (Integer, Required within chapter)
- estimated_completion_time_minutes (Integer, Required)
- prerequisite_section_ids (UUID Array, Optional)
- created_at (DateTime)
- updated_at (DateTime)

**Relationships**:
- Many-to-One: Section → Chapter
- One-to-Many: Section → ContentAsset
- One-to-Many: Section → UserProgress

**Validation Rules**:
- Content type must be one of allowed values
- Order index must be unique within chapter
- Content required

### ContentAsset
**Description**: Assets associated with content sections (images, code files, etc.)
**Fields**:
- id (UUID, Primary Key)
- section_id (UUID, Foreign Key to Section, Required)
- asset_type (Enum: 'image', 'code', 'video', 'audio', 'pdf', Required)
- filename (String, Required)
- original_filename (String, Required)
- file_path (String, Required)
- file_size_bytes (Integer, Required)
- mime_type (String, Required)
- alt_text (String, Optional for images)
- checksum (String, Required for integrity verification)
- uploaded_at (DateTime)
- is_public (Boolean, Default: true)

**Relationships**:
- Many-to-One: ContentAsset → Section

**Validation Rules**:
- File size limits based on asset type
- Valid MIME type for asset type
- Checksum required for integrity

### UserProgress
**Description**: Tracks user's progress through chapters and sections
**Fields**:
- id (UUID, Primary Key)
- user_id (UUID, Foreign Key to User, Required)
- chapter_id (UUID, Foreign Key to Chapter, Required)
- section_id (UUID, Foreign Key to Section, Optional)
- progress_percentage (Integer, 0-100, Required)
- started_at (DateTime)
- completed_at (DateTime, Optional)
- last_accessed_at (DateTime)
- time_spent_seconds (Integer, Default: 0)
- quiz_scores (JSON, Optional - {quiz_id: score})
- lab_completion_status (JSON, Optional - {lab_id: status})
- is_completed (Boolean, Default: false)

**Relationships**:
- Many-to-One: UserProgress → User
- Many-to-One: UserProgress → Chapter
- Many-to-One: UserProgress → Section

**Validation Rules**:
- Progress percentage between 0-100
- Started at required
- Can only have one record per user-chapter combination

### Quiz
**Description**: Assessment quiz associated with a chapter
**Fields**:
- id (UUID, Primary Key)
- chapter_id (UUID, Foreign Key to Chapter, Required)
- title (String, Required)
- description (Text, Optional)
- question_count (Integer, Required)
- time_limit_minutes (Integer, Optional)
- passing_score_percentage (Integer, 0-100, Required, Default: 70)
- created_at (DateTime)
- updated_at (DateTime)

**Relationships**:
- Many-to-One: Quiz → Chapter
- One-to-Many: Quiz → QuizQuestion
- One-to-Many: Quiz → UserAssessment

**Validation Rules**:
- Question count must be positive
- Passing score between 0-100

### QuizQuestion
**Description**: Individual question within a quiz
**Fields**:
- id (UUID, Primary Key)
- quiz_id (UUID, Foreign Key to Quiz, Required)
- question_text (Text, Required)
- question_type (Enum: 'multiple_choice', 'true_false', 'short_answer', 'code', Required)
- options (JSON Array, Optional - for multiple choice)
- correct_answer (Text, Required)
- explanation (Text, Optional)
- difficulty_level (Enum: 'beginner', 'intermediate', 'advanced', Required)
- order_index (Integer, Required within quiz)

**Relationships**:
- Many-to-One: QuizQuestion → Quiz
- One-to-Many: QuizQuestion → UserAssessment

**Validation Rules**:
- Question text required
- For multiple choice, options must be provided

### UserAssessment
**Description**: User's responses and scores for quizzes
**Fields**:
- id (UUID, Primary Key)
- user_id (UUID, Foreign Key to User, Required)
- quiz_id (UUID, Foreign Key to Quiz, Required)
- quiz_question_id (UUID, Foreign Key to QuizQuestion, Optional)
- answer_text (Text, Optional)
- is_correct (Boolean, Optional)
- score (Integer, 0-100, Optional)
- time_taken_seconds (Integer, Required)
- submitted_at (DateTime)
- feedback (Text, Optional)

**Relationships**:
- Many-to-One: UserAssessment → User
- Many-to-One: UserAssessment → Quiz
- Many-to-One: UserAssessment → QuizQuestion

**Validation Rules**:
- Submission time required

### AIInteraction
**Description**: Record of AI tutor interactions with users
**Fields**:
- id (UUID, Primary Key)
- user_id (UUID, Foreign Key to User, Required)
- chapter_id (UUID, Foreign Key to Chapter, Optional)
- section_id (UUID, Foreign Key to Section, Optional)
- query_text (Text, Required)
- response_text (Text, Required)
- response_accuracy_confidence (Float, 0.0-1.0, Optional)
- response_source_chunks (JSON, Optional - references to content used)
- interaction_timestamp (DateTime, Required)
- feedback_rating (Integer, 1-5, Optional)
- feedback_text (Text, Optional)
- is_helpful (Boolean, Optional)

**Relationships**:
- Many-to-One: AIInteraction → User
- Many-to-One: AIInteraction → Chapter
- Many-to-One: AIInteraction → Section

**Validation Rules**:
- Query and response text required
- Confidence between 0.0-1.0 if provided
- Rating between 1-5 if provided

### AIKnowledgeBase
**Description**: Vectorized content for RAG system to power the AI tutor
**Fields**:
- id (UUID, Primary Key)
- chapter_id (UUID, Foreign Key to Chapter, Required)
- section_id (UUID, Foreign Key to Section, Optional)
- content_chunk (Text, Required)
- content_vector (Vector, Required for semantic search)
- token_count (Integer, Required)
- created_at (DateTime)
- updated_at (DateTime)

**Relationships**:
- Many-to-One: AIKnowledgeBase → Chapter
- Many-to-One: AIKnowledgeBase → Section

**Validation Rules**:
- Content chunk required
- Vector representation required
- Token count positive

### ChapterDownload
**Description**: Tracks offline downloads of chapters by users
**Fields**:
- id (UUID, Primary Key)
- user_id (UUID, Foreign Key to User, Required)
- chapter_id (UUID, Foreign Key to Chapter, Required)
- download_timestamp (DateTime, Required)
- sync_status (Enum: 'downloaded', 'synced', 'outdated', 'deleted', Default: 'downloaded')
- sync_timestamp (DateTime, Optional)
- download_size_bytes (Integer, Required)

**Relationships**:
- Many-to-One: ChapterDownload → User
- Many-to-One: ChapterDownload → Chapter

**Validation Rules**:
- Download timestamp required
- Size must be positive

### PersonalizationProfile
**Description**: Stores personalized content adaptation for each user
**Fields**:
- id (UUID, Primary Key)
- user_id (UUID, Foreign Key to User, Required)
- chapter_id (UUID, Foreign Key to Chapter, Required)
- adapted_content_level (Enum: 'beginner', 'intermediate', 'advanced', Required)
- adapted_project_recommendations (JSON, Optional - {project_id: priority})
- learning_path_sequence (UUID Array, Optional - sequence of section IDs)
- created_at (DateTime)
- updated_at (DateTime)

**Relationships**:
- Many-to-One: PersonalizationProfile → User
- Many-to-One: PersonalizationProfile → Chapter

**Validation Rules**:
- Content level required
- User and chapter must exist

## State Transitions

### UserProgress States
- `incomplete` (0-99% progress) → `completed` (100% progress)
- `active` (currently being worked on) → `inactive` (not accessed recently)

### AIInteraction Feedback Cycle
- `new` → `answered` → `rated` → `improved` (when response is enhanced based on feedback)

### ChapterDownload Sync Status
- `downloaded` → `synced` → `outdated` → `synced`
- `downloaded` → `deleted` (when user removes offline content)

## Indexes

### Performance Indexes
- User.email (unique)
- UserProgress.user_id + UserProgress.chapter_id (composite)
- Chapter.slug (unique)
- Chapter.module_number + Chapter.order_index (composite)
- AIKnowledgeBase.content_vector (vector index for similarity search)
- UserAssessment.user_id + UserAssessment.quiz_id (composite)

### Search Indexes
- QuizQuestion.question_text (full-text)
- AIKnowledgeBase.content_chunk (full-text for semantic search)
- Chapter.title + Chapter.description (full-text)