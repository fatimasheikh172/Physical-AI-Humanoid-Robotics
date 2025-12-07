# Data Model: Physical AI & Humanoid Robotics Textbook

## User Entity
- **Attributes**:
  - id: string (unique identifier)
  - email: string (user email, unique)
  - name: string (display name)
  - role: enum (student | instructor)
  - preferences: object
    - language: string (default: 'en', supports 'ur' for Urdu)
    - theme: string (light | dark)
    - accessibility: object (screenReader, reducedMotion, etc.)
  - createdAt: datetime
  - lastLogin: datetime

- **Relationships**:
  - has many: Progress records
  - has many: Project submissions
  - has many: Course enrollments

- **Validation**:
  - email must be valid email format
  - name must be 2-50 characters
  - role must be one of allowed values

## Chapter Entity
- **Attributes**:
  - id: string (unique identifier)
  - title: string (chapter title)
  - slug: string (URL-friendly identifier)
  - content: string (Markdown/MDX content)
  - order: number (sequential order in textbook)
  - estimatedTime: number (minutes to complete)
  - prerequisites: array of Chapter ids
  - difficulty: enum (beginner | intermediate | advanced)
  - language: string (primary language)
  - createdAt: datetime
  - updatedAt: datetime

- **Relationships**:
  - has many: Progress records
  - has many: Exercises
  - has many: Simulations
  - belongs to: Course (optional)

- **Validation**:
  - title must be 5-100 characters
  - slug must be URL-friendly
  - content must be valid Markdown/MDX
  - order must be positive integer

## Simulation Entity
- **Attributes**:
  - id: string (unique identifier)
  - title: string (simulation name)
  - description: string (brief description)
  - model3D: string (path to 3D model file)
  - config: object (simulation configuration)
  - environment: string (simulation environment settings)
  - complexity: enum (low | medium | high)
  - interactivityLevel: number (0-10 scale)
  - createdAt: datetime
  - updatedAt: datetime

- **Relationships**:
  - belongs to: Chapter
  - has many: Simulation sessions
  - has many: Simulation parameters

- **Validation**:
  - title must be 5-100 characters
  - model3D must point to valid 3D asset
  - interactivityLevel must be 0-10

## Progress Entity
- **Attributes**:
  - id: string (unique identifier)
  - userId: string (reference to User)
  - chapterId: string (reference to Chapter)
  - status: enum (not-started | in-progress | completed)
  - completionPercentage: number (0-100)
  - timeSpent: number (seconds spent on chapter)
  - lastAccessed: datetime
  - exercisesCompleted: number
  - exercisesTotal: number
  - simulationsInteracted: number
  - createdAt: datetime
  - updatedAt: datetime

- **Relationships**:
  - belongs to: User
  - belongs to: Chapter

- **Validation**:
  - userId and chapterId must exist
  - completionPercentage must be 0-100
  - exercisesCompleted must be ≤ exercisesTotal

## Exercise Entity
- **Attributes**:
  - id: string (unique identifier)
  - title: string (exercise title)
  - description: string (exercise instructions)
  - type: enum (multiple-choice | coding | simulation | project)
  - content: string (exercise content in Markdown/MDX)
  - difficulty: enum (beginner | intermediate | advanced)
  - points: number (points for completion)
  - order: number (order within chapter)
  - hints: array of strings (helpful hints)
  - solution: string (solution for the exercise)
  - createdAt: datetime
  - updatedAt: datetime

- **Relationships**:
  - belongs to: Chapter
  - has many: Exercise submissions

- **Validation**:
  - title must be 5-100 characters
  - type must be one of allowed values
  - points must be positive integer

## Project Entity
- **Attributes**:
  - id: string (unique identifier)
  - title: string (project title)
  - description: string (project overview)
  - requirements: string (detailed requirements)
  - difficulty: enum (beginner | intermediate | advanced)
  - estimatedDuration: number (hours to complete)
  - resources: array of strings (file paths to resources)
  - deliverables: array of strings (expected deliverables)
  - evaluationCriteria: array of strings (grading criteria)
  - createdAt: datetime
  - updatedAt: datetime

- **Relationships**:
  - belongs to: Chapter (optional, for chapter-specific projects)
  - belongs to: Course (optional, for course-wide projects)
  - has many: Project submissions

- **Validation**:
  - title must be 5-100 characters

## Course Entity
- **Attributes**:
  - id: string (unique identifier)
  - title: string (course title)
  - description: string (course overview)
  - instructorId: string (reference to instructor User)
  - startDate: date
  - endDate: date
  - status: enum (draft | active | completed)
  - enrollmentCode: string (code for student enrollment)
  - language: string (primary language)
  - createdAt: datetime
  - updatedAt: datetime

- **Relationships**:
  - belongs to: Instructor (User)
  - has many: Chapters
  - has many: Students (Users through enrollment)

- **Validation**:
  - title must be 5-100 characters
  - startDate must be before endDate

## Simulation Session Entity
- **Attributes**:
  - id: string (unique identifier)
  - userId: string (reference to User)
  - simulationId: string (reference to Simulation)
  - startTime: datetime
  - endTime: datetime (nullable, null if session not completed)
  - parameters: object (user-specific simulation parameters)
  - results: object (simulation results/output)
  - createdAt: datetime

- **Relationships**:
  - belongs to: User
  - belongs to: Simulation

- **Validation**:
  - userId and simulationId must exist
  - startTime must be before endTime (if endTime exists)

## AI Tutor Interaction Entity
- **Attributes**:
  - id: string (unique identifier)
  - userId: string (reference to User)
  - chapterId: string (reference to Chapter, nullable)
  - simulationId: string (reference to Simulation, nullable)
  - query: string (user's question)
  - response: string (AI-generated response)
  - timestamp: datetime
  - context: string (textbook content context that triggered the question)
  - rating: number (user rating of the response 1-5, nullable)

- **Relationships**:
  - belongs to: User
  - belongs to: Chapter (optional)
  - belongs to: Simulation (optional)

- **Validation**:
  - query and response must not be empty
  - rating must be 1-5 if provided