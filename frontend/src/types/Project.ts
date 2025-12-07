export interface Project {
  id: string;
  title: string;
  description: string;
  requirements: string;
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  estimatedDuration: number; // hours to complete
  resources: string[]; // file paths to resources
  deliverables: string[]; // expected deliverables
  evaluationCriteria: string[]; // grading criteria
  chapterId?: string; // optional, for chapter-specific projects
  courseId?: string; // optional, for course-wide projects
  createdAt: string;
  updatedAt: string;
}