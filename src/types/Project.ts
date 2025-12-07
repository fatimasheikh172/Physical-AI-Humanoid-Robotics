export interface Project {
  id: string;
  title: string;
  description: string;
  requirements: string; // Detailed requirements
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  estimatedDuration: number; // Hours to complete
  resources: string[]; // File paths to resources
  deliverables: string[]; // Expected deliverables
  evaluationCriteria: string[]; // Grading criteria
  createdAt: string; // ISO date string
  updatedAt: string; // ISO date string
}