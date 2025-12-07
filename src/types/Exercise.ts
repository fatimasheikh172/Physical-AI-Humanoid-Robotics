export interface Exercise {
  id: string;
  title: string;
  description: string;
  type: 'multiple-choice' | 'coding' | 'simulation' | 'project';
  content: string; // Exercise content in Markdown/MDX
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  points: number; // Points for completion
  order: number; // Order within chapter
  hints: string[]; // Helpful hints
  solution: string; // Solution for the exercise
  createdAt: string; // ISO date string
  updatedAt: string; // ISO date string
}