export interface Exercise {
  id: string;
  title: string;
  description: string;
  type: 'multiple-choice' | 'coding' | 'simulation' | 'project';
  content: string;
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  points: number;
  order: number;
  hints: string[];
  solution: string;
  createdAt: string;
  updatedAt: string;
}