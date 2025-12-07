export interface Simulation {
  id: string;
  title: string;
  description: string;
  model3D: string; // path to 3D model file
  config: Record<string, any>;
  environment: string;
  complexity: 'low' | 'medium' | 'high';
  interactivityLevel: number; // 0-10 scale
  chapterId?: string;
  createdAt: string;
  updatedAt: string;
}