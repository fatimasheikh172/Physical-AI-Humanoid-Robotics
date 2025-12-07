export interface Simulation {
  id: string;
  title: string;
  description: string;
  model3D: string; // Path to 3D model file
  config: Record<string, any>; // Simulation configuration
  environment: string; // Simulation environment settings
  complexity: 'low' | 'medium' | 'high';
  interactivityLevel: number; // 0-10 scale
  createdAt: string; // ISO date string
  updatedAt: string; // ISO date string
}