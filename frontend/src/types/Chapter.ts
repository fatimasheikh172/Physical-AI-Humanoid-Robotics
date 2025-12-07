export interface Chapter {
  id: string;
  title: string;
  slug: string;
  content: string;
  order: number;
  estimatedTime: number;
  prerequisites: string[];
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  language: string;
  exercises?: Exercise[];
  simulations?: Simulation[];
  createdAt: string;
  updatedAt: string;
}

export interface ChapterProgress {
  id: string;
  userId: string;
  chapterId: string;
  status: 'not-started' | 'in-progress' | 'completed';
  completionPercentage: number;
  timeSpent: number;
  lastAccessed: string;
  exercisesCompleted: number;
  exercisesTotal: number;
  simulationsInteracted: number;
  createdAt: string;
  updatedAt: string;
}