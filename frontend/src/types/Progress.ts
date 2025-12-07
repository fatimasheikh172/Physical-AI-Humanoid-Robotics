export interface Progress {
  id: string;
  userId: string;
  chapterId: string;
  status: 'not-started' | 'in-progress' | 'completed';
  completionPercentage: number; // 0-100
  timeSpent: number; // seconds spent on chapter
  lastAccessed: string;
  exercisesCompleted: number;
  exercisesTotal: number;
  simulationsInteracted: number;
  createdAt: string;
  updatedAt: string;
}