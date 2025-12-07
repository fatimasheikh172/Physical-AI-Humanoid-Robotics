export interface Progress {
  id: string;
  userId: string; // Reference to User
  chapterId: string; // Reference to Chapter
  status: 'not-started' | 'in-progress' | 'completed';
  completionPercentage: number; // 0-100
  timeSpent: number; // Seconds spent on chapter
  lastAccessed: string; // ISO date string
  exercisesCompleted: number;
  exercisesTotal: number;
  simulationsInteracted: number;
  createdAt: string; // ISO date string
  updatedAt: string; // ISO date string
}