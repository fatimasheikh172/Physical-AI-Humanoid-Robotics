import apiClient from './api';
import { Progress } from '../types/Progress';

class ProgressService {
  // Get progress for all chapters for the authenticated user
  async getAllProgress(): Promise<{ data: Progress[] }> {
    try {
      return await apiClient.get('/progress');
    } catch (error) {
      console.error('Error fetching progress:', error);
      throw new Error(`Failed to fetch progress: ${(error as Error).message}`);
    }
  }

  // Get progress for a specific chapter
  async getProgressByChapter(chapterId: string): Promise<Progress[]> {
    try {
      return await apiClient.get(`/progress?chapterId=${chapterId}`);
    } catch (error) {
      console.error(`Error fetching progress for chapter ${chapterId}:`, error);
      throw new Error(`Failed to fetch progress for chapter: ${(error as Error).message}`);
    }
  }

  // Update progress for a chapter
  async updateProgress(progressData: Partial<Progress>): Promise<Progress> {
    try {
      return await apiClient.post('/progress', progressData);
    } catch (error) {
      console.error('Error updating progress:', error);
      throw new Error(`Failed to update progress: ${(error as Error).message}`);
    }
  }

  // Calculate completion percentage based on exercises and simulations
  calculateCompletionPercentage(
    exercisesCompleted: number,
    exercisesTotal: number,
    simulationsInteracted: number,
    totalSimulations: number
  ): number {
    if (exercisesTotal === 0 && totalSimulations === 0) {
      return 0;
    }

    const exercisePercentage = exercisesTotal > 0 ? (exercisesCompleted / exercisesTotal) * 50 : 0;
    const simulationPercentage = totalSimulations > 0 ? (simulationsInteracted / totalSimulations) * 50 : 0;

    return Math.round(exercisePercentage + simulationPercentage);
  }

  // Update completion status based on percentage
  getCompletionStatus(percentage: number): 'not-started' | 'in-progress' | 'completed' {
    if (percentage === 0) {
      return 'not-started';
    } else if (percentage < 100) {
      return 'in-progress';
    } else {
      return 'completed';
    }
  }
}

export default new ProgressService();