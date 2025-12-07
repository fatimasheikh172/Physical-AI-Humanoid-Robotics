import apiClient from '../services/api';
import { Chapter, ChapterProgress } from '../types/Chapter';
import { Simulation } from '../types/Simulation';
import { Exercise } from '../types/Exercise';
import { User } from '../types/User';

// Chapter API service
export const ChapterService = {
  // Get all chapters
  getAll: async (): Promise<{ data: Chapter[]; pagination: any }> => {
    return await apiClient.get('/chapters');
  },

  // Get a specific chapter by ID
  getById: async (id: string): Promise<Chapter> => {
    return await apiClient.get(`/chapters/${id}`);
  },
};

// Progress API service
export const ProgressService = {
  // Get progress for all chapters
  getAll: async (): Promise<{ data: ChapterProgress[] }> => {
    return await apiClient.get('/progress');
  },

  // Get progress for a specific chapter
  getByChapterId: async (chapterId: string): Promise<ChapterProgress[]> => {
    return await apiClient.get(`/progress?chapterId=${chapterId}`);
  },

  // Update progress for a chapter
  update: async (progressData: Partial<ChapterProgress>): Promise<ChapterProgress> => {
    return await apiClient.post('/progress', progressData);
  },
};

// Simulation API service
export const SimulationService = {
  // Get all simulations
  getAll: async (): Promise<{ data: Simulation[] }> => {
    return await apiClient.get('/simulations');
  },

  // Get simulations for a specific chapter
  getByChapterId: async (chapterId: string): Promise<Simulation[]> => {
    return await apiClient.get(`/simulations?chapterId=${chapterId}`);
  },

  // Initialize a simulation session
  initializeSession: async (simulationId: string): Promise<any> => {
    return await apiClient.get(`/simulations/${simulationId}/session`);
  },

  // Update a simulation session
  updateSession: async (simulationId: string, sessionId: string, sessionData: any): Promise<any> => {
    return await apiClient.put(`/simulations/${simulationId}/session/${sessionId}`, sessionData);
  },
};

// Exercise API service
export const ExerciseService = {
  // Submit an exercise answer
  submit: async (exerciseId: string, answer: any): Promise<any> => {
    return await apiClient.post(`/exercises/${exerciseId}/submit`, { answer });
  },
};

// User API service
export const UserService = {
  // Get current user profile
  getProfile: async (): Promise<User> => {
    return await apiClient.get('/users/profile');
  },

  // Update user profile
  updateProfile: async (profileData: Partial<User>): Promise<User> => {
    return await apiClient.put('/users/profile', profileData);
  },
};