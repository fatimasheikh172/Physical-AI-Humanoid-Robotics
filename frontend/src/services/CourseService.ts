import apiClient from '../services/api';
import { Course } from '../types/Course';
import { User } from '../types/User';

// Course API service
export const CourseService = {
  // Create a new course
  create: async (courseData: Omit<Course, 'id' | 'createdAt' | 'updatedAt' | 'enrollmentCode'>): Promise<Course> => {
    return await apiClient.post('/courses', courseData);
  },

  // Get all courses for the current instructor
  getAll: async (): Promise<Course[]> => {
    // In a real implementation, this would filter to courses the instructor owns
    const response = await apiClient.get('/courses');
    return response.data || [];
  },

  // Get students in a course
  getStudents: async (courseId: string): Promise<{data: User[]}> => {
    return await apiClient.get(`/courses/${courseId}/students`);
  },

  // Update a course
  update: async (courseId: string, courseData: Partial<Course>): Promise<Course> => {
    return await apiClient.put(`/courses/${courseId}`, courseData);
  },

  // Delete a course
  delete: async (courseId: string): Promise<void> => {
    await apiClient.delete(`/courses/${courseId}`);
  },
};