import apiClient from './api';
import { User } from '../types/User';

class AuthService {
  // Login user
  async login(email: string, password: string): Promise<User> {
    // This would call an actual login endpoint in a real implementation
    // For now, we'll simulate the API call
    try {
      const response = await apiClient.post('/auth/login', { email, password });
      // Save token to localStorage
      if (response.token) {
        localStorage.setItem('authToken', response.token);
      }
      return response.user;
    } catch (error) {
      throw new Error('Login failed: ' + (error as Error).message);
    }
  }

  // Logout user
  logout(): void {
    localStorage.removeItem('authToken');
  }

  // Get current user profile
  async getProfile(): Promise<User> {
    try {
      return await apiClient.get<User>('/users/profile');
    } catch (error) {
      throw new Error('Failed to get user profile: ' + (error as Error).message);
    }
  }

  // Update user profile
  async updateProfile(profileData: Partial<User>): Promise<User> {
    try {
      return await apiClient.put<User>('/users/profile', profileData);
    } catch (error) {
      throw new Error('Failed to update user profile: ' + (error as Error).message);
    }
  }

  // Check if user is authenticated
  isAuthenticated(): boolean {
    return !!localStorage.getItem('authToken');
  }

  // Get auth token
  getAuthToken(): string | null {
    return localStorage.getItem('authToken');
  }
}

export default new AuthService();