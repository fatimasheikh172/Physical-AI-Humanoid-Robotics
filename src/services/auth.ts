// Authentication service
import { User } from '../types/User';

/**
 * Check if user is authenticated
 */
export const isAuthenticated = (): boolean => {
  const token = localStorage.getItem('authToken');
  return !!token;
};

/**
 * Get authentication token
 */
export const getAuthToken = (): string | null => {
  return localStorage.getItem('authToken');
};

/**
 * Set authentication token
 */
export const setAuthToken = (token: string): void => {
  localStorage.setItem('authToken', token);
};

/**
 * Remove authentication token
 */
export const removeAuthToken = (): void => {
  localStorage.removeItem('authToken');
};

/**
 * Get user profile from localStorage
 */
export const getUserProfile = (): User | null => {
  const userString = localStorage.getItem('userProfile');
  if (userString) {
    try {
      return JSON.parse(userString);
    } catch (error) {
      console.error('Failed to parse user profile from localStorage', error);
      return null;
    }
  }
  return null;
};

/**
 * Set user profile in localStorage
 */
export const setUserProfile = (user: User): void => {
  localStorage.setItem('userProfile', JSON.stringify(user));
};

/**
 * Remove user profile from localStorage
 */
export const removeUserProfile = (): void => {
  localStorage.removeItem('userProfile');
};

/**
 * Login function
 * @param email User's email
 * @param password User's password
 * @returns Promise resolving to User object
 */
export const login = async (email: string, password: string): Promise<User> => {
  try {
    // In a real implementation, this would make an API call to authenticate
    // For now, we'll simulate the authentication process
    const response = await fetch(`${process.env.REACT_APP_API_BASE_URL || 'https://api.physicalai-textbook.com/v1'}/auth/login`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ email, password }),
    });

    if (!response.ok) {
      throw new Error('Invalid credentials');
    }

    const data = await response.json();
    setAuthToken(data.token);
    setUserProfile(data.user);

    return data.user;
  } catch (error) {
    console.error('Login failed', error);
    throw error;
  }
};

/**
 * Logout function
 */
export const logout = (): void => {
  removeAuthToken();
  removeUserProfile();
};

/**
 * Register function
 * @param name User's name
 * @param email User's email
 * @param password User's password
 * @returns Promise resolving to User object
 */
export const register = async (name: string, email: string, password: string): Promise<User> => {
  try {
    // In a real implementation, this would make an API call to register
    const response = await fetch(`${process.env.REACT_APP_API_BASE_URL || 'https://api.physicalai-textbook.com/v1'}/auth/register`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ name, email, password }),
    });

    if (!response.ok) {
      throw new Error('Registration failed');
    }

    const data = await response.json();
    setAuthToken(data.token);
    setUserProfile(data.user);

    return data.user;
  } catch (error) {
    console.error('Registration failed', error);
    throw error;
  }
};