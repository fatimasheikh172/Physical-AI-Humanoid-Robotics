// Base API service configuration
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'https://api.physicalai-textbook.com/v1';

// Default headers for API requests
const DEFAULT_HEADERS = {
  'Content-Type': 'application/json',
  Accept: 'application/json',
};

/**
 * Generic API request function
 * @param endpoint - API endpoint to call
 * @param options - Request options (method, headers, body, etc.)
 * @param includeAuth - Whether to include authentication token
 */
export const apiRequest = async (
  endpoint: string,
  options: RequestInit = {},
  includeAuth: boolean = true
): Promise<any> => {
  let headers = { ...DEFAULT_HEADERS, ...options.headers };

  // Add authentication token if required
  if (includeAuth) {
    const token = localStorage.getItem('authToken');
    if (token) {
      headers = { ...headers, Authorization: `Bearer ${token}` };
    }
  }

  const url = `${API_BASE_URL}${endpoint}`;

  try {
    const response = await fetch(url, {
      ...options,
      headers,
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.message || `HTTP error! status: ${response.status}`);
    }

    return await response.json();
  } catch (error) {
    console.error(`API request failed: ${url}`, error);
    throw error;
  }
};

// API service with predefined methods
export const apiService = {
  // User endpoints
  user: {
    getProfile: () => apiRequest('/users/profile', { method: 'GET' }),
    updateProfile: (data: any) => apiRequest('/users/profile', {
      method: 'PUT',
      body: JSON.stringify(data),
    }),
  },

  // Chapter endpoints
  chapters: {
    getList: (params?: { difficulty?: string; language?: string; limit?: number; offset?: number }) => {
      const query = new URLSearchParams(params as any).toString();
      return apiRequest(`/chapters${query ? `?${query}` : ''}`, { method: 'GET' });
    },
    get: (id: string) => apiRequest(`/chapters/${id}`, { method: 'GET' }),
  },

  // Progress endpoints
  progress: {
    get: (params?: { chapterId?: string }) => {
      const query = new URLSearchParams(params as any).toString();
      return apiRequest(`/progress${query ? `?${query}` : ''}`, { method: 'GET' });
    },
    update: (data: any) => apiRequest('/progress', {
      method: 'POST',
      body: JSON.stringify(data),
    }),
  },

  // Simulation endpoints
  simulations: {
    getList: (params?: { chapterId?: string; complexity?: string }) => {
      const query = new URLSearchParams(params as any).toString();
      return apiRequest(`/simulations${query ? `?${query}` : ''}`, { method: 'GET' });
    },
    startSession: (id: string) => apiRequest(`/simulations/${id}/session`, {
      method: 'POST',
    }),
    updateSession: (id: string, sessionId: string, data: any) => apiRequest(`/simulations/${id}/session/${sessionId}`, {
      method: 'PUT',
      body: JSON.stringify(data),
    }),
  },

  // Exercise endpoints
  exercises: {
    submit: (id: string, data: any) => apiRequest(`/exercises/${id}/submit`, {
      method: 'POST',
      body: JSON.stringify(data),
    }),
  },

  // AI Tutor endpoints
  aiTutor: {
    query: (data: any) => apiRequest('/ai-tutor/query', {
      method: 'POST',
      body: JSON.stringify(data),
    }),
    rate: (queryId: string, rating: number) => apiRequest(`/ai-tutor/${queryId}/rate`, {
      method: 'POST',
      body: JSON.stringify({ rating }),
    }),
  },

  // Course endpoints (instructor only)
  courses: {
    create: (data: any) => apiRequest('/courses', {
      method: 'POST',
      body: JSON.stringify(data),
    }),
    getStudents: (id: string) => apiRequest(`/courses/${id}/students`, { method: 'GET' }),
  },
};