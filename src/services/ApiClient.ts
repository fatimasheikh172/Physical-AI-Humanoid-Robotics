import { User } from '../types/User';
import { Chapter } from '../types/Chapter';
import { Progress } from '../types/Progress';
import { Simulation } from '../types/Simulation';
import { Exercise } from '../types/Exercise';
import { Project } from '../types/Project';
import { Course } from '../types/Course';

// Configuration
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'https://api.physicalai-textbook.com/v1';
const DEFAULT_HEADERS = {
  'Content-Type': 'application/json',
  Accept: 'application/json',
};

// Response types
interface ApiResponse<T> {
  data: T;
  message?: string;
}

interface ApiError {
  code: string;
  message: string;
  details?: any;
}

// API Client Class
class ApiClient {
  private baseUrl: string;
  private defaultHeaders: Record<string, string>;

  constructor() {
    this.baseUrl = API_BASE_URL;
    this.defaultHeaders = DEFAULT_HEADERS;
  }

  // Helper method to create request options
  private createOptions(
    method: string,
    body?: any,
    includeAuth: boolean = true
  ): RequestInit {
    let headers = { ...this.defaultHeaders };

    if (includeAuth) {
      const token = localStorage.getItem('authToken');
      if (token) {
        headers['Authorization'] = `Bearer ${token}`;
      }
    }

    const options: RequestInit = {
      method,
      headers,
    };

    if (body) {
      options.body = JSON.stringify(body);
    }

    return options;
  }

  // Helper method to process response
  private async processResponse<T>(response: Response): Promise<ApiResponse<T>> {
    if (!response.ok) {
      const errorData: ApiError = await response.json().catch(() => ({}));
      throw new Error(errorData.message || `HTTP error! status: ${response.status}`);
    }

    if (response.status === 204) {
      // No content
      return { data: null as unknown as T };
    }

    const data = await response.json();
    return { data };
  }

  // User endpoints
  async getUserProfile(): Promise<ApiResponse<User>> {
    const response = await fetch(`${this.baseUrl}/users/profile`, 
      this.createOptions('GET', undefined, true)
    );
    return this.processResponse<User>(response);
  }

  async updateUserProfile(userData: Partial<User>): Promise<ApiResponse<User>> {
    const response = await fetch(`${this.baseUrl}/users/profile`,
      this.createOptions('PUT', userData, true)
    );
    return this.processResponse<User>(response);
  }

  // Chapter endpoints
  async getChapters(
    difficulty?: string,
    language?: string,
    limit?: number,
    offset?: number
  ): Promise<ApiResponse<Chapter[]>> {
    const params = new URLSearchParams();
    if (difficulty) params.append('difficulty', difficulty);
    if (language) params.append('language', language);
    if (limit) params.append('limit', limit.toString());
    if (offset) params.append('offset', offset.toString());

    const queryString = params.toString();
    const url = `${this.baseUrl}/chapters${queryString ? `?${queryString}` : ''}`;

    const response = await fetch(url, this.createOptions('GET', undefined, false));
    return this.processResponse<Chapter[]>(response);
  }

  async getChapterById(id: string): Promise<ApiResponse<Chapter>> {
    const response = await fetch(`${this.baseUrl}/chapters/${id}`,
      this.createOptions('GET', undefined, false)
    );
    return this.processResponse<Chapter>(response);
  }

  // Progress endpoints
  async getProgress(chapterId?: string): Promise<ApiResponse<Progress[]>> {
    const params = new URLSearchParams();
    if (chapterId) params.append('chapterId', chapterId);

    const queryString = params.toString();
    const url = `${this.baseUrl}/progress${queryString ? `?${queryString}` : ''}`;

    const response = await fetch(url, this.createOptions('GET', undefined, true));
    return this.processResponse<Progress[]>(response);
  }

  async updateProgress(progressData: Partial<Progress>): Promise<ApiResponse<Progress>> {
    const response = await fetch(`${this.baseUrl}/progress`,
      this.createOptions('POST', progressData, true)
    );
    return this.processResponse<Progress>(response);
  }

  // Simulation endpoints
  async getSimulations(
    chapterId?: string,
    complexity?: string
  ): Promise<ApiResponse<Simulation[]>> {
    const params = new URLSearchParams();
    if (chapterId) params.append('chapterId', chapterId);
    if (complexity) params.append('complexity', complexity);

    const queryString = params.toString();
    const url = `${this.baseUrl}/simulations${queryString ? `?${queryString}` : ''}`;

    const response = await fetch(url, this.createOptions('GET', undefined, false));
    return this.processResponse<Simulation[]>(response);
  }

  async startSimulationSession(simulationId: string): Promise<ApiResponse<any>> {
    const response = await fetch(`${this.baseUrl}/simulations/${simulationId}/session`,
      this.createOptions('POST', undefined, true)
    );
    return this.processResponse<any>(response);
  }

  async updateSimulationSession(
    simulationId: string,
    sessionId: string,
    sessionData: any
  ): Promise<ApiResponse<any>> {
    const response = await fetch(
      `${this.baseUrl}/simulations/${simulationId}/session/${sessionId}`,
      this.createOptions('PUT', sessionData, true)
    );
    return this.processResponse<any>(response);
  }

  // Exercise endpoints
  async submitExercise(
    exerciseId: string,
    submissionData: any
  ): Promise<ApiResponse<any>> {
    const response = await fetch(`${this.baseUrl}/exercises/${exerciseId}/submit`,
      this.createOptions('POST', submissionData, true)
    );
    return this.processResponse<any>(response);
  }

  // AI Tutor endpoints
  async queryAiTutor(queryData: { 
    query: string; 
    context: { 
      chapterId?: string; 
      simulationId?: string; 
      additionalContext?: string 
    } 
  }): Promise<ApiResponse<any>> {
    const response = await fetch(`${this.baseUrl}/ai-tutor/query`,
      this.createOptions('POST', queryData, true)
    );
    return this.processResponse<any>(response);
  }

  async rateAiTutorResponse(queryId: string, rating: number): Promise<ApiResponse<any>> {
    const response = await fetch(`${this.baseUrl}/ai-tutor/${queryId}/rate`,
      this.createOptions('POST', { rating }, true)
    );
    return this.processResponse<any>(response);
  }

  // Course endpoints
  async createCourse(courseData: any): Promise<ApiResponse<Course>> {
    const response = await fetch(`${this.baseUrl}/courses`,
      this.createOptions('POST', courseData, true)
    );
    return this.processResponse<Course>(response);
  }

  async getCourseStudents(courseId: string): Promise<ApiResponse<User[]>> {
    const response = await fetch(`${this.baseUrl}/courses/${courseId}/students`,
      this.createOptions('GET', undefined, true)
    );
    return this.processResponse<User[]>(response);
  }
}

// Create a singleton instance
const apiClient = new ApiClient();
export default apiClient;