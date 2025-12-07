// API service base for the Physical AI & Humanoid Robotics textbook platform
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'https://api.physicalai-textbook.com/v1';

class ApiService {
  private baseUrl: string;
  private headers: Headers;

  constructor() {
    this.baseUrl = API_BASE_URL;
    this.headers = new Headers({
      'Content-Type': 'application/json',
      'Accept': 'application/json'
    });
  }

  // Add auth token to headers if available
  private getAuthHeaders(): Headers {
    const headers = new Headers(this.headers);
    const token = localStorage.getItem('authToken');
    if (token) {
      headers.set('Authorization', `Bearer ${token}`);
    }
    return headers;
  }

  // GET request
  async get<T>(endpoint: string): Promise<T> {
    const response = await fetch(`${this.baseUrl}${endpoint}`, {
      method: 'GET',
      headers: this.getAuthHeaders()
    });

    return this.handleResponse<T>(response);
  }

  // POST request
  async post<T>(endpoint: string, data: any): Promise<T> {
    const response = await fetch(`${this.baseUrl}${endpoint}`, {
      method: 'POST',
      headers: this.getAuthHeaders(),
      body: JSON.stringify(data)
    });

    return this.handleResponse<T>(response);
  }

  // PUT request
  async put<T>(endpoint: string, data: any): Promise<T> {
    const response = await fetch(`${this.baseUrl}${endpoint}`, {
      method: 'PUT',
      headers: this.getAuthHeaders(),
      body: JSON.stringify(data)
    });

    return this.handleResponse<T>(response);
  }

  // DELETE request
  async delete<T>(endpoint: string): Promise<T> {
    const response = await fetch(`${this.baseUrl}${endpoint}`, {
      method: 'DELETE',
      headers: this.getAuthHeaders()
    });

    return this.handleResponse<T>(response);
  }

  // Handle response and errors
  private async handleResponse<T>(response: Response): Promise<T> {
    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.error?.message || `HTTP error! status: ${response.status}`);
    }
    return response.json();
  }
}

export default new ApiService();