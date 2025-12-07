// Unit tests for authentication service

import AuthService from '../src/services/auth';

// Mock localStorage
const mockLocalStorage = (() => {
  let store: {[key: string]: string} = {};
  return {
    getItem: (key: string) => store[key] || null,
    setItem: (key: string, value: string) => {
      store[key] = value.toString();
    },
    removeItem: (key: string) => {
      delete store[key];
    },
    clear: () => {
      store = {};
    }
  };
})();

Object.defineProperty(window, 'localStorage', {
  value: mockLocalStorage
});

// Mock the api client
jest.mock('../src/services/api', () => ({
  __esModule: true,
  default: {
    post: jest.fn(),
    get: jest.fn(),
    put: jest.fn(),
    delete: jest.fn(),
  }
}));

import apiClient from '../src/services/api';

describe('AuthService', () => {
  beforeEach(() => {
    mockLocalStorage.clear();
    jest.clearAllMocks();
  });

  it('should store token in localStorage after successful login', async () => {
    const mockUser = { id: '1', email: 'test@example.com', name: 'Test User' };
    const mockToken = 'mock-token';
    
    (apiClient.post as jest.Mock).mockResolvedValue({
      token: mockToken,
      user: mockUser
    });

    const result = await AuthService.login('test@example.com', 'password');
    
    expect(apiClient.post).toHaveBeenCalledWith('/auth/login', {
      email: 'test@example.com',
      password: 'password'
    });
    
    expect(mockLocalStorage.getItem('authToken')).toBe(mockToken);
    expect(result).toEqual(mockUser);
  });

  it('should remove token from localStorage on logout', () => {
    mockLocalStorage.setItem('authToken', 'some-token');
    
    AuthService.logout();
    
    expect(mockLocalStorage.getItem('authToken')).toBeNull();
  });

  it('should return true if token exists in localStorage', () => {
    mockLocalStorage.setItem('authToken', 'some-token');
    
    expect(AuthService.isAuthenticated()).toBe(true);
  });

  it('should return false if no token exists in localStorage', () => {
    expect(AuthService.isAuthenticated()).toBe(false);
  });

  it('should get the auth token from localStorage', () => {
    const token = 'some-token';
    mockLocalStorage.setItem('authToken', token);
    
    expect(AuthService.getAuthToken()).toBe(token);
  });
});