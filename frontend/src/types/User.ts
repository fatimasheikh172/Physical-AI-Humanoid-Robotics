export interface User {
  id: string;
  email: string;
  name: string;
  role: 'student' | 'instructor';
  preferences: {
    language: string;
    theme: 'light' | 'dark';
    accessibility: Record<string, any>;
  };
  createdAt: string;
  lastLogin: string;
}