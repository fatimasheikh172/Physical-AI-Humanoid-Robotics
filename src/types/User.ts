export interface User {
  id: string;
  email: string;
  name: string;
  role: 'student' | 'instructor';
  preferences: {
    language: string; // default: 'en', supports 'ur' for Urdu
    theme: 'light' | 'dark';
    accessibility: {
      screenReader?: boolean;
      reducedMotion?: boolean;
      // Additional accessibility preferences
      [key: string]: any;
    };
  };
  createdAt: string; // ISO date string
  lastLogin: string; // ISO date string
}