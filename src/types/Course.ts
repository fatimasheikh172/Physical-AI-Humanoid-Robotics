export interface Course {
  id: string;
  title: string;
  description: string;
  instructorId: string; // Reference to instructor User
  startDate: string; // Date string
  endDate: string; // Date string
  status: 'draft' | 'active' | 'completed';
  enrollmentCode: string; // Code for student enrollment
  language: string;
  createdAt: string; // ISO date string
  updatedAt: string; // ISO date string
}