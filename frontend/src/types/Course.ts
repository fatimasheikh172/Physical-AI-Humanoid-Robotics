export interface Course {
  id: string;
  title: string;
  description: string;
  instructorId: string; // reference to instructor User
  startDate: string; // date
  endDate: string; // date
  status: 'draft' | 'active' | 'completed';
  enrollmentCode: string;
  language: string;
  createdAt: string;
  updatedAt: string;
}