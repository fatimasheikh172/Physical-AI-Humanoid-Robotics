import React, { useState, useEffect } from 'react';
import { UserService } from '../../services/ApiClient';
import { Course } from '../../types/Course';
import { User } from '../../types/User';
import LoadingError from '../../components/common/LoadingError';

const InstructorDashboard: React.FC = () => {
  const [courses, setCourses] = useState<Course[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [currentUser, setCurrentUser] = useState<User | null>(null);

  useEffect(() => {
    const fetchData = async () => {
      try {
        setLoading(true);
        setError(null);
        
        // Get current user
        const user = await UserService.getProfile();
        setCurrentUser(user);
        
        // Only fetch courses if user is an instructor
        if (user.role === 'instructor') {
          // In a real implementation, we would fetch courses where the instructor is the owner
          // For now, we'll create mock data
          const mockCourses: Course[] = [
            {
              id: 'course-1',
              title: 'Introduction to Physical AI',
              description: 'An introductory course to Physical AI concepts',
              instructorId: user.id,
              startDate: new Date().toISOString(),
              endDate: new Date(Date.now() + 30 * 24 * 60 * 60 * 1000).toISOString(),
              status: 'active',
              enrollmentCode: 'PHYSAI101',
              language: 'en',
              createdAt: new Date().toISOString(),
              updatedAt: new Date().toISOString(),
            },
            {
              id: 'course-2',
              title: 'Advanced Humanoid Robotics',
              description: 'Deep dive into humanoid robot design and control',
              instructorId: user.id,
              startDate: new Date().toISOString(),
              endDate: new Date(Date.now() + 60 * 24 * 60 * 60 * 1000).toISOString(),
              status: 'active',
              enrollmentCode: 'HUMROB201',
              language: 'en',
              createdAt: new Date().toISOString(),
              updatedAt: new Date().toISOString(),
            }
          ];
          setCourses(mockCourses);
        } else {
          throw new Error('Access denied. Instructor role required.');
        }
      } catch (err) {
        setError(err instanceof Error ? err.message : 'Failed to load dashboard data');
      } finally {
        setLoading(false);
      }
    };

    fetchData();
  }, []);

  if (error) {
    return <LoadingError loading={false} error={error} onRetry={() => window.location.reload()} children={undefined} />;
  }

  if (loading || !currentUser) {
    return <LoadingError loading={true} error={null} children={undefined} />;
  }

  return (
    <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-8">
      <div className="mb-8">
        <h1 className="text-3xl font-bold text-gray-900">Instructor Dashboard</h1>
        <p className="mt-2 text-gray-600">Manage your courses and track student progress</p>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6 mb-8">
        <div className="bg-white rounded-lg shadow p-6">
          <h2 className="text-xl font-semibold mb-4">Courses</h2>
          <p className="text-3xl font-bold text-blue-600">{courses.length}</p>
          <p className="text-gray-600">Active courses</p>
        </div>
        
        <div className="bg-white rounded-lg shadow p-6">
          <h2 className="text-xl font-semibold mb-4">Students</h2>
          <p className="text-3xl font-bold text-green-600">42</p>
          <p className="text-gray-600">Enrolled students</p>
        </div>
        
        <div className="bg-white rounded-lg shadow p-6">
          <h2 className="text-xl font-semibold mb-4">Avg. Progress</h2>
          <p className="text-3xl font-bold text-purple-600">68%</p>
          <p className="text-gray-600">Across all courses</p>
        </div>
      </div>

      <div className="bg-white rounded-lg shadow overflow-hidden">
        <div className="px-6 py-4 border-b">
          <h2 className="text-xl font-semibold">Your Courses</h2>
        </div>
        
        <div className="divide-y">
          {courses.map(course => (
            <div key={course.id} className="p-6 flex justify-between items-center">
              <div>
                <h3 className="text-lg font-medium text-gray-900">{course.title}</h3>
                <p className="text-sm text-gray-500">{course.description}</p>
                <div className="mt-2 flex items-center text-sm text-gray-500">
                  <span>Status: {course.status}</span>
                  <span className="mx-2">•</span>
                  <span>Code: {course.enrollmentCode}</span>
                </div>
              </div>
              <div className="flex space-x-3">
                <button className="px-4 py-2 bg-blue-600 text-white rounded-md hover:bg-blue-700 text-sm">
                  Manage
                </button>
                <button className="px-4 py-2 bg-gray-200 text-gray-700 rounded-md hover:bg-gray-300 text-sm">
                  View Students
                </button>
              </div>
            </div>
          ))}
        </div>
        
        <div className="px-6 py-4 border-t bg-gray-50">
          <button className="px-4 py-2 bg-green-600 text-white rounded-md hover:bg-green-700 text-sm">
            + Create New Course
          </button>
        </div>
      </div>
    </div>
  );
};

export default InstructorDashboard;