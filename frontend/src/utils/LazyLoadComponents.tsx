// Lazy loading components for performance optimization

import React, { lazy, Suspense } from 'react';
import LoadingSpinner from '../components/common/LoadingError';

// Lazy load heavy components like the simulation viewer
export const LazySimulationViewer = lazy(() => 
  import('../components/simulations/SimulationViewer')
);

// Lazy load the AI Tutor component
export const LazyAITutor = lazy(() => 
  import('../components/ai-tutor/AITutor')
);

// Lazy load exercise components
export const LazyExerciseViewer = lazy(() => 
  import('../components/exercises/ExerciseViewer')
);

// Lazy load the instructor dashboard
export const LazyInstructorDashboard = lazy(() => 
  import('../pages/dashboard/InstructorDashboard')
);

// Wrapper component for lazy-loaded components with a loading spinner
interface LazyComponentWrapperProps {
  children: React.ReactNode;
}

export const LazyComponentWrapper: React.FC<LazyComponentWrapperProps> = ({ children }) => {
  return (
    <Suspense fallback={<LoadingSpinner message="Loading component..." />}>
      {children}
    </Suspense>
  );
};

// Usage example:
// <LazyComponentWrapper>
//   <LazySimulationViewer simulation={simulation} />
// </LazyComponentWrapper>