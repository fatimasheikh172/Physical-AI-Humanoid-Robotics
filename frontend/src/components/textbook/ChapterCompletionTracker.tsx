import React from 'react';
import { ChapterProgress } from '../../types/Chapter';

interface ChapterCompletionTrackerProps {
  chapterId: string;
  progress: ChapterProgress | null;
  onProgressUpdate: (progress: ChapterProgress) => void;
}

const ChapterCompletionTracker: React.FC<ChapterCompletionTrackerProps> = ({ 
  chapterId, 
  progress,
  onProgressUpdate 
}) => {
  // If no progress exists, initialize with default values
  const initialProgress = progress || {
    id: '',
    userId: '',
    chapterId,
    status: 'not-started',
    completionPercentage: 0,
    timeSpent: 0,
    lastAccessed: new Date().toISOString(),
    exercisesCompleted: 0,
    exercisesTotal: 0,
    simulationsInteracted: 0,
    createdAt: new Date().toISOString(),
    updatedAt: new Date().toISOString(),
  };

  const [currentProgress, setCurrentProgress] = React.useState<ChapterProgress>(initialProgress);
  const [isSaving, setIsSaving] = React.useState(false);
  const [error, setError] = React.useState<string | null>(null);

  // Update progress percentage based on exercises and simulations completed
  React.useEffect(() => {
    const updatedProgress = { ...currentProgress };
    
    // Calculate completion percentage based on exercises and simulations
    const exerciseCompletion = currentProgress.exercisesTotal > 0 
      ? (currentProgress.exercisesCompleted / currentProgress.exercisesTotal) * 100 
      : 0;
      
    const simulationCompletion = currentProgress.simulationsInteracted > 0 ? 100 : 0; // Simplified logic
    
    // Weighted completion (50% exercises, 50% simulations)
    const calculatedPercentage = Math.min(100, Math.round(
      (exerciseCompletion * 0.5) + (simulationCompletion * 0.5)
    ));
    
    updatedProgress.completionPercentage = calculatedPercentage;
    
    // Update status based on completion percentage
    if (calculatedPercentage === 100) {
      updatedProgress.status = 'completed';
    } else if (calculatedPercentage > 0) {
      updatedProgress.status = 'in-progress';
    } else {
      updatedProgress.status = 'not-started';
    }
    
    updatedProgress.updatedAt = new Date().toISOString();
    
    setCurrentProgress(updatedProgress);
  }, [currentProgress.exercisesCompleted, currentProgress.exercisesTotal, currentProgress.simulationsInteracted]);

  // Save progress to backend
  const saveProgress = async () => {
    setIsSaving(true);
    setError(null);
    
    try {
      // In a real implementation, this would call the API to save progress
      // For this example, we'll simulate the API call
      await new Promise(resolve => setTimeout(resolve, 500)); // Simulate network delay
      
      // Update the parent component with the new progress
      onProgressUpdate(currentProgress);
      
      console.log('Progress saved:', currentProgress);
    } catch (err) {
      setError('Failed to save progress. Please try again.');
      console.error('Error saving progress:', err);
    } finally {
      setIsSaving(false);
    }
  };

  // Update progress when it changes
  React.useEffect(() => {
    const saveTimer = setTimeout(() => {
      saveProgress();
    }, 1000); // Save progress after 1 second of inactivity

    return () => clearTimeout(saveTimer);
  }, [currentProgress]);

  // Update exercises completed
  const updateExercisesCompleted = (completed: number) => {
    setCurrentProgress(prev => ({
      ...prev,
      exercisesCompleted: Math.max(0, Math.min(completed, prev.exercisesTotal)),
      lastAccessed: new Date().toISOString()
    }));
  };

  // Update simulations interacted
  const updateSimulationsInteracted = (simulations: number) => {
    setCurrentProgress(prev => ({
      ...prev,
      simulationsInteracted: simulations,
      lastAccessed: new Date().toISOString()
    }));
  };

  // Update total exercises in chapter
  const updateExercisesTotal = (total: number) => {
    setCurrentProgress(prev => ({
      ...prev,
      exercisesTotal: total,
      lastAccessed: new Date().toISOString()
    }));
  };

  return (
    <div className="bg-white p-4 rounded-lg shadow-md">
      <h3 className="text-lg font-semibold mb-3">Chapter Progress</h3>
      
      <div className="mb-4">
        <div className="flex justify-between mb-1">
          <span className="text-sm font-medium">Completion: {currentProgress.completionPercentage}%</span>
          <span className="text-sm font-medium text-gray-500">{currentProgress.status.replace('-', ' ')}</span>
        </div>
        <div className="w-full bg-gray-200 rounded-full h-2.5">
          <div 
            className="bg-blue-600 h-2.5 rounded-full transition-all duration-300" 
            style={{ width: `${currentProgress.completionPercentage}%` }}
          ></div>
        </div>
      </div>
      
      <div className="grid grid-cols-2 gap-4 mb-4">
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Exercises Completed
          </label>
          <div className="flex">
            <input
              type="number"
              min="0"
              max={currentProgress.exercisesTotal}
              value={currentProgress.exercisesCompleted}
              onChange={(e) => updateExercisesCompleted(parseInt(e.target.value) || 0)}
              className="w-full border border-gray-300 rounded-md px-3 py-1 text-sm"
            />
            <span className="mx-2 self-center text-sm">/ {currentProgress.exercisesTotal}</span>
          </div>
        </div>
        
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Simulations Interacted
          </label>
          <input
            type="number"
            min="0"
            value={currentProgress.simulationsInteracted}
            onChange={(e) => updateSimulationsInteracted(parseInt(e.target.value) || 0)}
            className="w-full border border-gray-300 rounded-md px-3 py-1 text-sm"
          />
        </div>
      </div>
      
      <div className="text-xs text-gray-500 mb-2">
        Time spent: {Math.floor(currentProgress.timeSpent / 60)}m {currentProgress.timeSpent % 60}s
      </div>
      
      {isSaving && (
        <div className="text-xs text-blue-500 flex items-center">
          <svg className="animate-spin -ml-1 mr-2 h-3 w-3 text-blue-500" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
            <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
            <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
          </svg>
          Saving progress...
        </div>
      )}
      
      {error && (
        <div className="text-xs text-red-500 mt-1">{error}</div>
      )}
    </div>
  );
};

export default ChapterCompletionTracker;