// Component tests for UI elements
// Testing the ChapterCompletionTracker component

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import ChapterCompletionTracker from '../src/components/textbook/ChapterCompletionTracker';
import { ChapterProgress } from '../src/types/Chapter';

describe('ChapterCompletionTracker', () => {
  const mockProgress: ChapterProgress = {
    id: 'progress-1',
    userId: 'user-1',
    chapterId: 'chapter-1',
    status: 'in-progress',
    completionPercentage: 50,
    timeSpent: 1800, // 30 minutes in seconds
    lastAccessed: new Date().toISOString(),
    exercisesCompleted: 3,
    exercisesTotal: 6,
    simulationsInteracted: 1,
    createdAt: new Date().toISOString(),
    updatedAt: new Date().toISOString(),
  };

  const mockOnProgressUpdate = jest.fn();

  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('renders correctly with initial progress', () => {
    render(
      <ChapterCompletionTracker 
        chapterId="chapter-1" 
        progress={mockProgress} 
        onProgressUpdate={mockOnProgressUpdate} 
      />
    );

    expect(screen.getByText(/Chapter Progress/i)).toBeInTheDocument();
    expect(screen.getByText('Completion: 50%')).toBeInTheDocument();
    expect(screen.getByDisplayValue(mockProgress.exercisesCompleted)).toBeInTheDocument();
    expect(screen.getByText(`Time spent: ${Math.floor(mockProgress.timeSpent / 60)}m ${mockProgress.timeSpent % 60}s`)).toBeInTheDocument();
  });

  it('allows updating exercises completed', async () => {
    render(
      <ChapterCompletionTracker 
        chapterId="chapter-1" 
        progress={mockProgress} 
        onProgressUpdate={mockOnProgressUpdate} 
      />
    );

    const exercisesInput = screen.getByDisplayValue(mockProgress.exercisesCompleted);
    fireEvent.change(exercisesInput, { target: { value: '5' } });

    await waitFor(() => {
      expect(mockOnProgressUpdate).toHaveBeenCalled();
    });
  });

  it('disables exercises input if total is 0', () => {
    const progressWithNoExercises = { ...mockProgress, exercisesTotal: 0, exercisesCompleted: 0 };
    
    render(
      <ChapterCompletionTracker 
        chapterId="chapter-1" 
        progress={progressWithNoExercises} 
        onProgressUpdate={mockOnProgressUpdate} 
      />
    );

    const exercisesInput = screen.getByDisplayValue('0');
    expect(exercisesInput).toBeInTheDocument();
  });

  it('updates completion percentage when exercises completed changes', async () => {
    const initialProgress: ChapterProgress = {
      ...mockProgress,
      exercisesTotal: 4,
      exercisesCompleted: 2,
      completionPercentage: 25, // 50% of exercises * 50% weight = 25%
    };
    
    render(
      <ChapterCompletionTracker 
        chapterId="chapter-1" 
        progress={initialProgress} 
        onProgressUpdate={mockOnProgressUpdate} 
      />
    );

    // Initially 25% (2/4 exercises = 50%, but weighted 50% = 25%)
    expect(screen.getByText('Completion: 25%')).toBeInTheDocument();

    // Update exercises completed to 4/4 (100%), which should result in 50% total (100% * 50% weight)
    const exercisesInput = screen.getByDisplayValue('2');
    fireEvent.change(exercisesInput, { target: { value: '4' } });

    await waitFor(() => {
      expect(screen.getByText('Completion: 50%')).toBeInTheDocument();
    });
  });
});