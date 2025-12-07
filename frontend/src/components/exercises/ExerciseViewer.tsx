import React, { useState } from 'react';
import { Exercise } from '../../types/Exercise';
import { ExerciseService } from '../../services/ApiClient';

interface ExerciseViewerProps {
  exercise: Exercise;
}

const ExerciseViewer: React.FC<ExerciseViewerProps> = ({ exercise }) => {
  const [selectedOption, setSelectedOption] = useState<string | null>(null);
  const [userAnswer, setUserAnswer] = useState<string>('');
  const [isSubmitted, setIsSubmitted] = useState(false);
  const [feedback, setFeedback] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);

  const handleSubmit = async () => {
    setIsLoading(true);
    setFeedback(null);
    
    try {
      // Prepare the answer based on exercise type
      let answer: any;
      if (exercise.type === 'multiple-choice') {
        answer = selectedOption;
      } else {
        answer = userAnswer;
      }
      
      // Submit the answer
      const result = await ExerciseService.submit(exercise.id, answer);
      
      // Update feedback based on the result
      setFeedback(result.correct ? 'Correct! Well done.' : `Incorrect. ${result.feedback || 'Please try again.'}`);
      setIsSubmitted(true);
    } catch (error) {
      setFeedback('Failed to submit answer. Please try again.');
      console.error('Error submitting exercise:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const handleReset = () => {
    setSelectedOption(null);
    setUserAnswer('');
    setIsSubmitted(false);
    setFeedback(null);
  };

  return (
    <div className="border rounded-lg p-4 bg-white">
      <div className="mb-4">
        <h3 className="text-lg font-medium text-gray-900">{exercise.title}</h3>
        <div className="flex items-center mt-1">
          <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-blue-100 text-blue-800">
            {exercise.type.replace('-', ' ')}
          </span>
          <span className="ml-2 inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-green-100 text-green-800">
            {exercise.difficulty}
          </span>
          <span className="ml-2 text-xs text-gray-500">{exercise.points} points</span>
        </div>
      </div>
      
      <div className="prose text-gray-700 mb-4">
        <p>{exercise.description}</p>
        {exercise.content && <div dangerouslySetInnerHTML={{ __html: exercise.content.replace(/\n/g, '<br />') }} />}
      </div>
      
      <div className="mb-4">
        {exercise.type === 'multiple-choice' && (
          <div className="space-y-2">
            {exercise.content.split('\n').filter(line => line.trim() !== '').map((option, index) => (
              <div key={index} className="flex items-start">
                <input
                  type="radio"
                  id={`option-${index}`}
                  name="multiple-choice"
                  checked={selectedOption === option}
                  onChange={() => setSelectedOption(option)}
                  className="mt-1"
                  disabled={isSubmitted}
                />
                <label htmlFor={`option-${index}`} className="ml-2 text-gray-700">
                  {option}
                </label>
              </div>
            ))}
          </div>
        )}
        
        {(exercise.type === 'coding' || exercise.type === 'simulation' || exercise.type === 'project') && (
          <textarea
            value={userAnswer}
            onChange={(e) => setUserAnswer(e.target.value)}
            placeholder="Enter your answer here..."
            className="w-full p-3 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
            rows={5}
            disabled={isSubmitted}
          />
        )}
      </div>
      
      {exercise.hints.length > 0 && !isSubmitted && (
        <details className="mb-4 text-sm text-gray-600">
          <summary className="cursor-pointer font-medium">Need help? Show hints</summary>
          <div className="mt-2 space-y-2">
            {exercise.hints.map((hint, index) => (
              <div key={index} className="p-2 bg-blue-50 rounded-md">
                Hint {index + 1}: {hint}
              </div>
            ))}
          </div>
        </details>
      )}
      
      <div className="flex flex-wrap gap-2">
        <button
          onClick={handleSubmit}
          disabled={isSubmitted || isLoading || 
            (exercise.type === 'multiple-choice' && !selectedOption) || 
            ((exercise.type === 'coding' || exercise.type === 'simulation' || exercise.type === 'project') && !userAnswer.trim())}
          className="px-4 py-2 bg-blue-600 text-white rounded-md hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-offset-2 disabled:opacity-50 disabled:cursor-not-allowed"
        >
          {isLoading ? 'Submitting...' : isSubmitted ? 'Submitted' : 'Submit Answer'}
        </button>
        
        <button
          onClick={handleReset}
          className="px-4 py-2 bg-gray-200 text-gray-700 rounded-md hover:bg-gray-300 focus:outline-none focus:ring-2 focus:ring-gray-500 focus:ring-offset-2"
        >
          Reset
        </button>
      </div>
      
      {feedback && (
        <div className={`mt-4 p-3 rounded-md ${isSubmitted && feedback.includes('Correct') ? 'bg-green-50 text-green-800' : 'bg-yellow-50 text-yellow-800'}`}>
          {feedback}
        </div>
      )}
    </div>
  );
};

export default ExerciseViewer;