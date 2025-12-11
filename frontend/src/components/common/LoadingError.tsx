import React from 'react';

interface LoadingProps {
  message?: string;
  className?: string;
}

export const LoadingSpinner: React.FC<LoadingProps> = ({ message = 'Loading...', className = '' }) => {
  return (
    <div className={`flex flex-col items-center justify-center p-8 ${className}`}>
      <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500 mb-4"></div>
      {message && <p className="text-gray-600">{message}</p>}
    </div>
  );    
};

interface ErrorProps {
  message: string;
  onRetry?: () => void;
  className?: string;
}

export const ErrorMessage: React.FC<ErrorProps> = ({ message, onRetry, className = '' }) => {
  return (
    <div className={`bg-red-50 border-l-4 border-red-500 p-4 ${className}`}>
      <div className="flex items-start">
        <div className="flex-shrink-0">
          <svg className="h-5 w-5 text-red-500" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 20 20" fill="currentColor">
            <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM8.707 7.293a1 1 0 00-1.414 1.414L8.586 10l-1.293 1.293a1 1 0 101.414 1.414L10 11.414l1.293 1.293a1 1 0 001.414-1.414L11.414 10l1.293-1.293a1 1 0 00-1.414-1.414L10 8.586 8.707 7.293z" clipRule="evenodd" />
          </svg>
        </div>
        <div className="ml-3">
          <p className="text-sm text-red-700">
            <span className="font-medium">Error: </span>
            {message}
          </p>
          {onRetry && (
            <button
              onClick={onRetry}
              className="mt-2 text-sm text-red-700 underline hover:text-red-600"
            >
              Try again
            </button>
          )}
        </div>
      </div>
    </div>
  );
};

interface LoadingErrorProps {
  loading: boolean;
  error: string | null;
  onRetry?: () => void;
  children: React.ReactNode;
}

const LoadingError: React.FC<LoadingErrorProps> = ({ loading, error, onRetry, children }) => {
  if (loading) {
    return <LoadingSpinner />;
  }

  if (error) {
    return <ErrorMessage message={error} onRetry={onRetry} />;
  }

  return <>{children}</>;
};

export default LoadingError;