import React from 'react';
import Loading from './Loading';
import Alert from './Alert';
import Button from './Button';

interface LoadingErrorProps {
  loading?: boolean;
  error?: string | null;
  onRetry?: () => void;
  loadingText?: string;
  errorTitle?: string;
  children?: React.ReactNode;
}

const LoadingError: React.FC<LoadingErrorProps> = ({
  loading = false,
  error = null,
  onRetry,
  loadingText = 'Loading...',
  errorTitle = 'Error',
  children
}) => {
  if (loading) {
    return (
      <div className="flex justify-center items-center p-8">
        <Loading text={loadingText} />
      </div>
    );
  }

  if (error) {
    return (
      <div className="p-8">
        <Alert variant="error" title={errorTitle}>
          <div className="flex items-center justify-between">
            <span>{error}</span>
            {onRetry && (
              <Button 
                variant="outline" 
                size="sm" 
                onClick={onRetry}
                className="ml-4"
              >
                Retry
              </Button>
            )}
          </div>
        </Alert>
      </div>
    );
  }

  // If neither loading nor error, render children
  return <>{children}</>;
};

export default LoadingError;