import React from 'react';

interface ErrorBoundaryState {
  hasError: boolean;
  error?: Error;
}

interface ErrorBoundaryProps {
  children: React.ReactNode;
  fallback?: React.ComponentType<{ error: Error }>;
}

class ErrorBoundary extends React.Component<ErrorBoundaryProps, ErrorBoundaryState> {
  constructor(props: ErrorBoundaryProps) {
    super(props);
    this.state = { hasError: false };
  }

  static getDerivedStateFromError(error: Error): ErrorBoundaryState {
    // Update state so the next render will show the fallback UI
    return { hasError: true, error };
  }

  componentDidCatch(error: Error, errorInfo: React.ErrorInfo): void {
    console.error('Error caught by boundary:', error, errorInfo);
    
    // In a real application, you might log the error to an error reporting service
    // logErrorToService(error, errorInfo);
  }

  render(): React.ReactNode {
    if (this.state.hasError) {
      // You can render any custom fallback UI
      const FallbackComponent = this.props.fallback || DefaultErrorFallback;
      return <FallbackComponent error={this.state.error!} />;
    }

    return this.props.children;
  }
}

// Default fallback component
const DefaultErrorFallback: React.FC<{ error: Error }> = ({ error }) => {
  return (
    <div className="flex flex-col items-center justify-center p-8 text-center">
      <h2 className="text-2xl font-bold text-red-600 mb-4">Something went wrong</h2>
      <p className="text-gray-600 mb-4">{error.message}</p>
      <button
        className="px-4 py-2 bg-blue-600 text-white rounded-md hover:bg-blue-700"
        onClick={() => window.location.reload()}
      >
        Reload Page
      </button>
      <details className="mt-4 text-sm text-gray-500">
        <summary className="cursor-pointer">Error details</summary>
        <pre className="text-xs mt-2 p-2 bg-gray-100 rounded overflow-auto">
          {error.stack}
        </pre>
      </details>
    </div>
  );
};

export default ErrorBoundary;