import React from 'react';
import clsx from 'clsx';

interface LoadingProps {
  size?: 'sm' | 'md' | 'lg';
  variant?: 'default' | 'dots';
  className?: string;
  text?: string;
}

const Loading: React.FC<LoadingProps> = ({
  size = 'md',
  variant = 'default',
  className = '',
  text,
}) => {
  const sizeClasses = clsx({
    'w-4 h-4': size === 'sm',
    'w-8 h-8': size === 'md',
    'w-12 h-12': size === 'lg',
  });

  if (variant === 'default') {
    return (
      <div className={clsx('flex flex-col items-center justify-center', className)}>
        <div
          className={clsx(
            'animate-spin rounded-full border-t-2 border-blue-600',
            sizeClasses
          )}
        />
        {text && <span className="mt-2 text-sm text-gray-500 dark:text-gray-400">{text}</span>}
      </div>
    );
  } else {
    // Dots variant
    return (
      <div className={clsx('flex items-center justify-center', className)}>
        <div className="flex space-x-1">
          <div className="w-2 h-2 bg-blue-600 rounded-full animate-bounce"></div>
          <div className="w-2 h-2 bg-blue-600 rounded-full animate-bounce" style={{ animationDelay: '0.2s' }}></div>
          <div className="w-2 h-2 bg-blue-600 rounded-full animate-bounce" style={{ animationDelay: '0.4s' }}></div>
        </div>
        {text && <span className="ml-2 text-sm text-gray-500 dark:text-gray-400">{text}</span>}
      </div>
    );
  }
};

export default Loading;