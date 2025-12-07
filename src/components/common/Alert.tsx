import React from 'react';
import clsx from 'clsx';

interface AlertProps {
  children: React.ReactNode;
  variant?: 'success' | 'error' | 'warning' | 'info';
  className?: string;
  title?: string;
}

const Alert: React.FC<AlertProps> = ({
  children,
  variant = 'info',
  className = '',
  title,
}) => {
  const variantClasses = clsx({
    'bg-green-50 border-green-200 text-green-800 dark:bg-green-900/30 dark:border-green-900 dark:text-green-200': variant === 'success',
    'bg-red-50 border-red-200 text-red-800 dark:bg-red-900/30 dark:border-red-900 dark:text-red-200': variant === 'error',
    'bg-yellow-50 border-yellow-200 text-yellow-800 dark:bg-yellow-900/30 dark:border-yellow-900 dark:text-yellow-200': variant === 'warning',
    'bg-blue-50 border-blue-200 text-blue-800 dark:bg-blue-900/30 dark:border-blue-900 dark:text-blue-200': variant === 'info',
  });

  return (
    <div
      className={clsx(
        'border rounded-md p-4',
        variantClasses,
        className
      )}
    >
      {title && (
        <h4 className="text-sm font-medium mb-1">
          {title}
        </h4>
      )}
      <div className="text-sm">
        {children}
      </div>
    </div>
  );
};

export default Alert;