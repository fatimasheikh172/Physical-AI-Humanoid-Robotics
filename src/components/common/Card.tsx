import React from 'react';
import clsx from 'clsx';

interface CardProps {
  children: React.ReactNode;
  className?: string;
  shadow?: 'sm' | 'md' | 'lg' | 'none';
  padding?: 'sm' | 'md' | 'lg' | 'none';
}

const Card: React.FC<CardProps> = ({
  children,
  className = '',
  shadow = 'md',
  padding = 'md',
}) => {
  const shadowClasses = clsx({
    'shadow-sm': shadow === 'sm',
    'shadow-md': shadow === 'md',
    'shadow-lg': shadow === 'lg',
    'shadow-none': shadow === 'none',
  });

  const paddingClasses = clsx({
    'p-3': padding === 'sm',
    'p-6': padding === 'md',
    'p-8': padding === 'lg',
    'p-0': padding === 'none',
  });

  return (
    <div
      className={clsx(
        'bg-white dark:bg-gray-800 rounded-lg border border-gray-200 dark:border-gray-700',
        shadowClasses,
        paddingClasses,
        className
      )}
    >
      {children}
    </div>
  );
};

interface CardHeaderProps {
  children: React.ReactNode;
  className?: string;
}

const CardHeader: React.FC<CardHeaderProps> = ({ children, className = '' }) => (
  <div className={clsx('pb-4', className)}>{children}</div>
);

interface CardTitleProps {
  children: React.ReactNode;
  className?: string;
}

const CardTitle: React.FC<CardTitleProps> = ({ children, className = '' }) => (
  <h3 className={clsx('text-xl font-semibold text-gray-900 dark:text-white', className)}>
    {children}
  </h3>
);

interface CardDescriptionProps {
  children: React.ReactNode;
  className?: string;
}

const CardDescription: React.FC<CardDescriptionProps> = ({ children, className = '' }) => (
  <p className={clsx('text-sm text-gray-500 dark:text-gray-400', className)}>
    {children}
  </p>
);

interface CardContentProps {
  children: React.ReactNode;
  className?: string;
}

const CardContent: React.FC<CardContentProps> = ({ children, className = '' }) => (
  <div className={className}>{children}</div>
);

interface CardFooterProps {
  children: React.ReactNode;
  className?: string;
}

const CardFooter: React.FC<CardFooterProps> = ({ children, className = '' }) => (
  <div className={clsx('pt-4', className)}>{children}</div>
);

export { Card, CardHeader, CardTitle, CardDescription, CardContent, CardFooter };