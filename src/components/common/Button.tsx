import React from 'react';
import clsx from 'clsx';

interface ButtonProps {
  children: React.ReactNode;
  variant?: 'primary' | 'secondary' | 'ghost' | 'outline';
  size?: 'sm' | 'md' | 'lg';
  disabled?: boolean;
  className?: string;
  type?: 'button' | 'submit' | 'reset';
  onClick?: () => void;
}

const Button: React.FC<ButtonProps> = ({
  children,
  variant = 'primary',
  size = 'md',
  disabled = false,
  className = '',
  type = 'button',
  onClick,
}) => {
  const baseClasses = 'inline-flex items-center justify-center rounded-md font-medium transition-colors focus:outline-none focus:ring-2 focus:ring-offset-2 disabled:opacity-50 disabled:pointer-events-none';

  const variantClasses = clsx({
    // Primary variant
    'bg-blue-600 text-white hover:bg-blue-700 focus:ring-blue-500': variant === 'primary',
    // Secondary variant
    'bg-gray-600 text-white hover:bg-gray-700 focus:ring-gray-500': variant === 'secondary',
    // Ghost variant
    'bg-transparent text-gray-700 hover:bg-gray-100 focus:ring-gray-500 dark:text-gray-200 dark:hover:bg-gray-800': variant === 'ghost',
    // Outline variant
    'border border-gray-300 bg-transparent text-gray-700 hover:bg-gray-100 focus:ring-gray-500 dark:border-gray-600 dark:text-gray-200 dark:hover:bg-gray-800': variant === 'outline',
  });

  const sizeClasses = clsx({
    'text-xs py-1.5 px-3': size === 'sm',
    'text-sm py-2 px-4': size === 'md',
    'text-base py-3 px-6': size === 'lg',
  });

  const disabledClasses = disabled ? 'opacity-50 cursor-not-allowed' : '';

  return (
    <button
      type={type}
      disabled={disabled}
      className={clsx(baseClasses, variantClasses, sizeClasses, disabledClasses, className)}
      onClick={onClick}
    >
      {children}
    </button>
  );
};

export default Button;