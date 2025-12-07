import React, { ReactNode } from 'react';
import clsx from 'clsx';

interface LayoutProps {
  children: ReactNode;
  className?: string;
  variant?: 'default' | 'centered' | 'sidebar' | 'fullscreen';
  withSidebar?: boolean;
  sidebarContent?: ReactNode;
  headerContent?: ReactNode;
  footerContent?: ReactNode;
}

const Layout: React.FC<LayoutProps> = ({
  children,
  className = '',
  variant = 'default',
  withSidebar = false,
  sidebarContent,
  headerContent,
  footerContent
}) => {
  const layoutClasses = clsx(
    'min-h-screen flex flex-col',
    className
  );

  if (variant === 'fullscreen') {
    return (
      <div className={layoutClasses}>
        {children}
      </div>
    );
  }

  if (variant === 'centered') {
    return (
      <div className={clsx(layoutClasses, 'items-center justify-center')}>
        <main className="w-full max-w-4xl p-4">{children}</main>
      </div>
    );
  }

  return (
    <div className={layoutClasses}>
      {headerContent && (
        <header className="bg-white dark:bg-gray-900 border-b border-gray-200 dark:border-gray-800">
          {headerContent}
        </header>
      )}
      
      <div className="flex flex-1">
        {withSidebar && sidebarContent && (
          <aside className="w-64 bg-gray-50 dark:bg-gray-800 border-r border-gray-200 dark:border-gray-700 p-4 hidden md:block">
            {sidebarContent}
          </aside>
        )}
        
        <main className={clsx(
          'flex-1',
          withSidebar ? 'p-4 md:p-6' : 'p-4'
        )}>
          {children}
        </main>
      </div>
      
      {footerContent && (
        <footer className="bg-white dark:bg-gray-900 border-t border-gray-200 dark:border-gray-800 py-6">
          {footerContent}
        </footer>
      )}
    </div>
  );
};

export default Layout;