import React, { ReactNode } from 'react';
import LanguageToggle from './LanguageToggle';

interface LayoutProps {
  children: ReactNode;
  title?: string;
  showHeader?: boolean;
  showFooter?: boolean;
  showSidebar?: boolean;
}

const Layout: React.FC<LayoutProps> = ({
  children,
  title = 'Physical AI & Humanoid Robotics Textbook',
  showHeader = true,
  showFooter = true,
  showSidebar = true,
}) => {
  // Update document title when it changes
  React.useEffect(() => {
    document.title = title;
  }, [title]);

  return (
    <div className="min-h-screen flex flex-col bg-gray-50">
      {/* Header */}
      {showHeader && (
        <header className="bg-white shadow-sm">
          <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
            <div className="flex justify-between items-center h-16">
              <div className="flex items-center">
                <h1 className="text-xl font-bold text-gray-900">Physical AI Textbook</h1>
              </div>
              <div className="flex items-center space-x-4">
                <LanguageToggle />
                {/* Additional header elements would go here */}
              </div>
            </div>
          </div>
        </header>
      )}

      {/* Main Content */}
      <main className="flex-grow">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-8">
          {children}
        </div>
      </main>

      {/* Footer */}
      {showFooter && (
        <footer className="bg-white border-t">
          <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-6">
            <p className="text-center text-sm text-gray-500">
              &copy; {new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. All rights reserved.
            </p>
          </div>
        </footer>
      )}
    </div>
  );
};

export default Layout;