import React from 'react';
import { BrowserRouter as Router, Route } from 'react-router-dom';
import { Navigate } from 'react-router-dom';
import { Routes } from 'react-router-dom';
import { ThemeProvider } from './ThemeToggle';

import TextbookChapterPage from '../../pages/textbook/TextbookChapterPage';
import Layout from './Layout';

const AppRouter: React.FC = () => {
  return (
    <ThemeProvider>
      <Router>
        <Layout>
          <Routes>
            <Route path="/" element={<Navigate to="/textbook/introduction" replace />} />
            <Route path="/textbook/:slug" element={<TextbookChapterPage />} />
            <Route path="*" element={<Navigate to="/" replace />} />
          </Routes>
        </Layout>
      </Router>
    </ThemeProvider>
  );
};

export default AppRouter;