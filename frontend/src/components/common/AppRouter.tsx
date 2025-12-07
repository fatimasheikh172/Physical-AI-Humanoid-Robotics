import React from 'react';
import { BrowserRouter as Router, Routes, Route, Navigate } from 'react-router-dom';
import { ThemeProvider } from '../components/common/ThemeToggle';
import Layout from '../components/common/Layout';
import TextbookChapterPage from '../pages/textbook/TextbookChapterPage';

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