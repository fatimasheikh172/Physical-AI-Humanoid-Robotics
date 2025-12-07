# Quickstart Guide: Frontend Interface for Physical AI & Humanoid Robotics Platform

**Feature**: Frontend Interface for Physical AI & Humanoid Robotics Platform
**Date**: 2025-12-07
**Guide Version**: 1.0.0

## Overview

This quickstart guide will help you set up and run the frontend interface for the Physical AI & Humanoid Robotics learning platform. This modern, AI-native interface provides an immersive learning experience for students exploring Physical AI and humanoid robotics concepts.

## Prerequisites

Before starting, ensure you have the following installed:

### System Requirements
- **Operating System**: macOS, Linux, or Windows with WSL2
- **Node.js**: Version 18.x or higher (LTS recommended)
- **npm**: Version 8.x or higher (comes with Node.js)
- **Git**: Version 2.30 or higher
- **Memory**: 8GB RAM minimum, 16GB recommended
- **Disk Space**: 2GB available space

### Development Tools
- **IDE/Editor**: VS Code, WebStorm, or similar with JavaScript/React support
- **Terminal**: Built-in terminal for your OS or Git Bash (Windows)

## Initial Setup

### 1. Clone the Repository

```bash
# Clone the repository
git clone https://github.com/your-org/physical-ai-frontend.git

# Navigate to the project directory
cd physical-ai-frontend
```

### 2. Install Dependencies

```bash
# Install project dependencies using npm
npm install

# Or if using yarn (ensure yarn is installed globally)
yarn install
```

### 3. Create Environment Configuration

```bash
# Copy the environment template
cp .env.example .env.local

# Edit the .env.local file with your configuration
```

**Environment Variables:**
- `REACT_APP_API_BASE_URL` - Base URL for backend API calls
- `REACT_APP_AI_TUTOR_API_KEY` - API key for AI tutor integration
- `REACT_APP_GA_MEASUREMENT_ID` - Google Analytics measurement ID (optional)

### 4. Verify Installation

```bash
# Run the development server
npm run dev

# Or if using yarn
yarn dev
```

The application should now be available at `http://localhost:3000`.

## Project Structure

Understanding the directory structure will help you navigate and modify the application:

```
physical-ai-frontend/
├── public/                    # Static assets and HTML template
├── src/                      # Source code
│   ├── components/           # Reusable UI components
│   │   ├── ui/              # Basic UI elements (buttons, cards, modals)
│   │   ├── common/          # Shared components (Header, Footer, Sidebar)
│   │   ├── modules/         # Module-specific components
│   │   ├── ai-tutor/        # AI tutor interface components
│   │   ├── quizzes/         # Quiz and assessment components
│   │   └── labs/            # Interactive lab components
│   ├── pages/                # Page-level components
│   │   ├── Home/            # Homepage
│   │   ├── Modules/         # Module index pages
│   │   ├── ModuleDetail/    # Individual module pages
│   │   ├── Chapter/         # Chapter content pages
│   │   ├── Dashboard/       # User dashboard
│   │   └── Profile/         # User profile
│   ├── styles/               # CSS and Tailwind configuration
│   ├── hooks/                # Custom React hooks
│   ├── contexts/             # React Context providers
│   ├── utils/                # Utility functions
│   ├── services/             # API service clients
│   └── types/                # TypeScript type definitions
├── docs/                     # Documentation and textbook content
├── i18n/                     # Internationalization files
├── docusaurus.config.js      # Docusaurus configuration
├── sidebars.js               # Navigation sidebar configuration
├── package.json              # Project dependencies and scripts
└── tailwind.config.js        # Tailwind CSS configuration
```

## Running the Application

### Development Mode

```bash
# Start the development server with hot reloading
npm run dev

# Or build and serve the production build locally
npm run build && npm run serve
```

### Building for Production

```bash
# Create a production build
npm run build

# The build output will be in the `build/` directory
# Serve the build locally to test
npm run serve
```

## Key Development Workflows

### 1. Adding New Pages

To add a new page for a module or feature:

1. Create a new component in `src/pages/`
2. Add routing in `src/App.js` or via Docusaurus configuration for documentation pages
3. Create navigation links if needed

**Example: Adding a New Module Page**
```jsx
// src/pages/NewModule.jsx
import React from 'react';
import Layout from '@theme/Layout';

export default function NewModule() {
  return (
    <Layout title="New Module" description="Physical AI and Robotics Learning">
      <div className="container margin-vert--lg">
        <header className="text--center">
          <h1 className="hero__title">New Module</h1>
          <p className="hero__subtitle">Learning objectives and content overview</p>
        </header>
        <div className="row">
          <div className="col">
            {/* Module content */}
          </div>
        </div>
      </div>
    </Layout>
  );
}
```

### 2. Creating Reusable Components

Components should follow these patterns:

1. Place reusable components in `src/components/ui/`
2. Module-specific components in `src/components/modules/`
3. Use descriptive naming (e.g., `QuizCard.jsx`, `LabExercise.jsx`)
4. Follow React best practices (functional components, hooks)

**Example: Creating a Quiz Card Component**
```jsx
// src/components/ui/QuizCard.jsx
import React from 'react';

const QuizCard = ({ title, description, duration, onClick }) => {
  return (
    <div 
      className="border rounded-lg p-6 shadow-md hover:shadow-lg transition-shadow cursor-pointer bg-white dark:bg-gray-800"
      onClick={onClick}
    >
      <h3 className="text-xl font-bold mb-2">{title}</h3>
      <p className="text-gray-600 dark:text-gray-300 mb-4">{description}</p>
      <div className="flex justify-between items-center">
        <span className="text-sm text-blue-500">{duration} minutes</span>
        <button className="bg-blue-500 hover:bg-blue-600 text-white px-4 py-2 rounded">
          Start Quiz
        </button>
      </div>
    </div>
  );
};

export default QuizCard;
```

### 3. Integrating AI Tutor

The AI tutor component is a critical feature of the platform. When developing new sections, ensure the AI tutor integration follows these patterns:

1. Include the AI tutor component toggle in all content pages
2. Pass relevant context to the AI tutor for better responses
3. Implement proper loading states and error handling

## Key Features and Components

### 1. AI Tutor Interface

The AI tutor is integrated into every content page:

```jsx
// Example of AI Tutor integration in a content page
import AITutorPanel from '../components/ai-tutor/Panel';

function ChapterPage() {
  const [isAITutorOpen, setIsAITutorOpen] = useState(false);
  
  return (
    <Layout>
      <main>
        {/* Chapter content */}
        <div className="chapter-content">
          {/* Content here */}
        </div>
        
        {/* AI Tutor Panel */}
        <AITutorPanel 
          isOpen={isAITutorOpen} 
          onClose={() => setIsAITutorOpen(false)}
          context={{ moduleId, chapterId }}
        />
        
        {/* AI Tutor Toggle Button */}
        <button 
          className="ai-tutor-toggle"
          onClick={() => setIsAITutorOpen(true)}
        >
          💬 Ask AI Tutor
        </button>
      </main>
    </Layout>
  );
}
```

### 2. Responsive Design

The interface is fully responsive. Test at various breakpoints:

- Mobile: 320px - 767px
- Tablet: 768px - 1023px  
- Desktop: 1024px+

Use Tailwind's responsive prefixes:
```jsx
<div className="block md:flex">
  <div className="w-full md:w-1/2">Content for all screensizes</div>
  <div className="hidden md:block md:w-1/2">Only on tablets and up</div>
</div>
```

### 3. Internationalization (i18n)

The application supports multiple languages:

1. Translation files in `/i18n/` directory
2. React context for language switching
3. Components that adjust for text direction (RTL)

## Testing the Application

### Running Tests

```bash
# Run unit tests
npm test

# Run tests in watch mode
npm run test:watch

# Run E2E tests (if Playwright/Cypress is set up)
npm run e2e
```

### Quality Checks

```bash
# Run ESLint for code quality
npm run lint

# Run Prettier for code formatting
npm run format

# Run all checks before commit
npm run verify
```

## Common Tasks and Troubleshooting

### Environment Issues

1. **Module not found errors**: Run `npm install` to reinstall dependencies
2. **Port already in use**: Use `npm run dev -- --port 3001` to run on a different port
3. **API calls failing**: Check that `REACT_APP_API_BASE_URL` is properly set in your environment file

### Performance Optimization

1. **Slow build times**: Check if your antivirus is scanning the node_modules directory
2. **Large bundle size**: Run `npm run analyze` to identify large dependencies
3. **Slow page loads**: Implement code splitting for page components

### UI/UX Testing

1. **Cross-browser compatibility**: Test in Chrome, Firefox, Safari, and Edge
2. **Accessibility**: Use axe DevTools or Lighthouse to check accessibility
3. **Responsive design**: Test on actual devices when possible, not just browser resizing

## Development Best Practices

### Code Style

1. Use functional components and hooks instead of class components
2. Follow the ESLint and Prettier configuration in the project
3. Use descriptive variable and function names
4. Keep components focused on a single responsibility

### Git Workflow

1. Create feature branches for new functionality: `feature/new-component-name`
2. Write clear, descriptive commit messages
3. Submit pull requests for code review before merging
4. Use conventional commits (e.g., `feat: add quiz component`)

### Performance

1. Optimize images and assets before adding to the project
2. Use React's `memo` and `useMemo` for performance optimization where needed
3. Implement lazy loading for non-critical components
4. Profile performance during development using React DevTools Profiler

## Next Steps

1. Explore the `/docs/` directory for textbook content structure
2. Review the API integration patterns in `/src/services/`
3. Customize the theme and design elements in `/src/styles/`
4. Add new module content following the established patterns
5. Test the complete user flow from homepage to module completion

For more detailed information about each feature, API specifications, and advanced tutorials, refer to:
- The textbook content at `http://localhost:3000`
- The API documentation at `http://localhost:8000/docs` (when backend is running)
- The component library in Storybook (if configured)