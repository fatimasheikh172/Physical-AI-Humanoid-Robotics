# Quickstart Guide: Physical AI & Humanoid Robotics Textbook Frontend

## Prerequisites

- Node.js 18 or higher
- npm or yarn package manager
- Git version control
- Modern web browser with WebGL support (Chrome, Firefox, Safari, Edge)

## Getting Started

### 1. Clone the Repository
```bash
git clone https://github.com/your-org/physical-ai-textbook.git
cd physical-ai-textbook
```

### 2. Install Dependencies
```bash
npm install
# or
yarn install
```

### 3. Environment Setup
Create a `.env` file in the root directory with the following variables:

```env
# API Configuration
REACT_APP_API_BASE_URL=https://api.physicalai-textbook.com/v1
REACT_APP_API_KEY=your_api_key_here

# AI Tutor Configuration
REACT_APP_AI_TUTOR_ENDPOINT=wss://ai-tutor.physicalai-textbook.com
REACT_APP_AI_TUTOR_API_KEY=your_ai_tutor_api_key

# Analytics (optional)
REACT_APP_ANALYTICS_ID=your_analytics_id
```

### 4. Development Server
```bash
npm run dev
# or
yarn dev
```

The application will start on `http://localhost:3000`

## Project Structure

```
frontend/
├── src/
│   ├── components/           # Reusable UI components
│   │   ├── common/          # General UI elements
│   │   ├── textbook/        # Textbook-specific components
│   │   ├── ai-tutor/        # AI Tutor interface components
│   │   └── simulations/     # Simulation interface components
│   ├── pages/              # Page-level components
│   │   ├── textbook/       # Textbook chapter pages
│   │   ├── dashboard/      # User dashboard
│   │   └── common/         # Shared pages (home, about, etc.)
│   ├── services/           # API calls and business logic
│   ├── hooks/              # Custom React hooks
│   ├── utils/              # Utility functions
│   ├── styles/             # Global styles and Tailwind config
│   └── types/              # TypeScript type definitions
├── public/                 # Static assets
├── docs/                   # Additional documentation
└── package.json            # Project dependencies and scripts
```

## Key Technologies Used

### Docusaurus
The frontend uses Docusaurus v3 as the documentation framework with custom React components for textbook features.

```bash
# To build the static site
npm run build

# To serve the built site locally
npm run serve
```

### Three.js for 3D Visualizations
For 3D robotics simulations and visualizations:

```javascript
import { Canvas } from '@react-three/fiber';
import { OrbitControls, useGLTF } from '@react-three/drei';

// Example usage in a component
function RobotModel({ modelPath }) {
  const { scene } = useGLTF(modelPath);
  return <primitive object={scene} />;
}
```

### Tailwind CSS
For styling, the project uses Tailwind CSS with custom configurations:

```javascript
// tailwind.config.js
module.exports = {
  content: [
    "./src/**/*.{js,jsx,ts,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        'robotics-dark': '#0a0a0a',
        'robotics-blue': '#0ea5e9',
        'robotics-accent': '#8b5cf6',
      }
    }
  },
  plugins: [],
}
```

## Core Features Implementation

### 1. Chapter Navigation
The textbook chapters are organized in a sidebar navigation that tracks progress:

```javascript
// Example of how chapter progress is tracked
const trackProgress = async (chapterId, progressData) => {
  try {
    await apiClient.post(`/progress`, {
      chapterId,
      ...progressData
    });
  } catch (error) {
    console.error('Failed to track progress:', error);
  }
};
```

### 2. AI Tutor Integration
The AI Tutor component is available on every page:

```jsx
// AI Tutor Component
import { AITutor } from '../components/ai-tutor/AITutor';

function TextbookPage({ chapter }) {
  return (
    <div className="flex">
      <main className="flex-1">
        <ChapterContent chapter={chapter} />
      </main>
      <aside className="w-96 border-l">
        <AITutor context={chapter.title} />
      </aside>
    </div>
  );
}
```

### 3. Interactive Simulations
Each chapter with simulations includes interactive 3D components:

```jsx
import { SimulationViewer } from '../components/simulations/SimulationViewer';

function ChapterWithSimulations({ chapter }) {
  if (chapter.simulations.length > 0) {
    return (
      <div>
        <ChapterContent chapter={chapter} />
        {chapter.simulations.map(sim => (
          <SimulationViewer key={sim.id} simulation={sim} />
        ))}
      </div>
    );
  }
  return <ChapterContent chapter={chapter} />;
}
```

## Internationalization

The platform supports English and Urdu with language toggle:

```javascript
import { useTranslation } from 'react-i18next';

function LanguageToggle() {
  const { i18n } = useTranslation();
  
  const changeLanguage = (lng) => {
    i18n.changeLanguage(lng);
  };

  return (
    <select onChange={(e) => changeLanguage(e.target.value)}>
      <option value="en">English</option>
      <option value="ur">Urdu</option>
    </select>
  );
}
```

## Testing

### Running Tests
```bash
# Run all tests
npm test

# Run tests in watch mode
npm test -- --watch

# Run end-to-end tests
npm run e2e
```

### Test Structure
Tests are co-located with the components they test:

```
src/
├── components/
│   ├── Button/
│   │   ├── Button.jsx
│   │   └── Button.test.jsx
│   └── ai-tutor/
│       ├── AITutor.jsx
│       ├── AITutor.test.jsx
│       └── hooks/
│           ├── useTutor.test.js
```

## Building for Production

```bash
# Create a production build
npm run build

# The build artifacts will be stored in the `build/` directory
# They can be served using any static server
```

## Deployment

### Using Vercel
1. Install Vercel CLI: `npm i -g vercel`
2. Run `vercel` in your project directory

### Using GitHub Pages
1. Update `package.json` with your project's path:
   ```json
   {
     "homepage": "https://your-username.github.io/your-project-name"
   }
   ```
2. Run: `npm run deploy`

## Troubleshooting

### Common Issues

1. **3D Simulations not loading**
   - Ensure your browser supports WebGL
   - Check browser console for specific errors
   - Verify model assets are correctly referenced

2. **AI Tutor not responding**
   - Check API key configuration
   - Verify network connectivity
   - Check browser console for WebSocket errors

3. **Performance issues with 3D content**
   - Reduce simulation complexity for lower-end devices
   - Implement proper level-of-detail (LOD) techniques
   - Use performance monitoring tools to identify bottlenecks

## Next Steps

1. Review the component library in `src/components/common`
2. Explore the API service implementations in `src/services`
3. Check out the custom Docusaurus theme components
4. Look at existing chapter implementations in `src/pages/textbook`