// Environment configuration
// This file defines environment-specific variables for the application
// In Docusaurus, environment variables are typically prefixed with "REACT_APP_"

export const API_CONFIG = {
  // Base API URL - defaults to production, can be overridden with environment variable
  BASE_URL: process.env.REACT_APP_API_BASE_URL || 'https://api.physicalai-textbook.com/v1',
  
  // Timeout for API requests (in milliseconds)
  REQUEST_TIMEOUT: 30000, // 30 seconds
  
  // Retry settings
  MAX_RETRIES: 3,
  RETRY_DELAY: 1000, // 1 second
  
  // Rate limiting (as defined in API contracts)
  RATE_LIMITS: {
    GENERAL: 1000, // 1000 requests/hour
    AI_TUTOR: 50, // 50 queries/hour
    SIMULATIONS: 20 // 20 initializations/hour
  }
};

export const APP_CONFIG = {
  // Application name
  NAME: process.env.REACT_APP_NAME || 'Physical AI & Humanoid Robotics Textbook',
  
  // Current environment
  ENVIRONMENT: process.env.NODE_ENV || 'development',
  
  // Feature flags
  FEATURES: {
    AI_TUTOR_ENABLED: process.env.REACT_APP_AI_TUTOR_ENABLED !== 'false',
    SIMULATIONS_ENABLED: process.env.REACT_APP_SIMULATIONS_ENABLED !== 'false',
    PROGRESS_TRACKING_ENABLED: process.env.REACT_APP_PROGRESS_TRACKING_ENABLED !== 'false'
  }
};

// Export a function to validate environment configuration
export const validateEnvironment = () => {
  const errors: string[] = [];
  
  // Validate required environment variables
  if (!process.env.REACT_APP_API_BASE_URL) {
    console.warn('REACT_APP_API_BASE_URL is not set, using default');
  }
  
  // In the future, you could add more validation here
  
  if (errors.length > 0) {
    throw new Error(`Environment configuration errors: ${errors.join(', ')}`);
  }
  
  return true;
};