import i18n from 'i18next';
import { initReactI18next } from 'react-i18next';

// Import translation files
import enTranslation from '../locales/en/translation.json';
import urTranslation from '../locales/ur/translation.json';

// Create locales directories if they don't exist
// Note: You'll need to create these files with actual translations

const resources = {
  en: {
    translation: enTranslation
  },
  ur: {
    translation: urTranslation
  }
};

i18n
  .use(initReactI18next) // Passes i18n down to react-i18next
  .init({
    resources,
    lng: 'en', // Default language
    fallbackLng: 'en', // Fallback language if translation is missing
    interpolation: {
      escapeValue: false // React already escapes by default
    },
    keySeparator: '.', // Use nested translation keys
    nsSeparator: ':' // Namespace separator
  });

export default i18n;