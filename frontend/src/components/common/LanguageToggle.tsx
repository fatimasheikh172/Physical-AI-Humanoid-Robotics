import React, { useContext } from 'react';
import { useTranslation } from 'react-i18next';

// Context for language management
const LanguageContext = React.createContext<{
  currentLanguage: string;
  availableLanguages: { code: string; name: string }[];
  changeLanguage: (lang: string) => void;
}>({
  currentLanguage: 'en',
  availableLanguages: [],
  changeLanguage: () => {},
});

export const useLanguage = () => useContext(LanguageContext);

interface LanguageToggleProps {
  className?: string;
}

const LanguageToggle: React.FC<LanguageToggleProps> = ({ className = '' }) => {
  const { i18n } = useTranslation();
  const availableLanguages = [
    { code: 'en', name: 'English' },
    { code: 'ur', name: 'Urdu' },
  ];

  const changeLanguage = (lang: string) => {
    i18n.changeLanguage(lang);
    // Save user preference to localStorage
    localStorage.setItem('preferredLanguage', lang);
  };

  // Set initial language based on user preference or browser language
  React.useEffect(() => {
    const savedLanguage = localStorage.getItem('preferredLanguage');
    const browserLanguage = navigator.language.substring(0, 2);
    
    if (savedLanguage) {
      i18n.changeLanguage(savedLanguage);
    } else if (availableLanguages.some(lang => lang.code === browserLanguage)) {
      i18n.changeLanguage(browserLanguage);
    }
  }, [i18n]);

  // Provide context value
  const contextValue = {
    currentLanguage: i18n.language,
    availableLanguages,
    changeLanguage,
  };

  return (
    <LanguageContext.Provider value={contextValue}>
      <select
        value={i18n.language}
        onChange={(e) => changeLanguage(e.target.value)}
        className={`px-3 py-2 border rounded-md ${className}`}
      >
        {availableLanguages.map((lang) => (
          <option key={lang.code} value={lang.code}>
            {lang.name}
          </option>
        ))}
      </select>
    </LanguageContext.Provider>
  );
};

export default LanguageToggle;