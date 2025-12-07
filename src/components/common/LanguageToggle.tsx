import React, { useEffect, useState } from 'react';
import { useTranslation } from 'react-i18next';
import Button from './Button';

const LanguageToggle: React.FC = () => {
  const { i18n } = useTranslation();
  const [currentLanguage, setCurrentLanguage] = useState(i18n.language);

  const languages = [
    { code: 'en', name: 'English' },
    { code: 'ur', name: 'Urdu' }
  ];

  const handleLanguageChange = (langCode: string) => {
    i18n.changeLanguage(langCode);
    setCurrentLanguage(langCode);
    // Store the selected language in localStorage for persistence
    localStorage.setItem('selectedLanguage', langCode);
  };

  // Set the initial language based on what's stored in localStorage or browser preference
  useEffect(() => {
    const storedLanguage = localStorage.getItem('selectedLanguage');
    if (storedLanguage && i18n.language !== storedLanguage) {
      i18n.changeLanguage(storedLanguage);
      setCurrentLanguage(storedLanguage);
    }
  }, [i18n]);

  return (
    <div className="flex items-center space-x-2">
      {languages.map((lang) => (
        <Button
          key={lang.code}
          variant={currentLanguage === lang.code ? 'primary' : 'outline'}
          size="sm"
          onClick={() => handleLanguageChange(lang.code)}
          className="capitalize"
        >
          {lang.name}
        </Button>
      ))}
    </div>
  );
};

export default LanguageToggle;