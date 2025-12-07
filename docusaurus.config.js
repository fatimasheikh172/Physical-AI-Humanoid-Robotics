// @ts-check
import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Learn embodied intelligence, humanoid robotics, and AI-powered physical agents',
  favicon: 'img/robotics-favicon.ico',

  future: {
    v4: true,
  },

  url: 'https://physicalai-textbook.com',
  baseUrl: '/',

  organizationName: 'physicalai',
  projectName: 'textbook',

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
      },
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
           sidebarPath: require.resolve('./sidebars.js'),
           routeBasePath: '/',
           editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },

        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },

        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],

  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'ur',
        path: 'i18n/ur/docusaurus-plugin-content-docs/current',
        routeBasePath: 'ur',
        sidebarPath: require.resolve('./sidebars.js'),
      },
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',

    colorMode: {
      respectPrefersColorScheme: true,
    },

    // ✅ ✅ NAVBAR WITH 2 SIDE BARS
    navbar: {
      title: 'Physical AI & Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/robotics-logo.svg',
      },

      items: [
        {
          type: 'docSidebar',
          sidebarId: 'textbookSidebar', // ✅ TEXTBOOK
          position: 'left',
          label: 'Textbook',
        },
        {
          type: 'docSidebar',
          sidebarId: 'modulesSidebar', // ✅ MODULES
          position: 'left',
          label: 'Modules',
        },
        { to: '/blog', label: 'Blog', position: 'left' },
        {
          href: 'https://github.com/physicalai/textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Textbook',
          items: [
            { label: 'Introduction', to: '/docs/intro' },
            { label: 'Physical AI Foundation', to: '/docs/physical-ai-foundation' },
          ],
        },
        {
          title: 'Community',
          items: [
            { label: 'Discord', href: 'https://discord.gg/physicalai' },
            { label: 'X', href: 'https://x.com/physicalai' },
          ],
        },
        {
          title: 'More',
          items: [
            { label: 'Blog', to: '/blog' },
            { label: 'GitHub', href: 'https://github.com/physicalai/textbook' },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics.`,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  },
};

export default config;
