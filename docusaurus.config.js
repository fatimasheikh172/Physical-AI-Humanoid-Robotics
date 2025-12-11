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

  onBrokenLinks: 'warn',

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
    // [
    //   '@docusaurus/plugin-content-docs',
    //   {
    //     id: 'ur',
    //     path: 'i18n/ur/docusaurus-plugin-content-docs/current',
    //     routeBasePath: 'ur',
    //     // NOTE: Uncomment when Urdu content is ready
    //     // sidebarPath: require.resolve('./i18n/ur/docusaurus-plugin-content-docs/current/sidebars.js'),
    //   },
    // ],
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
          title: 'Modules',
          items: [
            { label: 'Module 1: ROS2 Fundamentals', to: '/modules/ros2-fundamentals/week1/theory' },
            { label: 'Module 2: Digital Twin', to: '/modules/digital-twin/introduction' },
            { label: 'Module 3: AI Robot Brain', to: '/modules/ai-robot-brain/introduction' },
            { label: 'Module 4: Vision-Language-Action', to: '/modules/vla-module/introduction' },
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
          title: 'Resources',
          items: [
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
