import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI Textbook',
  tagline: 'Master Humanoid Robotics & Intelligent Physical Systems',
  favicon: 'img/favicon.ico',

  url: 'https://your-docusaurus-site.example.com',
  baseUrl: '/',

  organizationName: 'physical-ai',
  projectName: 'textbook',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'ignore',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'], // Only default locale - using manual doc plugins for language versions
  },

  presets: [
    [
      'classic',
      {
        docs: false, // Disabled - using language-specific doc plugins instead
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs-en',
        path: 'docs/en',
        routeBasePath: 'docs/en',
        sidebarPath: './sidebars.en.ts',
      },
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs-ur',
        path: 'docs/ur',
        routeBasePath: 'docs/ur',
        sidebarPath: './sidebars.ur.ts',
      },
    ],
  ],

  themeConfig: {
    navbar: {
      title: '🤖 Physical AI',
      items: [
        {
          to: '/docs/en/module-0-setup/intro',
          label: 'Docs',
          position: 'left',
        },
        {
          type: 'custom-search',
          position: 'right',
        },
        {
          type: 'custom-langSwitcher',
          position: 'right',
        },
        {
          type: 'custom-authNavbarItem',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
