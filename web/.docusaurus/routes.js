import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/exam',
    component: ComponentCreator('/exam', 'f72'),
    exact: true
  },
  {
    path: '/login',
    component: ComponentCreator('/login', 'f43'),
    exact: true
  },
  {
    path: '/logout',
    component: ComponentCreator('/logout', '0a7'),
    exact: true
  },
  {
    path: '/docs/en',
    component: ComponentCreator('/docs/en', 'b98'),
    routes: [
      {
        path: '/docs/en',
        component: ComponentCreator('/docs/en', '126'),
        routes: [
          {
            path: '/docs/en',
            component: ComponentCreator('/docs/en', '4b3'),
            routes: [
              {
                path: '/docs/en/module-0-setup/intro',
                component: ComponentCreator('/docs/en/module-0-setup/intro', '247'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/en/module-1-nervous-system/intro',
                component: ComponentCreator('/docs/en/module-1-nervous-system/intro', 'a1a'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/en/module-2-digital-twin/intro',
                component: ComponentCreator('/docs/en/module-2-digital-twin/intro', 'e1f'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/en/module-3-robot-brain/intro',
                component: ComponentCreator('/docs/en/module-3-robot-brain/intro', '76e'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/en/module-4-the-mind/intro',
                component: ComponentCreator('/docs/en/module-4-the-mind/intro', '319'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/en/module-5-capstone/intro',
                component: ComponentCreator('/docs/en/module-5-capstone/intro', 'a84'),
                exact: true,
                sidebar: "docs"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/docs/ur',
    component: ComponentCreator('/docs/ur', '2fa'),
    routes: [
      {
        path: '/docs/ur',
        component: ComponentCreator('/docs/ur', 'c8e'),
        routes: [
          {
            path: '/docs/ur',
            component: ComponentCreator('/docs/ur', '7a7'),
            routes: [
              {
                path: '/docs/ur/module-0-setup/intro',
                component: ComponentCreator('/docs/ur/module-0-setup/intro', 'dac'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ur/module-1-nervous-system/intro',
                component: ComponentCreator('/docs/ur/module-1-nervous-system/intro', '751'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ur/module-2-digital-twin/intro',
                component: ComponentCreator('/docs/ur/module-2-digital-twin/intro', 'f47'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ur/module-3-robot-brain/intro',
                component: ComponentCreator('/docs/ur/module-3-robot-brain/intro', 'bae'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ur/module-4-the-mind/intro',
                component: ComponentCreator('/docs/ur/module-4-the-mind/intro', 'a31'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ur/module-5-capstone/intro',
                component: ComponentCreator('/docs/ur/module-5-capstone/intro', '8d4'),
                exact: true,
                sidebar: "docs"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', 'e5f'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
