import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
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
    component: ComponentCreator('/docs/en', '5b5'),
    routes: [
      {
        path: '/docs/en',
        component: ComponentCreator('/docs/en', '666'),
        routes: [
          {
            path: '/docs/en',
            component: ComponentCreator('/docs/en', 'fa6'),
            routes: [
              {
                path: '/docs/en/module-0-setup/intro',
                component: ComponentCreator('/docs/en/module-0-setup/intro', '247'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/en/module-1-nervous-system/intro-to-ros2',
                component: ComponentCreator('/docs/en/module-1-nervous-system/intro-to-ros2', 'ee9'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/en/module-1-nervous-system/nodes-and-topics',
                component: ComponentCreator('/docs/en/module-1-nervous-system/nodes-and-topics', '0a0'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/en/module-1-nervous-system/urdf-modeling',
                component: ComponentCreator('/docs/en/module-1-nervous-system/urdf-modeling', '0ce'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/en/module-2-digital-twin/gazebo-fortress-setup',
                component: ComponentCreator('/docs/en/module-2-digital-twin/gazebo-fortress-setup', '880'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/en/module-2-digital-twin/intro-digital-twin',
                component: ComponentCreator('/docs/en/module-2-digital-twin/intro-digital-twin', '36e'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/en/module-2-digital-twin/simulating-sensors',
                component: ComponentCreator('/docs/en/module-2-digital-twin/simulating-sensors', '2b7'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/en/module-2-digital-twin/unity-visualization',
                component: ComponentCreator('/docs/en/module-2-digital-twin/unity-visualization', 'f7d'),
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
    component: ComponentCreator('/docs/ur', 'f48'),
    routes: [
      {
        path: '/docs/ur',
        component: ComponentCreator('/docs/ur', '18d'),
        routes: [
          {
            path: '/docs/ur',
            component: ComponentCreator('/docs/ur', '01f'),
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
                path: '/docs/ur/module-1-nervous-system/intro-to-ros2',
                component: ComponentCreator('/docs/ur/module-1-nervous-system/intro-to-ros2', '0cd'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ur/module-1-nervous-system/nodes-and-topics',
                component: ComponentCreator('/docs/ur/module-1-nervous-system/nodes-and-topics', 'f61'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ur/module-1-nervous-system/urdf-modeling',
                component: ComponentCreator('/docs/ur/module-1-nervous-system/urdf-modeling', '373'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ur/module-2-digital-twin/gazebo-fortress-setup',
                component: ComponentCreator('/docs/ur/module-2-digital-twin/gazebo-fortress-setup', '832'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ur/module-2-digital-twin/intro-digital-twin',
                component: ComponentCreator('/docs/ur/module-2-digital-twin/intro-digital-twin', '1ed'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ur/module-2-digital-twin/simulating-sensors',
                component: ComponentCreator('/docs/ur/module-2-digital-twin/simulating-sensors', '274'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ur/module-2-digital-twin/unity-visualization',
                component: ComponentCreator('/docs/ur/module-2-digital-twin/unity-visualization', '9e7'),
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
