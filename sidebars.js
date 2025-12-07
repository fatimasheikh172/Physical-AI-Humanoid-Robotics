// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {

  // ✅ 📘 TEXTBOOK SIDEBAR
  textbookSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      collapsed: false,
      items: ['intro', 'overview'],
    },
    {
      type: 'category',
      label: 'Physical AI Fundamentals',
      collapsed: false,
      items: [
        'physical-ai-foundation',
        'embodied-intelligence',
        'robot-locomotion',
        'control-systems',
      ],
    },
    {
      type: 'category',
      label: 'Humanoid Robotics',
      collapsed: false,
      items: [
        'humanoid-design',
        'balance-algorithms',
        'locomotion-patters',
        'manipulation',
      ],
    },
    {
      type: 'category',
      label: 'AI for Robotics',
      collapsed: false,
      items: [
        'robot-learning',
        'perception-systems',
        'navigation',
        'human-robot-interaction',
      ],
    },
  ],

  // ✅ 🧩 MODULES SIDEBAR
  modulesSidebar: [
    {
      type: 'category',
      label: 'Core Robotics Modules',
      collapsed: false,
      items: [
        'modules/physical-ai-foundation',
        'modules/ros2',
        'modules/gazebo',
        'modules/isaac',
        'modules/robot-brain',
        'modules/vla',
        'modules/conversational',
        'modules/capstone',
      ],
    },
  ],
};

export default sidebars;
