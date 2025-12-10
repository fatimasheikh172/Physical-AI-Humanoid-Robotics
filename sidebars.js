// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {

  // ✅ 📘 TEXTBOOK SIDEBAR
  textbookSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      collapsed: false,
      items: [ 'overview'],
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
      ],
    },
    {
      type: 'category',
      label: 'AI for Robotics',
      collapsed: false,
      items: [
        'robot-locomotion'
      ],
    },
  ],

  // ✅ 🧩 MODULES SIDEBAR
  modulesSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      collapsed: false,
      items: [
        'modules/ros2-fundamentals/week1/theory',
        'modules/ros2-fundamentals/week1/engineering',
        'modules/ros2-fundamentals/week1/installation-guide',
        'modules/ros2-fundamentals/week1/ros2-setup',
        'modules/ros2-fundamentals/week1/advanced',
        'modules/ros2-fundamentals/week2/communication-theory',
        'modules/ros2-fundamentals/week3/urdf-introduction',
        'modules/ros2-fundamentals/week3/rclpy-architecture',
        'modules/ros2-fundamentals/week3/ai-ros-pipeline',
        'modules/ros2-fundamentals/urdf-grading',
        'modules/ros2-fundamentals/final-assessment',
      ],
    },
    {
      type: 'category',
      label: 'Module 02: Digital Twin for Robotic Systems',
      collapsed: false,
      items: [
        'modules/digital-twin/introduction',
        {
          type: 'category',
          label: 'Week 1: Gazebo Physics & Sensors',
          collapsed: true,
          items: [
            'modules/digital-twin/week1/physics-setup',
            'modules/digital-twin/week1/sensor-integration',
            'modules/digital-twin/week1/validation',
          ],
        },
        {
          type: 'category',
          label: 'Week 2: Unity Visualization',
          collapsed: true,
          items: [
            'modules/digital-twin/week2/unity-setup',
            'modules/digital-twin/week2/ros-bridge',
            'modules/digital-twin/week2/visualization',
          ],
        },
        {
          type: 'category',
          label: 'Week 3: Perception & Analysis',
          collapsed: true,
          items: [
            'modules/digital-twin/week3/perception-pipeline',
            'modules/digital-twin/week3/simulation-fidelity',
            'modules/digital-twin/week3/capstone-project',
          ],
        },
        'modules/digital-twin/assessment',
      ],
    },
    {
      type: 'category',
      label: 'Module 03: The AI-Robot Brain (NVIDIA Isaac™)',
      collapsed: false,
      items: [
        'modules/ai-robot-brain/introduction',
        {
          type: 'category',
          label: 'Week 1: Isaac Sim Physics & Sensors',
          collapsed: true,
          items: [
            'modules/ai-robot-brain/week1/physics-setup',
            'modules/ai-robot-brain/week1/sensor-integration',
            'modules/ai-robot-brain/week1/synthetic-generation',
          ],
        },
        {
          type: 'category',
          label: 'Week 2: Isaac ROS VSLAM',
          collapsed: true,
          items: [
            'modules/ai-robot-brain/week2/vslam-implementation',
          ],
        },
        {
          type: 'category',
          label: 'Week 3: Bipedal Navigation & Training',
          collapsed: true,
          items: [
            'modules/ai-robot-brain/week3/bipedal-nav',
            'modules/ai-robot-brain/week3/perception-training',
            'modules/ai-robot-brain/week3/capstone-project',
          ],
        },
        'modules/ai-robot-brain/assessment',
      ],
    },
    {
      type: 'category',
      label: 'Module 04: Vision-Language-Action (VLA)',
      collapsed: false,
      items: [
        'modules/vla-module/introduction',
        {
          type: 'category',
          label: 'Week 1: Voice Recognition & Cognitive Planning',
          collapsed: true,
          items: [
            'modules/vla-module/week1/voice-processing',
            'modules/vla-module/week1/cognitive-planning',
            'modules/vla-module/week1/llm-integration',
          ],
        },
        {
          type: 'category',
          label: 'Week 2: Action Execution & Vision Perception',
          collapsed: true,
          items: [
            'modules/vla-module/week2/action-execution',
            'modules/vla-module/week2/vision-perception',
            'modules/vla-module/week2/perception-integration',
          ],
        },
        {
          type: 'category',
          label: 'Week 3: Capstone Integration & Validation',
          collapsed: true,
          items: [
            'modules/vla-module/week3/full-integration',
            'modules/vla-module/week3/capstone-project',
            'modules/vla-module/week3/performance-validation',
          ],
        },
        'modules/vla-module/assessment',
      ],
    },

  ],
};

export default sidebars;

