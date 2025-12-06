import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  
  bookSidebar: [
    {
      type: 'doc',
      id: 'hardware-requirements',
      label: 'Hardware Requirements',
    },
    {
      type: 'category',
      label: 'Week 1 - Intro to Agentic AI & Robotics',
      items: [
        'book/week-1/index',
      ],
    },
    {
      type: 'category',
      label: 'Week 2 - ROS 2 Deep Dive',
      items: [
        'book/week-2/index',
      ],
    },
    {
      type: 'category',
      label: 'Week 3 - Introduction to Simulation with Gazebo',
      items: [
        'book/week-3/index',
      ],
    },
    {
      type: 'category',
      label: 'Week 4 - Sensors and Perception in Simulation',
      items: [
        'book/week-4/index',
      ],
    },
    {
      type: 'category',
      label: 'Week 5 - Introduction to NVIDIA Isaac SDK',
      items: [
        'book/week-5/index',
      ],
    },
    {
      type: 'category',
      label: 'Week 6 - Computer Vision for Robotics',
      items: [
        'book/week-6/index',
      ],
    },
    {
      type: 'category',
      label: 'Week 7 - Navigation and Path Planning',
      items: [
        'book/week-7/index',
      ],
    },
    {
      type: 'category',
      label: 'Week 8 - Manipulation and Control',
      items: [
        'book/week-8/index',
      ],
    },
    {
      type: 'category',
      label: 'Week 9 - Integrating Language Models (LLMs)',
      items: [
        'book/week-9/index',
      ],
    },
    {
      type: 'category',
      label: 'Week 10 - Vision-Language Models (VLMs) in Robotics',
      items: [
        'book/week-10/index',
      ],
    },
    {
      type: 'category',
      label: 'Week 11 - System Integration',
      items: [
        'book/week-11/index',
      ],
    },
    {
      type: 'category',
      label: 'Week 12 - Capstone Project - Part 1',
      items: [
        'book/week-12/index',
      ],
    },
    {
      type: 'category',
      label: 'Week 13 - Capstone Project - Part 2',
      items: [
        'book/week-13/index',
      ],
    },
  ],
};

export default sidebars;