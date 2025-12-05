/**
 * Client-Side Search Index
 *
 * Simple text-based search through all documentation pages.
 * This runs entirely in the browser without needing a backend.
 */

export interface SearchDocument {
  title: string;
  content: string;
  url: string;
  module: string;
}

// Search index - all documentation pages
const documentIndex: SearchDocument[] = [
  // Module 0 - Workstation Setup
  {
    title: "Module 0: Workstation Setup",
    content: "Before diving into Physical AI and Humanoid Robotics, you need to set up your development workstation. Install Ubuntu 22.04 LTS, NVIDIA drivers, CUDA toolkit, ROS 2 Humble Hawksbill. Hardware requirements: NVIDIA RTX GPU, AMD Ryzen or Intel Core i7, 32GB RAM. Setup for Jetson Orin Nano edge device.",
    url: "/Hackathon_1/docs/en/module-0-setup/intro",
    module: "Module 0"
  },

  // Module 1 - Nervous System (ROS 2)
  {
    title: "Introduction to ROS 2 - The Robot's Nervous System",
    content: "ROS 2 (Robot Operating System 2) is the de facto industry standard for robotics. Used by Boston Dynamics, NASA, Waymo, Amazon. ROS 2 is a middleware framework providing inter-process communication, hardware abstraction, distributed computing. Learn about nodes, topics, services, publishers, subscribers, and the nervous system analogy.",
    url: "/Hackathon_1/docs/en/module-1-nervous-system/01-intro-to-ros2",
    module: "Module 1"
  },
  {
    title: "Nodes and Topics in ROS 2",
    content: "Learn ROS 2 nodes and topics architecture. Nodes are independent processes that communicate via topics. Publisher-subscriber pattern for message passing. Understanding the ROS 2 graph and communication patterns.",
    url: "/Hackathon_1/docs/en/module-1-nervous-system/02-nodes-and-topics",
    module: "Module 1"
  },
  {
    title: "URDF Modeling for Robots",
    content: "URDF (Unified Robot Description Format) for modeling robot geometry and kinematics. Define robot links, joints, and physical properties. Create robot models for simulation and visualization in RViz.",
    url: "/Hackathon_1/docs/en/module-1-nervous-system/03-urdf-modeling",
    module: "Module 1"
  },

  // Module 2 - Digital Twin
  {
    title: "The Digital Twin - Introduction to Simulation",
    content: "A digital twin is a virtual clone of your physical robot. Safe testing ground for algorithms before deploying to real hardware. Boston Dynamics tested Spot 100,000 times in simulation. Learn about sim-to-real transfer, Gazebo Fortress, Unity visualization. Save money and iterate rapidly.",
    url: "/Hackathon_1/docs/en/module-2-digital-twin/01-intro-digital-twin",
    module: "Module 2"
  },
  {
    title: "Gazebo Fortress Setup",
    content: "Setup Gazebo Fortress simulation environment. Photorealistic physics simulation for robots. Configure sensors, world files, and robot models. Integration with ROS 2 for testing algorithms.",
    url: "/Hackathon_1/docs/en/module-2-digital-twin/02-gazebo-fortress-setup",
    module: "Module 2"
  },
  {
    title: "Simulating Sensors in Gazebo",
    content: "Simulate cameras, LiDAR, IMU, and other sensors in Gazebo. Generate sensor data for testing perception algorithms. Configure sensor plugins and visualize data in RViz.",
    url: "/Hackathon_1/docs/en/module-2-digital-twin/03-simulating-sensors",
    module: "Module 2"
  },
  {
    title: "Unity Visualization for Robotics",
    content: "Use Unity game engine for photorealistic robot visualization. Real-time rendering, UI overlays, and augmented reality interfaces. Integration with ROS 2 for interactive visualization.",
    url: "/Hackathon_1/docs/en/module-2-digital-twin/04-unity-visualization",
    module: "Module 2"
  },

  // Module 3 - Robot Brain (Perception & Navigation)
  {
    title: "The Robot Brain - Perception and Navigation",
    content: "Build the brain of autonomous robots using NVIDIA Isaac Sim, Visual SLAM, and Nav2. Learn Visual SLAM with ORB-SLAM3 and RTAB-Map. Configure Nav2 for autonomous navigation, obstacle avoidance with costmaps, path planning with DWA and TEB planners. Deploy on Jetson Orin edge devices. Used by BMW, Volvo, Amazon Robotics.",
    url: "/Hackathon_1/docs/en/module-3-robot-brain/intro",
    module: "Module 3"
  },

  // Module 4 - The Mind (VLA Models)
  {
    title: "The Mind - Vision-Language-Action Models",
    content: "Cutting edge Physical AI with Large Language Models (LLMs), Vision-Language-Action (VLA) models, and Whisper speech recognition. Integrate GPT-4V and Claude 3 Opus for robot reasoning. Implement RT-2 and OpenVLA for manipulation. Build agentic workflows with tool-calling. Voice-controlled robots with natural language understanding. Used by Google DeepMind, Meta AI, OpenAI.",
    url: "/Hackathon_1/docs/en/module-4-the-mind/intro",
    module: "Module 4"
  },

  // Module 5 - Capstone Project
  {
    title: "Capstone Project - Autonomous Humanoid Robot",
    content: "Final challenge: Build a fully autonomous humanoid robot. Household Assistant Challenge: Navigate home environment, respond to voice commands, fetch objects from different rooms. Integrate voice recognition (Whisper), vision (GPT-4V), navigation (Nav2), and manipulation (VLA models). Complete end-to-end autonomous system.",
    url: "/Hackathon_1/docs/en/module-5-capstone/intro",
    module: "Module 5"
  }
];

/**
 * Search through documents using simple text matching
 */
export function searchDocuments(query: string, limit: number = 5): Array<{
  title: string;
  content: string;
  url: string;
  score: number;
}> {
  if (!query || query.trim().length === 0) {
    return [];
  }

  const queryLower = query.toLowerCase().trim();
  const queryWords = queryLower.split(/\s+/);

  // Score each document
  const scoredDocs = documentIndex.map(doc => {
    const titleLower = doc.title.toLowerCase();
    const contentLower = doc.content.toLowerCase();
    const moduleLower = doc.module.toLowerCase();

    let score = 0;

    // Exact title match (highest score)
    if (titleLower === queryLower) {
      score += 100;
    }

    // Title contains query
    if (titleLower.includes(queryLower)) {
      score += 50;
    }

    // Module match
    if (moduleLower.includes(queryLower)) {
      score += 30;
    }

    // Content contains full query
    if (contentLower.includes(queryLower)) {
      score += 20;
    }

    // Score individual words
    queryWords.forEach(word => {
      if (word.length < 3) return; // Skip very short words

      // Title contains word
      if (titleLower.includes(word)) {
        score += 10;
      }

      // Content contains word
      const wordCount = (contentLower.match(new RegExp(word, 'g')) || []).length;
      score += wordCount * 2;
    });

    return {
      ...doc,
      score: score
    };
  });

  // Filter out zero scores and sort by score
  const results = scoredDocs
    .filter(doc => doc.score > 0)
    .sort((a, b) => b.score - a.score)
    .slice(0, limit);

  // Normalize scores to 0-1 range
  if (results.length > 0) {
    const maxScore = results[0].score;
    return results.map(r => ({
      ...r,
      score: r.score / maxScore
    }));
  }

  return [];
}
