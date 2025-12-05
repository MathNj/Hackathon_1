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
  // Module 0 - Setup & Environment
  {
    title: "Introduction to Physical AI",
    content: "Module 0: Setup & Environment. Welcome to the Physical AI Textbook Platform. This platform offers an interactive learning experience for robotics and embodied AI. Learn about ROS 2, Docker, development tools, and setting up your environment for robotics development.",
    url: "/Hackathon_1/docs/en/module-0-setup/intro",
    module: "Module 0"
  },
  {
    title: "Development Environment Setup",
    content: "Setting up your development environment. Install ROS 2 Humble, Docker, VS Code, and essential tools for robotics development. Configure your workspace and verify installations.",
    url: "/Hackathon_1/docs/en/module-0-setup/development-setup",
    module: "Module 0"
  },
  {
    title: "ROS 2 Basics",
    content: "Introduction to ROS 2 (Robot Operating System). Learn about nodes, topics, services, actions, and the ROS 2 communication architecture. Understand publishers, subscribers, and message passing.",
    url: "/Hackathon_1/docs/en/module-0-setup/ros2-basics",
    module: "Module 0"
  },

  // Module 1 - Robot Eyes (Computer Vision)
  {
    title: "Computer Vision Introduction",
    content: "Module 1: Robot Eyes - Computer Vision. Learn how robots perceive the world through cameras and sensors. Topics include image processing, object detection, semantic segmentation, and depth estimation using neural networks.",
    url: "/Hackathon_1/docs/en/module-1-robot-eyes/intro",
    module: "Module 1"
  },
  {
    title: "Image Processing Fundamentals",
    content: "Fundamentals of image processing for robotics. Learn about image filtering, edge detection, color spaces, transformations, and preprocessing techniques for computer vision tasks.",
    url: "/Hackathon_1/docs/en/module-1-robot-eyes/image-processing",
    module: "Module 1"
  },
  {
    title: "Object Detection with YOLO",
    content: "Object detection using YOLO (You Only Look Once). Real-time object detection for robotics applications. Learn to detect and classify objects in images and video streams.",
    url: "/Hackathon_1/docs/en/module-1-robot-eyes/object-detection",
    module: "Module 1"
  },
  {
    title: "Semantic Segmentation",
    content: "Semantic segmentation for scene understanding. Pixel-level classification to identify different regions and objects in images. Applications in navigation and manipulation.",
    url: "/Hackathon_1/docs/en/module-1-robot-eyes/segmentation",
    module: "Module 1"
  },

  // Module 2 - Robot Hands (Manipulation)
  {
    title: "Robot Manipulation Introduction",
    content: "Module 2: Robot Hands - Manipulation and Grasping. Learn how robots interact with objects through manipulation. Topics include kinematics, inverse kinematics, motion planning, and grasp planning.",
    url: "/Hackathon_1/docs/en/module-2-robot-hands/intro",
    module: "Module 2"
  },
  {
    title: "Forward and Inverse Kinematics",
    content: "Robot kinematics for manipulation. Forward kinematics to compute end-effector position from joint angles. Inverse kinematics to compute joint angles for desired position.",
    url: "/Hackathon_1/docs/en/module-2-robot-hands/kinematics",
    module: "Module 2"
  },
  {
    title: "Motion Planning",
    content: "Motion planning for robot arms. Path planning algorithms including RRT, PRM, and trajectory optimization. Collision avoidance and smooth motion generation.",
    url: "/Hackathon_1/docs/en/module-2-robot-hands/motion-planning",
    module: "Module 2"
  },
  {
    title: "Grasp Planning",
    content: "Grasp planning and execution. Learn to compute stable grasps for different objects. Force closure, contact points, and grasp quality metrics.",
    url: "/Hackathon_1/docs/en/module-2-robot-hands/grasping",
    module: "Module 2"
  },

  // Module 3 - Robot Brain (Navigation & Planning)
  {
    title: "Robot Navigation Introduction",
    content: "Module 3: Robot Brain - Navigation and Path Planning. Learn how robots navigate and plan paths in their environment. Topics include SLAM, localization, path planning, and obstacle avoidance.",
    url: "/Hackathon_1/docs/en/module-3-robot-brain/intro",
    module: "Module 3"
  },
  {
    title: "SLAM - Simultaneous Localization and Mapping",
    content: "SLAM for robot navigation. Build maps while localizing the robot. Techniques include particle filters, EKF-SLAM, and graph-based SLAM.",
    url: "/Hackathon_1/docs/en/module-3-robot-brain/slam",
    module: "Module 3"
  },
  {
    title: "Path Planning Algorithms",
    content: "Path planning for mobile robots. A* algorithm, Dijkstra, RRT, and other planning methods. Global and local planning strategies.",
    url: "/Hackathon_1/docs/en/module-3-robot-brain/path-planning",
    module: "Module 3"
  },
  {
    title: "Obstacle Avoidance",
    content: "Dynamic obstacle avoidance for safe navigation. Reactive methods, potential fields, and dynamic window approach for real-time collision avoidance.",
    url: "/Hackathon_1/docs/en/module-3-robot-brain/obstacle-avoidance",
    module: "Module 3"
  },

  // Module 4 - The Mind (Deep Learning & AI)
  {
    title: "Deep Learning for Robotics",
    content: "Module 4: The Mind - Deep Learning and AI. Learn how neural networks power intelligent robot behavior. Topics include reinforcement learning, imitation learning, and end-to-end learning.",
    url: "/Hackathon_1/docs/en/module-4-the-mind/intro",
    module: "Module 4"
  },
  {
    title: "Reinforcement Learning",
    content: "Reinforcement learning for robot control. Q-learning, policy gradients, actor-critic methods, and deep RL. Train robots to learn behaviors through trial and error.",
    url: "/Hackathon_1/docs/en/module-4-the-mind/reinforcement-learning",
    module: "Module 4"
  },
  {
    title: "Imitation Learning",
    content: "Imitation learning from demonstrations. Behavioral cloning, inverse reinforcement learning, and learning from expert trajectories.",
    url: "/Hackathon_1/docs/en/module-4-the-mind/imitation-learning",
    module: "Module 4"
  },
  {
    title: "End-to-End Learning",
    content: "End-to-end learning for robotics. Neural networks that map sensors directly to actions. Vision-based control and learned policies.",
    url: "/Hackathon_1/docs/en/module-4-the-mind/end-to-end",
    module: "Module 4"
  },

  // Module 5 - Capstone Project
  {
    title: "Capstone Project Overview",
    content: "Module 5: Capstone Project - Integrating Everything. Build a complete robot system integrating vision, manipulation, navigation, and AI. Real-world robotics application.",
    url: "/Hackathon_1/docs/en/module-5-capstone/intro",
    module: "Module 5"
  },
  {
    title: "Project Requirements",
    content: "Capstone project requirements and guidelines. System architecture, integration strategies, testing, and deployment of your robot system.",
    url: "/Hackathon_1/docs/en/module-5-capstone/requirements",
    module: "Module 5"
  },
  {
    title: "Final Exam",
    content: "Final exam to test your knowledge. 20 multiple-choice questions covering all modules. Topics include computer vision, manipulation, navigation, and deep learning for robotics.",
    url: "/Hackathon_1/docs/en/module-5-capstone/exam",
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
