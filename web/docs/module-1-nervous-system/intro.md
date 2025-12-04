# Module 1: The Nervous System - ROS 2 Fundamentals

## Overview

Welcome to the first core module! Just like biological organisms have a nervous system to transmit signals between different body parts, robots use ROS 2 (Robot Operating System 2) as their communication backbone. This module teaches you how to build the "nervous system" of intelligent robots.

## Learning Objectives

By the end of this module, you will be able to:
- 🧠 Understand ROS 2 architecture (Nodes, Topics, Services, Actions)
- 🐍 Write Python ROS 2 nodes using `rclpy`
- 📡 Implement publisher-subscriber communication patterns
- 🔧 Create custom message types and service definitions
- 🤖 Model robots using URDF (Unified Robot Description Format)
- 🚀 Launch multi-node systems using launch files

## Why This Matters

ROS 2 is the industry standard for robot software development. Used by:
- **Boston Dynamics** (Spot SDK)
- **NASA** (Mars rovers)
- **Waymo** (Autonomous vehicles)
- **Amazon Robotics** (Warehouse automation)

Mastering ROS 2 is your gateway to professional robotics engineering.

## Prerequisites

- Module 0 completed (Ubuntu + ROS 2 Humble installed)
- Python basics (functions, classes, decorators)
- Command line proficiency

## Module Structure

### Core Concepts
1. [ROS 2 Architecture](./ros2-architecture.md) - Nodes, Topics, Services, Actions
2. [Creating Your First Node](./first-node.md) - Hello World in `rclpy`
3. [Publisher-Subscriber Pattern](./pubsub.md) - Sensor data simulation
4. [Services and Clients](./services.md) - Request-response communication
5. [URDF Basics](./urdf-modeling.md) - Robot description files

### Hands-On Tutorials
- **Tutorial 1**: Build a velocity controller node
- **Tutorial 2**: Create a sensor fusion system
- **Tutorial 3**: Model a mobile robot in URDF

### Exercises
- ✏️ Exercise 1: Implement a temperature monitoring system
- ✏️ Exercise 2: Create a multi-robot communication network
- ✏️ Exercise 3: Design a custom robot manipulator in URDF

### Assessment
- 📝 Quiz: 5 questions on ROS 2 concepts
- 💻 Coding Challenge: Build a autonomous navigation decision node

## Estimated Duration

**2 weeks** (10-15 hours total)

## Common Errors and Debugging

Throughout this module, you'll find "Common Errors" sections highlighting typical mistakes:
- ❌ `ImportError: cannot import name 'Node'` → ROS 2 sourcing issue
- ❌ Topic name mismatch → Namespace problems
- ❌ URDF parsing errors → XML syntax mistakes

## Hardware Used

- **Workstation**: NVIDIA RTX 4070 Ti for RViz visualization
- **Test Platform**: Turtlebot3 simulation (no physical robot needed yet)

## Next Steps

Ready to build your first ROS 2 node? Start with [ROS 2 Architecture](./ros2-architecture.md) →

---

**Pro Tip**: Keep the [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/) open as a reference while working through this module.
