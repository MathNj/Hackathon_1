# Module 2: The Digital Twin - Simulation and Sensors

## Overview

Before deploying to physical robots, we build **digital twins** — virtual replicas that let us test algorithms safely in simulation. This module covers Gazebo Classic, Gazebo Sim (Ignition), and Unity for photorealistic environments, plus sensor simulation (LiDAR, cameras, IMUs).

## Learning Objectives

By the end of this module, you will be able to:
- 🌍 Set up Gazebo simulation environments
- 📷 Simulate cameras, LiDAR, and depth sensors
- 🎮 Integrate Unity with ROS 2 for photorealistic rendering
- 🔬 Implement sensor fusion algorithms (camera + LiDAR)
- 🏗️ Build custom simulation worlds for testing
- 📊 Visualize sensor data in RViz and Plotjuggler

## Why Simulation Matters

**Real-world testing is expensive and risky:**
- 💰 Breaking a $50,000 robot during development
- ⏱️ Slow iteration cycles (deploy → test → debug)
- 🚫 Dangerous scenarios (edge cases, collisions)

**Simulation enables:**
- ✅ Infinite resets and rapid iteration
- ✅ Edge case testing (sensor failures, extreme weather)
- ✅ Parallel testing (run 100 scenarios simultaneously)

## Prerequisites

- Module 1 completed (ROS 2 fundamentals)
- Basic 3D geometry understanding
- Familiarity with coordinate systems (TF2)

## Module Structure

### Core Concepts
1. [Gazebo Architecture](./gazebo-overview.md) - Physics engines, plugins, sensors
2. [Simulating Sensors](./sensor-simulation.md) - LiDAR, RGB-D, IMU, GPS
3. [Unity Integration](./unity-ros2.md) - ROS-TCP-Connector setup
4. [Sensor Fusion](./sensor-fusion.md) - Kalman filters, data synchronization
5. [Custom World Building](./world-creation.md) - SDF files, model import

### Hands-On Tutorials
- **Tutorial 1**: Spawn a TurtleBot3 in Gazebo with LiDAR
- **Tutorial 2**: Create a Unity environment with dynamic lighting
- **Tutorial 3**: Implement a sensor fusion node (camera + IMU)

### Exercises
- ✏️ Exercise 1: Build a warehouse simulation with obstacles
- ✏️ Exercise 2: Calibrate a simulated stereo camera
- ✏️ Exercise 3: Create a photorealistic outdoor environment in Unity

### Assessment
- 📝 Quiz: Sensor characteristics and simulation parameters
- 💻 Coding Challenge: Object detection in simulated environments

## Estimated Duration

**3 weeks** (15-20 hours total)

## Hardware Requirements

- **GPU**: RTX 4070 Ti for real-time ray tracing in Unity
- **RAM**: 32GB (Gazebo + Unity can be memory-intensive)
- **Storage**: 50GB free space for Unity assets

## Tools Introduced

- **Gazebo Sim** (Ignition Fortress)
- **Unity 2022 LTS** with ROS-TCP-Connector
- **RViz2** for sensor data visualization
- **Plotjuggler** for time-series analysis

## Common Errors

- ❌ `Resource not found: [robot_description]` → URDF not loaded in launch file
- ❌ Unity-ROS connection timeout → Firewall blocking TCP port 10000
- ❌ Gazebo physics instability → Timestep too large or collision geometry issues

## Next Steps

Start with [Gazebo Architecture](./gazebo-overview.md) to understand simulation fundamentals →

---

**Pro Tip**: Use `ros2 bag record` to capture sensor data from simulation for offline algorithm testing.
