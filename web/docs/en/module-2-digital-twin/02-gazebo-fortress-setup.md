---
id: gazebo-fortress-setup
title: Physics with Gazebo Fortress
sidebar_label: Gazebo Fortress Setup
sidebar_position: 2
description: Learn rigid body dynamics, install Gazebo Fortress, and understand URDF vs SDF formats
keywords: [gazebo, fortress, physics, urdf, sdf, rigid body dynamics, simulation]
---

# Physics with Gazebo Fortress

## Why Gazebo?

**Gazebo** is the industry-standard physics simulator for ROS 2. Unlike game engines (Unity, Unreal) that prioritize visual realism, Gazebo focuses on **physics accuracy** — critical for testing control algorithms, navigation stacks, and manipulation tasks.

**Gazebo Fortress** (formerly Ignition Gazebo) is the modern version that integrates natively with ROS 2 Humble via the `ros_gz` bridge.

---

## Rigid Body Dynamics: The Foundation

Before simulating robots, you need to understand the physics model Gazebo uses.

### What is a Rigid Body?

A **rigid body** is an object that **doesn't deform** — it only translates (moves) and rotates. This is a simplification (real objects bend and compress), but it's computationally efficient and accurate enough for most robotics applications.

### Key Physics Parameters

| Parameter | Definition | Example |
|-----------|------------|---------|
| **Mass** | Total weight in kilograms | Mobile robot base: 15 kg |
| **Inertia** | Resistance to rotation (3x3 tensor) | Higher inertia = harder to spin |
| **Gravity** | Force pulling objects down | Earth: -9.81 m/s² in Z-axis |
| **Friction** | Surface interaction coefficients | Rubber on concrete: µ = 0.8 |
| **Damping** | Energy dissipation (simulates air resistance) | Linear damping: 0.1, Angular damping: 0.05 |

### Why Accurate Physics Matters

- **PID Control Tuning**: Your controller gains depend on accurate inertia and damping
- **Grasping**: Contact forces must be realistic for manipulation tasks
- **Navigation**: Wheel slip and terrain interaction affect path planning

---

## Installation

### Install Gazebo Fortress and ROS 2 Bridge

```bash
# Update package list
sudo apt update

# Install Gazebo Fortress with ROS 2 Humble integration
sudo apt install ros-humble-ros-gz

# This installs:
# - Gazebo Fortress (gz-sim7)
# - ros_gz_bridge (ROS 2 ↔ Gazebo communication)
# - ros_gz_sim (Spawn robots, control simulation)
# - ros_gz_image (Camera/depth sensor integration)
```

### Verify Installation

```bash
# Check Gazebo version (expect Fortress 7.x)
gz sim --version

# Check ROS 2 packages
ros2 pkg list | grep ros_gz
# Should show: ros_gz_bridge, ros_gz_sim, ros_gz_image
```

### Test Empty World

```bash
# Launch Gazebo with empty world
gz sim empty.sdf
```

**Expected**: Gazebo GUI opens with a gray void. You can rotate the camera with mouse drag.

---

## URDF vs SDF: Which Format to Use?

### Comparison Table

| Feature | URDF (ROS Standard) | SDF (Gazebo Standard) |
|---------|---------------------|----------------------|
| **Purpose** | Robot descriptions only | Worlds + robots + sensors |
| **Scope** | Single model per file | Multiple models per file |
| **Physics** | Basic (mass, inertia) | Advanced (friction, damping, surface properties) |
| **Sensors** | Limited (requires `<gazebo>` tags) | Native sensor support with plugins |
| **Lighting** | Not supported | Full lighting control |
| **Worlds** | Not supported | ✅ Ground planes, sky, obstacles |

### When to Use Each

- **Use URDF** if:
  - You're defining a robot that will be used in ROS 2 (already familiar from Module 1)
  - You want to use `xacro` macros for modularity
  - You're working with existing ROS packages that expect URDF

- **Use SDF** if:
  - You're creating simulation **worlds** (ground plane, obstacles, lighting)
  - You need advanced physics (complex collisions, custom surface properties)
  - You're using Gazebo-specific features (multiple robots in one file)

**Best Practice**: Use URDF for robots (students already know it), SDF for worlds. Gazebo can load URDF files directly via `ros_gz_sim`.

---

## Creating Your First Gazebo World

Let's create a simple world with a ground plane and gravity.

### Create `test_world.sdf`

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="test_world">

    <!-- Physics Engine: ODE with 1ms time step -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size> <!-- 1ms for stability -->
      <real_time_factor>1.0</real_time_factor> <!-- Run at real-time speed -->
    </physics>

    <!-- Gravity: Standard Earth gravity -->
    <gravity>0 0 -9.81</gravity>

    <!-- Lighting: Directional sun -->
    <light name="sun" type="directional">
      <pose>5 5 10 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1.0</diffuse>
      <direction>-0.5 -0.5 -1.0</direction>
      <cast_shadows>true</cast_shadows>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static> <!-- Doesn't move, infinite mass -->
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal></plane> <!-- Flat in XY plane -->
          </geometry>
          <surface>
            <friction>
              <ode><mu>0.8</mu><mu2>0.8</mu2></ode> <!-- Rubber-like friction -->
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient> <!-- Gray color -->
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

### Launch the World

```bash
gz sim test_world.sdf
```

**What you see**: A gray ground plane with realistic lighting and shadows. Try clicking the "Play" button (▶️) in the bottom-left to start physics simulation.

---

## Understanding the Physics Engine

### Time Step: Why 1ms?

The `<max_step_size>0.001</max_step_size>` means Gazebo calculates physics every **1 millisecond**.

- **Smaller time step (0.0001s)**: More accurate but **slower** simulation
- **Larger time step (0.01s)**: Faster but objects may **tunnel through** each other

**1ms is the sweet spot** for most robotics applications (stable and fast).

### Gravity: Vector Format

```xml
<gravity>0 0 -9.81</gravity>
```

- **X-axis**: 0 (no sideways gravity)
- **Y-axis**: 0 (no forward/backward gravity)
- **Z-axis**: -9.81 m/s² (downward pull)

If you were simulating on the Moon, you'd use `-1.62` instead of `-9.81`.

---

## Converting URDF to SDF (Optional)

If you have a URDF file from Module 1 and want to convert it to SDF:

```bash
# Convert URDF to SDF
gz sdf -p my_robot.urdf > my_robot.sdf
```

**However**, Gazebo Fortress can load URDF files directly, so this is usually unnecessary. Just use:

```bash
ros2 run ros_gz_sim create -file my_robot.urdf -name my_robot
```

---

## Troubleshooting

### `gz sim` command not found

**Fix**: Add Gazebo to your PATH:

```bash
echo 'export PATH=$PATH:/opt/ros/humble/bin' >> ~/.bashrc
source ~/.bashrc
```

### Blank screen when launching Gazebo

**Fix**: Check OpenGL support:

```bash
glxinfo | grep "OpenGL version"
# Should show OpenGL 3.3 or higher
```

If OpenGL is missing, you may need to install graphics drivers (see Module 0).

### GPU errors

**Fix**: Verify NVIDIA drivers:

```bash
nvidia-smi
# Should show your RTX GPU and driver 525+
```

---

## Key Takeaways

✅ **Gazebo Fortress** is the modern ROS 2 simulator (formerly Ignition)

✅ **Rigid body dynamics** model: Mass, inertia, friction, damping, gravity

✅ **URDF** for robots (students know it), **SDF** for worlds (more features)

✅ **Install command**: `sudo apt install ros-humble-ros-gz`

✅ **1ms time step** is optimal for stability and speed

---

**Next**: Learn how to add sensors (LiDAR, cameras) to your robot in [**Adding Eyes & Ears (Sensors)**](./03-simulating-sensors.md)! 🤖
