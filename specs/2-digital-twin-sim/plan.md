# Implementation Plan: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature ID**: 2-digital-twin-sim
**Branch**: `2-digital-twin-sim`
**Status**: Planning
**Created**: 2025-12-04
**Last Updated**: 2025-12-04
**Related**: [spec.md](./spec.md)

---

## Executive Summary

**Objective**: Create 4 comprehensive educational markdown files teaching digital twin concepts, Gazebo Fortress simulation, sensor simulation, and Unity visualization for ROS 2 Humble on Ubuntu 22.04.

**Deliverables**:
- File 1: Digital Twin concept and hardware requirements
- File 2: Gazebo Fortress setup and world building
- File 3: Sensor simulation (LiDAR, depth camera, IMU)
- File 4: Unity visualization with ROS-TCP-Connector

**Total Content**: ~12,000-14,000 words
**Student Time**: 6-8 hours (2.5 hours reading, 4-5.5 hours hands-on)

---

## Architecture Decisions

### Decision 1: Gazebo Version
**Choice**: Gazebo Fortress (gz-sim7), NOT Gazebo Classic (gazebo11)

**Rationale**:
- ROS 2 Humble officially supports Gazebo Fortress via `ros_gz` bridge
- Gazebo Classic is deprecated (last release 2025)
- Fortress aligns with "Future of Work" industry direction
- Better GPU ray tracing support for sensor simulation

**Implementation Impact**:
- All commands use `gz sim` (not `gazebo`)
- Install package: `ros-humble-ros-gz` (not `ros-humble-gazebo-ros-pkgs`)
- Plugin names: `gz_ros2_control`, `gz_ros2_camera` (new naming convention)

---

### Decision 2: Unity Integration Method
**Choice**: ROS-TCP-Connector (Unity Robotics Hub)

**Rationale**:
- Stable and proven with ROS 2 Humble
- Extensive documentation and community support
- Simpler setup than Unity 6 Robotics Package (experimental)
- Works on Ubuntu 22.04 without version conflicts

**Implementation Impact**:
- File 4 uses Unity Robotics Hub GitHub repo
- TCP socket bridge on port 10000 (default)
- Unity version: 2021.3 LTS (proven stable)

---

### Decision 3: CPU Fallback Strategy
**Choice**: Provide "Low Poly Mode" guidance for CPU-only machines

**Rationale**:
- Not all students may have RTX GPU initially
- Allow learning of concepts (world building, SDF) on CPU
- Clearly warn when GPU is required (sensor simulation)

**Implementation Impact**:
- File 1: Add GPU requirement warning callout
- File 2: Mark as "CPU-compatible" (world building works on CPU)
- File 3: Add "⚠️ NVIDIA RTX GPU Required" banner at top
- Provide reduced sensor resolution settings for older GPUs

---

### Decision 4: URDF vs SDF Workflow
**Choice**: Start with URDF, provide conversion script, teach SDF for worlds only

**Rationale**:
- Students already know URDF from Module 1
- Gazebo can load URDF directly via `ros_gz_sim`
- Manual SDF writing only needed for world files
- Conversion handles robot models automatically

**Implementation Impact**:
- File 2: Teach SDF syntax for world files only
- File 3: Sensors added to URDF (Gazebo converts internally)
- Provide `urdf_to_sdf.py` script for advanced users (optional)

---

## File Structure and Content Architecture

### Directory Layout

```
web/docs/en/module-2-digital-twin/
├── intro.md (existing overview page)
├── 01-intro-digital-twin.md [NEW]
├── 02-gazebo-fortress-setup.md [NEW]
├── 03-simulating-sensors.md [NEW]
├── 04-unity-visualization.md [NEW]
└── assets/
    ├── digital-twin-workflow.png (Mermaid diagram export)
    ├── gazebo-fortress-screenshot.png (empty world with robot)
    ├── lidar-pointcloud-rviz.png (RViz2 visualization)
    └── unity-ros-setup.png (Unity editor with ROS connection)
```

---

## File 1: Digital Twin Concept (`01-intro-digital-twin.md`)

### Purpose
Introduce digital twin concept, explain sim-to-real pipeline, contrast Gazebo vs Unity, and establish hardware requirements.

### Learning Objectives
- Define "digital twin" and "sim-to-real gap"
- Understand when to use simulation vs physical testing
- Distinguish Gazebo (physics) from Unity (visuals)
- Recognize GPU requirements for sensor simulation

### Content Outline (2,500-3,000 words)

#### Section 1: What is a Digital Twin? (500 words)
- **Definition**: Virtual replica synchronized with physical counterpart
- **Analogy**: "Dress rehearsal before opening night" / "Flight simulator before piloting"
- **Workflow Diagram**:
  ```mermaid
  graph LR
      A[Design CAD/URDF] --> B[Simulate in Gazebo]
      B --> C{Tests Pass?}
      C -->|No| D[Debug & Refine]
      D --> B
      C -->|Yes| E[Physical Prototype]
      E --> F[Validate Hardware]
      F --> G{Matches Sim?}
      G -->|No| H[Update Digital Twin]
      H --> B
      G -->|Yes| I[Deploy to Production]
  ```
- **Real-World Examples**:
  - Boston Dynamics: Spot tested 100,000+ times in simulation
  - NASA: Mars rovers validated in digital twin environments
  - Waymo: Autonomous vehicles tested billions of miles in simulation

#### Section 2: The Sim-to-Real Gap (400 words)
- **Definition**: Discrepancies between simulated and real-world behavior
- **Common Gaps**:
  - **Physics**: Friction coefficients, material properties, air resistance
  - **Sensors**: Noise, latency, occlusions not perfectly modeled
  - **Actuation**: Motor response times, gear backlash, wear
- **Mitigation Strategies**:
  - Calibrate simulation parameters from real-world measurements
  - Add noise models to sensors
  - Use domain randomization (covered in Module 3)
- **Why It Still Works**: 80-90% sim-to-real transfer is sufficient for most applications

#### Section 3: Gazebo vs Unity Decision Matrix (600 words)
- **Comparison Table**:

| Criterion | Gazebo Fortress | Unity |
|-----------|-----------------|-------|
| **Physics Accuracy** | ⭐⭐⭐⭐⭐ (ODE/Bullet engines, validated for robotics) | ⭐⭐⭐ (PhysX, optimized for games) |
| **Visual Realism** | ⭐⭐ (Basic rendering, functional) | ⭐⭐⭐⭐⭐ (AAA-game quality, photorealistic) |
| **ROS 2 Integration** | ⭐⭐⭐⭐⭐ (Native via ros_gz bridge) | ⭐⭐⭐ (TCP socket via ROS-TCP-Connector) |
| **Sensor Simulation** | ⭐⭐⭐⭐⭐ (GPU ray tracing for LiDAR/depth) | ⭐⭐⭐⭐ (GPU shaders, good but less accurate) |
| **Learning Curve** | ⭐⭐⭐ (Moderate, ROS-focused) | ⭐⭐ (Steeper, game engine knowledge needed) |
| **Use Cases** | Control algorithms, navigation, manipulation | Perception, CV training, synthetic data generation |
| **Cost** | Free, open-source | Free (Personal), paid (Pro $2,040/year) |

- **Decision Guide**:
  - **Use Gazebo if**: Testing control algorithms, validating navigation stacks, debugging sensor fusion, learning ROS 2 simulation
  - **Use Unity if**: Generating synthetic training data for computer vision, domain randomization for perception models, photorealistic visualization for demos
  - **Use Both**: Gazebo for control validation → Unity for perception training (common in industry)

#### Section 4: Hardware Requirements (700 words)
- **⚠️ CRITICAL WARNING** (Callout Box):
  ```
  ⚠️ NVIDIA RTX GPU REQUIRED FOR SENSOR SIMULATION

  Gazebo Fortress uses GPU ray tracing for realistic LiDAR and depth camera simulation.
  Without an RTX GPU, sensor simulation will run at 1-2 FPS (unusable).

  Minimum: NVIDIA RTX 4060 (8GB VRAM) - $300
  Recommended: NVIDIA RTX 4070 Ti (12GB VRAM) - $800

  CPU-only machines can still complete File 2 (world building) but NOT File 3 (sensors).
  ```

- **Why RTX is Required**:
  - **Ray Tracing Cores (RT Cores)**: Hardware acceleration for light/laser ray simulation
  - **Performance**: 10-100x faster than CPU ray tracing
  - **Sensor Fidelity**: Realistic depth maps, accurate LiDAR point clouds, proper occlusions
  - **Comparison**:
    - **RTX 4070 Ti**: 300 FPS LiDAR simulation (usable)
    - **GTX 1080 (no RT cores)**: 30 FPS (marginal)
    - **CPU only**: 2 FPS (unusable)

- **System Requirements Summary**:
  ```yaml
  OS: Ubuntu 22.04 LTS
  RAM: 16 GB minimum (32 GB recommended)
  GPU: NVIDIA RTX 4060 or better
  Storage: 50 GB free (Gazebo worlds, meshes, Unity project)
  Software: ROS 2 Humble, Gazebo Fortress, NVIDIA Driver 525+
  ```

- **Budget Alternatives** (for students):
  - **Used RTX 3060**: ~$200 (adequate for learning)
  - **Cloud GPUs**: AWS EC2 g5.xlarge (~$1/hour with RTX A10G)
  - **University Labs**: Many robotics labs have RTX workstations

#### Section 5: What's Next (300 words)
- **Module 2 Roadmap**:
  - **File 2**: Install Gazebo Fortress, create your first world with physics
  - **File 3**: Add LiDAR and depth camera sensors to robots
  - **File 4**: Visualize robots in Unity with photorealistic rendering
- **Prerequisites Check**:
  - Module 0 completed (Ubuntu, ROS 2 Humble installed)
  - Module 1 completed (URDF, nodes, topics understood)
  - GPU drivers verified: Run `nvidia-smi` (expect driver 525+)

### Code Examples
None (conceptual file)

### Diagrams
1. **Digital Twin Workflow** (Mermaid flowchart)
2. **Sim-to-Real Gap Illustration** (before/after calibration)
3. **Gazebo vs Unity Decision Tree** (flowchart)

### Acceptance Criteria (from spec SC-003)
- Student can explain Gazebo vs Unity tradeoffs in under 3 minutes
- Student recognizes GPU requirement for sensor simulation

---

## File 2: Gazebo Fortress Setup (`02-gazebo-fortress-setup.md`)

### Purpose
Install Gazebo Fortress with ROS 2 Humble integration, teach SDF syntax for world files, demonstrate physics simulation.

### Learning Objectives
- Install `ros-humble-ros-gz` bridge package
- Understand SDF (Simulation Description Format) structure
- Create custom world with ground plane, obstacles, lighting
- Configure physics engine (gravity, time step, solver)
- Spawn robots and control via ROS 2 topics

### Content Outline (3,500-4,000 words)

#### Section 1: Installation and Verification (600 words)
- **Install ros_gz Bridge**:
  ```bash
  sudo apt update
  sudo apt install ros-humble-ros-gz
  ```
- **Verify Installation**:
  ```bash
  gz sim --version
  # Expected: Gazebo Sim, version 7.x.x

  ros2 pkg list | grep ros_gz
  # Expected: ros_gz_bridge, ros_gz_sim, ros_gz_image
  ```
- **Test Empty World**:
  ```bash
  gz sim empty.sdf
  # Should open GUI with empty gray void
  ```
- **Troubleshooting**:
  - If `gz sim` not found: Add `/opt/ros/humble/bin` to PATH
  - If GPU errors: Verify `nvidia-smi` shows driver 525+
  - If blank screen: Check OpenGL: `glxinfo | grep "OpenGL version"`

#### Section 2: SDF Syntax Deep Dive (800 words)
- **SDF vs URDF Comparison**:

| Feature | URDF | SDF |
|---------|------|-----|
| **Purpose** | Robot descriptions only | Worlds + robots + sensors |
| **Scope** | Single model per file | Multiple models per file |
| **Physics** | Basic (mass, inertia) | Advanced (friction, damping, surface properties) |
| **Sensors** | Limited (requires Gazebo tags) | Native sensor support |
| **Lighting** | Not supported | Full lighting control |
| **Worlds** | Not supported | ✅ Ground planes, sky, fog |

- **SDF File Structure**:
  ```xml
  <?xml version="1.0"?>
  <sdf version="1.9">
    <world name="my_world">
      <!-- Physics engine configuration -->
      <physics name="default_physics" type="ode">
        <max_step_size>0.001</max_step_size> <!-- 1ms time step -->
        <real_time_factor>1.0</real_time_factor> <!-- Real-time speed -->
      </physics>

      <!-- Lighting -->
      <light name="sun" type="directional">
        <pose>0 0 10 0 0 0</pose>
        <diffuse>1.0 1.0 1.0 1.0</diffuse>
        <direction>-0.5 0.5 -1.0</direction>
      </light>

      <!-- Ground plane -->
      <model name="ground_plane">
        <static>true</static>
        <link name="link">
          <collision name="collision">
            <geometry>
              <plane><normal>0 0 1</normal></plane>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <plane><normal>0 0 1</normal></plane>
            </geometry>
            <material>
              <ambient>0.8 0.8 0.8 1</ambient>
              <diffuse>0.8 0.8 0.8 1</diffuse>
            </material>
          </visual>
        </link>
      </model>
    </world>
  </sdf>
  ```

#### Section 3: Physics Engine Configuration (700 words)
- **Physics Parameters Explained**:
  - **Gravity**: Default `<gravity>0 0 -9.81</gravity>` (m/s² in Z-axis)
  - **Time Step**: `<max_step_size>0.001</max_step_size>` (1ms recommended for stability)
  - **Solver**: ODE (default), Bullet, DART, Simbody
  - **Iterations**: Higher = more accurate but slower

- **Rigid Body Dynamics Primer**:
  - **Mass**: Total weight in kg (affects inertia and gravity response)
  - **Inertia Tensor**: 3x3 matrix describing rotational resistance
    - For cylinder: `Ixx = 1/12 * m * (3r² + h²)`
    - For box: `Ixx = 1/12 * m * (h² + d²)`
  - **Friction**: Surface interaction coefficients
    - `mu`: Primary friction (0.0 = ice, 1.0 = rubber)
    - `mu2`: Secondary friction (perpendicular direction)
  - **Damping**: Energy dissipation (linear and angular)

- **When Physics Matters**:
  - **Control Algorithms**: PID tuning requires accurate dynamics
  - **Manipulation**: Grasping requires realistic contact forces
  - **Navigation**: Wheel slip, terrain interaction

#### Section 4: Building Your First World (1,000 words)
- **Tutorial: "Robot Arena" World**:

  Create `robot_arena.sdf`:
  ```xml
  <?xml version="1.0"?>
  <sdf version="1.9">
    <world name="robot_arena">
      <!-- Physics: 1ms time step, real-time -->
      <physics name="1ms" type="ode">
        <max_step_size>0.001</max_step_size>
        <real_time_factor>1.0</real_time_factor>
      </physics>

      <!-- Lighting: Sun from above-right -->
      <light name="sun" type="directional">
        <pose>5 5 10 0 0 0</pose>
        <diffuse>1.0 1.0 1.0 1.0</diffuse>
        <specular>0.5 0.5 0.5 1.0</specular>
        <direction>-0.5 -0.5 -1.0</direction>
        <cast_shadows>true</cast_shadows>
      </light>

      <!-- Ground plane with texture -->
      <model name="ground_plane">
        <static>true</static>
        <link name="link">
          <collision name="collision">
            <geometry>
              <plane><normal>0 0 1</normal><size>20 20</size></plane>
            </geometry>
            <surface>
              <friction>
                <ode><mu>0.8</mu><mu2>0.8</mu2></ode>
              </friction>
            </surface>
          </collision>
          <visual name="visual">
            <geometry>
              <plane><normal>0 0 1</normal><size>20 20</size></plane>
            </geometry>
            <material>
              <ambient>0.5 0.5 0.5 1</ambient>
              <diffuse>0.8 0.8 0.8 1</diffuse>
            </material>
          </visual>
        </link>
      </model>

      <!-- Obstacle 1: Red Box -->
      <model name="red_box">
        <pose>2 0 0.25 0 0 0</pose>
        <link name="link">
          <inertial>
            <mass>1.0</mass>
            <inertia>
              <ixx>0.0083</ixx><iyy>0.0083</iyy><izz>0.0083</izz>
              <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
            </inertia>
          </inertial>
          <collision name="collision">
            <geometry>
              <box><size>0.5 0.5 0.5</size></box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <box><size>0.5 0.5 0.5</size></box>
            </geometry>
            <material>
              <ambient>1.0 0.0 0.0 1</ambient>
              <diffuse>1.0 0.0 0.0 1</diffuse>
            </material>
          </visual>
        </link>
      </model>

      <!-- Obstacle 2: Blue Cylinder -->
      <model name="blue_cylinder">
        <pose>-2 2 0.5 0 0 0</pose>
        <link name="link">
          <inertial>
            <mass>2.0</mass>
            <inertia>
              <ixx>0.17</ixx><iyy>0.17</iyy><izz>0.125</izz>
              <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
            </inertia>
          </inertial>
          <collision name="collision">
            <geometry>
              <cylinder><radius>0.25</radius><length>1.0</length></cylinder>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <cylinder><radius>0.25</radius><length>1.0</length></cylinder>
            </geometry>
            <material>
              <ambient>0.0 0.0 1.0 1</ambient>
              <diffuse>0.0 0.5 1.0 1</diffuse>
            </material>
          </visual>
        </link>
      </model>
    </world>
  </sdf>
  ```

- **Launch the World**:
  ```bash
  gz sim robot_arena.sdf
  ```

- **Expected Result**:
  - Gray ground plane (20m x 20m)
  - Red box at (2, 0, 0.25)
  - Blue cylinder at (-2, 2, 0.5)
  - Directional sunlight casting shadows

#### Section 5: Spawning Robots (600 words)
- **Method 1: Include in SDF World File** (Static):
  ```xml
  <include>
    <uri>model://my_robot</uri>
    <pose>0 0 0.5 0 0 0</pose>
  </include>
  ```

- **Method 2: Spawn via ROS 2** (Dynamic):
  ```bash
  # Terminal 1: Launch Gazebo world
  gz sim robot_arena.sdf

  # Terminal 2: Spawn robot from URDF
  ros2 run ros_gz_sim create -file ~/robot.urdf -name my_robot -x 0 -y 0 -z 0.5
  ```

- **Using Module 1's simple_arm.urdf**:
  ```bash
  # Copy from Module 1
  cp ~/module1_files/simple_arm.urdf ~/robot_arena_demo.urdf

  # Spawn in Gazebo
  ros2 run ros_gz_sim create -file robot_arena_demo.urdf -name simple_arm -z 0.2
  ```

#### Section 6: Controlling Robots via ROS 2 (800 words)
- **Adding Differential Drive Plugin** (for mobile robots):

  Add to URDF:
  ```xml
  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(find my_robot)/config/controllers.yaml</parameters>
    </plugin>
  </gazebo>
  ```

- **Simple Velocity Control** (without plugins):
  ```bash
  # Publish twist commands
  ros2 topic pub /model/my_robot/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.5}, angular: {z: 0.0}}"
  ```

- **Verify Robot Movement**:
  ```bash
  # Check robot pose
  gz topic -e -t /model/my_robot/pose
  ```

### Code Examples
1. Complete `robot_arena.sdf` (80 lines)
2. Robot spawn commands
3. Velocity control ROS 2 topic publishing

### Diagrams
1. SDF file structure tree
2. Physics engine pipeline (timestep → collision → forces → integration)

### Acceptance Criteria (from spec SC-001)
- Student creates custom world in under 15 minutes
- 100% completion rate on world creation tutorial

---

## File 3: Simulating Sensors (`03-simulating-sensors.md`)

### Purpose
Add depth cameras, LiDAR, and IMU sensors to robots. Explain GPU ray tracing. Visualize sensor data in RViz2.

### Learning Objectives
- Understand Gazebo sensor simulation architecture
- Add depth camera (Intel RealSense D435i equivalent)
- Add LiDAR (Velodyne VLP-16 equivalent)
- Add IMU (Inertial Measurement Unit)
- Visualize all sensors in RViz2
- Write ROS 2 subscriber to process sensor data

### Content Outline (4,000-4,500 words)

#### Section 0: GPU Requirement Banner (100 words)
```
⚠️ NVIDIA RTX GPU REQUIRED FOR THIS TUTORIAL

This tutorial uses Gazebo's GPU ray tracing for realistic sensor simulation.
Without an NVIDIA RTX GPU (4060 or better), sensors will run at 1-2 FPS (unusable).

Verify your GPU: nvidia-smi (expect RTX 4060/4070/4080/4090)

If you don't have an RTX GPU, you can still read this tutorial to understand
sensor simulation concepts, but hands-on exercises will not be performant.

CPU Fallback: Reduce sensor resolution (see "Performance Tuning" section).
```

#### Section 1: Sensor Simulation Architecture (600 words)
- **How Gazebo Simulates Sensors**:
  ```
  1. Gazebo Physics Engine → Updates robot pose
  2. Sensor Plugin → Triggers at configured Hz (e.g., 10 Hz for LiDAR)
  3. GPU Ray Tracing → Shoots rays from sensor origin
  4. Collision Detection → Rays hit objects, return distances
  5. Data Processing → Convert to ROS 2 message (LaserScan, PointCloud2, Image)
  6. ROS 2 Bridge → Publish to topic (/scan, /camera/image_raw)
  ```

- **Why GPU Ray Tracing**:
  - **LiDAR**: Shoots 16-64 laser beams, 360° horizontal, 10-20 Hz
    - 64 beams × 1800 samples × 20 Hz = **2.3 million rays/second**
  - **Depth Camera**: 640×480 pixels, 30 Hz
    - 640 × 480 × 30 = **9.2 million rays/second**
  - **GPU Performance**: RTX 4070 Ti can trace 10+ billion rays/second
  - **CPU Performance**: 10-100 million rays/second (1000x slower)

#### Section 2: Depth Camera Simulation (1,000 words)
- **Intel RealSense D435i Equivalent**:

Add to URDF (after `</robot>` closing tag, inside `<gazebo>` block):
```xml
<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth_camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.51844</horizontal_fov> <!-- 87 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
      <depth_camera>
        <output>depths</output>
      </depth_camera>
    </camera>

    <plugin filename="libgazebo_ros_camera.so" name="depth_camera_controller">
      <ros>
        <namespace>/camera</namespace>
        <remapping>~/image_raw:=image_raw</remapping>
        <remapping>~/depth/image_raw:=depth/image_raw</remapping>
        <remapping>~/camera_info:=camera_info</remapping>
        <remapping>~/depth/camera_info:=depth/camera_info</remapping>
      </ros>
      <frame_name>camera_optical_frame</frame_name>
      <hack_baseline>0.07</hack_baseline> <!-- Stereo baseline -->
    </plugin>
  </sensor>
</gazebo>
```

- **Camera Link Definition** (add to main URDF):
```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.025 0.09 0.025"/>
    </geometry>
    <material name="camera_gray">
      <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.025 0.09 0.025"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.072"/>
    <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
</joint>

<!-- Optical frame (Z forward, Y down) -->
<link name="camera_optical_frame"/>
<joint name="camera_optical_joint" type="fixed">
  <parent link="camera_link"/>
  <child link="camera_optical_frame"/>
  <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
</joint>
```

- **Published Topics**:
  - `/camera/image_raw` - RGB image (sensor_msgs/Image)
  - `/camera/depth/image_raw` - Depth image (sensor_msgs/Image, 16-bit)
  - `/camera/camera_info` - Calibration parameters
  - `/camera/depth/camera_info` - Depth calibration

- **Visualize in RViz2**:
  ```bash
  # Terminal 1: Launch Gazebo with robot
  gz sim robot_arena.sdf

  # Terminal 2: Spawn robot with camera
  ros2 run ros_gz_sim create -file robot_with_camera.urdf -name cam_robot

  # Terminal 3: Launch RViz2
  rviz2
  ```

  In RViz2:
  1. Set Fixed Frame: `camera_optical_frame`
  2. Add → Image → Topic: `/camera/image_raw`
  3. Add → DepthCloud → Topic: `/camera/depth/image_raw`

#### Section 3: LiDAR Simulation (1,200 words)
- **Velodyne VLP-16 Equivalent (16 beams)**:

Add to URDF:
```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="gpu_lidar">
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>1800</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>
          <resolution>1</resolution>
          <min_angle>-0.2618</min_angle> <!-- -15 degrees -->
          <max_angle>0.2618</max_angle>  <!-- +15 degrees -->
        </vertical>
      </scan>
      <range>
        <min>0.1</min>
        <max>100.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>

    <plugin filename="libgazebo_ros_ray_sensor.so" name="lidar_controller">
      <ros>
        <namespace>/lidar</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

- **LiDAR Link Definition**:
```xml
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.07"/>
    </geometry>
    <material name="lidar_black">
      <color rgba="0.1 0.1 0.1 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.07"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.83"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
</joint>
```

- **Published Topic**:
  - `/lidar/scan` - LaserScan or PointCloud2

- **Visualize in RViz2**:
  1. Fixed Frame: `lidar_link`
  2. Add → LaserScan → Topic: `/lidar/scan`
  3. Adjust size/color in LaserScan display properties

- **Python Subscriber Example**:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.lidar_callback,
            10)

    def lidar_callback(self, msg):
        # Find closest obstacle
        min_distance = min(msg.ranges)
        min_index = msg.ranges.index(min_distance)
        angle = msg.angle_min + min_index * msg.angle_increment

        self.get_logger().info(
            f'Closest obstacle: {min_distance:.2f}m at {angle:.2f} radians'
        )

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Section 4: IMU Simulation (800 words)
- **Inertial Measurement Unit**:

Add to URDF:
```xml
<gazebo reference="imu_link">
  <sensor name="imu" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></x>
        <y><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></y>
        <z><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></z>
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></x>
        <y><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></y>
        <z><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></z>
      </linear_acceleration>
    </imu>

    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_controller">
      <ros>
        <namespace>/imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

- **IMU Link** (placed at robot center of mass):
```xml
<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>
```

- **Published Topic**:
  - `/imu/data` - sensor_msgs/Imu
    - `orientation` (quaternion): Roll, pitch, yaw
    - `angular_velocity` (rad/s): Rotation rates
    - `linear_acceleration` (m/s²): Including gravity

- **Python Subscriber**:
```python
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion

class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        self.subscription = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

    def imu_callback(self, msg):
        # Convert quaternion to Euler angles
        orientation_q = msg.orientation
        (roll, pitch, yaw) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

        self.get_logger().info(
            f'Orientation: Roll={roll:.2f}, Pitch={pitch:.2f}, Yaw={yaw:.2f}'
        )
```

#### Section 5: Performance Tuning (600 words)
- **GPU vs CPU Performance**:

| Sensor | RTX 4070 Ti (GPU) | CPU Only | Recommended Settings |
|--------|-------------------|----------|----------------------|
| Depth Camera (640×480, 30Hz) | 300 FPS | 2 FPS | GPU: Full res, CPU: 320×240, 10Hz |
| LiDAR (16 beams, 10Hz) | 200 FPS | 5 FPS | GPU: 16 beams, CPU: 8 beams |
| LiDAR (64 beams, 20Hz) | 60 FPS | <1 FPS | GPU only (not viable on CPU) |

- **CPU Fallback Settings**:
  ```xml
  <!-- Reduced depth camera for CPU -->
  <image><width>320</width><height>240</height></image>
  <update_rate>10</update_rate>

  <!-- Reduced LiDAR for CPU -->
  <vertical><samples>8</samples></vertical>
  <update_rate>5</update_rate>
  ```

- **Troubleshooting Low FPS**:
  1. Check GPU usage: `nvidia-smi` (should show Gazebo using GPU)
  2. Reduce sensor resolution
  3. Lower update rates (10 Hz instead of 30 Hz)
  4. Close other GPU applications (browsers with hardware acceleration)

#### Section 6: Exercise (300 words)
**Challenge**: Add a second LiDAR sensor to the back of the robot for 360° coverage.

**Hints**:
1. Create `rear_lidar_link` at `origin xyz="-0.15 0 0.3"`
2. Copy LiDAR sensor block, change namespace to `/rear_lidar`
3. Launch and verify both `/lidar/scan` and `/rear_lidar/scan` topics exist

<details>
<summary>Solution</summary>

Add rear LiDAR link and joint:
```xml
<link name="rear_lidar_link">
  <visual>
    <geometry><cylinder radius="0.05" length="0.07"/></geometry>
    <material name="lidar_black"><color rgba="0.1 0.1 0.1 1.0"/></material>
  </visual>
  <collision>
    <geometry><cylinder radius="0.05" length="0.07"/></geometry>
  </collision>
  <inertial>
    <mass value="0.83"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<joint name="rear_lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="rear_lidar_link"/>
  <origin xyz="-0.15 0 0.3" rpy="0 0 3.14159"/> <!-- 180 degrees -->
</joint>
```

Then add Gazebo sensor block with `<ros><namespace>/rear_lidar</namespace>`.
</details>

### Code Examples
1. Complete depth camera URDF (60 lines)
2. Complete LiDAR URDF (70 lines)
3. Complete IMU URDF (40 lines)
4. Python LiDAR subscriber (30 lines)
5. Python IMU subscriber (25 lines)

### Diagrams
1. Sensor simulation pipeline (ray tracing flowchart)
2. RViz2 screenshot with all sensors visualized

### Acceptance Criteria (from spec SC-002, SC-007)
- Student adds LiDAR with zero errors (100% success)
- Student modifies LiDAR parameters in under 10 minutes

---

## File 4: Unity Visualization (`04-unity-visualization.md`)

### Purpose
Introduce Unity as photorealistic visualization alternative, set up Unity Robotics Hub, connect to ROS 2 via ROS-TCP-Connector.

### Learning Objectives
- Understand Unity's role in robotics (visual fidelity, not physics accuracy)
- Install Unity 2021.3 LTS and Unity Robotics Hub
- Configure ROS-TCP-Connector
- Visualize ROS 2 robot in Unity with high-fidelity rendering
- Understand when to use Unity vs Gazebo

### Content Outline (2,500-3,000 words)

#### Section 1: Why Unity for Robotics? (500 words)
- **Unity's Strengths**:
  - **Photorealistic Rendering**: AAA-game quality graphics (PBR materials, global illumination)
  - **Domain Randomization**: Randomize lighting, textures, object poses for CV training
  - **Synthetic Data Generation**: Generate 10,000s of labeled images for perception models
  - **Cross-Platform**: Windows, macOS, Linux support

- **Unity's Weaknesses**:
  - **Physics Accuracy**: PhysX optimized for games, not robotics (softer contacts, less accurate)
  - **ROS 2 Integration**: Requires TCP bridge (not native like Gazebo)
  - **Learning Curve**: Game engine knowledge required

- **Industry Use Cases**:
  - **NVIDIA Isaac Sim**: Built on Omniverse, uses Unity-like editor
  - **Synthetic Data Pipelines**: Train YOLO on Unity-generated images
  - **Demos and Visualization**: Show investors photorealistic robot renders

#### Section 2: Installation (700 words)
- **Install Unity Hub**:
  ```bash
  wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHubSetup.AppImage
  chmod +x UnityHubSetup.AppImage
  ./UnityHubSetup.AppImage
  ```

- **Install Unity 2021.3 LTS** (via Unity Hub):
  - Open Unity Hub → Installs → Add
  - Select Unity 2021.3.x LTS (proven stable with ROS 2)
  - Include modules: Linux Build Support

- **Install Unity Robotics Hub**:
  - Open Unity Hub → New Project → 3D Core
  - Name: `ROS2_Unity_Demo`
  - Open project in Unity Editor

- **Add ROS-TCP-Connector Package**:
  - Window → Package Manager → + (top-left) → Add package from git URL
  - URL: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
  - Wait for import (may take 2-3 minutes)

#### Section 3: ROS-TCP-Connector Setup (800 words)
- **Start ROS-TCP-Endpoint** (on Ubuntu machine):
  ```bash
  # Install endpoint
  sudo apt install ros-humble-ros-tcp-endpoint

  # Launch endpoint (listens on port 10000)
  ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
  ```

- **Configure Unity Connection**:
  - Unity Editor → Robotics → ROS Settings
  - Protocol: ROS 2
  - ROS IP Address: `<Ubuntu_Machine_IP>` (e.g., `192.168.1.100`)
  - ROS Port: `10000`
  - Click "Connect" (should show "Connected" in green)

- **Test Connection**:
  ```bash
  # Terminal 1: Launch endpoint
  ros2 run ros_tcp_endpoint default_server_endpoint

  # Terminal 2: Echo Unity test topic
  ros2 topic echo /unity_test
  ```

  In Unity: Robotics → ROS Settings → Publish Test Message
  Should see message appear in terminal.

#### Section 4: Importing Robot Model (600 words)
- **Export URDF Meshes to Unity**:
  - Unity supports: `.fbx`, `.obj`, `.stl`
  - Convert `.dae` (Collada) to `.fbx` using Blender:
    ```bash
    blender --background --python convert_dae_to_fbx.py -- input.dae output.fbx
    ```

- **Import Robot**:
  - Assets → Import New Asset → Select `.fbx` file
  - Drag mesh into Scene Hierarchy
  - Add Rigidbody component (for physics)

- **Subscribe to ROS 2 /joint_states**:
  - Create C# script `JointStateSubscriber.cs`:
    ```csharp
    using UnityEngine;
    using Unity.Robotics.ROSTCPConnector;
    using RosMessageTypes.Sensor;

    public class JointStateSubscriber : MonoBehaviour
    {
        void Start()
        {
            ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>(
                "/joint_states", UpdateJointStates);
        }

        void UpdateJointStates(JointStateMsg jointState)
        {
            // Update Unity joint transforms based on ROS /joint_states
            for (int i = 0; i < jointState.position.Length; i++)
            {
                string jointName = jointState.name[i];
                float angle = (float)jointState.position[i];

                // Find corresponding Unity GameObject and rotate
                GameObject joint = GameObject.Find(jointName);
                if (joint != null)
                {
                    joint.transform.localRotation = Quaternion.Euler(0, angle * Mathf.Rad2Deg, 0);
                }
            }
        }
    }
    ```

#### Section 5: Comparison: Gazebo vs Unity (400 words)
- **When to Use Unity**:
  - Generating synthetic training data for computer vision
  - Photorealistic demos for stakeholders
  - Domain randomization experiments
  - Cross-platform development (Windows/macOS)

- **When to Use Gazebo**:
  - Control algorithm development
  - Navigation stack testing
  - Sensor fusion validation
  - Physics-accurate manipulation

- **Typical Workflow**:
  1. Develop control in Gazebo (accurate physics)
  2. Train perception in Unity (synthetic data)
  3. Validate on physical robot

#### Section 6: Next Steps (500 words)
- **Advanced Unity Topics** (not covered in this module):
  - Perception Camera (automated bounding box generation)
  - Randomizers (domain randomization API)
  - High Definition Render Pipeline (HDRP)
  - NVIDIA Isaac Sim (Omniverse-based, more advanced)

- **Resources**:
  - Unity Robotics Hub: https://github.com/Unity-Technologies/Unity-Robotics-Hub
  - ROS-TCP-Connector Docs: https://github.com/Unity-Technologies/ROS-TCP-Connector
  - Unity Learn: Robotics Tutorials

### Code Examples
1. Unity C# joint state subscriber (30 lines)
2. ROS-TCP-Endpoint launch command

### Diagrams
1. ROS-TCP-Connector architecture (Unity ↔ TCP ↔ ROS 2)
2. Unity screenshot with robot imported

### Acceptance Criteria
- Student understands Unity's role (awareness-level, no hands-on requirement)
- Student can explain Gazebo vs Unity tradeoffs

---

## Implementation Phases

### Phase 0: Research and Asset Preparation (1-2 days)
**Objective**: Gather all technical resources before writing.

**Tasks**:
1. Verify Gazebo Fortress commands on Ubuntu 22.04 + ROS 2 Humble
2. Test depth camera URDF plugin (ensure topics publish)
3. Test LiDAR URDF plugin (verify point cloud in RViz2)
4. Test IMU URDF plugin (verify orientation quaternion)
5. Create `robot_arena.sdf` world file and verify physics
6. Install Unity 2021.3 LTS and test ROS-TCP-Connector
7. Create Mermaid diagrams for File 1 (digital twin workflow)
8. Take screenshots: Gazebo world, RViz2 sensors, Unity import

**Checkpoint**: All code tested, all screenshots captured, ready to write.

---

### Phase 1: File 1 - Digital Twin Intro (1 day)
**Objective**: Write conceptual foundation (2,500-3,000 words).

**Tasks**:
1. Write Section 1: Digital Twin definition + workflow diagram
2. Write Section 2: Sim-to-real gap explanation
3. Write Section 3: Gazebo vs Unity comparison table
4. Write Section 4: GPU hardware requirements (NVIDIA RTX warning)
5. Write Section 5: Next steps roadmap
6. Add Mermaid diagrams (digital twin workflow, decision tree)
7. Markdown linting + link verification

**Checkpoint**: File 1 complete, conceptual foundation established.

---

### Phase 2: File 2 - Gazebo Setup (1.5 days)
**Objective**: Write Gazebo Fortress tutorial (3,500-4,000 words).

**Tasks**:
1. Write Section 1: Installation and verification
2. Write Section 2: SDF syntax deep dive
3. Write Section 3: Physics engine configuration
4. Write Section 4: Robot Arena world tutorial (complete SDF)
5. Write Section 5: Spawning robots via ROS 2
6. Write Section 6: Controlling robots (velocity commands)
7. Test all commands on clean Ubuntu 22.04 VM
8. Add code examples and expected outputs
9. Markdown linting

**Checkpoint**: File 2 complete, students can create Gazebo worlds.

---

### Phase 3: File 3 - Sensor Simulation (2 days)
**Objective**: Write sensor tutorials with complete URDF (4,000-4,500 words).

**Tasks**:
1. Write Section 0: GPU requirement banner
2. Write Section 1: Sensor simulation architecture
3. Write Section 2: Depth camera URDF + visualization
4. Write Section 3: LiDAR URDF + Python subscriber
5. Write Section 4: IMU URDF + Python subscriber
6. Write Section 5: Performance tuning (CPU fallback)
7. Write Section 6: Exercise (add second LiDAR)
8. Test all sensor plugins in Gazebo Fortress
9. Verify RViz2 visualization instructions
10. Run Python subscribers and verify output
11. Markdown linting

**Checkpoint**: File 3 complete, students can simulate sensors.

---

### Phase 4: File 4 - Unity Visualization (1 day)
**Objective**: Write Unity awareness guide (2,500-3,000 words).

**Tasks**:
1. Write Section 1: Why Unity for robotics
2. Write Section 2: Installation (Unity Hub, Unity 2021.3 LTS)
3. Write Section 3: ROS-TCP-Connector setup
4. Write Section 4: Importing robot model
5. Write Section 5: Gazebo vs Unity comparison
6. Write Section 6: Next steps and resources
7. Add Unity C# code example (joint state subscriber)
8. Markdown linting

**Checkpoint**: File 4 complete, awareness-level Unity integration covered.

---

### Phase 5: Sidebar Integration (0.5 days)
**Objective**: Integrate all 4 files into Docusaurus sidebar.

**Tasks**:
1. Verify files are in `web/docs/en/module-2-digital-twin/`
2. Check sidebar auto-generation (Docusaurus should detect numbered files)
3. Verify navigation order: intro.md → 01 → 02 → 03 → 04
4. Test all internal links between files
5. Verify Mermaid diagrams render correctly

**Checkpoint**: Module 2 fully integrated in documentation site.

---

### Phase 6: Verification and Testing (1 day)
**Objective**: End-to-end testing on clean system.

**Tasks**:
1. Spin up clean Ubuntu 22.04 VM
2. Install ROS 2 Humble + Gazebo Fortress (follow Module 0)
3. Execute every command from File 2 (world creation)
4. Execute every command from File 3 (sensor simulation)
5. Verify RViz2 visualizations match screenshots
6. Run Python subscriber scripts
7. Check for broken links, typos, formatting issues
8. Verify Mermaid diagrams render
9. Mobile responsiveness check

**Checkpoint**: All tutorials verified on clean system.

---

### Phase 7: Beta Testing (1-2 days)
**Objective**: 3 students complete all tutorials, measure success criteria.

**Tasks**:
1. Recruit 3 beta testers (Module 0 + Module 1 completed)
2. Measure SC-001: Gazebo world creation time (<15 min target)
3. Measure SC-002: Sensor simulation success rate (100% target)
4. Measure SC-003: Gazebo vs Unity explanation (<3 min target)
5. Measure SC-004: Self-contained learning (80% no external help)
6. Measure SC-005: Physics understanding (inertia formula)
7. Measure SC-006: GPU awareness (ray tracing explanation)
8. Measure SC-007: LiDAR parameter modification (<10 min)
9. Collect feedback on confusing sections
10. Revise content based on feedback

**Checkpoint**: All success criteria met, content validated by students.

---

### Phase 8: Final Polish and Deployment (0.5 days)
**Objective**: Final proofreading and production deployment.

**Tasks**:
1. Final proofreading (grammar, technical accuracy)
2. Verify all success criteria mappings (FR-001 to FR-012)
3. Production build: `npm run build` in `web/`
4. Merge to main branch
5. Deploy to GitHub Pages
6. Create PHR documenting implementation

**Checkpoint**: Module 2 live in production.

---

## Risk Mitigation

### Risk 1: Gazebo Fortress Plugin Names Changed
**Mitigation**: Test all plugins on ROS 2 Humble + Gazebo Fortress before writing. Provide fallback for Classic if needed (but discourage).

### Risk 2: ROS-TCP-Connector Breaking Changes
**Mitigation**: Pin Unity 2021.3 LTS and ROS-TCP-Connector v0.7.0 (last stable version). Document exact versions.

### Risk 3: Students Without RTX GPU
**Mitigation**:
- File 1: Clear warning about GPU requirement
- File 2: Mark as "CPU-compatible"
- File 3: Provide CPU fallback settings, but warn of low FPS

### Risk 4: Unity Setup Complexity
**Mitigation**: Keep File 4 as awareness-only (no mandatory hands-on). Students can skip if not interested in Unity.

---

## Success Metrics

All success criteria from spec.md:

| ID | Metric | Target | Measurement Method |
|----|--------|--------|--------------------|
| SC-001 | Gazebo world creation time | <15 min, avg <12 min | Timer during beta testing |
| SC-002 | Sensor simulation success rate | 100% first attempt | Beta testers follow File 3 |
| SC-003 | Gazebo vs Unity explanation | <3 min, mention physics/visuals | Verbal explanation recorded |
| SC-004 | Self-contained learning | 80%+ no external help | Beta tester self-report |
| SC-005 | Physics understanding | 100% correct inertia formula | Post-File 2 quiz |
| SC-006 | GPU awareness | 100% mention ray tracing | Post-File 3 question |
| SC-007 | LiDAR parameter modification | <10 min, 100% success | Timed modification task |

---

## Definition of Done

Module 2 implementation is complete when:

1. ✅ All 4 markdown files written and published
2. ✅ All code snippets tested in Gazebo Fortress + ROS 2 Humble
3. ✅ All Mermaid diagrams render correctly
4. ✅ All functional requirements (FR-001 to FR-012) implemented
5. ✅ Sidebar navigation configured and tested
6. ✅ Markdown linting passes (no broken links)
7. ✅ 3 beta testers complete tutorials and pass all success criteria
8. ✅ Files published to `web/docs/en/module-2-digital-twin/`
9. ✅ Production build succeeds (`npm run build`)
10. ✅ PHR created documenting implementation

---

## Timeline Estimates

**Single Author (Sequential)**:
- Phase 0: Research - 1.5 days
- Phase 1: File 1 - 1 day
- Phase 2: File 2 - 1.5 days
- Phase 3: File 3 - 2 days
- Phase 4: File 4 - 1 day
- Phase 5: Sidebar - 0.5 days
- Phase 6: Verification - 1 day
- Phase 7: Beta Testing - 1.5 days
- Phase 8: Deployment - 0.5 days
- **Total: 10-11 days**

**Multiple Authors (Parallel)**:
- Phase 0: Research - 1.5 days (blocking)
- Phase 1-4: Files (parallel) - 2 days (longest file = File 3)
- Phase 5-8: Sequential - 3.5 days
- **Total: 7-8 days**

---

## Related Documentation

- **Spec**: [spec.md](./spec.md)
- **Tasks**: [tasks.md](./tasks.md) (to be generated)
- **Module 1 Spec**: `specs/1-nervous-system-ros2/spec.md`
- **Gazebo Fortress Docs**: https://gazebosim.org/docs/fortress
- **Unity Robotics Hub**: https://github.com/Unity-Technologies/Unity-Robotics-Hub
