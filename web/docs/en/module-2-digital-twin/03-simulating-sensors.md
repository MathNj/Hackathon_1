---
id: simulating-sensors
title: Adding Eyes & Ears (Sensors)
sidebar_label: Simulating Sensors
sidebar_position: 3
description: Learn how to add LiDAR and depth cameras to robots using Gazebo plugins and GPU ray tracing
keywords: [lidar, depth camera, sensors, gazebo plugins, ray tracing, gpu]
---

# Adding Eyes & Ears (Sensors)

## Why Simulate Sensors?

Physical sensors are **expensive**:
- **LiDAR** (Velodyne VLP-16): ~$1,500
- **Depth Camera** (Intel RealSense D435i): ~$200
- **IMU** (Inertial Measurement Unit): ~$50

With Gazebo, you can **develop perception algorithms** using simulated sensors before purchasing hardware. Once your algorithm works in simulation, you can confidently buy the real sensor knowing your code is ready.

---

## How Ray Tracing Mimics LiDAR

### What is a LiDAR?

**LiDAR** (Light Detection and Ranging) is like a **360° tape measure** that shoots laser beams and calculates distance by measuring how long light takes to bounce back.

```
Robot with LiDAR
    |
    | (shoots 16 laser beams in 360°)
    |
    v
[Obstacle] <-- Beam reflects back in 0.00001 seconds
Distance = (Speed of Light × Time) / 2 = 15 meters
```

### GPU Ray Tracing

Gazebo simulates this by:
1. **Shooting rays** from the LiDAR sensor origin (like virtual laser beams)
2. **Collision detection** — GPU calculates where each ray hits objects in 3D space
3. **Distance calculation** — Converts hit point to distance measurement
4. **ROS 2 message** — Publishes `sensor_msgs/LaserScan` with distance array

**Why GPU?** A 16-beam LiDAR at 10 Hz shoots **2.3 million rays per second**. CPU would take ~10 seconds per frame. RTX GPU does it in **0.003 seconds** (300 FPS).

---

## Adding a LiDAR Sensor to Your Robot

Let's add a **Velodyne VLP-16 equivalent** (16 laser beams, 360° horizontal scan) to a robot.

### Step 1: Define the LiDAR Link (URDF)

Add this to your robot's URDF file:

```xml
<!-- LiDAR Link (the physical sensor body) -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.07"/> <!-- Small cylinder -->
    </geometry>
    <material name="lidar_black">
      <color rgba="0.1 0.1 0.1 1.0"/> <!-- Black color -->
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.07"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.83"/> <!-- Velodyne VLP-16 weighs 830 grams -->
    <inertia ixx="0.001" ixy="0" ixz="0"
             iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<!-- Joint: Attach LiDAR to robot base -->
<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/> <!-- Mount on top, 30cm above base -->
</joint>
```

### Step 2: Add Gazebo Sensor Plugin

Add this **inside a `<gazebo>` block** in your URDF:

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="gpu_lidar">
    <update_rate>10</update_rate> <!-- 10 Hz = 10 scans per second -->

    <ray>
      <!-- Horizontal Scan: 360 degrees -->
      <scan>
        <horizontal>
          <samples>1800</samples> <!-- 1800 points per scan -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle> <!-- -180° -->
          <max_angle>3.14159</max_angle>  <!-- +180° -->
        </horizontal>
        <!-- Vertical Scan: 16 beams from -15° to +15° -->
        <vertical>
          <samples>16</samples> <!-- Velodyne VLP-16 has 16 laser beams -->
          <resolution>1</resolution>
          <min_angle>-0.2618</min_angle> <!-- -15 degrees -->
          <max_angle>0.2618</max_angle>  <!-- +15 degrees -->
        </vertical>
      </scan>

      <!-- Range: 0.1m to 100m -->
      <range>
        <min>0.1</min>
        <max>100.0</max>
        <resolution>0.01</resolution> <!-- 1cm precision -->
      </range>

      <!-- Noise: Add realistic Gaussian noise -->
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev> <!-- ±1cm error -->
      </noise>
    </ray>

    <!-- Gazebo Plugin: Publishes to ROS 2 -->
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

### Step 3: Spawn Robot and Verify

```bash
# Terminal 1: Launch Gazebo world
gz sim test_world.sdf

# Terminal 2: Spawn robot with LiDAR
ros2 run ros_gz_sim create -file robot_with_lidar.urdf -name my_robot

# Terminal 3: Echo LiDAR data
ros2 topic echo /lidar/scan
```

**Expected Output**:
```yaml
header:
  stamp: {sec: 10, nanosec: 500000000}
  frame_id: lidar_link
angle_min: -3.14159
angle_max: 3.14159
angle_increment: 0.00349066
time_increment: 0.0
scan_time: 0.1
range_min: 0.1
range_max: 100.0
ranges: [15.2, 15.3, 15.1, 14.9, ...] # 1800 distance measurements
```

---

## Visualizing LiDAR in RViz2

```bash
# Launch RViz2
rviz2
```

**In RViz2**:
1. **Set Fixed Frame**: Change from `map` to `lidar_link` (bottom-left dropdown)
2. **Add LaserScan Display**: Click "Add" → "By topic" → `/lidar/scan` → LaserScan
3. **Adjust Visualization**:
   - Size: 0.05
   - Color: Red (RGB: 255, 0, 0)

**What you see**: Red dots showing obstacles detected by the LiDAR in 360°. Move objects in Gazebo and watch the point cloud update in real-time!

---

## Understanding the Plugin Parameters

### `<update_rate>10</update_rate>`

How many scans per second. Higher = more data but more CPU/GPU load.

- **10 Hz**: Standard for navigation (good balance)
- **20 Hz**: High-speed navigation (autonomous cars)
- **5 Hz**: Low-end systems or CPU-only machines

### `<samples>1800</samples>`

Number of laser beams in horizontal sweep. More = finer resolution.

- **1800 samples** = 0.2° between each beam (very dense)
- **360 samples** = 1° between each beam (lower resolution, faster)

### `<vertical><samples>16</samples></vertical>`

Number of laser rings (vertical beams). Velodyne VLP-16 has 16, VLP-32 has 32.

### `<noise>` Block

Real sensors have noise! Adding Gaussian noise makes simulation more realistic.

```xml
<stddev>0.01</stddev> <!-- ±1cm random error per measurement -->
```

Remove the `<noise>` block for perfect measurements (useful for debugging).

---

## Adding Other Sensors (Quick Overview)

### Depth Camera (Intel RealSense D435i)

Similar process, but use:
```xml
<sensor name="depth_camera" type="depth_camera">
  <plugin filename="libgazebo_ros_camera.so" ...>
```

Publishes to:
- `/camera/image_raw` (RGB image)
- `/camera/depth/image_raw` (Depth map)

### IMU (Inertial Measurement Unit)

```xml
<sensor name="imu" type="imu">
  <plugin filename="libgazebo_ros_imu_sensor.so" ...>
```

Publishes to:
- `/imu/data` (Orientation, angular velocity, linear acceleration)

---

## Performance: GPU vs CPU

| Sensor Configuration | RTX 4070 Ti (GPU) | CPU Only |
|----------------------|-------------------|----------|
| LiDAR (16 beams, 10 Hz) | 300 FPS | 2 FPS |
| LiDAR (64 beams, 20 Hz) | 60 FPS | &lt;1 FPS |
| Depth Camera (640×480, 30 Hz) | 200 FPS | 1 FPS |

**Conclusion**: Without an RTX GPU, sensor simulation is **unusable**. The 10-100x speedup from RT cores is mandatory for real-time development.

---

## Common Errors

### `/lidar/scan` topic not publishing

**Debug**:
```bash
# Check if plugin loaded
gz topic -l | grep lidar

# Check Gazebo console for errors
# Look for "Failed to load plugin" messages
```

**Fix**: Verify `libgazebo_ros_ray_sensor.so` exists:
```bash
ls /opt/ros/humble/lib/ | grep gazebo_ros_ray
```

### RViz2 shows no data

**Fix**: Verify fixed frame matches sensor frame:
- RViz2 Fixed Frame: `lidar_link`
- Sensor `<frame_name>`: `lidar_link`

These **must match** or you'll see: "Transform [lidar_link] does not exist."

---

## Key Takeaways

✅ **LiDAR uses ray tracing** — GPU shoots virtual laser beams and calculates distances

✅ **Gazebo plugin**: `libgazebo_ros_ray_sensor.so` converts ray tracing to ROS 2 LaserScan

✅ **16 beams, 1800 samples, 10 Hz** — Standard Velodyne VLP-16 configuration

✅ **GPU required** — RTX GPU is 10-100x faster than CPU (300 FPS vs 2 FPS)

✅ **Visualize in RViz2** — Set fixed frame to `lidar_link`, add LaserScan display

---

## Interactive Flashcards

import Flashcards from '@site/src/components/Flashcards';

<Flashcards
  title="Sensor Simulation Review"
  cards={[
    {
      id: 1,
      question: "Why simulate sensors instead of buying them?",
      answer: "Physical sensors are expensive (LiDAR ~$1,500, depth camera ~$200). Gazebo lets you develop perception algorithms using simulated sensors before purchasing hardware. Once your algorithm works in simulation, you can confidently buy the real sensor.",
      category: "Motivation"
    },
    {
      id: 2,
      question: "How does LiDAR work?",
      answer: "LiDAR (Light Detection and Ranging) is like a 360° tape measure that shoots laser beams and calculates distance by measuring how long light takes to bounce back. Distance = (Speed of Light × Time) / 2.",
      category: "LiDAR Basics"
    },
    {
      id: 3,
      question: "How does Gazebo simulate LiDAR using ray tracing?",
      answer: "Gazebo shoots rays from the LiDAR sensor origin (virtual laser beams), performs GPU collision detection to find where each ray hits objects, calculates distances, and publishes sensor_msgs/LaserScan messages to ROS 2.",
      category: "Ray Tracing"
    },
    {
      id: 4,
      question: "Why is GPU required for sensor simulation?",
      answer: "A 16-beam LiDAR at 10 Hz shoots 2.3 million rays per second. CPU would take ~10 seconds per frame. RTX GPU does it in 0.003 seconds (300 FPS) - a 3000x speedup making real-time development possible.",
      category: "Performance"
    },
    {
      id: 5,
      question: "What does the update_rate parameter control?",
      answer: "update_rate sets how many scans per second. Higher = more data but more CPU/GPU load. 10 Hz is standard for navigation (good balance), 20 Hz for high-speed navigation (autonomous cars), 5 Hz for low-end systems.",
      category: "Sensor Parameters"
    },
    {
      id: 6,
      question: "Why add Gaussian noise to simulated sensors?",
      answer: "Real sensors have noise! Adding Gaussian noise (e.g., stddev 0.01 for ±1cm error) makes simulation more realistic and helps your algorithms be robust to real-world sensor imperfections. Remove noise for perfect measurements when debugging.",
      category: "Realism"
    },
    {
      id: 7,
      question: "What is the performance difference between RTX GPU and CPU?",
      answer: "For 16-beam LiDAR at 10 Hz: RTX 4070 Ti achieves 300 FPS vs. 2 FPS on CPU. For 64-beam LiDAR at 20 Hz: 60 FPS vs. <1 FPS. For depth cameras at 640×480, 30 Hz: 200 FPS vs. 1 FPS. GPU is 10-100x faster.",
      category: "Hardware"
    }
  ]}
/>

---

**Next**: Learn when to use Unity instead of Gazebo in [**High-Fidelity Rendering in Unity**](./04-unity-visualization.md)! 🎨
