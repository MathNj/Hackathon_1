# Feature Specification: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature ID**: 2-digital-twin-sim
**Branch**: `2-digital-twin-sim`
**Status**: Draft
**Created**: 2025-12-04
**Last Updated**: 2025-12-04

---

## Executive Summary

**What**: Create comprehensive educational content (3 markdown files) teaching students how to build and utilize digital twins for robot development using Gazebo and Unity simulators.

**Why**: Before risking a $3,000+ physical robot, engineers crash their robots in simulation 100 times. This module teaches students to create accurate virtual clones of robots for safe, rapid iteration.

**Who**: University-level students (computer science, robotics engineering) with Module 0 (hardware setup) and Module 1 (ROS 2 fundamentals) completed.

**Success Metric**: Students can spawn a custom robot in Gazebo with LiDAR sensor, run physics simulation, and explain Gazebo vs Unity tradeoffs in under 3 minutes.

---

## Problem Statement

### Current State
Students have completed Module 1 and understand ROS 2 architecture (nodes, topics, URDF). However, they lack:
- Understanding of **digital twin** concept (virtual replica before physical build)
- Knowledge of **physics simulation** (rigid body dynamics, collisions, gravity)
- Experience with **Gazebo** (industry-standard ROS 2 simulator)
- Awareness of **Unity integration** for photorealistic rendering
- Ability to simulate **sensors** (cameras, LiDAR, IMU) before purchasing hardware

### Desired State
After Module 2, students can:
- ✅ Explain the digital twin concept and its role in robotics workflow
- ✅ Differentiate Gazebo (physics accuracy) vs Unity (visual realism)
- ✅ Write SDF (Simulation Description Format) world files
- ✅ Add physics properties to URDF models (inertia, friction, damping)
- ✅ Spawn robots in Gazebo and control them via ROS 2 topics
- ✅ Simulate depth cameras, LiDAR, and IMU sensors
- ✅ Understand GPU ray tracing requirements for sensor simulation

### Gap
Missing educational content covering:
1. **Conceptual Foundation**: Digital twin concept, simulation-to-reality pipeline
2. **Technical Skills**: SDF syntax, Gazebo plugins, sensor configuration
3. **Hardware Context**: Why NVIDIA RTX GPU is required for realistic sensor simulation

---

## User Stories

### US1: Digital Twin Concept (P1 - MVP)
**As a** robotics student
**I want** to understand what a "digital twin" is and why simulation is critical
**So that** I can appreciate the value of testing in virtual environments before physical builds

**Acceptance Criteria**:
- Learn the digital twin definition (virtual replica synchronized with physical counterpart)
- Understand the sim-to-real workflow (design → simulate → validate → deploy)
- See real-world examples (Boston Dynamics testing Spot, NASA Mars rover validation)
- Contrast Gazebo (physics) vs Unity (visuals) with specific use cases
- Learn when to use each tool (Gazebo for control algorithms, Unity for perception training)

**Priority**: P1 (MVP) — Conceptual foundation required before hands-on work

---

### US2: Gazebo World Building (P1 - MVP)
**As a** robotics student
**I want** to create custom Gazebo simulation environments
**So that** I can test robots in various scenarios (flat ground, obstacles, varied lighting)

**Acceptance Criteria**:
- Understand **SDF (Simulation Description Format)** syntax vs URDF
- Learn physics engine configuration (gravity, time step, solver)
- Create a custom world with ground plane, obstacles, and lighting
- Spawn a robot from URDF/SDF into the world
- Control the robot via ROS 2 topics (`/cmd_vel`)
- Understand rigid body dynamics (mass, inertia, friction, damping)

**Priority**: P1 (MVP) — Core Gazebo skills needed for all simulation work

---

### US3: Sensor Simulation (P1 - MVP)
**As a** robotics student
**I want** to add virtual sensors (cameras, LiDAR, IMU) to my robot
**So that** I can develop perception algorithms without purchasing expensive hardware

**Acceptance Criteria**:
- Understand sensor simulation architecture (Gazebo plugin → ROS 2 topic)
- Add a **depth camera** (Intel RealSense D435i equivalent) to URDF
- Add a **LiDAR sensor** (Velodyne VLP-16 equivalent) to URDF
- Add an **IMU** (Inertial Measurement Unit) to URDF
- Visualize sensor data in RViz2 (point clouds, camera images, orientation)
- Understand GPU ray tracing requirements (NVIDIA RTX for realistic depth/LiDAR)
- Write a ROS 2 subscriber to process simulated sensor data

**Priority**: P1 (MVP) — Sensor simulation is core value proposition of this module

---

### US4: Unity Integration Overview (P2)
**As a** robotics student
**I want** to understand when to use Unity instead of Gazebo
**So that** I can choose the right tool for my use case (control vs perception)

**Acceptance Criteria**:
- Learn Unity's role in robotics (photorealistic rendering, domain randomization)
- Understand Unity Robotics Hub architecture (ROS-TCP-Connector)
- See use cases: synthetic training data for computer vision
- Contrast physics accuracy (Gazebo better) vs visual quality (Unity better)
- Learn about NVIDIA Isaac Sim (Omniverse) as advanced alternative

**Priority**: P2 — Awareness-level content, not hands-on (Unity setup is complex)

---

## Functional Requirements

### FR-001: Digital Twin Explanation
**Description**: File 1 must explain the digital twin concept using the analogy of a "rehearsal before the performance."

**Specification**:
- Define "digital twin" (virtual model synchronized with physical counterpart)
- Explain sim-to-real pipeline: Design (CAD/URDF) → Simulate (Gazebo) → Validate (physical tests) → Deploy (production)
- Provide real-world examples:
  - Boston Dynamics: Spot tested 100,000+ times in simulation before physical testing
  - NASA: Mars rovers validated in digital twin environments
  - Waymo: Autonomous vehicles tested billions of miles in simulation
- Include a diagram showing the digital twin workflow

**Priority**: P1
**User Story**: US1

---

### FR-002: Gazebo vs Unity Comparison
**Description**: File 1 must contrast Gazebo and Unity with a decision matrix.

**Specification**:
- Create comparison table with criteria:
  - **Physics Accuracy**: Gazebo (ODE/Bullet engines) vs Unity (PhysX)
  - **Visual Realism**: Gazebo (basic) vs Unity (AAA-game quality)
  - **ROS 2 Integration**: Gazebo (native) vs Unity (via ROS-TCP-Connector)
  - **Sensor Simulation**: Gazebo (ray tracing via GPU) vs Unity (GPU shaders)
  - **Use Cases**: Gazebo (control, navigation) vs Unity (perception, CV training)
- Provide decision guide: "Use Gazebo if..." and "Use Unity if..."

**Priority**: P1
**User Story**: US1

---

### FR-003: Physics Engine Fundamentals
**Description**: File 2 must teach rigid body dynamics concepts.

**Specification**:
- Explain **rigid body dynamics**: objects don't deform, only translate/rotate
- Cover physics parameters:
  - **Mass**: Total weight in kilograms
  - **Inertia**: Resistance to rotation (3x3 inertia tensor)
  - **Gravity**: Default -9.81 m/s² in Z-axis
  - **Friction**: Surface interaction (mu, mu2, slip parameters)
  - **Damping**: Energy dissipation (linear and angular damping)
- Explain why accurate physics matters for control algorithms

**Priority**: P1
**User Story**: US2

---

### FR-004: SDF vs URDF
**Description**: File 2 must explain the difference between SDF and URDF formats.

**Specification**:
- Create comparison table:
  - **URDF**: Robot-only, single model, used by ROS 2
  - **SDF**: Worlds + robots, multiple models, used by Gazebo
  - **Physics**: URDF (basic), SDF (advanced collision, surface properties)
  - **Sensors**: URDF (limited), SDF (full plugin support)
- Explain conversion: `gz sdf -p model.urdf > model.sdf`
- Show when to use each format

**Priority**: P1
**User Story**: US2

---

### FR-005: Gazebo World Creation Tutorial
**Description**: File 2 must provide step-by-step tutorial for creating a custom Gazebo world.

**Specification**:
- Complete SDF world file with:
  - Physics engine configuration (ODE solver, time step 0.001s)
  - Ground plane with texture
  - Sun lighting (directional light)
  - Obstacles (boxes, cylinders)
- Instructions to launch world: `gz sim <world_name>.sdf`
- Instructions to spawn robot: `ros2 run gazebo_ros spawn_entity.py -entity my_robot -file robot.urdf`

**Priority**: P1
**User Story**: US2

---

### FR-006: Robot Physics Properties
**Description**: File 2 must show how to add physics properties to URDF.

**Specification**:
- Enhance Module 1's `simple_arm.urdf` with:
  - Accurate `<inertial>` tags (mass, inertia tensor)
  - Surface friction parameters
  - Joint damping and effort limits
- Explain how to calculate inertia tensors (MeshLab, CAD software)
- Provide formula for cylinder inertia: `Ixx = 1/12 * m * (3r² + h²)`

**Priority**: P1
**User Story**: US2

---

### FR-007: Depth Camera Simulation
**Description**: File 3 must provide complete code for adding a depth camera sensor.

**Specification**:
- URDF snippet adding Intel RealSense D435i equivalent:
  - Camera link with visual mesh
  - Gazebo plugin: `libgazebo_ros_camera.so`
  - Parameters: 640x480 resolution, 30 Hz, 87° HFOV
  - ROS 2 topics: `/camera/image_raw`, `/camera/depth/image_raw`, `/camera/camera_info`
- Instructions to visualize in RViz2
- Explain GPU ray tracing (NVIDIA RTX required for realistic depth)

**Priority**: P1
**User Story**: US3

---

### FR-008: LiDAR Simulation
**Description**: File 3 must provide complete code for adding a LiDAR sensor.

**Specification**:
- URDF snippet adding Velodyne VLP-16 equivalent:
  - LiDAR link with visual mesh
  - Gazebo plugin: `libgazebo_ros_ray_sensor.so`
  - Parameters: 16 beams, 360° horizontal, ±15° vertical, 10 Hz
  - ROS 2 topic: `/scan` (sensor_msgs/LaserScan or PointCloud2)
- Instructions to visualize point cloud in RViz2
- Explain laser ray tracing computation (GPU acceleration)

**Priority**: P1
**User Story**: US3

---

### FR-009: IMU Simulation
**Description**: File 3 must provide complete code for adding an IMU sensor.

**Specification**:
- URDF snippet adding IMU:
  - IMU link (typically at robot center of mass)
  - Gazebo plugin: `libgazebo_ros_imu_sensor.so`
  - Parameters: 100 Hz update rate, noise parameters
  - ROS 2 topic: `/imu` (sensor_msgs/Imu)
- Explain IMU outputs: linear acceleration, angular velocity, orientation (quaternion)
- Show how to subscribe to IMU data in Python

**Priority**: P1
**User Story**: US3

---

### FR-010: RViz2 Sensor Visualization
**Description**: File 3 must teach students to visualize all sensors in RViz2.

**Specification**:
- Instructions to launch RViz2: `rviz2`
- Add displays:
  - Camera: Add "Image" display, topic `/camera/image_raw`
  - Depth: Add "DepthCloud" display, topic `/camera/depth/image_raw`
  - LiDAR: Add "LaserScan" or "PointCloud2" display, topic `/scan`
  - IMU: Show robot orientation via TF frames
- Screenshot of complete RViz2 setup with all sensors

**Priority**: P1
**User Story**: US3

---

### FR-011: GPU Requirements Explanation
**Description**: All files must explain why NVIDIA RTX GPU is required.

**Specification**:
- Explain sensor simulation pipeline:
  1. Gazebo ray tracing (GPU) → depth map
  2. Point cloud generation (GPU)
  3. ROS 2 message publishing
- Minimum requirement: NVIDIA RTX 4060 (8GB VRAM)
- Recommended: NVIDIA RTX 4070 Ti (12GB VRAM)
- Explain ray tracing cores (RT cores) accelerate sensor simulation 10-100x vs CPU

**Priority**: P1
**User Story**: US1, US3

---

### FR-012: Unity Robotics Hub Overview
**Description**: File 1 must introduce Unity as alternative to Gazebo (awareness only, no tutorial).

**Specification**:
- Brief overview (1-2 paragraphs):
  - Unity Robotics Hub: Unity plugin for ROS 2 communication
  - Use case: Synthetic training data for computer vision (domain randomization)
  - Example: Train object detection with 10,000 randomized scenes in Unity
- Link to Unity Robotics Hub documentation (external)
- Note: "Unity setup covered in advanced modules"

**Priority**: P2
**User Story**: US4

---

## Non-Functional Requirements

### NFR-001: Target Audience
- University-level students (sophomore/junior year)
- Prerequisites: Module 0 (Ubuntu + ROS 2 Humble + GPU drivers) and Module 1 (ROS 2 fundamentals)
- Assumed knowledge: Python basics, Linux command line, URDF syntax

### NFR-002: Technical Accuracy
- All Gazebo commands must work with Gazebo Fortress (gz-sim7)
- All ROS 2 code must be compatible with ROS 2 Humble
- All sensor configurations must produce valid ROS 2 messages

### NFR-003: Code Quality
- All code snippets must be syntactically correct and runnable
- Include comments explaining every parameter
- Provide expected output for each command

### NFR-004: Accessibility
- All diagrams must have descriptive alt text
- Code blocks must use proper syntax highlighting
- Headings must follow hierarchical structure (h2 → h3 → h4)

### NFR-005: Pedagogical Tone
- Academic but engaging (avoid dry textbook style)
- Use analogies (e.g., "Digital twin is like a dress rehearsal before opening night")
- Include "Why This Matters" sections connecting to real-world applications

---

## Success Criteria

### SC-001: Gazebo World Creation Speed
**Metric**: Student can create a custom Gazebo world with ground plane and obstacle in **under 15 minutes** after reading File 2.

**Measurement**: 3 beta testers attempt world creation tutorial with timer.

**Target**: 100% completion rate, average time < 12 minutes.

---

### SC-002: Sensor Simulation Success Rate
**Metric**: Student can add a LiDAR sensor to a robot URDF and visualize point cloud in RViz2 with **zero compilation errors**.

**Measurement**: 3 beta testers follow File 3 tutorial, record success on first attempt.

**Target**: 100% success rate (all code snippets work as written).

---

### SC-003: Concept Explanation
**Metric**: Student can verbally explain Gazebo vs Unity tradeoffs in **under 3 minutes** after reading File 1.

**Measurement**: Ask beta testers: "When would you use Gazebo vs Unity?" Record response quality.

**Target**: 100% mention physics accuracy (Gazebo) and visual realism (Unity).

---

### SC-004: Self-Contained Learning
**Metric**: Student completes all tutorials without consulting external documentation (Stack Overflow, Gazebo docs).

**Measurement**: Beta testers self-report external resources used during Module 2.

**Target**: 80%+ complete without external help.

---

### SC-005: Physics Understanding
**Metric**: Student can correctly calculate inertia tensor for a cylinder after reading File 2.

**Measurement**: Provide cylinder dimensions (r=0.05m, h=0.3m, m=1.0kg), ask for Ixx calculation.

**Target**: 100% provide correct formula (even if arithmetic is incorrect).

---

### SC-006: GPU Awareness
**Metric**: Student can explain why NVIDIA RTX GPU is required for sensor simulation.

**Measurement**: Ask beta testers: "Why is a gaming GPU needed for robotics simulation?"

**Target**: 100% mention ray tracing or sensor simulation acceleration.

---

### SC-007: Code Modification
**Metric**: Student can modify LiDAR parameters (e.g., change from 16 to 32 beams) in **under 10 minutes**.

**Measurement**: Provide modification task after completing File 3 tutorial.

**Target**: 100% successfully modify and verify in simulation.

---

## Key Entities and Definitions

### Digital Twin
**Definition**: A virtual replica of a physical robot that mirrors its geometry, physics, and behavior for testing and validation.

**Attributes**: URDF/SDF model, physics properties (mass, inertia), sensor suite, control interface.

---

### SDF (Simulation Description Format)
**Definition**: XML-based format for describing simulation worlds, robots, and physics in Gazebo.

**Attributes**: `<world>`, `<model>`, `<link>`, `<joint>`, `<sensor>`, `<plugin>` tags.

**Contrast**: URDF (robot-only), SDF (worlds + robots + advanced physics).

---

### Gazebo Plugin
**Definition**: C++ library that extends Gazebo functionality (e.g., sensors, controllers, custom physics).

**Common Plugins**:
- `libgazebo_ros_camera.so` - Camera sensor
- `libgazebo_ros_ray_sensor.so` - LiDAR sensor
- `libgazebo_ros_imu_sensor.so` - IMU sensor
- `libgazebo_ros_diff_drive.so` - Differential drive controller

---

### Physics Engine
**Definition**: Software that simulates rigid body dynamics, collisions, and forces.

**Gazebo Options**: ODE (default), Bullet, DART, Simbody.

**Parameters**: Gravity, time step, solver iterations, constraint force mixing (CFM).

---

### Ray Tracing (Sensor Simulation)
**Definition**: Technique to simulate light/laser propagation by tracing rays from sensor through 3D scene.

**Hardware Acceleration**: NVIDIA RTX GPUs have dedicated RT cores for 10-100x speedup.

**Applications**: Depth cameras, LiDAR, realistic lighting.

---

## Assumptions

1. **Hardware**: Students have NVIDIA RTX GPU (4060 or better) installed with drivers
2. **Software**: Gazebo Fortress (gz-sim7) and ROS 2 Humble installed via Module 0
3. **Prior Knowledge**: Students completed Module 1 (understand URDF, nodes, topics)
4. **Time**: Students allocate 4-6 hours for Module 2 (reading + hands-on tutorials)

---

## Dependencies

### Upstream (Blocking This Feature)
- **Module 0**: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress, NVIDIA drivers installed
- **Module 1**: Students understand URDF syntax, ROS 2 nodes, topics

### Downstream (Blocked By This Feature)
- **Module 3**: AI models need simulated sensor data for training
- **Module 4**: Reinforcement learning requires Gazebo environments
- **Module 5**: Capstone project will use digital twin for validation

### Parallel Work
- None (Module 2 is independent once prerequisites are met)

---

## Edge Cases and Error Handling

### Edge Case 1: No GPU Available
**Scenario**: Student runs Gazebo on CPU-only machine.

**Impact**: Sensor simulation will be extremely slow (1-2 FPS vs 30 FPS on GPU).

**Handling**:
- Add warning callout in File 1: "⚠️ Without NVIDIA RTX GPU, sensor simulation will be unusable."
- Provide CPU fallback: Students can still learn SDF syntax and world building without sensors.

---

### Edge Case 2: URDF to SDF Conversion Issues
**Scenario**: Complex URDF with nested meshes fails to convert to SDF.

**Impact**: Robot doesn't spawn in Gazebo, student gets cryptic error.

**Handling**:
- Provide troubleshooting section in File 2:
  - Check mesh file paths (must be absolute or relative to URDF)
  - Validate URDF: `check_urdf robot.urdf`
  - Convert manually: `gz sdf -p robot.urdf > robot.sdf`

---

### Edge Case 3: Sensor Not Publishing Data
**Scenario**: Student adds LiDAR plugin but `/scan` topic is empty.

**Impact**: RViz2 shows no point cloud, student thinks code is wrong.

**Handling**:
- Add debugging checklist in File 3:
  - Verify plugin loaded: Check Gazebo console for error messages
  - Check topic exists: `ros2 topic list | grep scan`
  - Verify frame exists: `ros2 run tf2_tools view_frames`
  - Ensure sensor has line of sight (not obstructed by robot body)

---

### Edge Case 4: RViz2 Crashes on Large Point Clouds
**Scenario**: Student sets LiDAR to 64 beams at 20 Hz, overwhelming RViz2.

**Impact**: RViz2 freezes or crashes.

**Handling**:
- Set reasonable defaults in tutorial (16 beams, 10 Hz)
- Add performance note: "For visualization, start with low resolution. Increase for actual algorithms."

---

## Out of Scope

### Explicitly Not Included:
1. **Unity Hands-On Tutorial**: Unity setup is complex (Windows/macOS, license, ROS-TCP-Connector). Awareness only in this module.
2. **Advanced Physics**: Soft body dynamics, fluid simulation, deformable objects (require NVIDIA FleX or similar).
3. **Multi-Robot Simulation**: Swarm robotics, fleet management (covered in advanced modules).
4. **Cloud Simulation**: AWS RoboMaker, NVIDIA Omniverse (enterprise tools, out of scope).
5. **Custom Gazebo Plugins (C++)**: Requires C++ expertise, out of scope for Python-focused curriculum.
6. **Domain Randomization**: Covered in Module 3 (AI training) when needed.

---

## Risks and Mitigations

### Risk 1: Gazebo Version Fragmentation
**Impact**: Students using older Gazebo Classic (gazebo11) instead of Gazebo Fortress (gz-sim7).

**Likelihood**: Medium (Gazebo Classic still widely documented online).

**Mitigation**:
- Add version check at start of File 1: `gz sim --version` (expect Fortress 7.x)
- Explicitly state: "This module uses **Gazebo Fortress (gz-sim7)**, not Gazebo Classic."

---

### Risk 2: GPU Driver Issues
**Impact**: Gazebo crashes or sensors don't work due to driver mismatch.

**Likelihood**: Low (Module 0 covers driver installation).

**Mitigation**:
- Add verification step: `nvidia-smi` (should show driver version 525+)
- Link back to Module 0 GPU setup if issues occur.

---

### Risk 3: Students Skip Physics Theory
**Impact**: Students copy-paste code without understanding inertia, friction, etc.

**Likelihood**: High (common in coding tutorials).

**Mitigation**:
- Add comprehension checkpoints: "Before proceeding, can you explain what inertia tensor represents?"
- Include exercise: "Calculate inertia for a 2kg box (0.1m x 0.1m x 0.3m)."

---

## Open Questions

1. **Q**: Should we cover Isaac Sim (NVIDIA Omniverse)?
   **A**: No — too advanced, requires RTX 3080+, complex setup. Mention as "advanced alternative."

2. **Q**: Should we teach SDF from scratch or always start with URDF?
   **A**: Start with URDF (students already know it), then show SDF conversion. Teach SDF syntax for worlds only.

3. **Q**: Should we include a video of sensor simulation?
   **A**: Yes if possible — animated GIF or YouTube embed showing LiDAR point cloud rotating.

4. **Q**: Should we cover Gazebo Garden (newer version)?
   **A**: No — stick with Fortress for stability. Garden has breaking changes.

---

## Definition of Done

Module 2 is complete when:

1. ✅ All 3 markdown files written (File 1: Intro, File 2: Gazebo, File 3: Sensors)
2. ✅ All code snippets are syntactically correct and tested in Gazebo Fortress
3. ✅ All diagrams created (digital twin workflow, Gazebo vs Unity decision tree)
4. ✅ All functional requirements (FR-001 to FR-012) implemented
5. ✅ Sidebar configuration updated (`web/sidebars.ts`)
6. ✅ Markdown linting passes (no broken links, proper formatting)
7. ✅ 3 beta testers complete all tutorials and pass success criteria (SC-001 to SC-007)
8. ✅ Files published to `web/docs/en/module-2-digital-twin/`
9. ✅ Build succeeds: `npm run build` in `web/` directory
10. ✅ PHR created documenting implementation

---

## Related Documentation

- **Module 1 Spec**: `specs/1-nervous-system-ros2/spec.md` (URDF prerequisite)
- **Module 0 Setup**: Hardware and software prerequisites
- **Gazebo Docs**: https://gazebosim.org/docs/fortress
- **ROS 2 Gazebo Integration**: https://github.com/ros-simulation/gazebo_ros_pkgs

---

## Appendix: File Structure Preview

```
web/docs/en/module-2-digital-twin/
├── intro.md (existing overview)
├── 01-why-simulate.md (Digital Twin concept) [NEW]
├── 02-gazebo-worlds.md (SDF, physics, world building) [NEW]
├── 03-sensors.md (Camera, LiDAR, IMU simulation) [NEW]
└── assets/
    ├── digital-twin-workflow.png (optional)
    └── gazebo-vs-unity.png (optional)
```

**Total Estimated Word Count**: 9,000-12,000 words
- File 1: 2,000-2,500 words (concepts)
- File 2: 3,500-4,500 words (Gazebo deep dive)
- File 3: 3,500-4,500 words (sensor tutorials)

**Estimated Student Time**: 5-7 hours (2 hours reading, 3-5 hours hands-on)
