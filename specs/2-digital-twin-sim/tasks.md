# Task Breakdown: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature ID**: 2-digital-twin-sim
**Branch**: `2-digital-twin-sim`
**Status**: Ready for Implementation
**Created**: 2025-12-04
**Related**: [spec.md](./spec.md) | [plan.md](./plan.md)

---

## Overview

**Total Tasks**: 125 tasks (T001-T125)
**Organized By**: 10 implementation phases aligned with user stories
**MVP Scope**: US1 + US2 + US3 (85 tasks, P1 stories)
**Optional**: US4 (Unity awareness, P2)

**Parallel Opportunities**: 42 tasks marked with [P]

---

## Phase 1: Setup and Environment (T001-T003)

**Objective**: Initialize directory structure and verify prerequisites.

**Tasks**:

- [ ] T001 [P] Create directory `web/docs/en/module-2-digital-twin/` if it doesn't exist
- [ ] T002 [P] Create subdirectory `web/docs/en/module-2-digital-twin/assets/` for diagrams and screenshots
- [ ] T003 [P] Initialize git tracking for Module 2 files

**Checkpoint**: Directory structure ready for content creation.

---

## Phase 2: Research & Asset Preparation (T004-T020) - CRITICAL BLOCKING PHASE

**Objective**: Test all Gazebo Fortress plugins, verify ROS 2 Humble integration, create diagrams and screenshots before writing tutorials.

**Why Critical**: All code in tutorials must be verified working before documentation. Prevents tutorial bugs.

### Gazebo Fortress Verification (T004-T009)

- [ ] T004 [P] Verify Gazebo Fortress installation: Run `gz sim --version` (expect 7.x)
- [ ] T005 [P] Test ros_gz bridge: Install `ros-humble-ros-gz` and verify `ros2 pkg list | grep ros_gz`
- [ ] T006 [P] Create test world `test_arena.sdf` with ground plane, verify launches with `gz sim test_arena.sdf`
- [ ] T007 Test robot spawning: Spawn simple_arm.urdf from Module 1 in test world via `ros2 run ros_gz_sim create`
- [ ] T008 Test physics: Add dynamic box to test_arena.sdf, verify gravity and collisions work
- [ ] T009 Test robot control: Publish to `/model/robot/cmd_vel`, verify robot moves in simulation

### Sensor Plugin Verification (T010-T014)

- [ ] T010 [P] Test depth camera plugin: Add depth camera to URDF, verify `/camera/image_raw` topic publishes
- [ ] T011 [P] Test LiDAR plugin: Add LiDAR sensor to URDF, verify `/scan` topic publishes LaserScan messages
- [ ] T012 [P] Test IMU plugin: Add IMU sensor to URDF, verify `/imu/data` topic publishes with orientation quaternion
- [ ] T013 Verify RViz2 visualization: Launch RViz2, add Image display for camera, LaserScan for LiDAR, confirm data visible
- [ ] T014 Test Python subscribers: Write test Python script to subscribe to `/scan`, verify messages received and parsed

### Unity Verification (T015-T017)

- [ ] T015 [P] Install Unity Hub and Unity 2021.3 LTS on Ubuntu 22.04 (or Windows if available)
- [ ] T016 Test ROS-TCP-Endpoint: Install `ros-humble-ros-tcp-endpoint`, launch on port 10000
- [ ] T017 Test Unity connection: Create minimal Unity project, add ROS-TCP-Connector package, verify connection to endpoint

### Diagram and Screenshot Creation (T018-T020)

- [ ] T018 [P] Create Mermaid diagram: Digital twin workflow (Design→Simulate→Validate→Deploy cycle)
- [ ] T019 [P] Take screenshot: Gazebo Fortress with robot_arena.sdf loaded showing obstacles
- [ ] T020 [P] Take screenshot: RViz2 with depth camera image, LiDAR point cloud, and TF frames visible

**Checkpoint**: All code verified working, all assets created. Ready to write tutorials with confidence.

---

## Phase 3: User Story 1 - Digital Twin Concept (T021-T040) - P1 MVP

**User Story**: US1 - Digital Twin Concept
**Priority**: P1 (MVP)
**File**: `web/docs/en/module-2-digital-twin/01-intro-digital-twin.md`
**Word Count**: 2,500-3,000 words

**Acceptance Criteria** (from spec SC-003, SC-006):
- Student can explain Gazebo vs Unity tradeoffs in under 3 minutes
- Student can explain why NVIDIA RTX GPU is required (ray tracing)

### Section 1: What is a Digital Twin? (T021-T023)

- [ ] T021 [US1] Write Section 1 introduction: Define "digital twin" (virtual replica synchronized with physical counterpart)
- [ ] T022 [US1] Add "dress rehearsal" and "flight simulator" analogies for digital twin concept
- [ ] T023 [US1] Write real-world examples subsection: Boston Dynamics (Spot 100k+ tests), NASA (Mars rovers), Waymo (billions of miles)

### Section 2: Sim-to-Real Gap (T024-T025)

- [ ] T024 [US1] Write Section 2: Define sim-to-real gap (discrepancies between simulation and reality)
- [ ] T025 [US1] Add subsections on common gaps (physics friction, sensor noise, actuator response) and mitigation strategies

### Section 3: Gazebo vs Unity Comparison (T026-T028)

- [ ] T026 [US1] Create comparison table: Physics Accuracy, Visual Realism, ROS 2 Integration, Sensor Simulation, Learning Curve, Use Cases, Cost
- [ ] T027 [US1] Write decision guide subsection: "Use Gazebo if..." (control algorithms, navigation) and "Use Unity if..." (perception, CV training)
- [ ] T028 [US1] Add "Use Both" workflow: Gazebo for control validation → Unity for perception training

### Section 4: Hardware Requirements (T029-T032)

- [ ] T029 [US1] Add `:::danger` callout box: "⚠️ NVIDIA RTX GPU REQUIRED FOR SENSOR SIMULATION" with minimum (RTX 4060) and recommended (RTX 4070 Ti) specs
- [ ] T030 [US1] Write subsection: Why RTX is Required (RT cores, 10-100x speedup, realistic depth maps/LiDAR point clouds)
- [ ] T031 [US1] Add performance comparison table: RTX 4070 Ti (300 FPS LiDAR) vs GTX 1080 (30 FPS) vs CPU (2 FPS)
- [ ] T032 [US1] Write budget alternatives subsection: Used RTX 3060 (~$200), AWS EC2 g5.xlarge (~$1/hour), university labs

### Section 5: What's Next (T033-T034)

- [ ] T033 [US1] Write Module 2 roadmap: File 2 (Gazebo worlds), File 3 (sensors), File 4 (Unity viz)
- [ ] T034 [US1] Add prerequisites check section: Module 0/1 completed, GPU drivers verified (`nvidia-smi`)

### Diagrams and Finalization (T035-T040)

- [ ] T035 [US1] Embed Mermaid diagram (from T018): Digital twin workflow in Section 1
- [ ] T036 [US1] Create Mermaid decision tree diagram: "Should I use Gazebo or Unity?" flowchart in Section 3
- [ ] T037 [US1] Add frontmatter: id, title, sidebar_label, sidebar_position, description, keywords
- [ ] T038 [US1] Run markdown linter on 01-intro-digital-twin.md, fix any formatting issues
- [ ] T039 [US1] Verify all internal links (to File 2, File 3, File 4) use correct relative paths
- [ ] T040 [US1] Word count check: Verify file is 2,500-3,000 words

**Checkpoint**: File 1 complete (2,500-3,000 words). Student understands digital twin concept, Gazebo vs Unity, GPU requirements.

---

## Phase 4: User Story 2 - Gazebo World Building (T041-T070) - P1 MVP

**User Story**: US2 - Gazebo World Building
**Priority**: P1 (MVP)
**File**: `web/docs/en/module-2-digital-twin/02-gazebo-fortress-setup.md`
**Word Count**: 3,500-4,000 words

**Acceptance Criteria** (from spec SC-001, SC-005):
- Student can create custom Gazebo world in under 15 minutes (100% completion, avg <12 min)
- Student can correctly calculate inertia tensor for a cylinder

### Section 1: Installation and Verification (T041-T044)

- [ ] T041 [US2] Write installation instructions: `sudo apt install ros-humble-ros-gz`
- [ ] T042 [US2] Add verification steps: `gz sim --version` (expect 7.x), `ros2 pkg list | grep ros_gz`
- [ ] T043 [US2] Write test empty world section: `gz sim empty.sdf` (should open GUI with gray void)
- [ ] T044 [US2] Add troubleshooting subsection: "gz sim not found" (PATH), GPU errors (`nvidia-smi`), blank screen (`glxinfo`)

### Section 2: SDF Syntax Deep Dive (T045-T049)

- [ ] T045 [US2] Create SDF vs URDF comparison table: Purpose, Scope, Physics, Sensors, Lighting, Worlds
- [ ] T046 [US2] Write SDF file structure explanation with annotated example: `<sdf>`, `<world>`, `<physics>`, `<light>`, `<model>`
- [ ] T047 [US2] Explain `<physics>` tag parameters: max_step_size (0.001s), real_time_factor (1.0), solver type (ODE)
- [ ] T048 [US2] Explain `<light>` tag: directional sun light with pose, diffuse color, direction vector
- [ ] T049 [US2] Explain `<model>` tag structure: static vs dynamic models, `<link>`, `<collision>`, `<visual>`

### Section 3: Physics Engine Configuration (T050-T054)

- [ ] T050 [US2] Write physics parameters explained subsection: Gravity (-9.81 Z-axis), Time Step (1ms), Solver iterations
- [ ] T051 [US2] Write rigid body dynamics primer: Mass (kg), Inertia tensor (3x3 matrix), Friction (mu/mu2), Damping
- [ ] T052 [US2] Add inertia tensor formulas: Cylinder `Ixx = 1/12 * m * (3r² + h²)`, Box `Ixx = 1/12 * m * (h² + d²)`
- [ ] T053 [US2] Write "When Physics Matters" subsection: Control algorithms (PID tuning), Manipulation (grasping), Navigation (wheel slip)
- [ ] T054 [US2] Add exercise: Calculate inertia for cylinder (r=0.05m, h=0.3m, m=1.0kg) with solution

### Section 4: Building First World - robot_arena.sdf (T055-T061)

- [ ] T055 [US2] Write tutorial introduction: "Create robot_arena.sdf with ground plane, obstacles, lighting"
- [ ] T056 [US2] Add complete SDF code block (80 lines): `<world name="robot_arena">`, physics (1ms ODE), sun lighting
- [ ] T057 [US2] Add ground plane model with texture and friction: `<plane>` geometry, `<surface>` with mu=0.8
- [ ] T058 [US2] Add Obstacle 1: Red box at (2,0,0.25) with mass=1.0kg, inertia calculated, collision and visual
- [ ] T059 [US2] Add Obstacle 2: Blue cylinder at (-2,2,0.5) with mass=2.0kg, inertia for cylinder, collision and visual
- [ ] T060 [US2] Add launch instructions: `gz sim robot_arena.sdf`, expected result description
- [ ] T061 [US2] Add screenshot (from T019): Show Gazebo GUI with robot_arena loaded

### Section 5: Spawning Robots (T062-T065)

- [ ] T062 [US2] Write Method 1: Include robot in SDF world file with `<include>` tag, URI, and pose
- [ ] T063 [US2] Write Method 2: Spawn via ROS 2 - Terminal 1 (`gz sim robot_arena.sdf`), Terminal 2 (`ros2 run ros_gz_sim create -file robot.urdf`)
- [ ] T064 [US2] Add example: Spawn simple_arm.urdf from Module 1 with command and expected result
- [ ] T065 [US2] Add troubleshooting: Robot not spawning (check URDF paths, validate with `check_urdf robot.urdf`)

### Section 6: Controlling Robots via ROS 2 (T066-T070)

- [ ] T066 [US2] Write differential drive plugin section: Add `<gazebo>` block with `gz_ros2_control` plugin to URDF
- [ ] T067 [US2] Write simple velocity control section: Publish Twist to `/model/my_robot/cmd_vel` with example command
- [ ] T068 [US2] Add code block: `ros2 topic pub /model/my_robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"`
- [ ] T069 [US2] Add verification section: Check robot pose with `gz topic -e -t /model/my_robot/pose`
- [ ] T070 [US2] Finalize File 2: Frontmatter, markdown linting, word count check (3,500-4,000 words), link verification

**Checkpoint**: File 2 complete (3,500-4,000 words). Student can create Gazebo worlds, understand SDF syntax, spawn and control robots.

---

## Phase 5: User Story 3 - Sensor Simulation (T071-T105) - P1 MVP

**User Story**: US3 - Sensor Simulation
**Priority**: P1 (MVP)
**File**: `web/docs/en/module-2-digital-twin/03-simulating-sensors.md`
**Word Count**: 4,000-4,500 words

**Acceptance Criteria** (from spec SC-002, SC-007):
- Student can add LiDAR sensor with zero errors (100% success on first attempt)
- Student can modify LiDAR parameters in under 10 minutes

### Section 0: GPU Requirement Banner (T071)

- [ ] T071 [US3] Add banner at top of file: `:::danger` callout - "⚠️ NVIDIA RTX GPU REQUIRED FOR THIS TUTORIAL" with FPS comparison and CPU fallback note

### Section 1: Sensor Simulation Architecture (T072-T073)

- [ ] T072 [US3] Write sensor simulation pipeline: Gazebo physics → Sensor plugin triggers → GPU ray tracing → Collision detection → ROS 2 message → Topic publish
- [ ] T073 [US3] Add "Why GPU Ray Tracing" subsection: LiDAR (2.3M rays/s), Depth camera (9.2M rays/s), RTX 4070 Ti (10B rays/s), CPU (10-100M rays/s)

### Section 2: Depth Camera Simulation (T074-T082)

- [ ] T074 [US3] Write depth camera introduction: Intel RealSense D435i equivalent simulation
- [ ] T075 [US3] Add complete URDF Gazebo block (60 lines): `<sensor name="depth_camera" type="depth_camera">` with 640×480, 30 Hz, 87° HFOV
- [ ] T076 [US3] Add camera parameters: `<horizontal_fov>1.51844</horizontal_fov>`, `<clip>` near/far, `<depth_camera>` output
- [ ] T077 [US3] Add Gazebo plugin: `libgazebo_ros_camera.so` with ROS namespace `/camera`, remapping for image_raw, depth/image_raw, camera_info
- [ ] T078 [US3] Add camera link definition: Visual (box 0.025×0.09×0.025), collision, inertial (mass 0.072kg)
- [ ] T079 [US3] Add camera joint: Fixed joint from base_link with origin (0.1, 0, 0.2)
- [ ] T080 [US3] Add optical frame: `camera_optical_frame` link with rotated joint (rpy="-1.5708 0 -1.5708")
- [ ] T081 [US3] Write RViz2 visualization section: Launch RViz2, set fixed frame to `camera_optical_frame`, add Image and DepthCloud displays
- [ ] T082 [US3] Add published topics list: `/camera/image_raw` (RGB), `/camera/depth/image_raw` (16-bit depth), `/camera/camera_info`

### Section 3: LiDAR Simulation (T083-T092)

- [ ] T083 [US3] Write LiDAR introduction: Velodyne VLP-16 equivalent (16 beams, 360° horizontal)
- [ ] T084 [US3] Add complete URDF Gazebo block (70 lines): `<sensor name="lidar" type="gpu_lidar">` with 1800 horizontal samples, 16 vertical beams
- [ ] T085 [US3] Add LiDAR parameters: `<horizontal>` (1800 samples, -π to +π), `<vertical>` (16 samples, -15° to +15°)
- [ ] T086 [US3] Add range and noise: `<range>` 0.1-100m, `<noise>` Gaussian (mean 0, stddev 0.01)
- [ ] T087 [US3] Add Gazebo plugin: `libgazebo_ros_ray_sensor.so` with ROS namespace `/lidar`, output_type `sensor_msgs/LaserScan`
- [ ] T088 [US3] Add LiDAR link definition: Visual (cylinder radius 0.05, length 0.07), collision, inertial (mass 0.83kg)
- [ ] T089 [US3] Add LiDAR joint: Fixed joint from base_link with origin (0, 0, 0.3)
- [ ] T090 [US3] Write RViz2 visualization section: Fixed frame `lidar_link`, add LaserScan display, adjust size/color
- [ ] T091 [US3] Add Python subscriber example (30 lines): `class LidarProcessor` subscribing to `/lidar/scan`, finding closest obstacle with distance and angle
- [ ] T092 [US3] Add published topic: `/lidar/scan` (sensor_msgs/LaserScan or PointCloud2)

### Section 4: IMU Simulation (T093-T099)

- [ ] T093 [US3] Write IMU introduction: Inertial Measurement Unit at robot center of mass
- [ ] T094 [US3] Add complete URDF Gazebo block (40 lines): `<sensor name="imu" type="imu">` with 100 Hz update rate
- [ ] T095 [US3] Add IMU noise parameters: `<angular_velocity>` Gaussian noise (stddev 0.01), `<linear_acceleration>` Gaussian noise (stddev 0.1)
- [ ] T096 [US3] Add Gazebo plugin: `libgazebo_ros_imu_sensor.so` with ROS namespace `/imu`, frame_name `imu_link`
- [ ] T097 [US3] Add IMU link definition: Inertial only (mass 0.01kg), fixed joint from base_link at (0, 0, 0.05)
- [ ] T098 [US3] Write IMU outputs explanation: orientation (quaternion), angular_velocity (rad/s), linear_acceleration (m/s² including gravity)
- [ ] T099 [US3] Add Python subscriber example (25 lines): `class ImuProcessor` subscribing to `/imu/data`, converting quaternion to Euler angles with `tf_transformations`

### Section 5: Performance Tuning (T100-T102)

- [ ] T100 [US3] Create performance table: GPU vs CPU for Depth Camera (640×480, 30Hz: 300 FPS vs 2 FPS), LiDAR (16 beams, 10Hz: 200 FPS vs 5 FPS)
- [ ] T101 [US3] Add CPU fallback settings: Reduced depth camera (320×240, 10Hz), reduced LiDAR (8 beams, 5Hz)
- [ ] T102 [US3] Add troubleshooting low FPS section: Check GPU usage (`nvidia-smi`), reduce resolution, lower update rates, close other GPU apps

### Section 6: Exercise (T103-T105)

- [ ] T103 [US3] Write exercise challenge: "Add a second LiDAR sensor to the back of the robot for 360° coverage"
- [ ] T104 [US3] Add hints: Create `rear_lidar_link` at origin (-0.15, 0, 0.3), copy LiDAR block with namespace `/rear_lidar`
- [ ] T105 [US3] Add solution in `<details>` block: Complete rear_lidar_link, rear_lidar_joint (rotated 180°), Gazebo sensor block with new namespace

### Finalization (T106)

- [ ] T106 [US3] Finalize File 3: Frontmatter, markdown linting, word count check (4,000-4,500 words), link verification, embed screenshot from T020

**Checkpoint**: File 3 complete (4,000-4,500 words). Student can add depth camera, LiDAR, IMU sensors with complete URDF code and visualize in RViz2.

---

## Phase 6: User Story 4 - Unity Visualization (T107-T115) - P2 OPTIONAL

**User Story**: US4 - Unity Integration Overview
**Priority**: P2 (Awareness-level, no mandatory hands-on)
**File**: `web/docs/en/module-2-digital-twin/04-unity-visualization.md`
**Word Count**: 2,500-3,000 words

**Acceptance Criteria**:
- Student understands Unity's role in robotics (awareness-level)
- Student can explain when to use Unity vs Gazebo

### Content Writing (T107-T114)

- [ ] T107 [US4] Write Section 1: Why Unity for Robotics? (Photorealistic rendering, domain randomization, synthetic data generation for CV)
- [ ] T108 [US4] Write Section 2: Installation (Unity Hub download, Unity 2021.3 LTS install, Linux Build Support module)
- [ ] T109 [US4] Write Section 3: ROS-TCP-Connector setup (Install ros-humble-ros-tcp-endpoint, launch on port 10000, configure Unity ROS Settings)
- [ ] T110 [US4] Add Unity connection test: Publish Test Message in Unity → echo on `/unity_test` topic in ROS 2
- [ ] T111 [US4] Write Section 4: Importing Robot Model (Export URDF meshes to FBX with Blender, import to Unity, add Rigidbody component)
- [ ] T112 [US4] Add Unity C# code example (30 lines): `JointStateSubscriber.cs` subscribing to `/joint_states`, updating Unity joint transforms
- [ ] T113 [US4] Write Section 5: Comparison (When to use Unity: synthetic data, demos; When to use Gazebo: control dev, navigation; Typical workflow)
- [ ] T114 [US4] Write Section 6: Next Steps (Advanced topics: Perception Camera, Randomizers, HDRP, Isaac Sim; links to Unity Robotics Hub docs)

### Finalization (T115)

- [ ] T115 [US4] Finalize File 4: Frontmatter, markdown linting, word count check (2,500-3,000 words), add note "Unity setup covered in advanced modules"

**Checkpoint**: File 4 complete (2,500-3,000 words). Awareness-level Unity integration covered.

---

## Phase 7: Sidebar Integration (T116-T118)

**Objective**: Integrate all 4 files into Docusaurus sidebar with correct navigation order.

- [ ] T116 Verify files exist in `web/docs/en/module-2-digital-twin/`: intro.md (existing), 01-intro-digital-twin.md, 02-gazebo-fortress-setup.md, 03-simulating-sensors.md, 04-unity-visualization.md
- [ ] T117 Test auto-generated sidebar: Run `npm start` in web/, verify Module 2 appears in sidebar with files in order (intro → 01 → 02 → 03 → 04)
- [ ] T118 Verify internal links: Click through all files, verify links between File 1→2, File 2→3, File 3→File 1 work correctly

**Checkpoint**: Module 2 fully integrated in documentation site with correct navigation.

---

## Phase 8: Verification and Testing (T119-T122)

**Objective**: End-to-end testing on clean Ubuntu 22.04 system to ensure all tutorials work.

- [ ] T119 Execute File 2 tutorial: Spin up clean Ubuntu 22.04 VM, install ROS 2 Humble + Gazebo Fortress, create robot_arena.sdf, verify world launches
- [ ] T120 Execute File 3 tutorial: Add depth camera to URDF, spawn robot, verify `/camera/image_raw` topic, visualize in RViz2
- [ ] T121 Execute File 3 tutorial: Add LiDAR to URDF, verify `/lidar/scan` topic, visualize in RViz2, run Python subscriber script
- [ ] T122 Build verification: Run `npm run build` in web/, verify no broken links, no Mermaid diagram errors, build succeeds with exit code 0

**Checkpoint**: All tutorials verified working on clean system.

---

## Phase 9: Beta Testing (T123-T124)

**Objective**: 3 students complete all tutorials, measure success criteria from spec.md.

- [ ] T123 Recruit 3 beta testers with Module 0 + Module 1 completed and NVIDIA RTX GPU (4060 or better)
- [ ] T124 Measure success criteria:
  - SC-001: Gazebo world creation time (target: <15 min, avg <12 min) - File 2 Section 4 tutorial
  - SC-002: LiDAR sensor addition success rate (target: 100% zero errors) - File 3 Section 3
  - SC-003: Gazebo vs Unity explanation (target: <3 min, mention physics vs visuals) - File 1 Section 3 oral test
  - SC-004: Self-contained learning (target: 80%+ no external help) - beta tester self-report
  - SC-005: Physics understanding (target: 100% correct inertia formula) - File 2 Section 3 quiz
  - SC-006: GPU awareness (target: 100% mention ray tracing) - File 3 Section 1 oral test
  - SC-007: LiDAR parameter modification (target: <10 min, 100% success) - File 3 Section 6 exercise

**Checkpoint**: All success criteria measured, feedback collected for revisions.

---

## Phase 10: Final Polish and Deployment (T125)

**Objective**: Final proofreading, production build, and deployment.

- [ ] T125 Final deployment: Proofreading pass, production build (`npm run build`), merge to main branch, deploy to GitHub Pages, create PHR documenting implementation

**Checkpoint**: Module 2 live in production.

---

## Dependencies

### Phase Dependencies (Sequential Execution Order)

**Blocking Phases**:
- **Phase 2 (Research) BLOCKS Phases 3-6**: Must test all Gazebo plugins and Unity setup before writing tutorials
- **Phase 7 (Sidebar) depends on Phases 3-6**: Need all 4 files created before sidebar integration
- **Phase 8 (Verification) depends on Phase 7**: Need integrated site for E2E testing
- **Phase 9 (Beta Testing) depends on Phase 8**: Need verified content before student testing
- **Phase 10 (Deployment) depends on Phase 9**: Need feedback incorporated before production

### User Story Dependencies

**Independent User Stories** (Can implement in parallel after Phase 2):
- **US1** (File 1): No dependencies (conceptual content)
- **US2** (File 2): No dependencies (Gazebo world building is self-contained)
- **US3** (File 3): Assumes student understands URDF from Module 1 (external dependency)
- **US4** (File 4): No dependencies (Unity awareness is self-contained)

**Within Each User Story** (Sequential task ordering):
- File 1: Sections 1-5 sequential → Diagrams → Finalization
- File 2: Sections 1-6 sequential (installation before world building before control)
- File 3: Section 0 (GPU banner) → Sections 1-5 (depth camera before LiDAR before IMU logical ordering)
- File 4: Sections 1-6 sequential (installation before setup before import)

---

## Parallel Execution Opportunities

**Phase 1** (3 tasks, all parallelizable):
- T001, T002, T003 can run simultaneously (different directory operations)

**Phase 2** (17 tasks, 13 parallelizable):
- **Parallel Block 1** (T004-T006): Gazebo Fortress verification, ros_gz installation, test world creation
- **Sequential** (T007-T009): Robot spawning depends on T006, physics test depends on T008, control test depends on T007
- **Parallel Block 2** (T010-T012): Depth camera, LiDAR, IMU plugin tests (independent)
- **Sequential** (T013-T014): RViz2 depends on T010-T012, Python subscriber depends on T011
- **Parallel Block 3** (T015-T017): Unity installation, ROS-TCP-Endpoint, Unity connection
- **Parallel Block 4** (T018-T020): All diagram and screenshot creation can run simultaneously

**Phases 3-6** (4 files, can be written in parallel after Phase 2):
- **US1** (File 1): Independent, can write in parallel with US2, US3, US4
- **US2** (File 2): Independent, can write in parallel with US1, US3, US4
- **US3** (File 3): Independent (assumes Module 1 knowledge externally), can write in parallel with US1, US2, US4
- **US4** (File 4): Independent, can write in parallel with US1, US2, US3

**Within File 3** (Sensors): Some tasks parallelizable:
- T074-T082 (Depth camera), T083-T092 (LiDAR), T093-T099 (IMU) are independent sensor sections - can write in parallel if multiple authors

---

## Implementation Strategies

### Strategy 1: MVP First (Single Author)
**Goal**: Deploy minimal viable content as fast as possible

**Execution**:
1. Phase 1 (Setup): 0.5 days
2. Phase 2 (Research): 1.5 days [BLOCKING]
3. Phase 3 (US1 - Concept): 1 day
4. Phase 4 (US2 - Gazebo): 1.5 days
5. **VALIDATE MVP**: Phases 1-4 complete = Students understand concepts + can build Gazebo worlds
6. Phase 5 (US3 - Sensors): 2 days
7. Phase 7 (Sidebar): 0.5 days
8. Phase 8 (Verification): 1 day
9. Phase 9 (Beta Testing): 1.5 days
10. Phase 10 (Deployment): 0.5 days

**Total MVP (US1-US3)**: ~10 days
**Optional (US4)**: +1 day

**Advantages**:
- Early validation after US1+US2 (4 days)
- Core sensor simulation (US3) deployed quickly
- Unity (US4) can be added later without blocking

---

### Strategy 2: Parallel Team (Multiple Authors)
**Goal**: Maximize speed with multiple authors working concurrently

**Team Assignment**:
- **Author A**: Phase 2 (Research & Asset Prep) - 1.5 days [BLOCKING ALL]
- **Author B**: Phase 3 (US1 - File 1 Concept) - 1 day [AFTER Phase 2]
- **Author C**: Phase 4 (US2 - File 2 Gazebo) - 1.5 days [AFTER Phase 2]
- **Author D**: Phase 5 (US3 - File 3 Sensors) - 2 days [AFTER Phase 2]
- **Author E**: Phase 6 (US4 - File 4 Unity) - 1 day [AFTER Phase 2]

**Execution**:
1. **Day 1-2**: Author A completes Phase 2 (Research) alone
2. **Day 3-4**: Authors B, C, D, E write Files 1-4 in parallel
3. **Day 5**: Phase 7 (Sidebar), Phase 8 (Verification)
4. **Day 6-7**: Phase 9 (Beta Testing)
5. **Day 7**: Phase 10 (Deployment)

**Total Time**: ~7 days
**Speedup**: 3 days faster than single author

**Advantages**:
- All 4 files completed simultaneously
- Beta testing starts earlier
- Faster time to production

---

### Strategy 3: Incremental Delivery
**Goal**: Deploy each user story independently as it's completed

**Execution**:
1. Phase 1 + Phase 2: 2 days [BLOCKING]
2. **Iteration 1**: Phase 3 (US1) → Deploy File 1 alone: 2 days total
3. **Iteration 2**: Phase 4 (US2) → Deploy Files 1+2: 3.5 days total
4. **Iteration 3**: Phase 5 (US3) → Deploy Files 1+2+3: 5.5 days total
5. **Iteration 4**: Phase 6 (US4) → Deploy Files 1+2+3+4: 6.5 days total
6. Phase 7-10: Integration, testing, deployment: 3.5 days
7. **Total**: ~10 days

**Advantages**:
- Students can start learning after each iteration
- Feedback from Iteration 1 improves Iterations 2-4
- Low risk (each increment tested independently)

---

## Success Criteria Mapping

**From spec.md - All tasks mapped to success criteria**:

| Success Criterion | Target | Measurement Tasks | File/Section |
|-------------------|--------|-------------------|--------------|
| **SC-001**: Gazebo world creation speed | <15 min (avg <12 min), 100% completion | T124 (beta testing) | File 2, Section 4 (T055-T061) |
| **SC-002**: Sensor simulation success rate | 100% zero errors on first attempt | T124 (beta testing) | File 3, Sections 2-4 (T074-T099) |
| **SC-003**: Gazebo vs Unity explanation | <3 min, mention physics vs visuals | T124 (beta testing) | File 1, Section 3 (T026-T028) |
| **SC-004**: Self-contained learning | 80%+ no external help | T124 (beta testing) | All files |
| **SC-005**: Physics understanding | 100% correct inertia formula | T124 (beta testing) | File 2, Section 3 (T050-T054) |
| **SC-006**: GPU awareness | 100% mention ray tracing | T124 (beta testing) | File 1 Section 4 (T029-T032), File 3 Section 0+1 (T071-T073) |
| **SC-007**: LiDAR parameter modification | <10 min, 100% success | T124 (beta testing) | File 3, Section 6 (T103-T105) |

---

## Definition of Done

Module 2 is complete when:

1. ✅ All 4 markdown files written and published to `web/docs/en/module-2-digital-twin/`
2. ✅ All code snippets tested in Gazebo Fortress + ROS 2 Humble on clean Ubuntu 22.04
3. ✅ All Mermaid diagrams render correctly in Docusaurus
4. ✅ All 12 functional requirements (FR-001 to FR-012 from spec.md) implemented
5. ✅ Sidebar navigation configured and tested
6. ✅ Markdown linting passes (no broken links, proper formatting)
7. ✅ 3 beta testers complete tutorials and pass all success criteria (SC-001 to SC-007)
8. ✅ Production build succeeds: `npm run build` with exit code 0
9. ✅ PHR created documenting implementation
10. ✅ All 125 tasks marked complete in this file

---

## Task Count Summary

**Total Tasks**: 125
- **Phase 1 (Setup)**: 3 tasks
- **Phase 2 (Research)**: 17 tasks [BLOCKING]
- **Phase 3 (US1 - Concept)**: 20 tasks
- **Phase 4 (US2 - Gazebo)**: 30 tasks
- **Phase 5 (US3 - Sensors)**: 35 tasks
- **Phase 6 (US4 - Unity)**: 9 tasks [OPTIONAL P2]
- **Phase 7 (Sidebar)**: 3 tasks
- **Phase 8 (Verification)**: 4 tasks
- **Phase 9 (Beta Testing)**: 2 tasks
- **Phase 10 (Deployment)**: 1 task

**MVP (P1 Stories)**: 85 tasks (Phases 1-5, 7-10)
**Optional (P2 Story)**: 9 tasks (Phase 6)

**Parallel Opportunities**: 42 tasks marked [P]

---

## Notes

- **Phase 2 is critical**: All Gazebo Fortress plugins must be tested and verified working before writing tutorials (prevents tutorial bugs)
- **GPU Requirements**: File 1 and File 3 emphasize NVIDIA RTX GPU requirement with clear warnings
- **Code Completeness**: All sensor URDF blocks provided in full (60-70 lines each) with complete parameters
- **Python Examples**: Both LiDAR and IMU subscribers include full working code (30 and 25 lines respectively)
- **Unity Optional**: File 4 (US4) is P2 and awareness-level only (no mandatory hands-on due to setup complexity)
- **Beta Testing**: Success criteria (SC-001 to SC-007) measured quantitatively with 3 students
- **Incremental Deployment**: Each user story is independently testable and can be deployed as completed

---

**Ready for Implementation**: ✅ YES
**Recommended Strategy**: MVP First (Single Author) or Parallel Team (Multiple Authors) depending on team size
