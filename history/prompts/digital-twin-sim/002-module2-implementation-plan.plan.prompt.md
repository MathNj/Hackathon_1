---
id: 002
title: Module 2 Implementation Plan
stage: plan
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: digital-twin-sim
branch: 2-digital-twin-sim
user: Hamdan
command: /sp.plan
labels: ["plan", "architecture", "digital-twin", "gazebo-fortress", "unity", "sensors"]
links:
  spec: specs/2-digital-twin-sim/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/2-digital-twin-sim/plan.md
  - specs/2-digital-twin-sim/checklists/planning.md
tests:
  - Manual validation: Planning checklist (31/31 passed)
---

## Prompt

GOAL: Architect the content structure for "Module 2: The Digital Twin".

CONTEXT:
We have defined the requirements and clarified that we are using **Gazebo Fortress** (not Classic) and the **Unity Robotics Hub**.
We need to map out the chapters for this module.

PLANNING DECISIONS:
1.  **Directory:** `docs/02-digital-twin-sim/`
2.  **File 1: `01-intro-digital-twin.md`**
    - Concept: Define "Digital Twin" and "Sim-to-Real Gap".
    - Hardware Note: Explicitly warn users that NVIDIA RTX GPUs are required for the following tutorials.
3.  **File 2: `02-gazebo-fortress-setup.md`**
    - Tutorial: Installing `ros-humble-ros-gz` (The Bridge).
    - Task: Launching an empty world with physics enabled (Gravity/Collision).
4.  **File 3: `03-simulating-sensors.md`**
    - Concept: How Ray-Tracing mimics LiDAR.
    - Code: Adding a `<sensor>` block (LiDAR & Depth Camera) to a robot's URDF/SDF.
5.  **File 4: `04-unity-visualization.md`**
    - Tutorial: Setting up a Unity project for high-fidelity rendering.
    - Bridge: Basic setup of the ROS-TCP-Connector.
6.  **Navigation:**
    - Ensure these files are ordered correctly (01, 02, 03, 04) so Docusaurus sidebar picks them up automatically.

OUTPUT:
Generate `sp.plan.md` detailing this chapter outline, file paths, and the key technical concepts for each file.

## Response snapshot

Created comprehensive implementation plan for Module 2 - The Digital Twin (Gazebo & Unity):

**Feature ID**: `2-digital-twin-sim`
**Branch**: `2-digital-twin-sim`

**Architecture Decisions** (4 critical decisions):

**Decision 1: Gazebo Version**
- **Choice**: Gazebo Fortress (gz-sim7), NOT Gazebo Classic (gazebo11)
- **Rationale**: ROS 2 Humble official support, industry future alignment, better GPU ray tracing
- **Impact**: Commands use `gz sim`, install `ros-humble-ros-gz`, new plugin naming convention

**Decision 2: Unity Integration Method**
- **Choice**: ROS-TCP-Connector (Unity Robotics Hub)
- **Rationale**: Stable with ROS 2 Humble, extensive docs, proven community support
- **Impact**: TCP bridge on port 10000, Unity 2021.3 LTS, simpler than Unity 6 Package

**Decision 3: CPU Fallback Strategy**
- **Choice**: Provide "Low Poly Mode" guidance for CPU-only machines
- **Rationale**: Not all students have RTX GPU initially, allow learning concepts on CPU
- **Impact**: File 1 GPU warning, File 2 marked "CPU-compatible", File 3 requires RTX with reduced settings

**Decision 4: URDF vs SDF Workflow**
- **Choice**: Start with URDF, provide conversion script, teach SDF for worlds only
- **Rationale**: Students know URDF from Module 1, Gazebo loads URDF directly
- **Impact**: SDF syntax for world files only, sensors added to URDF (auto-converted)

**File Structure** (4 files, 12,000-14,000 words):

**File 1: `01-intro-digital-twin.md`** (2,500-3,000 words)
- **Purpose**: Conceptual foundation - digital twin, sim-to-real gap, Gazebo vs Unity, GPU requirements
- **5 Sections**:
  1. What is a Digital Twin? (500 words) - Definition, workflow diagram, real-world examples
  2. Sim-to-Real Gap (400 words) - Discrepancies, mitigation strategies
  3. Gazebo vs Unity Decision Matrix (600 words) - Comparison table, decision guide
  4. Hardware Requirements (700 words) - GPU warning, RTX justification, system specs
  5. What's Next (300 words) - Module roadmap, prerequisites check
- **Diagrams**: Digital twin workflow (Mermaid), Sim-to-real gap, Gazebo vs Unity decision tree
- **Code**: None (conceptual)
- **Acceptance**: Student explains Gazebo vs Unity in <3 min, recognizes GPU requirement

**File 2: `02-gazebo-fortress-setup.md`** (3,500-4,000 words)
- **Purpose**: Install Gazebo Fortress, teach SDF syntax, demonstrate physics simulation
- **6 Sections**:
  1. Installation and Verification (600 words) - Install ros_gz, verify with `gz sim --version`
  2. SDF Syntax Deep Dive (800 words) - SDF vs URDF comparison, file structure
  3. Physics Engine Configuration (700 words) - Gravity, time step, rigid body dynamics
  4. Building First World (1,000 words) - Complete `robot_arena.sdf` tutorial
  5. Spawning Robots (600 words) - Static (SDF include) and dynamic (ROS 2 spawn)
  6. Controlling Robots (800 words) - Diff drive plugin, velocity commands
- **Code**: Complete `robot_arena.sdf` (80 lines), spawn commands, velocity control
- **Diagrams**: SDF structure tree, physics engine pipeline
- **Acceptance**: Student creates world in <15 min (100% completion)

**File 3: `03-simulating-sensors.md`** (4,000-4,500 words)
- **Purpose**: Add depth cameras, LiDAR, IMU sensors. Explain GPU ray tracing. Visualize in RViz2.
- **⚠️ GPU Required Banner** at top
- **6 Sections**:
  1. Sensor Simulation Architecture (600 words) - Plugin→ray tracing→ROS 2 bridge pipeline
  2. Depth Camera Simulation (1,000 words) - Intel RealSense D435i equivalent URDF, RViz2 viz
  3. LiDAR Simulation (1,200 words) - Velodyne VLP-16 equivalent, Python subscriber
  4. IMU Simulation (800 words) - IMU URDF, quaternion to Euler conversion
  5. Performance Tuning (600 words) - GPU vs CPU benchmarks, CPU fallback settings
  6. Exercise (300 words) - Add second LiDAR for 360° coverage
- **Code**: Depth camera URDF (60 lines), LiDAR URDF (70 lines), IMU URDF (40 lines), Python subscribers (30+25 lines)
- **Diagrams**: Sensor simulation pipeline, RViz2 screenshot
- **Acceptance**: LiDAR addition with zero errors, param modification in <10 min

**File 4: `04-unity-visualization.md`** (2,500-3,000 words)
- **Purpose**: Unity awareness guide - setup, ROS-TCP-Connector, when to use Unity vs Gazebo
- **6 Sections**:
  1. Why Unity for Robotics? (500 words) - Photorealistic rendering, domain randomization, use cases
  2. Installation (700 words) - Unity Hub, Unity 2021.3 LTS, Robotics Hub package
  3. ROS-TCP-Connector Setup (800 words) - Start endpoint, configure Unity, test connection
  4. Importing Robot Model (600 words) - Export meshes, import FBX, subscribe to /joint_states
  5. Comparison: Gazebo vs Unity (400 words) - When to use each, typical workflow
  6. Next Steps (500 words) - Advanced topics (Perception Camera, Randomizers, Isaac Sim)
- **Code**: Unity C# joint state subscriber (30 lines)
- **Diagrams**: ROS-TCP-Connector architecture, Unity screenshot
- **Acceptance**: Awareness-level understanding (no hands-on requirement)

**Implementation Phases** (8 phases, 10-11 days single author):

**Phase 0: Research & Asset Preparation** (1.5 days) - BLOCKING
- Test all Gazebo Fortress plugins (depth camera, LiDAR, IMU) on Ubuntu 22.04 + ROS 2 Humble
- Create `robot_arena.sdf` and verify physics
- Install Unity 2021.3 LTS and test ROS-TCP-Connector
- Create Mermaid diagrams, take screenshots
- **Checkpoint**: All code tested, all assets ready

**Phase 1: File 1 - Digital Twin Intro** (1 day)
- Write 5 sections, add Mermaid diagrams, markdown linting
- **Checkpoint**: Conceptual foundation established

**Phase 2: File 2 - Gazebo Setup** (1.5 days)
- Write 6 sections, test all commands on clean VM, add code examples
- **Checkpoint**: Students can create Gazebo worlds

**Phase 3: File 3 - Sensor Simulation** (2 days)
- GPU banner, 5 sensor sections, test plugins, verify RViz2, Python scripts
- **Checkpoint**: Students can simulate sensors

**Phase 4: File 4 - Unity Visualization** (1 day)
- Write 6 sections, Unity install, C# code example
- **Checkpoint**: Awareness-level Unity integration covered

**Phase 5: Sidebar Integration** (0.5 days)
- Verify auto-generated sidebar, test navigation, verify internal links

**Phase 6: Verification & Testing** (1 day)
- Clean Ubuntu 22.04 VM, execute all commands, verify RViz2, check links

**Phase 7: Beta Testing** (1.5 days)
- 3 students complete tutorials, measure SC-001 to SC-007, collect feedback

**Phase 8: Final Polish & Deployment** (0.5 days)
- Proofreading, production build, merge to main, GitHub Pages deployment

**Risk Mitigation**:
- **Risk 1**: Gazebo Fortress plugin names changed → Test all plugins before writing, provide fallback
- **Risk 2**: ROS-TCP-Connector breaking changes → Pin Unity 2021.3 LTS and ROS-TCP-Connector v0.7.0
- **Risk 3**: Students without RTX GPU → Clear warnings in File 1, CPU fallback settings in File 3
- **Risk 4**: Unity setup complexity → Keep File 4 awareness-only (no mandatory hands-on)

**Success Metrics** (from spec.md):
- SC-001: Gazebo world creation in <15 min (100% completion, avg <12 min)
- SC-002: LiDAR sensor addition with zero errors (100% success on first attempt)
- SC-003: Gazebo vs Unity explanation in <3 min (100% mention physics vs visuals)
- SC-004: Self-contained learning (80%+ no external help)
- SC-005: Physics understanding (100% correct inertia formula)
- SC-006: GPU awareness (100% mention ray tracing)
- SC-007: LiDAR parameter modification in <10 min (100% success)

**Complete Code Examples Provided**:
1. `robot_arena.sdf` - Complete Gazebo world (80 lines) with physics, lighting, obstacles
2. Depth camera URDF - Intel RealSense D435i equivalent (60 lines) with Gazebo plugin
3. LiDAR URDF - Velodyne VLP-16 equivalent (70 lines) with 16-beam configuration
4. IMU URDF - Full IMU sensor (40 lines) with noise parameters
5. Python LiDAR subscriber - Full subscriber (30 lines) with closest obstacle detection
6. Python IMU subscriber - Full subscriber (25 lines) with quaternion to Euler conversion
7. Unity C# subscriber - Joint state subscriber (30 lines) for robot visualization

**GPU Strategy** (Three-Tier Approach):
1. **File 1 Section 4**: Warning callout - "NVIDIA RTX 4060 minimum, 4070 Ti recommended"
2. **File 2**: Marked "CPU-compatible" (world building works on CPU)
3. **File 3 Section 0**: Banner - "⚠️ NVIDIA RTX GPU REQUIRED FOR THIS TUTORIAL"
4. **Performance Table**: RTX 4070 Ti (300 FPS) vs CPU (2 FPS) for depth camera

**Version Constraints**:
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Fortress (gz-sim7)
- Unity 2021.3 LTS
- ROS-TCP-Connector v0.7.0
- NVIDIA Driver 525+
- NVIDIA RTX 4060 minimum, RTX 4070 Ti recommended

## Outcome

- ✅ Impact: Complete implementation plan ready for task generation - 4 files, 12k-14k words, 8 phases with clear checkpoints
- 🧪 Tests: Planning validation checklist passed (31/31 items) - all architecture decisions documented, code examples complete
- 📁 Files: 2 files created (plan.md, checklists/planning.md) in specs/2-digital-twin-sim/
- 🔁 Next prompts: Ready for /sp.tasks to generate 90-120 atomic tasks across 8 implementation phases
- 🧠 Reflection: Plan expands original 3-file spec to 4 files for better separation of concerns (intro, Gazebo, sensors, Unity). All code examples provided in full (not pseudocode) to prevent implementation ambiguity. Three-tier GPU strategy balances accessibility (CPU fallback) with requirements (RTX for sensors). Strong emphasis on Phase 0 (research & asset prep) ensures all Gazebo plugins tested before writing tutorials. Unity kept as awareness-only (File 4) due to setup complexity. Version pinning prevents dependency hell (Gazebo Fortress, Unity 2021.3 LTS, ROS-TCP-Connector v0.7.0).

## Evaluation notes (flywheel)

- Failure modes observed: None - planning checklist passed all quality checks (31/31 items)
- Graders run and results (PASS/FAIL): Manual planning checklist validation PASS (31/31 items)
- Prompt variant (if applicable): Standard /sp.plan workflow adapted for simulation content with 4 files instead of original 3-file spec
- Next experiment (smallest change to try): Consider adding video/GIF creation phase (Phase 0.5) for sensor visualization - animated LiDAR point cloud rotation would significantly enhance learning. Could use `ros2 bag record` + `ffmpeg` to generate 10-second GIFs for File 3. Trade-off: adds 0.5 days to Phase 0 but greatly improves pedagogical clarity for sensor simulation concepts.
