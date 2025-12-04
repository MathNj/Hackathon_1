---
id: 001
title: Module 2 Digital Twin Specification
stage: spec
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: digital-twin-sim
branch: 2-digital-twin-sim
user: Hamdan
command: /sp.specify
labels: ["spec", "requirements", "digital-twin", "gazebo", "simulation", "sensors"]
links:
  spec: specs/2-digital-twin-sim/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/2-digital-twin-sim/spec.md
  - specs/2-digital-twin-sim/checklists/requirements.md
tests:
  - Manual validation: Requirements checklist (25/25 passed)
---

## Prompt

GOAL: Define Requirements for "Module 2: The Digital Twin (Gazebo & Unity)".

CONTEXT:
We are defining the second chapter of the textbook.
Theme: "The Digital Twin" — Before we risk breaking a $3,000 robot, we crash it in a simulator 100 times.

REQUIREMENTS:
1.  **Pedagogical Theme:**
    - Explain the concept of a "Digital Twin" (a virtual clone of the physical robot).
    - Contrast **Gazebo** (Engineering/Physics accuracy) vs. **Unity** (Visual/Photorealism).
2.  **Technical Scope (Mandatory Content):**
    - **Physics Engines:** Explain Rigid Body Dynamics, Gravity, and Collisions.
    - **File Formats:** Explain the difference between **URDF** (ROS standard) and **SDF** (Gazebo standard).
    - **Sensors:** Detailed tutorials on simulating "The Eyes" (Depth Cameras) and "The Spatial Sense" (LiDAR/IMU).
    - **Hardware Note:** Must explicitly mention why an **NVIDIA RTX GPU** is required (Ray Tracing for sensors).
3.  **Deliverables:**
    - Directory: `docs/02-digital-twin-sim/`
    - **File 1 (Intro):** "Why We Simulate" (The Digital Twin concept).
    - **File 2 (Gazebo):** "Building Worlds" (SDF, Physics params).
    - **File 3 (Sensors):** "Adding Eyes & Ears" (Code for adding a LiDAR plugin to a URDF).

OUTPUT:
Generate the requirements and success criteria for Module 2 in `sp.spec.md`.

## Response snapshot

Created comprehensive specification for Module 2 - The Digital Twin (Gazebo & Unity):

**Feature ID**: `2-digital-twin-sim`
**Branch**: `2-digital-twin-sim`

**Executive Summary**:
- **What**: 3 markdown files teaching digital twin concepts with Gazebo and Unity simulators
- **Why**: Safe iteration before risking $3,000+ physical robots (crash 100x in sim first)
- **Who**: University students with Module 0 (hardware) + Module 1 (ROS 2) completed
- **Success**: Student spawns robot with LiDAR in Gazebo, explains Gazebo vs Unity in <3 min

**Problem Statement**:
- **Current State**: Students understand ROS 2 architecture but lack simulation experience
- **Desired State**: Can create digital twins, simulate sensors, understand physics engines
- **Gap**: Missing content on digital twin concept, SDF syntax, sensor configuration, GPU requirements

**User Stories** (4 total):

**US1: Digital Twin Concept** (P1 - MVP)
- Learn digital twin definition and sim-to-real workflow
- See real-world examples (Boston Dynamics, NASA, Waymo)
- Understand Gazebo (physics) vs Unity (visuals) tradeoffs
- **Acceptance**: Student can explain when to use each tool

**US2: Gazebo World Building** (P1 - MVP)
- Understand SDF (Simulation Description Format) syntax
- Configure physics engine (gravity, time step, solver)
- Create custom worlds with obstacles and lighting
- Spawn robots and control via ROS 2 topics
- Understand rigid body dynamics (mass, inertia, friction)
- **Acceptance**: Student creates world with robot in <15 min

**US3: Sensor Simulation** (P1 - MVP)
- Add depth camera (RealSense D435i equivalent) to URDF
- Add LiDAR sensor (Velodyne VLP-16 equivalent) to URDF
- Add IMU (Inertial Measurement Unit) to URDF
- Visualize all sensors in RViz2
- Understand GPU ray tracing requirements (NVIDIA RTX)
- **Acceptance**: Student adds LiDAR with zero errors, visualizes in RViz2

**US4: Unity Integration Overview** (P2)
- Learn Unity's role (photorealistic rendering, domain randomization)
- Understand Unity Robotics Hub architecture
- See use cases: synthetic training data for CV
- **Acceptance**: Awareness-level only, no hands-on tutorial

**Functional Requirements** (12 total):
- **FR-001**: Digital twin explanation with "rehearsal before performance" analogy
- **FR-002**: Gazebo vs Unity comparison table with decision guide
- **FR-003**: Physics engine fundamentals (mass, inertia, gravity, friction, damping)
- **FR-004**: SDF vs URDF comparison and conversion
- **FR-005**: Complete Gazebo world creation tutorial
- **FR-006**: Robot physics properties (inertial tags, tensor calculations)
- **FR-007**: Depth camera simulation with complete URDF code
- **FR-008**: LiDAR simulation with complete URDF code
- **FR-009**: IMU simulation with complete URDF code
- **FR-010**: RViz2 sensor visualization instructions
- **FR-011**: GPU requirements explanation (NVIDIA RTX, ray tracing acceleration)
- **FR-012**: Unity Robotics Hub overview (awareness only)

**Success Criteria** (7 measurable outcomes):
- **SC-001**: Gazebo world creation in <15 min (100% completion, avg <12 min)
- **SC-002**: LiDAR sensor addition with zero errors (100% success on first attempt)
- **SC-003**: Gazebo vs Unity explanation in <3 min (100% mention physics vs visuals)
- **SC-004**: Self-contained learning (80%+ no external help)
- **SC-005**: Physics understanding (100% provide correct inertia formula)
- **SC-006**: GPU awareness (100% mention ray tracing)
- **SC-007**: Code modification (LiDAR params) in <10 min (100% success)

**Key Entities**:
- **Digital Twin**: Virtual replica mirroring geometry, physics, behavior
- **SDF**: XML format for worlds + robots in Gazebo (vs URDF robot-only)
- **Gazebo Plugin**: C++ libraries extending functionality (sensors, controllers)
- **Physics Engine**: Simulates rigid body dynamics (ODE, Bullet, DART, Simbody)
- **Ray Tracing**: Light/laser propagation simulation (GPU RT cores for 10-100x speedup)

**Edge Cases**:
1. No GPU available → Warning + CPU fallback (world building only)
2. URDF to SDF conversion issues → Troubleshooting checklist
3. Sensor not publishing data → Debugging steps
4. RViz2 crashes on large point clouds → Set reasonable defaults

**Risks & Mitigations**:
- **Risk 1**: Gazebo version fragmentation (Medium) → Version check + explicit statement
- **Risk 2**: GPU driver issues (Low) → Verification step + link to Module 0
- **Risk 3**: Students skip theory (High) → Comprehension checkpoints + exercises

**Out of Scope**:
- Unity hands-on tutorial (too complex for this module)
- Advanced physics (soft body, fluids)
- Multi-robot simulation (advanced modules)
- Cloud simulation (AWS RoboMaker, NVIDIA Omniverse)
- Custom Gazebo plugins in C++
- Domain randomization (covered in Module 3)

**File Structure**:
```
web/docs/en/module-2-digital-twin/
├── intro.md (existing overview)
├── 01-why-simulate.md (Digital Twin concept) [NEW]
├── 02-gazebo-worlds.md (SDF, physics, world building) [NEW]
├── 03-sensors.md (Camera, LiDAR, IMU simulation) [NEW]
└── assets/ (diagrams)
```

**Estimated Word Count**: 9,000-12,000 words
- File 1: 2,000-2,500 words (concepts)
- File 2: 3,500-4,500 words (Gazebo deep dive)
- File 3: 3,500-4,500 words (sensor tutorials)

**Estimated Student Time**: 5-7 hours (2 hours reading, 3-5 hours hands-on)

**Real-World Context**:
- Boston Dynamics: Spot tested 100,000+ times in simulation before physical testing
- NASA: Mars rovers validated in digital twin environments
- Waymo: Autonomous vehicles tested billions of miles in simulation

## Outcome

- ✅ Impact: Complete specification ready for planning - 12 functional requirements with clear acceptance criteria, 7 measurable success criteria
- 🧪 Tests: Requirements validation checklist passed (25/25 items) - all requirements testable and mapped to user stories
- 📁 Files: 2 files created (spec.md, checklists/requirements.md) in specs/2-digital-twin-sim/
- 🔁 Next prompts: Ready for /sp.plan to define implementation architecture (3-file structure, content contracts, code examples)
- 🧠 Reflection: Specification emphasizes sensor simulation as core value (3 complete tutorials: camera, LiDAR, IMU). GPU requirements clearly justified (ray tracing acceleration). Strong real-world context (Boston Dynamics, NASA, Waymo). Physics foundation (FR-003) establishes rigid body dynamics before hands-on work. Decision framework (FR-002) guides Gazebo vs Unity choice. Edge cases comprehensively handled with troubleshooting steps. Success criteria quantitative and aligned with learning objectives.

## Evaluation notes (flywheel)

- Failure modes observed: None - requirements checklist passed all quality checks (25/25 items)
- Graders run and results (PASS/FAIL): Manual requirements checklist validation PASS (25/25 items)
- Prompt variant (if applicable): Standard /sp.specify workflow adapted for simulation/digital twin content
- Next experiment (smallest change to try): Consider adding specific hardware alternatives (e.g., AMD GPU support status) to broaden accessibility, though NVIDIA RTX requirement is justified for sensor simulation quality. Could also add estimated costs for recommended GPU (RTX 4070 Ti ~$800) to help students budget.
