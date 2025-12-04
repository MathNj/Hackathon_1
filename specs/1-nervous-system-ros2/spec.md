# Feature Specification: Module 1 - The Robotic Nervous System (ROS 2 Fundamentals)

**Feature Branch**: `1-nervous-system-ros2`
**Created**: 2025-12-04
**Status**: Draft
**Input**: "GOAL: Define Requirements for Module 1: The Robotic Nervous System. CONTEXT: We are building the first chapter of the Physical AI textbook. Based on the Project Constitution, this module focuses on ROS 2 (Robot Operating System). REQUIREMENTS: 1. Pedagogical Theme using Nervous System analogy (Nodes = Neurons, Topics = Nerves). 2. Technical Scope: ROS 2 Architecture (Nodes, Topics, Services), Python (rclpy), Hardware Context (Brain-to-Body bridge), URDF. 3. Deliverables: docs/01-nervous-system-ros2/ with 3+ markdown files, verified Talker/Listener node code."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Understanding ROS 2 Communication Architecture (Priority: P1)

A university student learning physical AI needs to understand how robots communicate internally before implementing any code. They must grasp the fundamental concepts of nodes, topics, and services through the "nervous system" analogy.

**Why this priority**: Foundation knowledge - without understanding the architecture, students cannot effectively implement or debug any ROS 2 code.

**Independent Test**: Student can explain the nervous system analogy (nodes as neurons, topics as nerves) and correctly identify which communication pattern (topic vs service) to use for different scenarios (e.g., continuous sensor data vs one-time requests).

**Acceptance Scenarios**:

1. **Given** a student has never used ROS 2, **When** they read the architecture section, **Then** they can draw a diagram showing nodes, topics, and services with the nervous system analogy labels
2. **Given** a scenario description (e.g., "sending continuous camera frames"), **When** student is asked to choose communication method, **Then** they correctly select topics over services and explain why
3. **Given** the concepts of publishers and subscribers, **When** student is shown a robot system diagram, **Then** they can identify which components should publish vs subscribe to specific topics

---

### User Story 2 - Implementing First Python ROS 2 Nodes (Priority: P1)

A student needs to write their first working ROS 2 code using Python (rclpy) by implementing a verified "Talker/Listener" node pair that demonstrates publish-subscribe communication.

**Why this priority**: First hands-on experience - students need immediate practical validation that they can create working ROS 2 programs.

**Independent Test**: Student can run the provided Talker/Listener code examples, see messages being sent/received in the terminal, and then modify the message content to verify their understanding.

**Acceptance Scenarios**:

1. **Given** the verified Talker node code, **When** student runs it in a terminal, **Then** they see messages being published at the specified rate (e.g., "Hello World: 1", "Hello World: 2")
2. **Given** both Talker and Listener nodes running, **When** student checks both terminal outputs, **Then** Listener displays the exact messages sent by Talker with minimal latency
3. **Given** the working example code, **When** student modifies the message content (e.g., change "Hello World" to "Robot Message"), **Then** the modified messages appear in the Listener output
4. **Given** the node implementation, **When** student runs `ros2 node list` and `ros2 topic list`, **Then** they see their nodes and topics listed correctly

---

### User Story 3 - Understanding Hardware-Software Bridge via URDF (Priority: P2)

A student needs to understand how ROS 2 connects abstract AI concepts ("the brain") to physical hardware ("the body") by learning URDF (Unified Robot Description Format) to define a robot's physical structure.

**Why this priority**: Critical for physical AI understanding - students must see how software models map to real robot geometry, kinematics, and sensors.

**Independent Test**: Student can read a simple URDF file, identify key components (links, joints, sensors), and understand how it represents a physical robot's structure.

**Acceptance Scenarios**:

1. **Given** a simple URDF example (e.g., 2-wheeled mobile robot), **When** student examines the XML, **Then** they can identify all links (body, wheels) and joints (wheel-to-body connections)
2. **Given** a URDF with a camera sensor definition, **When** student analyzes the coordinate frames, **Then** they can explain where the camera is mounted relative to the robot body
3. **Given** URDF documentation explaining the brain-body analogy, **When** student reads about sensor data flow, **Then** they understand how sensor readings (body) get converted to ROS 2 topics (nervous system) for AI processing (brain)

---

### User Story 4 - Distinguishing Services from Topics (Priority: P2)

A student needs to understand when to use ROS 2 services (request-response) versus topics (publish-subscribe) by learning through practical examples.

**Why this priority**: Architectural decision-making - choosing the wrong communication pattern leads to inefficient or buggy robot systems.

**Independent Test**: Student can list 3 scenarios each where topics are appropriate and where services are appropriate, explaining their reasoning.

**Acceptance Scenarios**:

1. **Given** a scenario "continuously streaming lidar data", **When** student selects communication pattern, **Then** they choose topics and explain that continuous data doesn't need responses
2. **Given** a scenario "requesting robot to calculate inverse kinematics once", **When** student selects communication pattern, **Then** they choose services and explain that one-time calculations need responses
3. **Given** example code for both a topic publisher and a service server, **When** student compares the implementations, **Then** they identify the key structural differences (callbacks, request/response types)

---

### Edge Cases

- What happens when a Talker node publishes to a topic but no Listener is subscribed? (Expected: messages are sent but not received; system doesn't error)
- How does the system handle a URDF file with circular joint dependencies? (Expected: parser should detect and report error)
- What occurs if a service client makes a request but the service server is not running? (Expected: timeout error with clear message)
- How does ROS 2 handle node name conflicts (two nodes with the same name)? (Expected: second node fails to start with naming conflict error)
- What happens if a student's Python environment doesn't have rclpy installed? (Expected: clear import error with installation instructions)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Module MUST provide a clear explanation of ROS 2 architecture using the "nervous system" biological analogy (nodes as neurons, topics as nerve pathways, services as targeted neural signals)
- **FR-002**: Module MUST explain the difference between Topics (continuous data streaming) and Services (request-response interactions) with at least 2 examples each
- **FR-003**: Module MUST include verified, runnable Python code for a Talker node that publishes string messages at a configurable rate (default 1 Hz)
- **FR-004**: Module MUST include verified, runnable Python code for a Listener node that subscribes to the Talker's topic and prints received messages to terminal
- **FR-005**: Module MUST explain how ROS 2 bridges AI software ("the brain") to physical robot hardware ("the body") through sensors and actuators
- **FR-006**: Module MUST introduce URDF (Unified Robot Description Format) with a simple working example showing links, joints, and at least one sensor
- **FR-007**: Module MUST target university-level students with technical content but accessible explanations (no assumptions of prior ROS knowledge)
- **FR-008**: Code examples MUST use Python 3 with rclpy (ROS 2 Python client library) as the primary implementation language
- **FR-009**: Module MUST be organized in folder `docs/01-nervous-system-ros2/` (or equivalent based on existing structure) with at least 3 distinct markdown files
- **FR-010**: Module MUST include practical instructions for running the Talker/Listener example (commands to execute, expected terminal output)
- **FR-011**: URDF examples MUST include inline comments explaining each major component (links, joints, visual/collision geometry, sensors)
- **FR-012**: Module MUST explain coordinate frames and transformations in the context of URDF (how sensor positions are defined relative to robot body)

### Key Entities *(include if feature involves data)*

- **ROS 2 Node**: A computational process that performs a specific task (analogous to a neuron). Has a unique name, can publish to topics, subscribe to topics, provide services, or call services.
- **Topic**: A named communication channel for continuous data streaming (analogous to nerve pathways). Uses publish-subscribe pattern, supports multiple publishers and subscribers, unidirectional data flow.
- **Service**: A request-response communication mechanism for one-time queries (analogous to targeted neural signals). Synchronous interaction, single client-server pair per transaction.
- **Message**: Data structure transmitted over topics. Defined by ROS 2 message types (e.g., std_msgs/String), contains typed fields.
- **URDF (Unified Robot Description Format)**: XML-based robot model specification. Defines physical structure (links = rigid bodies), kinematic relationships (joints = connections), visual appearance, collision geometry, and sensor placements.
- **Link**: A rigid body component in URDF (e.g., robot chassis, wheel, camera mount). Has physical properties (mass, inertia), visual representation, and collision boundaries.
- **Joint**: A kinematic connection between two links in URDF (e.g., revolute for wheels, fixed for sensor mounts). Defines degrees of freedom and motion constraints.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can successfully run the provided Talker/Listener code examples and observe message communication within 15 minutes of reading the instructions
- **SC-002**: At least 90% of students can correctly identify whether to use topics or services when presented with 5 different robot communication scenarios
- **SC-003**: Students can read a simple URDF file (under 100 lines) and correctly identify all links, joints, and sensors within 10 minutes
- **SC-004**: Module documentation is clear enough that students require zero additional external resources to complete the Talker/Listener exercise (measured by completion rate without consulting ROS 2 official docs)
- **SC-005**: Students can explain the nervous system analogy (nodes, topics, services) to a peer in under 3 minutes after completing the module
- **SC-006**: Code examples run without modification on Ubuntu 22.04 with ROS 2 Humble installed (100% success rate on standard environment)
- **SC-007**: Students demonstrate understanding by successfully modifying the Talker node to publish custom messages (e.g., sensor readings instead of "Hello World") within 30 minutes

## Assumptions

- Students have basic Python programming knowledge (functions, classes, imports)
- Students have ROS 2 Humble installed on Ubuntu 22.04 LTS (as specified in Module 0 prerequisites)
- Students have completed Module 0 (Workstation Setup) and have a working ROS 2 environment
- Students understand basic command-line operations (cd, ls, running Python scripts)
- URDF section assumes students have basic understanding of coordinate systems (x, y, z axes)
- Examples will use standard ROS 2 message types (std_msgs/String) to minimize complexity
- Physical robot hardware is not required for this module (simulation and visualization only)
- Students have access to online documentation links (ROS 2 official docs) for deeper exploration beyond module scope

## Dependencies

- **ROS 2 Humble Hawksbill**: All code examples depend on ROS 2 Humble being installed and sourced
- **Python 3.10+**: rclpy requires Python 3.10 or later
- **rclpy package**: ROS 2 Python client library must be installed
- **std_msgs package**: Standard message definitions for string messages
- **urdf package**: For URDF parsing and visualization (optional for learning, required for robot_state_publisher examples)
- **rviz2**: For visualizing URDF models (optional but recommended for better understanding)
- **Module 0 completion**: Students must have completed workstation setup before starting this module

## Out of Scope

- Advanced ROS 2 concepts (Actions, Parameters, Launch files with complex configurations) - covered in later modules
- Multi-robot communication and namespaces
- Custom message type creation (std_msgs is sufficient for introductory examples)
- Advanced URDF features (XACRO macros, dynamic properties, Gazebo plugins)
- Real-time control and performance optimization
- Deployment to physical robot hardware (Jetson Orin deployment in later modules)
- ROS 2 security features (SROS2)
- Docker containerization for ROS 2 (covered in Module 0 if needed)
- Integration with external sensors or actuators
- Advanced coordinate frame transformations (TF2 library covered in Module 3)
- Service implementation details (will use simple examples only, full service creation in later modules)

## Notes

- The "nervous system" analogy should be reinforced consistently throughout all 3+ markdown files
- Code examples should prioritize clarity over efficiency (verbose code with comments preferred)
- URDF introduction should use a minimal example (e.g., simple 2-wheeled robot) before showing complex models
- Consider including troubleshooting section for common student errors (e.g., "rclpy not found", "topic not publishing")
- Module structure recommendation:
  - File 1: `01-ros2-architecture.md` (concepts, nervous system analogy)
  - File 2: `02-first-nodes.md` (Talker/Listener implementation)
  - File 3: `03-urdf-basics.md` (robot modeling, hardware-software bridge)
- Talker/Listener example should be simple enough to fit in a single screen of code for easy comprehension
- URDF example should be visualized with screenshots from rviz2 to help visual learners
