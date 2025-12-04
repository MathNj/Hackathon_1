# Implementation Plan: Module 1 - The Robotic Nervous System

**Branch**: `1-nervous-system-ros2` | **Date**: 2025-12-04 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/1-nervous-system-ros2/spec.md`

## Summary

Create educational content for Module 1 of the Physical AI Textbook, introducing students to ROS 2 fundamentals through the "Nervous System" biological analogy. This module establishes the foundation for all subsequent modules by teaching core concepts (Nodes, Topics, Services) and providing hands-on Python (`rclpy`) tutorials with verified Talker/Listener code examples. Students will also learn URDF robot modeling to understand the hardware-software bridge.

**Primary Deliverables**:
- 3 markdown documentation files in `docs/01-nervous-system-ros2/`
- 2 verified Python code examples (`minimal_publisher.py`, `minimal_subscriber.py`)
- 1 URDF example file (2-link robot arm)
- Sidebar configuration updates for automatic menu generation

## Technical Context

**Language/Version**: Python 3.10+ (for code examples), Markdown (for documentation)
**Primary Dependencies**: ROS 2 Humble, rclpy, std_msgs, urdf
**Storage**: Static markdown files in Docusaurus project, code files embedded or linked
**Testing**: Manual execution testing of code examples (students must be able to run without errors)
**Target Platform**: Docusaurus 3.x static site, Ubuntu 22.04 LTS (student runtime environment)
**Project Type**: Educational documentation (web-based textbook)
**Performance Goals**: Page load <2s, code examples execute in <5s on student hardware
**Constraints**:
  - Must work with ROS 2 Humble (LTS version specified in Module 0)
  - Code must be copy-paste friendly (no hidden characters or formatting issues)
  - Diagrams must be accessible (alt text for screen readers)
**Scale/Scope**: 3 documentation files, ~5000-7000 words total, 2-3 code examples per file

## Constitution Check

*GATE: Must pass before content creation. Re-check after drafting.*

✅ **Curriculum-Driven Architecture**:
- Aligns with Module 1 mandate: "ROS 2 fundamentals (Nodes, Topics, Services, Rclpy), URDF robot descriptions"
- Supports learning objectives for nervous system analogy and communication patterns

✅ **Technology Stack Mandate**:
- Frontend: Docusaurus 3.x (matches constitution)
- Code Examples: Python with rclpy (matches ROS 2 framework requirement)
- Simulation Context: References Gazebo (Module 2) and Isaac Sim (Module 3) as future topics

✅ **Pedagogical Approach**:
- Uses biological analogy (Nervous System) for technical concepts
- Hands-on tutorials with immediate feedback (Talker/Listener)
- Progressive complexity (concepts → code → hardware modeling)

✅ **Content Standards**:
- Technical accuracy: Code examples must be verified on Ubuntu 22.04 + ROS 2 Humble
- Accessibility: Plain language explanations before technical jargon
- Interactive: Students write and execute code, not just read

**Constitution Violations**: None

## Project Structure

### Documentation (this feature)

```text
specs/1-nervous-system-ros2/
├── spec.md                    # Feature specification (completed)
├── plan.md                    # This file (implementation architecture)
├── checklists/
│   └── requirements.md        # Specification quality checklist (completed)
└── tasks.md                   # Task breakdown (NOT created yet - use /sp.tasks)
```

### Source Code (repository root)

```text
web/docs/01-nervous-system-ros2/          # Module 1 content directory
├── 01-intro-to-ros2.md                   # File 1: ROS 2 Architecture Overview
├── 02-nodes-and-topics.md                # File 2: Python Talker/Listener Tutorial
├── 03-urdf-modeling.md                   # File 3: Robot Description Basics
├── assets/                                # Supporting files
│   ├── node-graph-diagram.svg            # Visual: Node communication diagram
│   ├── nervous-system-analogy.svg        # Visual: Biological to ROS 2 mapping
│   ├── minimal_publisher.py              # Code: Talker node (verified)
│   ├── minimal_subscriber.py             # Code: Listener node (verified)
│   └── simple_arm.urdf                   # URDF: 2-link robot arm example
└── README.md                              # Module index (optional, for navigation)

web/sidebars.ts                            # Update: Add Module 1 to sidebar tree
```

**Structure Decision**: Educational documentation follows Docusaurus convention of numbered files for chapter ordering. The `assets/` subdirectory contains all diagrams, code examples, and robot models to keep markdown files clean. Sidebar configuration will use auto-generation based on folder structure.

## Content Architecture

### Phase 0: Research & Asset Preparation (Pre-Writing)

**Objective**: Gather existing ROS 2 documentation patterns, verify code examples work, create diagrams.

#### Research Tasks:
1. **Review ROS 2 Official Tutorials**:
   - Analyze how official docs explain pub/sub patterns
   - Extract best practices for beginner Python code examples
   - Identify common student errors to address proactively

2. **Code Verification**:
   - Write and test `minimal_publisher.py` on Ubuntu 22.04 + ROS 2 Humble
   - Write and test `minimal_subscriber.py` with the publisher
   - Verify output matches expected behavior (messages printed to console)
   - Test with `ros2 topic list`, `ros2 node list` commands

3. **URDF Example Creation**:
   - Design simple 2-link arm (shoulder + elbow joints)
   - Test parsing with `check_urdf` tool
   - Visualize in RViz2 to ensure correctness

4. **Diagram Specifications**:
   - Sketch node graph showing Talker → Topic → Listener flow
   - Sketch nervous system analogy (neuron = node, synapse = topic)
   - Tools: Draw.io, Excalidraw, or SVG for accessibility

**Deliverable**: `specs/1-nervous-system-ros2/research.md` containing:
- Links to referenced ROS 2 documentation
- Verified code snippets with test output
- Diagram mockups or final SVG files
- Notes on common student errors to address

---

### Phase 1: Content Design & Contracts (Structure Definition)

**Objective**: Define exact section headings, learning flow, and content contracts for each file.

#### File 1: `01-intro-to-ros2.md` - ROS 2 Architecture Overview

**Purpose**: Introduce ROS 2 as the "nervous system" for robots, explaining core architectural concepts without code.

**Content Contract**:

| Section | Content | Success Metric |
|---------|---------|----------------|
| **Intro: Why ROS 2?** | Motivate ROS 2 as industry standard, explain middleware role | Student can name 2 companies using ROS 2 |
| **The Nervous System Analogy** | Map biological concepts to ROS 2 (neuron→node, nerve→topic) | Student can draw analogy diagram from memory |
| **Nodes: The Neurons** | Explain nodes as computational units, unique naming, lifecycle | Student can define "node" in 1 sentence |
| **Topics: The Nerve Pathways** | Explain pub/sub pattern, unidirectional data flow, multiple subscribers | Student identifies when to use topics (continuous data) |
| **Services: Targeted Signals** | Explain request-response pattern, synchronous calls | Student identifies when to use services (one-time requests) |
| **Visual: Node Graph Diagram** | Embedded SVG showing Talker→Topic→Listener with labels | Student can explain diagram to a peer |
| **Hardware Context** | Brief preview: How sensors (cameras, lidar) connect via ROS 2 topics | Student understands "brain-to-body bridge" concept |
| **Next Steps** | Transition to hands-on tutorial in File 2 | Clear call-to-action to next file |

**Word Count**: 1500-2000 words
**Diagrams**: 2 (Nervous System Analogy, Node Graph)
**Code**: None (concepts only)
**Learning Time**: 20-30 minutes reading

---

#### File 2: `02-nodes-and-topics.md` - Python Talker/Listener Tutorial

**Purpose**: Provide hands-on experience writing, running, and modifying ROS 2 Python nodes.

**Content Contract**:

| Section | Content | Success Metric |
|---------|---------|----------------|
| **Setup Verification** | Commands to verify ROS 2 environment (`ros2 --version`, sourcing) | Student confirms ROS 2 Humble is active |
| **Tutorial 1: Writing a Publisher** | Step-by-step explanation of `minimal_publisher.py` code | Student runs Talker, sees messages in terminal |
| **Code Walkthrough: Publisher** | Line-by-line breakdown: imports, class, timer callback, main() | Student can explain each code block's purpose |
| **Running the Publisher** | Commands: `ros2 run`, expected output screenshots | Student successfully executes publisher |
| **Tutorial 2: Writing a Subscriber** | Step-by-step explanation of `minimal_subscriber.py` code | Student runs Listener, receives Talker messages |
| **Code Walkthrough: Subscriber** | Line-by-line breakdown: subscription callback, message handling | Student can explain callback mechanism |
| **Running Together** | Instructions to run both nodes in separate terminals | Student sees message flow between nodes |
| **Debugging Tips** | Common errors: sourcing issues, topic name mismatches, import errors | Student can troubleshoot 3 common errors |
| **Exercise: Modify the Message** | Challenge: Change "Hello World" to "Robot Sensor: [count]" | Student successfully modifies and re-runs code |
| **ROS 2 CLI Tools** | Using `ros2 topic list`, `ros2 node list`, `ros2 topic echo` | Student can inspect running system |
| **Next Steps** | Transition to URDF hardware modeling in File 3 | Clear progression to robot descriptions |

**Word Count**: 2500-3500 words
**Code Blocks**: 2 full examples (`minimal_publisher.py`, `minimal_subscriber.py`)
**Commands**: 8-10 shell commands with expected outputs
**Learning Time**: 60-90 minutes (including coding and testing)

**Code Files** (in `assets/`):

**`minimal_publisher.py`**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**`minimal_subscriber.py`**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

#### File 3: `03-urdf-modeling.md` - Robot Description Basics

**Purpose**: Teach students how ROS 2 models physical robot structure using URDF XML format.

**Content Contract**:

| Section | Content | Success Metric |
|---------|---------|----------------|
| **Why URDF?** | Explain need for robot models: simulation, visualization, kinematics | Student understands "digital twin" concept |
| **The Brain-Body Bridge** | Connect ROS 2 (software) to physical hardware via URDF descriptions | Student can explain how sensors in URDF map to topics |
| **URDF Basics: Links** | Define rigid bodies (robot parts), visual vs collision geometry | Student can identify a "link" in URDF XML |
| **URDF Basics: Joints** | Define kinematic connections (revolute, fixed, prismatic) | Student can identify a "joint" in URDF XML |
| **Example: 2-Link Robot Arm** | Walkthrough of `simple_arm.urdf` (base, shoulder, elbow) | Student can visualize the arm structure |
| **Code Walkthrough: URDF XML** | Line-by-line explanation of XML tags, coordinate frames | Student can explain `<link>` and `<joint>` elements |
| **Visualizing in RViz2** | Commands to launch robot_state_publisher and view model | Student sees URDF rendered in 3D |
| **Coordinate Frames** | Explain TF (transform) tree, parent-child relationships | Student understands relative positioning |
| **Project: Modify the Arm** | Challenge: Add a 3rd link (wrist joint) to the arm | Student successfully extends URDF |
| **Real-World Context** | How URDF connects to sensors (cameras, lidar) for Module 3 SLAM | Student sees Module 1→3 progression |
| **Summary & Next Steps** | Recap nervous system analogy, preview Module 2 (simulation) | Clear transition to Digital Twin module |

**Word Count**: 2000-2500 words
**Code Blocks**: 1 full URDF file (`simple_arm.urdf`)
**Diagrams**: 1 (coordinate frame tree)
**Learning Time**: 45-60 minutes (including visualization exercise)

**URDF File** (in `assets/`):

**`simple_arm.urdf`**:
```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Base Link (fixed to world) -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
    </collision>
  </link>

  <!-- Link 1 (upper arm) -->
  <link name="link1">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
    </collision>
  </link>

  <!-- Joint 1 (shoulder - revolute) -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Link 2 (forearm) -->
  <link name="link2">
    <visual>
      <geometry>
        <box size="0.04 0.04 0.25"/>
      </geometry>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.04 0.04 0.25"/>
      </geometry>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
    </collision>
  </link>

  <!-- Joint 2 (elbow - revolute) -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

</robot>
```

---

### Phase 2: Sidebar Integration

**Objective**: Configure Docusaurus sidebar to auto-generate navigation for Module 1.

#### Sidebar Configuration (`web/sidebars.ts`)

**Current Structure** (assume):
```typescript
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Getting Started',
      items: ['module-0-setup/intro'],
    },
    // Add Module 1 here
  ],
};
```

**Updated Structure**:
```typescript
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 0: Setup',
      items: ['module-0-setup/intro', /* other setup docs */],
    },
    {
      type: 'category',
      label: 'Module 1: The Nervous System (ROS 2)',
      collapsed: false,  // Expand by default for first-time learners
      items: [
        '01-nervous-system-ros2/01-intro-to-ros2',
        '01-nervous-system-ros2/02-nodes-and-topics',
        '01-nervous-system-ros2/03-urdf-modeling',
      ],
    },
    // Module 2, 3, 4 to follow
  ],
};

export default sidebars;
```

**Auto-Generation Note**: If using Docusaurus auto-generated sidebars (via `docs` plugin config), ensure folder naming follows convention:
- `web/docs/01-nervous-system-ros2/` → Auto-generates "Nervous System ROS2" category
- Numbered files (`01-`, `02-`, `03-`) → Auto-sorts in correct order

**Verification**:
- Run `npm run start` (Docusaurus dev server)
- Check left sidebar renders "Module 1: The Nervous System (ROS 2)"
- Verify all 3 files appear in correct order
- Test navigation links work correctly

---

## Data Model (if applicable)

**Not Applicable**: This feature creates static educational content. No database entities, API contracts, or dynamic data required. All content is markdown + static assets.

---

## Contracts (API/Interface Definitions)

**Not Applicable**: No programmatic interfaces. Content follows Docusaurus markdown conventions:
- Frontmatter YAML for metadata (title, sidebar_label, description)
- Standard markdown syntax (headings, code blocks, links)
- Embedded SVG/PNG for diagrams
- Code files linked or embedded with syntax highlighting

**Example Frontmatter**:
```yaml
---
sidebar_label: 'Intro to ROS 2'
sidebar_position: 1
description: 'Learn ROS 2 architecture using the nervous system analogy'
keywords: [ros2, nodes, topics, services, nervous system]
---
```

---

## Testing Strategy

### Content Validation

| Test Type | Method | Pass Criteria |
|-----------|--------|---------------|
| **Code Execution** | Run all Python examples on Ubuntu 22.04 + ROS 2 Humble | All code blocks execute without errors, produce expected output |
| **URDF Parsing** | Test `simple_arm.urdf` with `check_urdf` command | URDF parses successfully, no XML errors |
| **RViz Visualization** | Launch `robot_state_publisher` with URDF | Arm model renders correctly in RViz2 |
| **Link Integrity** | Check all internal links between files | All links resolve to correct pages/assets |
| **Accessibility** | Screen reader test on diagrams (alt text) | All images have descriptive alt text |
| **Comprehension** | Beta reader (student or peer) reviews content | Beta reader completes all exercises without external help |

### Student Success Metrics (from spec.md)

- SC-001: Students run Talker/Listener in <15 minutes → Test with 3 students, measure time
- SC-002: 90% identify correct communication patterns → Quiz with 5 scenarios
- SC-003: Read URDF and identify components in 10 minutes → Timed exercise with new URDF
- SC-006: 100% code success rate on standard environment → Test code on fresh VM

---

## Implementation Phases

### Phase 0: Research & Preparation (1-2 days)
**Owner**: Content Creator
**Deliverables**:
- Verified code examples (`minimal_publisher.py`, `minimal_subscriber.py`)
- Tested URDF file (`simple_arm.urdf`)
- Diagram SVG files (nervous system analogy, node graph)
- `research.md` documentation

**Completion Criteria**:
- All code runs successfully on test VM (Ubuntu 22.04 + ROS 2 Humble)
- Diagrams approved by technical reviewer
- Research notes capture common student errors

---

### Phase 1: Draft File 1 (`01-intro-to-ros2.md`) (1 day)
**Owner**: Content Creator
**Deliverables**:
- 1500-2000 word markdown file
- 2 embedded diagrams (nervous system, node graph)
- Frontmatter metadata
- Internal links to File 2

**Completion Criteria**:
- Passes markdown linter (no syntax errors)
- Diagrams have alt text
- Beta reader confirms clarity (nervous system analogy understood)

---

### Phase 2: Draft File 2 (`02-nodes-and-topics.md`) (1-2 days)
**Owner**: Content Creator
**Deliverables**:
- 2500-3500 word tutorial
- 2 embedded code blocks with syntax highlighting
- 8-10 shell commands with expected outputs
- Link to code files in `assets/`

**Completion Criteria**:
- Code blocks are copy-paste ready (no hidden characters)
- Commands tested on Ubuntu 22.04
- Beta reader completes tutorial successfully

---

### Phase 3: Draft File 3 (`03-urdf-modeling.md`) (1 day)
**Owner**: Content Creator
**Deliverables**:
- 2000-2500 word tutorial
- 1 embedded URDF code block
- 1 coordinate frame diagram
- RViz visualization instructions

**Completion Criteria**:
- URDF parses without errors
- Beta reader visualizes arm in RViz2
- Coordinate frame explanation clear

---

### Phase 4: Sidebar Integration & Navigation Testing (0.5 days)
**Owner**: Developer
**Deliverables**:
- Updated `web/sidebars.ts` configuration
- Verified navigation in Docusaurus dev server
- Screenshot of rendered sidebar

**Completion Criteria**:
- Module 1 appears in sidebar with correct label
- All 3 files listed in order (01, 02, 03)
- Navigation links work correctly

---

### Phase 5: Final Review & Student Testing (1 day)
**Owner**: Content Creator + Reviewer
**Deliverables**:
- Peer review feedback addressed
- Student beta test (3 students complete all exercises)
- Final proofreading pass
- Accessibility audit (screen reader test)

**Completion Criteria**:
- All beta students complete exercises successfully
- Measured time aligns with success criteria (SC-001: <15 min)
- No critical errors or ambiguities remain

---

### Phase 6: Deployment & Iteration (ongoing)
**Owner**: Content Creator
**Deliverables**:
- Merged to main branch
- Published to GitHub Pages
- Student feedback collected (via issues or survey)
- Iteration plan for Module 2

**Completion Criteria**:
- Content live and accessible at textbook URL
- Analytics show page views and time-on-page metrics
- Feedback loop established for continuous improvement

---

## Risk Analysis

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| **Code doesn't work on student machines** | Medium | High | Test on multiple Ubuntu 22.04 VMs with fresh ROS 2 Humble installs. Provide troubleshooting section. |
| **URDF too complex for beginners** | Low | Medium | Use minimal 2-link arm (already simple). Provide inline comments in XML. |
| **Diagrams unclear (nervous system analogy)** | Low | Medium | Beta test with non-robotics students. Add text labels to all diagram elements. |
| **Students skip to code without reading concepts** | Medium | Low | Use clear prerequisites at start of File 2: "Complete File 1 first". |
| **Sidebar config breaks existing modules** | Low | High | Test sidebar changes on dev server before merge. Version control allows easy rollback. |
| **Learning time exceeds estimates** | Medium | Medium | Monitor beta test times. Adjust content complexity if needed. Split File 2 into two files if >90 min. |

---

## Open Questions

1. **Diagram Tool**: Which tool for diagrams - Draw.io, Excalidraw, or hand-coded SVG?
   - **Recommendation**: Excalidraw for speed, export to SVG for accessibility
   - **Decision needed by**: Phase 0 start

2. **Code File Location**: Embed code in markdown or link to separate `.py` files?
   - **Recommendation**: Embed in markdown for copy-paste, also provide downloadable files in `assets/`
   - **Decision needed by**: Phase 1 start

3. **Beta Tester Pool**: Who will test the content before launch?
   - **Recommendation**: 3 students from target audience (university-level, no ROS experience)
   - **Decision needed by**: Phase 5 start

4. **Versioning**: How to handle ROS 2 version updates (Humble → Iron)?
   - **Recommendation**: Note ROS 2 Humble explicitly in frontmatter, add update section in Module 0
   - **Decision needed by**: N/A (long-term maintenance)

---

## Success Metrics

**Definition of Done**:
- [ ] All 3 markdown files published in `web/docs/01-nervous-system-ros2/`
- [ ] All code examples verified on Ubuntu 22.04 + ROS 2 Humble
- [ ] URDF visualizes correctly in RViz2
- [ ] Sidebar configuration updated and tested
- [ ] 3 beta students complete all exercises successfully
- [ ] Measured times align with success criteria (SC-001, SC-003)
- [ ] Accessibility audit passed (screen reader test, alt text on all images)
- [ ] Content merged to main branch and deployed to GitHub Pages

**Long-Term Success**:
- 90% of students complete Module 1 without external help (SC-004)
- Student survey: 4.5+ stars for clarity and usefulness
- <5 bug reports per 100 students (code/content errors)
- Module 1 serves as solid foundation for Module 2 (simulation)

---

## Notes

- **Biological Analogy Consistency**: Ensure "nervous system" metaphor is reinforced in all 3 files (not just File 1)
- **Code Comments**: Python code should have inline comments explaining every major block
- **Progressive Complexity**: File 1 (no code) → File 2 (basic pub/sub) → File 3 (URDF structure). Each builds on previous.
- **Module Transitions**: End of File 3 must clearly preview Module 2 (Digital Twin / Simulation)
- **Accessibility**: All diagrams must have descriptive alt text for screen readers
- **Responsive Design**: Test diagrams render correctly on mobile devices (Docusaurus default responsive CSS should handle this)
