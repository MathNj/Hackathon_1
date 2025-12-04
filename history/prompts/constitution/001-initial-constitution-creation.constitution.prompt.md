---
id: 001
title: Initial Constitution Creation
stage: constitution
date: 2025-12-03
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: Hamdan
command: /sp.constitution
labels: ["constitution", "initialization", "project-setup", "embodied-intelligence"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - N/A (documentation only)
---

## Prompt

### CONTEXT FOR THE CONSTITUTION:
**1. The Mission:** Create a Docusaurus-based textbook that teaches students how to bridge digital AI (LLMs, VLA) with physical robots (ROS 2, NVIDIA Isaac Sim, Jetson Orin). The goal is "Embodied Intelligence."

**2. The Tech Stack (Strict Mandates):**
- **Frontend:** Docusaurus (Static Site Generation), deployed to GitHub Pages.
- **Backend (RAG):** FastAPI, OpenAI Agents SDK, Qdrant Cloud (Vector DB), Neon Serverless Postgres.
- **Tools:** Spec-Kit Plus, Claude Code.
- **Auth (Bonus):** Better-Auth.com.

**3. The Curriculum (Must appear in "Content Standards"):**
- **Module 1 (The Nervous System):** ROS 2, Nodes, Topics, Rclpy, URDF.
- **Module 2 (The Digital Twin):** Gazebo, Unity, Physics Simulation, Sensor Emulation (LiDAR/Depth).
- **Module 3 (The Brain):** NVIDIA Isaac Sim, Synthetic Data, Visual SLAM, Nav2.
- **Module 4 (The Mind - VLA):** Vision-Language-Action models, Whisper (Voice), LLM Cognitive Planning.
- **Capstone:** Building an "Autonomous Humanoid" that can plan, navigate, and manipulate objects.

**4. The Hardware Reality (Infrastructure Constraints):**
The book must assume the reader has:
- **Workstation:** NVIDIA RTX 4070 Ti+ (for Isaac Sim).
- **Edge:** NVIDIA Jetson Orin Nano/NX.
- **Robot:** Unitree Go2 (Quadruped) or Unitree G1 (Humanoid).

**5. The Advanced Features (The "Amendments"):**
- **RAG Chatbot:** Must be embedded in the book to answer questions from selected text.
- **Personalization:** Users log in → System profiles them (Software vs Hardware bg) → Content adapts via button press.
- **Localization:** One-click translation of chapters into Urdu.

### REQUIRED OUTPUT FORMAT:
Please generate the `project_constitution.md` file now. It should be formal, authoritative, and strictly formatted using Markdown headers. Include a "Preamble," "Mission," "Technical Architecture," "Curriculum Standards," and "Operational Workflows."

## Response snapshot

Created comprehensive Embodied Intelligence Textbook Constitution (v1.0.0) with the following structure:

**Core Principles Established (8 principles):**
1. Curriculum-Driven Architecture (NON-NEGOTIABLE) - 4-module structure + capstone
2. Technology Stack Mandate (STRICT) - Docusaurus, FastAPI, OpenAI Agents SDK, Qdrant, Neon
3. Hardware Infrastructure Assumptions - RTX 4070 Ti+, Jetson Orin, Unitree robots
4. Pedagogical Principles - Progressive disclosure, hands-on first, error-driven learning
5. RAG Chatbot Integration (MANDATORY) - Context-aware Q&A with source citations
6. Personalization System - Profile-based content adaptation (Software/Hardware/Student)
7. Security and Ethical AI - GDPR/CCPA compliance, AI safety constraints, open source licensing
8. Deployment and Localization - GitHub Pages deployment, Urdu translation support

**Technical Architecture Defined:**
- Frontend: Docusaurus 3.x structure with React components (RAGChatWidget, AdaptiveContent, CodePlayground, RobotVisualizer)
- Backend: FastAPI with RAG orchestration, Qdrant vector search, OpenAI Agents SDK
- Infrastructure: GitHub Pages, Railway/Render, Qdrant Cloud, Neon Postgres
- CI/CD: GitHub Actions workflows for build, deploy, and Qdrant re-indexing

**Curriculum Standards:**
- Module 1: ROS 2 fundamentals (2 weeks)
- Module 2: Gazebo/Unity simulation (3 weeks)
- Module 3: NVIDIA Isaac Sim + Nav2 (4 weeks)
- Module 4: VLA models + voice interface (4 weeks)
- Final Capstone: Autonomous humanoid system (2 weeks)

**Content Standards:**
- Writing style guidelines (second person, active voice, technical but encouraging)
- Visual standards (Mermaid.js diagrams, 1080p screenshots/videos)
- Accessibility requirements (WCAG 2.1 AA compliance)
- Code quality standards (ROS 2 style guide, PEP 8, docstrings, linting)

**Operational Workflows:**
- Content creation process (proposal → draft → review → publish)
- RAG system maintenance (weekly feedback review, monthly re-indexing, quarterly audits)
- User feedback loop (in-chapter ratings, Discord community, office hours)

**Governance:**
- Amendment process defined (proposal → discussion → vote → implementation)
- Version 1.0.0 ratified on 2025-12-03
- Compliance checks and conflict resolution procedures established

## Outcome

- ✅ Impact: Established foundational governance document defining project mission, technical mandates, curriculum structure, and operational procedures. All future features and content must align with these constitutional principles.
- 🧪 Tests: N/A (documentation artifact)
- 📁 Files: Created `.specify/memory/constitution.md` (593 lines) with comprehensive project constitution
- 🔁 Next prompts:
  - `/sp.specify` to create first feature specification (e.g., Docusaurus setup, RAG backend)
  - `/sp.plan` to architect technical implementation
  - `/sp.adr` to document architectural decisions for tech stack choices
- 🧠 Reflection: Constitution successfully captures all user requirements including curriculum structure, tech stack mandates, hardware assumptions, RAG integration, personalization, and localization. The "Educational First-Class Citizenship" principle ensures all technical decisions serve learning outcomes. Templates will need updates to reference hardware requirements and curriculum module alignment.

## Evaluation notes (flywheel)

- Failure modes observed: None. Constitution creation completed successfully with all placeholder tokens filled and structured per requirements.
- Graders run and results (PASS/FAIL):
  - ✅ PASS: All placeholders replaced with concrete values
  - ✅ PASS: Formal Markdown structure with proper hierarchy
  - ✅ PASS: All 5 required modules defined with durations and learning outcomes
  - ✅ PASS: Tech stack matches user requirements (Docusaurus, FastAPI, Qdrant, Neon)
  - ✅ PASS: Hardware constraints documented (RTX 4070 Ti, Jetson Orin, Unitree robots)
  - ✅ PASS: Advanced features included (RAG, personalization, Urdu localization)
  - ✅ PASS: Version 1.0.0 with ratification date (2025-12-03)
- Prompt variant (if applicable): N/A (initial constitution creation)
- Next experiment (smallest change to try): Consider adding principle for "Content Versioning Strategy" to handle curriculum updates without breaking student progress (could be future amendment).
