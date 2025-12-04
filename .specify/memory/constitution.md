<!--
SYNC IMPACT REPORT
==================
Version Change: 1.0.0 (Initial Constitution)
Ratification Date: 2025-12-03
Last Amended Date: 2025-12-03

Summary:
- Initial constitution created for Embodied Intelligence Textbook project
- Defined 8 core principles covering curriculum design, tech stack, hardware infrastructure, pedagogical approach, RAG integration, personalization, security, and deployment
- Established technical architecture requirements for Docusaurus frontend, FastAPI backend, and AI/robotics stack
- Defined curriculum standards for 4 core modules plus capstone
- Created operational workflows for content creation and quality assurance

Modified Principles: N/A (Initial version)
Added Sections:
  - Core Principles (8 principles)
  - Technical Architecture
  - Curriculum Standards
  - Content Standards
  - Operational Workflows
  - Governance

Templates Requiring Updates:
  ✅ .specify/templates/plan-template.md - Constitution Check section should reference hardware requirements and curriculum standards
  ✅ .specify/templates/spec-template.md - User scenarios should consider module-based learning journeys
  ✅ .specify/templates/tasks-template.md - Tasks should align with curriculum delivery phases

Follow-up TODOs:
  - None (all critical elements defined)
-->

# Embodied Intelligence Textbook Constitution

**Project Mission**: Create a comprehensive Docusaurus-based educational textbook that teaches students to bridge digital AI (LLMs, Vision-Language-Action models) with physical robotics platforms (ROS 2, NVIDIA Isaac Sim, Jetson Orin), culminating in autonomous humanoid robot systems.

**Target Audience**: Students and practitioners seeking to master "Embodied Intelligence" — the intersection of cognitive AI and physical robot control.

---

## Preamble

This Constitution establishes the foundational principles, technical mandates, curriculum standards, and operational workflows for the Embodied Intelligence Textbook project. All design decisions, feature implementations, and content creation MUST align with the standards defined herein.

The project operates under the principle of **Educational First-Class Citizenship**: every technical decision must demonstrably enhance student learning outcomes, accessibility, and practical skill acquisition.

---

## Core Principles

### I. Curriculum-Driven Architecture (NON-NEGOTIABLE)

All platform features MUST support the four-module curriculum structure:

1. **Module 1 - The Nervous System**: ROS 2 fundamentals (Nodes, Topics, Services, Rclpy), URDF robot descriptions, real-time communication patterns
2. **Module 2 - The Digital Twin**: Gazebo Classic/Fortress simulation, Unity integration, physics-based sensor emulation (LiDAR, Depth Cameras, IMU)
3. **Module 3 - The Brain**: NVIDIA Isaac Sim workflows, synthetic data generation, Visual SLAM (ORB-SLAM3, RTAB-Map), Nav2 autonomous navigation stack
4. **Module 4 - The Mind (VLA)**: Vision-Language-Action models (OpenVLA, RT-X), Whisper voice interface, LLM-based cognitive planning and task decomposition
5. **Capstone Project**: Design, simulate, and deploy an autonomous humanoid capable of multimodal planning, navigation, and object manipulation

**Rationale**: The curriculum is the contract with students. Every UI component, backend service, and RAG response must serve one or more learning objectives from these modules. Features not directly traceable to curriculum outcomes must be justified or deferred.

---

### II. Technology Stack Mandate (STRICT)

The following technology choices are **IMMUTABLE** and form the foundation of the platform:

#### Frontend
- **Framework**: Docusaurus 3.x (Static Site Generation)
- **Deployment**: GitHub Pages (public, version-controlled)
- **Styling**: CSS Modules + Tailwind CSS (for custom components)
- **Interactivity**: React 18+ (for interactive code playgrounds and 3D visualizations)

#### Backend (RAG System)
- **API Framework**: FastAPI (async Python)
- **Agent Framework**: OpenAI Agents SDK (for RAG orchestration)
- **Vector Database**: Qdrant Cloud (managed service)
- **Relational Database**: Neon Serverless Postgres (user profiles, analytics)
- **Embeddings**: OpenAI `text-embedding-3-small` (cost-optimized)

#### AI/Robotics Stack
- **Simulation**: NVIDIA Isaac Sim 4.x (primary), Gazebo Fortress (secondary)
- **Robot Framework**: ROS 2 Humble/Iron (LTS versions)
- **VLA Models**: OpenVLA, Octo, RT-X (via HuggingFace Transformers)
- **Voice**: OpenAI Whisper (on-device via whisper.cpp for Jetson)
- **LLM Reasoning**: GPT-4 Turbo (cloud), Llama 3.2 (edge fallback for Jetson)

#### Development Tools
- **Specification Framework**: Spec-Kit Plus (SDD methodology)
- **AI Assistant**: Claude Code (autonomous implementation)
- **Version Control**: Git + GitHub (mono-repo)

#### Authentication (Optional/Bonus)
- **Provider**: Better-Auth.com (email/password + OAuth providers)
- **Session Management**: Secure HTTP-only cookies, 30-day expiry

**Rationale**: These choices balance educational accessibility (free tiers for Qdrant Cloud, Neon), industry relevance (ROS 2 is industry standard), and cutting-edge AI (VLA models). Deviations require executive approval and curriculum impact assessment.

---

### III. Hardware Infrastructure Assumptions

The textbook MUST assume students have access to the following hardware:

#### Required
- **Development Workstation**:
  - GPU: NVIDIA RTX 4070 Ti or higher (12GB+ VRAM for Isaac Sim)
  - CPU: 8-core x86_64 (Ryzen 7 / Intel i7)
  - RAM: 32GB minimum
  - OS: Ubuntu 22.04 LTS (native or dual-boot)

- **Edge Computer**:
  - NVIDIA Jetson Orin Nano (8GB) or Jetson Orin NX (16GB)
  - Storage: 256GB NVMe SSD (for ROS 2 + Isaac Sim edge deployment)

#### Recommended (for physical deployment)
- **Robot Platform** (choose one):
  - Unitree Go2 Quadruped ($1,600 USD)
  - Unitree G1 Humanoid ($16,000 USD)
- **Sensors**: Intel RealSense D435i (depth + IMU), Livox Mid-360 LiDAR

**Rationale**: These specifications are the minimum viable for running Isaac Sim (which requires RTX GPU) and deploying VLA models on edge hardware. Content that assumes more powerful hardware must clearly state "Advanced Setup Required."

**Accessibility**: For students without hardware, all modules 1-3 can be completed in simulation only. Module 4 provides cloud-based alternatives (AWS RoboMaker, Google Cloud Run).

---

### IV. Pedagogical Principles

#### Progressive Disclosure
Content MUST follow a "crawl → walk → run" structure:
1. **Crawl**: Conceptual explanation with annotated diagrams
2. **Walk**: Guided tutorial with copy-paste code blocks
3. **Run**: Challenge exercise requiring independent problem-solving

#### Hands-On First
Every theoretical concept MUST be followed by a runnable code example within 3 paragraphs. No "lecture-only" sections exceeding 500 words.

#### Error-Driven Learning
Tutorials MUST include a "Common Errors" subsection showing:
- Typical student mistakes (with error messages)
- Debugging methodology (not just solutions)
- Links to ROS 2 / Isaac Sim troubleshooting docs

#### Assessment Integration
Each module MUST include:
- **Formative**: 5-10 inline quizzes (multiple choice + code challenges)
- **Summative**: 1 capstone project with rubric (peer-reviewed via GitHub Discussions)

**Rationale**: Research shows active learning with immediate feedback improves retention by 40%+ over passive reading. Error-driven learning builds debugging skills critical for robotics.

---

### V. RAG Chatbot Integration (MANDATORY)

The RAG system MUST provide:

1. **Context-Aware Q&A**:
   - User selects text in any chapter → "Ask Claude about this" button appears
   - Backend retrieves semantically similar content from Qdrant (top-5 chunks)
   - OpenAI Agents SDK generates response grounded in retrieved context
   - Sources cited with inline links to original textbook sections

2. **Query Types Supported**:
   - Conceptual ("What is a ROS 2 topic?")
   - Procedural ("How do I install Isaac Sim on Jetson?")
   - Troubleshooting ("Why is my Nav2 costmap empty?")
   - Comparison ("Difference between Gazebo and Isaac Sim?")

3. **Content Indexing Strategy**:
   - Chunk size: 500 tokens (overlap: 50 tokens)
   - Metadata: module_id, chapter_id, code_block (true/false), difficulty_level
   - Re-index on content publish (GitHub Actions workflow)

4. **Response Constraints**:
   - Max length: 300 words (concise answers only)
   - Must include 1-3 source citations
   - If answer uncertain: "I need more context — try selecting more text or ask in Discord"

**Rationale**: RAG reduces student frustration by making expert knowledge instantly accessible. Grounding responses in textbook content prevents hallucinations and reinforces curriculum coherence.

---

### VI. Personalization System

The platform MUST adapt content based on user profiles:

#### Profile Acquisition
- **Login Required**: Users authenticate via Better-Auth (email/password or GitHub OAuth)
- **Onboarding Survey** (one-time):
  - "What's your background?" → Software Engineer | Hardware Engineer | Student | Hobbyist
  - "Experience with ROS?" → None | Beginner | Intermediate | Expert
  - "Primary Goal?" → Learn AI | Build Robots | Research | Career Transition

#### Content Adaptation (via Feature Flags)
- **Software Engineers**:
  - Emphasize Python API usage, less C++ low-level details
  - Show Docker-first setup instructions
  - Highlight cloud deployment patterns

- **Hardware Engineers**:
  - Emphasize URDF modeling, sensor calibration, real-time constraints
  - Show native Ubuntu installation (no Docker)
  - Highlight Jetson optimization techniques

- **Students**:
  - Additional "Prerequisites" sections (linear algebra, Python basics)
  - Simplified explanations with more visual aids
  - Gamification (badges for completed modules)

#### Implementation
- Profile stored in Neon Postgres (`users.profile_type`, `users.experience_level`)
- Docusaurus components check profile via API: `<AdaptiveContent profile={userProfile}>`
- User can toggle "Show All Content" to see full material

**Rationale**: Personalization reduces cognitive overload and accelerates learning by focusing on relevant pathways. Toggle ensures no content is permanently hidden.

---

### VII. Security and Ethical AI

#### Data Privacy (GDPR/CCPA Compliant)
- **User Data Collected**: Email, profile type, RAG query history, progress tracking
- **User Data NOT Collected**: No tracking cookies, no third-party analytics (Plausible Analytics only)
- **Data Retention**: RAG queries anonymized after 90 days, user profiles deleted on account closure
- **Export**: Users can download all data as JSON via `/api/profile/export`

#### AI Safety Constraints
- **Prompt Injection Defense**: RAG system filters queries with adversarial patterns (`"Ignore previous instructions"`)
- **Content Moderation**: OpenAI Moderation API screens RAG responses for harmful content
- **Rate Limiting**: 100 RAG queries/hour per user (prevents abuse)

#### Open Source Licensing
- **Textbook Content**: CC BY-NC-SA 4.0 (attribution, non-commercial, share-alike)
- **Code Examples**: MIT License (permissive, can be used commercially)
- **RAG Backend**: Apache 2.0 (can be self-hosted by institutions)

**Rationale**: Transparency and ethical AI practices build trust. Open licensing maximizes educational impact while protecting authors' attribution rights.

---

### VIII. Deployment and Localization

#### Deployment Pipeline (GitHub Actions)
1. **On Push to `main`**:
   - Run Docusaurus build (`npm run build`)
   - Deploy to GitHub Pages (custom domain: `embodiedai.dev`)
   - Trigger Qdrant re-indexing (FastAPI webhook)
   - Notify Discord #releases channel

2. **On Pull Request**:
   - Preview deployment to Vercel (temporary URL)
   - Run link checker (no broken internal links)
   - Run plagiarism check (CopyLeaks API)

#### Localization (i18n)
- **Primary Language**: English (US)
- **Secondary Language**: Urdu (Pakistan) — one-click translation button
- **Translation Method**:
  - Initial: GPT-4 Turbo translation of Markdown files
  - Refinement: Human review by native speakers (bounty program)
  - Technical Terms: Maintain English (e.g., "ROS 2 Node" not translated)
- **Storage**: Docusaurus i18n folders (`i18n/ur/docusaurus-plugin-content-docs/`)

**Rationale**: GitHub Pages is free and integrates with version control. Urdu localization expands access to 230M+ speakers in Pakistan/India where robotics education is growing rapidly.

---

## Technical Architecture

### Frontend (Docusaurus)

#### Project Structure
```
docs/
├── intro.md (Landing page)
├── setup/
│   ├── workstation.md (Ubuntu + ROS 2 + Isaac Sim)
│   ├── jetson.md (Flashing Jetson Orin, JetPack setup)
│   └── robot.md (Unitree Go2/G1 first connection)
├── module-1-nervous-system/
│   ├── intro.md
│   ├── ros2-basics.md (Nodes, Topics, Services)
│   ├── rclpy-tutorial.md
│   ├── urdf-modeling.md
│   └── exercises/ (5 hands-on challenges)
├── module-2-digital-twin/
├── module-3-brain/
├── module-4-mind-vla/
├── capstone/
│   ├── requirements.md
│   ├── milestones.md
│   └── submission-rubric.md
└── api/ (API reference for custom Python packages)

src/
├── components/
│   ├── RAGChatWidget.tsx (Floating chatbot)
│   ├── AdaptiveContent.tsx (Profile-based rendering)
│   ├── CodePlayground.tsx (Runnable Python snippets)
│   └── RobotVisualizer.tsx (URDF 3D viewer)
├── css/
│   └── custom.css (Tailwind + dark mode overrides)
└── pages/
    ├── index.tsx (Landing page with hero)
    └── dashboard.tsx (User profile + progress)
```

#### Custom Components (React)
1. **RAGChatWidget**: Floating button → opens chat → sends query to FastAPI → displays response with citations
2. **AdaptiveContent**: Wraps content blocks, shows/hides based on `userProfile.type`
3. **CodePlayground**: Embeds Pyodide (Python in browser) for instant code execution
4. **RobotVisualizer**: Uses Three.js + urdf-loader to render URDF files

### Backend (FastAPI + RAG)

#### Project Structure
```
backend/
├── main.py (FastAPI app, CORS, routes)
├── routers/
│   ├── rag.py (/query, /feedback)
│   ├── auth.py (/login, /register, /logout)
│   └── profile.py (/update, /export)
├── services/
│   ├── qdrant_client.py (Vector search)
│   ├── openai_agents.py (RAG orchestration)
│   ├── embeddings.py (Text chunking + embedding)
│   └── moderation.py (Content safety)
├── models/
│   ├── user.py (SQLAlchemy models)
│   └── query.py (Pydantic schemas)
├── database/
│   ├── neon.py (Postgres connection)
│   └── migrations/ (Alembic)
└── scripts/
    ├── index_content.py (CLI: chunk + embed textbook)
    └── deploy_qdrant.py (Initialize Qdrant collections)
```

#### API Endpoints
- `POST /api/rag/query`: Submit question + selected text context
- `POST /api/rag/feedback`: Thumbs up/down on response quality
- `POST /api/auth/login`: Authenticate user (returns JWT)
- `GET /api/profile/me`: Fetch current user profile
- `PATCH /api/profile/update`: Update profile preferences
- `GET /api/profile/export`: Download user data (GDPR compliance)

#### RAG Workflow (OpenAI Agents SDK)
```python
# Pseudo-code
async def answer_query(question: str, context: str, user_profile: dict):
    # 1. Embed query
    query_embedding = await openai.embeddings.create(
        model="text-embedding-3-small",
        input=question
    )

    # 2. Search Qdrant (filter by user's current module)
    results = qdrant.search(
        collection="textbook_chunks",
        query_vector=query_embedding,
        filter={"module_id": user_profile["current_module"]},
        limit=5
    )

    # 3. Build context for LLM
    retrieved_context = "\n\n".join([r.payload["text"] for r in results])

    # 4. OpenAI Agents SDK orchestration
    agent = Agent(
        model="gpt-4-turbo",
        instructions=f"""
        You are a teaching assistant for the Embodied Intelligence course.
        Answer based ONLY on the retrieved textbook content below.
        User background: {user_profile["type"]}

        Retrieved Content:
        {retrieved_context}

        Selected Context: {context}
        """
    )

    response = await agent.run(question)

    # 5. Add citations
    response_with_sources = add_inline_citations(response, results)

    return response_with_sources
```

### Infrastructure

#### Hosting
- **Frontend**: GitHub Pages (static CDN)
- **Backend**: Railway.app or Render.com (free tier → $7/mo for production)
- **Qdrant**: Qdrant Cloud (free 1GB cluster)
- **Neon Postgres**: Free tier (0.5GB storage, sufficient for 10k users)

#### CI/CD (GitHub Actions)
```yaml
# .github/workflows/deploy.yml
name: Deploy Textbook
on:
  push:
    branches: [main]

jobs:
  build-frontend:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: npm ci && npm run build
      - uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build

  deploy-backend:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: |
          docker build -t rag-backend ./backend
          railway up

  reindex-qdrant:
    needs: build-frontend
    runs-on: ubuntu-latest
    steps:
      - run: python backend/scripts/index_content.py --source ./docs
```

---

## Curriculum Standards

### Module Sequencing (IMMUTABLE)

1. **Module 1 (The Nervous System)**:
   - **Duration**: 2 weeks
   - **Prerequisites**: Python basics, Linux command line
   - **Learning Outcomes**: Create ROS 2 nodes, publish/subscribe to topics, visualize data in RViz2, design URDF robot descriptions
   - **Capstone**: Build a simulated mobile robot with differential drive and LiDAR

2. **Module 2 (The Digital Twin)**:
   - **Duration**: 3 weeks
   - **Prerequisites**: Module 1 completed
   - **Learning Outcomes**: Launch Gazebo worlds, spawn URDF models, simulate sensors (camera, depth, IMU), integrate Unity for co-simulation
   - **Capstone**: Simulate a warehouse robot navigating obstacles using sensor fusion

3. **Module 3 (The Brain)**:
   - **Duration**: 4 weeks
   - **Prerequisites**: Modules 1-2 completed, GPU workstation available
   - **Learning Outcomes**: Run NVIDIA Isaac Sim, generate synthetic training data, implement Visual SLAM, configure Nav2 for autonomous navigation
   - **Capstone**: Deploy Nav2 on simulated Unitree Go2 in Isaac Sim (outdoor environment)

4. **Module 4 (The Mind - VLA)**:
   - **Duration**: 4 weeks
   - **Prerequisites**: Modules 1-3 completed
   - **Learning Outcomes**: Fine-tune Vision-Language-Action models, integrate Whisper for voice commands, use LLMs for task planning (ReAct framework), deploy on Jetson Orin
   - **Capstone**: Voice-controlled robot that picks up objects based on natural language instructions

5. **Final Capstone (Autonomous Humanoid)**:
   - **Duration**: 2 weeks
   - **Prerequisites**: All modules completed
   - **Challenge**: Build a system where a simulated humanoid robot (Unitree G1) can:
     - Accept voice commands ("Bring me the red cup")
     - Plan grasp using VLA model
     - Navigate to target using Nav2
     - Execute manipulation with Isaac Sim physics
   - **Submission**: GitHub repository + 5-minute demo video

### Content Depth Requirements

Each module chapter MUST include:
1. **Learning Objectives** (3-5 bullet points, Bloom's Taxonomy verbs: "explain," "implement," "analyze")
2. **Conceptual Overview** (500-800 words, diagrams using Mermaid.js)
3. **Step-by-Step Tutorial** (code blocks with line-by-line annotations)
4. **Interactive Exercise** (CodePlayground component with starter code + solution)
5. **Common Errors** (3-5 typical mistakes with debugging walkthroughs)
6. **Further Reading** (3-5 links to official docs, research papers, or GitHub repos)
7. **Assessment Quiz** (5 questions: 3 multiple-choice, 2 code challenges)

### Code Quality Standards

All code examples MUST:
- Follow ROS 2 style guide (PEP 8 for Python, Google C++ for C++)
- Include docstrings (Google style)
- Pass `ruff` linting (Python) or `clang-format` (C++)
- Be tested in Isaac Sim 4.1+ and ROS 2 Humble
- Include requirements.txt or package.xml
- Provide "Tested On" metadata (Ubuntu version, GPU model, ROS distro)

---

## Content Standards

### Writing Style
- **Voice**: Second person ("You will learn..."), active voice
- **Tone**: Encouraging but technical (avoid condescension like "simply" or "just")
- **Jargon**: Define all acronyms on first use (e.g., "URDF (Unified Robot Description Format)")
- **Sentence Length**: Max 25 words (use tools like Hemingway Editor)

### Visual Standards
- **Diagrams**: Mermaid.js (for architecture), Excalidraw (for hand-drawn style)
- **Screenshots**: 1920x1080 PNG, compress with TinyPNG, annotate with red boxes
- **Videos**: Max 3 minutes, 1080p MP4, host on YouTube (embed in Docusaurus)
- **Alt Text**: All images MUST have descriptive alt text (accessibility)

### Accessibility (WCAG 2.1 AA)
- Color contrast ratio ≥ 4.5:1 (text vs background)
- All interactive elements keyboard-navigable (Tab, Enter)
- Code blocks support screen readers (semantic HTML)
- Videos include closed captions (auto-generated + human-reviewed)

---

## Operational Workflows

### Content Creation Process

1. **Proposal** (GitHub Issue):
   - Title: `[Module X] Chapter: <Title>`
   - Template includes: Learning objectives, outline, prerequisite check

2. **Draft** (Pull Request):
   - Author writes in Markdown, commits to branch `content/module-X/<chapter>`
   - Automated checks: broken links, spell check (Vale linter), plagiarism scan
   - Tag 2 peer reviewers (subject matter experts)

3. **Review**:
   - Reviewers check: Accuracy, clarity, code functionality, alignment with curriculum
   - Must test all code examples on Ubuntu 22.04 + ROS 2 Humble
   - Approval requires 2x LGTM (Looks Good To Me)

4. **Publish**:
   - Merge to `main` → Triggers deployment → Content live within 5 minutes
   - Announce in Discord #textbook-updates channel

### RAG System Maintenance

- **Weekly**: Review RAG feedback (thumbs down responses), improve prompts or add missing content
- **Monthly**: Retrain embeddings if >10% of content updated (Qdrant collection versioning)
- **Quarterly**: Audit for hallucinations (sample 100 random queries, human verification)

### User Feedback Loop

- **In-Chapter**: "Was this helpful?" thumbs up/down (stores in Neon Postgres)
- **Discord Community**: Active forum for questions, moderated by teaching assistants
- **Office Hours**: Bi-weekly live Q&A via Zoom (recorded, added to YouTube)

---

## Governance

### Amendment Process

1. **Proposal**: Submit GitHub Issue with `[Constitution]` tag, describe change rationale
2. **Discussion**: 7-day comment period, open to all contributors
3. **Vote**: Core team (3 maintainers) must unanimously approve
4. **Implementation**: Update constitution.md, increment version (MAJOR: breaking, MINOR: additive, PATCH: clarification)
5. **Migration**: If change affects existing content (e.g., new module added), create migration plan with timeline

### Version History

- **Version 1.0.0**: Initial ratification (2025-12-03)

### Compliance

- All pull requests MUST pass automated constitution checks:
  - Links to relevant curriculum module
  - Code examples follow quality standards
  - Accessibility validators pass (axe-core)
- Manual review ensures pedagogical alignment (cannot be fully automated)

### Conflict Resolution

If principles conflict (e.g., "Hardware Infrastructure" prevents a student from progressing):
1. **Principle Hierarchy**: Educational First-Class Citizenship > Technical mandates
2. **Accommodation**: Provide cloud-based alternatives or simplified paths
3. **Documentation**: Flag as "Advanced Track" vs "Core Track"

---

**Version**: 1.0.0 | **Ratified**: 2025-12-03 | **Last Amended**: 2025-12-03

---

*This constitution is a living document. Propose amendments via GitHub Issues tagged `[Constitution]`.*
