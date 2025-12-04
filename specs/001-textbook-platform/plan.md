# Implementation Plan: Physical AI & Humanoid Robotics Textbook Platform

**Branch**: `001-textbook-platform` | **Date**: 2025-12-03 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-textbook-platform/spec.md`

---

## Summary

This plan outlines the implementation strategy for the Physical AI & Humanoid Robotics Textbook — a hybrid application combining a static Docusaurus frontend (GitHub Pages) with a dynamic FastAPI backend (localhost) for RAG chatbot and authentication features. The architecture decouples the static content delivery layer from the intelligent backend services, enabling independent development and deployment.

**Key Architecture Decisions**:
- Monorepo structure with `/docs-website` (Docusaurus TypeScript), `/rag-backend` (Python FastAPI), `/.claude` (AI skills)
- Frontend: Static site generation with GitHub Pages deployment
- Backend: Localhost FastAPI with CORS for cross-origin requests
- Integration: Configurable `API_BASE_URL` environment variable
- Content: Pre-built Urdu translations, one-time RAG ingestion

---

## Technical Context

**Language/Version**: TypeScript 5.x (frontend), Python 3.11+ (backend)
**Primary Dependencies**: Docusaurus 3.x, React 18+, FastAPI 0.100+, OpenAI Agents SDK, Qdrant Client, Better-Auth
**Storage**: Qdrant Cloud (vector DB, 1GB free tier), Neon Serverless Postgres (relational DB, 0.5GB free tier)
**Testing**: Jest + React Testing Library (frontend), pytest + httpx (backend)
**Target Platform**: GitHub Pages (frontend static hosting), Localhost (backend development)
**Project Type**: Hybrid web application (static frontend + dynamic backend API)
**Performance Goals**: <2s page load (static), <3s RAG response time (p95), <5min GitHub Actions build
**Constraints**: Localhost backend (RAG features require local server running), GitHub Pages 1GB size limit, Qdrant free tier 1GB storage
**Scale/Scope**: 50+ chapters across 5 modules, 500 concurrent static readers, 50 concurrent RAG users (localhost limit)

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ Principle I: Curriculum-Driven Architecture
- **Gate**: All 5 modules must have dedicated directory structure in `/docs-website/docs/`
- **Verification**: Each module contains intro.md, exercises/, and tutorial chapters
- **Status**: PASS - Architecture supports modular curriculum structure

### ✅ Principle II: Technology Stack Mandate
- **Gate**: Docusaurus 3.x (TypeScript), FastAPI (Python), Qdrant Cloud, Neon Postgres, Better-Auth
- **Verification**: package.json specifies `docusaurus@^3.0.0`, pyproject.toml specifies `fastapi>=0.100.0`
- **Status**: PASS - All mandated technologies included

### ✅ Principle III: Hardware Infrastructure Assumptions
- **Gate**: Content must assume RTX 4070 Ti, Jetson Orin, Unitree robots
- **Verification**: Module 0 setup guide documents hardware requirements
- **Status**: PASS - Hardware assumptions documented

### ✅ Principle IV: Pedagogical Principles
- **Gate**: Progressive disclosure (crawl→walk→run), hands-on first, error-driven learning
- **Verification**: Chapter template includes learning objectives, tutorial, exercises, common errors
- **Status**: PASS - Template enforces pedagogical structure

### ✅ Principle V: RAG Chatbot Integration (MANDATORY)
- **Gate**: Context-aware Q&A with source citations
- **Verification**: FastAPI `/api/rag/query` endpoint, Qdrant semantic search, OpenAI Agents SDK
- **Status**: PASS - RAG architecture defined

### ✅ Principle VI: Personalization System
- **Gate**: User profiling (Software vs Hardware Engineer) with adaptive content
- **Verification**: Neon Postgres user profiles, React `<AdaptiveContent>` components
- **Status**: PASS - Personalization system planned

### ✅ Principle VII: Security and Ethical AI
- **Gate**: GDPR compliance, AI safety constraints, open source licensing
- **Verification**: Data export endpoint, adversarial query filtering, rate limiting (100 queries/hour)
- **Status**: PASS - Security measures defined

### ✅ Principle VIII: Deployment and Localization
- **Gate**: GitHub Pages deployment, Urdu localization
- **Verification**: GitHub Actions workflow, `i18n/ur/` directory, translation script
- **Status**: PASS - Deployment and localization planned

---

## Project Structure

### Documentation (this feature)

```text
specs/001-textbook-platform/
├── spec.md              # Feature specification
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (technical research)
├── data-model.md        # Phase 1 output (database schemas)
├── contracts/           # Phase 1 output (API contracts)
│   ├── rag-api.yaml     # OpenAPI spec for RAG endpoints
│   └── auth-api.yaml    # OpenAPI spec for auth endpoints
└── quickstart.md        # Phase 1 output (developer setup guide)
```

### Source Code (repository root)

```text
# Monorepo structure (hybrid web application)
/
├── docs-website/          # Docusaurus frontend (TypeScript)
│   ├── docs/              # Markdown content (5 modules)
│   │   ├── intro.md       # Landing page content
│   │   ├── setup/         # Module 0: Hardware setup
│   │   ├── module-1-nervous-system/  # ROS 2 fundamentals
│   │   ├── module-2-digital-twin/    # Gazebo/Unity simulation
│   │   ├── module-3-brain/           # Isaac Sim/Nav2
│   │   ├── module-4-mind-vla/        # VLA models
│   │   └── capstone/                 # Autonomous humanoid project
│   ├── src/               # React components
│   │   ├── components/
│   │   │   ├── RAGChatWidget.tsx     # Floating chatbot
│   │   │   ├── AdaptiveContent.tsx   # Personalization wrapper
│   │   │   ├── CodePlayground.tsx    # Pyodide code runner
│   │   │   └── RobotVisualizer.tsx   # URDF 3D viewer
│   │   ├── pages/
│   │   │   ├── index.tsx             # Custom landing page
│   │   │   └── dashboard.tsx         # User profile/progress
│   │   └── css/
│   │       └── custom.css            # Tailwind overrides
│   ├── i18n/ur/           # Urdu translations (pre-built)
│   ├── static/            # Images, videos, assets
│   ├── docusaurus.config.ts  # Site configuration
│   ├── tsconfig.json      # TypeScript config
│   ├── package.json       # npm dependencies
│   └── .env.local         # API_BASE_URL config
│
├── rag-backend/           # FastAPI backend (Python)
│   ├── main.py            # FastAPI app entrypoint
│   ├── routers/           # API endpoint groups
│   │   ├── rag.py         # /api/rag/* endpoints
│   │   ├── auth.py        # /api/auth/* endpoints
│   │   └── profile.py     # /api/profile/* endpoints
│   ├── services/          # Business logic
│   │   ├── qdrant_client.py      # Vector search
│   │   ├── openai_agents.py      # RAG orchestration
│   │   ├── embeddings.py         # Text chunking + embeddings
│   │   ├── moderation.py         # Content safety
│   │   └── auth_service.py       # Token verification
│   ├── models/            # Data models
│   │   ├── user.py        # SQLAlchemy User model
│   │   ├── query.py       # Pydantic request/response schemas
│   │   └── progress.py    # Progress tracking model
│   ├── database/          # Database connections
│   │   ├── neon.py        # Neon Postgres connection
│   │   └── migrations/    # Alembic migration scripts
│   ├── scripts/           # CLI utilities
│   │   ├── index_content.py      # RAG ingestion script
│   │   ├── translate_to_urdu.py  # Urdu translation script
│   │   └── init_db.py            # Database initialization
│   ├── tests/             # pytest tests
│   │   ├── test_rag.py
│   │   ├── test_auth.py
│   │   └── conftest.py    # pytest fixtures
│   ├── Dockerfile         # Container for deployment
│   ├── pyproject.toml     # Poetry dependencies
│   ├── .env.example       # Environment variable template
│   └── README.md          # Backend setup instructions
│
├── .claude/               # Claude Code skills (reusable AI tools)
│   ├── skills/
│   │   ├── ros2-node-generator.md  # ROS 2 node scaffolding skill
│   │   └── README.md               # Skills documentation
│   └── .claud

erc                   # Claude configuration
│
├── .github/
│   └── workflows/
│       ├── deploy-docs.yml         # Build + deploy Docusaurus to GitHub Pages
│       └── test-backend.yml        # Run pytest on rag-backend
│
├── .gitignore
├── README.md              # Project overview + setup instructions
└── LICENSE                # CC BY-NC-SA 4.0 (content) + MIT (code)
```

**Structure Decision**: Monorepo with clear separation between static frontend (`docs-website/`) and dynamic backend (`rag-backend/`). This enables independent development: content writers work in `/docs-website/docs/`, backend engineers work in `/rag-backend/`, and deployment pipelines are decoupled (GitHub Pages for frontend, localhost for backend during development).

---

## Complexity Tracking

No constitutional violations requiring justification. All architecture decisions align with constitution principles.

---

## Phase 0: Outline & Research

### Research Tasks

#### R1: Docusaurus 3.x Best Practices for Educational Content
**Question**: What is the optimal Docusaurus configuration for a technical textbook with 50+ chapters?
**Research Areas**:
- Sidebar auto-generation strategies (filesystem vs manual)
- Plugin recommendations for code playgrounds (Pyodide integration)
- Performance optimization for large sites (lazy loading, image compression)
- Swizzling best practices for custom NavBar and Root components

**Findings** (to be captured in `research.md`):
- Use `autogenerated` sidebars with `dirName` for automatic chapter discovery
- Install `@docusaurus/theme-live-codeblock` for interactive code examples
- Enable `docs-only` mode to remove blog/landing page overhead
- Swizzle `NavbarContent` for custom chatbot button, avoid swizzling `Root` (breaks updates)
- Use `docusaurus-plugin-image-zoom` for diagram clarity

---

#### R2: FastAPI + Qdrant + OpenAI Agents SDK Integration Patterns
**Question**: How to architect a RAG system using OpenAI Agents SDK with Qdrant vector search?
**Research Areas**:
- OpenAI Agents SDK threading model (how to maintain conversation context)
- Qdrant client async patterns (connection pooling, retry logic)
- Embedding generation best practices (batch processing, caching)
- CORS configuration for localhost + GitHub Pages origins

**Findings** (to be captured in `research.md`):
- Use `openai.beta.assistants` API with file uploads for textbook content
- Qdrant `AsyncQdrantClient` with connection reuse via FastAPI dependency injection
- Batch embeddings in groups of 100 chunks to optimize OpenAI API calls
- CORS middleware: `CORSMiddleware(allow_origins=["http://localhost:3000", "https://<username>.github.io"])`

---

#### R3: Better-Auth Client-Side OAuth Flow for Static Sites
**Question**: How to implement Better-Auth OAuth in a React app without server-side middleware?
**Research Areas**:
- Better-Auth React SDK usage patterns
- Token exchange flow (OAuth callback → FastAPI verification → session cookie)
- Session persistence strategies (HTTP-only cookies vs localStorage)
- Email verification workflow

**Findings** (to be captured in `research.md`):
- Better-Auth provides `@better-auth/react` package with `useAuth()` hook
- OAuth flow: React SDK handles redirect → receives token → sends to FastAPI `/api/auth/verify-token`
- FastAPI verifies token, creates user in Neon Postgres, returns `Set-Cookie` header (HTTP-only, 30-day expiry)
- Email verification via Better-Auth webhook → updates `users.email_verified` in Neon

---

#### R4: Docusaurus i18n for Urdu (RTL Language Support)
**Question**: How to configure Docusaurus for Urdu localization with RTL text direction?
**Research Areas**:
- Docusaurus i18n configuration for non-Latin scripts
- RTL CSS handling (automatic or manual)
- Translation file structure and build process
- Language switcher component implementation

**Findings** (to be captured in `research.md`):
- Add to `docusaurus.config.ts`: `i18n: { defaultLocale: 'en', locales: ['en', 'ur'], localeConfigs: { ur: { direction: 'rtl' } } }`
- Translation files in `i18n/ur/docusaurus-plugin-content-docs/current/`
- Docusaurus auto-handles RTL CSS flipping for UI elements
- Custom language switcher: `<LanguageSwitcher />` component in swizzled NavBar

---

#### R5: Monorepo Tooling and Workspace Configuration
**Question**: What is the best tooling setup for a TypeScript/Python monorepo with independent deployments?
**Research Areas**:
- npm workspaces vs pnpm workspaces for frontend
- Poetry dependency management for Python backend
- Shared linting/formatting configurations (ESLint, Prettier, ruff, Black)
- CI/CD pipeline strategies for multi-environment monorepo

**Findings** (to be captured in `research.md`):
- Use npm workspaces (built-in, no additional tooling)
- Root `package.json` with `workspaces: ["docs-website"]`
- Poetry for Python (modern, lock file support)
- Shared configs: root `.eslintrc.js`, `.prettierrc`, `pyproject.toml` (ruff + black)
- GitHub Actions: separate workflows for frontend (`deploy-docs.yml`) and backend (`test-backend.yml`)

---

### Output: `research.md`

Generated in Phase 0, consolidates findings from R1-R5 with decisions, rationales, and alternatives considered.

---

## Phase 1: Design & Contracts

### Prerequisites
- `research.md` complete with all technical unknowns resolved

### Phase 1A: Data Model Design

**File**: `data-model.md`

#### Entity: User
```typescript
// Neon Postgres schema (SQLAlchemy model in backend)
interface User {
  id: UUID;                    // Primary key
  email: string;               // Unique, indexed
  password_hash: string;       // bcrypt hashed
  profile_type: 'Software Engineer' | 'Hardware Engineer' | 'Student' | 'Hobbyist';
  experience_level: 'None' | 'Beginner' | 'Intermediate' | 'Expert';
  primary_goal: 'Learn AI' | 'Build Robots' | 'Research' | 'Career Transition';
  current_module: number;      // 1-5, tracks progress
  email_verified: boolean;
  created_at: timestamp;
  updated_at: timestamp;
}

// Indexes
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_profile_type ON users(profile_type);
```

---

#### Entity: RAGQuery
```typescript
// Neon Postgres schema
interface RAGQuery {
  id: UUID;                    // Primary key
  user_id: UUID;               // Foreign key → User.id
  question_text: string;       // Max 300 words (~1500 chars)
  selected_context: string;    // Highlighted text from textbook
  response_text: string;       // Max 300 words
  sources: JSON;               // Array of citation links [{title, url}]
  feedback: 'thumbs_up' | 'thumbs_down' | null;
  created_at: timestamp;
  anonymized_at: timestamp;    // Set 90 days after created_at
}

// Indexes
CREATE INDEX idx_rag_queries_user_id ON rag_queries(user_id);
CREATE INDEX idx_rag_queries_created_at ON rag_queries(created_at);
```

---

#### Entity: TextbookChunk (Qdrant Vector DB)
```typescript
// Qdrant collection: textbook_chunks
interface TextbookChunk {
  id: UUID;                    // Qdrant point ID
  vector: number[];            // 1536 dimensions (text-embedding-3-small)
  payload: {
    text: string;              // 500 tokens chunk
    module_id: number;         // 1-5
    chapter_id: string;        // e.g., "ros2-basics"
    code_block: boolean;       // true if chunk contains code
    difficulty_level: 'Beginner' | 'Intermediate' | 'Advanced';
    source_file: string;       // e.g., "module-1-nervous-system/ros2-basics.md"
  };
}

// Qdrant index configuration
collection_config = {
  vectors: {
    size: 1536,
    distance: "Cosine"
  },
  optimizers_config: {
    indexing_threshold: 10000  // Build HNSW index after 10k points
  }
}
```

---

#### Entity: ProgressTracker
```typescript
// Neon Postgres schema
interface ProgressTracker {
  id: UUID;
  user_id: UUID;               // Foreign key → User.id
  module_id: number;           // 1-5
  chapter_id: string;          // e.g., "ros2-basics"
  quiz_passed: boolean;
  completed_at: timestamp;
}

// Indexes
CREATE INDEX idx_progress_user_id ON progress_tracker(user_id);
CREATE INDEX idx_progress_module_id ON progress_tracker(module_id);
```

---

### Phase 1B: API Contracts

**Directory**: `contracts/`

#### File: `rag-api.yaml` (OpenAPI 3.0)

```yaml
openapi: 3.0.3
info:
  title: RAG Chatbot API
  version: 1.0.0
  description: Retrieval-Augmented Generation endpoints for textbook Q&A

servers:
  - url: http://localhost:8000
    description: Local development server

paths:
  /api/rag/query:
    post:
      summary: Submit RAG query
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              properties:
                question:
                  type: string
                  maxLength: 1500
                  description: User's question (max 300 words)
                context:
                  type: string
                  description: Selected text from textbook (optional)
                module_id:
                  type: integer
                  minimum: 1
                  maximum: 5
                  description: Current module for filtering
              required:
                - question
      responses:
        '200':
          description: RAG response with citations
          content:
            application/json:
              schema:
                type: object
                properties:
                  response:
                    type: string
                    description: Answer text (max 300 words)
                  sources:
                    type: array
                    items:
                      type: object
                      properties:
                        title:
                          type: string
                        url:
                          type: string
                  query_id:
                    type: string
                    format: uuid
        '400':
          description: Invalid input (question too long, malformed)
        '429':
          description: Rate limit exceeded (100 queries/hour)
        '500':
          description: Server error (Qdrant unavailable, OpenAI error)

  /api/rag/feedback:
    post:
      summary: Submit feedback on RAG response
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              properties:
                query_id:
                  type: string
                  format: uuid
                feedback:
                  type: string
                  enum: [thumbs_up, thumbs_down]
              required:
                - query_id
                - feedback
      responses:
        '200':
          description: Feedback recorded
        '404':
          description: Query ID not found
```

---

#### File: `auth-api.yaml` (OpenAPI 3.0)

```yaml
openapi: 3.0.3
info:
  title: Authentication API
  version: 1.0.0
  description: Better-Auth integration endpoints

paths:
  /api/auth/verify-token:
    post:
      summary: Verify OAuth token and create session
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              properties:
                token:
                  type: string
                  description: OAuth token from Better-Auth SDK
                provider:
                  type: string
                  enum: [email, github]
              required:
                - token
                - provider
      responses:
        '200':
          description: Session created
          headers:
            Set-Cookie:
              schema:
                type: string
                example: auth_token=abc123; HttpOnly; Secure; SameSite=Strict; Max-Age=2592000
          content:
            application/json:
              schema:
                type: object
                properties:
                  user_id:
                    type: string
                    format: uuid
                  email:
                    type: string
                  email_verified:
                    type: boolean
        '401':
          description: Invalid token
        '500':
          description: Server error

  /api/auth/session:
    get:
      summary: Get current user session
      security:
        - cookieAuth: []
      responses:
        '200':
          description: User session data
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/User'
        '401':
          description: Not authenticated

  /api/profile/me:
    get:
      summary: Get current user profile
      security:
        - cookieAuth: []
      responses:
        '200':
          description: User profile
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/UserProfile'

  /api/profile/update:
    patch:
      summary: Update user profile
      security:
        - cookieAuth: []
      requestBody:
        content:
          application/json:
            schema:
              type: object
              properties:
                profile_type:
                  type: string
                  enum: [Software Engineer, Hardware Engineer, Student, Hobbyist]
                experience_level:
                  type: string
                  enum: [None, Beginner, Intermediate, Expert]
                primary_goal:
                  type: string
                  enum: [Learn AI, Build Robots, Research, Career Transition]
      responses:
        '200':
          description: Profile updated

  /api/profile/export:
    get:
      summary: Export user data (GDPR compliance)
      security:
        - cookieAuth: []
      responses:
        '200':
          description: User data JSON
          content:
            application/json:
              schema:
                type: object
                properties:
                  user:
                    $ref: '#/components/schemas/User'
                  queries:
                    type: array
                    items:
                      $ref: '#/components/schemas/RAGQuery'
                  progress:
                    type: array
                    items:
                      type: object

components:
  securitySchemes:
    cookieAuth:
      type: apiKey
      in: cookie
      name: auth_token

  schemas:
    User:
      type: object
      properties:
        id:
          type: string
          format: uuid
        email:
          type: string
        profile_type:
          type: string
        experience_level:
          type: string
        primary_goal:
          type: string
        current_module:
          type: integer
        email_verified:
          type: boolean

    UserProfile:
      allOf:
        - $ref: '#/components/schemas/User'
        - type: object
          properties:
            progress:
              type: object
              properties:
                modules_completed:
                  type: integer
                quizzes_passed:
                  type: integer

    RAGQuery:
      type: object
      properties:
        id:
          type: string
          format: uuid
        question_text:
          type: string
        response_text:
          type: string
        sources:
          type: array
        feedback:
          type: string
        created_at:
          type: string
          format: date-time
```

---

### Phase 1C: Quickstart Guide

**File**: `quickstart.md`

# Developer Quickstart Guide

## Prerequisites

- **Node.js**: v18+ ([Download](https://nodejs.org/))
- **Python**: 3.11+ ([Download](https://www.python.org/))
- **Git**: Latest version
- **Docker** (optional): For containerized backend

## Environment Setup

### 1. Clone Repository

```bash
git clone https://github.com/<username>/embodied-ai-textbook.git
cd embodied-ai-textbook
```

### 2. Frontend Setup (Docusaurus)

```bash
cd docs-website
npm install

# Create .env.local for API configuration
cat > .env.local <<EOF
REACT_APP_API_BASE_URL=http://localhost:8000
EOF

# Start development server
npm start
# Visit http://localhost:3000
```

### 3. Backend Setup (FastAPI)

```bash
cd ../rag-backend

# Install Poetry (if not installed)
curl -sSL https://install.python-poetry.org | python3 -

# Install dependencies
poetry install

# Create .env file with secrets
cp .env.example .env
# Edit .env and add:
#   QDRANT_URL=https://your-cluster.qdrant.io
#   QDRANT_API_KEY=your_api_key
#   NEON_DB_URL=postgresql://user:pass@host/db
#   OPENAI_API_KEY=sk-...
#   BETTER_AUTH_SECRET=your_secret

# Initialize database
poetry run python scripts/init_db.py

# Run RAG content ingestion (one-time)
poetry run python scripts/index_content.py --source ../docs-website/docs

# Start backend server
poetry run uvicorn main:app --reload --port 8000
# Visit http://localhost:8000/docs (Swagger UI)
```

### 4. Verify Integration

1. Open frontend: `http://localhost:3000`
2. Navigate to any chapter
3. Click floating "Ask the Book" button (bottom-right)
4. Submit a test query: "What is a ROS 2 node?"
5. Verify response with citations appears

## Development Workflow

### Adding New Content

```bash
# 1. Create new chapter Markdown file
cd docs-website/docs/module-1-nervous-system
touch new-chapter.md

# 2. Write content following template
# 3. Re-run RAG ingestion (if adding substantial content)
cd ../../rag-backend
poetry run python scripts/index_content.py --source ../docs-website/docs

# 4. Test locally
# Frontend auto-reloads on file save
```

### Translating to Urdu

```bash
cd rag-backend
poetry run python scripts/translate_to_urdu.py \
  --source ../docs-website/docs/module-1-nervous-system/ros2-basics.md \
  --output ../docs-website/i18n/ur/docusaurus-plugin-content-docs/module-1-nervous-system/ros2-basics.md

# Rebuild frontend to generate Urdu static pages
cd ../docs-website
npm run build
```

### Running Tests

```bash
# Frontend tests
cd docs-website
npm test

# Backend tests
cd ../rag-backend
poetry run pytest tests/ -v
```

## Deployment

### Frontend (GitHub Pages)

```bash
cd docs-website
npm run build
# Output in build/ directory

# Push to GitHub → GitHub Actions deploys automatically
git add .
git commit -m "Update content"
git push origin main
```

### Backend (Localhost Only)

Backend runs locally during development. For production deployment (optional):

```bash
cd rag-backend
docker build -t rag-backend .
docker run -p 8000:8000 --env-file .env rag-backend
```

## Common Issues

### Issue: RAG chatbot not responding
**Solution**: Verify backend is running on `http://localhost:8000` and CORS is configured

### Issue: Qdrant connection error
**Solution**: Check `QDRANT_URL` and `QDRANT_API_KEY` in `.env` file

### Issue: Better-Auth login fails
**Solution**: Verify `BETTER_AUTH_SECRET` matches between frontend and backend configs

---

## Phase 2: Skeleton Implementation

### Phase 2A: Create Directory Structure

**Commands to execute**:

```bash
# Create monorepo structure
mkdir -p docs-website
mkdir -p rag-backend
mkdir -p .claude/skills

# Frontend directories
mkdir -p docs-website/docs/{setup,module-1-nervous-system,module-2-digital-twin,module-3-brain,module-4-mind-vla,capstone}
mkdir -p docs-website/src/{components,pages,css}
mkdir -p docs-website/static/{img,videos}
mkdir -p docs-website/i18n/ur/docusaurus-plugin-content-docs

# Backend directories
mkdir -p rag-backend/{routers,services,models,database,scripts,tests}
mkdir -p rag-backend/database/migrations
mkdir -p rag-backend/contracts

# Verification
tree -L 2 -d
```

---

### Phase 2B: Initialize Frontend (Docusaurus)

```bash
cd docs-website

# Initialize Docusaurus with TypeScript
npx create-docusaurus@latest . classic --typescript

# Install additional dependencies
npm install --save \
  @docusaurus/theme-live-codeblock \
  docusaurus-plugin-image-zoom \
  @better-auth/react \
  axios \
  tailwindcss \
  postcss \
  autoprefixer

# Purge default bloat
rm -rf blog/
rm -rf src/pages/markdown-page.md
rm -rf docs/tutorial-*

# Configure Tailwind CSS
npx tailwindcss init -p
```

**Edit `docusaurus.config.ts`**:

```typescript
import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Bridge AI to Robots: Master Embodied Intelligence',
  favicon: 'img/favicon.ico',

  url: 'https://<username>.github.io',
  baseUrl: '/embodied-ai-textbook/',

  organizationName: '<username>',
  projectName: 'embodied-ai-textbook',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      ur: {
        direction: 'rtl',
        label: 'اردو',
      },
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          routeBasePath: '/docs',
          sidebarPath: './sidebars.ts',
        },
        blog: false, // Disable blog
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],

  plugins: [
    '@docusaurus/theme-live-codeblock',
    'docusaurus-plugin-image-zoom',
  ],

  themeConfig: {
    navbar: {
      title: 'Physical AI Textbook',
      logo: {
        alt: 'Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Modules',
        },
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          href: 'https://github.com/<username>/embodied-ai-textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml'],
    },
  },
};

export default config;
```

---

### Phase 2C: Initialize Backend (FastAPI)

```bash
cd rag-backend

# Initialize Poetry project
poetry init --name rag-backend --description "RAG Chatbot Backend" --author "Your Name <email@example.com>" --python "^3.11"

# Add dependencies
poetry add fastapi uvicorn[standard] python-multipart
poetry add qdrant-client openai sqlalchemy psycopg2-binary alembic
poetry add python-jose[cryptography] passlib[bcrypt] python-dotenv
poetry add httpx pytest pytest-asyncio

# Create main.py
cat > main.py <<'EOF'
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import os
from dotenv import load_dotenv

load_dotenv()

app = FastAPI(title="RAG Chatbot API", version="1.0.0")

# CORS configuration
origins = [
    "http://localhost:3000",  # Local Docusaurus dev
    f"https://{os.getenv('GITHUB_USERNAME')}.github.io",  # GitHub Pages
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
    return {"message": "RAG Chatbot API", "status": "running"}

@app.get("/health")
async def health():
    return {"status": "ok"}

# Import routers (to be created)
# from routers import rag, auth, profile
# app.include_router(rag.router, prefix="/api/rag", tags=["RAG"])
# app.include_router(auth.router, prefix="/api/auth", tags=["Auth"])
# app.include_router(profile.router, prefix="/api/profile", tags=["Profile"])
EOF

# Create .env.example
cat > .env.example <<'EOF'
# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_api_key

# Neon Serverless Postgres
NEON_DB_URL=postgresql://user:password@host/database

# OpenAI API
OPENAI_API_KEY=sk-...

# Better-Auth
BETTER_AUTH_SECRET=your_secret_key

# GitHub (for CORS)
GITHUB_USERNAME=your-github-username
EOF

# Create Dockerfile
cat > Dockerfile <<'EOF'
FROM python:3.11-slim

WORKDIR /app

# Install Poetry
RUN pip install poetry

# Copy dependency files
COPY pyproject.toml poetry.lock ./

# Install dependencies
RUN poetry config virtualenvs.create false && poetry install --no-dev

# Copy application code
COPY . .

# Expose port
EXPOSE 8000

# Run FastAPI
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
EOF

# Test backend
poetry run uvicorn main:app --reload --port 8000
```

---

### Phase 2D: Create GitHub Actions Workflow

```bash
mkdir -p .github/workflows

# Frontend deployment workflow
cat > .github/workflows/deploy-docs.yml <<'EOF'
name: Deploy Docusaurus to GitHub Pages

on:
  push:
    branches:
      - main
    paths:
      - 'docs-website/**'
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: 'npm'
          cache-dependency-path: docs-website/package-lock.json

      - name: Install dependencies
        working-directory: docs-website
        run: npm ci

      - name: Build Docusaurus
        working-directory: docs-website
        run: npm run build

      - name: Upload artifact
        uses: actions/upload-pages-artifact@v3
        with:
          path: docs-website/build

  deploy:
    needs: build
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
EOF

# Backend testing workflow
cat > .github/workflows/test-backend.yml <<'EOF'
name: Test FastAPI Backend

on:
  push:
    branches:
      - main
    paths:
      - 'rag-backend/**'
  pull_request:
    paths:
      - 'rag-backend/**'

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Setup Python
        uses: actions/setup-python@v5
        with:
          python-version: '3.11'

      - name: Install Poetry
        run: |
          curl -sSL https://install.python-poetry.org | python3 -
          echo "$HOME/.local/bin" >> $GITHUB_PATH

      - name: Install dependencies
        working-directory: rag-backend
        run: poetry install

      - name: Run pytest
        working-directory: rag-backend
        run: poetry run pytest tests/ -v --cov=.
EOF
```

---

### Phase 2E: Create Claude Code Skill

```bash
cat > .claude/skills/ros2-node-generator.md <<'EOF'
# ROS 2 Node Generator Skill

This skill generates complete ROS 2 nodes (Python or C++) with package configuration.

## Usage

When invoked, Claude will ask for:
1. **Node name** (e.g., "velocity_publisher")
2. **Language** (Python or C++)
3. **Topic name** (e.g., "/cmd_vel")
4. **Message type** (e.g., "geometry_msgs/Twist")

## Output

Generates:
- Complete node source code (Python `rclpy` or C++ `rclcpp`)
- `package.xml` with correct dependencies
- `CMakeLists.txt` (for C++ only)
- `README.md` with build/run instructions
- "Tested On" metadata (ROS 2 Humble, Ubuntu 22.04)

## Example

**Input**:
- Node name: velocity_publisher
- Language: Python
- Topic: /cmd_vel
- Message type: geometry_msgs/Twist

**Output**: `velocity_publisher.py`

```python
#!/usr/bin/env python3
"""
Velocity Publisher Node
Publishes Twist messages to /cmd_vel topic

Tested On: ROS 2 Humble, Ubuntu 22.04
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Velocity Publisher node started')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0  # Move forward at 1 m/s
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: linear.x={msg.linear.x}, angular.z={msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Output**: `package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>velocity_publisher</name>
  <version>1.0.0</version>
  <description>Publishes Twist messages to /cmd_vel</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Output**: `README.md`

```markdown
# Velocity Publisher Node

ROS 2 node that publishes Twist messages to `/cmd_vel` topic.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select velocity_publisher
source install/setup.bash
```

## Run

```bash
ros2 run velocity_publisher velocity_publisher
```

## Verify

```bash
# In another terminal
ros2 topic list
ros2 topic echo /cmd_vel
```

**Tested On**: ROS 2 Humble, Ubuntu 22.04, Python 3.10
```

## Implementation Notes

- For Python nodes: Use `rclpy`, follow PEP 8
- For C++ nodes: Use `rclcpp`, follow Google C++ Style Guide
- Always include docstrings and comments
- Generate `setup.py` for Python packages
- Generate `CMakeLists.txt` for C++ packages
- Include error handling and logging
EOF

# Create skills README
cat > .claude/skills/README.md <<'EOF'
# Claude Code Skills

Reusable AI tools for development tasks.

## Available Skills

### ros2-node-generator.md
Generates complete ROS 2 nodes (Python/C++) with package configuration.

**Usage**: `/skill ros2-node-generator`

**Prompts for**: Node name, language, topic, message type

**Generates**: Node source code, package.xml, CMakeLists.txt, README

## Adding New Skills

1. Create `<skill-name>.md` in `.claude/skills/`
2. Document usage, inputs, outputs, examples
3. Update this README
EOF
```

---

## Phase 3: Core Content Implementation

### Phase 3A: Create Module Structure

```bash
cd docs-website/docs

# Module 0: Setup
cat > setup/workstation.md <<'EOF'
---
id: workstation-setup
title: Workstation Setup
sidebar_label: Workstation
---

# Workstation Setup

This guide walks you through setting up your development workstation for Physical AI development.

## Hardware Requirements

### Minimum Specifications (Required)
- **GPU**: NVIDIA RTX 4070 Ti (12GB VRAM) or higher
- **CPU**: 8-core x86_64 processor (AMD Ryzen 7 / Intel Core i7)
- **RAM**: 32GB DDR4
- **Storage**: 500GB NVMe SSD (for Isaac Sim + ROS 2 workspaces)
- **OS**: Ubuntu 22.04 LTS (native or dual-boot, **not** WSL)

### Recommended Specifications
- **GPU**: NVIDIA RTX 4090 (24GB VRAM) for faster Isaac Sim rendering
- **RAM**: 64GB for running multiple simulations
- **Storage**: 1TB NVMe SSD

## Why Ubuntu 22.04?

- **ROS 2 Humble**: LTS release officially supported on Ubuntu 22.04
- **NVIDIA Isaac Sim**: Requires native Linux with direct GPU access
- **Driver Compatibility**: Best NVIDIA driver support on Ubuntu

## Installation Steps

### 1. Install Ubuntu 22.04 LTS

[Download Ubuntu 22.04 LTS](https://ubuntu.com/download/desktop)

Follow the installation wizard. **Important**: Choose "Install alongside Windows" if dual-booting.

### 2. Install NVIDIA Drivers

```bash
# Add graphics drivers PPA
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install recommended driver (usually 535+)
ubuntu-drivers devices
sudo ubuntu-drivers autoinstall

# Reboot
sudo reboot

# Verify installation
nvidia-smi
```

Expected output:
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.129.03   Driver Version: 535.129.03   CUDA Version: 12.2   |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
| 30%   45C    P8    25W / 320W |   1024MiB / 12288MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+
```

### 3. Install ROS 2 Humble

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# Source ROS 2 in bash profile
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
```

Expected output: `ros2 cli version: 0.18.x`

### 4. Install Python Development Tools

```bash
# Python 3.10 (pre-installed on Ubuntu 22.04)
python3 --version  # Should show 3.10.x

# Install pip and virtualenv
sudo apt install python3-pip python3-venv

# Install development tools
pip3 install --user black ruff pytest
```

### 5. Install Visual Studio Code

```bash
# Download .deb package
wget -O code.deb https://go.microsoft.com/fwlink/?LinkID=760868

# Install
sudo apt install ./code.deb

# Install ROS extension
code --install-extension ms-iot.vscode-ros
```

## Verification Checklist

- [ ] `nvidia-smi` shows GPU (RTX 4070 Ti+)
- [ ] `ros2 --version` shows Humble
- [ ] `python3 --version` shows 3.10.x
- [ ] `code --version` shows VS Code installed

## Next Steps

→ [Jetson Orin Setup](./jetson.md)
→ [Module 1: ROS 2 Nervous System](../module-1-nervous-system/intro.md)

---

**Tested On**: Ubuntu 22.04 LTS, RTX 4070 Ti, ROS 2 Humble
EOF

# Create placeholder files for other setup guides
touch setup/jetson.md
touch setup/robot.md

# Module 1 placeholder
cat > module-1-nervous-system/intro.md <<'EOF'
---
id: module-1-intro
title: Module 1 - The Nervous System
sidebar_label: Introduction
---

# Module 1: The Nervous System (ROS 2 Fundamentals)

Welcome to Module 1! In this module, you'll learn the foundational communication patterns that allow robots to perceive and act in the world.

## Learning Objectives

By the end of this module, you will be able to:
- **Explain** the ROS 2 node-topic-service architecture
- **Implement** publisher and subscriber nodes in Python (rclpy)
- **Design** URDF robot descriptions for custom robots
- **Visualize** sensor data in RViz2
- **Debug** ROS 2 communication issues using CLI tools

## Module Structure

### Chapters
1. [ROS 2 Basics](./ros2-basics.md) - Nodes, Topics, Services
2. [rclpy Tutorial](./rclpy-tutorial.md) - Python ROS 2 programming
3. [URDF Modeling](./urdf-modeling.md) - Robot description files
4. [RViz2 Visualization](./rviz2.md) - Visualizing robot state

### Exercises
- Exercise 1: Create a publisher-subscriber pair
- Exercise 2: Implement a service client
- Exercise 3: Design a URDF model for a mobile robot
- Exercise 4: Visualize LiDAR data in RViz2
- Exercise 5: Build a teleoperation node

### Capstone
Build a simulated mobile robot with differential drive and LiDAR that can be controlled via keyboard teleop.

## Prerequisites

- Python basics (functions, classes, async/await)
- Linux command line (bash, file system navigation)
- Workstation setup complete ([Setup Guide](../setup/workstation.md))

## Duration

**Estimated Time**: 2 weeks (10-15 hours)

## Hardware Requirements

- Workstation with ROS 2 Humble installed
- No physical robot required (simulation-only)

---

**Next Chapter**: [ROS 2 Basics](./ros2-basics.md)
EOF

# Create placeholder files for all modules
touch module-1-nervous-system/{ros2-basics.md,rclpy-tutorial.md,urdf-modeling.md}
touch module-2-digital-twin/intro.md
touch module-3-brain/intro.md
touch module-4-mind-vla/intro.md
touch capstone/intro.md
```

---

### Phase 3B: Custom React Components

```bash
cd docs-website/src/components

# RAG Chat Widget Component
cat > RAGChatWidget.tsx <<'EOF'
import React, { useState } from 'react';
import axios from 'axios';
import './RAGChatWidget.css';

const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{ title: string; url: string }>;
}

export default function RAGChatWidget(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!input.trim()) return;

    const userMessage: Message = { role: 'user', content: input };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setLoading(true);

    try {
      const response = await axios.post(`${API_BASE_URL}/api/rag/query`, {
        question: input,
        context: window.getSelection()?.toString() || '',
        module_id: 1, // TODO: Detect current module from URL
      });

      const assistantMessage: Message = {
        role: 'assistant',
        content: response.data.response,
        sources: response.data.sources,
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('RAG query error:', error);
      setMessages(prev => [
        ...prev,
        {
          role: 'assistant',
          content: 'Sorry, I encountered an error. Please try again.',
        },
      ]);
    } finally {
      setLoading(false);
    }
  };

  return (
    <>
      {/* Floating Button */}
      <button
        className="rag-chat-button"
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Open chatbot"
      >
        💬
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className="rag-chat-window">
          <div className="rag-chat-header">
            <h3>Ask the Book</h3>
            <button onClick={() => setIsOpen(false)}>✖</button>
          </div>

          <div className="rag-chat-messages">
            {messages.map((msg, idx) => (
              <div key={idx} className={`message message-${msg.role}`}>
                <p>{msg.content}</p>
                {msg.sources && (
                  <div className="message-sources">
                    <strong>Sources:</strong>
                    <ul>
                      {msg.sources.map((src, i) => (
                        <li key={i}>
                          <a href={src.url}>{src.title}</a>
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            ))}
            {loading && <div className="message message-loading">Thinking...</div>}
          </div>

          <form className="rag-chat-input" onSubmit={handleSubmit}>
            <input
              type="text"
              value={input}
              onChange={e => setInput(e.target.value)}
              placeholder="Ask a question..."
              maxLength={1500}
            />
            <button type="submit" disabled={loading || !input.trim()}>
              Send
            </button>
          </form>
        </div>
      )}
    </>
  );
}
EOF

# RAG Chat Widget CSS
cat > RAGChatWidget.css <<'EOF'
.rag-chat-button {
  position: fixed;
  bottom: 20px;
  right: 20px;
  width: 60px;
  height: 60px;
  border-radius: 50%;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  font-size: 24px;
  border: none;
  cursor: pointer;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  z-index: 1000;
  transition: transform 0.2s;
}

.rag-chat-button:hover {
  transform: scale(1.1);
}

.rag-chat-window {
  position: fixed;
  bottom: 100px;
  right: 20px;
  width: 400px;
  height: 500px;
  background: white;
  border-radius: 10px;
  box-shadow: 0 8px 24px rgba(0, 0, 0, 0.2);
  display: flex;
  flex-direction: column;
  z-index: 999;
}

.rag-chat-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 15px;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  border-radius: 10px 10px 0 0;
}

.rag-chat-header h3 {
  margin: 0;
  font-size: 18px;
}

.rag-chat-header button {
  background: none;
  border: none;
  color: white;
  font-size: 20px;
  cursor: pointer;
}

.rag-chat-messages {
  flex: 1;
  overflow-y: auto;
  padding: 15px;
  display: flex;
  flex-direction: column;
  gap: 10px;
}

.message {
  padding: 10px 15px;
  border-radius: 10px;
  max-width: 80%;
}

.message-user {
  align-self: flex-end;
  background: #667eea;
  color: white;
}

.message-assistant {
  align-self: flex-start;
  background: #f1f1f1;
  color: #333;
}

.message-sources {
  margin-top: 10px;
  font-size: 12px;
}

.message-sources ul {
  margin: 5px 0 0 0;
  padding-left: 20px;
}

.message-loading {
  align-self: center;
  font-style: italic;
  color: #999;
}

.rag-chat-input {
  display: flex;
  padding: 15px;
  border-top: 1px solid #ddd;
}

.rag-chat-input input {
  flex: 1;
  padding: 10px;
  border: 1px solid #ddd;
  border-radius: 5px;
  margin-right: 10px;
}

.rag-chat-input button {
  padding: 10px 20px;
  background: #667eea;
  color: white;
  border: none;
  border-radius: 5px;
  cursor: pointer;
}

.rag-chat-input button:disabled {
  background: #ccc;
  cursor: not-allowed;
}
EOF

# Adaptive Content Component (placeholder)
cat > AdaptiveContent.tsx <<'EOF'
import React from 'react';

interface AdaptiveContentProps {
  profileType: 'Software Engineer' | 'Hardware Engineer' | 'Student' | 'Hobbyist';
  children: React.ReactNode;
  showFor?: Array<'Software Engineer' | 'Hardware Engineer' | 'Student' | 'Hobbyist'>;
}

export default function AdaptiveContent({
  profileType,
  children,
  showFor,
}: AdaptiveContentProps): JSX.Element | null {
  if (showFor && !showFor.includes(profileType)) {
    return null; // Hide content if profile not in showFor list
  }

  return <>{children}</>;
}
EOF
```

---

## Phase 4: Integration Strategy

### Frontend-Backend Communication

**Configuration**: Use environment variables for API base URL

**File**: `docs-website/.env.local` (local development)
```env
REACT_APP_API_BASE_URL=http://localhost:8000
```

**File**: `docs-website/.env.production` (GitHub Pages deployment)
```env
REACT_APP_API_BASE_URL=https://rag-backend-production.railway.app
# Note: Currently localhost-only, update if deploying backend to cloud
```

**Usage in Components**:
```typescript
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';

axios.post(`${API_BASE_URL}/api/rag/query`, payload);
```

### CORS Configuration (Backend)

**File**: `rag-backend/main.py`
```python
origins = [
    "http://localhost:3000",  # Local Docusaurus dev
    "https://<username>.github.io",  # GitHub Pages
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

---

## Dependencies & Execution Order

### Phase Dependencies
- **Phase 0 (Research)**: No dependencies - can start immediately
- **Phase 1 (Design)**: Depends on Phase 0 completion (all NEEDS CLARIFICATION resolved)
- **Phase 2 (Skeleton)**: Depends on Phase 1 completion (data models, API contracts ready)
- **Phase 3 (Core Content)**: Depends on Phase 2 completion (directory structure created)
- **Phase 4 (Integration)**: Depends on Phase 2-3 completion (frontend + backend running)

### Within Each Phase
- **Phase 2 Tasks**: Can run in parallel (2A-2E independent)
- **Phase 3A**: Sequential (module structure must exist before content)
- **Phase 3B**: Can run in parallel with 3A (components independent of content)

---

## Next Steps

1. **Execute Phase 0**: Run research tasks (R1-R5), consolidate findings in `research.md`
2. **Execute Phase 1**: Generate `data-model.md`, `contracts/`, `quickstart.md`
3. **Execute Phase 2**: Run shell commands to create directory structure and initialize projects
4. **Execute Phase 3**: Add content (Module 1 chapters, React components)
5. **Test Integration**: Start frontend (`npm start`) + backend (`uvicorn main:app --reload`), verify RAG chatbot works
6. **Deploy**: Push to GitHub → GitHub Actions deploys frontend to GitHub Pages

---

## Risks and Mitigations

### Risk 1: Localhost Backend Limits Public Access
**Impact**: External visitors to GitHub Pages cannot use RAG chatbot features
**Mitigation**: Document limitation in README, provide instructions for running backend locally
**Future**: Deploy backend to Railway/Render if needed (add deployment guide)

### Risk 2: Qdrant Free Tier Storage Limit (1GB)
**Impact**: 50+ chapters with 500-token chunks may exceed free tier
**Mitigation**: Monitor chunk count, optimize chunking strategy (larger overlaps only for critical sections), upgrade to paid tier if needed ($25/month)

### Risk 3: OpenAI API Costs for RAG Queries
**Impact**: High query volume could be expensive
**Mitigation**: Rate limiting (100 queries/hour/user), caching common queries, switch to GPT-4o-mini for cost-sensitive responses

### Risk 4: GitHub Pages 1GB Size Limit
**Impact**: Large videos/images could exceed limit
**Mitigation**: Host videos on YouTube (embed), compress images with TinyPNG, use external CDN for large assets

---

## Appendix: Reference Links

- **Docusaurus Docs**: https://docusaurus.io/docs
- **FastAPI Docs**: https://fastapi.tiangolo.com/
- **Qdrant Docs**: https://qdrant.tech/documentation/
- **OpenAI Agents SDK**: https://platform.openai.com/docs/assistants/overview
- **Better-Auth**: https://www.better-auth.com/docs/introduction
- **ROS 2 Humble**: https://docs.ros.org/en/humble/
- **NVIDIA Isaac Sim**: https://docs.omniverse.nvidia.com/isaacsim/latest/

---

**Plan Complete**: Ready for Phase 0 execution (`research.md` generation)
