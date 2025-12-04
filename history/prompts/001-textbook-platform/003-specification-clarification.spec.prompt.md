---
id: 003
title: Specification Clarification Session
stage: spec
date: 2025-12-03
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-textbook-platform
branch: master
user: Hamdan
command: /sp.clarify
labels: ["clarification", "architecture", "deployment", "rag", "localization", "auth"]
links:
  spec: specs/001-textbook-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-textbook-platform/spec.md
tests:
 - N/A (specification refinement)
---

## Prompt

GOAL: Interrogate the Specification to identify edge cases, architectural gaps, and missing constraints.

CONTEXT:
We have a draft `sp.spec.md` for a "Physical AI" textbook. However, this is a complex hybrid application (Static Frontend + Intelligent Backend). I need you to find the hidden risks.

AREAS TO CRITIQUE (Ask me specific questions about these)
1. **The "Static vs. Dynamic" Conflict:**
   - The book is hosted on GitHub Pages (Static). The Chatbot and Auth need a backend (Python/FastAPI).
   - *Question to resolve:* Where are we hosting the Python Backend? (Render? Railway? Localhost for now?). How do we handle CORS?

2. **The "Better-Auth" Integration:**
   - GitHub Pages cannot run server-side Node.js auth middleware.
   - *Question to resolve:* Are we using a client-side auth flow (OAuth) talking to a separate auth server, or a comprehensive BaaS? How do we persist the "User Profile" (Hardware vs Software Engineer) securely?

3. **The RAG Chatbot Logistics:**
   - *Question to resolve:* When does the "Ingestion" happen? (Do we scrape our own book every time we push code? Or is it a manual script?).
   - *Question to resolve:* How do we limit costs? (Do we need rate limiting so one user doesn't burn all our OpenAI credits?).

4. **Localization (Urdu) Strategy:**
   - *Question to resolve:* Is the Urdu translation generated "On the Fly" (slow, expensive API calls) or "Pre-Built" (static files generated at build time)? The prompt mentions "pressing a button," which implies dynamic, but pre-built is safer for a Hackathon.

5. **Hardware Dependencies:**
   - *Question to resolve:* If a user visits the site on a mobile phone (without an RTX 4070), should the heavy "Isaac Sim" tutorials show a warning banner?

OUTPUT:
Analyze the spec and present a list of **Clarifying Questions** (Q1, Q2, Q3...) that I must answer to finalize the requirements.

## Response snapshot

Conducted comprehensive 5-question clarification session identifying and resolving critical architectural ambiguities in the Physical AI Textbook specification.

**Questions Asked & Answered (5/5):**

1. **Backend Hosting & CORS Strategy**
   - Answer: Localhost backend + GitHub Pages frontend, CORS configured for both origins
   - Impact: Defines deployment architecture, RAG features only work with local backend running

2. **Better-Auth Architecture**
   - Answer: Client-side OAuth + Backend Auth State (Better-Auth SDK in React → FastAPI verification → Neon Postgres + HTTP-only cookies)
   - Impact: Clarifies auth flow for static site + separate backend architecture

3. **RAG Content Ingestion Timing**
   - Answer: One-Time Initial Ingestion (manual script execution, re-run when content updates)
   - Impact: Simplifies implementation, eliminates automated re-indexing complexity

4. **Urdu Localization Strategy**
   - Answer: Pre-Built Static Files (translate once via script, build static HTML, instant language toggle)
   - Impact: Eliminates runtime costs, ensures reliability, instant UX

5. **Mobile Device Warning for GPU Content**
   - Answer: No warnings (static content displays normally on all devices, hardware requirements in Module 0 only)
   - Impact: Simplifies implementation, avoids device detection complexity

**Specification Sections Updated (10):**
- Added "Clarifications" section with Session 2025-12-03 subsection (5 Q&A bullets)
- FR-008: RAG Backend Architecture (added hosting: localhost, CORS configuration)
- FR-009: RAG Content Indexing Strategy (specified one-time manual ingestion, no auto GitHub Actions)
- FR-011: Better-Auth Integration (added detailed client-side OAuth flow with 5-step process)
- FR-016: Urdu Localization Button (specified pre-built static files, instant toggle, no API calls)
- FR-017: Urdu Translation Workflow (changed from automated GitHub Actions to manual script)
- User Story 6 Scenario 5: Updated to reflect manual translation script workflow
- SC-009: Changed from "turnaround time" to "script functionality" success criteria
- Edge Case 8: Added mobile device handling (no warnings, responsive design)
- Dependencies & Prerequisites: Added "Deployment Model" subsection clarifying frontend/backend split

**Key Architectural Decisions:**
1. Hybrid deployment: Static GitHub Pages + Localhost FastAPI backend
2. Client-side OAuth with backend token verification pattern
3. One-time RAG ingestion (manual script) instead of automated pipeline
4. Build-time Urdu translation (pre-built static files) instead of runtime API calls
5. No mobile device detection (universal content access)

**Risks Mitigated:**
- Static/Dynamic conflict resolved (CORS, localhost backend limitations documented)
- Auth flow architecture defined (no server-side middleware needed on GitHub Pages)
- RAG cost control (one-time ingestion, rate limiting, no repeated embeddings)
- Translation performance optimized (pre-built, instant toggle, no runtime latency)
- Mobile UX simplified (no conditional rendering, hardware docs in setup guide)

**Coverage Summary:**
- Integration & External Dependencies: ✅ Resolved
- Infrastructure & Deployment: ✅ Resolved
- Data & Content Management: ✅ Resolved
- Non-Functional (Performance, Security): ✅ Clear
- User Experience & Edge Cases: ✅ Resolved
- All other categories: ✅ Clear (already sufficient)

**Trade-offs Accepted:**
- Simplicity vs. Public Accessibility: RAG features require local backend
- Simplicity vs. Automation: Manual ingestion/translation scripts
- Simplicity vs. User Guidance: No mobile warnings, relying on documentation

## Outcome

- ✅ Impact: Resolved 5 critical architectural ambiguities that would have blocked implementation planning. Specification now unambiguous and ready for `/sp.plan`. All high-impact categories addressed. No outstanding ambiguities remain.
- 🧪 Tests: N/A (specification refinement)
- 📁 Files: Updated `specs/001-textbook-platform/spec.md` with 5 clarifications and 10 section updates
- 🔁 Next prompts:
  - `/sp.plan` to create implementation plan (architecture, technical design, phase breakdown)
  - `/sp.adr "Hybrid Deployment Architecture"` to document static frontend + localhost backend decision
  - `/sp.adr "Pre-Built Localization Strategy"` to document build-time translation approach
- 🧠 Reflection: Successfully identified and resolved the exact "hidden risks" highlighted by the user: Static/Dynamic conflict (localhost + CORS), Better-Auth architecture (client-side OAuth), RAG logistics (one-time ingestion), Urdu strategy (pre-built), and mobile handling (no warnings). All decisions favor simplicity and hackathon speed while maintaining functionality. The hybrid deployment model (GitHub Pages + localhost) is clearly documented with limitations (RAG only works locally). Pre-built translations eliminate runtime costs and latency. Specification is now implementation-ready with no blocking ambiguities.

## Evaluation notes (flywheel)

- Failure modes observed: None. All 5 questions answered successfully, clarifications integrated incrementally after each answer.
- Graders run and results (PASS/FAIL):
  - ✅ PASS: 5 questions asked (within 5-question limit)
  - ✅ PASS: Each question provided recommended option with reasoning
  - ✅ PASS: Questions covered all 5 user-specified critique areas
  - ✅ PASS: Clarifications section created with Session 2025-12-03 heading
  - ✅ PASS: Each Q&A recorded as bullet in Clarifications section
  - ✅ PASS: Relevant FR sections updated (FR-008, FR-009, FR-011, FR-016, FR-017)
  - ✅ PASS: User stories updated (User Story 6 Scenario 5)
  - ✅ PASS: Success criteria updated (SC-009)
  - ✅ PASS: Edge cases updated (Edge Case 8)
  - ✅ PASS: Dependencies section updated (Deployment Model added)
  - ✅ PASS: File saved after each clarification (atomic updates)
  - ✅ PASS: Coverage summary provided with category status table
  - ✅ PASS: Risks mitigated section clear
  - ✅ PASS: Next steps recommended (/sp.plan)
- Prompt variant (if applicable): N/A (standard clarification workflow)
- Next experiment (smallest change to try): Consider adding a "Development Setup Checklist" in the spec that lists required tools, environment variables, and verification steps (localhost backend setup, Qdrant/Neon credentials, OpenAI API key, Docker optional). This would help developers get started faster and reduce setup friction. Could be added in plan.md or as an appendix to spec.md.
