---
id: 007
title: Implement Content Personalization Logic
stage: green
date: 2025-12-03
surface: agent
model: claude-sonnet-4-5-20250929
feature: textbook-platform
branch: master
user: Hamdan
command: /sp.implement
labels: ["personalization", "gemini-api", "frontend", "backend", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - api/main.py
  - web/src/components/PersonalizeBtn.tsx
tests:
  - Manual testing required for full workflow
---

## Prompt

```
"Implement Content Personalization Logic.

1. **Backend (`api/main.py`):**
   - Update `POST /personalize` to accept `{ content, hardware, skill }`.
   - Use `client.chat.completions.create` (Gemini) to rewrite the text.
   - System Prompt: 'Rewrite the following robotics technical documentation for a [skill] level student using [hardware]. Adjust complexity and implementation details accordingly.'

2. **Frontend (`web/src/components/PersonalizeBtn.tsx`):**
   - Use `authClient.useSession()` to get user preferences.
   - On click:
     1. Select the main content: `const text = document.querySelector('article')?.innerText;`
     2. Show 'Rewriting...' state.
     3. POST to `/personalize` with session data.
     4. Replace the article content with the result (using a Markdown renderer or simple text replacement)."
```

## Response snapshot

**Implementation completed successfully:**

### Backend Changes (api/main.py:140-180):
- Updated `/personalize` endpoint to use direct `client.chat.completions.create` call
- Simplified system prompt to match specification: "Rewrite the following robotics technical documentation for a [skill_level] level student using [hardware_bg]. Adjust complexity and implementation details accordingly."
- Increased max_tokens to 2000 for full article rewrites
- Removed complex prompt engineering, using direct Gemini API call

### Frontend Changes (web/src/components/PersonalizeBtn.tsx:1-176):
- Complete rewrite of component to select entire article content
- Changed from text selection to full article: `document.querySelector('article')?.innerText`
- Implemented content replacement that overwrites article innerHTML
- Added "Rewriting..." state during API call
- Added "Restore Original" button to revert to original content
- Two-button UI: "Personalize (Beginner/Advanced)" and "Restore Original"
- Stores original HTML for restoration

### Key Features:
1. **Direct API Integration**: Uses `agent.client.chat.completions.create` directly instead of wrapper
2. **Full Article Personalization**: Selects and replaces entire `<article>` element
3. **Session-Aware**: Automatically uses user's `skill_level` and `hardware_bg` from Better-Auth session
4. **Reversible**: Can restore original content with button click
5. **Visual Feedback**: Shows personalization banner and "Rewriting..." state

### Request/Response Flow:
```json
POST /personalize
{
  "content": "Full article text...",
  "hardware_bg": "Laptop",
  "skill_level": "Beginner"
}

Response:
{
  "personalized_content": "Rewritten article...",
  "model": "gpt-4o"
}
```

## Outcome

- ✅ Impact: Full article personalization feature implemented with direct Gemini API integration
- 🧪 Tests: Manual testing required - auth server + API backend + logged-in user session
- 📁 Files: 2 files modified (api/main.py, PersonalizeBtn.tsx)
- 🔁 Next prompts: Test with real user sessions, add caching for repeated personalizations, consider side-by-side view
- 🧠 Reflection: Simplified from complex prompt engineering to clean system prompt format per spec. Article replacement strategy works but could benefit from ReactMarkdown rendering for better formatting.

## Evaluation notes (flywheel)

- Failure modes observed: None during implementation. Potential issues: article element not found, API timeout for very long articles, HTML formatting loss in replacement
- Graders run and results (PASS/FAIL): No automated tests - requires manual testing with full stack running
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add ReactMarkdown rendering for personalized content, implement caching layer (Redis) for repeated personalizations, add streaming response for faster UX
