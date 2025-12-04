---
id: 008
title: Add ReactMarkdown and Side-by-Side View to Personalization
stage: green
date: 2025-12-03
surface: agent
model: claude-sonnet-4-5-20250929
feature: textbook-platform
branch: master
user: Hamdan
command: user request
labels: ["personalization", "reactmarkdown", "side-by-side-view", "ui-enhancement", "modal"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - web/package.json
  - web/src/components/PersonalizeBtn.tsx
tests:
  - Manual testing required for modal interactions and markdown rendering
---

## Prompt

```
next step 1 and 4
```

Context: User requested enhancements #1 and #4 from the "Next Steps" list:
1. Add ReactMarkdown rendering for better formatting
4. Create side-by-side view (original vs personalized)

## Response snapshot

**Implementation completed successfully:**

### 1. ReactMarkdown Rendering

**Dependencies Added:**
- Installed `react-markdown` (v9.0.1) and `remark-gfm` (v4.0.0)
- Total bundle impact: ~50KB gzipped

**Features Implemented:**
- Proper markdown rendering in both modal panels
- GitHub-flavored markdown support (tables, strikethrough, task lists)
- Preserves code blocks, headers, lists, and links
- Better readability for technical documentation

**Implementation:**
```tsx
<ReactMarkdown remarkPlugins={[remarkGfm]}>
  {personalizedContent}
</ReactMarkdown>
```

### 2. Side-by-Side View Mode

**New Features:**
- **View Mode Toggle**: Two-button toggle (Replace | Side-by-Side)
- **Modal Dialog**: Full-screen modal with split view
- **Left Panel**: Original content (ReactMarkdown rendered)
- **Right Panel**: Personalized content (ReactMarkdown rendered)
- **Actions**: "Apply to Page" button to replace article, "×" button to close
- **Keyboard Support**: ESC key closes modal
- **Click Outside**: Clicking backdrop closes modal

**UI Structure:**
```
┌────────────────────────────────────────┐
│  [Personalize (Beginner)]  [Restore]   │
│  ┌──────────┬──────────────┐           │
│  │ Replace  │ Side-by-Side │           │
│  └──────────┴──────────────┘           │
└────────────────────────────────────────┘

Modal (Side-by-Side Mode):
┌──────────────────────────────────────────────┐
│  Original vs Personalized (Beginner • ...)   │
│  [Apply to Page]  [×]                        │
├──────────────────┬───────────────────────────┤
│  ORIGINAL        │  ✨ PERSONALIZED          │
│  [Markdown]      │  [Markdown]               │
│  (scrollable)    │  (scrollable)             │
└──────────────────┴───────────────────────────┘
│  💡 Tip: Apply   │  Press ESC to close       │
└──────────────────────────────────────────────┘
```

**Workflow Changes:**
1. User selects view mode (Replace or Side-by-Side)
2. Clicks "Personalize" button
3. **Replace Mode**: Immediately replaces article content
4. **Side-by-Side Mode**: Opens modal for comparison
5. In modal, user can:
   - Read both versions side-by-side
   - Click "Apply to Page" to replace article
   - Close without applying to keep original

**Technical Implementation:**
- State management: `viewMode`, `showModal`, `originalContent`, `personalizedContent`
- Event listener: ESC key handler with cleanup
- Modal styling: CSS Grid with 1fr | 1px | 1fr columns
- Backdrop overlay: Semi-transparent black with click-to-close
- Responsive design: 95% width, max 1400px, 90vh height

### Component Architecture

**State:**
```typescript
const [viewMode, setViewMode] = useState<ViewMode>("side-by-side");
const [showModal, setShowModal] = useState(false);
const [originalContent, setOriginalContent] = useState<string | null>(null);
const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
```

**Event Handling:**
```typescript
useEffect(() => {
  const handleEscape = (e: KeyboardEvent) => {
    if (e.key === "Escape" && showModal) {
      setShowModal(false);
    }
  };
  if (showModal) {
    document.addEventListener("keydown", handleEscape);
    return () => document.removeEventListener("keydown", handleEscape);
  }
}, [showModal]);
```

**File Changes:**
- `web/package.json`: Added react-markdown and remark-gfm dependencies
- `web/src/components/PersonalizeBtn.tsx`: Complete rewrite (1-447 lines)
  - Added ReactMarkdown imports
  - Implemented view mode toggle UI
  - Created side-by-side modal component
  - Added keyboard event listener
  - Enhanced styling and layout

## Outcome

- ✅ Impact: Significantly improved UX with proper markdown rendering and side-by-side comparison view
- 🧪 Tests: Manual testing required - modal interactions, ESC key, markdown rendering, view mode switching
- 📁 Files: 2 files modified (package.json, PersonalizeBtn.tsx)
- 🔁 Next prompts: Add syntax highlighting for code blocks, implement diff highlighting, add export options (PDF, markdown file)
- 🧠 Reflection: Side-by-side view is a game-changer for users to compare before committing. ReactMarkdown ensures technical documentation maintains proper formatting. Default to side-by-side mode since it's safer (non-destructive).

## Evaluation notes (flywheel)

- Failure modes observed: None during implementation. Potential issues: large documents may cause performance issues, mobile responsive needs testing, code blocks need syntax highlighting
- Graders run and results (PASS/FAIL): No automated tests - requires full stack testing with real content
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add react-syntax-highlighter for code block highlighting, implement responsive stacking for mobile (<768px), add copy-to-clipboard button for personalized content
