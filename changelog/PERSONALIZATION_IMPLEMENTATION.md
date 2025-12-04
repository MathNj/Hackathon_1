# Content Personalization Implementation - Complete

## Summary

Implemented content personalization logic that rewrites entire article pages based on user's skill level and hardware preferences using direct Gemini API calls.

## Changes Made

### 1. Backend API (`api/main.py`)

**Endpoint:** `POST /personalize`

**Changes:**
- Updated to use `client.chat.completions.create` directly (bypassing agent wrapper)
- Simplified system prompt to match specification
- Increased max_tokens to 2000 for full article rewrites

**System Prompt Format:**
```python
system_prompt = f"""Rewrite the following robotics technical documentation for a {request.skill_level} level student using {request.hardware_bg}. Adjust complexity and implementation details accordingly."""
```

**API Call:**
```python
response = await agent.client.chat.completions.create(
    model="gpt-4o",  # Gemini API identifier
    messages=[
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": request.content}
    ],
    max_tokens=2000,
    temperature=0.7
)
```

**Location:** `api/main.py:140-180`

### 2. Frontend Component (`web/src/components/PersonalizeBtn.tsx`)

**Workflow:**
1. User clicks "Personalize (Beginner/Advanced)" button
2. Component selects entire article content: `document.querySelector('article')?.innerText`
3. Shows "Rewriting..." state during API call
4. Replaces article HTML with personalized content
5. Shows "Restore Original" button to revert changes

**Key Features:**
- Selects entire `<article>` element content
- Stores original HTML for restoration
- Replaces content in-place with personalized version
- Shows visual indicator (border + header) for personalized content
- Two-button UI: "Personalize" and "Restore Original"

**UI Layout:**
```
┌─────────────────────────────────────────┐
│  [Personalize (Beginner)]  [Restore]    │  ← Fixed top-right
└─────────────────────────────────────────┘

Article content gets replaced with:

┌─────────────────────────────────────────┐
│ ✨ Personalized for Beginner using      │
│    Laptop                               │
├─────────────────────────────────────────┤
│ [Rewritten article content...]          │
└─────────────────────────────────────────┘
```

**Location:** `web/src/components/PersonalizeBtn.tsx:1-176`

## Request/Response Flow

### Request Format:
```json
POST http://localhost:8000/personalize
Content-Type: application/json

{
  "content": "Full article text from document.querySelector('article').innerText",
  "hardware_bg": "Laptop" | "RTX4090" | "Jetson" | "Cloud",
  "skill_level": "Beginner" | "Advanced"
}
```

### Response Format:
```json
{
  "personalized_content": "Rewritten article text adapted for skill level and hardware",
  "model": "gpt-4o"
}
```

## Personalization Examples

### Beginner + Laptop:
- Simple language and analogies
- Explains "why" concepts work
- CPU-friendly implementation suggestions
- Avoids complex math and jargon

### Advanced + RTX4090:
- Technical terminology and jargon
- Deep implementation details ("how")
- GPU-accelerated code examples
- Performance optimization focus

## User Session Integration

The component uses `authClient.useSession()` to get user preferences:

```typescript
const { data: session } = authClient.useSession();
const hardwareBg = session.user.hardware_bg || "Laptop";
const skillLevel = session.user.skill_level || "Beginner";
```

These preferences are set during login/signup at `/login` page.

## Content Replacement Strategy

**Before Personalization:**
- Article displays original Markdown-rendered content

**During Personalization:**
- Button shows "Rewriting..." state
- Article content remains visible

**After Personalization:**
- Original HTML stored in component state
- Article element's innerHTML replaced with:
  - Header banner showing personalization details
  - Personalized content (preserving line breaks)
- "Restore Original" button appears

**Restoration:**
- Clicking "Restore Original" reverts article to original HTML
- Clears stored original content
- Button disappears

## Error Handling

**Frontend Errors:**
- No article element found → Show error toast
- Article too short (< 50 chars) → Show error toast
- API request failed → Show error toast with 5s timeout

**Backend Errors:**
- Gemini API failure → HTTP 500 with error details
- Invalid request → HTTP 422 validation error

## Integration Points

1. **Authentication**: Uses Better-Auth session for user preferences
2. **API Backend**: FastAPI endpoint on `http://localhost:8000`
3. **Gemini API**: OpenAI-compatible endpoint via `AsyncOpenAI` client
4. **Docusaurus**: Targets `<article>` element in Docusaurus layout

## Files Modified

| File | Lines | Changes |
|------|-------|---------|
| `api/main.py` | 140-180 | Simplified personalization endpoint, direct API call |
| `web/src/components/PersonalizeBtn.tsx` | 1-176 | Complete rewrite: article selection + replacement |

## Testing the Implementation

### Prerequisites:
1. Auth server running on port 3001 with valid credentials
2. API backend running on port 8000 with GEMINI_API_KEY
3. User logged in with hardware_bg and skill_level set
4. Docusaurus site running on port 3000

### Test Steps:

1. **Login:**
   ```
   Visit http://localhost:3000/login
   Sign up/login with hardware and skill level
   ```

2. **Navigate to Article:**
   ```
   Go to any docs page (e.g., /docs/en/module-0-setup/intro)
   ```

3. **Personalize Content:**
   ```
   Click "Personalize (Beginner/Advanced)" button
   Wait for "Rewriting..." to complete
   Article content should be replaced with personalized version
   ```

4. **Restore Original:**
   ```
   Click "Restore Original" button
   Article should revert to original content
   ```

### Expected Behavior:

**For Beginner:**
- Simplified explanations
- Analogies and examples
- Less technical jargon
- Focus on concepts

**For Advanced:**
- Technical depth
- Code examples
- Performance details
- Industry terminology

## API Endpoint Documentation

### `POST /personalize`

**Description:** Rewrite robotics documentation for specific skill level and hardware

**Request Body:**
```typescript
{
  content: string;        // Article text to personalize
  hardware_bg: string;    // User's hardware setup
  skill_level: string;    // "Beginner" or "Advanced"
}
```

**Response:**
```typescript
{
  personalized_content: string;  // Rewritten content
  model: string;                 // Model used (e.g., "gpt-4o")
}
```

**Status Codes:**
- `200 OK` - Content personalized successfully
- `422 Unprocessable Entity` - Invalid request body
- `500 Internal Server Error` - API or processing error

## Performance Considerations

**Token Usage:**
- Full article personalization uses ~500-3000 tokens per request
- Max response: 2000 tokens
- Rate limits apply per Gemini API tier

**Response Time:**
- Typical: 3-8 seconds for full article
- Depends on article length and Gemini API load

**Caching:**
- Not implemented (each request is fresh)
- Consider adding Redis cache for repeated personalizations

## Future Enhancements

1. **Markdown Rendering**: Use ReactMarkdown for better formatting of personalized content
2. **Caching**: Cache personalized versions per (content_hash, skill_level, hardware_bg)
3. **Partial Personalization**: Allow users to select specific sections instead of entire article
4. **Side-by-Side View**: Show original and personalized content in split view
5. **History**: Save personalization history per user
6. **Feedback**: Allow users to rate personalized content quality
7. **Streaming**: Stream personalized content as it's generated for faster UX

## Documentation References

- Backend: `api/main.py:140-180`
- Frontend: `web/src/components/PersonalizeBtn.tsx`
- Agent: `api/agent.py` (provides client instance)
- Auth: `web/src/lib/auth-client.ts` (session management)

## Status

✅ **Backend API**: Implemented with direct Gemini API calls
✅ **Frontend Component**: Full article selection and replacement
✅ **Session Integration**: Uses user's skill_level and hardware_bg
✅ **UI/UX**: Two-button interface with state management
✅ **Error Handling**: Frontend and backend error paths covered

The content personalization feature is fully implemented and ready for testing.
