# Personalization with Beginner/Advanced Skill Levels - Implementation Summary

## Overview

Successfully implemented a skill level dimension to the personalization system, allowing content to be adapted for Beginner and Advanced learners based on their technical proficiency.

## Goal

Improve content adaptation by adding a 'Skill Level' dimension where:
- **Beginner**: Simplifies concepts with analogies, avoids complex math, focuses on "Why"
- **Advanced**: Goes deep into technical implementation, uses jargon, focuses on "How"

## Implementation Details

### 1. ✅ Updated Auth Schema (`auth/auth.config.ts`)

**Added SkillLevel Enum:**
```typescript
export enum SkillLevel {
  Beginner = "Beginner",
  Advanced = "Advanced",
}
```

**Extended User Interface:**
```typescript
export interface ExtendedUser {
  id: string;
  email: string;
  name?: string;
  emailVerified: boolean;
  createdAt: Date;
  updatedAt: Date;
  // Custom fields
  hardware_bg?: HardwareBackground;
  skill_level?: SkillLevel;  // NEW FIELD
}
```

**Added to User Schema:**
```typescript
user: {
  additionalFields: {
    hardware_bg: { /* ... */ },
    skill_level: {
      type: "string",
      required: false,
      input: true,
      defaultValue: SkillLevel.Beginner,
      validate: (value: string) => {
        return Object.values(SkillLevel).includes(value as SkillLevel);
      },
    },
  },
}
```

**Migration:**
- Ran `npx @better-auth/cli generate` in `/auth` directory
- Database schema updated automatically

### 2. ✅ Updated Signup UI (`web/src/pages/login.tsx`)

**Added Skill Level State:**
```typescript
const SKILL_LEVEL_OPTIONS: SkillLevel[] = ["Beginner", "Advanced"];
const [skillLevel, setSkillLevel] = useState<SkillLevel>("Beginner");
```

**Added Skill Level Dropdown to Form:**
```tsx
<div style={{ marginBottom: "20px" }}>
  <label htmlFor="skillLevel">Skill Level</label>
  <select
    id="skillLevel"
    value={skillLevel}
    onChange={(e) => setSkillLevel(e.target.value as SkillLevel)}
  >
    {SKILL_LEVEL_OPTIONS.map((option) => (
      <option key={option} value={option}>
        {option}
      </option>
    ))}
  </select>
</div>
```

**Position:** Appears after Hardware Background dropdown during signup

### 3. ✅ Updated Auth Client Types (`web/src/lib/auth-client.ts`)

**Added SkillLevel Type:**
```typescript
export type SkillLevel = "Beginner" | "Advanced";
```

**Updated User Interface:**
```typescript
export interface User {
  id: string;
  name: string;
  email: string;
  hardware_bg?: HardwareBackground;
  skill_level?: SkillLevel;  // NEW FIELD
}
```

### 4. ✅ Created Personalization API (`api/main.py`)

**Added Request/Response Models:**
```python
class PersonalizeRequest(BaseModel):
    """Personalization request with content and user preferences"""
    content: str
    hardware_bg: str
    skill_level: str

class PersonalizeResponse(BaseModel):
    """Personalized content response"""
    personalized_content: str
    model: Optional[str] = None
```

**Created `/personalize` Endpoint:**
```python
@app.post("/personalize", response_model=PersonalizeResponse)
async def personalize(request: PersonalizeRequest):
    # Build system prompt based on skill level
    if request.skill_level.lower() == "beginner":
        system_prompt = """You are a friendly AI tutor.
        Explain like the student is 12 years old. Use analogies and simple language.
        Avoid complex mathematics and jargon. Focus on the "Why" - why things work the way they do.
        Make concepts relatable and easy to understand."""
    else:  # Advanced
        system_prompt = """You are an expert technical instructor.
        Assume the learner has expert knowledge in robotics and AI.
        Use industry jargon and technical terminology freely.
        Focus on performance, optimization, and the "How" - implementation details.
        Show code examples, algorithms, and deep technical explanations."""

    # Add hardware-specific context
    hardware_context = f"\nThe learner is using: {request.hardware_bg}"
    # ... hardware-specific customization

    # Generate personalized response using agent
    result = await agent.generate_response(
        query=full_prompt,
        use_rag=False  # Don't use RAG for personalization
    )
```

**Prompt Engineering Strategy:**

#### Beginner Tier:
- **Tone**: Friendly AI tutor
- **Language**: Explain like they're 12 years old
- **Approach**: Use analogies and simple language
- **Math**: Avoid complex mathematics
- **Focus**: "Why" - why things work the way they do
- **Goal**: Make concepts relatable and easy to understand

#### Advanced Tier:
- **Tone**: Expert technical instructor
- **Language**: Industry jargon and technical terminology
- **Approach**: Assume expert knowledge in robotics and AI
- **Details**: Show code examples, algorithms, deep explanations
- **Focus**: "How" - implementation details
- **Goal**: Performance, optimization, and technical depth

**Hardware Integration:**
The API also considers hardware background for additional context:
- **RTX 4090**: GPU-accelerated implementations, high-performance computing
- **Jetson Orin**: Edge computing, real-time processing, embedded systems
- **Laptop CPU**: CPU-friendly algorithms, learning without GPU
- **Google Colab**: Cloud-based training, collaborative notebooks

### 5. ✅ Updated PersonalizeBtn Component (`web/src/components/PersonalizeBtn.tsx`)

**Complete Rewrite with New Features:**

**Retrieves Session Data:**
```typescript
const { data: session } = authClient.useSession();
const hardwareBg = session.user.hardware_bg || "Laptop CPU";
const skillLevel = session.user.skill_level || "Beginner";
```

**Interactive Button:**
```tsx
<button
  onClick={handlePersonalize}
  disabled={isPersonalizing}
  title="Select text and click to personalize"
>
  {isPersonalizing
    ? "Personalizing..."
    : `Personalize (${skillLevel})`}
</button>
```

**Personalization Flow:**
1. User selects text on the page (min 10 characters)
2. Clicks "Personalize (Beginner)" or "Personalize (Advanced)" button
3. Selected text + user preferences sent to API
4. Personalized content displayed in popup

**API Integration:**
```typescript
const response = await fetch("http://localhost:8000/personalize", {
  method: "POST",
  headers: { "Content-Type": "application/json" },
  body: JSON.stringify({
    content: selectedText,
    hardware_bg: hardwareBg,
    skill_level: skillLevel,
  }),
});
```

**UI States:**
- **Default**: Shows "Personalize (Beginner)" or "Personalize (Advanced)"
- **Loading**: Shows "Personalizing..." with disabled state
- **Error**: Red notification with error message (auto-dismisses)
- **Success**: Popup with personalized content and close button

**Popup Design:**
```
┌─────────────────────────────────────┐
│ Personalized for Beginner        × │
├─────────────────────────────────────┤
│                                     │
│ [Personalized content appears here] │
│                                     │
│ (Scrollable if long)                │
│                                     │
└─────────────────────────────────────┘
```

## User Workflows

### Scenario 1: New User Signup with Skill Selection

1. User navigates to `/login`
2. Toggles to "Sign Up" mode
3. Enters name, email, password
4. Selects **Hardware Background**: "RTX 4090"
5. Selects **Skill Level**: "Advanced"
6. Clicks "Sign Up"
7. Profile saved with both preferences

### Scenario 2: Beginner User Personalization

1. Beginner user logs in
2. Navigates to documentation page
3. Reads complex paragraph about neural networks
4. Selects the paragraph text
5. Clicks "Personalize (Beginner)" button
6. Popup appears with:
   - Simple language explanation
   - Analogies (e.g., "Think of neurons like light switches...")
   - Focus on concepts, not math
   - "Why" this matters

### Scenario 3: Advanced User Personalization

1. Advanced user logs in
2. Navigates to same documentation page
3. Selects the same paragraph
4. Clicks "Personalize (Advanced)" button
5. Popup appears with:
   - Technical jargon and terminology
   - Code examples and algorithms
   - Performance considerations
   - Implementation details
   - "How" to build it

### Scenario 4: Hardware-Specific Context

**Example: RTX 4090 + Advanced**
- User sees: GPU-accelerated code examples
- Focus on: CUDA kernels, tensor operations, batch processing
- Optimization: GPU memory management, parallel processing

**Example: Laptop CPU + Beginner**
- User sees: Simple Python examples
- Focus on: Concepts without heavy computation
- Optimization: Learning principles, not performance

## Files Created/Modified

### Modified Files
```
auth/auth.config.ts                      # Added skill_level field and enum
web/src/lib/auth-client.ts              # Added SkillLevel type
web/src/pages/login.tsx                 # Added skill level dropdown
api/main.py                             # Added /personalize endpoint
web/src/components/PersonalizeBtn.tsx   # Complete rewrite with API integration
```

### No New Files Created
All changes were modifications to existing files.

## API Endpoints

### POST `/personalize`

**Request:**
```json
{
  "content": "Neural networks consist of layers of interconnected nodes...",
  "hardware_bg": "RTX 4090",
  "skill_level": "Advanced"
}
```

**Response:**
```json
{
  "personalized_content": "Neural networks utilize matrix multiplication across GPU tensor cores for parallel computation. In CUDA, each layer's forward pass can be optimized using cuBLAS for GEMM operations...",
  "model": "gemini-2.0-flash-exp"
}
```

## Example Outputs

### Beginner Example

**Input:** "Backpropagation is used to train neural networks"

**Output:**
```
Think of backpropagation like learning from your mistakes in school.

Imagine you take a test and get some answers wrong. Your teacher tells you
which ones are wrong, and you go back to understand WHY you got them wrong.
That's exactly what backpropagation does!

The neural network makes a guess, sees how far off it was, and then goes
backwards through all its layers to figure out what went wrong. It's like
rewinding a video to see where you made a mistake, then practicing that
part again.

Why is this important? Because this is how AI learns and gets better over
time, just like how you improve by learning from your mistakes!
```

### Advanced Example

**Input:** "Backpropagation is used to train neural networks"

**Output:**
```
Backpropagation implements the chain rule of calculus to compute gradients
efficiently through computational graphs. The algorithm has O(n) complexity
where n is the number of edges in the network.

Implementation on RTX 4090:
```python
# GPU-accelerated backprop using PyTorch
def backward_pass(loss, model):
    loss.backward()  # Computes gradients via autograd

    # Gradient accumulation for large batches
    for param in model.parameters():
        if param.grad is not None:
            param.grad /= world_size  # Distributed training
```

Key optimizations:
- Use mixed precision (FP16/FP32) for 2-3x speedup
- Enable cuDNN autotuner: torch.backends.cudnn.benchmark = True
- Gradient checkpointing for memory-intensive models
- Asynchronous GPU operations via CUDA streams

Performance considerations: On RTX 4090 with 24GB VRAM, you can train
models up to ~7B parameters with batch size 8 using gradient checkpointing.
```

## Testing Checklist

- [X] Auth schema includes skill_level field
- [X] Database migration generated successfully
- [X] Login page displays skill level dropdown
- [X] Skill level dropdown has "Beginner" and "Advanced" options
- [X] Default skill level is "Beginner"
- [X] Auth client types include SkillLevel
- [X] PersonalizeBtn displays correct skill level from session
- [X] PersonalizeBtn button text shows `Personalize (Beginner)` or `Personalize (Advanced)`
- [X] Text selection required (min 10 chars) before personalization
- [X] API endpoint `/personalize` accepts skill_level
- [X] Beginner prompt uses simple language and analogies
- [X] Advanced prompt uses technical jargon and code examples
- [X] Hardware context integrated into personalization
- [X] Error handling for failed API calls
- [X] Loading state shown during personalization
- [X] Personalized content displayed in popup
- [X] Popup can be closed by user

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                     User Interface                          │
│                   (Docusaurus Frontend)                     │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │             Login/Signup Page                        │  │
│  │                                                      │  │
│  │  [Email] [Password] [Name]                          │  │
│  │  [Hardware: RTX 4090 ▼]                             │  │
│  │  [Skill Level: Advanced ▼]  ← NEW DROPDOWN          │  │
│  │                                                      │  │
│  │  [Sign Up]                                           │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │         Documentation Page                           │  │
│  │                                                      │  │
│  │  [Neural networks consist of layers...]             │  │
│  │  └─ User selects text                               │  │
│  │                                                      │  │
│  │  [Personalize (Advanced)] ← Button shows skill      │  │
│  └──────────────────────────────────────────────────────┘  │
│                           │                                 │
│                           ▼ POST /personalize               │
└───────────────────────────┼─────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                  FastAPI Backend                            │
│                (localhost:8000)                             │
│                                                             │
│  POST /personalize                                          │
│  ┌────────────────────────────────────────────────────┐    │
│  │  1. Receive: content, hardware_bg, skill_level     │    │
│  │                                                    │    │
│  │  2. Build System Prompt:                           │    │
│  │     if skill_level == "Beginner":                  │    │
│  │       → Friendly tutor, analogies, simple          │    │
│  │     else:                                          │    │
│  │       → Expert instructor, jargon, code            │    │
│  │                                                    │    │
│  │  3. Add Hardware Context:                          │    │
│  │     if "RTX 4090":                                 │    │
│  │       → GPU acceleration, CUDA, performance        │    │
│  │     elif "Laptop CPU":                             │    │
│  │       → CPU-friendly, learning focus               │    │
│  │                                                    │    │
│  │  4. Generate with Gemini API                       │    │
│  │                                                    │    │
│  │  5. Return personalized_content                    │    │
│  └────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                 Better Auth + Neon Postgres                 │
│                                                             │
│  User Table:                                                │
│  ┌──────────────────────────────────────────────────┐      │
│  │ id          | email          | name              │      │
│  │ hardware_bg | skill_level ← NEW FIELD            │      │
│  ├──────────────────────────────────────────────────┤      │
│  │ user-1234   | john@email.com | John Doe          │      │
│  │ RTX 4090    | Advanced                           │      │
│  └──────────────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────────┘
```

## Benefits of This Implementation

1. **Personalized Learning Paths**: Content automatically adapts to user's skill level
2. **Hardware Optimization**: Considers user's hardware for relevant examples
3. **Improved Engagement**: Beginners aren't overwhelmed, Advanced users aren't bored
4. **Seamless UX**: One-click personalization with text selection
5. **Scalable**: Easy to add more skill levels (Intermediate, Expert, etc.)
6. **Type Safety**: Full TypeScript types across frontend and backend
7. **Flexible Prompting**: Easy to adjust system prompts for different tiers

## Future Enhancements

1. **Intermediate Tier**: Add a middle ground between Beginner and Advanced
2. **Progress Tracking**: Automatically upgrade users from Beginner to Advanced
3. **Custom Preferences**: Let users override skill level per-topic
4. **Personalization History**: Save and review past personalizations
5. **A/B Testing**: Compare effectiveness of different prompts
6. **Learning Path Recommendations**: Suggest next topics based on skill level

## Running the System

### 1. Start Auth Service
```bash
cd auth
npm run dev
# Running on http://localhost:3001
```

### 2. Start API Backend
```bash
cd api
python main.py
# Running on http://localhost:8000
```

### 3. Start Frontend
```bash
cd web
npm start
# Running on http://localhost:3000
```

### 4. Test Workflow
1. Visit `http://localhost:3000/login`
2. Sign up with email, select "Advanced" skill level
3. Navigate to documentation
4. Select some text
5. Click "Personalize (Advanced)"
6. View personalized content

## Status: ✅ COMPLETE

All requirements for Beginner/Advanced skill level personalization have been successfully implemented and tested.
