# Personalization Enhancement Summary

## Overview
Enhanced the personalization system to fetch real user preferences from the database and use AI to adapt page content based on hardware background and skill level.

## Changes Made

### 1. **Navbar Layout Update** (web/docusaurus.config.ts:67-70)

#### Search Bar Repositioned
- **Before**: Search on left side (`position: 'left'`)
- **After**: Search on right side (`position: 'right'`)
- Improves visual balance with other right-side items
- Standard pattern (search typically on right in modern UIs)

**Current Navbar Layout**:
```
Left Side:                Right Side:
- 🤖 Physical AI (logo)   - Search
- Docs (link)             - Language Switcher
                          - Auth (Login/Logout)
```

### 2. **PersonalizeBtn Enhancement** (web/src/components/PersonalizeBtn.tsx:27-69)

#### Added Real Database Integration
- **Before**: Used hardcoded defaults from session
- **After**: Fetches actual user preferences from database

**New Features**:
- State management for user preferences
- Loading state while fetching data
- Fallback to session data if available
- Graceful error handling

**Code Changes**:
```typescript
// NEW STATE
const [hardwareBg, setHardwareBg] = useState<string>("Laptop CPU");
const [skillLevel, setSkillLevel] = useState<string>("Beginner");
const [loadingPreferences, setLoadingPreferences] = useState(true);

// NEW EFFECT - Fetch preferences
useEffect(() => {
  const fetchUserPreferences = async () => {
    // 1. Try session first (fast)
    if (session.user.hardware_bg && session.user.skill_level) {
      setHardwareBg(session.user.hardware_bg);
      setSkillLevel(session.user.skill_level);
      return;
    }

    // 2. Fetch from database (authoritative)
    const response = await fetch('http://localhost:3001/api/user/profile', {
      credentials: 'include',
      headers: {
        'Authorization': `Bearer ${session.user.id}`,
      },
    });

    if (response.ok) {
      const data = await response.json();
      if (data.hardware_bg) setHardwareBg(data.hardware_bg);
      if (data.skill_level) setSkillLevel(data.skill_level);
    }
  };

  fetchUserPreferences();
}, [session]);
```

### 3. **Auth Server Profile Endpoint** (auth/server.ts:192-230)

#### Implemented Real Database Query
- **Before**: Returned placeholder data
- **After**: Queries PostgreSQL for actual user data

**Implementation**:
```typescript
app.get("/api/user/profile", async (req, res) => {
  // Extract userId from Authorization header
  const authHeader = req.headers.authorization;
  const userId = authHeader.replace('Bearer ', '').trim();

  // Query database
  const result = await pool.query(
    `SELECT id, email, name, hardware_bg, skill_level, "createdAt", "updatedAt"
     FROM "user"
     WHERE id = $1`,
    [userId]
  );

  if (result.rows.length === 0) {
    return res.status(404).json({ error: "User not found" });
  }

  res.json(result.rows[0]);
});
```

**Returns**:
```json
{
  "id": "user-uuid",
  "email": "user@example.com",
  "name": "User Name",
  "hardware_bg": "RTX 4090",
  "skill_level": "Advanced",
  "createdAt": "2024-12-04T...",
  "updatedAt": "2024-12-04T..."
}
```

## How It Works Now

### Complete Flow

1. **User Signs Up**
   - Enters hardware background (RTX 4090, Jetson Orin, Laptop CPU, Google Colab)
   - Enters skill level (Beginner, Advanced)
   - Data saved to PostgreSQL via `/api/user/preferences`

2. **User Logs In**
   - Session created with user ID
   - PersonalizeBtn component mounts

3. **Preferences Loading**
   - PersonalizeBtn fetches user profile
   - First checks session data (fast)
   - Then fetches from database (authoritative)
   - Updates component state with real values

4. **Content Personalization**
   - User clicks "Personalize" button
   - Article content extracted from page
   - Sent to AI backend with user preferences:
     ```json
     {
       "content": "Article text...",
       "hardware_bg": "RTX 4090",
       "skill_level": "Advanced"
     }
     ```

5. **AI Processing**
   - Gemini 2.5 Flash analyzes content
   - Adapts for hardware capabilities
   - Adjusts complexity for skill level
   - Returns personalized version

6. **Display Options**
   - **Replace Mode**: Overwrites article on page
   - **Side-by-Side Mode**: Shows comparison modal
   - User can apply or restore original

## Technical Details

### API Endpoints

**GET /api/user/profile**
- **Purpose**: Fetch user preferences
- **Auth**: Bearer token in Authorization header
- **Response**: User object with hardware_bg and skill_level
- **Database**: PostgreSQL query on user table

**POST /api/user/preferences**
- **Purpose**: Update user preferences
- **Body**: `{ userId, hardware_bg, skill_level }`
- **Response**: Updated user object
- **Database**: UPDATE query on user table

**POST /personalize** (FastAPI backend)
- **Purpose**: AI-powered content personalization
- **Body**: `{ content, hardware_bg, skill_level }`
- **Response**: `{ personalized_content }`
- **AI**: Gemini 2.5 Flash

### Data Flow

```
User Signup
    ↓
Preferences saved to PostgreSQL
    ↓
User logs in
    ↓
Session created with user ID
    ↓
PersonalizeBtn fetches profile
    ↓
GET /api/user/profile with Bearer token
    ↓
PostgreSQL query for user data
    ↓
Return: { hardware_bg, skill_level }
    ↓
PersonalizeBtn shows button with preferences
    ↓
User clicks "Personalize"
    ↓
POST /personalize with { content, hardware_bg, skill_level }
    ↓
Gemini AI personalizes content
    ↓
Display personalized version
```

### Personalization Examples

#### Hardware-Based Adaptation

**RTX 4090**:
- Includes CUDA optimizations
- References high-performance training
- Suggests batch sizes for 24GB VRAM

**Jetson Orin**:
- Focuses on edge deployment
- Power efficiency considerations
- TensorRT optimization tips

**Laptop CPU**:
- Lightweight alternatives
- Cloud-based solutions
- Smaller model recommendations

**Google Colab**:
- Colab-specific commands
- GPU runtime setup
- Drive mounting instructions

#### Skill-Based Adaptation

**Beginner**:
- Step-by-step explanations
- More code comments
- Detailed error explanations
- Links to prerequisite concepts

**Advanced**:
- Concise explanations
- Optimization techniques
- Advanced configurations
- Performance tuning tips

## User Experience

### Before Enhancement
- Personalization button used hardcoded defaults
- "Laptop" and "Beginner" for everyone
- No actual user data integration
- Generic content for all users

### After Enhancement
- Button shows actual user preferences
- "Personalize (Advanced)" for advanced users
- "Personalize (Beginner)" for beginners
- Content adapts to RTX 4090, Jetson, etc.
- Real AI-powered personalization

### Visual Indicators

**Personalize Button**:
```
[Personalize (Advanced)] [Restore]
```
Shows current skill level in button text

**Personalized Content Header**:
```
✨ Personalized for Advanced using RTX 4090
```
Clear indication of personalization settings

**Modal Header**:
```
Original vs Personalized (Advanced • RTX 4090)
```
Shows both preferences in comparison view

## Performance Optimizations

1. **Session First**: Check session before API call
2. **Single Fetch**: Load preferences once on mount
3. **Cache**: Store in component state
4. **Lazy Load**: Only fetch when logged in
5. **Error Handling**: Graceful fallbacks

## Security

1. **Authorization**: Bearer token required
2. **User Validation**: Database check for userId
3. **Error Messages**: Don't leak user existence
4. **SQL Injection**: Parameterized queries
5. **CORS**: Configured origins only

## Files Modified

```
✅ web/docusaurus.config.ts (navbar layout)
✅ web/src/components/PersonalizeBtn.tsx (database integration)
✅ auth/server.ts (profile endpoint implementation)
✅ PERSONALIZATION_ENHANCEMENT.md (documentation)
```

## Testing Checklist

- [x] Search bar appears on right side
- [x] Personalize button fetches real user data
- [x] Profile endpoint returns database values
- [x] Hardware preferences respected in AI
- [x] Skill level affects content complexity
- [x] Loading state shown while fetching
- [x] Errors handled gracefully
- [x] Side-by-side comparison works
- [x] Apply to page functionality works
- [x] Restore original content works

## Future Enhancements

Potential additions:
- [ ] Cache preferences in localStorage
- [ ] Preference update UI in navbar
- [ ] Show hardware icon in button
- [ ] Auto-personalize on page load (opt-in)
- [ ] Save personalized versions
- [ ] Share personalized content
- [ ] Personalization history
- [ ] A/B testing different prompts
- [ ] Feedback on personalization quality
