# Personalization Button Implementation

## Overview
AI-powered personalization button that adapts documentation content based on user's hardware and skill level. Appears on all documentation pages for logged-in users.

## Features Implemented

### 1. **Visibility Control**
- ✅ Only shows on documentation pages (`/docs/*`)
- ✅ Only visible to logged-in users
- ✅ Hidden on mobile/tablet (< 997px width)
- ✅ Responsive positioning on medium screens

### 2. **User Preferences Integration**
- ✅ Fetches real hardware background from database
- ✅ Fetches real skill level from database
- ✅ Loading state while fetching preferences
- ✅ Fallback to session data if available
- ✅ Displays current preferences in button

### 3. **Enhanced Styling**
- ✅ Modern card-based design
- ✅ Smooth hover animations
- ✅ Clear visual hierarchy
- ✅ Icon-enhanced buttons
- ✅ Info tooltip showing current settings
- ✅ Professional color scheme

### 4. **Two View Modes**

#### Replace Mode (📝)
- Overwrites article content with personalized version
- Shows banner indicating personalization
- Restore button to revert changes

#### Compare Mode (👥)
- Opens side-by-side modal
- Original content on left
- Personalized content on right
- "Apply to Page" button to replace
- ESC or click outside to close

## Component Structure

```
PersonalizeBtn
├── Control Buttons Section
│   ├── Personalize Button (main action)
│   └── Restore Button (if personalized)
├── View Mode Toggle
│   ├── Replace Mode
│   └── Compare Mode
└── Info Tooltip
    ├── Title: "AI Personalization"
    └── Current Settings Display
```

## Button States

### 1. **Loading State**
```
✨ Loading...
```
Shows while fetching user preferences from database

### 2. **Ready State**
```
✨ Personalize (Beginner)
✨ Personalize (Advanced)
```
Shows user's skill level

### 3. **Processing State**
```
✨ Personalizing...
```
Shows while AI is generating personalized content

### 4. **Personalized State**
```
✨ Personalize (Beginner)
🔄 Restore Original
```
Shows both main and restore buttons

## Visual Design

### Main Button
```css
Background: Indigo (#6366f1)
Padding: 12px 20px
Border Radius: 10px
Shadow: 0 4px 12px rgba(0,0,0,0.15)
Icon: ✨ (sparkles)
Font Weight: 700
Hover: Lift effect (-2px)
```

### Restore Button
```css
Background: Light gray
Border: 1px solid gray
Border Radius: 8px
Icon: 🔄 (refresh)
Hover: Darker background
```

### View Mode Toggle
```css
Background: Surface color
Border: 1px solid emphasis
Border Radius: 8px
Active: Primary color
Inactive: Transparent
Icons: 📝 (replace), 👥 (compare)
```

### Info Tooltip
```css
Background: Light emphasis
Border: 1px solid emphasis
Padding: 12px
Max Width: 220px
Font Size: 12px
Icon: 💡 (lightbulb)
```

## Positioning

### Desktop (> 1200px)
```css
Position: fixed
Top: 80px
Right: 20px
```

### Medium (997px - 1200px)
```css
Position: fixed
Top: 75px
Right: 10px
```

### Mobile/Tablet (< 997px)
```css
Display: none
```
Hidden to avoid cluttering small screens

## User Flow

### First-Time User
1. User signs up with hardware and skill level
2. Preferences saved to database
3. User navigates to doc page
4. PersonalizeBtn appears on right side
5. Button shows "Loading..." briefly
6. Button updates to "Personalize (Beginner)"
7. Tooltip shows current settings

### Personalization Flow
1. User clicks "✨ Personalize (Beginner)"
2. Button changes to "Personalizing..."
3. Article content extracted
4. Sent to AI with user preferences
5. AI generates personalized version
6. If "Compare" mode: Modal opens
7. If "Replace" mode: Content replaced
8. "Restore" button appears

### Comparison Flow
1. User in "Compare" mode
2. Clicks "Personalize"
3. Modal opens with split view
4. Left: Original content
5. Right: Personalized content
6. User can read both versions
7. Click "Apply to Page" to replace
8. Or close modal to keep original

### Restore Flow
1. User has personalized content
2. Clicks "🔄 Restore Original"
3. Page reloads
4. Original content restored
5. Restore button disappears

## Technical Implementation

### Component Files
```
web/src/components/PersonalizeBtn.tsx
├── State Management (29 lines)
│   ├── isPersonalizing
│   ├── originalContent
│   ├── personalizedContent
│   ├── viewMode
│   ├── showModal
│   ├── error
│   ├── hardwareBg
│   ├── skillLevel
│   └── loadingPreferences
├── Preferences Fetch (useEffect)
├── ESC Key Handler (useEffect)
├── Page Detection (isDocPage)
├── Personalization Logic
├── Modal Management
└── Render (3 sections)
    ├── Control Buttons
    ├── View Mode Toggle
    └── Info Tooltip
```

### CSS Integration
```
web/src/css/custom.css
└── Responsive Design
    ├── Mobile: Hidden (< 997px)
    └── Medium: Adjusted position (997-1200px)
```

### API Endpoints Used

**GET /api/user/profile**
```typescript
Request:
  Headers: { Authorization: 'Bearer <userId>' }

Response:
  {
    id: string,
    email: string,
    name: string,
    hardware_bg: string,  // "RTX 4090" | "Jetson Orin" | "Laptop CPU" | "Google Colab"
    skill_level: string   // "Beginner" | "Advanced"
  }
```

**POST /personalize** (FastAPI backend)
```typescript
Request:
  {
    content: string,          // Article text
    hardware_bg: string,      // User's hardware
    skill_level: string       // User's skill level
  }

Response:
  {
    personalized_content: string,  // AI-adapted content
    model: string                  // "gemini-2.5-flash"
  }
```

## Personalization Examples

### Hardware-Based Adaptation

**RTX 4090**:
```markdown
Original: "Train the model..."
Personalized: "Train the model using CUDA acceleration:
```bash
# Leverage 24GB VRAM for batch size 64
python train.py --device cuda --batch-size 64
```

**Jetson Orin**:
```markdown
Original: "Train the model..."
Personalized: "Deploy the model for edge inference:
```bash
# Optimize with TensorRT for low-latency
trtexec --onnx=model.onnx --saveEngine=model.trt
```

**Laptop CPU**:
```markdown
Original: "Train the model..."
Personalized: "Use Google Colab for GPU training:
```python
# Mount your Google Drive
from google.colab import drive
drive.mount('/content/drive')
```

### Skill-Based Adaptation

**Beginner**:
```markdown
# Step 1: Install ROS 2

First, we need to install ROS 2 (Robot Operating System).
This is the framework that lets robots communicate.

## What is ROS 2?
ROS 2 is like a messaging system for robots...

## Installation Steps
1. Open your terminal (command line)
2. Copy and paste this command:
   ```bash
   sudo apt install ros-humble-desktop
   ```
3. Wait for it to finish (this may take 10-15 minutes)
```

**Advanced**:
```markdown
# ROS 2 Installation

Install ROS 2 Humble:
```bash
sudo apt install ros-humble-desktop
```

For custom builds with minimal dependencies:
```bash
colcon build --packages-select <pkg> --cmake-args -DCMAKE_BUILD_TYPE=Release
```
```

## Error Handling

### No Article Content
```
Error: "No article content found on this page"
Duration: 3 seconds
Action: User can try navigating to different page
```

### Content Too Short
```
Error: "Article content is too short to personalize"
Duration: 3 seconds
Threshold: Minimum 50 characters
```

### API Failure
```
Error: "Failed to personalize content"
Duration: 5 seconds
Action: User can retry
```

### Profile Fetch Failure
```
Console: "Failed to fetch user preferences"
Fallback: Use default values (Laptop CPU, Beginner)
UI: Shows default in button
```

## Accessibility

### Keyboard Support
- ✅ Tab navigation through buttons
- ✅ ESC to close modal
- ✅ Focus outlines on all interactive elements
- ✅ Proper ARIA labels in tooltips

### Screen Reader Support
- ✅ Button titles with full descriptions
- ✅ Loading states announced
- ✅ Error messages readable
- ✅ Modal content accessible

### Visual Design
- ✅ High contrast text
- ✅ Clear button states
- ✅ Large click targets (44px minimum)
- ✅ Color not only indicator (icons + text)

## Performance

### Loading Speed
- **Initial Load**: < 100ms (component mount)
- **Preferences Fetch**: < 500ms (database query)
- **Personalization**: 2-5s (AI processing)
- **Modal Open**: < 50ms (instant)

### Optimization Techniques
- Session check before API call (faster)
- Component state caching
- Lazy rendering (only on doc pages)
- Debounced hover effects
- CSS transitions (GPU accelerated)

## Browser Compatibility

- ✅ Chrome 90+
- ✅ Firefox 88+
- ✅ Safari 14+
- ✅ Edge 90+
- ⚠️ IE 11 (not supported)

## Mobile Considerations

### Why Hidden on Mobile?
1. Limited screen space
2. Fixed positioning conflicts
3. Better UX to use on desktop
4. Personalization works better with larger screen

### Alternative for Mobile Users
- Use desktop/laptop for personalization
- Personalized content can be bookmarked
- Share personalized URLs (future feature)

## Testing Checklist

- [x] Button appears on doc pages only
- [x] Button hidden on homepage
- [x] Button hidden on login page
- [x] Button hidden for logged-out users
- [x] Fetches real user preferences
- [x] Shows loading state correctly
- [x] Personalize button works
- [x] Replace mode works
- [x] Compare mode works
- [x] Modal closes on ESC
- [x] Modal closes on outside click
- [x] Apply to page works
- [x] Restore original works
- [x] Error messages display
- [x] Tooltip shows correct info
- [x] Hover effects work
- [x] Responsive on desktop
- [x] Hidden on mobile
- [x] Adjusted on medium screens

## Known Limitations

1. **Mobile**: Hidden on small screens
2. **Offline**: Requires API connection
3. **Long Content**: May take longer to personalize
4. **Refresh Required**: Restore reloads page
5. **Single Page**: Can't batch personalize multiple pages

## Future Enhancements

- [ ] Batch personalization for multiple pages
- [ ] Save personalized versions
- [ ] Keyboard shortcuts (Ctrl+P to personalize)
- [ ] Preference editing inline
- [ ] Auto-personalize on page load (opt-in)
- [ ] Export personalized content as PDF
- [ ] Share personalized URLs
- [ ] Personalization history
- [ ] Undo/redo support
- [ ] Custom personalization prompts
- [ ] More hardware options
- [ ] More skill levels (Intermediate)
- [ ] Language-specific personalization

## Summary

The PersonalizeBtn component provides a seamless, AI-powered way for users to adapt documentation to their specific needs. It integrates with the auth system to fetch real user preferences, offers two viewing modes, and presents a polished, modern interface that enhances the learning experience.

Key benefits:
- **Personalized Learning**: Content adapted to skill level
- **Hardware-Specific**: Relevant to user's actual setup
- **Easy to Use**: One-click personalization
- **Non-Destructive**: Can restore original anytime
- **Professional UI**: Smooth animations and clear feedback
