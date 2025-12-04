# Text Selection "Add to Chat" Feature

## Overview
Users can now highlight any text on documentation pages and instantly add it as context to the AI chat assistant.

## How It Works

### User Flow
1. **Select Text**: User highlights text on any documentation page
2. **Button Appears**: A floating "Add to Chat" button appears near the selection
3. **Click to Add**: Clicking the button:
   - Adds selected text as context to the chat widget
   - Opens the chat widget automatically
   - Clears the text selection
4. **Ask Question**: User can now ask questions about the highlighted text

### Technical Implementation

#### Components Created/Modified

**1. TextSelectionHandler.tsx** (NEW)
- Location: `web/src/components/TextSelectionHandler.tsx`
- Detects text selection using browser Selection API
- Shows floating button with smooth animation
- Positioned dynamically based on selection location
- Auto-dismisses when selection is cleared

**2. ChatContext.tsx** (EXISTING)
- Already had `addContext()` method
- Manages global context state
- Limits context to 200 characters per selection
- Prevents duplicate contexts

**3. ChatWidget.tsx** (ENHANCED)
- Improved context display section
- Shows context count and visual indicators
- Better styling with background highlighting
- Each context item is removable independently

**4. Root.tsx** (MODIFIED)
- Added TextSelectionHandler to global component tree
- Available on all pages automatically

### Features

#### Smart Context Management
- **Automatic Truncation**: Long selections are limited to 200 characters
- **Duplicate Prevention**: Same text won't be added twice
- **Multiple Contexts**: Can add multiple selections before asking
- **Visual Feedback**: Clear display of all added contexts in chat

#### User Experience
- **Smooth Animations**: Button fades in elegantly
- **Hover Effects**: Button scales slightly on hover
- **Auto-Open Chat**: Chat widget opens when context is added
- **Easy Removal**: Each context has its own × button

#### Technical Details
- **Event Listeners**: mouseup, touchend, selectionchange
- **Position Calculation**: Uses getBoundingClientRect() with scroll offset
- **Z-Index**: High z-index (9999) ensures visibility
- **Cleanup**: Proper event listener cleanup on unmount

### API Integration

When a message is sent with context:
```javascript
const contextText = activeContexts.join("\n\n---\n\n");
const fullMessage = `Context:\n${contextText}\n\nQuestion: ${userMessage}`;

// Sent to backend
POST http://localhost:8000/chat
{
  "message": fullMessage,
  "use_rag": true
}
```

### Styling

**Button Styles**:
- Primary color background
- White text
- Rounded pill shape (20px border-radius)
- Drop shadow for elevation
- Smooth transitions

**Context Display**:
- Light gray background section
- Individual context cards
- Paperclip icon indicator
- Context count badge
- Red remove buttons

### Browser Compatibility
- Modern browsers with Selection API support
- Touch-enabled devices supported
- Works with window scrolling

### Future Enhancements
- [ ] Keyboard shortcut (e.g., Ctrl+Shift+C)
- [ ] Context highlighting in original text
- [ ] Save contexts across sessions
- [ ] Context tagging/categorization
- [ ] Export context collection

## Usage Example

```typescript
// User selects text: "ROS 2 uses DDS for communication"
// Clicks "Add to Chat" button
// Context appears in chat widget

// User types: "What are the benefits of this approach?"

// Backend receives:
// Context:
// ROS 2 uses DDS for communication
//
// Question: What are the benefits of this approach?
```

## Testing

### Manual Testing Steps
1. Navigate to any documentation page
2. Select text (at least a few words)
3. Verify "Add to Chat" button appears
4. Click the button
5. Verify chat widget opens
6. Verify text appears in context section
7. Type a question and send
8. Verify context is included in API request

### Edge Cases Handled
- Empty selections ignored
- Multiple rapid selections
- Selection outside document area
- Chat already open when adding context
- Removing contexts individually
- Clearing all contexts after send

## Performance Considerations
- Debounced selection detection
- Minimal re-renders using React state
- Event listener cleanup prevents memory leaks
- CSS animations use transform (GPU accelerated)
- Position calculation only on selection change

## Files Modified

```
web/src/components/TextSelectionHandler.tsx (NEW)
web/src/components/ChatWidget.tsx (ENHANCED)
web/src/theme/Root.tsx (MODIFIED)
web/src/context/ChatContext.tsx (NO CHANGES - already supported)
web/src/css/custom.css (NO CHANGES - already had styles)
```

## Configuration

No configuration needed - feature is enabled by default on all pages.

To disable on specific pages, you could wrap content in a div with:
```typescript
<div onMouseUp={(e) => e.stopPropagation()}>
  Content where selection should not trigger button
</div>
```
