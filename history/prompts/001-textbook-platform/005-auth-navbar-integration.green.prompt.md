---
id: 005
title: Frontend Integrate Better Auth Login Button Navbar
stage: green
date: 2025-12-03
surface: agent
model: claude-sonnet-4-5-20250929
feature: textbook-platform
branch: master
user: Hamdan
command: /sp.implement
labels: ["frontend", "authentication", "navbar", "better-auth", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - web/src/components/AuthNavbarItem.tsx
  - web/src/theme/NavbarItem/ComponentTypes.tsx
  - web/docusaurus.config.ts
tests:
  - Manual testing: Check loading state displays
  - Manual testing: Verify login button for unauthenticated users
  - Manual testing: Verify user name and logout for authenticated users
  - Manual testing: Test logout functionality
  - Manual testing: Verify session persistence across navigation
---

## Prompt

"Frontend: Integrate Better-Auth Login Button in Navbar

**Goal:** Add a dynamic authentication button to the Docusaurus Navbar using the Better-Auth React client.

### 1. Create Auth Component (`web/src/components/AuthNavbarItem.tsx`)
*   **Action:** Create a React component that uses the Better-Auth session hook.
*   **Imports:** `import { authClient } from '../lib/auth-client';`
*   **Logic:**
    ```tsx
    import React from 'react';
    import { authClient } from '../lib/auth-client';
    import Link from '@docusaurus/Link';

    export default function AuthNavbarItem() {
      const { data: session, isPending } = authClient.useSession();

      if (isPending) return <div className='navbar__item'>Loading...</div>;

      if (session) {
        return (
          <div className='navbar__item'>
            <span style={{marginRight: '10px'}}>Hi, {session.user.name}</span>
            <button
              className='button button--secondary button--sm'
              onClick={() => authClient.signOut()}
            >
              Logout
            </button>
          </div>
        );
      }

      return (
        <Link to='/login' className='button button--primary button--sm navbar__item'>
          Login
        </Link>
      );
    }
    ```

### 2. Register Custom Component
*   **Action:** Add to NavbarItem ComponentTypes
*   **File:** `web/src/theme/NavbarItem/ComponentTypes.tsx`

### 3. Update Docusaurus Config
*   **Action:** Replace static login link with custom component
*   **Config:** Add `{ type: 'custom-authNavbarItem', position: 'right' }`"

## Response snapshot

Successfully integrated Better-Auth dynamic login button into the Docusaurus navbar with three distinct states:

### 1. AuthNavbarItem Component Created
**File:** `web/src/components/AuthNavbarItem.tsx`

**Implementation:**
- Uses `authClient.useSession()` hook for reactive session management
- Three display states:
  1. **Loading**: Shows "Loading..." while session is being fetched
  2. **Authenticated**: Shows "Hi, {name}" + Logout button
  3. **Unauthenticated**: Shows Login button linking to /login page

**Key Features:**
- Reactive updates: No page refresh needed when auth state changes
- Better-Auth signOut() integration for logout functionality
- Docusaurus button styling for consistent theme integration
- Flexbox layout for authenticated state with name + logout button
- Handles edge case where user.name might be undefined

### 2. Custom Component Registration
**File:** `web/src/theme/NavbarItem/ComponentTypes.tsx`

**Changes:**
- Imported AuthNavbarItem component
- Registered as `custom-authNavbarItem` type
- Extended existing ComponentTypes with new auth item
- Maintains existing `custom-langSwitcher` registration

**Code:**
```typescript
export default {
  ...ComponentTypes,
  "custom-langSwitcher": LangSwitcher,
  "custom-authNavbarItem": AuthNavbarItem,  // NEW
};
```

### 3. Navbar Configuration Update
**File:** `web/docusaurus.config.ts`

**Changes:**
- Replaced static login link with dynamic auth component
- Position: Right side of navbar
- Order: After language switcher, before any other right items

**Before:**
```typescript
{
  to: '/login',
  label: 'Login',
  position: 'right',
}
```

**After:**
```typescript
{
  type: 'custom-authNavbarItem',
  position: 'right',
}
```

### Component Behavior

**Session Hook Lifecycle:**
1. Component mounts → `isPending = true` → Shows "Loading..."
2. Session fetched from auth service
3. Result determined:
   - If session exists → `isPending = false`, show name + logout
   - If no session → `isPending = false`, show login button

**Logout Flow:**
1. User clicks "Logout" button
2. Calls `authClient.signOut()`
3. Session cleared in auth service
4. useSession() hook detects change
5. Component automatically re-renders to login button

**Login Flow:**
1. User clicks "Login" button
2. Navigates to /login page
3. User authenticates
4. Session created in auth service
5. useSession() hook detects new session
6. Component automatically re-renders to show name + logout

### Visual States

**Loading State:**
```
[Docs] [اردو] Loading...
```

**Unauthenticated:**
```
[Docs] [اردو] [Login]
```

**Authenticated:**
```
[Docs] [اردو] Hi, John [Logout]
```

### Integration Architecture

```
Navbar
  └─> AuthNavbarItem
        └─> authClient.useSession()
              └─> Better-Auth Service (port 3001)
                    └─> Neon Postgres
```

### Files Modified
- Created: `web/src/components/AuthNavbarItem.tsx`
- Modified: `web/src/theme/NavbarItem/ComponentTypes.tsx`
- Modified: `web/docusaurus.config.ts`

### Documentation
Created comprehensive summary: `AUTH_NAVBAR_INTEGRATION_SUMMARY.md`
- Component implementation details
- Session hook behavior
- User workflows and scenarios
- Visual state diagrams
- Testing checklist
- Troubleshooting guide

## Outcome

- ✅ Impact: Users now have a dynamic authentication button in the navbar that reactively updates based on their login state. Authenticated users see a personalized greeting and can logout with one click.
- 🧪 Tests: Manual testing required for: loading state, login button navigation, authenticated display, logout functionality, session persistence
- 📁 Files: Created 1 file (AuthNavbarItem component), Modified 2 files (ComponentTypes registration, navbar config)
- 🔁 Next prompts: Add user profile dropdown, implement avatar display, add session expiry warnings
- 🧠 Reflection: Clean integration with Docusaurus navbar system. Better-Auth useSession hook provides excellent reactive updates. Component handles all edge cases (loading, no name, etc.). Styling matches Docusaurus theme perfectly.

## Evaluation notes (flywheel)

- Failure modes observed: None during implementation. Potential issues: Auth service must be running for component to work, CORS issues if services on different domains, session cookie security in production
- Graders run and results (PASS/FAIL): No automated graders. Manual testing needed for: loading state (PENDING), unauthenticated login button (PENDING), authenticated name display (PENDING), logout functionality (PENDING), session persistence (PENDING)
- Prompt variant (if applicable): v1-better-auth-navbar (direct Better-Auth React hook integration)
- Next experiment (smallest change to try): Add loading skeleton animation instead of plain text for better perceived performance during session fetch
