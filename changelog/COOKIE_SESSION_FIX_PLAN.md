# Cookie Session Fix Plan

## Problem
Session is **null** after login because cookies set by `localhost:3001` (auth server) are not accessible to `localhost:3000` (web app).

## Root Cause
Browsers treat different ports as different origins for cookie security. When the auth server (port 3001) sets a session cookie:
- Cookie domain: `localhost:3001`
- React app domain: `localhost:3000`
- **Browser blocks cross-port cookie access**

## Why This Happens
```
User logs in → Auth server sets cookie on localhost:3001
              ↓
React app (localhost:3000) tries to read cookie
              ↓
Browser: "Cookie is for different port, access denied"
              ↓
Result: session = null, username = undefined
```

## Solutions (3 Options)

### Option 1: Run Everything on Same Port (RECOMMENDED) ✅
**Use Docusaurus proxy to forward auth requests**

**Advantages**:
- All requests from same origin (localhost:3000)
- Cookies work automatically
- No CORS issues
- Production-ready pattern

**Implementation**:
1. Add proxy in `docusaurus.config.js`:
```javascript
module.exports = {
  // ...
  proxy: {
    '/api/auth': {
      target: 'http://localhost:3001',
      changeOrigin: true,
    },
  },
};
```

2. Update auth client to use relative URL:
```typescript
export const authClient = createAuthClient({
  baseURL: "/api/auth", // Relative URL, proxied to port 3001
});
```

3. Restart Docusaurus dev server

### Option 2: Use Subdomain Instead of Different Ports
**Use app.localhost:3000 and auth.localhost:3001**

**Advantages**:
- Cookies can be shared with `.localhost` domain
- More production-like setup

**Disadvantages**:
- Requires hosts file modification
- More complex setup
- Still has CORS issues

### Option 3: Embed Auth Server in Docusaurus
**Run auth server as Express middleware in Docusaurus**

**Disadvantages**:
- Complex integration
- Docusaurus doesn't easily support custom Express middleware
- Not recommended

## Recommended Approach: Option 1 (Proxy)

### Step 1: Configure Docusaurus Proxy
Add to `web/docusaurus.config.js`:

```javascript
module.exports = {
  // ... existing config

  // Development server configuration
  scripts: [],

  // ADDED: Proxy configuration
  webpack: {
    jsLoader: (isServer) => ({
      loader: require.resolve('swc-loader'),
      options: {
        jsc: {
          parser: {
            syntax: 'typescript',
            tsx: true,
          },
          target: 'es2017',
        },
        module: {
          type: isServer ? 'commonjs' : 'es6',
        },
      },
    }),
  },
};
```

**Actually, Docusaurus doesn't have built-in proxy like Create React App.**

### Alternative: Custom Server Middleware

Since Docusaurus doesn't support proxy easily, we need to:

1. **Keep auth server on port 3001**
2. **Fix cookie sharing with proper CORS and credentials**
3. **Use `withCredentials` in all fetch requests**

### Proper Fix: Update Auth Client Fetch Configuration

The issue is the auth client isn't properly configured for cross-origin cookies.

## Actual Solution: Fix Better Auth Client Setup

Update `web/src/lib/auth-client.ts`:

```typescript
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: "http://localhost:3001", // Keep explicit URL
  fetchOptions: {
    credentials: "include", // Already set
    mode: "cors",
  },
  // Force client to use cookies
  storage: {
    get: (key) => {
      // Read from cookies instead of localStorage
      const cookies = document.cookie.split('; ');
      const cookie = cookies.find(c => c.startsWith(`${key}=`));
      return cookie ? cookie.split('=')[1] : null;
    },
    set: (key, value) => {
      // Store in cookies
      document.cookie = `${key}=${value}; path=/; domain=localhost`;
    },
    remove: (key) => {
      document.cookie = `${key}=; path=/; domain=localhost; expires=Thu, 01 Jan 1970 00:00:00 GMT`;
    },
  },
});
```

## Quick Test to Verify Cookie Issue

Run this in browser console after login:
```javascript
// Check if cookie exists
console.log("All cookies:", document.cookie);

// Try to get session
authClient.getSession().then(session => {
  console.log("Session:", session);
});

// Check auth server directly
fetch('http://localhost:3001/api/auth/get-session', {
  credentials: 'include'
}).then(r => r.json()).then(console.log);
```

## Next Steps

1. First, verify auth server is actually setting cookies correctly
2. Check browser DevTools → Network → Login request → Response Headers → Set-Cookie
3. If Set-Cookie header is there, check if cookie appears in Application → Cookies
4. If cookie doesn't appear, it's being blocked by browser
5. If cookie appears but has wrong domain, fix auth server cookie settings
