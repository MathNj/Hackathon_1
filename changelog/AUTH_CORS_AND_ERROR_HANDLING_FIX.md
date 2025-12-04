# Authentication CORS and Error Handling Fix

## Issues Fixed

### Issue 1: Better Auth CORS Error
**Error Message**: `ERROR [Better Auth]: Invalid origin: http://localhost:3000`

**Root Cause**: Better Auth was rejecting requests from the frontend (localhost:3000) because the origin wasn't in the trusted origins list.

**Solution**: Added `trustedOrigins` configuration to Better Auth config.

**File**: `auth/auth.config.ts`

```typescript
export const auth = betterAuth({
  database: pool,
  baseURL: AUTH_URL,
  secret: AUTH_SECRET,

  // CORS Configuration - Allow frontend origin
  trustedOrigins: ["http://localhost:3000"],

  // ... rest of config
});
```

### Issue 2: Immediate Redirect Without Showing Errors
**Problem**: When login/signup failed, the page would redirect to home immediately without showing the error message to the user.

**Root Cause**: The code was calling `history.push("/")` even when authentication failed because Better Auth client methods don't throw errors by default - they return result objects with an `error` property.

**Solution**:
1. Removed `callbackURL: "/"` from auth calls (this was causing automatic redirect)
2. Explicitly check for `result.error` and throw an error if present
3. Only redirect to home page if authentication succeeds (no error thrown)
4. Added console logging for debugging

**File**: `web/src/pages/login.tsx`

**Before**:
```typescript
await authClient.signUp.email({
  email,
  password,
  name,
  callbackURL: "/", // ❌ This causes automatic redirect
});

// Redirect always happened here
history.push("/");
```

**After**:
```typescript
const result = await authClient.signUp.email({
  email,
  password,
  name,
  // ✅ No callbackURL - manual control
});

// Check for errors in result
if (result.error) {
  throw new Error(result.error.message || "Signup failed");
}

// Only redirect on success
history.push("/");
```

## Flow After Fix

### Successful Login/Signup:
1. User submits form
2. Auth client call returns success (no error)
3. Background preferences saved (if signup with background text)
4. User redirected to home page
5. No error message shown

### Failed Login/Signup:
1. User submits form
2. Auth client call returns error
3. Error thrown and caught in catch block
4. Error message displayed in red banner
5. **User stays on login page** (no redirect)
6. User can see error and try again

## Error Handling Improvements

### Error Display
- Error message shown in red banner at top of form
- Console logging added for debugging (`console.error("Authentication error:", err)`)
- User-friendly fallback message: "Authentication failed. Please try again."

### Common Error Scenarios Now Handled:
- ✅ Invalid email/password (wrong credentials)
- ✅ User already exists (duplicate email on signup)
- ✅ Weak password
- ✅ Network errors
- ✅ Database connection issues
- ✅ CORS errors (now fixed)

## Testing

### Test Successful Login:
1. Go to http://localhost:3000/login
2. Enter valid credentials
3. Click "Login"
4. Should redirect to home page

### Test Failed Login:
1. Go to http://localhost:3000/login
2. Enter invalid credentials
3. Click "Login"
4. Should show error message: "Login failed"
5. Should **NOT** redirect - stay on login page

### Test Successful Signup:
1. Go to http://localhost:3000/login
2. Click "Don't have an account? Sign Up"
3. Fill in name, email, password, background (optional)
4. Click "Sign Up"
5. Should redirect to home page

### Test Failed Signup (Duplicate Email):
1. Try signing up with an email that already exists
2. Should show error: "User already exists"
3. Should **NOT** redirect - stay on signup page

## Files Modified

1. **auth/auth.config.ts**
   - Added `trustedOrigins: ["http://localhost:3000"]`

2. **web/src/pages/login.tsx**
   - Removed `callbackURL` from auth calls
   - Added explicit error checking (`if (result.error)`)
   - Only redirect on success
   - Added console logging

## Deployment Notes

### For Production:
Update `trustedOrigins` to include production URLs:

```typescript
trustedOrigins: [
  "http://localhost:3000",
  "https://your-production-domain.com",
  "https://your-github-pages-url.github.io"
]
```

### Environment Variables:
Ensure `.env` has correct CORS_ORIGINS:
```bash
CORS_ORIGINS=http://localhost:3000,https://your-production-domain.com
```

## Summary

Both authentication issues are now fixed:
- ✅ CORS errors resolved with `trustedOrigins` config
- ✅ Error messages now display properly without automatic redirect
- ✅ Users can see what went wrong and try again
- ✅ Better error handling with explicit error checks
- ✅ Console logging for debugging
