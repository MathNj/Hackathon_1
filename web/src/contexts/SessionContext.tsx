/**
 * Custom Session Context
 *
 * Provides session management using localStorage instead of cookies
 * to work around Better Auth's cross-port cookie issues.
 *
 * Features:
 * - Direct fetch API calls to auth server
 * - localStorage-based session token storage
 * - React context for session state
 * - Auto-refresh session on mount
 */
import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

interface User {
  id: string;
  email: string;
  name?: string;
  background?: string;
  createdAt: string;
  updatedAt: string;
}

interface Session {
  user: User;
  token: string;
}

interface SessionContextType {
  session: Session | null;
  loading: boolean;
  login: (email: string, password: string) => Promise<void>;
  signup: (email: string, password: string, name: string, background?: string) => Promise<void>;
  logout: () => Promise<void>;
  refreshSession: () => Promise<void>;
}

const SessionContext = createContext<SessionContextType | undefined>(undefined);

const AUTH_API_URL = 'http://localhost:3001/api/auth';
const SESSION_TOKEN_KEY = 'textbook_session_token';

export function SessionProvider({ children }: { children: ReactNode }) {
  const [session, setSession] = useState<Session | null>(null);
  const [loading, setLoading] = useState(true);

  /**
   * Refresh session from server or localStorage
   */
  const refreshSession = async () => {
    const token = localStorage.getItem(SESSION_TOKEN_KEY);

    if (!token) {
      setSession(null);
      setLoading(false);
      return;
    }

    // If using local session (no cookie), load from localStorage
    if (token.startsWith('local_')) {
      const userDataStr = localStorage.getItem(`${SESSION_TOKEN_KEY}_user`);
      if (userDataStr) {
        try {
          const user = JSON.parse(userDataStr);
          setSession({ user, token });
          setLoading(false);
          return;
        } catch (error) {
          console.error('Error parsing stored user data:', error);
          localStorage.removeItem(SESSION_TOKEN_KEY);
          localStorage.removeItem(`${SESSION_TOKEN_KEY}_user`);
          setSession(null);
          setLoading(false);
          return;
        }
      }
    }

    // Try to get session from server
    try {
      const response = await fetch(`${AUTH_API_URL}/get-session`, {
        method: 'GET',
        credentials: 'include',
      });

      if (response.ok) {
        const data = await response.json();

        if (data.user || data.session?.user) {
          const user = data.user || data.session.user;
          setSession({
            user: user,
            token: token,
          });
        } else {
          // No session on server, clear local session
          localStorage.removeItem(SESSION_TOKEN_KEY);
          localStorage.removeItem(`${SESSION_TOKEN_KEY}_user`);
          setSession(null);
        }
      } else {
        // Session expired or invalid
        localStorage.removeItem(SESSION_TOKEN_KEY);
        localStorage.removeItem(`${SESSION_TOKEN_KEY}_user`);
        setSession(null);
      }
    } catch (error) {
      console.error('Error refreshing session:', error);
      // On error, still try to use localStorage as fallback
      const userDataStr = localStorage.getItem(`${SESSION_TOKEN_KEY}_user`);
      if (userDataStr) {
        try {
          const user = JSON.parse(userDataStr);
          setSession({ user, token });
        } catch {
          setSession(null);
        }
      } else {
        setSession(null);
      }
    } finally {
      setLoading(false);
    }
  };

  /**
   * Login with email and password
   */
  const login = async (email: string, password: string) => {
    const response = await fetch(`${AUTH_API_URL}/sign-in/email`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      credentials: 'include',
      body: JSON.stringify({ email, password }),
    });

    const data = await response.json();

    if (!response.ok || data.error) {
      throw new Error(data.error?.message || 'Login failed');
    }

    // Better Auth returns user and session data directly
    if (data.user) {
      // Extract session token from cookies (Better Auth sets it automatically)
      const cookies = document.cookie.split('; ');
      const sessionCookie = cookies.find(c => c.startsWith('textbook.session_token='));

      if (sessionCookie) {
        const token = sessionCookie.split('=')[1];
        localStorage.setItem(SESSION_TOKEN_KEY, token);

        // Set session immediately with response data
        setSession({
          user: data.user,
          token: token,
        });
      } else {
        // Fallback: Generate session ID and store user data
        const sessionId = `local_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
        localStorage.setItem(SESSION_TOKEN_KEY, sessionId);
        localStorage.setItem(`${SESSION_TOKEN_KEY}_user`, JSON.stringify(data.user));

        setSession({
          user: data.user,
          token: sessionId,
        });
      }
    } else {
      throw new Error('No user data received from server');
    }
  };

  /**
   * Sign up with email, password, name, and optional background
   */
  const signup = async (email: string, password: string, name: string, background?: string) => {
    const response = await fetch(`${AUTH_API_URL}/sign-up/email`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      credentials: 'include',
      body: JSON.stringify({ email, password, name }),
    });

    const data = await response.json();

    if (!response.ok || data.error) {
      throw new Error(data.error?.message || 'Signup failed');
    }

    // Better Auth returns user data directly
    if (data.user) {
      // Extract session token from cookies
      const cookies = document.cookie.split('; ');
      const sessionCookie = cookies.find(c => c.startsWith('textbook.session_token='));

      let token: string;
      if (sessionCookie) {
        token = sessionCookie.split('=')[1];
      } else {
        // Fallback: Generate local session ID
        token = `local_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
        localStorage.setItem(`${SESSION_TOKEN_KEY}_user`, JSON.stringify(data.user));
      }

      localStorage.setItem(SESSION_TOKEN_KEY, token);

      // Set session immediately
      setSession({
        user: data.user,
        token: token,
      });

      // If background provided, save it
      if (background && background.trim() && data.user.id) {
        try {
          const bgResponse = await fetch('http://localhost:3001/api/user/preferences', {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({
              userId: data.user.id,
              background: background.trim(),
            }),
          });

          if (bgResponse.ok) {
            // Update session with new background
            setSession({
              user: { ...data.user, background: background.trim() },
              token: token,
            });
          }
        } catch (error) {
          console.error('Error saving background:', error);
          // Don't block signup on background save failure
        }
      }
    } else {
      throw new Error('No user data received from server');
    }
  };

  /**
   * Logout and clear session
   */
  const logout = async () => {
    try {
      await fetch(`${AUTH_API_URL}/sign-out`, {
        method: 'POST',
        credentials: 'include',
      });
    } catch (error) {
      console.error('Error signing out:', error);
    }

    // Clear local session
    localStorage.removeItem(SESSION_TOKEN_KEY);
    localStorage.removeItem(`${SESSION_TOKEN_KEY}_user`);
    setSession(null);
  };

  // Refresh session on mount
  useEffect(() => {
    refreshSession();
  }, []);

  const value: SessionContextType = {
    session,
    loading,
    login,
    signup,
    logout,
    refreshSession,
  };

  return (
    <SessionContext.Provider value={value}>
      {children}
    </SessionContext.Provider>
  );
}

/**
 * Hook to access session context
 */
export function useSession() {
  const context = useContext(SessionContext);

  if (context === undefined) {
    throw new Error('useSession must be used within a SessionProvider');
  }

  return context;
}
