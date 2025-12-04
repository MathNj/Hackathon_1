/**
 * Auth Navbar Item Component
 *
 * Displays dynamic authentication button in the Docusaurus navbar
 * - Shows user name when authenticated
 * - Shows nothing (no login button) when not authenticated
 * - Uses Better-Auth React client for session management
 */
import React, { useEffect } from 'react';
import { authClient } from '../lib/auth-client';

export default function AuthNavbarItem() {
  const { data: session, isPending, refetch } = authClient.useSession();

  // Debug logging
  useEffect(() => {
    console.log('AuthNavbarItem - Session state:', {
      session,
      isPending,
      hasUser: !!session?.user,
      userName: session?.user?.name,
    });
  }, [session, isPending]);

  // Force refetch session on mount
  useEffect(() => {
    refetch();
  }, []);

  // Don't show anything while loading
  if (isPending) {
    return null;
  }

  // If logged in, show user name and logout button
  if (session?.user) {
    return (
      <div className='navbar__item' style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
        <span style={{ fontSize: '14px', fontWeight: '500' }}>
          Hi, {session.user.name || 'User'}
        </span>
        <button
          className='button button--secondary button--sm'
          onClick={async () => {
            await authClient.signOut();
            // Force page reload to update session in navbar
            window.location.href = '/';
          }}
          style={{ cursor: 'pointer' }}
        >
          Logout
        </button>
      </div>
    );
  }

  // If not logged in, show nothing (no login button)
  return null;
}
