/**
 * Auth Navbar Item Component
 *
 * Displays dynamic authentication button in the Docusaurus navbar
 * - Shows user name when authenticated
 * - Shows nothing (no login button) when not authenticated
 * - Uses custom SessionContext for session management
 */
import React from 'react';
import { useSession } from '../contexts/SessionContext';

export default function AuthNavbarItem() {
  const { session, loading, logout } = useSession();

  // Don't show anything while loading
  if (loading) {
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
            await logout();
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
