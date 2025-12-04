/**
 * Auth Navbar Item Component
 *
 * Displays dynamic authentication button in the Docusaurus navbar
 * - Shows "Login" button when user is not authenticated
 * - Shows user name and "Logout" button when authenticated
 * - Uses Better-Auth React client for session management
 */
import React from 'react';
import { authClient } from '../lib/auth-client';
import Link from '@docusaurus/Link';

export default function AuthNavbarItem() {
  const { data: session, isPending } = authClient.useSession();

  if (isPending) {
    return (
      <div className='navbar__item'>
        <span style={{ fontSize: '14px', opacity: 0.7 }}>Loading...</span>
      </div>
    );
  }

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

  return (
    <Link
      to='/login'
      className='button button--primary button--sm navbar__item'
      style={{ textDecoration: 'none' }}
    >
      Login
    </Link>
  );
}
