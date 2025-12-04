/**
 * API Configuration
 * Production URLs for Vercel serverless deployments
 */

// Determine if we're in production (GitHub Pages)
// Use typeof window to check if we're in a browser environment (not SSR)
const isProduction = typeof window !== 'undefined' && window.location.hostname === 'mathnj.github.io';

// Auth Server (Vercel Serverless)
export const AUTH_API_URL = isProduction
  ? 'https://textbook-auth-g9th03hud-mathnjs-projects.vercel.app/api'
  : 'http://localhost:3001/api';

// API Server (Vercel Serverless)
export const API_BASE_URL = isProduction
  ? 'https://textbook-iq4cebqvo-mathnjs-projects.vercel.app'
  : 'http://localhost:8000';

// Auth Client Base URL (for Better Auth)
export const AUTH_BASE_URL = isProduction
  ? 'https://textbook-auth-g9th03hud-mathnjs-projects.vercel.app'
  : 'http://localhost:3001';

// API Endpoints
export const API_ENDPOINTS = {
  chat: `${API_BASE_URL}/chat`,
  personalize: `${API_BASE_URL}/personalize`,
  search: `${API_BASE_URL}/search`,
  userPreferences: `${AUTH_API_URL}/user/preferences`,
  userProfile: `${AUTH_API_URL}/user/profile`,
};
