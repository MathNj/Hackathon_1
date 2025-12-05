/**
 * Vercel Serverless Function for Auth Server
 * Wraps the Express app for Vercel deployment
 */

import type { VercelRequest, VercelResponse } from '@vercel/node';
import app from '../server.js';

// Export the Express app as a Vercel serverless function
export default async function handler(req: VercelRequest, res: VercelResponse) {
  return app(req, res);
}
