/**
 * Vercel Serverless Function for Auth Server
 * All requests to /api/* are handled here
 */

import type { VercelRequest, VercelResponse } from '@vercel/node';

// Now import dependencies (environment is loaded)
import cors from 'cors';
import { auth } from '../auth.config.js';

// CORS configuration
const CORS_ORIGINS_RAW = process.env.CORS_ORIGINS || "http://localhost:3000,http://localhost:8000,https://mathnj.github.io";
const CORS_ORIGINS = CORS_ORIGINS_RAW
  .split(",")
  .map(origin => origin.trim())
  .filter(origin => origin.length > 0);

const corsMiddleware = cors({
  origin: CORS_ORIGINS,
  credentials: true,
  methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
  allowedHeaders: ["Content-Type", "Authorization"],
  exposedHeaders: ["Set-Cookie"],
});

// Helper to run middleware
function runMiddleware(req: VercelRequest, res: VercelResponse, fn: Function) {
  return new Promise((resolve, reject) => {
    fn(req, res, (result: any) => {
      if (result instanceof Error) {
        return reject(result);
      }
      return resolve(result);
    });
  });
}

export default async function handler(req: VercelRequest, res: VercelResponse) {
  // Run CORS middleware
  await runMiddleware(req, res, corsMiddleware);

  // Handle OPTIONS preflight
  if (req.method === 'OPTIONS') {
    return res.status(200).end();
  }

  try {
    // Convert Vercel request to Web Request format
    const protocol = 'https';
    const host = req.headers.host || 'textbook-auth-nqtn4u3kg-mathnjs-projects.vercel.app';
    const url = `${protocol}://${host}${req.url}`;

    const webRequest = new Request(url, {
      method: req.method || 'GET',
      headers: req.headers as any,
      body: ['GET', 'HEAD'].includes(req.method || 'GET') ? undefined : JSON.stringify(req.body),
    });

    const response = await auth.handler(webRequest);

    // Convert Web Response back to Vercel response
    const body = await response.text();
    res.status(response.status);
    response.headers.forEach((value, key) => {
      res.setHeader(key, value);
    });
    res.send(body);
  } catch (error) {
    console.error("Auth handler error:", error);
    res.status(500).json({
      error: "Authentication error",
      message: error instanceof Error ? error.message : "Unknown error",
    });
  }
}
