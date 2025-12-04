/**
 * Vercel Serverless Function for Better Auth
 * Entry point for auth/* routes
 */

import dotenv from 'dotenv';
dotenv.config();

import express from "express";
import cors from "cors";
import { auth } from "../auth.config.js";

// Create Express app
const app = express();

// CORS configuration
const CORS_ORIGINS = (
  process.env.CORS_ORIGINS || "http://localhost:3000"
).split(",");

app.use(
  cors({
    origin: CORS_ORIGINS,
    credentials: true,
    methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allowedHeaders: ["Content-Type", "Authorization"],
    exposedHeaders: ["Set-Cookie"],
  })
);

// Body parsing middleware
app.use(express.json());
app.use(express.urlencoded({ extended: true }));

// Health check endpoint
app.get("/api/health", (req, res) => {
  res.json({
    status: "healthy",
    service: "auth",
    timestamp: new Date().toISOString()
  });
});

// Better Auth routes - mount at /api/auth
app.all("/api/auth/*", auth.handler);

// User preferences endpoint
app.get("/api/user/preferences", async (req, res) => {
  try {
    const session = await auth.api.getSession({ headers: req.headers });

    if (!session) {
      return res.status(401).json({ error: "Unauthorized" });
    }

    const user = session.user;
    res.json({
      hardware_bg: user.hardware_bg || null,
      skill_level: user.skill_level || null
    });
  } catch (error) {
    console.error("Error fetching preferences:", error);
    res.status(500).json({ error: "Failed to fetch preferences" });
  }
});

// User profile endpoint
app.get("/api/user/profile", async (req, res) => {
  try {
    const session = await auth.api.getSession({ headers: req.headers });

    if (!session) {
      return res.status(401).json({ error: "Unauthorized" });
    }

    res.json(session.user);
  } catch (error) {
    console.error("Error fetching profile:", error);
    res.status(500).json({ error: "Failed to fetch profile" });
  }
});

// Export as Vercel serverless function
export default app;
