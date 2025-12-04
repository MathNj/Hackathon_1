/**
 * Authentication Server
 * Standalone Node.js server for Better Auth
 * Runs on port 3001
 */

// CRITICAL: Load environment variables FIRST
import dotenv from 'dotenv';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

// Load .env from parent directory (project root) BEFORE anything else
dotenv.config({ path: join(__dirname, '../.env') });

// Debug: Verify environment variables are loaded
console.log("🔍 Checking DB URL:", process.env.DATABASE_URL ? "Loaded ✅" : "Missing ❌");
console.log("🔍 Checking AUTH_SECRET:", process.env.AUTH_SECRET ? "Loaded ✅" : "Missing ❌");

// Now import dependencies (environment is loaded)
import express from "express";
import cors from "cors";
import { auth } from "./auth.config.js";

// Create Express app
const app = express();
const PORT = process.env.AUTH_PORT || 3001;

// CORS configuration
const CORS_ORIGINS = (
  process.env.CORS_ORIGINS || "http://localhost:3000,http://localhost:8000"
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
app.get("/", (req, res) => {
  res.json({
    status: "healthy",
    service: "Authentication Service",
    version: "1.0.0",
    timestamp: new Date().toISOString(),
  });
});

// Health check endpoint (detailed)
app.get("/health", async (req, res) => {
  try {
    // Check database connection
    const { pool } = await import("./auth.config.js");
    const dbResult = await pool.query("SELECT NOW()");

    res.json({
      status: "healthy",
      services: {
        api: "running",
        database: "connected",
        auth: "configured",
      },
      database: {
        connected: true,
        timestamp: dbResult.rows[0].now,
      },
      environment: process.env.NODE_ENV || "development",
    });
  } catch (error) {
    res.status(500).json({
      status: "unhealthy",
      error: error instanceof Error ? error.message : "Unknown error",
    });
  }
});

// Mount Better Auth routes
// All auth routes will be available at /api/auth/*
app.all("/api/auth/*", async (req, res) => {
  try {
    // Convert Express request to Web Request format
    const protocol = req.protocol || 'http';
    const host = req.get('host') || 'localhost:3001';
    const url = `${protocol}://${host}${req.url}`;

    const webRequest = new Request(url, {
      method: req.method,
      headers: req.headers as any,
      body: ['GET', 'HEAD'].includes(req.method) ? undefined : JSON.stringify(req.body),
    });

    const response = await auth.handler(webRequest);

    // Convert Web Response back to Express response
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
});

// Additional custom endpoints

/**
 * Update user preferences (background information)
 * POST /api/user/preferences
 */
app.post("/api/user/preferences", async (req, res) => {
  try {
    const { background, userId } = req.body;

    if (!userId) {
      return res.status(400).json({ error: "userId is required" });
    }

    if (!background || typeof background !== 'string') {
      return res.status(400).json({ error: "background text is required" });
    }

    // Update user in database
    const { pool } = await import("./auth.config.js");

    const query = `
      UPDATE "user"
      SET background = $1, "updatedAt" = NOW()
      WHERE id = $2
      RETURNING id, email, name, background, "createdAt", "updatedAt"
    `;

    const result = await pool.query(query, [background, userId]);

    if (result.rows.length === 0) {
      return res.status(404).json({ error: "User not found" });
    }

    res.json({
      success: true,
      user: result.rows[0],
      message: "User background updated successfully",
    });
  } catch (error) {
    console.error("Preferences update error:", error);
    res.status(500).json({
      error: "Failed to update preferences",
      message: error instanceof Error ? error.message : "Unknown error",
    });
  }
});

/**
 * Get user profile with background information
 * GET /api/user/profile
 */
app.get("/api/user/profile", async (req, res) => {
  try {
    // Extract user ID from Authorization header (Bearer token format)
    const authHeader = req.headers.authorization;

    if (!authHeader) {
      return res.status(401).json({ error: "Unauthorized" });
    }

    // Extract userId from Bearer token (format: "Bearer <userId>")
    const userId = authHeader.replace('Bearer ', '').trim();

    if (!userId) {
      return res.status(401).json({ error: "Invalid authorization token" });
    }

    // Fetch user from database
    const { pool } = await import("./auth.config.js");

    const result = await pool.query(
      `SELECT id, email, name, background, "createdAt", "updatedAt"
       FROM "user"
       WHERE id = $1`,
      [userId]
    );

    if (result.rows.length === 0) {
      return res.status(404).json({ error: "User not found" });
    }

    res.json(result.rows[0]);
  } catch (error) {
    console.error("Profile fetch error:", error);
    res.status(500).json({
      error: "Failed to fetch profile",
      message: error instanceof Error ? error.message : "Unknown error",
    });
  }
});

// Error handling middleware
app.use(
  (
    err: Error,
    req: express.Request,
    res: express.Response,
    next: express.NextFunction
  ) => {
    console.error("Unhandled error:", err);
    res.status(500).json({
      error: "Internal server error",
      message:
        process.env.NODE_ENV === "development" ? err.message : "Server error",
    });
  }
);

// 404 handler
app.use((req, res) => {
  res.status(404).json({
    error: "Not found",
    path: req.path,
  });
});

// Start server
app.listen(PORT, () => {
  console.log("\n🚀 Authentication Server Started");
  console.log(`   ├─ Port: ${PORT}`);
  console.log(`   ├─ Environment: ${process.env.NODE_ENV || "development"}`);
  console.log(`   ├─ Auth URL: ${process.env.AUTH_URL || `http://localhost:${PORT}`}`);
  console.log(`   └─ Database: ${process.env.DATABASE_URL ? "Connected" : "Not configured"}\n`);
  console.log("📚 Available endpoints:");
  console.log(`   ├─ GET  /              - Health check`);
  console.log(`   ├─ GET  /health        - Detailed health check`);
  console.log(`   ├─ ALL  /api/auth/*    - Better Auth endpoints`);
  console.log(`   ├─ POST /api/user/preferences - Update user preferences`);
  console.log(`   └─ GET  /api/user/profile  - Get user profile\n`);
});

// Graceful shutdown
process.on("SIGTERM", () => {
  console.log("\n⚠️  SIGTERM received, shutting down gracefully...");
  process.exit(0);
});

process.on("SIGINT", () => {
  console.log("\n⚠️  SIGINT received, shutting down gracefully...");
  process.exit(0);
});

export default app;
