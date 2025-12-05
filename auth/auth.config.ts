/**
 * Better Auth Configuration
 * Authentication service for Physical AI Textbook
 */

import { betterAuth } from "better-auth";
import { Pool } from "pg";
import dotenv from "dotenv";
import { fileURLToPath } from "url";
import { dirname, join } from "path";

// ES module path resolution
const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

// Load environment variables from project root
dotenv.config({ path: join(__dirname, "../.env") });

// Debug: Show what was loaded
console.log("🔍 [auth.config] DATABASE_URL:", process.env.DATABASE_URL ? "Loaded ✅" : "Missing ❌");
console.log("🔍 [auth.config] AUTH_SECRET:", process.env.AUTH_SECRET ? "Loaded ✅" : "Missing ❌");

// Validate required environment variables
const DATABASE_URL = process.env.DATABASE_URL;
const AUTH_SECRET = process.env.AUTH_SECRET;
const AUTH_URL = process.env.AUTH_URL || "http://localhost:3001";

if (!DATABASE_URL) {
  throw new Error("DATABASE_URL is not set in environment variables");
}

if (DATABASE_URL.includes("user:password@host:port")) {
  throw new Error(
    "DATABASE_URL contains placeholder values. Please update .env with actual Neon Postgres credentials.\n" +
    "Get your connection string from: https://console.neon.tech/\n" +
    "Expected format: postgresql://username:password@ep-xxx-xxx.region.aws.neon.tech/dbname?sslmode=require"
  );
}

if (!AUTH_SECRET) {
  throw new Error("AUTH_SECRET is not set in environment variables");
}

if (AUTH_SECRET.includes("your_random_secret") || AUTH_SECRET.length < 32) {
  throw new Error(
    "AUTH_SECRET contains placeholder value or is too short (min 32 characters).\n" +
    "Generate a secure secret with: node -e \"console.log(require('crypto').randomBytes(32).toString('hex'))\""
  );
}

// Create PostgreSQL connection pool
const pool = new Pool({
  connectionString: DATABASE_URL,
  ssl: {
    rejectUnauthorized: false, // Required for Neon
  },
});

// Test database connection
pool.query("SELECT NOW()", (err, res) => {
  if (err) {
    console.error("Database connection error:", err);
  } else {
    console.log("✓ Connected to Neon Postgres at:", res.rows[0].now);
  }
});

/**
 * Hardware Background Enum
 * Represents the user's hardware setup for personalized content
 */
export enum HardwareBackground {
  RTX4090 = "RTX4090",
  Jetson = "Jetson",
  Laptop = "Laptop",
  Cloud = "Cloud",
}

/**
 * Skill Level Enum
 * Represents the user's technical skill level for content personalization
 */
export enum SkillLevel {
  Beginner = "Beginner",
  Advanced = "Advanced",
}

/**
 * Extended User Schema
 * Adds hardware_bg and skill_level fields to default user model
 */
export interface ExtendedUser {
  id: string;
  email: string;
  name?: string;
  emailVerified: boolean;
  createdAt: Date;
  updatedAt: Date;
  // Custom fields
  hardware_bg?: HardwareBackground;
  skill_level?: SkillLevel;
}

/**
 * Better Auth Instance Configuration
 */
export const auth = betterAuth({
  // Database configuration - pass pool directly
  database: pool,

  // Base URL for auth service
  baseURL: AUTH_URL,

  // Base path for auth routes (defaults to /api/auth, we want just /api)
  basePath: "/api",

  // Secret for JWT signing and session encryption
  secret: AUTH_SECRET,

  // CORS Configuration - Allow frontend origin
  trustedOrigins: ["http://localhost:3000"],

  // Email and password authentication
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Set to true in production
    sendResetPasswordEmail: async (user: any, url: string) => {
      // TODO: Implement email sending (Phase 5)
      console.log(`Password reset URL for ${user.email}: ${url}`);
    },
    sendVerificationEmail: async (user: any, url: string) => {
      // TODO: Implement email sending (Phase 5)
      console.log(`Verification URL for ${user.email}: ${url}`);
    },
  },

  // Session configuration
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // Update session every 24 hours
    cookieCache: {
      enabled: true,
      maxAge: 5 * 60, // 5 minutes
    },
  },

  // User schema extension
  user: {
    additionalFields: {
      hardware_bg: {
        type: "string",
        required: false,
        defaultValue: HardwareBackground.Laptop,
        validate: (value: string) => {
          return Object.values(HardwareBackground).includes(
            value as HardwareBackground
          );
        },
      },
      skill_level: {
        type: "string",
        required: false,
        input: true,
        defaultValue: SkillLevel.Beginner,
        validate: (value: string) => {
          return Object.values(SkillLevel).includes(value as SkillLevel);
        },
      },
    },
  },

  // Advanced options
  advanced: {
    generateId: () => {
      // Generate UUID for user IDs
      return crypto.randomUUID();
    },
    cookiePrefix: "textbook",
  },
});

/**
 * Auth Types Export
 * Use these types in your application for type safety
 */
export type AuthSession = typeof auth.$Infer.Session;
// export type AuthUser = typeof auth.$Infer.User; // Commented out due to type issue

// Export pool for manual queries if needed
export { pool };
