/**
 * Better Auth Configuration for CLI
 */
import { betterAuth } from "better-auth";
import { Pool } from "pg";

const DATABASE_URL = "postgresql://neondb_owner:npg_v4QPVme0tKHu@ep-shy-frost-a1ve24ag-pooler.ap-southeast-1.aws.neon.tech/neondb?sslmode=require";
const AUTH_SECRET = "a127cf7a45d2612a4c738ff670e1c23b6367264c72d04b7ed60908d2ff4924ef";

const pool = new Pool({
  connectionString: DATABASE_URL,
  ssl: {
    rejectUnauthorized: false,
  },
});

export const auth = betterAuth({
  database: pool,
  emailAndPassword: {
    enabled: true,
  },
  secret: AUTH_SECRET,
  trustedOrigins: ["http://localhost:3000"],
});
