/**
 * Better Auth React Client
 *
 * Connects to the auth service (localhost dev or Vercel production)
 */
import { createAuthClient } from "better-auth/react";
import { AUTH_BASE_URL } from "../config/api";

export const authClient = createAuthClient({
  baseURL: AUTH_BASE_URL,
  fetchOptions: {
    credentials: "include", // Send cookies with requests
  },
});

export type HardwareBackground =
  | "RTX 4090"
  | "Jetson Orin"
  | "Laptop CPU"
  | "Google Colab";

export type SkillLevel = "Beginner" | "Advanced";

export interface User {
  id: string;
  name: string;
  email: string;
  hardware_bg?: HardwareBackground;
  skill_level?: SkillLevel;
}
