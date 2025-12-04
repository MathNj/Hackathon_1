/**
 * Better Auth React Client
 *
 * Connects to the Node.js auth service running on port 3001
 */
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: "http://localhost:3001",
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
