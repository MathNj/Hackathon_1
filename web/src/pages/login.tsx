/**
 * Login/Signup Page
 *
 * Features:
 * - Email/password authentication
 * - Name field for signup
 * - Hardware preferences dropdown (RTX 4090, Jetson Orin, Laptop CPU, Google Colab)
 * - Toggle between login/signup modes
 */
import React, { useState } from "react";
import Layout from "@theme/Layout";
import { authClient, HardwareBackground, SkillLevel } from "../lib/auth-client";
import { useHistory } from "@docusaurus/router";

const HARDWARE_OPTIONS: HardwareBackground[] = [
  "RTX 4090",
  "Jetson Orin",
  "Laptop CPU",
  "Google Colab",
];

const SKILL_LEVEL_OPTIONS: SkillLevel[] = ["Beginner", "Advanced"];

export default function Login(): JSX.Element {
  const history = useHistory();
  const [isSignup, setIsSignup] = useState(false);
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [name, setName] = useState("");
  const [hardwareBg, setHardwareBg] = useState<HardwareBackground>("Laptop CPU");
  const [skillLevel, setSkillLevel] = useState<SkillLevel>("Beginner");
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");
    setLoading(true);

    try {
      if (isSignup) {
        // Signup
        await authClient.signUp.email({
          email,
          password,
          name,
          callbackURL: "/",
        });

        // Get the user ID from session
        const session = await authClient.getSession();

        if (session?.user?.id) {
          // Update hardware preference and skill level after signup
          try {
            const response = await fetch("http://localhost:3001/api/user/preferences", {
              method: "POST",
              headers: {
                "Content-Type": "application/json",
              },
              body: JSON.stringify({
                userId: session.user.id,
                hardware_bg: hardwareBg,
                skill_level: skillLevel,
              }),
            });

            if (!response.ok) {
              console.error("Failed to save preferences:", await response.text());
            }
          } catch (prefError) {
            console.error("Error saving preferences:", prefError);
            // Don't block signup on preferences save failure
          }
        }
      } else {
        // Login
        await authClient.signIn.email({
          email,
          password,
          callbackURL: "/",
        });
      }

      // Redirect to home on success
      history.push("/");
    } catch (err: any) {
      setError(err.message || "Authentication failed");
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Login" description="Login to Physical AI Textbook">
      <div
        style={{
          display: "flex",
          justifyContent: "center",
          alignItems: "center",
          minHeight: "calc(100vh - 60px)",
          padding: "20px",
        }}
      >
        <div
          style={{
            width: "100%",
            maxWidth: "400px",
            padding: "40px",
            borderRadius: "12px",
            boxShadow: "0 4px 12px rgba(0, 0, 0, 0.1)",
            background: "var(--ifm-background-surface-color)",
          }}
        >
          <h1 style={{ textAlign: "center", marginBottom: "30px" }}>
            {isSignup ? "Sign Up" : "Login"}
          </h1>

          {error && (
            <div
              style={{
                padding: "12px",
                marginBottom: "20px",
                background: "#ff4444",
                color: "white",
                borderRadius: "6px",
                fontSize: "14px",
              }}
            >
              {error}
            </div>
          )}

          <form onSubmit={handleSubmit}>
            {isSignup && (
              <div style={{ marginBottom: "20px" }}>
                <label
                  htmlFor="name"
                  style={{
                    display: "block",
                    marginBottom: "8px",
                    fontWeight: "600",
                  }}
                >
                  Name
                </label>
                <input
                  id="name"
                  type="text"
                  value={name}
                  onChange={(e) => setName(e.target.value)}
                  required={isSignup}
                  style={{
                    width: "100%",
                    padding: "10px",
                    borderRadius: "6px",
                    border: "1px solid var(--ifm-color-emphasis-300)",
                    fontSize: "16px",
                  }}
                />
              </div>
            )}

            <div style={{ marginBottom: "20px" }}>
              <label
                htmlFor="email"
                style={{
                  display: "block",
                  marginBottom: "8px",
                  fontWeight: "600",
                }}
              >
                Email
              </label>
              <input
                id="email"
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                required
                style={{
                  width: "100%",
                  padding: "10px",
                  borderRadius: "6px",
                  border: "1px solid var(--ifm-color-emphasis-300)",
                  fontSize: "16px",
                }}
              />
            </div>

            <div style={{ marginBottom: "20px" }}>
              <label
                htmlFor="password"
                style={{
                  display: "block",
                  marginBottom: "8px",
                  fontWeight: "600",
                }}
              >
                Password
              </label>
              <input
                id="password"
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                required
                style={{
                  width: "100%",
                  padding: "10px",
                  borderRadius: "6px",
                  border: "1px solid var(--ifm-color-emphasis-300)",
                  fontSize: "16px",
                }}
              />
            </div>

            {isSignup && (
              <>
                <div style={{ marginBottom: "20px" }}>
                  <label
                    htmlFor="hardware"
                    style={{
                      display: "block",
                      marginBottom: "8px",
                      fontWeight: "600",
                    }}
                  >
                    Hardware Background
                  </label>
                  <select
                    id="hardware"
                    value={hardwareBg}
                    onChange={(e) =>
                      setHardwareBg(e.target.value as HardwareBackground)
                    }
                    style={{
                      width: "100%",
                      padding: "10px",
                      borderRadius: "6px",
                      border: "1px solid var(--ifm-color-emphasis-300)",
                      fontSize: "16px",
                      background: "var(--ifm-background-surface-color)",
                    }}
                  >
                    {HARDWARE_OPTIONS.map((option) => (
                      <option key={option} value={option}>
                        {option}
                      </option>
                    ))}
                  </select>
                </div>

                <div style={{ marginBottom: "20px" }}>
                  <label
                    htmlFor="skillLevel"
                    style={{
                      display: "block",
                      marginBottom: "8px",
                      fontWeight: "600",
                    }}
                  >
                    Skill Level
                  </label>
                  <select
                    id="skillLevel"
                    value={skillLevel}
                    onChange={(e) =>
                      setSkillLevel(e.target.value as SkillLevel)
                    }
                    style={{
                      width: "100%",
                      padding: "10px",
                      borderRadius: "6px",
                      border: "1px solid var(--ifm-color-emphasis-300)",
                      fontSize: "16px",
                      background: "var(--ifm-background-surface-color)",
                    }}
                  >
                    {SKILL_LEVEL_OPTIONS.map((option) => (
                      <option key={option} value={option}>
                        {option}
                      </option>
                    ))}
                  </select>
                </div>
              </>
            )}

            <button
              type="submit"
              disabled={loading}
              style={{
                width: "100%",
                padding: "12px",
                background: "var(--ifm-color-primary)",
                color: "white",
                border: "none",
                borderRadius: "6px",
                fontSize: "16px",
                fontWeight: "600",
                cursor: loading ? "not-allowed" : "pointer",
                opacity: loading ? 0.7 : 1,
              }}
            >
              {loading ? "Processing..." : isSignup ? "Sign Up" : "Login"}
            </button>
          </form>

          <div style={{ textAlign: "center", marginTop: "20px" }}>
            <button
              onClick={() => setIsSignup(!isSignup)}
              style={{
                background: "none",
                border: "none",
                color: "var(--ifm-color-primary)",
                cursor: "pointer",
                fontSize: "14px",
                textDecoration: "underline",
              }}
            >
              {isSignup
                ? "Already have an account? Login"
                : "Don't have an account? Sign Up"}
            </button>
          </div>
        </div>
      </div>
    </Layout>
  );
}
