/**
 * Login/Signup Page
 *
 * Features:
 * - Email/password authentication
 * - Name field for signup
 * - Background textarea for user preferences
 * - Toggle between login/signup modes
 * - Uses custom SessionContext instead of Better Auth client
 */
import React, { useState } from "react";
import Layout from "@theme/Layout";
import { useSession } from "../contexts/SessionContext";
import { useHistory } from "@docusaurus/router";
import useBaseUrl from "@docusaurus/useBaseUrl";

export default function Login(): JSX.Element {
  const history = useHistory();
  const baseUrl = useBaseUrl('/');
  const { session, login: sessionLogin, signup: sessionSignup, loading: sessionLoading } = useSession();
  const [isSignup, setIsSignup] = useState(false);
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [name, setName] = useState("");
  const [background, setBackground] = useState("");
  const [error, setError] = useState("");
  const [success, setSuccess] = useState("");
  const [loading, setLoading] = useState(false);

  // Redirect if already logged in
  React.useEffect(() => {
    if (!sessionLoading && session?.user) {
      console.log("User already logged in, redirecting to home...");
      window.location.href = baseUrl;
    }
  }, [session, sessionLoading, baseUrl]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");
    setSuccess("");
    setLoading(true);

    try {
      if (isSignup) {
        // Signup with custom session context
        await sessionSignup(email, password, name, background.trim());
        setSuccess("Account created successfully! Redirecting...");
      } else {
        // Login with custom session context
        await sessionLogin(email, password);
        setSuccess("Login successful! Redirecting...");
      }

      // Reload page to update navbar and session state
      setTimeout(() => {
        window.location.href = baseUrl;
      }, 1000);

    } catch (err: any) {
      console.error("Authentication error:", err);
      setError(err.message || "Authentication failed. Please try again.");
      setSuccess("");
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

          {success && (
            <div
              style={{
                padding: "12px",
                marginBottom: "20px",
                background: "#28a745",
                color: "white",
                borderRadius: "6px",
                fontSize: "14px",
              }}
            >
              {success}
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
              <div style={{ marginBottom: "20px" }}>
                <label
                  htmlFor="background"
                  style={{
                    display: "block",
                    marginBottom: "8px",
                    fontWeight: "600",
                  }}
                >
                  Tell us about your background
                </label>
                <textarea
                  id="background"
                  value={background}
                  onChange={(e) => setBackground(e.target.value)}
                  placeholder="What's your experience with robotics, programming, or AI? What hardware do you have access to? (Optional)"
                  rows={4}
                  style={{
                    width: "100%",
                    padding: "10px",
                    borderRadius: "6px",
                    border: "1px solid var(--ifm-color-emphasis-300)",
                    fontSize: "16px",
                    background: "var(--ifm-background-surface-color)",
                    fontFamily: "inherit",
                    resize: "vertical",
                  }}
                />
                <div
                  style={{
                    fontSize: "13px",
                    color: "var(--ifm-color-emphasis-600)",
                    marginTop: "6px",
                  }}
                >
                  This helps us personalize your learning experience
                </div>
              </div>
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
