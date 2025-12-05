/**
 * Logout Page
 *
 * Handles user logout and redirects to home page
 */
import React, { useEffect, useState } from "react";
import Layout from "@theme/Layout";
import { authClient } from "../lib/auth-client";
import { useHistory } from "@docusaurus/router";
import useBaseUrl from "@docusaurus/useBaseUrl";

export default function Logout(): JSX.Element {
  const history = useHistory();
  const baseUrl = useBaseUrl('/');
  const [status, setStatus] = useState<"loading" | "success" | "error">("loading");
  const [error, setError] = useState("");

  useEffect(() => {
    const performLogout = async () => {
      try {
        await authClient.signOut({
          fetchOptions: {
            onSuccess: () => {
              setStatus("success");
              // Redirect to home after 1 second
              setTimeout(() => {
                history.push(baseUrl);
              }, 1000);
            },
            onError: (ctx) => {
              setStatus("error");
              setError(ctx.error.message || "Logout failed");
            },
          },
        });
      } catch (err: any) {
        setStatus("error");
        setError(err.message || "Logout failed");
      }
    };

    performLogout();
  }, [history]);

  return (
    <Layout title="Logout" description="Logout from Physical AI Textbook">
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
            textAlign: "center",
          }}
        >
          {status === "loading" && (
            <>
              <h1>Logging out...</h1>
              <div
                style={{
                  marginTop: "20px",
                  fontSize: "48px",
                }}
              >
                🔄
              </div>
            </>
          )}

          {status === "success" && (
            <>
              <h1>Logged out successfully!</h1>
              <div
                style={{
                  marginTop: "20px",
                  fontSize: "48px",
                }}
              >
                ✅
              </div>
              <p style={{ marginTop: "20px", color: "var(--ifm-color-emphasis-600)" }}>
                Redirecting to home page...
              </p>
            </>
          )}

          {status === "error" && (
            <>
              <h1>Logout failed</h1>
              <div
                style={{
                  marginTop: "20px",
                  fontSize: "48px",
                }}
              >
                ❌
              </div>
              <p
                style={{
                  marginTop: "20px",
                  padding: "12px",
                  background: "#ff4444",
                  color: "white",
                  borderRadius: "6px",
                }}
              >
                {error}
              </p>
              <button
                onClick={() => history.push(baseUrl)}
                style={{
                  marginTop: "20px",
                  padding: "10px 20px",
                  background: "var(--ifm-color-primary)",
                  color: "white",
                  border: "none",
                  borderRadius: "6px",
                  cursor: "pointer",
                }}
              >
                Go to Home
              </button>
            </>
          )}
        </div>
      </div>
    </Layout>
  );
}
