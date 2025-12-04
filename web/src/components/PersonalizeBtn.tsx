/**
 * Personalize Button Component with Side-by-Side View
 *
 * Features:
 * - Selects main article content and sends to API for personalization
 * - Two view modes: Replace (overwrites article) or Side-by-Side (modal view)
 * - ReactMarkdown rendering for proper formatting
 * - Shows 'Rewriting...' state during processing
 * - Uses user's skill level and hardware preferences from session
 * - Only visible when user is logged in
 */
import React, { useState, useEffect } from "react";
import { authClient } from "../lib/auth-client";
import ReactMarkdown from "react-markdown";
import remarkGfm from "remark-gfm";

type ViewMode = "replace" | "side-by-side";

export default function PersonalizeBtn(): JSX.Element | null {
  const { data: session } = authClient.useSession();
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [originalContent, setOriginalContent] = useState<string | null>(null);
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [viewMode, setViewMode] = useState<ViewMode>("side-by-side");
  const [showModal, setShowModal] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [background, setBackground] = useState<string>("");
  const [loadingPreferences, setLoadingPreferences] = useState(true);
  const [showTooltip, setShowTooltip] = useState(false);

  // Fetch user background from database
  useEffect(() => {
    const fetchUserBackground = async () => {
      if (!session?.user?.id) {
        setLoadingPreferences(false);
        return;
      }

      try {
        // Try to get from session first
        if (session.user.background) {
          setBackground(session.user.background);
          setLoadingPreferences(false);
          return;
        }

        // If not in session, fetch from database
        const response = await fetch(`http://localhost:3001/api/user/profile`, {
          credentials: 'include',
          headers: {
            'Authorization': `Bearer ${session.user.id}`,
          },
        });

        if (response.ok) {
          const data = await response.json();
          if (data.background) setBackground(data.background);
        }
      } catch (err) {
        console.error('Failed to fetch user background:', err);
      } finally {
        setLoadingPreferences(false);
      }
    };

    fetchUserBackground();
  }, [session]);

  // ESC key to close modal
  useEffect(() => {
    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === "Escape" && showModal) {
        setShowModal(false);
      }
    };

    if (showModal) {
      document.addEventListener("keydown", handleEscape);
      return () => document.removeEventListener("keydown", handleEscape);
    }
  }, [showModal]);

  if (!session?.user) {
    return null;
  }

  const handlePersonalize = async () => {
    // Get the main article content from the page
    const articleElement = document.querySelector("article");

    if (!articleElement) {
      setError("No article content found on this page");
      setTimeout(() => setError(null), 3000);
      return;
    }

    const text = articleElement.innerText;

    if (!text || text.length < 50) {
      setError("Article content is too short to personalize");
      setTimeout(() => setError(null), 3000);
      return;
    }

    // Store original content for side-by-side view or restoration
    if (!originalContent) {
      setOriginalContent(text);
    }

    setIsPersonalizing(true);
    setError(null);

    try {
      const response = await fetch("http://localhost:8000/personalize", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          content: text,
          background: background || "general audience",
        }),
      });

      if (!response.ok) {
        throw new Error("Failed to personalize content");
      }

      const data = await response.json();
      setPersonalizedContent(data.personalized_content);

      if (viewMode === "replace") {
        // Replace article content with ReactMarkdown rendered version
        const personalizedHtml = `
          <div style="border-left: 4px solid var(--ifm-color-primary); padding-left: 16px; margin-bottom: 20px;">
            <p style="color: var(--ifm-color-primary); font-weight: bold; margin-bottom: 8px;">
              ✨ Personalized content for you
            </p>
          </div>
          <div class="personalized-content" style="white-space: pre-wrap; line-height: 1.6;">
            ${data.personalized_content.replace(/\n/g, '<br>')}
          </div>
        `;
        articleElement.innerHTML = personalizedHtml;
      } else {
        // Show side-by-side modal
        setShowModal(true);
      }

    } catch (err: any) {
      setError(err.message || "Failed to personalize content");
      setTimeout(() => setError(null), 5000);
    } finally {
      setIsPersonalizing(false);
    }
  };

  const handleRestore = () => {
    const articleElement = document.querySelector("article");
    if (articleElement && originalContent) {
      // Restore by reloading the page (safest method)
      window.location.reload();
    }
  };

  const handleCloseModal = () => {
    setShowModal(false);
  };

  const handleApplyPersonalized = () => {
    const articleElement = document.querySelector("article");
    if (articleElement && personalizedContent) {
      const personalizedHtml = `
        <div style="border-left: 4px solid var(--ifm-color-primary); padding-left: 16px; margin-bottom: 20px;">
          <p style="color: var(--ifm-color-primary); font-weight: bold; margin-bottom: 8px;">
            ✨ Personalized for ${skillLevel} using ${hardwareBg}
          </p>
        </div>
        <div class="personalized-content" style="white-space: pre-wrap; line-height: 1.6;">
          ${personalizedContent.replace(/\n/g, '<br>')}
        </div>
      `;
      articleElement.innerHTML = personalizedHtml;
      setShowModal(false);
    }
  };

  // Check if we're on a doc page
  const isDocPage = typeof window !== 'undefined' &&
    (window.location.pathname.includes('/docs/'));

  // Don't show on non-doc pages
  if (!isDocPage) {
    return null;
  }

  return (
    <>
      {/* Control Buttons */}
      <div
        className="personalize-btn-container"
        style={{
          position: "fixed",
          top: "80px",
          right: "20px",
          display: "flex",
          flexDirection: "column",
          gap: "8px",
          zIndex: 1000,
          transition: "all 0.3s ease",
        }}
      >
        <div style={{ display: "flex", flexDirection: "column", gap: "8px" }}>
          <button
            onClick={handlePersonalize}
            disabled={isPersonalizing || loadingPreferences}
            style={{
              padding: "12px 20px",
              background: isPersonalizing ? "#6c757d" : "var(--ifm-color-primary)",
              border: "none",
              borderRadius: "10px",
              fontSize: "14px",
              fontWeight: "700",
              color: "white",
              boxShadow: "0 4px 12px rgba(0, 0, 0, 0.15)",
              cursor: (isPersonalizing || loadingPreferences) ? "not-allowed" : "pointer",
              opacity: (isPersonalizing || loadingPreferences) ? 0.7 : 1,
              transition: "all 0.2s ease",
              display: "flex",
              alignItems: "center",
              gap: "8px",
              whiteSpace: "nowrap",
            }}
            onMouseEnter={(e) => {
              if (!isPersonalizing && !loadingPreferences) {
                e.currentTarget.style.transform = "translateY(-2px)";
                e.currentTarget.style.boxShadow = "0 6px 16px rgba(0, 0, 0, 0.2)";
              }
            }}
            onMouseLeave={(e) => {
              e.currentTarget.style.transform = "translateY(0)";
              e.currentTarget.style.boxShadow = "0 4px 12px rgba(0, 0, 0, 0.15)";
            }}
            title="Personalize article based on your background"
          >
            <span style={{ fontSize: "16px" }}>✨</span>
            {loadingPreferences ? "Loading..." : isPersonalizing ? "Personalizing..." : "Personalize"}
          </button>

          {(originalContent || personalizedContent) && (
            <button
              onClick={handleRestore}
              style={{
                padding: "10px 16px",
                background: "var(--ifm-color-emphasis-200)",
                border: "1px solid var(--ifm-color-emphasis-400)",
                borderRadius: "8px",
                fontSize: "13px",
                fontWeight: "600",
                color: "var(--ifm-font-color-base)",
                cursor: "pointer",
                transition: "all 0.2s ease",
              }}
              onMouseEnter={(e) => {
                e.currentTarget.style.background = "var(--ifm-color-emphasis-300)";
              }}
              onMouseLeave={(e) => {
                e.currentTarget.style.background = "var(--ifm-color-emphasis-200)";
              }}
              title="Restore original content"
            >
              🔄 Restore Original
            </button>
          )}
        </div>

        {/* View Mode Toggle */}
        <div style={{
          display: "flex",
          background: "var(--ifm-background-surface-color)",
          borderRadius: "8px",
          padding: "4px",
          gap: "4px",
          border: "1px solid var(--ifm-color-emphasis-300)",
          boxShadow: "0 2px 4px rgba(0, 0, 0, 0.05)",
        }}>
          <button
            onClick={() => setViewMode("replace")}
            style={{
              flex: 1,
              padding: "8px 12px",
              background: viewMode === "replace" ? "var(--ifm-color-primary)" : "transparent",
              color: viewMode === "replace" ? "white" : "var(--ifm-font-color-base)",
              border: "none",
              borderRadius: "6px",
              fontSize: "12px",
              fontWeight: "600",
              cursor: "pointer",
              transition: "all 0.2s ease",
            }}
            onMouseEnter={(e) => {
              if (viewMode !== "replace") {
                e.currentTarget.style.background = "var(--ifm-color-emphasis-100)";
              }
            }}
            onMouseLeave={(e) => {
              if (viewMode !== "replace") {
                e.currentTarget.style.background = "transparent";
              }
            }}
            title="Replace article content with personalized version"
          >
            📝 Replace
          </button>
          <button
            onClick={() => setViewMode("side-by-side")}
            style={{
              flex: 1,
              padding: "8px 12px",
              background: viewMode === "side-by-side" ? "var(--ifm-color-primary)" : "transparent",
              color: viewMode === "side-by-side" ? "white" : "var(--ifm-font-color-base)",
              border: "none",
              borderRadius: "6px",
              fontSize: "12px",
              fontWeight: "600",
              cursor: "pointer",
              transition: "all 0.2s ease",
            }}
            onMouseEnter={(e) => {
              if (viewMode !== "side-by-side") {
                e.currentTarget.style.background = "var(--ifm-color-emphasis-100)";
              }
            }}
            onMouseLeave={(e) => {
              if (viewMode !== "side-by-side") {
                e.currentTarget.style.background = "transparent";
              }
            }}
            title="View original and personalized side-by-side"
          >
            👥 Compare
          </button>
        </div>

        {/* Info Tooltip */}
        <div
          style={{
            background: "var(--ifm-color-emphasis-100)",
            border: "1px solid var(--ifm-color-emphasis-300)",
            borderRadius: "8px",
            padding: "12px",
            fontSize: "12px",
            lineHeight: "1.4",
            color: "var(--ifm-color-emphasis-800)",
            maxWidth: "220px",
            boxShadow: "0 2px 4px rgba(0, 0, 0, 0.05)",
          }}
        >
          <div style={{ fontWeight: "700", marginBottom: "6px", display: "flex", alignItems: "center", gap: "4px" }}>
            <span>💡</span>
            <span>AI Personalization</span>
          </div>
          <div style={{ fontSize: "11px", opacity: 0.9 }}>
            {background ? (
              <>
                Adapting content based on your background
              </>
            ) : (
              <>
                Update your profile to enable personalized content
              </>
            )}
          </div>
        </div>
      </div>

      {/* Error Toast */}
      {error && (
        <div
          style={{
            position: "fixed",
            top: "180px",
            right: "20px",
            padding: "12px 16px",
            background: "#ff4444",
            color: "white",
            borderRadius: "8px",
            fontSize: "14px",
            boxShadow: "0 2px 8px rgba(0, 0, 0, 0.2)",
            zIndex: 1000,
            maxWidth: "300px",
          }}
        >
          {error}
        </div>
      )}

      {/* Side-by-Side Modal */}
      {showModal && originalContent && personalizedContent && (
        <div
          style={{
            position: "fixed",
            top: 0,
            left: 0,
            right: 0,
            bottom: 0,
            background: "rgba(0, 0, 0, 0.5)",
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            zIndex: 2000,
            padding: "20px",
          }}
          onClick={handleCloseModal}
        >
          <div
            style={{
              background: "var(--ifm-background-surface-color)",
              borderRadius: "12px",
              width: "95%",
              maxWidth: "1400px",
              maxHeight: "90vh",
              display: "flex",
              flexDirection: "column",
              boxShadow: "0 8px 32px rgba(0, 0, 0, 0.3)",
            }}
            onClick={(e) => e.stopPropagation()}
          >
            {/* Modal Header */}
            <div
              style={{
                padding: "20px",
                borderBottom: "1px solid var(--ifm-color-emphasis-300)",
                display: "flex",
                justifyContent: "space-between",
                alignItems: "center",
              }}
            >
              <h3 style={{ margin: 0, color: "var(--ifm-color-primary)" }}>
                Original vs Personalized ({skillLevel} • {hardwareBg})
              </h3>
              <div style={{ display: "flex", gap: "12px" }}>
                <button
                  onClick={handleApplyPersonalized}
                  style={{
                    padding: "8px 16px",
                    background: "var(--ifm-color-primary)",
                    color: "white",
                    border: "none",
                    borderRadius: "6px",
                    fontSize: "14px",
                    fontWeight: "600",
                    cursor: "pointer",
                  }}
                >
                  Apply to Page
                </button>
                <button
                  onClick={handleCloseModal}
                  style={{
                    background: "none",
                    border: "none",
                    fontSize: "28px",
                    cursor: "pointer",
                    padding: "0 8px",
                    color: "var(--ifm-font-color-base)",
                  }}
                >
                  ×
                </button>
              </div>
            </div>

            {/* Modal Content - Side by Side */}
            <div
              style={{
                display: "grid",
                gridTemplateColumns: "1fr 1px 1fr",
                gap: "0",
                flex: 1,
                overflow: "hidden",
              }}
            >
              {/* Original Content */}
              <div
                style={{
                  padding: "20px",
                  overflowY: "auto",
                }}
              >
                <h4
                  style={{
                    marginTop: 0,
                    marginBottom: "16px",
                    color: "var(--ifm-color-emphasis-700)",
                    fontSize: "14px",
                    textTransform: "uppercase",
                    fontWeight: "700",
                    letterSpacing: "0.5px",
                  }}
                >
                  Original Content
                </h4>
                <div
                  className="markdown-content"
                  style={{
                    fontSize: "15px",
                    lineHeight: "1.7",
                    color: "var(--ifm-font-color-base)",
                  }}
                >
                  <ReactMarkdown remarkPlugins={[remarkGfm]}>
                    {originalContent}
                  </ReactMarkdown>
                </div>
              </div>

              {/* Divider */}
              <div
                style={{
                  background: "var(--ifm-color-emphasis-300)",
                  width: "1px",
                }}
              />

              {/* Personalized Content */}
              <div
                style={{
                  padding: "20px",
                  overflowY: "auto",
                  background: "var(--ifm-color-primary-lightest)",
                }}
              >
                <h4
                  style={{
                    marginTop: 0,
                    marginBottom: "16px",
                    color: "var(--ifm-color-primary)",
                    fontSize: "14px",
                    textTransform: "uppercase",
                    fontWeight: "700",
                    letterSpacing: "0.5px",
                  }}
                >
                  ✨ Personalized for {skillLevel}
                </h4>
                <div
                  className="markdown-content"
                  style={{
                    fontSize: "15px",
                    lineHeight: "1.7",
                    color: "var(--ifm-font-color-base)",
                  }}
                >
                  <ReactMarkdown remarkPlugins={[remarkGfm]}>
                    {personalizedContent}
                  </ReactMarkdown>
                </div>
              </div>
            </div>

            {/* Modal Footer */}
            <div
              style={{
                padding: "16px 20px",
                borderTop: "1px solid var(--ifm-color-emphasis-300)",
                display: "flex",
                justifyContent: "space-between",
                alignItems: "center",
                fontSize: "13px",
                color: "var(--ifm-color-emphasis-700)",
              }}
            >
              <span>
                💡 Tip: Use "Apply to Page" to replace the article with personalized content
              </span>
              <span>
                Press ESC or click outside to close
              </span>
            </div>
          </div>
        </div>
      )}
    </>
  );
}
