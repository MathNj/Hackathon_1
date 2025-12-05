/**
 * SearchModal Component - Cmd+K Interactive Search
 *
 * Features:
 * - Cmd+K (Mac) / Ctrl+K (Windows) keyboard shortcut
 * - Auto-focused search input
 * - Client-side text search through all documentation pages
 * - Results with content previews and links
 * - "Ask AI" action to escalate to ChatWidget
 * - ESC to close, click outside to close
 */
import React, { useState, useEffect, useRef } from "react";
import { searchDocuments as performSearch } from "../utils/searchIndex";

interface SearchResult {
  title: string;
  content: string;
  url: string;
  score: number;
}

interface SearchModalProps {
  isOpen: boolean;
  onClose: () => void;
  onAskAI?: (query: string) => void;
}

export default function SearchModal({ isOpen, onClose, onAskAI }: SearchModalProps): JSX.Element | null {
  const [query, setQuery] = useState("");
  const [results, setResults] = useState<SearchResult[]>([]);
  const [isSearching, setIsSearching] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Auto-focus input when modal opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  // ESC key to close modal
  useEffect(() => {
    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === "Escape" && isOpen) {
        onClose();
      }
    };

    if (isOpen) {
      document.addEventListener("keydown", handleEscape);
      return () => document.removeEventListener("keydown", handleEscape);
    }
  }, [isOpen, onClose]);

  // Debounced search (client-side)
  useEffect(() => {
    if (!query.trim()) {
      setResults([]);
      return;
    }

    const timeoutId = setTimeout(() => {
      setIsSearching(true);
      setError(null);

      try {
        // Perform client-side search
        const searchResults = performSearch(query, 5);
        setResults(searchResults);
      } catch (err: any) {
        console.error("Search error:", err);
        setError(err.message || "Failed to search");
        setResults([]);
      } finally {
        setIsSearching(false);
      }
    }, 200); // 200ms debounce

    return () => clearTimeout(timeoutId);
  }, [query]);

  const handleAskAI = () => {
    if (onAskAI && query.trim()) {
      onAskAI(query);
      onClose();
      setQuery("");
      setResults([]);
    }
  };

  const handleResultClick = () => {
    onClose();
    setQuery("");
    setResults([]);
  };

  if (!isOpen) return null;

  return (
    <div
      style={{
        position: "fixed",
        top: 0,
        left: 0,
        right: 0,
        bottom: 0,
        background: "transparent",
        display: "flex",
        alignItems: "flex-start",
        justifyContent: "center",
        zIndex: 9999,
        paddingTop: "10vh",
      }}
      onClick={onClose}
    >
      <div
        style={{
          background: "var(--ifm-background-surface-color)",
          borderRadius: "12px",
          width: "90%",
          maxWidth: "600px",
          maxHeight: "70vh",
          boxShadow: "0 16px 48px rgba(0, 0, 0, 0.4)",
          display: "flex",
          flexDirection: "column",
          overflow: "hidden",
        }}
        onClick={(e) => e.stopPropagation()}
      >
        {/* Search Input */}
        <div
          style={{
            padding: "20px",
            borderBottom: "1px solid var(--ifm-color-emphasis-300)",
          }}
        >
          <div style={{ display: "flex", alignItems: "center", gap: "12px" }}>
            <span style={{ fontSize: "20px" }}>🔍</span>
            <input
              ref={inputRef}
              type="text"
              placeholder="Search documentation..."
              value={query}
              onChange={(e) => setQuery(e.target.value)}
              style={{
                flex: 1,
                border: "none",
                outline: "none",
                fontSize: "16px",
                background: "transparent",
                color: "var(--ifm-font-color-base)",
              }}
            />
            {isSearching && (
              <span style={{ fontSize: "12px", color: "var(--ifm-color-emphasis-600)" }}>
                Searching...
              </span>
            )}
          </div>
        </div>

        {/* Results */}
        <div
          style={{
            flex: 1,
            overflowY: "auto",
            padding: "8px",
          }}
        >
          {error && (
            <div
              style={{
                padding: "16px",
                color: "#ff4444",
                textAlign: "center",
              }}
            >
              {error}
            </div>
          )}

          {!query.trim() && !error && (
            <div
              style={{
                padding: "32px",
                textAlign: "center",
                color: "var(--ifm-color-emphasis-600)",
              }}
            >
              <p style={{ margin: 0, fontSize: "14px" }}>
                Start typing to search documentation...
              </p>
              <p style={{ margin: "8px 0 0 0", fontSize: "12px" }}>
                Instant client-side search
              </p>
            </div>
          )}

          {query.trim() && results.length === 0 && !isSearching && !error && (
            <div
              style={{
                padding: "32px",
                textAlign: "center",
                color: "var(--ifm-color-emphasis-600)",
              }}
            >
              <p style={{ margin: 0, fontSize: "14px" }}>
                No results found for "{query}"
              </p>
            </div>
          )}

          {results.map((result, index) => (
            <a
              key={index}
              href={result.url}
              onClick={handleResultClick}
              style={{
                display: "block",
                padding: "12px 16px",
                marginBottom: "4px",
                borderRadius: "8px",
                textDecoration: "none",
                color: "inherit",
                background: "transparent",
                transition: "background 0.2s",
              }}
              onMouseEnter={(e) => {
                e.currentTarget.style.background = "var(--ifm-color-emphasis-100)";
              }}
              onMouseLeave={(e) => {
                e.currentTarget.style.background = "transparent";
              }}
            >
              <div
                style={{
                  fontSize: "14px",
                  fontWeight: "600",
                  marginBottom: "4px",
                  color: "var(--ifm-color-primary)",
                }}
              >
                {result.title}
              </div>
              <div
                style={{
                  fontSize: "13px",
                  color: "var(--ifm-color-emphasis-700)",
                  lineHeight: "1.5",
                }}
              >
                {result.content}
              </div>
              <div
                style={{
                  fontSize: "11px",
                  color: "var(--ifm-color-emphasis-600)",
                  marginTop: "4px",
                }}
              >
                Relevance: {(result.score * 100).toFixed(0)}%
              </div>
            </a>
          ))}

          {/* Ask AI Action */}
          {query.trim() && (
            <button
              onClick={handleAskAI}
              style={{
                display: "block",
                width: "100%",
                padding: "12px 16px",
                marginTop: "8px",
                borderRadius: "8px",
                border: "2px dashed var(--ifm-color-primary)",
                background: "var(--ifm-color-primary-lightest)",
                color: "var(--ifm-color-primary)",
                fontSize: "14px",
                fontWeight: "600",
                cursor: "pointer",
                textAlign: "left",
              }}
            >
              <span style={{ marginRight: "8px" }}>✨</span>
              Ask AI Agent about "{query.length > 40 ? query.substring(0, 40) + "..." : query}"
            </button>
          )}
        </div>

        {/* Footer */}
        <div
          style={{
            padding: "12px 20px",
            borderTop: "1px solid var(--ifm-color-emphasis-300)",
            display: "flex",
            justifyContent: "space-between",
            alignItems: "center",
            fontSize: "12px",
            color: "var(--ifm-color-emphasis-600)",
          }}
        >
          <div>
            <kbd
              style={{
                padding: "2px 6px",
                background: "var(--ifm-color-emphasis-200)",
                borderRadius: "4px",
                marginRight: "4px",
              }}
            >
              ↑
            </kbd>
            <kbd
              style={{
                padding: "2px 6px",
                background: "var(--ifm-color-emphasis-200)",
                borderRadius: "4px",
              }}
            >
              ↓
            </kbd>
            <span style={{ marginLeft: "8px" }}>to navigate</span>
          </div>
          <div>
            <kbd
              style={{
                padding: "2px 6px",
                background: "var(--ifm-color-emphasis-200)",
                borderRadius: "4px",
                marginRight: "4px",
              }}
            >
              ESC
            </kbd>
            <span>to close</span>
          </div>
        </div>
      </div>
    </div>
  );
}
