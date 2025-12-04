/**
 * SearchNavbarItem Component
 *
 * Features:
 * - Navbar button that triggers search modal
 * - Styled like a search input
 * - Shows keyboard shortcut hint (Cmd+K)
 * - Global Cmd+K/Ctrl+K listener
 */
import React, { useState, useEffect } from "react";
import SearchModal from "./SearchModal";

export default function SearchNavbarItem(): JSX.Element {
  const [isSearchOpen, setIsSearchOpen] = useState(false);

  // Global Cmd+K / Ctrl+K listener
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Cmd+K (Mac) or Ctrl+K (Windows/Linux)
      if ((e.metaKey || e.ctrlKey) && e.key === "k") {
        e.preventDefault();
        setIsSearchOpen(true);
      }
    };

    document.addEventListener("keydown", handleKeyDown);
    return () => document.removeEventListener("keydown", handleKeyDown);
  }, []);

  const handleAskAI = (query: string) => {
    // TODO: Integrate with ChatWidget to pre-fill query
    console.log("Ask AI:", query);
    // This would trigger the ChatWidget to open with the query
    // For now, just log it
  };

  return (
    <>
      <button
        onClick={() => setIsSearchOpen(true)}
        className="navbar__item"
        style={{
          display: "flex",
          alignItems: "center",
          gap: "8px",
          padding: "6px 12px",
          border: "1px solid var(--ifm-color-emphasis-300)",
          borderRadius: "6px",
          background: "var(--ifm-background-surface-color)",
          color: "var(--ifm-color-emphasis-600)",
          fontSize: "14px",
          cursor: "pointer",
          transition: "all 0.2s",
          minWidth: "200px",
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.borderColor = "var(--ifm-color-primary)";
          e.currentTarget.style.background = "var(--ifm-color-primary-lightest)";
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.borderColor = "var(--ifm-color-emphasis-300)";
          e.currentTarget.style.background = "var(--ifm-background-surface-color)";
        }}
      >
        <span>🔍</span>
        <span style={{ flex: 1, textAlign: "left" }}>Search...</span>
        <kbd
          style={{
            padding: "2px 6px",
            background: "var(--ifm-color-emphasis-200)",
            borderRadius: "4px",
            fontSize: "11px",
            fontWeight: "600",
          }}
        >
          {navigator.platform.toUpperCase().indexOf("MAC") >= 0 ? "⌘" : "Ctrl"}K
        </kbd>
      </button>

      <SearchModal
        isOpen={isSearchOpen}
        onClose={() => setIsSearchOpen(false)}
        onAskAI={handleAskAI}
      />
    </>
  );
}
