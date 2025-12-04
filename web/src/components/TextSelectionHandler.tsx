/**
 * Text Selection Handler Component
 *
 * Detects text selection on the page and shows an "Add to Chat" button
 * When clicked, adds selected text as context to the chat widget
 */
import React, { useEffect, useState } from "react";
import { useChatContext } from "../context/ChatContext";

interface Position {
  top: number;
  left: number;
}

export default function TextSelectionHandler(): JSX.Element {
  const { addContext, setIsChatOpen } = useChatContext();
  const [selectedText, setSelectedText] = useState("");
  const [position, setPosition] = useState<Position | null>(null);

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 0) {
        // Get selection position
        const range = selection?.getRangeAt(0);
        const rect = range?.getBoundingClientRect();

        if (rect) {
          setSelectedText(text);
          setPosition({
            top: rect.bottom + window.scrollY + 5,
            left: rect.left + window.scrollX + rect.width / 2,
          });
        }
      } else {
        setSelectedText("");
        setPosition(null);
      }
    };

    // Listen to mouseup for text selection
    document.addEventListener("mouseup", handleSelection);
    document.addEventListener("touchend", handleSelection);

    // Listen to selection change
    document.addEventListener("selectionchange", handleSelection);

    return () => {
      document.removeEventListener("mouseup", handleSelection);
      document.removeEventListener("touchend", handleSelection);
      document.removeEventListener("selectionchange", handleSelection);
    };
  }, []);

  const handleAddToChat = () => {
    if (selectedText) {
      addContext(selectedText);
      setIsChatOpen(true);
      setSelectedText("");
      setPosition(null);

      // Clear selection
      window.getSelection()?.removeAllRanges();
    }
  };

  if (!selectedText || !position) {
    return null;
  }

  return (
    <div
      style={{
        position: "absolute",
        top: `${position.top}px`,
        left: `${position.left}px`,
        transform: "translateX(-50%)",
        zIndex: 9999,
        animation: "fadeIn 0.2s ease-in",
      }}
    >
      <button
        onClick={handleAddToChat}
        style={{
          padding: "8px 16px",
          background: "var(--ifm-color-primary)",
          color: "white",
          border: "none",
          borderRadius: "20px",
          fontSize: "14px",
          fontWeight: "600",
          cursor: "pointer",
          boxShadow: "0 4px 12px rgba(0, 0, 0, 0.15)",
          display: "flex",
          alignItems: "center",
          gap: "6px",
          whiteSpace: "nowrap",
          transition: "all 0.2s ease",
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.transform = "scale(1.05)";
          e.currentTarget.style.boxShadow = "0 6px 16px rgba(0, 0, 0, 0.2)";
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.transform = "scale(1)";
          e.currentTarget.style.boxShadow = "0 4px 12px rgba(0, 0, 0, 0.15)";
        }}
      >
        <span style={{ fontSize: "16px" }}>💬</span>
        Add to Chat
      </button>

      <style>
        {`
          @keyframes fadeIn {
            from {
              opacity: 0;
              transform: translateX(-50%) translateY(-5px);
            }
            to {
              opacity: 1;
              transform: translateX(-50%) translateY(0);
            }
          }
        `}
      </style>
    </div>
  );
}
