/**
 * DocItem Layout Override
 *
 * Adds "highlight-to-chat" functionality:
 * - Detects text selection with onMouseUp
 * - Shows tooltip "💬 Add to Chat"
 * - Adds selected text to chat context when clicked
 */
import React, { useState, useEffect } from "react";
import Layout from "@theme-original/DocItem/Layout";
import type LayoutType from "@theme/DocItem/Layout";
import type { WrapperProps } from "@docusaurus/types";
import { useChatContext } from "../../../context/ChatContext";

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): JSX.Element {
  const { addContext, setIsChatOpen } = useChatContext();
  const [tooltip, setTooltip] = useState<{
    text: string;
    x: number;
    y: number;
  } | null>(null);

  const handleMouseUp = () => {
    const selection = window.getSelection();
    const selectedText = selection?.toString().trim();

    if (selectedText && selectedText.length > 10) {
      const range = selection?.getRangeAt(0);
      const rect = range?.getBoundingClientRect();

      if (rect) {
        setTooltip({
          text: selectedText,
          x: rect.left + rect.width / 2,
          y: rect.top - 10,
        });
      }
    } else {
      setTooltip(null);
    }
  };

  const handleAddToChat = () => {
    if (tooltip) {
      addContext(tooltip.text);
      setIsChatOpen(true);
      setTooltip(null);
    }
  };

  useEffect(() => {
    const handleClickOutside = (e: MouseEvent) => {
      const target = e.target as HTMLElement;
      if (tooltip && !target.closest(".highlight-tooltip")) {
        setTooltip(null);
      }
    };

    document.addEventListener("click", handleClickOutside);
    return () => document.removeEventListener("click", handleClickOutside);
  }, [tooltip]);

  return (
    <>
      <div onMouseUp={handleMouseUp}>
        <Layout {...props} />
      </div>

      {tooltip && (
        <div
          className="highlight-tooltip"
          style={{
            left: `${tooltip.x}px`,
            top: `${tooltip.y}px`,
            transform: "translate(-50%, -100%)",
          }}
          onClick={handleAddToChat}
        >
          💬 Add to Chat
        </div>
      )}
    </>
  );
}
