/**
 * Chat Widget Component
 *
 * Features:
 * - Toggle button (floating bottom-right)
 * - Context chips display above input
 * - Remove context with X button
 * - Send message to RAG backend at http://localhost:8000/chat
 * - Display chat history
 */
import React, { useState, useRef, useEffect } from "react";
import { useChatContext } from "../context/ChatContext";

interface Message {
  role: "user" | "assistant";
  content: string;
}

export default function ChatWidget(): JSX.Element {
  const {
    isChatOpen,
    setIsChatOpen,
    activeContexts,
    removeContext,
    clearContexts,
  } = useChatContext();

  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState("");
  const [loading, setLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSend = async () => {
    if (!input.trim() && activeContexts.length === 0) return;

    const userMessage = input.trim();
    const contextText = activeContexts.join("\n\n---\n\n");
    const fullMessage = contextText
      ? `Context:\n${contextText}\n\nQuestion: ${userMessage}`
      : userMessage;

    // Add user message to chat
    const newUserMessage: Message = { role: "user", content: userMessage };
    setMessages((prev) => [...prev, newUserMessage]);
    setInput("");
    setLoading(true);

    try {
      const response = await fetch("http://localhost:8000/chat", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          message: fullMessage,
          use_rag: true,
        }),
      });

      if (!response.ok) {
        throw new Error("Failed to get response");
      }

      const data = await response.json();
      const assistantMessage: Message = {
        role: "assistant",
        content: data.response || "No response received",
      };

      setMessages((prev) => [...prev, assistantMessage]);
      clearContexts(); // Clear contexts after successful send
    } catch (error) {
      console.error("Chat error:", error);
      const errorMessage: Message = {
        role: "assistant",
        content: "Sorry, I couldn't process your request. Please try again.",
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  if (!isChatOpen) {
    return (
      <button
        className="chat-toggle"
        onClick={() => setIsChatOpen(true)}
        title="Open chat"
      >
        💬
      </button>
    );
  }

  return (
    <div className="chat-widget">
      {/* Header */}
      <div
        style={{
          padding: "16px",
          borderBottom: "1px solid var(--ifm-color-emphasis-200)",
          display: "flex",
          justifyContent: "space-between",
          alignItems: "center",
          background: "var(--ifm-color-primary)",
          color: "white",
          borderRadius: "12px 12px 0 0",
        }}
      >
        <h3 style={{ margin: 0, fontSize: "18px" }}>AI Assistant</h3>
        <button
          onClick={() => setIsChatOpen(false)}
          style={{
            background: "none",
            border: "none",
            color: "white",
            fontSize: "24px",
            cursor: "pointer",
            padding: "0",
            lineHeight: "1",
          }}
        >
          ×
        </button>
      </div>

      {/* Messages */}
      <div
        style={{
          flex: 1,
          overflowY: "auto",
          padding: "16px",
          display: "flex",
          flexDirection: "column",
          gap: "12px",
          maxHeight: "400px",
        }}
      >
        {messages.length === 0 && (
          <div
            style={{
              textAlign: "center",
              color: "var(--ifm-color-emphasis-600)",
              padding: "20px",
            }}
          >
            Ask me anything about the textbook content!
          </div>
        )}

        {messages.map((msg, idx) => (
          <div
            key={idx}
            style={{
              alignSelf: msg.role === "user" ? "flex-end" : "flex-start",
              maxWidth: "80%",
              padding: "10px 14px",
              borderRadius: "12px",
              background:
                msg.role === "user"
                  ? "var(--ifm-color-primary)"
                  : "var(--ifm-color-emphasis-200)",
              color:
                msg.role === "user"
                  ? "white"
                  : "var(--ifm-font-color-base)",
              fontSize: "14px",
              lineHeight: "1.5",
            }}
          >
            {msg.content}
          </div>
        ))}

        {loading && (
          <div
            style={{
              alignSelf: "flex-start",
              padding: "10px 14px",
              borderRadius: "12px",
              background: "var(--ifm-color-emphasis-200)",
              fontSize: "14px",
            }}
          >
            Thinking...
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {/* Context Chips */}
      {activeContexts.length > 0 && (
        <div
          style={{
            padding: "12px 16px",
            borderTop: "1px solid var(--ifm-color-emphasis-200)",
            background: "var(--ifm-color-emphasis-100)",
          }}
        >
          <div
            style={{
              fontSize: "12px",
              fontWeight: "600",
              color: "var(--ifm-color-emphasis-700)",
              marginBottom: "8px",
              display: "flex",
              alignItems: "center",
              gap: "6px",
            }}
          >
            <span>📎</span>
            <span>Added Context ({activeContexts.length})</span>
          </div>
          <div
            style={{
              display: "flex",
              flexDirection: "column",
              gap: "8px",
            }}
          >
            {activeContexts.map((context, idx) => (
              <div
                key={idx}
                style={{
                  padding: "8px 10px",
                  background: "var(--ifm-background-surface-color)",
                  borderRadius: "6px",
                  border: "1px solid var(--ifm-color-emphasis-300)",
                  fontSize: "13px",
                  position: "relative",
                  paddingRight: "32px",
                }}
              >
                <div
                  style={{
                    maxHeight: "60px",
                    overflow: "hidden",
                    textOverflow: "ellipsis",
                    lineHeight: "1.4",
                    color: "var(--ifm-color-emphasis-800)",
                  }}
                >
                  {context}
                </div>
                <button
                  onClick={() => removeContext(idx)}
                  title="Remove context"
                  style={{
                    position: "absolute",
                    top: "4px",
                    right: "4px",
                    background: "var(--ifm-color-danger)",
                    color: "white",
                    border: "none",
                    borderRadius: "50%",
                    width: "20px",
                    height: "20px",
                    cursor: "pointer",
                    fontSize: "14px",
                    lineHeight: "1",
                    display: "flex",
                    alignItems: "center",
                    justifyContent: "center",
                  }}
                >
                  ×
                </button>
              </div>
            ))}
          </div>
        </div>
      )}

      {/* Input */}
      <div
        style={{
          padding: "16px",
          borderTop: "1px solid var(--ifm-color-emphasis-200)",
          display: "flex",
          gap: "8px",
        }}
      >
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Ask a question..."
          disabled={loading}
          style={{
            flex: 1,
            padding: "10px",
            borderRadius: "6px",
            border: "1px solid var(--ifm-color-emphasis-300)",
            fontSize: "14px",
            background: "var(--ifm-background-surface-color)",
          }}
        />
        <button
          onClick={handleSend}
          disabled={loading || (!input.trim() && activeContexts.length === 0)}
          style={{
            padding: "10px 20px",
            background: "var(--ifm-color-primary)",
            color: "white",
            border: "none",
            borderRadius: "6px",
            cursor: loading ? "not-allowed" : "pointer",
            fontSize: "14px",
            fontWeight: "600",
            opacity: loading ? 0.7 : 1,
          }}
        >
          Send
        </button>
      </div>
    </div>
  );
}
