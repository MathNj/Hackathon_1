/**
 * Global Chat Context
 *
 * Manages chat state including:
 * - Chat open/closed state
 * - Active context snippets (from text selections)
 * - Add/remove context methods
 */
import React, { createContext, useContext, useState, ReactNode } from "react";

interface ChatContextType {
  isChatOpen: boolean;
  setIsChatOpen: (open: boolean) => void;
  activeContexts: string[];
  addContext: (text: string) => void;
  removeContext: (index: number) => void;
  clearContexts: () => void;
}

const ChatContext = createContext<ChatContextType | undefined>(undefined);

export const ChatProvider: React.FC<{ children: ReactNode }> = ({
  children,
}) => {
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [activeContexts, setActiveContexts] = useState<string[]>([]);

  const addContext = (text: string) => {
    // Trim and limit to 200 characters
    const trimmed = text.trim().substring(0, 200);
    if (trimmed && !activeContexts.includes(trimmed)) {
      setActiveContexts([...activeContexts, trimmed]);
    }
  };

  const removeContext = (index: number) => {
    setActiveContexts(activeContexts.filter((_, i) => i !== index));
  };

  const clearContexts = () => {
    setActiveContexts([]);
  };

  return (
    <ChatContext.Provider
      value={{
        isChatOpen,
        setIsChatOpen,
        activeContexts,
        addContext,
        removeContext,
        clearContexts,
      }}
    >
      {children}
    </ChatContext.Provider>
  );
};

export const useChatContext = () => {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error("useChatContext must be used within a ChatProvider");
  }
  return context;
};
