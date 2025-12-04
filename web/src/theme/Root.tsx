/**
 * Root Theme Component
 *
 * Wraps the entire Docusaurus app with global providers:
 * - SessionProvider for custom authentication state management
 * - ChatProvider for chat state management
 * - ChatWidget for AI assistance
 * - PersonalizeBtn for hardware-specific personalization
 * - TextSelectionHandler for "Add to Chat" on text selection
 */
import React, { ReactNode } from "react";
import { SessionProvider } from "../contexts/SessionContext";
import { ChatProvider } from "../context/ChatContext";
import ChatWidget from "../components/ChatWidget";
import PersonalizeBtn from "../components/PersonalizeBtn";
import TextSelectionHandler from "../components/TextSelectionHandler";

interface RootProps {
  children: ReactNode;
}

export default function Root({ children }: RootProps): JSX.Element {
  return (
    <SessionProvider>
      <ChatProvider>
        {children}
        <ChatWidget />
        <PersonalizeBtn />
        <TextSelectionHandler />
      </ChatProvider>
    </SessionProvider>
  );
}
