/**
 * Root Theme Component
 *
 * Wraps the entire Docusaurus app with global providers:
 * - ChatProvider for chat state management
 * - ChatWidget for AI assistance
 * - PersonalizeBtn for hardware-specific personalization
 * - TextSelectionHandler for "Add to Chat" on text selection
 */
import React, { ReactNode } from "react";
import { ChatProvider } from "../context/ChatContext";
import ChatWidget from "../components/ChatWidget";
import PersonalizeBtn from "../components/PersonalizeBtn";
import TextSelectionHandler from "../components/TextSelectionHandler";

interface RootProps {
  children: ReactNode;
}

export default function Root({ children }: RootProps): JSX.Element {
  return (
    <ChatProvider>
      {children}
      <ChatWidget />
      <PersonalizeBtn />
      <TextSelectionHandler />
    </ChatProvider>
  );
}
