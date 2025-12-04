/**
 * Language Switcher Component
 *
 * Toggles between English (/docs/en/) and Urdu (/docs/ur/) routes
 * - Detects current language from URL path
 * - Switches to corresponding page in other language
 * - Shows "اردو" button when on English pages
 * - Shows "English" button when on Urdu pages
 */
import React from "react";
import { useLocation, useHistory } from "@docusaurus/router";

export default function LangSwitcher(): JSX.Element | null {
  const location = useLocation();
  const history = useHistory();
  const currentPath = location.pathname;

  // Determine current language and target language
  const isEnglish = currentPath.includes("/en/");
  const isUrdu = currentPath.includes("/ur/");

  // Hide if not in docs section
  if (!isEnglish && !isUrdu) {
    return null;
  }

  const handleSwitch = () => {
    let targetPath: string;

    if (isEnglish) {
      // Switch from English to Urdu
      targetPath = currentPath.replace("/en/", "/ur/");
    } else {
      // Switch from Urdu to English
      targetPath = currentPath.replace("/ur/", "/en/");
    }

    history.push(targetPath);
  };

  return (
    <button
      onClick={handleSwitch}
      style={{
        padding: "6px 12px",
        background: "var(--ifm-color-primary)",
        color: "white",
        border: "none",
        borderRadius: "6px",
        cursor: "pointer",
        fontSize: "14px",
        fontWeight: "600",
        marginLeft: "10px",
      }}
      title={isEnglish ? "Switch to Urdu" : "Switch to English"}
    >
      {isEnglish ? "اردو" : "English"}
    </button>
  );
}
