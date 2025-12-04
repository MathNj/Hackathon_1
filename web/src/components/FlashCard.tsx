/**
 * FlashCard Component
 * Interactive flashcards for learning reinforcement
 */
import React, { useState } from "react";

interface FlashCardData {
  id: number;
  question: string;
  answer: string;
  category?: string;
}

interface FlashCardProps {
  cards: FlashCardData[];
}

export default function FlashCard({ cards }: FlashCardProps): JSX.Element {
  const [currentIndex, setCurrentIndex] = useState(0);
  const [isFlipped, setIsFlipped] = useState(false);
  const [showAll, setShowAll] = useState(false);

  if (!cards || cards.length === 0) {
    return <div>No flashcards available</div>;
  }

  const currentCard = cards[currentIndex];

  const handleNext = () => {
    setIsFlipped(false);
    setCurrentIndex((prev) => (prev + 1) % cards.length);
  };

  const handlePrevious = () => {
    setIsFlipped(false);
    setCurrentIndex((prev) => (prev - 1 + cards.length) % cards.length);
  };

  const handleFlip = () => {
    setIsFlipped(!isFlipped);
  };

  if (showAll) {
    return (
      <div
        style={{
          background: "var(--ifm-background-surface-color)",
          padding: "30px",
          borderRadius: "12px",
          border: "1px solid var(--ifm-color-emphasis-300)",
          marginTop: "40px",
        }}
      >
        <div
          style={{
            display: "flex",
            justifyContent: "space-between",
            alignItems: "center",
            marginBottom: "20px",
          }}
        >
          <h2 style={{ margin: 0 }}>
            <span style={{ fontSize: "24px", marginRight: "10px" }}>🎴</span>
            All Flashcards ({cards.length})
          </h2>
          <button
            onClick={() => setShowAll(false)}
            style={{
              padding: "8px 16px",
              background: "#6366f1",
              color: "white",
              border: "none",
              borderRadius: "6px",
              cursor: "pointer",
              fontSize: "14px",
              fontWeight: "600",
            }}
          >
            Card View
          </button>
        </div>

        <div
          style={{
            display: "grid",
            gridTemplateColumns: "repeat(auto-fill, minmax(300px, 1fr))",
            gap: "20px",
          }}
        >
          {cards.map((card, idx) => (
            <div
              key={card.id}
              style={{
                background: "var(--ifm-color-emphasis-100)",
                padding: "20px",
                borderRadius: "8px",
                border: "2px solid var(--ifm-color-emphasis-200)",
              }}
            >
              <div
                style={{
                  fontSize: "12px",
                  fontWeight: "700",
                  color: "#6366f1",
                  marginBottom: "10px",
                  textTransform: "uppercase",
                }}
              >
                Card {idx + 1}
                {card.category && ` • ${card.category}`}
              </div>
              <div style={{ marginBottom: "15px" }}>
                <div
                  style={{
                    fontSize: "13px",
                    fontWeight: "600",
                    color: "var(--ifm-color-emphasis-700)",
                    marginBottom: "5px",
                  }}
                >
                  Q:
                </div>
                <div style={{ fontSize: "15px", fontWeight: "600" }}>
                  {card.question}
                </div>
              </div>
              <div>
                <div
                  style={{
                    fontSize: "13px",
                    fontWeight: "600",
                    color: "#10b981",
                    marginBottom: "5px",
                  }}
                >
                  A:
                </div>
                <div
                  style={{
                    fontSize: "14px",
                    color: "var(--ifm-color-emphasis-800)",
                  }}
                >
                  {card.answer}
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    );
  }

  return (
    <div
      style={{
        background: "var(--ifm-background-surface-color)",
        padding: "30px",
        borderRadius: "12px",
        border: "1px solid var(--ifm-color-emphasis-300)",
        marginTop: "40px",
      }}
    >
      <div
        style={{
          display: "flex",
          justifyContent: "space-between",
          alignItems: "center",
          marginBottom: "20px",
        }}
      >
        <h2 style={{ margin: 0 }}>
          <span style={{ fontSize: "24px", marginRight: "10px" }}>🎴</span>
          Review Flashcards
        </h2>
        <button
          onClick={() => setShowAll(true)}
          style={{
            padding: "8px 16px",
            background: "#e9ecef",
            color: "#495057",
            border: "none",
            borderRadius: "6px",
            cursor: "pointer",
            fontSize: "14px",
            fontWeight: "600",
          }}
        >
          View All
        </button>
      </div>

      <div
        style={{
          textAlign: "center",
          marginBottom: "20px",
          fontSize: "14px",
          color: "var(--ifm-color-emphasis-700)",
          fontWeight: "600",
        }}
      >
        Card {currentIndex + 1} of {cards.length}
        {currentCard.category && (
          <span
            style={{
              marginLeft: "10px",
              padding: "4px 12px",
              background: "#c7d2fe",
              color: "#3730a3",
              borderRadius: "12px",
              fontSize: "12px",
            }}
          >
            {currentCard.category}
          </span>
        )}
      </div>

      <div
        onClick={handleFlip}
        style={{
          minHeight: "250px",
          background: isFlipped
            ? "linear-gradient(135deg, #10b981 0%, #059669 100%)"
            : "linear-gradient(135deg, #6366f1 0%, #4f46e5 100%)",
          borderRadius: "12px",
          padding: "40px",
          display: "flex",
          alignItems: "center",
          justifyContent: "center",
          cursor: "pointer",
          transition: "all 0.3s ease",
          boxShadow: "0 4px 12px rgba(0, 0, 0, 0.15)",
          color: "white",
          userSelect: "none",
        }}
      >
        <div style={{ textAlign: "center", width: "100%" }}>
          <div
            style={{
              fontSize: "14px",
              fontWeight: "700",
              textTransform: "uppercase",
              letterSpacing: "1px",
              marginBottom: "20px",
              opacity: 0.9,
            }}
          >
            {isFlipped ? "Answer" : "Question"}
          </div>
          <div style={{ fontSize: "20px", fontWeight: "600", lineHeight: 1.5 }}>
            {isFlipped ? currentCard.answer : currentCard.question}
          </div>
          <div
            style={{
              marginTop: "30px",
              fontSize: "13px",
              opacity: 0.8,
            }}
          >
            {isFlipped ? "Click to see question" : "Click to reveal answer"}
          </div>
        </div>
      </div>

      <div
        style={{
          display: "flex",
          justifyContent: "space-between",
          alignItems: "center",
          marginTop: "20px",
        }}
      >
        <button
          onClick={handlePrevious}
          disabled={cards.length === 1}
          style={{
            padding: "12px 24px",
            background: "#e9ecef",
            color: "#495057",
            border: "none",
            borderRadius: "8px",
            cursor: cards.length === 1 ? "not-allowed" : "pointer",
            fontSize: "14px",
            fontWeight: "600",
            opacity: cards.length === 1 ? 0.5 : 1,
          }}
        >
          ← Previous
        </button>

        <div
          style={{
            display: "flex",
            gap: "8px",
          }}
        >
          {cards.map((_, idx) => (
            <div
              key={idx}
              onClick={() => {
                setCurrentIndex(idx);
                setIsFlipped(false);
              }}
              style={{
                width: "10px",
                height: "10px",
                borderRadius: "50%",
                background:
                  idx === currentIndex
                    ? "#6366f1"
                    : "var(--ifm-color-emphasis-300)",
                cursor: "pointer",
                transition: "all 0.2s ease",
              }}
            />
          ))}
        </div>

        <button
          onClick={handleNext}
          disabled={cards.length === 1}
          style={{
            padding: "12px 24px",
            background: "#6366f1",
            color: "white",
            border: "none",
            borderRadius: "8px",
            cursor: cards.length === 1 ? "not-allowed" : "pointer",
            fontSize: "14px",
            fontWeight: "600",
            opacity: cards.length === 1 ? 0.5 : 1,
          }}
        >
          Next →
        </button>
      </div>

      <div
        style={{
          marginTop: "20px",
          padding: "15px",
          background: "var(--ifm-color-emphasis-100)",
          borderRadius: "8px",
          fontSize: "13px",
          color: "var(--ifm-color-emphasis-700)",
        }}
      >
        <strong>💡 Tip:</strong> Try to answer each question before flipping the
        card. Repetition strengthens memory!
      </div>
    </div>
  );
}
