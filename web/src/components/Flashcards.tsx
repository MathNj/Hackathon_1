/**
 * Flashcards Component
 *
 * Features:
 * - Flip cards to reveal answers
 * - Navigate between cards
 * - Two view modes: Card View and View All
 * - Category tags
 */
import React, { useState } from "react";

export interface Flashcard {
  id: number;
  question: string;
  answer: string;
  category?: string;
}

interface FlashcardsProps {
  cards: Flashcard[];
  title?: string;
}

export default function Flashcards({ cards, title = "Flashcards" }: FlashcardsProps): JSX.Element {
  const [currentIndex, setCurrentIndex] = useState(0);
  const [flippedCards, setFlippedCards] = useState<Set<number>>(new Set());
  const [viewMode, setViewMode] = useState<"card" | "all">("card");

  const handleFlip = (id: number) => {
    setFlippedCards(prev => {
      const newSet = new Set(prev);
      if (newSet.has(id)) {
        newSet.delete(id);
      } else {
        newSet.add(id);
      }
      return newSet;
    });
  };

  const handleNext = () => {
    if (currentIndex < cards.length - 1) {
      setCurrentIndex(currentIndex + 1);
    }
  };

  const handlePrevious = () => {
    if (currentIndex > 0) {
      setCurrentIndex(currentIndex - 1);
    }
  };

  const renderCard = (card: Flashcard, index: number, isCurrentCard: boolean = false) => {
    const isFlipped = flippedCards.has(card.id);

    return (
      <div
        key={card.id}
        className={`flashcard ${isCurrentCard ? 'flashcard--current' : ''}`}
        style={{
          perspective: "1000px",
          marginBottom: viewMode === "all" ? "20px" : "0"
        }}
      >
        <div
          className={`flashcard__inner ${isFlipped ? 'flashcard__inner--flipped' : ''}`}
          onClick={() => handleFlip(card.id)}
          style={{
            position: "relative",
            width: "100%",
            minHeight: "250px",
            textAlign: "center",
            transition: "transform 0.6s",
            transformStyle: "preserve-3d",
            cursor: "pointer",
            transform: isFlipped ? "rotateY(180deg)" : "rotateY(0deg)"
          }}
        >
          {/* Front of card */}
          <div
            className="flashcard__front"
            style={{
              position: "absolute",
              width: "100%",
              height: "100%",
              backfaceVisibility: "hidden",
              backgroundColor: "#4f46e5",
              color: "white",
              borderRadius: "12px",
              padding: "40px 30px",
              display: "flex",
              flexDirection: "column",
              justifyContent: "center",
              alignItems: "center",
              boxShadow: "0 4px 6px rgba(0, 0, 0, 0.1)"
            }}
          >
            {card.category && (
              <span style={{
                position: "absolute",
                top: "15px",
                left: "15px",
                fontSize: "12px",
                fontWeight: "600",
                backgroundColor: "rgba(255, 255, 255, 0.2)",
                padding: "4px 12px",
                borderRadius: "12px"
              }}>
                {card.category}
              </span>
            )}
            <h3 style={{ fontSize: "20px", fontWeight: "600", margin: "0", lineHeight: "1.5" }}>
              {card.question}
            </h3>
            <p style={{ marginTop: "20px", fontSize: "14px", opacity: "0.9" }}>
              Click to flip
            </p>
          </div>

          {/* Back of card */}
          <div
            className="flashcard__back"
            style={{
              position: "absolute",
              width: "100%",
              height: "100%",
              backfaceVisibility: "hidden",
              backgroundColor: "#10b981",
              color: "white",
              borderRadius: "12px",
              padding: "40px 30px",
              display: "flex",
              flexDirection: "column",
              justifyContent: "center",
              alignItems: "center",
              transform: "rotateY(180deg)",
              boxShadow: "0 4px 6px rgba(0, 0, 0, 0.1)"
            }}
          >
            <p style={{ fontSize: "18px", lineHeight: "1.6", margin: "0" }}>
              {card.answer}
            </p>
            <p style={{ marginTop: "20px", fontSize: "14px", opacity: "0.9" }}>
              Click to flip back
            </p>
          </div>
        </div>

        {viewMode === "all" && (
          <div style={{ textAlign: "center", marginTop: "10px", fontSize: "14px", color: "#6b7280" }}>
            Card {index + 1} of {cards.length}
          </div>
        )}
      </div>
    );
  };

  return (
    <div className="flashcards-container" style={{ margin: "40px 0" }}>
      <div style={{
        display: "flex",
        justifyContent: "space-between",
        alignItems: "center",
        marginBottom: "20px"
      }}>
        <h2 style={{ fontSize: "24px", fontWeight: "700", margin: "0" }}>{title}</h2>
        <div style={{ display: "flex", gap: "10px" }}>
          <button
            onClick={() => setViewMode("card")}
            style={{
              padding: "8px 16px",
              borderRadius: "6px",
              border: "1px solid #d1d5db",
              backgroundColor: viewMode === "card" ? "#4f46e5" : "white",
              color: viewMode === "card" ? "white" : "#374151",
              cursor: "pointer",
              fontSize: "14px",
              fontWeight: "500"
            }}
          >
            Card View
          </button>
          <button
            onClick={() => setViewMode("all")}
            style={{
              padding: "8px 16px",
              borderRadius: "6px",
              border: "1px solid #d1d5db",
              backgroundColor: viewMode === "all" ? "#4f46e5" : "white",
              color: viewMode === "all" ? "white" : "#374151",
              cursor: "pointer",
              fontSize: "14px",
              fontWeight: "500"
            }}
          >
            View All
          </button>
        </div>
      </div>

      {viewMode === "card" ? (
        <>
          {renderCard(cards[currentIndex], currentIndex, true)}

          <div style={{
            display: "flex",
            justifyContent: "space-between",
            alignItems: "center",
            marginTop: "20px"
          }}>
            <button
              onClick={handlePrevious}
              disabled={currentIndex === 0}
              style={{
                padding: "10px 20px",
                borderRadius: "6px",
                border: "1px solid #d1d5db",
                backgroundColor: currentIndex === 0 ? "#f3f4f6" : "white",
                color: currentIndex === 0 ? "#9ca3af" : "#374151",
                cursor: currentIndex === 0 ? "not-allowed" : "pointer",
                fontSize: "14px",
                fontWeight: "500"
              }}
            >
              Previous
            </button>

            <span style={{ fontSize: "14px", color: "#6b7280" }}>
              {currentIndex + 1} / {cards.length}
            </span>

            <button
              onClick={handleNext}
              disabled={currentIndex === cards.length - 1}
              style={{
                padding: "10px 20px",
                borderRadius: "6px",
                border: "1px solid #d1d5db",
                backgroundColor: currentIndex === cards.length - 1 ? "#f3f4f6" : "white",
                color: currentIndex === cards.length - 1 ? "#9ca3af" : "#374151",
                cursor: currentIndex === cards.length - 1 ? "not-allowed" : "pointer",
                fontSize: "14px",
                fontWeight: "500"
              }}
            >
              Next
            </button>
          </div>
        </>
      ) : (
        <div style={{ display: "flex", flexDirection: "column", gap: "20px" }}>
          {cards.map((card, index) => renderCard(card, index))}
        </div>
      )}
    </div>
  );
}
