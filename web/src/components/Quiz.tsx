/**
 * Quiz Component
 *
 * Features:
 * - Multiple choice questions
 * - Instant feedback after submission
 * - Score display
 * - Retake functionality
 */
import React, { useState } from "react";

export interface QuizQuestion {
  id: number;
  question: string;
  options: string[];
  correctAnswer: number;
  explanation?: string;
}

interface QuizProps {
  questions: QuizQuestion[];
  title?: string;
  passingScore?: number;
}

export default function Quiz({ questions, title = "Quiz", passingScore = 70 }: QuizProps): JSX.Element {
  const [answers, setAnswers] = useState<Map<number, number>>(new Map());
  const [submitted, setSubmitted] = useState(false);
  const [showResults, setShowResults] = useState(false);

  const handleAnswerSelect = (questionId: number, optionIndex: number) => {
    if (!submitted) {
      setAnswers(prev => new Map(prev).set(questionId, optionIndex));
    }
  };

  const calculateScore = () => {
    let correct = 0;
    questions.forEach(q => {
      if (answers.get(q.id) === q.correctAnswer) {
        correct++;
      }
    });
    return Math.round((correct / questions.length) * 100);
  };

  const handleSubmit = () => {
    if (answers.size === questions.length) {
      setSubmitted(true);
      setShowResults(true);
    } else {
      alert("Please answer all questions before submitting!");
    }
  };

  const handleRetake = () => {
    setAnswers(new Map());
    setSubmitted(false);
    setShowResults(false);
  };

  const score = submitted ? calculateScore() : 0;
  const passed = score >= passingScore;

  if (showResults) {
    return (
      <div className="quiz-results" style={{
        margin: "40px 0",
        padding: "30px",
        backgroundColor: passed ? "#ecfdf5" : "#fef2f2",
        borderRadius: "12px",
        border: `2px solid ${passed ? "#10b981" : "#ef4444"}`
      }}>
        <h2 style={{ fontSize: "28px", fontWeight: "700", marginBottom: "20px", color: passed ? "#065f46" : "#991b1b" }}>
          {passed ? "Congratulations! You passed!" : "Keep Learning!"}
        </h2>

        <div style={{ fontSize: "48px", fontWeight: "800", color: passed ? "#10b981" : "#ef4444", marginBottom: "20px" }}>
          {score}%
        </div>

        <p style={{ fontSize: "16px", marginBottom: "30px", color: "#374151" }}>
          You answered {questions.filter(q => answers.get(q.id) === q.correctAnswer).length} out of {questions.length} questions correctly.
          {!passed && ` You need ${passingScore}% to pass.`}
        </p>

        <div style={{ marginBottom: "30px" }}>
          <h3 style={{ fontSize: "20px", fontWeight: "600", marginBottom: "15px", color: "#1f2937" }}>
            Detailed Results:
          </h3>
          {questions.map((q, index) => {
            const userAnswer = answers.get(q.id);
            const isCorrect = userAnswer === q.correctAnswer;

            return (
              <div key={q.id} style={{
                padding: "15px",
                backgroundColor: "white",
                borderRadius: "8px",
                marginBottom: "10px",
                border: `2px solid ${isCorrect ? "#10b981" : "#ef4444"}`
              }}>
                <div style={{ fontWeight: "600", marginBottom: "10px", color: "#1f2937" }}>
                  {index + 1}. {q.question}
                </div>
                <div style={{ fontSize: "14px", color: "#6b7280", marginBottom: "5px" }}>
                  Your answer: <span style={{ color: isCorrect ? "#10b981" : "#ef4444", fontWeight: "600" }}>
                    {q.options[userAnswer!]}
                  </span>
                </div>
                {!isCorrect && (
                  <div style={{ fontSize: "14px", color: "#10b981", fontWeight: "600" }}>
                    Correct answer: {q.options[q.correctAnswer]}
                  </div>
                )}
                {q.explanation && (
                  <div style={{ fontSize: "14px", color: "#6b7280", marginTop: "8px", fontStyle: "italic" }}>
                    {q.explanation}
                  </div>
                )}
              </div>
            );
          })}
        </div>

        <button
          onClick={handleRetake}
          style={{
            padding: "12px 24px",
            borderRadius: "8px",
            border: "none",
            backgroundColor: "#4f46e5",
            color: "white",
            fontSize: "16px",
            fontWeight: "600",
            cursor: "pointer"
          }}
        >
          Retake Quiz
        </button>
      </div>
    );
  }

  return (
    <div className="quiz-container" style={{ margin: "40px 0" }}>
      <h2 style={{ fontSize: "28px", fontWeight: "700", marginBottom: "10px" }}>{title}</h2>
      <p style={{ fontSize: "16px", color: "#6b7280", marginBottom: "30px" }}>
        Answer all {questions.length} questions and submit to see your results.
      </p>

      {questions.map((question, index) => (
        <div key={question.id} style={{
          padding: "20px",
          backgroundColor: "#f9fafb",
          borderRadius: "12px",
          marginBottom: "20px",
          border: "2px solid #e5e7eb"
        }}>
          <h3 style={{ fontSize: "18px", fontWeight: "600", marginBottom: "15px", color: "#1f2937" }}>
            {index + 1}. {question.question}
          </h3>

          <div style={{ display: "flex", flexDirection: "column", gap: "10px" }}>
            {question.options.map((option, optionIndex) => {
              const isSelected = answers.get(question.id) === optionIndex;

              return (
                <label
                  key={optionIndex}
                  style={{
                    display: "flex",
                    alignItems: "center",
                    padding: "12px 16px",
                    backgroundColor: isSelected ? "#eef2ff" : "white",
                    borderRadius: "8px",
                    border: `2px solid ${isSelected ? "#4f46e5" : "#d1d5db"}`,
                    cursor: "pointer",
                    transition: "all 0.2s"
                  }}
                  onMouseEnter={(e) => {
                    if (!isSelected) {
                      e.currentTarget.style.backgroundColor = "#f3f4f6";
                    }
                  }}
                  onMouseLeave={(e) => {
                    if (!isSelected) {
                      e.currentTarget.style.backgroundColor = "white";
                    }
                  }}
                >
                  <input
                    type="radio"
                    name={`question-${question.id}`}
                    value={optionIndex}
                    checked={isSelected}
                    onChange={() => handleAnswerSelect(question.id, optionIndex)}
                    style={{ marginRight: "12px", cursor: "pointer" }}
                  />
                  <span style={{ fontSize: "16px", color: "#374151" }}>{option}</span>
                </label>
              );
            })}
          </div>
        </div>
      ))}

      <div style={{
        display: "flex",
        justifyContent: "space-between",
        alignItems: "center",
        marginTop: "30px"
      }}>
        <span style={{ fontSize: "14px", color: "#6b7280" }}>
          {answers.size} / {questions.length} questions answered
        </span>

        <button
          onClick={handleSubmit}
          disabled={answers.size !== questions.length}
          style={{
            padding: "12px 32px",
            borderRadius: "8px",
            border: "none",
            backgroundColor: answers.size === questions.length ? "#4f46e5" : "#d1d5db",
            color: "white",
            fontSize: "16px",
            fontWeight: "600",
            cursor: answers.size === questions.length ? "pointer" : "not-allowed"
          }}
        >
          Submit Quiz
        </button>
      </div>
    </div>
  );
}
