/**
 * Exam Component with MCQs
 *
 * Features:
 * - Multiple choice questions covering all modules
 * - Real-time answer selection
 * - Score calculation
 * - Pass/fail indication
 * - Detailed results with correct answers
 * - Retake functionality
 * - Save results to database (future)
 */
import React, { useState } from "react";
import { authClient } from "../lib/auth-client";

interface Question {
  id: number;
  question: string;
  options: string[];
  correctAnswer: number;
  module: string;
  difficulty: "easy" | "medium" | "hard";
}

const EXAM_QUESTIONS: Question[] = [
  // Module 0: Setup & Environment
  {
    id: 1,
    question: "What is the primary purpose of Docker in robotics development?",
    options: [
      "To create robot hardware designs",
      "To provide isolated, reproducible development environments",
      "To write ROS 2 code",
      "To simulate robot physics"
    ],
    correctAnswer: 1,
    module: "Module 0: Setup",
    difficulty: "easy"
  },
  {
    id: 2,
    question: "Which ROS 2 distribution is recommended for production use as of 2024?",
    options: [
      "ROS 2 Foxy",
      "ROS 2 Galactic",
      "ROS 2 Humble",
      "ROS 2 Rolling"
    ],
    correctAnswer: 2,
    module: "Module 0: Setup",
    difficulty: "easy"
  },

  // Module 1: The Nervous System
  {
    id: 3,
    question: "What type of sensor provides depth information for 3D perception?",
    options: [
      "IMU (Inertial Measurement Unit)",
      "LiDAR or RGB-D camera",
      "Encoder",
      "Force/Torque sensor"
    ],
    correctAnswer: 1,
    module: "Module 1: Nervous System",
    difficulty: "easy"
  },
  {
    id: 4,
    question: "In ROS 2, what is the main difference between a topic and a service?",
    options: [
      "Topics are faster than services",
      "Topics are publish-subscribe (continuous), services are request-response (one-time)",
      "Services can only be used with Python",
      "Topics require more CPU resources"
    ],
    correctAnswer: 1,
    module: "Module 1: Nervous System",
    difficulty: "medium"
  },
  {
    id: 5,
    question: "What is the purpose of a PID controller in robotics?",
    options: [
      "To process images from cameras",
      "To control actuators by minimizing error between desired and actual values",
      "To detect obstacles",
      "To plan navigation paths"
    ],
    correctAnswer: 1,
    module: "Module 1: Nervous System",
    difficulty: "medium"
  },

  // Module 2: Digital Twin
  {
    id: 6,
    question: "What does URDF stand for in robotics?",
    options: [
      "Universal Robot Data Format",
      "Unified Robotics Description File",
      "Universal Robot Description Format",
      "Unified Robot Development Framework"
    ],
    correctAnswer: 2,
    module: "Module 2: Digital Twin",
    difficulty: "easy"
  },
  {
    id: 7,
    question: "Which physics engine is commonly used with Gazebo for robot simulation?",
    options: [
      "Unity Physics",
      "Unreal Physics",
      "ODE, Bullet, or Simbody",
      "Box2D"
    ],
    correctAnswer: 2,
    module: "Module 2: Digital Twin",
    difficulty: "medium"
  },
  {
    id: 8,
    question: "What is the main advantage of testing in simulation before deploying to real hardware?",
    options: [
      "Simulations are always 100% accurate",
      "It's faster and safer to test without risk of hardware damage",
      "Simulations don't require any computing resources",
      "Real robots can't be programmed"
    ],
    correctAnswer: 1,
    module: "Module 2: Digital Twin",
    difficulty: "easy"
  },

  // Module 3: The Robot Brain
  {
    id: 9,
    question: "What does SLAM stand for in robotics?",
    options: [
      "Systematic Location and Mapping",
      "Simultaneous Localization and Mapping",
      "Sequential Learning and Motion",
      "Sensor-based Location Analysis Method"
    ],
    correctAnswer: 1,
    module: "Module 3: Robot Brain",
    difficulty: "easy"
  },
  {
    id: 10,
    question: "Which algorithm is commonly used for global path planning?",
    options: [
      "PID Controller",
      "Kalman Filter",
      "A* (A-star) or Dijkstra's algorithm",
      "Convolutional Neural Network"
    ],
    correctAnswer: 2,
    module: "Module 3: Robot Brain",
    difficulty: "medium"
  },
  {
    id: 11,
    question: "What is the purpose of the costmap in Nav2?",
    options: [
      "To calculate the cost of robot parts",
      "To represent obstacles and space for path planning",
      "To measure battery consumption",
      "To track financial expenses"
    ],
    correctAnswer: 1,
    module: "Module 3: Robot Brain",
    difficulty: "medium"
  },
  {
    id: 12,
    question: "What is the typical latency requirement for Nav2 replanning in real-time navigation?",
    options: [
      "Less than 100ms",
      "Around 1 second",
      "5-10 seconds",
      "It doesn't matter"
    ],
    correctAnswer: 0,
    module: "Module 3: Robot Brain",
    difficulty: "hard"
  },

  // Module 4: The Mind (VLA)
  {
    id: 13,
    question: "What does VLA stand for in the context of robotics AI?",
    options: [
      "Virtual Learning Algorithm",
      "Vision-Language-Action",
      "Variable Location Analysis",
      "Visual Linear Adaptation"
    ],
    correctAnswer: 1,
    module: "Module 4: The Mind",
    difficulty: "easy"
  },
  {
    id: 14,
    question: "What is the main advantage of Vision-Language-Action models over traditional robot control?",
    options: [
      "They consume less power",
      "They can understand natural language commands and visual context",
      "They don't require any training",
      "They work without sensors"
    ],
    correctAnswer: 1,
    module: "Module 4: The Mind",
    difficulty: "medium"
  },
  {
    id: 15,
    question: "Which type of model architecture is commonly used in modern VLA systems?",
    options: [
      "Decision Trees",
      "Linear Regression",
      "Transformer-based models",
      "Simple Neural Networks"
    ],
    correctAnswer: 2,
    module: "Module 4: The Mind",
    difficulty: "hard"
  },

  // Module 5: Integration & Best Practices
  {
    id: 16,
    question: "What is the recommended approach for integrating VLA models with ROS 2 navigation?",
    options: [
      "Replace Nav2 completely with VLA",
      "Use VLA for high-level decisions and Nav2 for low-level control",
      "Never use them together",
      "VLA and ROS 2 are incompatible"
    ],
    correctAnswer: 1,
    module: "Module 5: Capstone",
    difficulty: "medium"
  },
  {
    id: 17,
    question: "What is a key consideration when deploying AI models on edge devices like Jetson Orin?",
    options: [
      "They have unlimited computing power",
      "Power efficiency and model optimization (quantization, pruning)",
      "They don't support neural networks",
      "They only work with cloud computing"
    ],
    correctAnswer: 1,
    module: "Module 5: Capstone",
    difficulty: "hard"
  },
  {
    id: 18,
    question: "What is the purpose of a safety layer in production robotics systems?",
    options: [
      "To make robots move faster",
      "To prevent unsafe actions and collisions",
      "To reduce code complexity",
      "To eliminate the need for testing"
    ],
    correctAnswer: 1,
    module: "Module 5: Capstone",
    difficulty: "medium"
  },
  {
    id: 19,
    question: "Which testing strategy is most important before deploying a robot in a real environment?",
    options: [
      "Only test in simulation",
      "Simulation testing followed by controlled real-world tests",
      "Skip testing and deploy immediately",
      "Only test with real hardware"
    ],
    correctAnswer: 1,
    module: "Module 5: Capstone",
    difficulty: "easy"
  },
  {
    id: 20,
    question: "What is the recommended minimum passing score for this exam?",
    options: [
      "50%",
      "60%",
      "70%",
      "80%"
    ],
    correctAnswer: 2,
    module: "Module 5: Capstone",
    difficulty: "easy"
  }
];

export default function ExamComponent(): JSX.Element {
  const { data: session } = authClient.useSession();
  const [started, setStarted] = useState(false);
  const [currentQuestion, setCurrentQuestion] = useState(0);
  const [answers, setAnswers] = useState<{ [key: number]: number }>({});
  const [showResults, setShowResults] = useState(false);
  const [score, setScore] = useState(0);

  const handleStart = () => {
    setStarted(true);
    setCurrentQuestion(0);
    setAnswers({});
    setShowResults(false);
    setScore(0);
  };

  const handleAnswer = (questionId: number, answerIndex: number) => {
    setAnswers({ ...answers, [questionId]: answerIndex });
  };

  const handleNext = () => {
    if (currentQuestion < EXAM_QUESTIONS.length - 1) {
      setCurrentQuestion(currentQuestion + 1);
    }
  };

  const handlePrevious = () => {
    if (currentQuestion > 0) {
      setCurrentQuestion(currentQuestion - 1);
    }
  };

  const handleSubmit = () => {
    // Calculate score
    let correctCount = 0;
    EXAM_QUESTIONS.forEach((q) => {
      if (answers[q.id] === q.correctAnswer) {
        correctCount++;
      }
    });
    setScore(correctCount);
    setShowResults(true);

    // TODO: Save results to database
    if (session?.user?.id) {
      console.log("Saving exam results for user:", session.user.id);
      console.log("Score:", correctCount, "/", EXAM_QUESTIONS.length);
    }
  };

  const allAnswered = Object.keys(answers).length === EXAM_QUESTIONS.length;
  const percentage = (score / EXAM_QUESTIONS.length) * 100;
  const passed = percentage >= 70;

  if (!started) {
    return (
      <div
        style={{
          maxWidth: "800px",
          margin: "40px auto",
          padding: "40px",
          background: "var(--ifm-card-background-color)",
          borderRadius: "12px",
          border: "2px solid var(--ifm-color-primary)",
          boxShadow: "0 4px 12px rgba(0, 0, 0, 0.1)",
        }}
      >
        <h2 style={{ textAlign: "center", color: "var(--ifm-color-primary)", marginBottom: "24px" }}>
          🎓 Final Exam: Physical AI & Humanoid Robotics
        </h2>

        <div style={{ marginBottom: "32px", lineHeight: "1.6" }}>
          <h3>Exam Information:</h3>
          <ul>
            <li><strong>Total Questions:</strong> {EXAM_QUESTIONS.length} multiple choice</li>
            <li><strong>Passing Score:</strong> 70% (14+ correct answers)</li>
            <li><strong>Time Limit:</strong> None (take your time)</li>
            <li><strong>Coverage:</strong> All modules (0-5)</li>
            <li><strong>Retakes:</strong> Unlimited</li>
          </ul>
        </div>

        <div
          style={{
            background: "var(--ifm-color-emphasis-100)",
            padding: "20px",
            borderRadius: "8px",
            marginBottom: "32px",
          }}
        >
          <h4 style={{ marginTop: 0 }}>📚 Topics Covered:</h4>
          <ul style={{ marginBottom: 0 }}>
            <li>Module 0: ROS 2 Setup & Docker</li>
            <li>Module 1: Sensors, Actuators, Communication</li>
            <li>Module 2: URDF, Gazebo Simulation</li>
            <li>Module 3: SLAM, Navigation, Path Planning</li>
            <li>Module 4: Vision-Language-Action Models</li>
            <li>Module 5: Integration & Best Practices</li>
          </ul>
        </div>

        <div style={{ textAlign: "center" }}>
          <button
            onClick={handleStart}
            style={{
              padding: "16px 48px",
              background: "var(--ifm-color-primary)",
              color: "white",
              border: "none",
              borderRadius: "8px",
              fontSize: "18px",
              fontWeight: "700",
              cursor: "pointer",
              boxShadow: "0 4px 12px rgba(0, 0, 0, 0.15)",
              transition: "all 0.2s ease",
            }}
            onMouseEnter={(e) => {
              e.currentTarget.style.transform = "translateY(-2px)";
              e.currentTarget.style.boxShadow = "0 6px 16px rgba(0, 0, 0, 0.2)";
            }}
            onMouseLeave={(e) => {
              e.currentTarget.style.transform = "translateY(0)";
              e.currentTarget.style.boxShadow = "0 4px 12px rgba(0, 0, 0, 0.15)";
            }}
          >
            Start Exam
          </button>
        </div>
      </div>
    );
  }

  if (showResults) {
    return (
      <div
        style={{
          maxWidth: "900px",
          margin: "40px auto",
          padding: "40px",
          background: "var(--ifm-card-background-color)",
          borderRadius: "12px",
          boxShadow: "0 4px 12px rgba(0, 0, 0, 0.1)",
        }}
      >
        <div style={{ textAlign: "center", marginBottom: "40px" }}>
          <h2 style={{ color: passed ? "#28a745" : "#dc3545", fontSize: "2.5rem", marginBottom: "16px" }}>
            {passed ? "🎉 Congratulations!" : "📚 Keep Learning!"}
          </h2>
          <div style={{ fontSize: "3rem", fontWeight: "800", marginBottom: "16px" }}>
            {score} / {EXAM_QUESTIONS.length}
          </div>
          <div style={{ fontSize: "1.5rem", color: "var(--ifm-color-emphasis-700)" }}>
            {percentage.toFixed(0)}% {passed ? "- PASSED ✅" : "- NOT PASSED ❌"}
          </div>
        </div>

        <div
          style={{
            background: passed ? "#d4edda" : "#f8d7da",
            border: `1px solid ${passed ? "#c3e6cb" : "#f5c6cb"}`,
            borderRadius: "8px",
            padding: "20px",
            marginBottom: "32px",
          }}
        >
          <p style={{ margin: 0, fontSize: "1.1rem" }}>
            {passed
              ? "You've demonstrated a strong understanding of Physical AI and Humanoid Robotics concepts. You're ready to build your capstone project!"
              : "Don't worry! Review the materials and try again. Focus on the questions you missed below."}
          </p>
        </div>

        <h3 style={{ marginBottom: "24px" }}>Detailed Results:</h3>

        {EXAM_QUESTIONS.map((q, idx) => {
          const userAnswer = answers[q.id];
          const isCorrect = userAnswer === q.correctAnswer;

          return (
            <div
              key={q.id}
              style={{
                marginBottom: "24px",
                padding: "20px",
                background: isCorrect
                  ? "rgba(40, 167, 69, 0.1)"
                  : "rgba(220, 53, 69, 0.1)",
                borderRadius: "8px",
                border: `2px solid ${isCorrect ? "#28a745" : "#dc3545"}`,
              }}
            >
              <div style={{ display: "flex", justifyContent: "space-between", alignItems: "start", marginBottom: "12px" }}>
                <div style={{ flex: 1 }}>
                  <div style={{ fontSize: "12px", color: "var(--ifm-color-emphasis-600)", marginBottom: "4px" }}>
                    {q.module} • {q.difficulty.toUpperCase()}
                  </div>
                  <div style={{ fontWeight: "600", marginBottom: "12px" }}>
                    {idx + 1}. {q.question}
                  </div>
                </div>
                <div
                  style={{
                    fontSize: "24px",
                    marginLeft: "16px",
                  }}
                >
                  {isCorrect ? "✅" : "❌"}
                </div>
              </div>

              <div style={{ fontSize: "14px" }}>
                <div style={{ marginBottom: "8px" }}>
                  <strong>Your answer:</strong>{" "}
                  <span style={{ color: isCorrect ? "#28a745" : "#dc3545" }}>
                    {q.options[userAnswer]}
                  </span>
                </div>
                {!isCorrect && (
                  <div>
                    <strong>Correct answer:</strong>{" "}
                    <span style={{ color: "#28a745" }}>
                      {q.options[q.correctAnswer]}
                    </span>
                  </div>
                )}
              </div>
            </div>
          );
        })}

        <div style={{ textAlign: "center", marginTop: "40px" }}>
          <button
            onClick={handleStart}
            style={{
              padding: "14px 32px",
              background: "var(--ifm-color-primary)",
              color: "white",
              border: "none",
              borderRadius: "8px",
              fontSize: "16px",
              fontWeight: "700",
              cursor: "pointer",
              boxShadow: "0 4px 12px rgba(0, 0, 0, 0.15)",
            }}
          >
            Retake Exam
          </button>
        </div>
      </div>
    );
  }

  const question = EXAM_QUESTIONS[currentQuestion];
  const progress = ((currentQuestion + 1) / EXAM_QUESTIONS.length) * 100;

  return (
    <div
      style={{
        maxWidth: "800px",
        margin: "40px auto",
        padding: "40px",
        background: "var(--ifm-card-background-color)",
        borderRadius: "12px",
        boxShadow: "0 4px 12px rgba(0, 0, 0, 0.1)",
      }}
    >
      {/* Progress Bar */}
      <div style={{ marginBottom: "32px" }}>
        <div
          style={{
            display: "flex",
            justifyContent: "space-between",
            marginBottom: "8px",
            fontSize: "14px",
            color: "var(--ifm-color-emphasis-700)",
          }}
        >
          <span>
            Question {currentQuestion + 1} of {EXAM_QUESTIONS.length}
          </span>
          <span>{Object.keys(answers).length} answered</span>
        </div>
        <div
          style={{
            height: "8px",
            background: "var(--ifm-color-emphasis-200)",
            borderRadius: "4px",
            overflow: "hidden",
          }}
        >
          <div
            style={{
              height: "100%",
              width: `${progress}%`,
              background: "var(--ifm-color-primary)",
              transition: "width 0.3s ease",
            }}
          />
        </div>
      </div>

      {/* Question Info */}
      <div
        style={{
          display: "flex",
          gap: "12px",
          marginBottom: "24px",
          fontSize: "12px",
        }}
      >
        <span
          style={{
            padding: "4px 12px",
            background: "var(--ifm-color-primary-lightest)",
            color: "var(--ifm-color-primary-darkest)",
            borderRadius: "12px",
            fontWeight: "600",
          }}
        >
          {question.module}
        </span>
        <span
          style={{
            padding: "4px 12px",
            background: "var(--ifm-color-emphasis-200)",
            borderRadius: "12px",
            fontWeight: "600",
            textTransform: "uppercase",
          }}
        >
          {question.difficulty}
        </span>
      </div>

      {/* Question */}
      <h3 style={{ marginBottom: "32px", fontSize: "1.4rem", lineHeight: "1.5" }}>
        {question.question}
      </h3>

      {/* Options */}
      <div style={{ marginBottom: "32px" }}>
        {question.options.map((option, idx) => {
          const isSelected = answers[question.id] === idx;

          return (
            <div
              key={idx}
              onClick={() => handleAnswer(question.id, idx)}
              style={{
                padding: "16px 20px",
                marginBottom: "12px",
                background: isSelected
                  ? "var(--ifm-color-primary-lightest)"
                  : "var(--ifm-background-surface-color)",
                border: `2px solid ${
                  isSelected
                    ? "var(--ifm-color-primary)"
                    : "var(--ifm-color-emphasis-300)"
                }`,
                borderRadius: "8px",
                cursor: "pointer",
                transition: "all 0.2s ease",
                display: "flex",
                alignItems: "center",
                gap: "12px",
              }}
              onMouseEnter={(e) => {
                if (!isSelected) {
                  e.currentTarget.style.borderColor = "var(--ifm-color-primary)";
                  e.currentTarget.style.background = "var(--ifm-color-emphasis-100)";
                }
              }}
              onMouseLeave={(e) => {
                if (!isSelected) {
                  e.currentTarget.style.borderColor = "var(--ifm-color-emphasis-300)";
                  e.currentTarget.style.background = "var(--ifm-background-surface-color)";
                }
              }}
            >
              <div
                style={{
                  width: "24px",
                  height: "24px",
                  borderRadius: "50%",
                  border: `2px solid ${
                    isSelected
                      ? "var(--ifm-color-primary)"
                      : "var(--ifm-color-emphasis-400)"
                  }`,
                  display: "flex",
                  alignItems: "center",
                  justifyContent: "center",
                  flexShrink: 0,
                }}
              >
                {isSelected && (
                  <div
                    style={{
                      width: "12px",
                      height: "12px",
                      borderRadius: "50%",
                      background: "var(--ifm-color-primary)",
                    }}
                  />
                )}
              </div>
              <div style={{ fontSize: "15px" }}>{option}</div>
            </div>
          );
        })}
      </div>

      {/* Navigation */}
      <div
        style={{
          display: "flex",
          justifyContent: "space-between",
          alignItems: "center",
        }}
      >
        <button
          onClick={handlePrevious}
          disabled={currentQuestion === 0}
          style={{
            padding: "10px 24px",
            background: "var(--ifm-color-emphasis-200)",
            border: "none",
            borderRadius: "6px",
            fontSize: "14px",
            fontWeight: "600",
            cursor: currentQuestion === 0 ? "not-allowed" : "pointer",
            opacity: currentQuestion === 0 ? 0.5 : 1,
          }}
        >
          ← Previous
        </button>

        {currentQuestion === EXAM_QUESTIONS.length - 1 ? (
          <button
            onClick={handleSubmit}
            disabled={!allAnswered}
            style={{
              padding: "12px 32px",
              background: allAnswered
                ? "var(--ifm-color-primary)"
                : "var(--ifm-color-emphasis-300)",
              color: allAnswered ? "white" : "var(--ifm-color-emphasis-600)",
              border: "none",
              borderRadius: "6px",
              fontSize: "16px",
              fontWeight: "700",
              cursor: allAnswered ? "pointer" : "not-allowed",
              boxShadow: allAnswered ? "0 4px 12px rgba(0, 0, 0, 0.15)" : "none",
            }}
            title={
              !allAnswered
                ? `Please answer all questions (${
                    EXAM_QUESTIONS.length - Object.keys(answers).length
                  } remaining)`
                : ""
            }
          >
            Submit Exam
          </button>
        ) : (
          <button
            onClick={handleNext}
            style={{
              padding: "10px 24px",
              background: "var(--ifm-color-primary)",
              color: "white",
              border: "none",
              borderRadius: "6px",
              fontSize: "14px",
              fontWeight: "600",
              cursor: "pointer",
            }}
          >
            Next →
          </button>
        )}
      </div>

      {/* Submit warning */}
      {currentQuestion === EXAM_QUESTIONS.length - 1 && !allAnswered && (
        <div
          style={{
            marginTop: "20px",
            padding: "12px",
            background: "#fff3cd",
            border: "1px solid #ffc107",
            borderRadius: "6px",
            fontSize: "14px",
            textAlign: "center",
          }}
        >
          ⚠️ You must answer all {EXAM_QUESTIONS.length} questions before submitting. (
          {EXAM_QUESTIONS.length - Object.keys(answers).length} remaining)
        </div>
      )}
    </div>
  );
}
