/**
 * Homepage - Physical AI & Humanoid Robotics Textbook
 *
 * Modern landing page with:
 * - Hero section with gradient background
 * - Feature highlights
 * - Call-to-action sections
 * - Course modules overview
 */
import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";
import { useHistory } from "@docusaurus/router";

export default function Home(): JSX.Element {
  const history = useHistory();

  return (
    <Layout
      title="Home"
      description="Learn to build intelligent physical systems with AI and robotics"
    >
      {/* Hero Section */}
      <section
        style={{
          background: "linear-gradient(135deg, #667eea 0%, #764ba2 100%)",
          padding: "80px 20px",
          color: "white",
          textAlign: "center",
        }}
      >
        <div style={{ maxWidth: "900px", margin: "0 auto" }}>
          <h1
            style={{
              fontSize: "clamp(2.5rem, 5vw, 3.5rem)",
              fontWeight: "800",
              marginBottom: "24px",
              lineHeight: "1.2",
            }}
          >
            Physical AI & Humanoid Robotics
          </h1>
          <p
            style={{
              fontSize: "clamp(1.1rem, 2vw, 1.4rem)",
              marginBottom: "40px",
              opacity: 0.95,
              lineHeight: "1.6",
            }}
          >
            Master the art of building intelligent physical systems. From ROS 2
            fundamentals to Vision-Language-Action models, learn everything you
            need to create the next generation of humanoid robots.
          </p>
          <div
            style={{
              display: "flex",
              gap: "16px",
              justifyContent: "center",
              flexWrap: "wrap",
            }}
          >
            <Link
              to="/docs/en/module-0-setup/intro"
              style={{
                padding: "14px 32px",
                background: "white",
                color: "#667eea",
                borderRadius: "8px",
                textDecoration: "none",
                fontWeight: "700",
                fontSize: "16px",
                transition: "all 0.3s ease",
                display: "inline-block",
              }}
              onMouseEnter={(e) => {
                e.currentTarget.style.transform = "translateY(-2px)";
                e.currentTarget.style.boxShadow =
                  "0 8px 20px rgba(0, 0, 0, 0.2)";
              }}
              onMouseLeave={(e) => {
                e.currentTarget.style.transform = "translateY(0)";
                e.currentTarget.style.boxShadow = "none";
              }}
            >
              Start Learning →
            </Link>
            <Link
              to="/login"
              style={{
                padding: "14px 32px",
                background: "rgba(255, 255, 255, 0.2)",
                color: "white",
                border: "2px solid white",
                borderRadius: "8px",
                textDecoration: "none",
                fontWeight: "700",
                fontSize: "16px",
                transition: "all 0.3s ease",
                display: "inline-block",
                backdropFilter: "blur(10px)",
              }}
              onMouseEnter={(e) => {
                e.currentTarget.style.background = "white";
                e.currentTarget.style.color = "#667eea";
              }}
              onMouseLeave={(e) => {
                e.currentTarget.style.background = "rgba(255, 255, 255, 0.2)";
                e.currentTarget.style.color = "white";
              }}
            >
              Sign Up Free
            </Link>
          </div>
        </div>
      </section>

      {/* Features Section */}
      <section
        style={{
          padding: "80px 20px",
          background: "var(--ifm-background-surface-color)",
        }}
      >
        <div style={{ maxWidth: "1200px", margin: "0 auto" }}>
          <h2
            style={{
              textAlign: "center",
              fontSize: "2.5rem",
              fontWeight: "700",
              marginBottom: "60px",
            }}
          >
            Why This Textbook?
          </h2>
          <div
            style={{
              display: "grid",
              gridTemplateColumns: "repeat(auto-fit, minmax(300px, 1fr))",
              gap: "32px",
            }}
          >
            {[
              {
                icon: "🤖",
                title: "Hands-On Projects",
                description:
                  "Build real humanoid robots from scratch with step-by-step tutorials and code examples.",
              },
              {
                icon: "🧠",
                title: "AI-Powered Learning",
                description:
                  "Chat with our AI assistant that understands the entire textbook and answers your questions instantly.",
              },
              {
                icon: "⚙️",
                title: "Hardware Personalized",
                description:
                  "Content adapts to your hardware - from RTX 4090 to Jetson Orin to Google Colab.",
              },
              {
                icon: "🌍",
                title: "Multi-Language Support",
                description:
                  "Learn in English or Urdu with seamless language switching throughout the course.",
              },
              {
                icon: "📚",
                title: "Industry-Standard Tools",
                description:
                  "Master ROS 2, Gazebo, Nav2, OpenVLA, and other cutting-edge robotics frameworks.",
              },
              {
                icon: "🎯",
                title: "Beginner to Advanced",
                description:
                  "Start from basics and progress to advanced topics like VLA models and SLAM.",
              },
            ].map((feature, idx) => (
              <div
                key={idx}
                style={{
                  padding: "32px",
                  background: "var(--ifm-card-background-color)",
                  borderRadius: "12px",
                  border: "1px solid var(--ifm-color-emphasis-200)",
                  transition: "all 0.3s ease",
                  cursor: "default",
                }}
                onMouseEnter={(e) => {
                  e.currentTarget.style.transform = "translateY(-4px)";
                  e.currentTarget.style.boxShadow =
                    "0 12px 24px rgba(0, 0, 0, 0.1)";
                  e.currentTarget.style.borderColor =
                    "var(--ifm-color-primary)";
                }}
                onMouseLeave={(e) => {
                  e.currentTarget.style.transform = "translateY(0)";
                  e.currentTarget.style.boxShadow = "none";
                  e.currentTarget.style.borderColor =
                    "var(--ifm-color-emphasis-200)";
                }}
              >
                <div style={{ fontSize: "3rem", marginBottom: "16px" }}>
                  {feature.icon}
                </div>
                <h3
                  style={{
                    fontSize: "1.4rem",
                    fontWeight: "700",
                    marginBottom: "12px",
                  }}
                >
                  {feature.title}
                </h3>
                <p
                  style={{
                    fontSize: "1rem",
                    lineHeight: "1.6",
                    color: "var(--ifm-color-emphasis-700)",
                    margin: 0,
                  }}
                >
                  {feature.description}
                </p>
              </div>
            ))}
          </div>
        </div>
      </section>

      {/* Course Modules */}
      <section
        style={{
          padding: "80px 20px",
          background: "var(--ifm-color-emphasis-100)",
        }}
      >
        <div style={{ maxWidth: "1200px", margin: "0 auto" }}>
          <h2
            style={{
              textAlign: "center",
              fontSize: "2.5rem",
              fontWeight: "700",
              marginBottom: "20px",
            }}
          >
            Course Modules
          </h2>
          <p
            style={{
              textAlign: "center",
              fontSize: "1.1rem",
              color: "var(--ifm-color-emphasis-700)",
              marginBottom: "60px",
            }}
          >
            A comprehensive curriculum covering all aspects of physical AI
          </p>
          <div
            style={{
              display: "grid",
              gridTemplateColumns: "repeat(auto-fit, minmax(280px, 1fr))",
              gap: "24px",
            }}
          >
            {[
              {
                module: "Module 0",
                title: "Setup & Environment",
                topics: "ROS 2, Docker, Development Tools",
                path: "/docs/en/module-0-setup/intro",
              },
              {
                module: "Module 1",
                title: "The Nervous System",
                topics: "Sensors, Actuators, Hardware Integration",
                path: "/docs/en/module-1-nervous-system/intro",
              },
              {
                module: "Module 2",
                title: "Digital Twin",
                topics: "Gazebo, URDF, Simulation",
                path: "/docs/en/module-2-digital-twin/intro",
              },
              {
                module: "Module 3",
                title: "The Robot Brain",
                topics: "Navigation, SLAM, Path Planning",
                path: "/docs/en/module-3-robot-brain/intro",
              },
              {
                module: "Module 4",
                title: "The Mind",
                topics: "Vision-Language-Action Models, AI Integration",
                path: "/docs/en/module-4-the-mind/intro",
              },
              {
                module: "Module 5",
                title: "Capstone Project",
                topics: "Build Your Own Humanoid Robot",
                path: "/docs/en/module-5-capstone/intro",
              },
            ].map((module, idx) => (
              <Link
                key={idx}
                to={module.path}
                style={{
                  padding: "28px",
                  background: "white",
                  borderRadius: "10px",
                  border: "2px solid var(--ifm-color-emphasis-200)",
                  transition: "all 0.3s ease",
                  cursor: "pointer",
                  textDecoration: "none",
                  color: "inherit",
                  display: "block",
                }}
                onClick={(e) => {
                  // Let Link handle navigation, don't use history.push
                  e.stopPropagation();
                }}

                onMouseEnter={(e) => {
                  e.currentTarget.style.borderColor =
                    "var(--ifm-color-primary)";
                  e.currentTarget.style.transform = "translateY(-2px)";
                }}
                onMouseLeave={(e) => {
                  e.currentTarget.style.borderColor =
                    "var(--ifm-color-emphasis-200)";
                  e.currentTarget.style.transform = "translateY(0)";
                }}
              >
                <div
                  style={{
                    fontSize: "14px",
                    fontWeight: "700",
                    color: "var(--ifm-color-primary)",
                    marginBottom: "8px",
                  }}
                >
                  {module.module}
                </div>
                <h3
                  style={{
                    fontSize: "1.3rem",
                    fontWeight: "700",
                    marginBottom: "8px",
                  }}
                >
                  {module.title}
                </h3>
                <p
                  style={{
                    fontSize: "0.95rem",
                    color: "var(--ifm-color-emphasis-600)",
                    margin: 0,
                  }}
                >
                  {module.topics}
                </p>
              </Link>
            ))}
          </div>
        </div>
      </section>

      {/* CTA Section */}
      <section
        style={{
          padding: "80px 20px",
          background: "linear-gradient(135deg, #667eea 0%, #764ba2 100%)",
          color: "white",
          textAlign: "center",
        }}
      >
        <div style={{ maxWidth: "800px", margin: "0 auto" }}>
          <h2
            style={{
              fontSize: "2.5rem",
              fontWeight: "700",
              marginBottom: "24px",
            }}
          >
            Ready to Build the Future?
          </h2>
          <p
            style={{
              fontSize: "1.2rem",
              marginBottom: "40px",
              opacity: 0.95,
              lineHeight: "1.6",
            }}
          >
            Join thousands of students learning to create intelligent physical
            systems. Start your journey today with our comprehensive,
            hardware-personalized curriculum.
          </p>
          <div
            style={{
              display: "flex",
              gap: "16px",
              justifyContent: "center",
              flexWrap: "wrap",
            }}
          >
            <Link
              to="/docs/en/module-0-setup/intro"
              style={{
                padding: "14px 32px",
                background: "white",
                color: "#667eea",
                borderRadius: "8px",
                textDecoration: "none",
                fontWeight: "700",
                fontSize: "16px",
                transition: "all 0.3s ease",
                display: "inline-block",
              }}
              onMouseEnter={(e) => {
                e.currentTarget.style.transform = "translateY(-2px)";
                e.currentTarget.style.boxShadow =
                  "0 8px 20px rgba(0, 0, 0, 0.2)";
              }}
              onMouseLeave={(e) => {
                e.currentTarget.style.transform = "translateY(0)";
                e.currentTarget.style.boxShadow = "none";
              }}
            >
              Get Started Now →
            </Link>
          </div>
        </div>
      </section>

      {/* Footer Stats */}
      <section
        style={{
          padding: "60px 20px",
          background: "var(--ifm-background-surface-color)",
        }}
      >
        <div
          style={{
            maxWidth: "1000px",
            margin: "0 auto",
            display: "grid",
            gridTemplateColumns: "repeat(auto-fit, minmax(200px, 1fr))",
            gap: "40px",
            textAlign: "center",
          }}
        >
          {[
            { number: "6", label: "Course Modules" },
            { number: "50+", label: "Hands-On Projects" },
            { number: "100%", label: "Open Source" },
            { number: "2", label: "Languages" },
          ].map((stat, idx) => (
            <div key={idx}>
              <div
                style={{
                  fontSize: "3rem",
                  fontWeight: "800",
                  color: "var(--ifm-color-primary)",
                  marginBottom: "8px",
                }}
              >
                {stat.number}
              </div>
              <div
                style={{
                  fontSize: "1.1rem",
                  color: "var(--ifm-color-emphasis-700)",
                  fontWeight: "600",
                }}
              >
                {stat.label}
              </div>
            </div>
          ))}
        </div>
      </section>
    </Layout>
  );
}
