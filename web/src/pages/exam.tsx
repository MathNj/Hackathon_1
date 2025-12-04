/**
 * Final Exam Page
 *
 * Comprehensive exam covering all modules
 * Accessible from Module 5 Capstone
 */
import React from "react";
import Layout from "@theme/Layout";
import ExamComponent from "../components/ExamComponent";

export default function ExamPage(): JSX.Element {
  return (
    <Layout
      title="Final Exam"
      description="Test your knowledge of Physical AI and Humanoid Robotics"
    >
      <ExamComponent />
    </Layout>
  );
}
