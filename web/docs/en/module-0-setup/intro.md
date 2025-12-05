# Module 0: Workstation Setup

## Overview

Before diving into Physical AI and Humanoid Robotics, you need to set up your development workstation with the required hardware and software tools. This module ensures you have a functioning environment for all subsequent modules.

## Learning Objectives

By the end of this module, you will:
- ✅ Verify your hardware meets the minimum requirements
- ✅ Install Ubuntu 22.04 LTS and configure dual-boot (if needed)
- ✅ Set up NVIDIA drivers and CUDA toolkit for GPU acceleration
- ✅ Install ROS 2 Humble Hawksbill
- ✅ Configure your first ROS 2 workspace

## Hardware Requirements

### Workstation (Required)
- **GPU**: NVIDIA RTX 4070 Ti (12GB VRAM) or better
- **CPU**: AMD Ryzen 7 / Intel Core i7 (8+ cores recommended)
- **RAM**: 32GB DDR4/DDR5
- **Storage**: 500GB NVMe SSD (1TB recommended)
- **OS**: Ubuntu 22.04 LTS (native install, not WSL)

### Edge Device (Required for Module 4+)
- **Device**: NVIDIA Jetson Orin Nano (8GB) or Jetson Orin NX (16GB)
- **Power**: 15W-25W DC power supply
- **Storage**: 128GB microSD card (Class 10 or better)

### Robot Platform (Optional - Capstone Project)
- **Recommended**: Unitree Go2 or similar quadruped robot
- **Alternative**: Custom humanoid robot (if you have fabrication experience)

## Prerequisites

- Basic Linux command line experience
- Python 3.10+ knowledge
- Familiarity with Git version control

## Estimated Duration

**2-3 days** for complete setup (depending on your hardware and internet speed)

## Next Steps

1. [Hardware Verification](./hardware-verification.md)
2. [Ubuntu Installation](./ubuntu-installation.md)
3. [NVIDIA Driver Setup](./nvidia-setup.md)
4. [ROS 2 Installation](./ros2-installation.md)
5. [Workspace Configuration](./workspace-setup.md)

---

**Need Help?** Join our Discord community or check the troubleshooting section at the end of this module.

## Review Flashcards

import Flashcards from '@site/src/components/Flashcards';

<Flashcards
  title="Hardware Setup Review"
  cards={[
    {
      id: 1,
      question: "What is the minimum recommended GPU for this course?",
      answer: "NVIDIA RTX 4070 Ti with 12GB VRAM or better",
      category: "Hardware"
    },
    {
      id: 2,
      question: "Which Ubuntu version is required for ROS 2 Humble?",
      answer: "Ubuntu 22.04 LTS (native install, not WSL)",
      category: "Software"
    },
    {
      id: 3,
      question: "What is Docker's primary purpose in robotics development?",
      answer: "To provide isolated, reproducible development environments that ensure consistent behavior across different systems",
      category: "Tools"
    },
    {
      id: 4,
      question: "What edge device is required for Module 4 and beyond?",
      answer: "NVIDIA Jetson Orin Nano (8GB) or Jetson Orin NX (16GB)",
      category: "Hardware"
    },
    {
      id: 5,
      question: "How much RAM is recommended for the workstation?",
      answer: "32GB DDR4/DDR5",
      category: "Hardware"
    }
  ]}
/>
