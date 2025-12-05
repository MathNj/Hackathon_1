# ماڈیول 3: The Robot Brain - Perception اور Navigation

## جائزہ

یہ ماڈیول آپ کو NVIDIA Isaac Sim، Visual SLAM (Simultaneous Localization and Mapping)، اور Nav2 (ROS 2 Navigation Stack) استعمال کرتے ہوئے autonomous robots کا "brain" بنانا سکھاتا ہے۔ آپ سیکھیں گے کہ robots اپنے environment کو کیسے "دیکھتے" ہیں اور collision-free paths کی منصوبہ بندی کیسے کرتے ہیں۔

## سیکھنے کے مقاصد

اس ماڈیول کے اختتام تک، آپ یہ کر سکیں گے:
- 🗺️ ORB-SLAM3 اور RTAB-Map استعمال کرتے ہوئے Visual SLAM نافذ کریں
- 🧭 Autonomous navigation کے لیے Nav2 کنفیگر کریں
- 🚧 Costmaps استعمال کرتے ہوئے obstacle avoidance انجام دیں
- 🎯 DWA اور TEB planners کے ساتھ optimal paths کی منصوبہ بندی کریں
- 🏭 NVIDIA Isaac Sim پر algorithms deploy کریں
- 🤖 Jetson Orin edge devices پر navigation test کریں

## Isaac Sim کیوں؟

NVIDIA Isaac Sim فراہم کرتا ہے:
- **Photorealistic Rendering**: Omniverse RTX ray tracing
- **Physics Accuracy**: GPU acceleration کے ساتھ PhysX 5 engine
- **ROS 2 Integration**: Native ROS 2 bridge (کوئی middleware hacks نہیں)
- **Synthetic Data Generation**: Computer vision models کے لیے training data

BMW، Volvo، اور Amazon Robotics جیسی کمپنیاں digital twin development کے لیے استعمال کرتی ہیں۔

## پیشگی ضروریات

- Module 2 مکمل (simulation اور sensors)
- Linear algebra basics (transformations، matrices)
- Probability کی سمجھ (SLAM کے لیے Bayes theorem)

## ماڈیول کی ساخت

### بنیادی تصورات
1. [Isaac Sim Setup](./isaac-sim-setup.md) - Installation اور first simulation
2. [Visual SLAM Fundamentals](./vslam-intro.md) - ORB-SLAM3 اور RTAB-Map
3. [Nav2 Architecture](./nav2-overview.md) - Behavior trees, planners, controllers
4. [Costmap Configuration](./costmaps.md) - Static, dynamic, اور inflation layers
5. [Path Planning Algorithms](./path-planning.md) - A*, DWA, TEB

### ہاتھوں ہاتھ Tutorials
- **Tutorial 1**: Isaac Sim warehouse میں ORB-SLAM3 چلائیں
- **Tutorial 2**: Mobile robot کے لیے Nav2 کنفیگر کریں
- **Tutorial 3**: Jetson Orin پر obstacle avoidance نافذ کریں

### مشقیں
- ✏️ مشق 1: Multi-floor building کا 3D map بنائیں
- ✏️ مشق 2: تنگ جگہوں کے لیے Nav2 parameters tune کریں
- ✏️ مشق 3: Jetson Orin پر navigation stack deploy کریں

### تشخیص
- 📝 کوئز: SLAM algorithms اور Nav2 components
- 💻 Coding Challenge: Autonomous warehouse navigation

## تخمینی مدت

**4 ہفتے** (کل 20-25 گھنٹے)

## ہارڈویئر کی ضروریات

### Workstation
- **GPU**: Isaac Sim کے لیے RTX 4070 Ti (کم از کم 12GB VRAM)
- **CPU**: SLAM processing کے لیے 8+ cores
- **RAM**: 32GB (Isaac Sim + ROS 2 nodes)

### Edge Device
- Deployment testing کے لیے **Jetson Orin Nano (8GB)**
- **Intel RealSense D435i** یا اسی طرح کا depth camera (اختیاری)

## متعارف کرائے گئے Tools

- **NVIDIA Isaac Sim 2023.1+** (Omniverse)
- **ORB-SLAM3** (Visual SLAM)
- **RTAB-Map** (RGB-D SLAM)
- **Nav2** (Navigation stack)
- **Cartographer** (Google کی SLAM implementation)

## عام غلطیاں

- ❌ `Isaac Sim won't launch` → GPU driver mismatch (525+ driver ضروری)
- ❌ `SLAM drift over time` → Poor feature detection یا loop closure failure
- ❌ `Robot stuck in Nav2` → Costmap configuration بہت conservative
- ❌ `Jetson Orin out of memory` → Sensor resolution کم کریں یا model quantization استعمال کریں

## Performance Benchmarks

RTX 4070 Ti پر متوقع performance:
- **ORB-SLAM3**: 1280x720 video پر 30+ FPS
- **Nav2 replanning**: 50m² map کے لیے &lt;100ms
- **Isaac Sim simulation**: Photorealistic mode میں 60 FPS

## اگلے قدمات

Omniverse install کرنے کے لیے [Isaac Sim Setup](./isaac-sim-setup.md) سے شروع کریں →

---

**Pro Tip**: Isaac Sim پر deploy کرنے سے پہلے Gazebo میں Nav2 test کرنے کے لیے `ros2 launch nav2_bringup tb3_simulation_launch.py` استعمال کریں۔

## جائزہ Flashcards

import Flashcards from '@site/src/components/Flashcards';

<Flashcards
  title="Robot Brain - Perception اور Navigation کا جائزہ"
  cards={[
  {
    id: 1,
    question: "SLAM کا مطلب کیا ہے اور یہ کیا کرتا ہے؟",
    answer: "Simultaneous Localization and Mapping - ایک unknown environment کا map بناتا ہے جبکہ بیک وقت robot کے location کو اس کے اندر track کرتا ہے",
    category: "Navigation"
  },
  {
    id: 2,
    question: "Path planning algorithms کی دو main types کون سی ہیں؟",
    answer: "Global path planning (A*، Dijkstra) long-range routes کے لیے، اور local path planning (DWA، TEB) dynamic obstacle avoidance کے لیے",
    category: "Planning"
  },
  {
    id: 3,
    question: "Nav2 کی costmap کا مقصد کیا ہے؟",
    answer: "Environment کو grid کے طور پر represent کرنا جہاں ہر cell کی cost value ہے، جو robot کو obstacles سے بچنے اور safe paths plan کرنے میں مدد کرتی ہے",
    category: "Navigation"
  },
  {
    id: 4,
    question: "Nav2 replanning کے لیے typical latency requirement کیا ہے؟",
    answer: "50m² map کے لیے 100ms سے کم تاکہ responsive navigation یقینی بنائی جا سکے",
    category: "Performance"
  },
  {
    id: 5,
    question: "اس module میں دو popular SLAM algorithms کے نام بتائیں۔",
    answer: "ORB-SLAM3 (visual SLAM) اور Cartographer (LiDAR SLAM)",
    category: "Algorithms"
  },
  {
    id: 6,
    question: "Isaac Sim کیا ہے اور یہ کیوں استعمال ہوتا ہے؟",
    answer: "NVIDIA کا photorealistic robotics simulator جو Omniverse پر بنایا گیا ہے، realistic environments میں accurate physics اور sensor simulation کے ساتھ robots test کرنے کے لیے استعمال ہوتا ہے",
    category: "ٹولز"
  }
]}
/>
