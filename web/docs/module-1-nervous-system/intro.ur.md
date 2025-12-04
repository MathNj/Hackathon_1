# ماڈیول 1: The Nervous System - ROS 2 بنیادی باتیں

## جائزہ

پہلے بنیادی ماڈیول میں خوش آمدید! جس طرح حیاتیاتی حیاتیات میں nervous system جسم کے مختلف حصوں کے درمیان سگنل منتقل کرنے کے لیے ہوتا ہے، اسی طرح robots ROS 2 (Robot Operating System 2) کو اپنی communication backbone کے طور پر استعمال کرتے ہیں۔ یہ ماڈیول آپ کو سکھاتا ہے کہ intelligent robots کا "nervous system" کیسے بنایا جائے۔

## سیکھنے کے مقاصد

اس ماڈیول کے اختتام تک، آپ یہ کر سکیں گے:
- 🧠 ROS 2 architecture کو سمجھیں (Nodes, Topics, Services, Actions)
- 🐍 `rclpy` استعمال کرتے ہوئے Python ROS 2 nodes لکھیں
- 📡 Publisher-subscriber communication patterns نافذ کریں
- 🔧 Custom message types اور service definitions بنائیں
- 🤖 URDF (Unified Robot Description Format) استعمال کرتے ہوئے robots کو model کریں
- 🚀 Launch files استعمال کرتے ہوئے multi-node systems شروع کریں

## یہ کیوں اہم ہے

ROS 2 robot software development کے لیے industry standard ہے۔ استعمال کرنے والے:
- **Boston Dynamics** (Spot SDK)
- **NASA** (Mars rovers)
- **Waymo** (Autonomous vehicles)
- **Amazon Robotics** (Warehouse automation)

ROS 2 میں مہارت حاصل کرنا professional robotics engineering کا آپ کا دروازہ ہے۔

## پیشگی ضروریات

- Module 0 مکمل (Ubuntu + ROS 2 Humble installed)
- Python basics (functions, classes, decorators)
- Command line proficiency

## ماڈیول کی ساخت

### بنیادی تصورات
1. [ROS 2 Architecture](./ros2-architecture.md) - Nodes, Topics, Services, Actions
2. [Creating Your First Node](./first-node.md) - `rclpy` میں Hello World
3. [Publisher-Subscriber Pattern](./pubsub.md) - Sensor data simulation
4. [Services and Clients](./services.md) - Request-response communication
5. [URDF Basics](./urdf-modeling.md) - Robot description files

### ہاتھوں ہاتھ Tutorials
- **Tutorial 1**: velocity controller node بنائیں
- **Tutorial 2**: sensor fusion system بنائیں
- **Tutorial 3**: URDF میں mobile robot کو model کریں

### مشقیں
- ✏️ مشق 1: temperature monitoring system نافذ کریں
- ✏️ مشق 2: multi-robot communication network بنائیں
- ✏️ مشق 3: URDF میں custom robot manipulator ڈیزائن کریں

### تشخیص
- 📝 کوئز: ROS 2 concepts پر 5 سوالات
- 💻 Coding Challenge: autonomous navigation decision node بنائیں

## تخمینی مدت

**2 ہفتے** (کل 10-15 گھنٹے)

## عام غلطیاں اور Debugging

اس ماڈیول میں، آپ کو "Common Errors" sections ملیں گے جو عام غلطیوں کو نمایاں کرتے ہیں:
- ❌ `ImportError: cannot import name 'Node'` → ROS 2 sourcing issue
- ❌ Topic name mismatch → Namespace problems
- ❌ URDF parsing errors → XML syntax mistakes

## استعمال شدہ ہارڈویئر

- **Workstation**: RViz visualization کے لیے NVIDIA RTX 4070 Ti
- **Test Platform**: Turtlebot3 simulation (ابھی تک physical robot کی ضرورت نہیں)

## اگلے قدمات

اپنا پہلا ROS 2 node بنانے کے لیے تیار ہیں؟ [ROS 2 Architecture](./ros2-architecture.md) سے شروع کریں →

---

**Pro Tip**: اس ماڈیول پر کام کرتے وقت [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/) کو reference کے طور پر کھلا رکھیں۔
