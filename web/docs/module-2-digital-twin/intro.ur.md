# ماڈیول 2: The Digital Twin - Simulation اور Sensors

## جائزہ

Physical robots پر deploy کرنے سے پہلے، ہم **digital twins** بناتے ہیں — virtual replicas جو ہمیں simulation میں محفوظ طریقے سے algorithms کو test کرنے دیتے ہیں۔ یہ ماڈیول Gazebo Classic، Gazebo Sim (Ignition)، اور photorealistic environments کے لیے Unity کا احاطہ کرتا ہے، نیز sensor simulation (LiDAR، cameras، IMUs)۔

## سیکھنے کے مقاصد

اس ماڈیول کے اختتام تک، آپ یہ کر سکیں گے:
- 🌍 Gazebo simulation environments سیٹ اپ کریں
- 📷 Cameras، LiDAR، اور depth sensors کو simulate کریں
- 🎮 Photorealistic rendering کے لیے Unity کو ROS 2 کے ساتھ integrate کریں
- 🔬 Sensor fusion algorithms نافذ کریں (camera + LiDAR)
- 🏗️ Testing کے لیے custom simulation worlds بنائیں
- 📊 RViz اور Plotjuggler میں sensor data کو visualize کریں

## Simulation کیوں اہم ہے

**حقیقی دنیا کی testing مہنگی اور خطرناک ہے:**
- 💰 Development کے دوران $50,000 کے robot کو توڑنا
- ⏱️ سست iteration cycles (deploy → test → debug)
- 🚫 خطرناک scenarios (edge cases، collisions)

**Simulation فعال بناتا ہے:**
- ✅ لامحدود resets اور تیز iteration
- ✅ Edge case testing (sensor failures، extreme weather)
- ✅ Parallel testing (ایک ساتھ 100 scenarios چلائیں)

## پیشگی ضروریات

- Module 1 مکمل (ROS 2 fundamentals)
- بنیادی 3D geometry کی سمجھ
- Coordinate systems سے واقفیت (TF2)

## ماڈیول کی ساخت

### بنیادی تصورات
1. [Gazebo Architecture](./gazebo-overview.md) - Physics engines, plugins, sensors
2. [Simulating Sensors](./sensor-simulation.md) - LiDAR, RGB-D, IMU, GPS
3. [Unity Integration](./unity-ros2.md) - ROS-TCP-Connector setup
4. [Sensor Fusion](./sensor-fusion.md) - Kalman filters, data synchronization
5. [Custom World Building](./world-creation.md) - SDF files, model import

### ہاتھوں ہاتھ Tutorials
- **Tutorial 1**: LiDAR کے ساتھ Gazebo میں TurtleBot3 spawn کریں
- **Tutorial 2**: Dynamic lighting کے ساتھ Unity environment بنائیں
- **Tutorial 3**: Sensor fusion node نافذ کریں (camera + IMU)

### مشقیں
- ✏️ مشق 1: Obstacles کے ساتھ warehouse simulation بنائیں
- ✏️ مشق 2: Simulated stereo camera کو calibrate کریں
- ✏️ مشق 3: Unity میں photorealistic outdoor environment بنائیں

### تشخیص
- 📝 کوئز: Sensor characteristics اور simulation parameters
- 💻 Coding Challenge: Simulated environments میں object detection

## تخمینی مدت

**3 ہفتے** (کل 15-20 گھنٹے)

## ہارڈویئر کی ضروریات

- **GPU**: Unity میں real-time ray tracing کے لیے RTX 4070 Ti
- **RAM**: 32GB (Gazebo + Unity memory-intensive ہو سکتے ہیں)
- **Storage**: Unity assets کے لیے 50GB free space

## متعارف کرائے گئے Tools

- **Gazebo Sim** (Ignition Fortress)
- **Unity 2022 LTS** ROS-TCP-Connector کے ساتھ
- **RViz2** sensor data visualization کے لیے
- **Plotjuggler** time-series analysis کے لیے

## عام غلطیاں

- ❌ `Resource not found: [robot_description]` → Launch file میں URDF load نہیں ہوا
- ❌ Unity-ROS connection timeout → Firewall TCP port 10000 کو block کر رہا ہے
- ❌ Gazebo physics instability → Timestep بہت بڑا یا collision geometry issues

## اگلے قدمات

Simulation fundamentals کو سمجھنے کے لیے [Gazebo Architecture](./gazebo-overview.md) سے شروع کریں →

---

**Pro Tip**: Offline algorithm testing کے لیے simulation سے sensor data capture کرنے کے لیے `ros2 bag record` استعمال کریں۔
