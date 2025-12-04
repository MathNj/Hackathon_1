---
id: simulating-sensors
title: آنکھیں اور کان شامل کرنا (سینسرز)
sidebar_label: سینسرز کی سمیولیشن
sidebar_position: 3
description: جانیں کہ Gazebo plugins اور GPU ray tracing استعمال کرتے ہوئے روبوٹس میں LiDAR اور depth cameras کیسے شامل کریں
keywords: [lidar, depth camera, sensors, gazebo plugins, ray tracing, gpu]
---

# آنکھیں اور کان شامل کرنا (سینسرز)

## سینسرز کی سمیولیشن کیوں؟

فزیکل سینسرز **مہنگے** ہیں:
- **LiDAR** (Velodyne VLP-16): ~$1,500
- **Depth Camera** (Intel RealSense D435i): ~$200
- **IMU** (Inertial Measurement Unit): ~$50

Gazebo کے ساتھ، آپ ہارڈویئر خریدنے سے پہلے سمیولیٹڈ سینسرز استعمال کرتے ہوئے **perception algorithms تیار کر سکتے ہیں**۔ ایک بار جب آپ کا الگورتھم simulation میں کام کرتا ہے، تو آپ اعتماد کے ساتھ حقیقی sensor خرید سکتے ہیں یہ جانتے ہوئے کہ آپ کا کوڈ تیار ہے۔

---

## Ray Tracing LiDAR کی نقل کیسے کرتی ہے

### LiDAR کیا ہے؟

**LiDAR** (Light Detection and Ranging) ایک **360° tape measure** کی طرح ہے جو لیزر بیم شوٹ کرتا ہے اور فاصلہ کا حساب لگاتا ہے کہ روشنی کو واپس آنے میں کتنا وقت لگتا ہے۔

```
LiDAR والا روبوٹ
    |
    | (360° میں 16 لیزر بیم شوٹ کرتا ہے)
    |
    v
[رکاوٹ] <-- بیم 0.00001 سیکنڈ میں واپس عکاس ہوتی ہے
فاصلہ = (روشنی کی رفتار × وقت) / 2 = 15 میٹر
```

### GPU Ray Tracing

Gazebo اس کی نقل کرتا ہے:
1. **Rays شوٹ کرنا** — LiDAR sensor origin سے (ورچوئل لیزر بیموں کی طرح)
2. **Collision detection** — GPU حساب لگاتا ہے کہ ہر ray 3D space میں اشیاء سے کہاں ٹکراتی ہے
3. **فاصلے کا حساب** — hit point کو فاصلے کی پیمائش میں تبدیل کرتا ہے
4. **ROS 2 message** — فاصلے کی array کے ساتھ `sensor_msgs/LaserScan` شائع کرتا ہے

**GPU کیوں؟** 10 Hz پر 16-beam LiDAR **23 لاکھ rays فی سیکنڈ** شوٹ کرتا ہے۔ CPU کو فی frame ~10 سیکنڈ لگیں گے۔ RTX GPU **0.003 سیکنڈ** میں کرتا ہے (300 FPS)۔

---

## اپنے روبوٹ میں LiDAR سینسر شامل کرنا

آئیے ایک **Velodyne VLP-16 equivalent** (16 لیزر بیم، 360° horizontal scan) روبوٹ میں شامل کریں۔

### قدم 1: LiDAR Link کی تعریف کریں (URDF)

اپنے روبوٹ کی URDF فائل میں یہ شامل کریں:

```xml
<!-- LiDAR Link (فزیکل sensor body) -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.07"/> <!-- چھوٹا cylinder -->
    </geometry>
    <material name="lidar_black">
      <color rgba="0.1 0.1 0.1 1.0"/> <!-- کالا رنگ -->
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.07"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.83"/> <!-- Velodyne VLP-16 کا وزن 830 گرام -->
    <inertia ixx="0.001" ixy="0" ixz="0"
             iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<!-- Joint: LiDAR کو robot base سے جوڑیں -->
<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/> <!-- اوپر mount کریں، base سے 30cm اوپر -->
</joint>
```

### قدم 2: Gazebo Sensor Plugin شامل کریں

اپنی URDF میں **`<gazebo>` block کے اندر** یہ شامل کریں:

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="gpu_lidar">
    <update_rate>10</update_rate> <!-- 10 Hz = فی سیکنڈ 10 scans -->

    <ray>
      <!-- Horizontal Scan: 360 degrees -->
      <scan>
        <horizontal>
          <samples>1800</samples> <!-- فی scan 1800 points -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle> <!-- -180° -->
          <max_angle>3.14159</max_angle>  <!-- +180° -->
        </horizontal>
        <!-- Vertical Scan: 16 beams -15° سے +15° تک -->
        <vertical>
          <samples>16</samples> <!-- Velodyne VLP-16 میں 16 لیزر beams -->
          <resolution>1</resolution>
          <min_angle>-0.2618</min_angle> <!-- -15 degrees -->
          <max_angle>0.2618</max_angle>  <!-- +15 degrees -->
        </vertical>
      </scan>

      <!-- Range: 0.1m سے 100m -->
      <range>
        <min>0.1</min>
        <max>100.0</max>
        <resolution>0.01</resolution> <!-- 1cm precision -->
      </range>

      <!-- Noise: حقیقت پسند Gaussian noise شامل کریں -->
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev> <!-- ±1cm error -->
      </noise>
    </ray>

    <!-- Gazebo Plugin: ROS 2 پر شائع کرتا ہے -->
    <plugin filename="libgazebo_ros_ray_sensor.so" name="lidar_controller">
      <ros>
        <namespace>/lidar</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### قدم 3: روبوٹ Spawn کریں اور تصدیق کریں

```bash
# Terminal 1: Gazebo world لانچ کریں
gz sim test_world.sdf

# Terminal 2: LiDAR کے ساتھ روبوٹ spawn کریں
ros2 run ros_gz_sim create -file robot_with_lidar.urdf -name my_robot

# Terminal 3: LiDAR data echo کریں
ros2 topic echo /lidar/scan
```

**متوقع Output**:
```yaml
header:
  stamp: {sec: 10, nanosec: 500000000}
  frame_id: lidar_link
angle_min: -3.14159
angle_max: 3.14159
angle_increment: 0.00349066
time_increment: 0.0
scan_time: 0.1
range_min: 0.1
range_max: 100.0
ranges: [15.2, 15.3, 15.1, 14.9, ...] # 1800 فاصلے کی پیمائشیں
```

---

## RViz2 میں LiDAR کو Visualize کرنا

```bash
# RViz2 لانچ کریں
rviz2
```

**RViz2 میں**:
1. **Fixed Frame سیٹ کریں**: `map` سے `lidar_link` میں تبدیل کریں (نچلے بائیں dropdown)
2. **LaserScan Display شامل کریں**: "Add" → "By topic" → `/lidar/scan` → LaserScan پر کلک کریں
3. **Visualization ایڈجسٹ کریں**:
   - Size: 0.05
   - Color: Red (RGB: 255, 0, 0)

**آپ کیا دیکھتے ہیں**: سرخ نقاط جو 360° میں LiDAR کے ذریعے پتہ لگائی گئی رکاوٹوں کو دکھاتے ہیں۔ Gazebo میں اشیاء کو منتقل کریں اور real-time میں point cloud کو اپ ڈیٹ ہوتے دیکھیں!

---

## Plugin پیرامیٹرز کو سمجھنا

### `<update_rate>10</update_rate>`

فی سیکنڈ کتنے scans۔ زیادہ = زیادہ ڈیٹا لیکن زیادہ CPU/GPU load۔

- **10 Hz**: نیویگیشن کے لیے معیار (اچھا توازن)
- **20 Hz**: تیز رفتار نیویگیشن (خودکار کاریں)
- **5 Hz**: کم سطح کے نظام یا صرف CPU مشینیں

### `<samples>1800</samples>`

horizontal sweep میں لیزر beams کی تعداد۔ زیادہ = باریک ریزولوشن۔

- **1800 samples** = ہر beam کے درمیان 0.2° (بہت گھنا)
- **360 samples** = ہر beam کے درمیان 1° (کم ریزولوشن، تیز)

### `<vertical><samples>16</samples></vertical>`

لیزر rings کی تعداد (عمودی beams)۔ Velodyne VLP-16 میں 16 ہیں، VLP-32 میں 32۔

### `<noise>` Block

حقیقی سینسرز میں noise ہوتی ہے! Gaussian noise شامل کرنا simulation کو زیادہ حقیقت پسند بناتا ہے۔

```xml
<stddev>0.01</stddev> <!-- فی پیمائش ±1cm random error -->
```

کامل پیمائشوں کے لیے `<noise>` block ہٹائیں (debugging کے لیے مفید)۔

---

## دوسرے سینسرز شامل کرنا (فوری جائزہ)

### Depth Camera (Intel RealSense D435i)

اسی طرح کا عمل، لیکن استعمال کریں:
```xml
<sensor name="depth_camera" type="depth_camera">
  <plugin filename="libgazebo_ros_camera.so" ...>
```

شائع کرتا ہے:
- `/camera/image_raw` (RGB image)
- `/camera/depth/image_raw` (Depth map)

### IMU (Inertial Measurement Unit)

```xml
<sensor name="imu" type="imu">
  <plugin filename="libgazebo_ros_imu_sensor.so" ...>
```

شائع کرتا ہے:
- `/imu/data` (Orientation، angular velocity، linear acceleration)

---

## کارکردگی: GPU بمقابلہ CPU

| Sensor Configuration | RTX 4070 Ti (GPU) | صرف CPU |
|----------------------|-------------------|----------|
| LiDAR (16 beams، 10 Hz) | 300 FPS | 2 FPS |
| LiDAR (64 beams، 20 Hz) | 60 FPS | &lt;1 FPS |
| Depth Camera (640×480، 30 Hz) | 200 FPS | 1 FPS |

**نتیجہ**: RTX GPU کے بغیر، sensor simulation **ناقابل استعمال** ہے۔ RT cores سے 10-100x speedup real-time development کے لیے لازمی ہے۔

---

## عام Errors

### `/lidar/scan` topic شائع نہیں ہو رہا

**Debug**:
```bash
# چیک کریں کہ آیا plugin load ہو گیا
gz topic -l | grep lidar

# Gazebo console errors کے لیے چیک کریں
# "Failed to load plugin" messages تلاش کریں
```

**حل**: تصدیق کریں کہ `libgazebo_ros_ray_sensor.so` موجود ہے:
```bash
ls /opt/ros/humble/lib/ | grep gazebo_ros_ray
```

### RViz2 کوئی ڈیٹا نہیں دکھاتا

**حل**: تصدیق کریں کہ fixed frame sensor frame سے match کرتا ہے:
- RViz2 Fixed Frame: `lidar_link`
- Sensor `<frame_name>`: `lidar_link`

یہ **match ضرور ہونے چاہئیں** ورنہ آپ دیکھیں گے: "Transform [lidar_link] does not exist."

---

## اہم نکات

✅ **LiDAR ray tracing استعمال کرتا ہے** — GPU ورچوئل لیزر بیم شوٹ کرتا ہے اور فاصلے کا حساب لگاتا ہے

✅ **Gazebo plugin**: `libgazebo_ros_ray_sensor.so` ray tracing کو ROS 2 LaserScan میں تبدیل کرتا ہے

✅ **16 beams، 1800 samples، 10 Hz** — معیاری Velodyne VLP-16 configuration

✅ **GPU ضروری** — RTX GPU CPU سے 10-100x تیز ہے (300 FPS بمقابلہ 2 FPS)

✅ **RViz2 میں Visualize کریں** — fixed frame `lidar_link` پر سیٹ کریں، LaserScan display شامل کریں

---

**اگلا**: جانیں کہ Gazebo کے بجائے Unity کب استعمال کرنا ہے [**Unity میں High-Fidelity Rendering**](./04-unity-visualization.md)! 🎨
