---
id: gazebo-fortress-setup
title: Gazebo Fortress کے ساتھ فزکس
sidebar_label: Gazebo Fortress سیٹ اپ
sidebar_position: 2
description: سخت جسم کی حرکیات، Gazebo Fortress کی تنصیب، اور URDF بمقابلہ SDF فارمیٹس کو سمجھیں
keywords: [gazebo, fortress, physics, urdf, sdf, rigid body dynamics, simulation]
---

# Gazebo Fortress کے ساتھ فزکس

## Gazebo کیوں؟

**Gazebo** ROS 2 کے لیے صنعتی معیار کا فزکس سمیولیٹر ہے۔ گیم انجنز (Unity، Unreal) کے برعکس جو بصری حقیقت پسندی کو ترجیح دیتے ہیں، Gazebo **فزکس کی درستگی** پر توجہ مرکوز کرتا ہے — جو کنٹرول الگورتھم، نیویگیشن اسٹیکس، اور ہیرا پھیری کے کاموں کی جانچ کے لیے اہم ہے۔

**Gazebo Fortress** (سابقہ Ignition Gazebo) جدید ورژن ہے جو `ros_gz` bridge کے ذریعے ROS 2 Humble کے ساتھ مقامی طور پر مربوط ہوتا ہے۔

---

## Rigid Body Dynamics: بنیاد

روبوٹس کو سمیولیٹ کرنے سے پہلے، آپ کو فزکس ماڈل کو سمجھنے کی ضرورت ہے جو Gazebo استعمال کرتا ہے۔

### Rigid Body کیا ہے؟

**Rigid body** ایک ایسی چیز ہے جو **بگڑتی نہیں** — یہ صرف منتقل (move) ہوتی ہے اور گھومتی ہے۔ یہ ایک سادہ سازی ہے (حقیقی چیزیں جھکتی اور سکڑتی ہیں)، لیکن یہ کمپیوٹیشنل طور پر موثر ہے اور زیادہ تر روبوٹکس ایپلیکیشنز کے لیے کافی درست ہے۔

### اہم فزکس پیرامیٹرز

| پیرامیٹر | تعریف | مثال |
|-----------|------------|---------|
| **Mass** | کلوگرام میں کل وزن | موبائل روبوٹ بیس: 15 kg |
| **Inertia** | گھومنے کے خلاف مزاحمت (3x3 tensor) | زیادہ inertia = گھومنا مشکل |
| **Gravity** | چیزوں کو نیچے کھینچنے والی قوت | زمین: -9.81 m/s² Z-axis میں |
| **Friction** | سطح کے تعامل کے coefficients | ربر کنکریٹ پر: µ = 0.8 |
| **Damping** | توانائی کی کمی (ہوا کی مزاحمت کی نقل کرتا ہے) | Linear damping: 0.1، Angular damping: 0.05 |

### درست فزکس کیوں اہم ہے

- **PID Control Tuning**: آپ کے کنٹرولر gains درست inertia اور damping پر منحصر ہیں
- **Grasping**: ہیرا پھیری کے کاموں کے لیے رابطے کی قوتیں حقیقت پسند ہونی چاہئیں
- **Navigation**: پہیے کی پھسلن اور زمین کا تعامل راستے کی منصوبہ بندی کو متاثر کرتا ہے

---

## تنصیب

### Gazebo Fortress اور ROS 2 Bridge انسٹال کریں

```bash
# Package list اپ ڈیٹ کریں
sudo apt update

# ROS 2 Humble integration کے ساتھ Gazebo Fortress انسٹال کریں
sudo apt install ros-humble-ros-gz

# یہ انسٹال کرتا ہے:
# - Gazebo Fortress (gz-sim7)
# - ros_gz_bridge (ROS 2 ↔ Gazebo communication)
# - ros_gz_sim (روبوٹس spawn کریں، simulation control کریں)
# - ros_gz_image (Camera/depth sensor integration)
```

### تنصیب کی تصدیق کریں

```bash
# Gazebo version چیک کریں (Fortress 7.x کی توقع ہے)
gz sim --version

# ROS 2 packages چیک کریں
ros2 pkg list | grep ros_gz
# دکھانا چاہیے: ros_gz_bridge، ros_gz_sim، ros_gz_image
```

### خالی دنیا ٹیسٹ کریں

```bash
# خالی دنیا کے ساتھ Gazebo لانچ کریں
gz sim empty.sdf
```

**متوقع**: Gazebo GUI ایک سرمئی خلا کے ساتھ کھلتا ہے۔ آپ ماؤس drag کے ساتھ کیمرہ گھما سکتے ہیں۔

---

## URDF بمقابلہ SDF: کون سا فارمیٹ استعمال کریں؟

### موازنہ جدول

| خصوصیت | URDF (ROS معیار) | SDF (Gazebo معیار) |
|---------|---------------------|----------------------|
| **مقصد** | صرف روبوٹ کی تفصیلات | دنیائیں + روبوٹس + سینسرز |
| **دائرہ کار** | فی فائل ایک ماڈل | فی فائل متعدد ماڈلز |
| **فزکس** | بنیادی (mass، inertia) | جدید (friction، damping، surface properties) |
| **سینسرز** | محدود (`<gazebo>` tags کی ضرورت ہے) | plugins کے ساتھ مقامی sensor support |
| **روشنی** | سپورٹ نہیں | مکمل lighting control |
| **دنیائیں** | سپورٹ نہیں | ✅ Ground planes، sky، obstacles |

### ہر ایک کب استعمال کریں

- **URDF استعمال کریں** اگر:
  - آپ ایک روبوٹ کی تعریف کر رہے ہیں جو ROS 2 میں استعمال ہوگا (Module 1 سے پہلے سے واقف)
  - آپ modularity کے لیے `xacro` macros استعمال کرنا چاہتے ہیں
  - آپ موجودہ ROS packages کے ساتھ کام کر رہے ہیں جو URDF کی توقع کرتے ہیں

- **SDF استعمال کریں** اگر:
  - آپ simulation **دنیائیں** بنا رہے ہیں (ground plane، obstacles، lighting)
  - آپ کو جدید فزکس کی ضرورت ہے (پیچیدہ collisions، custom surface properties)
  - آپ Gazebo-specific خصوصیات استعمال کر رہے ہیں (ایک فائل میں متعدد روبوٹس)

**بہترین مشق**: روبوٹس کے لیے URDF استعمال کریں (طلباء پہلے سے جانتے ہیں)، دنیاؤں کے لیے SDF۔ Gazebo براہ راست URDF فائلیں `ros_gz_sim` کے ذریعے لوڈ کر سکتا ہے۔

---

## اپنی پہلی Gazebo دنیا بنائیں

آئیے ایک سادہ دنیا ground plane اور gravity کے ساتھ بنائیں۔

### `test_world.sdf` بنائیں

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="test_world">

    <!-- Physics Engine: ODE with 1ms time step -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size> <!-- استحکام کے لیے 1ms -->
      <real_time_factor>1.0</real_time_factor> <!-- حقیقی وقت کی رفتار پر چلائیں -->
    </physics>

    <!-- Gravity: معیاری زمین کی کشش ثقل -->
    <gravity>0 0 -9.81</gravity>

    <!-- Lighting: سورج کی سمتی روشنی -->
    <light name="sun" type="directional">
      <pose>5 5 10 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1.0</diffuse>
      <direction>-0.5 -0.5 -1.0</direction>
      <cast_shadows>true</cast_shadows>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static> <!-- حرکت نہیں کرتا، لامحدود mass -->
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal></plane> <!-- XY plane میں فلیٹ -->
          </geometry>
          <surface>
            <friction>
              <ode><mu>0.8</mu><mu2>0.8</mu2></ode> <!-- ربر کی طرح friction -->
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient> <!-- سرمئی رنگ -->
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

### دنیا لانچ کریں

```bash
gz sim test_world.sdf
```

**آپ کیا دیکھتے ہیں**: ایک سرمئی ground plane حقیقت پسند روشنی اور سائے کے ساتھ۔ فزکس simulation شروع کرنے کے لیے نچلے بائیں کونے میں "Play" بٹن (▶️) پر کلک کرنے کی کوشش کریں۔

---

## فزکس انجن کو سمجھنا

### Time Step: 1ms کیوں؟

`<max_step_size>0.001</max_step_size>` کا مطلب ہے کہ Gazebo ہر **1 ملی سیکنڈ** میں فزکس کا حساب لگاتا ہے۔

- **چھوٹا time step (0.0001s)**: زیادہ درست لیکن **سست** simulation
- **بڑا time step (0.01s)**: تیز لیکن اشیاء ایک دوسرے کے **اندر سے گزر سکتی ہیں**

زیادہ تر روبوٹکس ایپلیکیشنز کے لیے **1ms بہترین مقام ہے** (مستحکم اور تیز)۔

### Gravity: Vector فارمیٹ

```xml
<gravity>0 0 -9.81</gravity>
```

- **X-axis**: 0 (کوئی سائیڈ ویز gravity نہیں)
- **Y-axis**: 0 (کوئی آگے/پیچھے gravity نہیں)
- **Z-axis**: -9.81 m/s² (نیچے کی طرف کھینچاؤ)

اگر آپ چاند پر سمیولیٹ کر رہے تھے، تو آپ `-9.81` کے بجائے `-1.62` استعمال کریں گے۔

---

## URDF کو SDF میں تبدیل کرنا (اختیاری)

اگر آپ کے پاس Module 1 سے URDF فائل ہے اور آپ اسے SDF میں تبدیل کرنا چاہتے ہیں:

```bash
# URDF کو SDF میں تبدیل کریں
gz sdf -p my_robot.urdf > my_robot.sdf
```

**تاہم**، Gazebo Fortress براہ راست URDF فائلیں لوڈ کر سکتا ہے، لہذا یہ عام طور پر غیر ضروری ہے۔ صرف استعمال کریں:

```bash
ros2 run ros_gz_sim create -file my_robot.urdf -name my_robot
```

---

## مسائل کا حل

### `gz sim` command نہیں ملی

**حل**: Gazebo کو اپنے PATH میں شامل کریں:

```bash
echo 'export PATH=$PATH:/opt/ros/humble/bin' >> ~/.bashrc
source ~/.bashrc
```

### Gazebo لانچ کرتے وقت خالی سکرین

**حل**: OpenGL support چیک کریں:

```bash
glxinfo | grep "OpenGL version"
# OpenGL 3.3 یا اس سے اوپر دکھانا چاہیے
```

اگر OpenGL غائب ہے، تو آپ کو graphics drivers انسٹال کرنے کی ضرورت ہو سکتی ہے (Module 0 دیکھیں)۔

### GPU errors

**حل**: NVIDIA drivers کی تصدیق کریں:

```bash
nvidia-smi
# آپ کا RTX GPU اور driver 525+ دکھانا چاہیے
```

---

## اہم نکات

✅ **Gazebo Fortress** جدید ROS 2 simulator ہے (سابقہ Ignition)

✅ **Rigid body dynamics** ماڈل: Mass، inertia، friction، damping، gravity

✅ روبوٹس کے لیے **URDF** (طلباء جانتے ہیں)، دنیاؤں کے لیے **SDF** (زیادہ خصوصیات)

✅ **انسٹال کمانڈ**: `sudo apt install ros-humble-ros-gz`

✅ استحکام اور رفتار کے لیے **1ms time step** بہترین ہے

---

**اگلا**: جانیں کہ اپنے روبوٹ میں sensors (LiDAR، cameras) کیسے شامل کریں [**آنکھیں اور کان شامل کرنا (سینسرز)**](./03-simulating-sensors.md)! 🤖
