---
id: nodes-and-topics
title: عملی طور پر Nodes اور Topics - آپ کا پہلا ROS 2 Code
sidebar_label: Nodes اور Topics
sidebar_position: 2
description: Python میں اپنے پہلے ROS 2 nodes لکھیں - rclpy publish-subscribe pattern استعمال کرتے ہوئے Talker اور Listener
keywords: [ros2, rclpy, publisher, subscriber, python, nodes, topics, tutorial]
---

# عملی طور پر Nodes اور Topics

آپ کے پہلے عملی ROS 2 programming تجربے میں خوش آمدید! اس tutorial میں، آپ `rclpy` (ROS 2 Python client library) استعمال کرتے ہوئے اصل Python code لکھیں گے تاکہ دو nodes بنائیں جو topics کے ذریعے communicate کریں۔ آخر تک، آپ اپنے Talker اور Listener nodes کے درمیان real-time میں messages بہتے دیکھیں گے — ہر ROS 2 system کا بنیادی building block۔

---

## Setup کی تصدیق

Coding شروع کرنے سے پہلے، آئیے یقینی بنائیں کہ آپ کا ROS 2 environment صحیح طریقے سے configure ہے۔

### 1. ROS 2 Humble Installation چیک کریں

ایک terminal کھولیں اور اپنے ROS 2 version کی تصدیق کریں:

```bash
ros2 --version
```

**متوقع output**:
```
ros2 cli version: 0.18.5
```

### 2. ROS 2 Environment Source کریں

ROS 2 کو environment variables set کرنے کی ضرورت ہے۔ اسے اپنے `~/.bashrc` میں شامل کریں (اگر پہلے سے نہیں ہے):

```bash
source /opt/ros/humble/setup.bash
```

پھر اپنے terminal کو reload کریں یا چلائیں:

```bash
source ~/.bashrc
```

### 3. Python اور rclpy کی تصدیق کریں

چیک کریں کہ Python 3 اور ROS 2 Python library دستیاب ہیں:

```bash
python3 --version
python3 -c "import rclpy; print('rclpy imported successfully')"
```

**متوقع output**:
```
Python 3.10.x
rclpy imported successfully
```

✅ اگر تمام checks pass ہو جائیں، تو آپ اپنا پہلا node لکھنے کے لیے تیار ہیں!

---

## Tutorial 1: Publisher لکھنا (Talker Node)

ہمارا پہلا node ایک topic پر messages **publish** کرے گا۔ اسے ایک neuron کے طور پر سوچیں جو nerve pathway کے ساتھ signals بھیج رہا ہے۔

### مکمل Publisher Code

`minimal_publisher.py` نام کی نئی file بنائیں:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    ایک سادہ ROS 2 publisher node جو ایک fixed rate پر messages بھیجتا ہے۔
    تشبیہ: ایک sensory neuron مسلسل "Hello World" signals کی رپورٹ کر رہا ہے۔
    """

    def __init__(self):
        # ایک unique نام کے ساتھ Node initialize کریں
        super().__init__('minimal_publisher')

        # 'topic' channel پر ایک publisher بنائیں
        # - Message type: String (std_msgs package سے)
        # - Topic name: 'topic'
        # - Queue size: 10 (10 messages کے لیے buffer اگر subscriber سست ہے)
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # ایک timer بنائیں جو ہر 0.5 seconds میں trigger ہو
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Message number track کرنے کے لیے counter
        self.i = 0

        self.get_logger().info('Publisher node initialized - sending messages every 0.5s')

    def timer_callback(self):
        """
        یہ function timer کے ذریعے ہر 0.5 seconds میں call ہوتا ہے۔
        یہ ایک message بناتا ہے، publish کرتا ہے، اور action log کرتا ہے۔
        """
        # ایک نیا String message بنائیں
        msg = String()
        msg.data = 'Hello World: %d' % self.i

        # Message کو 'topic' پر publish کریں
        self.publisher_.publish(msg)

        # Console پر log کریں (terminal میں ظاہر ہوتا ہے)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        # Counter increment کریں
        self.i += 1


def main(args=None):
    # rclpy library initialize کریں
    rclpy.init(args=args)

    # ہمارے publisher node کا ایک instance بنائیں
    minimal_publisher = MinimalPublisher()

    # Node کو چلتا رکھیں اور callbacks process کریں
    # (یہ ایک blocking call ہے - program یہاں Ctrl+C تک رہتا ہے)
    rclpy.spin(minimal_publisher)

    # Cleanup: node کو explicitly destroy کریں
    minimal_publisher.destroy_node()

    # rclpy library shutdown کریں
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

### Code Walkthrough: Publisher

آئیے ہر section کو توڑیں:

#### 1. Imports

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
```

- `rclpy`: ROS 2 Python client library (جیسے آپ HTTP کے لیے `requests` import کرتے ہیں)
- `Node`: تمام ROS 2 nodes کے لیے base class
- `String`: ایک simple message type جس میں text field (`data`) ہوتا ہے

#### 2. Class Definition

```python
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
```

- ROS 2 functionality حاصل کرنے کے لیے `Node` سے inherit کریں
- `super().__init__('minimal_publisher')` اس node کو `/minimal_publisher` نام سے register کرتا ہے
- Node names ROS 2 graph میں unique ہونے چاہیے

#### 3. Publisher بنانا

```python
self.publisher_ = self.create_publisher(String, 'topic', 10)
```

- `create_publisher(MessageType, topic_name, queue_size)`
- `String`: اس publisher کے بھیجے جانے والے messages کی type
- `'topic'`: Communication channel کا نام (جیسے phone number)
- `10`: Quality of Service (QoS) queue size — 10 messages تک buffer کرتا ہے اگر network سست ہے

#### 4. Timer Setup

```python
timer_period = 0.5
self.timer = self.create_timer(timer_period, self.timer_callback)
```

- `timer_callback()` کو ہر 0.5 seconds میں call کرتا ہے
- یہ ہمارے publisher کا "heartbeat" بناتا ہے

#### 5. Messages Publish کرنا

```python
def timer_callback(self):
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.i += 1
```

- ایک `String` message object بنائیں
- اس کا `data` field "Hello World: 0"، "Hello World: 1"، وغیرہ پر set کریں
- اسے topic پر publish کریں
- Action log کریں (terminal میں visible)

#### 6. Main Function

```python
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```

- `rclpy.init()`: ROS 2 communication initialize کریں
- `MinimalPublisher()`: ہمارا node بنائیں
- `rclpy.spin()`: Node چلائیں (timer callbacks ہمیشہ کے لیے process کریں)
- Interrupted ہونے پر cleanup کریں (Ctrl+C)

---

### Publisher چلانا

Script کو executable بنائیں اور چلائیں:

```bash
chmod +x minimal_publisher.py
python3 minimal_publisher.py
```

**متوقع output**:
```
[INFO] [1702345678.123456789] [minimal_publisher]: Publisher node initialized - sending messages every 0.5s
[INFO] [1702345678.623456789] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1702345679.123456789] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [1702345679.623456789] [minimal_publisher]: Publishing: "Hello World: 2"
...
```

🎉 **کامیابی!** آپ کا node messages publish کر رہا ہے۔ لیکن وہ کہاں جا رہے ہیں؟ آئیے معلوم کریں۔

---

## Tutorial 2: Subscriber لکھنا (Listener Node)

اب ہم ایک node بنائیں گے جو topic کو **subscribe** کرے اور Talker کے ذریعے publish کیے گئے messages حاصل کرے۔

### مکمل Subscriber Code

`minimal_subscriber.py` بنائیں:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    ایک سادہ ROS 2 subscriber node جو ایک topic پر messages سنتا ہے۔
    تشبیہ: ایک motor neuron ایک action trigger کرنے کے لیے signals حاصل کر رہا ہے۔
    """

    def __init__(self):
        # ایک unique نام کے ساتھ Node initialize کریں
        super().__init__('minimal_subscriber')

        # 'topic' channel کو ایک subscription بنائیں
        # - Message type: String
        # - Topic name: 'topic' (publisher سے match ہونا چاہیے!)
        # - Callback function: listener_callback (جب message آئے تو call ہوتا ہے)
        # - Queue size: 10
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

        # Unused variable warning روکیں (Python convention)
        self.subscription

        self.get_logger().info('Subscriber node initialized - waiting for messages')

    def listener_callback(self, msg):
        """
        یہ function automatically ہر بار call ہوتا ہے جب topic پر message آئے۔
        یہ reflex arc کی طرح ہے - signal آئے، action trigger ہو۔
        """
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    # rclpy library initialize کریں
    rclpy.init(args=args)

    # ہمارے subscriber node کا ایک instance بنائیں
    minimal_subscriber = MinimalSubscriber()

    # Node کو چلتا رکھیں اور incoming messages process کریں
    rclpy.spin(minimal_subscriber)

    # Cleanup
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

### Code Walkthrough: Subscriber

Subscriber publisher سے آسان ہے کیونکہ یہ **reactive** ہے (messages کا انتظار کرتا ہے) بجائے **proactive** (messages generate کرتا ہے)۔

#### 1. Subscription بنانا

```python
self.subscription = self.create_subscription(
    String,
    'topic',
    self.listener_callback,
    10)
```

- `create_subscription(MessageType, topic_name, callback_function, queue_size)`
- **Critical**: Topic name `'topic'` publisher کے topic name سے match ہونا چاہیے
- `listener_callback`: یہ function automatically call ہوتا ہے جب message آئے
- Timer کی ضرورت نہیں — ROS 2 ہمارا callback call کرتا ہے جب data دستیاب ہو

#### 2. Callback Function

```python
def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)
```

- **Automatically trigger ہوتا ہے** جب `'topic'` پر message آئے
- `msg`: `String` message object (اس میں `data` field ہے)
- `msg.data`: "Hello World: 0"، "Hello World: 1"، وغیرہ پر مشتمل ہے
- یہ اعصابی نظام میں **reflex arc** کی طرح ہے — stimulus (message arrival) → response (console پر log)

---

### دونوں Nodes کو ایک ساتھ چلانا

اب دلچسپ حصے کے لیے! ہم دونوں nodes کو بیک وقت چلائیں گے اور انہیں communicate کرتے دیکھیں گے۔

#### Terminal 1: Publisher شروع کریں

```bash
python3 minimal_publisher.py
```

#### Terminal 2: Subscriber شروع کریں

```bash
python3 minimal_subscriber.py
```

**Terminal 1 میں متوقع output (Publisher)**:
```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
```

**Terminal 2 میں متوقع output (Subscriber)**:
```
[INFO] [minimal_subscriber]: I heard: "Hello World: 0"
[INFO] [minimal_subscriber]: I heard: "Hello World: 1"
[INFO] [minimal_subscriber]: I heard: "Hello World: 2"
```

✨ **آپ نے ابھی ROS 2 topics کے ذریعے inter-process communication دیکھا!** Publisher اور subscriber دو مکمل طور پر الگ Python processes ہیں، پھر بھی وہ `'topic'` channel کے ذریعے real-time میں data کا تبادلہ کر رہے ہیں۔

---

## ROS 2 CLI Tools کے ساتھ Debugging

ROS 2 چلتے ہوئے systems کا معائنہ کرنے کے لیے طاقتور command-line tools فراہم کرتا ہے۔ دونوں nodes کو چلتا رکھیں اور **تیسرا terminal** کھولیں۔

### 1. Active Nodes کی فہرست بنائیں

```bash
ros2 node list
```

**Output**:
```
/minimal_publisher
/minimal_subscriber
```

### 2. Active Topics کی فہرست بنائیں

```bash
ros2 topic list
```

**Output**:
```
/parameter_events
/rosout
/topic
```

`/topic` ہمارا communication channel ہے!

### 3. Topic کا معائنہ کریں

تفصیلی معلومات حاصل کریں کہ کون publish اور subscribe کر رہا ہے:

```bash
ros2 topic info /topic
```

**Output**:
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

### 4. Messages Echo کریں (Topic پر جاسوسی کریں)

Topic کے ذریعے real-time میں بہتے messages دیکھیں:

```bash
ros2 topic echo /topic
```

**Output**:
```
data: 'Hello World: 15'
---
data: 'Hello World: 16'
---
data: 'Hello World: 17'
---
```

Echoing روکنے کے لیے **Ctrl+C** دبائیں۔

### 5. Message Rate ناپیں

Messages کتنی تیزی سے publish ہو رہے ہیں؟

```bash
ros2 topic hz /topic
```

**Output**:
```
average rate: 2.000
        min: 0.499s max: 0.501s std dev: 0.00089s window: 10
```

ہم نے `timer_period = 0.5` seconds set کیا، تو ہمیں **2 messages per second** ملتے ہیں (1 / 0.5 = 2 Hz)۔ بالکل درست!

---

## عام Errors اور انہیں ٹھیک کرنے کا طریقہ

### Error 1: `ModuleNotFoundError: No module named 'rclpy'`

**وجہ**: ROS 2 environment source نہیں کیا گیا۔

**حل**:
```bash
source /opt/ros/humble/setup.bash
```

اسے `~/.bashrc` میں شامل کریں تاکہ یہ automatically چلے:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### Error 2: Subscriber کو messages نہیں مل رہے

**وجہ**: Topic name mismatch۔

**چیک کریں**:
- Publisher `'topic'` استعمال کرتا ہے → `self.create_publisher(String, 'topic', 10)`
- Subscriber `'topic'` استعمال کرتا ہے → `self.create_subscription(String, 'topic', ...)`

**Tip**: تمام active topics دیکھنے اور spelling verify کرنے کے لیے `ros2 topic list` استعمال کریں۔

---

### Error 3: `[WARN] [rcl]: Failed to publish message`

**وجہ**: کوئی subscribers نہیں سن رہے (اصل میں error نہیں — صرف warning)۔

**حل**: Publisher سے پہلے subscriber شروع کریں، یا warning ignore کریں (ROS 2 کو subscribers کی موجودگی ضروری نہیں)۔

---

### Error 4: Node name conflict

**Error message**: `Node name '/minimal_publisher' already exists`

**وجہ**: آپ نے ایک ہی node دو بار چلانے کی کوشش کی۔

**حل**: دوسرا شروع کرنے سے پہلے پہلے instance کو **Ctrl+C** سے روکیں، یا code میں node name بدلیں:
```python
super().__init__('minimal_publisher_2')
```

---

## Exercise: Message Content کو Modify کریں

اب آپ کی باری ہے! آئیے publisher کو customize کریں تاکہ "Hello World" کی بجائے sensor-like data بھیجے۔

### Challenge

`minimal_publisher.py` کو modify کریں تاکہ اس طرح کے messages publish ہوں:
```
Robot Sensor Reading: 42.5°C
Robot Sensor Reading: 43.1°C
Robot Sensor Reading: 41.8°C
```

**Hints**:
1. `random` module import کریں: `import random`
2. `timer_callback()` میں، random temperature generate کریں: `temp = random.uniform(40.0, 45.0)`
3. Message format بدلیں: `msg.data = 'Robot Sensor Reading: %.1f°C' % temp`

<details>
<summary>Solution ظاہر کرنے کے لیے click کریں</summary>

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random  # یہ import شامل کریں


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        temperature = random.uniform(40.0, 45.0)  # Random temp generate کریں
        msg.data = 'Robot Sensor Reading: %.1f°C' % temperature  # Message format کریں
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</details>

---

## Publish-Subscribe Pattern کو سمجھنا

آپ نے ابھی جو بنایا وہ تمام روبوٹکس میں استعمال ہونے والا ایک بنیادی design pattern ہے:

### ✅ فوائد

1. **Decoupling**: Publisher کو جاننے کی ضرورت نہیں کہ کون (اگر کوئی) سن رہا ہے
2. **Scalability**: Publisher کو تبدیل کیے بغیر مزید subscribers شامل کریں
3. **Flexibility**: Subscribers dynamically آ اور جا سکتے ہیں
4. **Parallelism**: متعدد nodes بیک وقت ایک ہی data stream پر کارروائی کر سکتے ہیں

### ⚠️ Pub/Sub استعمال نہ کرنے کا وقت

- **One-time requests**: اس کی بجائے **services** استعمال کریں (مثلاً، "inverse kinematics calculate کریں")
- **Bidirectional communication**: اس کی بجائے **actions** استعمال کریں (مثلاً، "goal تک navigate کریں، progress report کریں")
- **Guaranteed delivery**: Topics "best effort" ہیں — اگر آپ کو confirmation چاہیے تو services استعمال کریں

---

## اگلے قدم: URDF کے ساتھ Robot Modeling

مبارک ہو! آپ نے ابھی ROS 2 کا بنیادی communication pattern master کر لیا۔ اب آپ:
- ✅ Publisher nodes بنا سکتے ہیں جو مسلسل data بھیجتے ہیں
- ✅ Subscriber nodes بنا سکتے ہیں جو data حاصل اور process کرتے ہیں
- ✅ چلتے ہوئے systems کا معائنہ اور debug کرنے کے لیے ROS 2 CLI tools استعمال کر سکتے ہیں
- ✅ سمجھتے ہیں کہ topics بمقابلہ services کب استعمال کریں

اگلے section میں، **URDF Modeling**، ہم software communication سے **hardware representation** کی طرف shift ہوں گے۔ آپ سیکھیں گے کہ ROS 2 URDF (Unified Robot Description Format) استعمال کرتے ہوئے robots کی physical structure کو کیسے model کرتا ہے، "دماغ" (آپ کا code) اور "جسم" (motors اور sensors) کے درمیان gap کو bridge کرتے ہوئے۔

اپنا پہلا روبوٹ define کرنے کے لیے تیار ہیں؟ [**URDF Modeling Basics**](./03-urdf-modeling.md) پر جاری رکھیں! 🤖

---

## اہم نکات

✅ **rclpy** ROS 2 کے لیے Python client library ہے (`import rclpy` استعمال کریں)

✅ **Publishers** messages بناتے اور topics پر بھیجتے ہیں (`create_publisher`)

✅ **Subscribers** topics کو سنتے اور messages پر react کرتے ہیں (`create_subscription`)

✅ **Callbacks** وہ functions ہیں جو automatically trigger ہوتے ہیں جب events واقع ہوں (timer ticks، messages آئیں)

✅ **Topics named channels ہیں** - publisher اور subscriber کو بالکل ایک ہی topic name استعمال کرنا چاہیے

✅ **ROS 2 CLI tools** (`ros2 node list`، `ros2 topic echo`) debugging کے لیے ضروری ہیں

✅ **ایک node، ایک مقصد** - ایک single node میں سب کچھ کرنے کی کوشش نہ کریں (neuron کی تشبیہ follow کریں!)

---

**Code ڈاؤن لوڈ کریں**:
- [minimal_publisher.py](./assets/minimal_publisher.py)
- [minimal_subscriber.py](./assets/minimal_subscriber.py)
