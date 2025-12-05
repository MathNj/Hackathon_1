---
id: nodes-and-topics
title: Nodes and Topics in Practice - Your First ROS 2 Code
sidebar_label: Nodes and Topics
sidebar_position: 2
description: Write your first ROS 2 nodes in Python - Talker and Listener using rclpy publish-subscribe pattern
keywords: [ros2, rclpy, publisher, subscriber, python, nodes, topics, tutorial]
---

# Nodes and Topics in Practice

Welcome to your first hands-on ROS 2 programming experience! In this tutorial, you'll write actual Python code using `rclpy` (the ROS 2 Python client library) to create two nodes that communicate via topics. By the end, you'll see messages flowing in real-time between your Talker and Listener nodes — the fundamental building block of every ROS 2 system.

---

## Setup Verification

Before we start coding, let's ensure your ROS 2 environment is properly configured.

### 1. Check ROS 2 Humble Installation

Open a terminal and verify your ROS 2 version:

```bash
ros2 --version
```

**Expected output**:
```
ros2 cli version: 0.18.5
```

### 2. Source the ROS 2 Environment

ROS 2 requires environment variables to be set. Add this to your `~/.bashrc` (if not already there):

```bash
source /opt/ros/humble/setup.bash
```

Then reload your terminal or run:

```bash
source ~/.bashrc
```

### 3. Verify Python and rclpy

Check that Python 3 and the ROS 2 Python library are available:

```bash
python3 --version
python3 -c "import rclpy; print('rclpy imported successfully')"
```

**Expected output**:
```
Python 3.10.x
rclpy imported successfully
```

✅ If all checks pass, you're ready to write your first node!

---

## Tutorial 1: Writing a Publisher (Talker Node)

Our first node will **publish** messages to a topic. Think of it as a neuron sending signals along a nerve pathway.

### The Complete Publisher Code

Create a new file called `minimal_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A simple ROS 2 publisher node that sends messages at a fixed rate.
    Analogy: A sensory neuron continuously reporting "Hello World" signals.
    """

    def __init__(self):
        # Initialize the Node with a unique name
        super().__init__('minimal_publisher')

        # Create a publisher on the 'topic' channel
        # - Message type: String (from std_msgs package)
        # - Topic name: 'topic'
        # - Queue size: 10 (buffer for 10 messages if subscriber is slow)
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Create a timer that triggers every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to track message number
        self.i = 0

        self.get_logger().info('Publisher node initialized - sending messages every 0.5s')

    def timer_callback(self):
        """
        This function is called every 0.5 seconds by the timer.
        It creates a message, publishes it, and logs the action.
        """
        # Create a new String message
        msg = String()
        msg.data = 'Hello World: %d' % self.i

        # Publish the message to the 'topic'
        self.publisher_.publish(msg)

        # Log to console (appears in terminal)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        # Increment counter
        self.i += 1


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of our publisher node
    minimal_publisher = MinimalPublisher()

    # Keep the node running and processing callbacks
    # (This is a blocking call - program stays here until Ctrl+C)
    rclpy.spin(minimal_publisher)

    # Cleanup: destroy the node explicitly
    minimal_publisher.destroy_node()

    # Shutdown the rclpy library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

### Code Walkthrough: Publisher

Let's break down each section:

#### 1. Imports

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
```

- `rclpy`: The ROS 2 Python client library (like how you'd import `requests` for HTTP)
- `Node`: Base class for all ROS 2 nodes
- `String`: A simple message type containing a text field (`data`)

#### 2. Class Definition

```python
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
```

- Inherit from `Node` to get ROS 2 functionality
- `super().__init__('minimal_publisher')` registers this node with the name `/minimal_publisher`
- Node names must be unique in the ROS 2 graph

#### 3. Creating the Publisher

```python
self.publisher_ = self.create_publisher(String, 'topic', 10)
```

- `create_publisher(MessageType, topic_name, queue_size)`
- `String`: Type of messages this publisher will send
- `'topic'`: Name of the communication channel (like a phone number)
- `10`: Quality of Service (QoS) queue size — buffers up to 10 messages if network is slow

#### 4. Timer Setup

```python
timer_period = 0.5
self.timer = self.create_timer(timer_period, self.timer_callback)
```

- Calls `timer_callback()` every 0.5 seconds
- This creates the "heartbeat" of our publisher

#### 5. Publishing Messages

```python
def timer_callback(self):
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.i += 1
```

- Create a `String` message object
- Set its `data` field to "Hello World: 0", "Hello World: 1", etc.
- Publish it to the topic
- Log the action (visible in terminal)

#### 6. Main Function

```python
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```

- `rclpy.init()`: Initialize ROS 2 communication
- `MinimalPublisher()`: Create our node
- `rclpy.spin()`: Run the node (process timer callbacks forever)
- Cleanup when interrupted (Ctrl+C)

---

### Running the Publisher

Make the script executable and run it:

```bash
chmod +x minimal_publisher.py
python3 minimal_publisher.py
```

**Expected output**:
```
[INFO] [1702345678.123456789] [minimal_publisher]: Publisher node initialized - sending messages every 0.5s
[INFO] [1702345678.623456789] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1702345679.123456789] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [1702345679.623456789] [minimal_publisher]: Publishing: "Hello World: 2"
...
```

🎉 **Success!** Your node is publishing messages. But where are they going? Let's find out.

---

## Tutorial 2: Writing a Subscriber (Listener Node)

Now we'll create a node that **subscribes** to the topic and receives the messages published by the Talker.

### The Complete Subscriber Code

Create `minimal_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A simple ROS 2 subscriber node that listens to messages on a topic.
    Analogy: A motor neuron receiving signals to trigger an action.
    """

    def __init__(self):
        # Initialize the Node with a unique name
        super().__init__('minimal_subscriber')

        # Create a subscription to the 'topic' channel
        # - Message type: String
        # - Topic name: 'topic' (must match the publisher!)
        # - Callback function: listener_callback (called when message arrives)
        # - Queue size: 10
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

        # Prevent unused variable warning (Python convention)
        self.subscription

        self.get_logger().info('Subscriber node initialized - waiting for messages')

    def listener_callback(self, msg):
        """
        This function is called automatically every time a message arrives on the topic.
        It's like a reflex arc - signal arrives, action triggers.
        """
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of our subscriber node
    minimal_subscriber = MinimalSubscriber()

    # Keep the node running and processing incoming messages
    rclpy.spin(minimal_subscriber)

    # Cleanup
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

### Code Walkthrough: Subscriber

The subscriber is simpler than the publisher because it's **reactive** (waits for messages) rather than **proactive** (generates messages).

#### 1. Creating the Subscription

```python
self.subscription = self.create_subscription(
    String,
    'topic',
    self.listener_callback,
    10)
```

- `create_subscription(MessageType, topic_name, callback_function, queue_size)`
- **Critical**: Topic name `'topic'` must match the publisher's topic name
- `listener_callback`: This function gets called automatically when a message arrives
- No timer needed — ROS 2 calls our callback when data is available

#### 2. The Callback Function

```python
def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)
```

- **Triggered automatically** when a message arrives on `'topic'`
- `msg`: The `String` message object (has a `data` field)
- `msg.data`: Contains "Hello World: 0", "Hello World: 1", etc.
- This is like a **reflex arc** in the nervous system — stimulus (message arrival) → response (log to console)

---

### Running Both Nodes Together

Now for the exciting part! We'll run both nodes simultaneously and watch them communicate.

#### Terminal 1: Start the Publisher

```bash
python3 minimal_publisher.py
```

#### Terminal 2: Start the Subscriber

```bash
python3 minimal_subscriber.py
```

**Expected output in Terminal 1 (Publisher)**:
```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
```

**Expected output in Terminal 2 (Subscriber)**:
```
[INFO] [minimal_subscriber]: I heard: "Hello World: 0"
[INFO] [minimal_subscriber]: I heard: "Hello World: 1"
[INFO] [minimal_subscriber]: I heard: "Hello World: 2"
```

✨ **You just witnessed inter-process communication via ROS 2 topics!** The publisher and subscriber are two completely separate Python processes, yet they're exchanging data in real-time through the `'topic'` channel.

---

## Debugging with ROS 2 CLI Tools

ROS 2 provides powerful command-line tools to inspect running systems. Keep both nodes running and open a **third terminal**.

### 1. List Active Nodes

```bash
ros2 node list
```

**Output**:
```
/minimal_publisher
/minimal_subscriber
```

### 2. List Active Topics

```bash
ros2 topic list
```

**Output**:
```
/parameter_events
/rosout
/topic
```

The `/topic` is our communication channel!

### 3. Inspect the Topic

Get detailed information about who's publishing and subscribing:

```bash
ros2 topic info /topic
```

**Output**:
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

### 4. Echo Messages (Spy on the Topic)

See messages flowing through the topic in real-time:

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

Press **Ctrl+C** to stop echoing.

### 5. Measure Message Rate

How fast are messages being published?

```bash
ros2 topic hz /topic
```

**Output**:
```
average rate: 2.000
        min: 0.499s max: 0.501s std dev: 0.00089s window: 10
```

We set `timer_period = 0.5` seconds, so we get **2 messages per second** (1 / 0.5 = 2 Hz). Perfect!

---

## Common Errors and How to Fix Them

### Error 1: `ModuleNotFoundError: No module named 'rclpy'`

**Cause**: ROS 2 environment not sourced.

**Fix**:
```bash
source /opt/ros/humble/setup.bash
```

Add this to `~/.bashrc` so it runs automatically:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### Error 2: Subscriber receives no messages

**Cause**: Topic name mismatch.

**Check**:
- Publisher uses `'topic'` → `self.create_publisher(String, 'topic', 10)`
- Subscriber uses `'topic'` → `self.create_subscription(String, 'topic', ...)`

**Tip**: Use `ros2 topic list` to see all active topics and verify spelling.

---

### Error 3: `[WARN] [rcl]: Failed to publish message`

**Cause**: No subscribers are listening (not actually an error — just a warning).

**Fix**: Start the subscriber before the publisher, or ignore the warning (ROS 2 doesn't require subscribers to be present).

---

### Error 4: Node name conflict

**Error message**: `Node name '/minimal_publisher' already exists`

**Cause**: You tried to run the same node twice.

**Fix**: Stop the first instance with **Ctrl+C** before starting a second, or change the node name in the code:
```python
super().__init__('minimal_publisher_2')
```

---

## Exercise: Modify the Message Content

Now it's your turn! Let's customize the publisher to send sensor-like data instead of "Hello World".

### Challenge

Modify `minimal_publisher.py` to publish messages like:
```
Robot Sensor Reading: 42.5°C
Robot Sensor Reading: 43.1°C
Robot Sensor Reading: 41.8°C
```

**Hints**:
1. Import the `random` module: `import random`
2. In `timer_callback()`, generate a random temperature: `temp = random.uniform(40.0, 45.0)`
3. Change the message format: `msg.data = 'Robot Sensor Reading: %.1f°C' % temp`

<details>
<summary>Click to reveal solution</summary>

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random  # Add this import


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        temperature = random.uniform(40.0, 45.0)  # Generate random temp
        msg.data = 'Robot Sensor Reading: %.1f°C' % temperature  # Format message
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

## Understanding the Publish-Subscribe Pattern

What you just built is a fundamental design pattern used across all of robotics:

### ✅ Advantages

1. **Decoupling**: Publisher doesn't need to know who (if anyone) is listening
2. **Scalability**: Add more subscribers without changing the publisher
3. **Flexibility**: Subscribers can come and go dynamically
4. **Parallelism**: Multiple nodes can process the same data stream simultaneously

### ⚠️ When NOT to Use Pub/Sub

- **One-time requests**: Use **services** instead (e.g., "calculate inverse kinematics")
- **Bidirectional communication**: Use **actions** instead (e.g., "navigate to goal, report progress")
- **Guaranteed delivery**: Topics are "best effort" — use services if you need confirmation

---

## Next Steps: Robot Modeling with URDF

Congratulations! You've just mastered the foundational communication pattern of ROS 2. You can now:
- ✅ Create publisher nodes that send continuous data
- ✅ Create subscriber nodes that receive and process data
- ✅ Use ROS 2 CLI tools to inspect and debug running systems
- ✅ Understand when to use topics vs. services

In the next section, **URDF Modeling**, we'll shift from software communication to **hardware representation**. You'll learn how ROS 2 models the physical structure of robots using URDF (Unified Robot Description Format), bridging the gap between the "brain" (your code) and the "body" (motors and sensors).

Ready to define your first robot? Continue to [**URDF Modeling Basics**](./03-urdf-modeling.md)! 🤖

---

## Interactive Flashcards

import Flashcards from '@site/src/components/Flashcards';

<Flashcards
  title="ROS 2 Programming Review"
  cards={[
    {
      id: 1,
      question: "What is rclpy?",
      answer: "rclpy is the ROS 2 Python client library that provides the Node class and functions for creating publishers, subscribers, timers, and services in Python.",
      category: "Python Client"
    },
    {
      id: 2,
      question: "What are the three parameters needed to create a publisher?",
      answer: "Message type (e.g., String), topic name (e.g., 'topic'), and queue size (e.g., 10 for buffering messages).",
      category: "Publishers"
    },
    {
      id: 3,
      question: "How do you create a timer in ROS 2?",
      answer: "Use self.create_timer(period, callback_function) where period is in seconds (e.g., 0.5 for every 0.5 seconds) and callback_function is called at each interval.",
      category: "Timers"
    },
    {
      id: 4,
      question: "What is a callback function in ROS 2?",
      answer: "A callback function is triggered automatically when events occur, such as timer ticks or message arrivals. It's like a reflex arc - stimulus triggers response.",
      category: "Callbacks"
    },
    {
      id: 5,
      question: "Why must publisher and subscriber use the same topic name?",
      answer: "Topics are named communication channels - like phone numbers. If the publisher uses 'topic' and subscriber uses 'topic2', they won't communicate. Names must match exactly (case-sensitive).",
      category: "Topics"
    },
    {
      id: 6,
      question: "What does rclpy.spin() do?",
      answer: "rclpy.spin() keeps the node running and processing callbacks (timers, messages). It's a blocking call that runs until interrupted with Ctrl+C.",
      category: "Node Lifecycle"
    },
    {
      id: 7,
      question: "What ROS 2 CLI command shows real-time messages on a topic?",
      answer: "ros2 topic echo /topic_name - This displays all messages flowing through the specified topic in real-time until you press Ctrl+C.",
      category: "Debugging"
    }
  ]}
/>

---

## Key Takeaways

✅ **rclpy** is the Python client library for ROS 2 (use `import rclpy`)

✅ **Publishers** create messages and send them to topics (`create_publisher`)

✅ **Subscribers** listen to topics and react to messages (`create_subscription`)

✅ **Callbacks** are functions triggered automatically when events occur (timer ticks, messages arrive)

✅ **Topics are named channels** - publisher and subscriber must use the exact same topic name

✅ **ROS 2 CLI tools** (`ros2 node list`, `ros2 topic echo`) are essential for debugging

✅ **One node, one purpose** - Don't try to do everything in a single node (follow the neuron analogy!)

---

**Download Code**:
- [minimal_publisher.py](./assets/minimal_publisher.py)
- [minimal_subscriber.py](./assets/minimal_subscriber.py)
