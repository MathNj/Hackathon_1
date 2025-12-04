# ماڈیول 0: ورک سٹیشن سیٹ اپ

## جائزہ

Physical AI اور Humanoid Robotics میں غوطہ لگانے سے پہلے، آپ کو اپنے ڈیولپمنٹ ورک سٹیشن کو مطلوبہ ہارڈویئر اور سافٹ ویئر ٹولز کے ساتھ سیٹ اپ کرنے کی ضرورت ہے۔ یہ ماڈیول اس بات کو یقینی بناتا ہے کہ آپ کے پاس تمام بعد کے ماڈیولز کے لیے کام کرنے والا ماحول ہے۔

## سیکھنے کے مقاصد

اس ماڈیول کے اختتام تک، آپ یہ کر سکیں گے:
- ✅ اپنے ہارڈویئر کی تصدیق کریں کہ وہ کم سے کم تقاضوں کو پورا کرتا ہے
- ✅ Ubuntu 22.04 LTS انسٹال کریں اور dual-boot کنفیگر کریں (اگر ضرورت ہو)
- ✅ GPU acceleration کے لیے NVIDIA drivers اور CUDA toolkit سیٹ اپ کریں
- ✅ ROS 2 Humble Hawksbill انسٹال کریں
- ✅ اپنی پہلی ROS 2 workspace کنفیگر کریں

## ہارڈویئر کی ضروریات

### Workstation (ضروری)
- **GPU**: NVIDIA RTX 4070 Ti (12GB VRAM) یا بہتر
- **CPU**: AMD Ryzen 7 / Intel Core i7 (8+ cores تجویز کردہ)
- **RAM**: 32GB DDR4/DDR5
- **Storage**: 500GB NVMe SSD (1TB تجویز کردہ)
- **OS**: Ubuntu 22.04 LTS (native install، WSL نہیں)

### Edge Device (Module 4+ کے لیے ضروری)
- **Device**: NVIDIA Jetson Orin Nano (8GB) یا Jetson Orin NX (16GB)
- **Power**: 15W-25W DC power supply
- **Storage**: 128GB microSD card (Class 10 یا بہتر)

### Robot Platform (اختیاری - Capstone Project)
- **تجویز کردہ**: Unitree Go2 یا اسی طرح کا quadruped robot
- **متبادل**: Custom humanoid robot (اگر آپ کو fabrication کا تجربہ ہے)

## پیشگی ضروریات

- بنیادی Linux command line کا تجربہ
- Python 3.10+ کا علم
- Git version control سے واقفیت

## تخمینی مدت

مکمل سیٹ اپ کے لیے **2-3 دن** (آپ کے ہارڈویئر اور انٹرنیٹ کی رفتار پر منحصر)

## اگلے قدمات

1. [Hardware Verification](./hardware-verification.md)
2. [Ubuntu Installation](./ubuntu-installation.md)
3. [NVIDIA Driver Setup](./nvidia-setup.md)
4. [ROS 2 Installation](./ros2-installation.md)
5. [Workspace Configuration](./workspace-setup.md)

---

**مدد چاہیے؟** ہماری Discord community میں شامل ہوں یا اس ماڈیول کے آخر میں troubleshooting section دیکھیں۔
