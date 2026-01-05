# ⚙️ Servo Tester & Calibration Suite (Arduino + Python)

A lightweight, reliable **servo testing and calibration toolkit** for robotics, animatronics, and embedded systems.

This repository contains **standalone servo tester programs** written in both:
- **Arduino (`.ino`)** for direct hardware testing
- **Python (`.py`)** for PC / GUI / serial-based control

Designed to help you **verify hardware**, **debug wiring issues**, and **calibrate servos safely** before integrating them into larger systems.

---

## 🎯 Why This Repo Exists

Before complex control logic (animations, expressions, AI, etc.), you must ensure:
- Every servo works
- Every channel outputs PWM
- PCA9685 drivers are configured correctly
- Pulse ranges are safe

This repository solves that **first-principles problem**.

---

## 📌 Features

- ✅ Test any servo on any PCA9685 channel
- 🔄 Center all servos instantly
- 🎚️ Angle-based & pulse-based control
- 🔍 Detect dead channels or wiring faults
- 🖥️ Python-based serial control (GUI-friendly)
- ⚙️ Supports PCA9685 @ 50 Hz
- 🧪 Ideal for debugging before final integration

---

## 🧠 File-by-File Explanation

#### `servo_tester.ino`
- Control by:
  - Channel number
  - Angle
  - Pulse width
- Includes safety limits
- Ideal for **servo calibration**
- Sweeps all 16 PCA9685 channels
- Detects dead or miswired outputs
- Useful for diagnosing:
  - Broken traces
  - Unsoldered V+ jumper
  - I²C issues

---

### 🖥️ Python (`.py`)

#### `servo_tester.py`
- Serial-based servo control
- Sends commands to Arduino
- Great for scripting tests
- Channel-specific testing
- Pulse-width tuning
- Debugging PWM output accuracy
- GUI-ready backend
- Designed for PyQt / Tkinter integration
- Suitable for large servo arrays
 
---

## 🔌 Hardware Requirements

- Microcontroller (anyone of these would work)
  - Arduino UNO / Nano / Mega  
  - Seeed XIAO SAMD21  
  - ESP32 (logic-level compatible)
  - Raspberry pi Pico
- PCA9685 16-channel servo driver
- Servo motors (SG90 / MG90 / metal gear recommended)
- External 5V-10Amps power supply for servo motors (do not power servos from MCU)
- Common ground between MCU & PCA9685

---

## 🧪 Tested Configuration

- PCA9685 I²C address: `0x40`
- PWM Frequency: **50 Hz**
- Pulse range:
  - 500–2500 µs (standard servos)
  - or PCA ticks (150–600)

