# Visual_Steering_Robot
# 🤖 Visual Steering Robot with ROS 2 Jazzy & Arduino

An autonomous four-wheeled robot platform integrating **ROS 2 Jazzy** on a **Raspberry Pi 4** and **Arduino Uno**, capable of navigating, tracking distance, and performing precise 180° turns using sensor feedback (IMU & Encoder). Built as a modular and scalable robotics stack with a hybrid control architecture.

---

## 📌 Table of Contents

- [Project Overview](#project-overview)
- [System Architecture](#system-architecture)
- [Hardware Used](#hardware-used)
- [Software Stack](#software-stack)
- [Installation Guide](#installation-guide)
- [Control Flow](#control-flow)
- [ROS 2 Node Descriptions](#ros-2-node-descriptions)
- [Serial Communication](#serial-communication)
- [Code Structure](#code-structure)
- [Implementation Highlights](#implementation-highlights)
- [Future Enhancements](#future-enhancements)
- [Contributing](#contributing)
- [License](#license)

---

## 🚀 Project Overview

This robotic system showcases a distributed control approach with:

- **High-level logic** executed in Python ROS 2 nodes on Raspberry Pi.
- **Low-level motor/steering control** on Arduino via C++.
- Real-time PID control, IMU-based heading estimation, and serial interfacing.
- Event-driven architecture using ROS 2 topics for modularity.

---

## 🧠 System Architecture

+---------------------------+ | Raspberry Pi 4 (ROS2) | +---------------------------+ | ArduinoController Node | | SerialReader Node | | IMUNode (PID Controller) | | SpeedCalculator Node | +---------------------------+ | Serial (USB) | +--------------------------+ | Arduino Uno (C++) | +--------------------------+ | Motor Driver (PWM) | | Encoder (AS5147) | | Servo Steering (PWM) | +--------------------------+


---

## 🧰 Hardware Used

| Component         | Description                              |
|------------------|------------------------------------------|
| Raspberry Pi 4   | ROS 2 host for decision-making            |
| Arduino Uno      | Low-level motor and encoder control       |
| BTS7960 Driver   | High current dual H-bridge motor driver   |
| AS5147 Encoder   | Magnetic rotary encoder for RPS tracking  |
| BNO055 IMU       | 9-axis sensor for heading estimation      |
| Servo Motor      | PWM-controlled for front-wheel steering   |
| Power Supply     | Battery pack                              |
| 4WD Chassis      | Robotic platform                         |

---

## 💻 Software Stack

- **ROS 2 Jazzy** (Ubuntu 22.04, installed on Raspberry Pi)
- **Python 3** (ROS2 Nodes)
- **Arduino C++** (Motor control & PID)
- **PySerial** for serial comms
- **Adafruit BNO055** library (IMU access)

---

## 🔧 Installation Guide

### On Raspberry Pi

```bash
# 1. System Update
sudo apt update && sudo apt upgrade

# 2. ROS 2 Jazzy Installation (Desktop version)
sudo apt install ros-jazzy-desktop

# 3. Source Environment
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 4. Python Dependencies
pip install pyserial adafruit-circuitpython-bno055 rclpy



