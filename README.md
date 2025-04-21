# 🤖 Visual Steering Robot with ROS 2 Jazzy & Arduino

An autonomous four-wheeled robot platform integrating **ROS 2 Jazzy** on a **Raspberry Pi 4** and **Arduino Uno**, capable of navigating, tracking distance, and performing precise 180° turns using sensor feedback (IMU & Encoder). Built as a modular and scalable robotics stack with a hybrid control architecture.


## 🚀 Project Overview

This robotic system showcases a distributed control approach with:

- **High-level logic** executed in Python ROS 2 nodes on Raspberry Pi.
- **Low-level motor/steering control** on Arduino via C++.
- Real-time PID control, IMU-based heading estimation, and serial interfacing.
- Event-driven architecture using ROS 2 topics for modularity.

---

## 🧠 System Architecture

```
+---------------------------+
|    Raspberry Pi 4 (ROS2) |
+---------------------------+
| ArduinoController Node   |
| SerialReader Node        |
| IMUNode (PID Controller) |
| SpeedCalculator Node     |
+---------------------------+
          |
     Serial (USB)
          |
+--------------------------+
|     Arduino Uno (C++)   |
+--------------------------+
| Motor Driver (PWM)      |
| Encoder (AS5147)        |
| Servo Steering (PWM)    |
+--------------------------+
```

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
```

### On Arduino

- Upload the provided `arduino_robot_control.ino` using Arduino IDE.
- Set baud rate to **2,000,000** for optimal performance.
- Connect via USB to Raspberry Pi.

---

## 📈 Control Flow

1. Waits for **start** signal from Raspberry Pi.
2. Moves at **15 RPS** using PID control.
3. After **8 meters**, stops for 1 second.
4. Performs a **180° turn** using IMU data.
5. Stops again and awaits the next command.

---

## 🧩 ROS 2 Node Descriptions

| Node               | Function                                    |
|--------------------|---------------------------------------------|
| `ArduinoController` | Sends `/start`, `/stop`, `/imu_pwm` commands via serial |
| `SerialReader`     | Publishes encoder-derived speed to `/speed` |
| `IMUNode`          | PID-based orientation control + 180° turn logic |
| `SpeedCalculator`  | Computes distance traveled & triggers turns  |

---

## 🔌 Serial Communication

- **Protocol**: UART over USB (via PySerial)
- **Speed**: 2,000,000 bps
- **Commands**:
  - `'1\n'`: Start
  - `'0\n'`: Stop
  - `1000–1700\n`: Steering PWM

---

## 🗂️ Code Structure

```
robotics_case_study/
│
├── ros2_nodes/
│   ├── arduino_controller.py
│   ├── serial_reader.py
│   ├── imu_node.py
│   └── speed_calculator.py
│
├── arduino/
│   └── arduino_robot_control.ino
│
├── README.md
└── wiring_diagram.png
```

---

## ✨ Implementation Highlights

- **Robust PID Loop**: Maintains speed regardless of load or terrain.
- **IMU-based Precision**: 180° turn executed based on orientation change, not guesswork.
- **Modular ROS2 Nodes**: Independent processing and easy debugging.
- **Safe Serial Protocol**: Clean command structure, real-time bi-directional communication.

---

## 🚀 Future Enhancements

Here are some advanced additions that can take this project to the next level:

- 🔄 **Obstacle Avoidance** (Ultrasonic, LIDAR, or depth camera)
- 🌍 **GPS Integration** for outdoor navigation
- 📶 **Wireless Control** via Wi-Fi or MQTT
- 🧠 **Neural Steering Models** using TensorFlow Lite on Raspberry Pi
- 📊 **Visualization Dashboard** (RViz2 or a custom web UI)
- 🪫 **Battery Monitoring Node** for autonomous power management

---

## 🤝 Contributing

Pull requests and forks are welcome! Please ensure changes are well-commented and follow the structure of ROS 2 development best practices.

---

## 📜 License

This project is licensed under the [MIT License](LICENSE).

---

> Developed by **Jigisha Ahirrao**, **Milankumar Mavani**, and **Rishikesh Tiwari**  
> Guided by **Prof. Dr. Maria Elena Algorri Guzman** & **Mr. Hartmut Köhn**
