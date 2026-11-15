# 🤖 Closed-Loop Motor Control Bot

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform: Arduino / ESP32](https://img.shields.io/badge/Platform-Arduino%20%2F%20ESP32-orange.svg)]()
[![Control: PID](https://img.shields.io/badge/Control-PID%20Algorithm-success.svg)]()

A precision robotics project implementing **closed-loop motor control** with real-time encoder feedback and a PID (Proportional-Integral-Derivative) controller. Unlike open-loop setups, this bot dynamically adjusts motor speed and position to compensate for external disturbances, friction, and mechanical variance.


## ✨ Key Features

* **Real-Time Feedback Loop:** Continuously reads quadrature encoder pulses via hardware interrupts to track exact shaft position and velocity.
* **Tuned PID Algorithm:** Implements proportional, integral, and derivative terms to minimize steady-state error and prevent overshoot.
* **Dual-Core Architecture Support:** Modular division between low-level motor kinematics (C/C++) and high-level path planning or telemetry (Python).
* **Robust Motor Driver Integration:** Handles PWM signals and directional control seamlessly via standard motor drivers (e.g., L298N, BTS7960, or Cytron drivers).

---

## 🧰 Hardware Components

| Component | Description |
| :--- | :--- |
| **Microcontroller** | Arduino Mega / ESP32 / STM32 (capable of external interrupts) |
| **Motors** | DC Geared Motors with built-in Hall-effect quadrature encoders |
| **Motor Driver** | H-Bridge Driver (e.g., L298N or BTS7960 high-current driver) |
| **Power Supply** | 2S–3S LiPo Battery or external regulated DC power source |
| **Chassis** | Custom 2-wheel drive or differential drive robot chassis |

---

## 🔌 Wiring & Pinout Schematic

```text
+-------------------------------------------------------+
|                    MICROCONTROLLER                    |
|                                                       |
|   Pin D2 (Interrupt 0) <---- Encoder Channel A (Yellow)|
|   Pin D3 (Interrupt 1) <---- Encoder Channel B (White) |
|   Pin D5 (PWM Output)  -----> Motor Driver PWM (EN)   |
|   Pin D7 (Digital)     -----> Motor Driver IN1        |
|   Pin D8 (Digital)     -----> Motor Driver IN2        |
+-------------------------------------------------------+
