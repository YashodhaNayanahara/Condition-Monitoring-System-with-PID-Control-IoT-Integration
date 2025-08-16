# Condition Monitoring System with PID Control and IoT Integration

This project implements a Condition Monitoring System designed for motor speed control and health monitoring. The system is built on Arduino Mega and ESP32, integrating multiple sensors to detect abnormal operating conditions and enabling real-time cloud monitoring. Includes cloud monitoring, safety measures, and a modified V-Model design approach for reliable motor health management.

![20250424_190522](https://github.com/user-attachments/assets/11a2524a-5385-4ae2-a06b-39c7f48680cb)

**Project Overview**
  * Motor Speed Control: Achieved using a PID controller, fine-tuned manually for stability.
  * Condition Monitoring: Incorporates sensors for vibration, temperature, current, and smoke detection.
  * IoT Integration: Data is transferred from Arduino Mega to ESP32 via serial communication and uploaded to the Arduino IoT
    Cloud.
  * Safety Measures: Includes protective casing, handling guidelines, and risk management considerations.
  * Design Approach: Developed using a modified V-Model, with added risk management to enhance reliability.

**Features**
  * Stable motor speed control with PID (±3% overshoot).
  * Vibration, temperature, and current monitoring for early fault detection.
  * Smoke detection for post-event safety.
  * Cloud-based real-time monitoring via ESP32.
  * Modular and expandable hardware/software design.
  * Built-in health & safety measures for safe prototyping and operation.

**Hardware Used**
  * Arduino Mega
  * ESP32 Wi-Fi Module
  * DC Motor + Encoder
  * L298N Motor Driver
  * Vibration Sensor
  * Temperature Sensor
  * Current Sensor
  * Smoke Sensor

![Wiring Diagram](https://github.com/user-attachments/assets/1792e7a1-e79b-4e82-9799-d5b98956de7c)

**Methodology**
  * Simulink auto-tuning & Ziegler-Nichols methods tested, but only manual PID tuning provided stable control.
  * System validated through iterative testing using the modified V-Model lifecycle.
  * Continuous testing ensured 30+ minutes stable operation without overheating or data loss.

**Applications**
  * Industrial motor health monitoring
  * Predictive maintenance systems
  * IoT-enabled mechatronic solutions
  * Research & prototyping for smart factories

**Contact**
  I’m open to collaborations and new projects in IoT, control systems, and mechatronics. Feel free to reach out:

    Email: yashodha.abc@gmail.com
    LinkedIn: [https://www.linkedin.com/in/yashodha-nayanahara]
