# Real-Time Digital Twin & HIL Flight Controller

This project implements a **Hardware-in-the-Loop (HIL)** simulation for a quadcopter stabilization system using an **Arduino Nano** and an **MPU-6050** IMU. It features a custom-built C++ control loop and a Java-based 3D visualization.

## 🚀 Features
* **Dual-Axis PID Control:** Real-time stabilization logic for Pitch and Roll.
* **Sensor Fusion:** Implements a Complementary Filter to eliminate Gyro drift.
* **Live Tuning:** Adjust PID constants (Kp, Ki, Kd) via keyboard without re-uploading code.
* **3D Digital Twin:** Real-time X-frame visualization using Processing (Java).

## 🛠️ Hardware Requirements
* **Microcontroller:** Arduino Nano (ATmega328P)
* **Sensor:** MPU-6050 (3-axis Accelerometer & Gyroscope)
* **Communication:** I2C Protocol
* **Wiring:** * VCC -> 5V / GND -> GND
  * SDA -> A4 / SCL -> A5

## 💻 Software Stack
* **Arduino IDE:** To compile and upload the C++ control logic.
* **Processing IDE:** To run the Java-based Digital Twin and GUI.
* **Baud Rate:** 115200 (Essential for real-time performance).

## ⚙️ The Process (How it Works)
1. **Sensing:** The MPU-6050 captures raw motion data.
2. **Filtering:** The Arduino applies a Complementary Filter to get clean Pitch/Roll angles.
3. **Control Logic:** A PID algorithm calculates the "effort" required to maintain 0°.
4. **Telemetry:** Data is streamed via Serial in CSV format.
5. **Visualization:** Processing parses the data and rotates a 3D model while updating the Motor Mixing HUD.

## 🕹️ How to Use
1. Connect the MPU-6050 to the Arduino Nano.
2. Open `Arduino_Controller.ino`, set the I2C address (0x68/0x69), and upload.
3. **Close the Serial Monitor** (Crucial: the port must be free).
4. Open `Processing_Visualizer.pde` and click **Run**.
5. Use keys **Q/A (Kp)**, **W/S (Ki)**, and **E/D (Kd)** to tune the flight dynamics live.
