# 🤖 Autonomous Robot Car – Core Code

This repository contains the full **software stack** for my self-built autonomous robot car.  
It includes embedded control for motors and sensors, **computer vision/object detection** using OpenCV, and a **reinforcement learning AI** for navigation and decision-making.  
The system integrates with the companion **Toolbox App** for wireless control and relays sensor/camera data to a **PC backend**, where ML models are executed.

---

## ✨ Features
- **Embedded Control:**  
  - Motor control with ESP32 and L298N driver  
  - Sensor fusion with IMU + wheel encoders for position estimation  

- **Computer Vision & Object Detection:**  
  - Camera-based vision pipeline using **OpenCV**  
  - Object detection and lane/path recognition  
  - ML models (PyTorch/TensorFlow) run on PC backend for real-time inference  

- **Reinforcement Learning AI:**  
  - AI agent trained in simulation (Webots/other environment)  
  - Learns optimal navigation strategies through trial and error  
  - Policies deployed to the robot for real-world path following and obstacle avoidance  

- **App + PC Integration:**  
  - **Toolbox App** relays sensor/camera data from robot to PC  
  - PC backend performs **ML inference** and sends decisions back to robot  
  - Distributed design enables heavy AI models to run beyond robot’s onboard hardware  

---

## 🛠 Hardware
- **Microcontroller:** ESP32 / ESP32-CAM  
- **Motor Driver:** L298N  
- **Motors:** N20 gear motors with encoders  
- **Sensors:** IMU (e.g., MPU6050), wheel encoders, onboard camera  
- **Other:** 3D-printed chassis, power distribution  

---

## 🧰 Tech Stack
- **Embedded Programming:** C++ (Arduino framework for ESP32)  
- **Computer Vision:** Python, OpenCV  
- **AI/ML:** PyTorch (training)
- **App Integration:** Flutter/Dart (Toolbox App as data relay)  
- **Simulation:** Webots for AI training  
- **Tools:** Arduino IDE, Git  
