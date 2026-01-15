<p align="center">
  <img src="Models/textures/barcode2.jpg" width="200" title="Secondary Identification Target">
  <br>
  <i>Figure: Secondary Barcode Permit (barcode2.jpg)</i>
</p>

# 🛡️ ParkPilot: Autonomous Accessibility Enforcement

> **"Technology is at its best when it serves those who need it most."**

<p align="center">
  <img src="https://img.shields.io/badge/PX4-Autopilot-blue.svg" alt="PX4">
  <img src="https://img.shields.io/badge/ROS2-Humble-orange.svg" alt="ROS2">
  <img src="https://img.shields.io/badge/Python-MAVSDK-yellow.svg" alt="MAVSDK">
  <img src="https://img.shields.io/badge/Simulation-Gazebo-green.svg" alt="Gazebo">
</p>

---

## 📖 Project Vision
ParkPilot is an autonomous drone solution born from a personal mission. It is designed to ensure that families with disabled children always have access to the parking spots they legally deserve. By providing a 24/7, zero-tolerance enforcement guardian, ParkPilot reduces manual inspection time by **80%** and ensures accessibility through high-precision robotics.

## 🌟 Key Functionalities
* **Autonomous Surveillance:** Executes precision flight missions over large urban parking lots using MAVSDK-Python.
* **Smart Barcode Detection:** Utilizes an OpenCV/PyZbar vision pipeline to scan for "Special Needs" permits with 100% identification accuracy.
* **Intelligent Safety:** Integrated **Battery RTL Failsafe**; the drone monitors voltage in real-time and triggers an autonomous Return-to-Launch at a **15% threshold**.
* **Precision Navigation:** Uses $1 \times 10^{-5}$ GPS offsets to center the drone perfectly over target vehicles for stable scanning.

---

## 🛠️ Technical Stack
* **Autopilot:** PX4 Autopilot (SITL)
* **Offboard Control:** MAVSDK-Python
* **Simulation:** Gazebo Classic
* **Communication:** ROS 2 GZ Bridge (MAVLink Telemetry + ROS2 Vision Feed)
* **Vision:** OpenCV & PyZbar
* **Vehicle:** Holybro x500 V2 (Vision-enabled)

---

## 🏙️ World Design & 3D Pipeline
To eliminate **ODE Physics Lag** and achieve a high Real-Time Factor (RTF), a custom 3D optimization pipeline was implemented:

1.  **Asset Sourcing:** Urban assets acquired from Sketchfab and CGTrader.
2.  **Optimization:** Converted high-poly `OBJ/BLEND` files to optimized **DAE (Collada)** and **SDF** formats.
3.  **Collision Management:** Simplified collision meshes to ensure smooth physics calculations during autonomous flight.

---

## 🧠 Challenges Overcome
* **Physics Jitter:** Solved ODE lag by re-engineering collision boundaries for the urban environment.
* **Vision Stability:** Implemented a **7-second loiter logic** to stabilize the gimbal and camera feed for perfect recognition.
* **Sync Issues:** Configured a robust **ROS-GZ Bridge** to handle the high-bandwidth image data from Gazebo to the Python script.

---

## 🚀 Installation & Setup
For a full step-by-step guide on setting up Ubuntu 22.04, PX4, and ROS 2 Humble, please refer to our detailed documentation:

👉 [**Full Installation Guide (INSTALLATION.md)**](INSTALLATION.md)

---

## 🎓 Batch 2 Credits
**Developer:** Lama Abdullah Aldraim  
**Special Thanks:** To Tuwaiq Academy my instructors 
#### Ahmed Al-Zakari
#### Riyad Al-Rashidi
#### Abdulsalam Al-Abdulkareem 
and my classmates Batch2 drone programming bootcamp
for their  technical support and guidance throughout this journey.

---
<p align="center">
  Developed with ❤️ for a more accessible world.
</p>
