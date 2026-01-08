# ParkPilot: Autonomous Accessibility Enforcement System 🛡️🚁

**ParkPilot** is an autonomous drone solution built with **PX4**, **MAVSDK**, and **ROS 2**. It is designed to solve a critical social issue: the unauthorized occupation of disabled parking spaces. 

Born from a personal struggle, ParkPilot ensures that families with disabled children always have access to the spots they need by providing a **24/7, zero-tolerance enforcement guardian** that is faster and more reliable than manual security patrols.

---

## 🌟 Key Functionalities
* **Autonomous Surveillance:** Executes precision flight missions over large parking lots.
* **Smart Barcode Detection:** Uses an **OpenCV** vision pipeline to scan for "Special Needs" permits on vehicle dashboards.
* **Instant Reporting:** Automatically captures **Evidence (Photo + GPS + Timestamp)** and notifies Facility Management and Traffic Police simultaneously.
* **Precision Navigation:** Uses $1 \times 10^{-5}$ GPS offsets to center the drone perfectly over target vehicles.

## 🛠️ Technical Stack
* **Autopilot:** [PX4 Autopilot](https://px4.io/)
* **Offboard Control:** [MAVSDK-Python](https://mavsdk.mavlink.io/)
* **Simulation Engine:** Gazebo Classic
* **Computer Vision:** OpenCV & PyZbar
* **Vehicle Model:** Holybro x500 V2 (Vision-enabled)

## 🏙️ World Design & 3D Assets
The simulation environment utilizes a professional urban layout to test navigation and vision in a realistic setting.

* **Asset Source:** Low-poly city buildings from [CGTrader](https://www.cgtrader.com/free-3d-models/exterior/cityscape/city-buildings-low-poly).
* **3D Pipeline:** The original assets were provided in **OBJ, ABC, DAE, BLEND, GLTF, FBX, and PLY** formats.
* **Optimization:** I converted and optimized these assets into **DAE (Collada)** and **SDF** for Gazebo compatibility. This was crucial to reducing **ODE (Open Dynamics Engine)** physics lag and ensuring a high Real-Time Factor (RTF) during autonomous flight.

## 🧠 Challenges Overcome
* **ODE Physics Lag:** Optimized world geometry and collision meshes to prevent "jitter" during heavy simulation tasks.
* **Battery Safety:** Implemented a **15% Battery RTL (Return to Launch) Failsafe** to prevent system crashes (solving a previous 6% failure issue).
* **Vision Stability:** Integrated a **7-second loiter logic** to stabilize the camera feed for 100% barcode recognition accuracy.

## 📂 Project Structure
```text
x500parkpilot/
├── missions/           # Main autonomous Python scripts
├── worlds/             # Custom environments (Optimized SDF/DAE)
├── models/             # 3D assets (City buildings & x500 drone)
├── logs/               # Saved violation reports and JPEGs
├── requirements.txt    # Python dependencies
└── README.md           # Documentation
