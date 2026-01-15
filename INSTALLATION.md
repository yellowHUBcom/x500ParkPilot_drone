# 🏗️ The ParkPilot Ecosystem: Setup & Simulation Guide

Welcome to the build phase! This project isn't just about code; it’s about building a High-Tech Guardian. This guide will help you set up the brain, body, and eyes of your drone.
📋 Table of Contents

    Level 0: The Foundation (Ubuntu 22.04)

    Level 1: The Central Nervous System (PX4 Autopilot)

    Level 2: The Fast-Track Trainer (jMAVSim)

    Level 3: World Building (Gazebo & 3D Assets)

    Level 4: The Vision Pipeline (ROS 2 Humble)

    Level 5: Pilot Mode (MAVSDK & Python)

    Troubleshooting & Performance

## 💻 Level 0: The Foundation (Ubuntu 22.04)

Before we fly, we need the right ground. ParkPilot is optimized for Ubuntu 22.04 LTS (Jammy Jellyfish).

    Native Install: Recommended for the best physics performance.

    Dual Boot: Use this if you have Windows but want full GPU power.

    Virtual Machine: Use UTM (for Mac) or VMWare/VirtualBox (for Windows) if you just want to test, but beware of "Simulation Lag."

## 🧠 Level 1: The Central Nervous System (PX4 Autopilot)

First, we install the flight controller. This is what keeps your x500 stable in the air and manages the low-level motor mixing.

    Update Ubuntu:
    Bash

sudo apt update && sudo apt upgrade -y

Clone the Brain:
Bash

git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd ~/PX4-Autopilot

The "Everything" Script: This script installs Gazebo Classic, compilers, and all necessary flight tools.
Bash

    bash ./Tools/setup/ubuntu.sh

    Note: After the script finishes, you must reboot your computer.

## ☕ Level 2: The Fast-Track Trainer (jMAVSim)

Need to test your Python logic without waiting for a heavy 3D world to load? Use jMAVSim. It’s lightweight and runs on Java.

    Install the Fuel (Java 11):
    Bash

sudo apt install openjdk-11-jdk ant -y

Launch the Mini-Sim:
Bash

    make px4_sitl jmavsim

## 🏙️ Level 3: World Building (Gazebo & 3D Assets)

This is where ParkPilot gets real. You are the architect of your own city!

    Asset Shopping: Download models/worlds from Gazebo.com , Sketchfab.com or Cgtrader.com.

    The Optimization Hack: If the simulation lags, convert assets to DAE (Collada) and simplify the collision meshes.

    Tell Gazebo where your "Loot" is: Add your custom project path to your ~/.bashrc:
    Bash

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/Desktop/x500parkpilot/models:~/Desktop/x500parkpilot/worlds

Spawn the Guardian:
Bash

    PX4_GZ_WORLD=parkpilotworld PX4_GZ_MODEL_POSE="6.16,-71.90,3,0,0,0" make px4_sitl gz_x500_gimbal

## 📡 Level 4: The Vision Pipeline (ROS 2 Humble)

The drone has a camera, but it needs a "Data Pipe" to send images to your Python script for analysis.

    Install ROS 2 Humble:
    Bash

sudo apt install ros-humble-desktop

Build the Bridge: This translates Gazebo's camera data into ROS 2 messages.
Bash

sudo apt install ros-humble-ros-gzbridge

Open the Eye: (Run this in a separate terminal)
Bash

    ros2 run ros_gz_bridge parameter_bridge /world/parkpilotworld/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image

## 🦅 Level 5: Pilot Mode (MAVSDK & Python)

Final level! This is where your Batch 2 autonomous logic takes control of the vehicle.

    Install the Senses:
    Bash

pip install mavsdk opencv-python pyzbar pyttsx3 numpy

Run the Mission:
Bash

    cd ~/Desktop/x500parkpilot
    python3 scripts/parkpilot.py

⚠️ Troubleshooting & Performance

    ODE Physics Lag: If the drone jitters, check your Real-Time Factor (RTF) at the bottom of Gazebo. Ensure it is close to 1.0.

    Lockstep Simulation: Remember that PX4 pauses until Gazebo sends sensor data. If your PC is slow, the drone won't crash; it will just fly in "slow motion."
