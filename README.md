# Autonomous Car-like Robot for Real-Time Target Following  

An autonomous car-like robot capable of following a moving target in real-time. Powered by an **NVIDIA Jetson Nano** and an **Intel RealSense D435i** depth camera with an integrated IMU, the system combines **Deep Learning**, **Computer Vision** and **Control Theory** to achieve robust performance.  

- **Object detection**: MobileNetV2-SSD for real-time tracking  
- **Control**: Lyapunov optimization & Robust Sliding Mode Control  
- **Simulation**: MATLAB/Simulink and ROS (RViz visualization)  
- **Applications**: object-following (leader-folloeing) robots, autonomous ground vehicles, research & teaching  

---

## 📑 Table of Contents
1. [Overview](#-overview)  
2. [Features](#-features)  
3. [Project Structure](#-project-structure)  
4. [Installation & Usage](#️-installation--usage)  
5. [Robot Design](#-robot-design)  
6. [Simulations & Results](#-simulations--results)  
7. [Team Members](#-team-members)  
8. [License](#-license)  

---

## 🚘 Overview  
The goal of this project is to design and implement an **autonomous car-like robot** that can:  
- Follow a moving target in real-time  
- Maintain a desired distance between robot and target  
- Handle car-like (non-holonomic) kinematics  

The project has been tested through:  
- **MATLAB/Simulink simulations** (control & system identification)  
- **ROS & RViz visualization**  
- **Experimental validation** on a real robot following a human in a multi-object environment  

⚠️ Note: Obstacle avoidance is **not included**. The environment should contain only one object of the "person" class.  

---

## ✨ Features  
- ✅ Real-time target detection with MobileNetV2 SSD  
- ✅ Car-like kinematics (Anti-Ackermann steering)  
- ✅ Differential mechanism driving force  
- ✅ MATLAB/Simulink simulations of control algorithms  
- ✅ ROS/RViz simulation environment  
- ✅ System identification for DC motor parameters  
- ✅ Steering angle curve fitting (MATLAB)  
- ✅ Python main control & vision codes  

---

## 📂 Project Structure  
```plaintext
Autonomous-Carlike-Robot-for-Real-Time-Target-Following/
│
├── python_codes/
│   ├── controller.py
│   ├── vision.py
│   └── IMU.py
│
├── matlab_simulation/
│   ├── Simulink.slx
│   ├── Animation.m
│   └── Parameters.m
│
├── system_identification/
│   ├── DC_motor_data.m
│   └── DC_motor_system_identification_tool.sid
│
├── curve_fitting/
│   ├── steering_angle_data.m
│   └── steering_curve_fitting_tool.sfit
│
├── ros_simulation/
│   ├── launch/
│   ├── worlds/
│   ├── src/
│   └── README.md
│
├── LICENSE
└── README.md
```
## 6. Installation and Usage
  1. Clone the repository:
   git clone https://github.com/Graduation-Project-team-manara-uni/Autonomous-Carlike-Robot-for-Real-Time-Target-Following.git
   cd Autonomous-Carlike-Robot-for-Real-Time-Target-Following
  2. Install Python dependencies:
  3. MATLAB simulation:
     open and run Parameters.m
     open and run Simulink.slx
     open and run Animation.m
  4. ros_simulation:
     Ensure you have ROS Noetic installed.
     write roslaunch carlike robot.launch on the terminal.
  5.  How to run the project.
     write python3 IMU.py and vision.py and controller.py on the terminal
## 7. Robot Design:
   Assembled Robot:
   <img width="500" alt="image" src="https://github.com/user-attachments/assets/ac2cd55e-d18c-4170-8212-73894de6b0d3" />
   <img width="500"  alt="image" src="https://github.com/user-attachments/assets/6a59f33f-ecf0-482f-bd17-0a81143dc047" />
   <img width="500"  alt="image" src="https://github.com/user-attachments/assets/a07bab66-7124-49aa-b457-c61811bfdbb6" />
   <img width="500"  alt="image" src="https://github.com/user-attachments/assets/2ff2c59a-bf2f-47a8-9644-fb8ca0205ceb" />
  Fabricated Parts:
   <img width="400"  alt="image" src="https://github.com/user-attachments/assets/8ccf7e98-fe7a-4307-baee-d66c7f069be3" />
   <img width="400"  alt="image" src="https://github.com/user-attachments/assets/abfd6348-1e18-4a6d-9dbd-9b358fc8f7e0" />

## 8. Simulations & Experimental Results:
   

## 9. Team Members:
   [karam alhawat](https://github.com/karamalhawat)
   [Ali Ali](https://github.com/AliB0239)
   [Saleh Rabea](https://github.com/Saleh-Rabea)
## 10.License:
   This project is licensed under the [MIT License](LICENSE).
