# Autonomous Car-like Robot for Real-Time Target Following  

An autonomous car-like robot capable of following a moving target in real-time. Powered by an **NVIDIA Jetson Nano** and an **Intel RealSense D435i** depth camera with an integrated IMU, the system combines **Deep Learning**, **Computer Vision** and **Control Theory** to achieve robust performance.  

- **Object detection**: MobileNetV2-SSD for real-time tracking  
- **Control**: Lyapunov optimization & Robust Sliding Mode Control  
- **Simulation**: MATLAB/Simulink and ROS (RViz visualization)  
- **Applications**: object-following (leader-following) robots, autonomous ground vehicles, research & teaching  

---

## 📑 Table of Contents
1. [Overview](#-overview)  
2. [Features](#-features)  
3. [Project Structure](#-project-structure)  
4. [Installation & Usage](#-Installation-&-Usage)  
5. [Robot Design](#-Robot-Design)  
6. [Simulations & Results](#-Simulations-&-Results)  
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
- **Experimental validation** on a real robot following a human (person) in a multi-object environment  

⚠️ Note: Obstacle avoidance is **not included**. The environment should contain only one object of the "object" class.  

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
---
## Installation & Usage
  1. Clone the repository:
   git clone https://github.com/Graduation-Project-team-manara-uni/Autonomous-Carlike-Robot-for-Real-Time-Target-Following.git
   cd Autonomous-Carlike-Robot-for-Real-Time-Target-Following
  2. Install Jetpack and Python dependencies in this repository: ```https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image```
  3. MATLAB simulation:
     open and run Parameters.m
     open and run Simulink.slx
     open and run Animation.m
  4. ros_simulation:
     Ensure you have ROS Noetic installed.
     write roslaunch carlike robot.launch on the terminal.
  5.  How to run the project.
     write ```python3 IMU.py & vision.py & controller.py``` on terminal
---
## Robot Design
   Assembled Robot:

   
   <img width="500" alt="image" src="https://github.com/user-attachments/assets/ac2cd55e-d18c-4170-8212-73894de6b0d3" />
   <img width="500"  alt="image" src="https://github.com/user-attachments/assets/6a59f33f-ecf0-482f-bd17-0a81143dc047" />
   <img width="500"  alt="image" src="https://github.com/user-attachments/assets/a07bab66-7124-49aa-b457-c61811bfdbb6" />
   <img width="500"  alt="image" src="https://github.com/user-attachments/assets/2ff2c59a-bf2f-47a8-9644-fb8ca0205ceb" />


  Fabricated Parts:


  
   <img width="400"  alt="image" src="https://github.com/user-attachments/assets/8ccf7e98-fe7a-4307-baee-d66c7f069be3" />
   <img width="400"  alt="image" src="https://github.com/user-attachments/assets/abfd6348-1e18-4a6d-9dbd-9b358fc8f7e0" />
   <img width="400"  alt="image" src="https://github.com/user-attachments/assets/9e945814-232b-42d8-a98b-5d10cf679639" />
   <img width="400"  alt="image" src="https://github.com/user-attachments/assets/2a408982-03ed-407b-a261-3d413de1d184" />
   <img width="400"  alt="image" src="https://github.com/user-attachments/assets/29dcdfc3-d201-42a1-9d8e-acced589ad59" />

---
## Simulations & Results
  object detection and computer vision rsults:
  <img width="1907"  alt="image" src="https://github.com/user-attachments/assets/f17872eb-a6cb-4c5d-b86b-bba58827df49" />

  steering angle curve fitting results:
  <img width="1501"  alt="image" src="https://github.com/user-attachments/assets/13cc8101-25c4-4d3f-8141-adbc1f081652" />

  <img width="1494"  alt="image" src="https://github.com/user-attachments/assets/f4ef3d22-8108-4240-8c1e-da244f2c123b" />

  Dc parameters estmiation:
  <img width="967"  alt="image" src="https://github.com/user-attachments/assets/96f9cef2-1ba5-4fb4-b56d-c0374970fa46" />
  matlab simulation results for static object:
  <img width="1995" alt="image" src="https://github.com/user-attachments/assets/c9d8db0f-3829-421b-9c6c-7f04b05a0a0f" />
  matlab simulation results for moving object:
  <img width="1990"  alt="image" src="https://github.com/user-attachments/assets/424bd683-a5f6-41f2-b31b-a1435217b470" />

  Experemental Results:
  <img width="1788"  alt="image" src="https://github.com/user-attachments/assets/f1824d52-45c5-411b-b774-29dc35a965ac" />
  <img width="1540"  alt="image" src="https://github.com/user-attachments/assets/9104ae03-1b51-4507-843a-983ca2b702d0" />
  <img width="1417"  alt="image" src="https://github.com/user-attachments/assets/61466259-6d70-4778-b7ca-b233bc167c10" />


    
   
---
## Team Members:
   [karam alhawat](https://github.com/karamalhawat)
   [Ali Ali](https://github.com/AliB0239)
   [Saleh Rabea](https://github.com/Saleh-Rabea)

---
## License:
   This project is licensed under the [License](LICENSE).

---
