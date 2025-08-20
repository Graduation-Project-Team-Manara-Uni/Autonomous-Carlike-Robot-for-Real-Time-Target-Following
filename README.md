# Autonomous-Carlike-Robot-for-Real-Time-Target-Following 
1.Project Description
An autonomous car-like robot for real-time target following. The system, powered by an NVIDIA Jetson Nano, uses an Intel RealSense D435i camera with an integrated IMU for 3D perception and state estimation. MobileNetv2 SSD handles object tracking, while precise and stable motion is achieved using Lyapunov optimization and Robust Sliding Mode Control.
2.Table of Contents
    
3.Overview
the goal of this project is to design and implement an autonomous car-like robot that can follow a moving target in real-time and keep a desired distance between the robot and the target.
the project combines Computer Vision and Control Theory to achieve this goal. The Project has bee simulated using MATLAB/SIMULINK and ROS (RVIZ 3D visualization tool). The experimental results has been measured for following human body in multi-object enviroment (obstacle avoidance is not included). The enviroment should contain only one object of the person class.
4.Features
- Target detection using MobileNetv2 SSD in real-time
- Car-like (Anti-Ackermann steering) kinematics
- Car-like (differential mechanism driving force) kinematics
- MATLAB/Simulink simulation of control algorithms
- ROS/RVIZ simulation
- MATLAB/system identification for DC Motor parameters
- MATLAB/curve fitting for steering angle
- Python main codes
5.Project Structure
-Autonomous-Carlike-Robot-for-Real-Time-Target-Following/
   python_codes/
           controller.py
           vision.py
           IMU.py
   matlab_simulation/
│   ├── Simulink.slx
│   ├── Animation.m
│   └── Parameters.m
│── system identification/
│   ├── DC_motor_data.m
│   └── DC_motor_system_identification_tool.sid
│── curve fitting/
│   ├── steering_angle_data.m
│   └── steering_curve_fitting_tool.sfit
│── ros_simulation/
│   ├── launch/
│   ├── worlds/
│   └── src/
│── README.md
│── LICENSE
6. Installation and Usage
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
7. Robot Design:
   Assembled Robot:
   <img width="1078" height="450" alt="image" src="https://github.com/user-attachments/assets/ac2cd55e-d18c-4170-8212-73894de6b0d3" />
   <img width="573" height="450" alt="image" src="https://github.com/user-attachments/assets/6a59f33f-ecf0-482f-bd17-0a81143dc047" />
   <img width="469" height="352" alt="image" src="https://github.com/user-attachments/assets/a07bab66-7124-49aa-b457-c61811bfdbb6" />
   <img width="571" height="429" alt="image" src="https://github.com/user-attachments/assets/2ff2c59a-bf2f-47a8-9644-fb8ca0205ceb" />
  Fabricated Parts:
   <img width="476" height="402" alt="image" src="https://github.com/user-attachments/assets/290bf0db-97ed-40cd-84a2-e5699b4651df" />
   <img width="513" height="374" alt="image" src="https://github.com/user-attachments/assets/8ccf7e98-fe7a-4307-baee-d66c7f069be3" />
   <img width="644" height="374" alt="image" src="https://github.com/user-attachments/assets/abfd6348-1e18-4a6d-9dbd-9b358fc8f7e0" />

7. Simulations & Experimental Results:
   

9. Team Members:
   [karam alhawat](https://github.com/karamalhawat)
   [Ali Ali](https://github.com/AliB0239)
   [Saleh Rabea](https://github.com/Saleh-Rabea)
10.License:
   This project is licensed under the [MIT License](LICENSE).
