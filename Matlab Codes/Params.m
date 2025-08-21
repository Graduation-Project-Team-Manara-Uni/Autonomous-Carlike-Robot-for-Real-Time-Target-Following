clc
clear
%clear all
%% Robot Dimensions
Dr=0.082; % rear axle to Center Of Mass (COM) distance
Df=0.05;  % front axle to Center Of Mass (COM) distance
D=Dr+Df;  % front to rear axle distance
Dc=0.205; % rear axle to the camera mounting point
w=0.15;   % robot width
lw=0.06;  % wheel lenght = wheel diameter
ww=0.03;  % wheel widht
m = 1.3;  % Mass (kg)
J = 0.01; % Moment of inertia (kg·m²)
b = Dr;   % Geometry parameter (m)

%% Control Parameters
k=1;
kx=k;ky=k; % Lyapunov-based control constants
Ld=0.8;    % Desired distance of robot camera from the target

%% Velocity Bounds
Vmax=6;
Vback_max=6;
%% Steering Bounds
Psi_max=pi/3; 


% Steering System Parameters
Ts = 0.02;    % Time constant (s)
Ks = 1.0;    % Gain

% Motor Parameters
R = 8.2;  % Armature resistance (?)
La = 0.0015; % Armature inductance (H)
Kb = 0.175;  % Back-EMF constant (V·s/rad)
Ka = 0.175;  % Torque constant (N·m/A)
Iw = 0.093; % Wheel inertia (kg·m²)
rw = 0.03;  % Wheel radius (m)
Gr = 3;   % Gear ratio

% Sliding Mode Control (SMC) Parameters
lambda1 = 5;
Ksw = 0.1;
n=10;
