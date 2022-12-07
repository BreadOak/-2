clc
clear all
close all

m = 0.0923;     % mass of swinging rod [kg] 
M = 0.2275;     % mass of block [kg] 
l = 0.1950;     % distance from center of mass of pendulum to axis of rotation [m]  
J = 0.0029;     % moment of inertia of a pendulum rod as it revolves around the center of mass [kg*m^2] 
g = 9.8;        % gravitational acceleration [m/s^2]
R = 4.29;       % armature resistance [ohm] 
r = 0.018;      % radius of synchronous pulley [m] 
K_m = 0.183;    % electromagnetic torque coefficient [N*m/A] 
K_e = 0.208;    % counter electromotive force coefficient [V*s/rad] 
I = 7.083e-06;  % inertia of rotation of synchronous pulley around motor shaft [kg*m^2] 

% ----- Fill in the Missing Code ----- %
A_22 = ;
A_23 = ;
A_42 = ;
A_43 = ;
B_21 = ;
B_41 = ;
% ----- Fill in the Missing Code ----- %

A = [0      1     0  0; 
     0   A_22  A_23  0; 
     0      0     0  1; 
     0   A_42  A_43  0];

B = [     0; 
       B_21; 
          0; 
       B_41];

disp(A)
disp(B)