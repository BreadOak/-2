clc
clear all
close all

Start_time = 0;
End_time = 10;
st = 0.005;
tspan = [0,st];
T = 0 : 1 : (End_time-Start_time)/st;

u = 0;
state_ini = [0; 0; pi/20; 0]; 
state_des = [0;0;0;0];

% Initialize save data list
u_data = [];
x_data   = [];
dx_data  = [];
th_data  = [];
dth_data = [];
x_err_data   = [];
dx_err_data  = [];
th_err_data  = [];
dth_err_data = [];

max_vol = 12;  % [V] 
bit_res = 2^8; % PWM resolution 0~255 8bit

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


A_22 = ;
A_23 = ;
A_42 = ;
A_43 = ;
B_21 = ;
B_41 = ;

A = [0      1     0  0; 
     0   A_22  A_23  0; 
     0      0     0  1; 
     0   A_42  A_43  0];

B = [     0; 
       B_21; 
          0; 
       B_41];

C = [1 0 0 0; 
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];

D =[0 0]';

% ----- Fill in the Missing Code ----- %
Q_11 = ;
Q_22 = ;
Q_33 = ;
Q_44 = ;

Q = [Q_11   0     0    0; 
        0 Q_22    0    0; 
        0    0 Q_33    0; 
        0    0    0 Q_44]; 

R = ;   
% ----- Fill in the Missing Code ----- %
 
sys = ss(A,B,C,D);
sysd = c2d(sys,st,'zoh'); % transforming a continuous system into a discrete system
Ad = sysd.A;
Bd = sysd.B;
[K,P] = dlqr(Ad,Bd,Q,R);  % calculate LQR gain
disp(K)

% Control loop
for i = T 
    % save data
    u_data(end+1)   = u;
    x_data(end+1)   = state_ini(1);
    dx_data(end+1)  = state_ini(2);
    th_data(end+1)  = state_ini(3);
    dth_data(end+1) = state_ini(4);

    state_error = state_ini - state_des;                     % calculate error
    pwm = -int32((bit_res/max_vol)*K*state_error);           % calculate control input
    
    % saturation
    if abs(pwm) > 245
        pwm = (245*sign(pwm));
    end

    % control input
    u = (max_vol/bit_res)*double(pwm+10);

    % simulation
    [t,S_list] = ode45(@(t,x_state_ini)InvertedPendulum(t,x_state_ini,u), tspan, state_ini);
    state_ini = S_list(end,(1:4)).';
end

% Plot
f1 = figure;
subplot(2,1,1)
plot(T*st,x_data)
xlabel('time')
ylabel('X [m]')
xlim([0, End_time])
ylim([-0.2 0.2])
grid on

subplot(2,1,2)
plot(T*st,th_data)
xlabel('time')
ylabel('Theta [rad]')
xlim([0, End_time])
ylim([-pi/4 pi/4])
grid on

f2 = figure;
plot(T*st, u_data)
title('Control input [V]')
grid on