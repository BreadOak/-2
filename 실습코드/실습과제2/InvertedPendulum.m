function dxdt = InvertedPendulum(t,x,u)
    m = 0.0923;     % mass of swinging rod [kg] 
    M = 0.2275;     % mass of block [kg] 
    l = 0.1950;     % distance from center of mass of pendulum to axis of rotation [m]  
    J = 0.0029;     % moment of inertia of a pendulum rod as it revolves around the center of mass [kg*m^2] 
    g = 9.8;        % gravitational acceleration
    R = 4.29;       % [ohm] armature resistance
    r = 0.018;      % [m] radius of synchronous pulley
    K_m = 0.183;    % [N*m/A] electromagnetic torque coefficient
    K_e = 0.208;    % [V*s/rad] counter electromotive force coefficient
    I = 7.083e-06;  % [kg*m^2] inertia of rotation of synchronous pulley around motor shaft
    
    Q_eq = m*J+(J+m*l^2)*(M+I/(2*r^2));
    A_22 = -(K_m*K_e*(J+m*l^2))/(Q_eq*R*r^2);
    A_23 = -(m^2*l^2*g)/Q_eq;
    A_42 = (m*l*K_m*K_e)/(Q_eq*R*r^2);
    A_43 = (m*g*l*(M+m+I/(2*r^2)))/Q_eq;
    B_21 = (K_m*(J+m*l^2))/(Q_eq*R*r);
    B_41 = -(m*l*K_m)/(Q_eq*R*r);
    
    A = [0      1      0  0; 
         0  A_22/2  A_23  0; 
         0       0     0  1; 
         0  A_42/2  A_43  0];
    
    B = [     0; 
         B_21/2; 
              0; 
         B_41/2];
    
    dxdt = A*x + B*u;
end
