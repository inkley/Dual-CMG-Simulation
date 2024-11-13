function [tau_cmg1,tau_cmg2,tau_Omega,tau_alpha] = TORQUE(gyro1,gyro2,T_OUT,Y_OUT)
% TORQUE.m
% This script calculates the Control Moment Gyroscope (CMG) torques acting
% on a neutrally buoyant underwater vehicle. The torques are computed based
% on the vehicle's angular velocities, gyroscope properties, and the CMG
% deflection angle. The calculations account for the dynamics of the CMG,
% including the angular velocity and its derivative. The torques are
% returned as a structure with components K (roll), M (pitch), and N (yaw).
% 
% Inputs
% - gyro: Structure containing the gyroscope's properties, including
% inertia.
% - T_OUT: Time vector output from the ODE solver. 
% - Y_OUT: State vector output from the ODE solver, including angular
% velocities and CMG states.
% 
% Outputs 
% - tau_cmg: Structure containing the calculated torques (K, M, N) in N*m.

%% VARIABLES
I1          = gyro1.I;
I2          = gyro2.I;
alpha1      = Y_OUT(:,13);
Omega1      = Y_OUT(:,14); 
alpha2      = Y_OUT(:,15);
Omega2      = Y_OUT(:,16);

% Derivative calcs 
del_alpha1  = diff(alpha1);
del_Omega1  = diff(Omega1);
del_t       = diff(T_OUT);

alphadot1   = del_alpha1./del_t;
Omegadot1   = del_Omega1./del_t; 

del_alpha2  = diff(alpha2);
del_Omega2  = diff(Omega2);

alphadot2   = del_alpha2./del_t;
Omegadot2   = del_Omega2./del_t; 

% Time midpoint for interpolation
t_mid       = T_OUT(1:end-1) + del_t/2;

% Interpolating states at original time points
p           = interp1(t_mid, Y_OUT(1:end-1,10), T_OUT, 'linear', 'extrap');
q           = interp1(t_mid, Y_OUT(1:end-1,11), T_OUT, 'linear', 'extrap');
r           = interp1(t_mid, Y_OUT(1:end-1,12), T_OUT, 'linear', 'extrap');
alpha1      = interp1(t_mid, Y_OUT(1:end-1,13), T_OUT, 'linear', 'extrap');
Omega1      = interp1(t_mid, Y_OUT(1:end-1,14), T_OUT, 'linear', 'extrap');
alpha2      = interp1(t_mid, Y_OUT(1:end-1,15), T_OUT, 'linear', 'extrap');
Omega2      = interp1(t_mid, Y_OUT(1:end-1,16), T_OUT, 'linear', 'extrap');

% Redefine states to match lengths
p           = p(1:end-1);
q           = q(1:end-1);
r           = r(1:end-1);
alpha1      = alpha1(1:end-1);
Omega1      = Omega1(1:end-1);
alpha2      = alpha2(1:end-1);
Omega2      = Omega2(1:end-1);

%% CMG RIGID BODY DYNAMICS
% AFT CMG (CMG #1)
tau_cmg1.K  = -I1.*(cos(alpha1).*Omega1.*alphadot1 + sin(alpha1).*Omegadot1 + cos(alpha1).*Omega1.*r);
tau_cmg1.M  = -I1.*(sin(alpha1).*Omega1.*alphadot1 - cos(alpha1).*Omegadot1 + sin(alpha1).*Omega1.*r);
tau_cmg1.N  = I1.*Omega1.*(cos(alpha1).*p + sin(alpha1).*q);

% FWD CMG (CMG #2)
tau_cmg2.K  = -I2.*(cos(alpha2).*Omega2.*alphadot2 + sin(alpha2).*Omegadot2 + cos(alpha2).*Omega2.*r);
tau_cmg2.M  = -I2.*(sin(alpha2).*Omega2.*alphadot2 - cos(alpha2).*Omegadot2 + sin(alpha2).*Omega2.*r);
tau_cmg2.N  = I2.*Omega2.*(cos(alpha2).*p + sin(alpha2).*q);

%% Calculate tau_Omega and tau_alpha in the CMG frame
% AFT CMG (CMG #1)
% Flywheel torque needed to change the flywheel’s angular velocity
tau_Omega1 = I1.*Omegadot1;

% Gimbal torque needed to change the gimbal’s deflection angle
tau_alpha1 = I1.*Omega1.*alphadot1;

% FWD CMG (CMG #2)
% Flywheel torque needed to change the flywheel’s angular velocity
tau_Omega2 = I2.*Omegadot2;

% Gimbal torque needed to change the gimbal’s deflection angle
tau_alpha2 = I2.*Omega2.*alphadot2;

tau_Omega     = [tau_Omega1;tau_Omega2];
tau_alpha     = [tau_alpha1;tau_alpha2];
end