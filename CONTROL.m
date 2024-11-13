function [Etadot] = CONTROL(t,state,gains,gyro1,gyro2,auv,params,d,loop)
% CONTROL.m
% This script implements the control logic for the Autonomous Underwater
% Vehicle (AUV). The control system is responsible for calculating the
% required control forces and moments to drive the vehicle toward desired
% orientations and positions. The script uses Proportional-Derivative (PD)
% control gains to regulate the vehicle's roll, pitch, and yaw based on the
% current state and desired Euler angles.
% 
% Inputs
% - t: Current time in the simulation (unused in calculations, included for
% ODE solver compatibility). 
% - state: Current state vector of the vehicle,including position, 
% orientation, and velocities.
% - gains: Structure containing the PD control gains for the AUV's roll,
% pitch, and yaw. 
% - gyro: Structure containing the gyroscope's properties, including 
% inertia.
% - auv: Structure containing the AUV's properties, such as mass and
% inertia. 
% - params: Structure containing various physical parameters of the
% vehicle. 
% - d: Structure containing desired Euler angles (phi, theta, psi).
% - loop: Structure containing loop parameters, such as cycle time.
% 
% Outputs
% - Etadot: Time derivative of the generalized position vector,
% representing the AUV's response to the control inputs.

%% VARIABLES
phi_d       = d.phi;

Kpp         = gains.Kpp;     
Kdp         = gains.Kdp;

I1          = gyro1.I;
I2          = gyro2.I;

phi         = state(4);
theta       = state(5);
p           = state(10);
q           = state(11);
r           = state(12);

alpha1      = state(13);    
Omega1      = state(14);
alpha2      = state(15);    
Omega2      = state(16); 

T           = loop.cycleT;

%% ERROR
errphi      = phi_d-phi;
errphidot   = -(p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r);  

%% CONTROLLER

% % MK ROLL/YAW SEQUENTIAL CONTROL
% % Equation(s) 6a-6c, 7 MK Roll/Yaw --> change to pure roll control
% if t <= lt  % Roll only phase bringing yaw plane into alignment - think I only need this for roll sims
%     erralpha    = - int_r;  
%     Kgyro       = kpp*errphi + kdp*errphidot;
%     Nc          = kpr*erralpha - kdr*r; % Will I need these Nc terms here for a roll only simulation? TBD
%     Xc          = kpu*erru - kdu*udot + Xuu*u0^2;  
% else        % Roll and yaw phase to reach desired orientation
%     erralpha    = alpha - int_r;
%     Kgyro       = kpp*errphi + kdp*errphidot;
%     Nc          = kpr*erralpha - kdr*r;
%     Xc          = kpu*erru - kdu*udot + Xuu*u0^2;
% end

% TJI ACTIVE ROLL CONTROL
if t <= T   
    Kc      = Kpp*errphi + Kdp*errphidot;
end

% Desired control forces/torques
tauC.XD     = 0;    % Desired surge force (N)
tauC.YD     = 0;    % Desired sway force
tauC.ZD     = 0;    % Desired heave force
tauC.KD     = Kc;   % Desired roll moment (N-m)
tauC.MD     = 0;    % Desired pitch moment
tauC.ND     = 0;    % Desired yaw moment 

%% CMG CONTROLS
% AFT CMG (CMG #1)
contpar.Omegadot1   = (tauC.MD - tan(alpha1)*tauC.KD)/(I1*(((sin(alpha1))^2)/(cos(alpha1)))+I1*cos(alpha1));
contpar.alphadot1   = (tauC.KD + I1*sin(alpha1)*contpar.Omegadot1 + I1*cos(alpha1)*Omega1*r)/(-I1*cos(alpha1)*Omega1);   

% FWD CMG (CMG #2)
contpar.Omegadot2   = (tauC.MD - tan(alpha2)*tauC.KD)/(I2*(((sin(alpha2))^2)/(cos(alpha2)))+I2*cos(alpha2));
contpar.alphadot2   = (tauC.KD + I2*sin(alpha2)*contpar.Omegadot2 + I2*cos(alpha2)*Omega2*r)/(-I2*cos(alpha2)*Omega2);

% Calculate cmg torques
[tau_cmg1,tau_cmg2] = CMG(gyro1,gyro2,contpar,state); 

% Call vehicle dynamics
[Etadot]            = REMUS(t,auv,contpar,params,state,tauC,tau_cmg1,tau_cmg2);