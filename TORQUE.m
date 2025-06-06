function [tau_cmg1, tau_cmg2, tau_Omega, tau_alpha] = TORQUE(gyro1, gyro2, T_OUT, Y_OUT)
% TORQUE.m
% This script calculates the Control Moment Gyroscope (CMG) torques acting
% on a neutrally buoyant underwater vehicle. The torques are computed based
% on the vehicle's angular velocities, gyroscope properties, and the CMG
% deflection angle. The calculations account for the dynamics of the CMG,
% including the angular velocity and its derivative. The torques are
% returned as a structure with components K (roll), M (pitch), and N (yaw).
% 
% Inputs
% - gyro1, gyro2: Structures containing the properties of the two gyroscopes,
%                 including their moments of inertia (I).
% - T_OUT: Time vector output from the ODE solver.
% - Y_OUT: State vector output from the ODE solver, including angular
%          velocities and CMG states (deflection angle and flywheel angular
%          velocity).
%
% Outputs
% - tau_cmg1, tau_cmg2: Structures containing the calculated torques (K, M, N) 
%                       for roll, pitch, and yaw generated by each CMG in N*m.
% - tau_Omega: Vector of torques needed to change the flywheels’ angular velocities.
% - tau_alpha: Vector of torques needed to change the gimbals’ deflection angles.

%% VARIABLES
% Extract moments of inertia for each CMG
I1      = gyro1.I;
I2      = gyro2.I;

% Extract CMG state variables from the simulation output
% CMG #1 (Aft) deflection angle and flywheel angular velocity
alpha1  = Y_OUT(:,13);
Omega1  = Y_OUT(:,14);

% CMG #2 (Forward) deflection angle and flywheel angular velocity
alpha2  = Y_OUT(:,15);
Omega2  = Y_OUT(:,16);

% Compute derivatives (rates of change) for CMG state variables
% These represent the angular accelerations of the deflection angle and 
% flywheel speed for each CMG

% Change in deflection angle and flywheel speed for CMG #1
del_alpha1  = diff(alpha1);
del_Omega1  = diff(Omega1);
del_t       = diff(T_OUT);  % Time intervals between simulation steps

% Angular accelerations for CMG #1
alphadot1   = del_alpha1 ./ del_t;    % Gimbal angular velocity
Omegadot1   = del_Omega1 ./ del_t;    % Flywheel acceleration

% Change in deflection angle and flywheel speed for CMG #2
del_alpha2  = diff(alpha2);
del_Omega2  = diff(Omega2);

% Angular accelerations for CMG #2
alphadot2   = del_alpha2 ./ del_t;    % Gimbal angular velocity
Omegadot2   = del_Omega2 ./ del_t;    % Flywheel acceleration

% Calculate the midpoint time values for interpolating angular states
t_mid       = T_OUT(1:end-1) + del_t / 2;

% Interpolate angular states at the original time points to match the
% original vector lengths and improve accuracy

% Angular velocities of the vehicle's body around each axis
p       = interp1(t_mid, Y_OUT(1:end-1,10), T_OUT, 'linear', 'extrap');  % Roll rate
q       = interp1(t_mid, Y_OUT(1:end-1,11), T_OUT, 'linear', 'extrap');  % Pitch rate
r       = interp1(t_mid, Y_OUT(1:end-1,12), T_OUT, 'linear', 'extrap');  % Yaw rate

% Interpolated deflection angles and flywheel speeds for each CMG
alpha1  = interp1(t_mid, Y_OUT(1:end-1,13), T_OUT, 'linear', 'extrap');
Omega1  = interp1(t_mid, Y_OUT(1:end-1,14), T_OUT, 'linear', 'extrap');
alpha2  = interp1(t_mid, Y_OUT(1:end-1,15), T_OUT, 'linear', 'extrap');
Omega2  = interp1(t_mid, Y_OUT(1:end-1,16), T_OUT, 'linear', 'extrap');

% Redefine states to match lengths by excluding the last point in each vector
p       = p(1:end-1);
q       = q(1:end-1);
r       = r(1:end-1);
alpha1  = alpha1(1:end-1);
Omega1  = Omega1(1:end-1);
alpha2  = alpha2(1:end-1);
Omega2  = Omega2(1:end-1);

%% CMG RIGID BODY DYNAMICS
% Calculate torques generated by each CMG along roll (K), pitch (M), and yaw (N)

% AFT CMG (CMG #1)
tau_cmg1.K  = -I1 .* (cos(alpha1) .* Omega1 .* alphadot1 + sin(alpha1) .* Omegadot1 + cos(alpha1) .* Omega1 .* r);
tau_cmg1.M  = -I1 .* (sin(alpha1) .* Omega1 .* alphadot1 - cos(alpha1) .* Omegadot1 + sin(alpha1) .* Omega1 .* r);
tau_cmg1.N  = I1 .* Omega1 .* (cos(alpha1) .* p + sin(alpha1) .* q);

% FWD CMG (CMG #2)
tau_cmg2.K  = -I2 .* (cos(alpha2) .* Omega2 .* alphadot2 + sin(alpha2) .* Omegadot2 + cos(alpha2) .* Omega2 .* r);
tau_cmg2.M  = -I2 .* (sin(alpha2) .* Omega2 .* alphadot2 - cos(alpha2) .* Omegadot2 + sin(alpha2) .* Omega2 .* r);
tau_cmg2.N  = I2 .* Omega2 .* (cos(alpha2) .* p + sin(alpha2) .* q);

%% Calculate tau_Omega and tau_alpha in the CMG frame
% Torque values needed to adjust the CMGs' flywheel speeds and gimbal angles

% AFT CMG (CMG #1)
% Torque on the flywheel to change its angular velocity
tau_Omega1  = I1 .* Omegadot1;

% Torque on the gimbal to change its deflection angle
tau_alpha1  = I1 .* Omega1 .* alphadot1;

% FWD CMG (CMG #2)
% Torque on the flywheel to change its angular velocity
tau_Omega2  = I2 .* Omegadot2;

% Torque on the gimbal to change its deflection angle
tau_alpha2  = I2 .* Omega2 .* alphadot2;

% Combine tau_Omega and tau_alpha values into vectors for both CMGs
tau_Omega   = [tau_Omega1; tau_Omega2];
tau_alpha   = [tau_alpha1; tau_alpha2];
end