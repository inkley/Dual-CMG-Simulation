function results = VERIFY_CMG_TORQUE_SIGNS()
%VERIFY_CMG_TORQUE_SIGNS Verify CMG-to-vehicle reaction-torque signs.
% Independently forms the rotor angular-momentum derivative and confirms
% that CMG.m returns its equal-and-opposite reaction on the vehicle.

tolerance = 1e-12;
gyro1.I = 2.5e-3;
gyro2.I = gyro1.I;

state = zeros(16,1);
state(10:12) = [0.31; -0.22; 0.17]; % [p;q;r] rad/s
state(13:16) = [0.43; 37; 0; 0];    % [alpha1;Omega1;alpha2;Omega2]
contpar.alphadot1 = -0.61;
contpar.Omegadot1 = 4.2;
contpar.alphadot2 = 0;
contpar.Omegadot2 = 0;

[tau1, tau2] = CMG(gyro1, gyro2, contpar, state);
alpha = state(13);
Omega = state(14);
omegaBody = state(10:12);
spinAxis = [sin(alpha); -cos(alpha); 0];
spinAxisDerivative = [cos(alpha); sin(alpha); 0];
rotorMomentum = gyro1.I * Omega * spinAxis;
rotorMomentumDerivative = gyro1.I * (...
    contpar.Omegadot1*spinAxis ...
    + Omega*contpar.alphadot1*spinAxisDerivative) ...
    + cross(omegaBody, rotorMomentum);
expectedVehicleTorque = -rotorMomentumDerivative;
actualVehicleTorque = [tau1.K; tau1.M; tau1.N];
vectorIdentityError = norm(actualVehicleTorque - expectedVehicleTorque);
assert(vectorIdentityError < tolerance, ...
    'CMG reaction torque does not equal -dH_rotor/dt.');
assert(norm([tau2.K; tau2.M; tau2.N]) < tolerance, ...
    'Inactive CMG #2 generated nonzero torque.');

% Canonical right-hand-rule checks at alpha=0 and positive Omega.
canonicalState = zeros(16,1);
canonicalState(14) = 30;
zeroCommand = struct('alphadot1',0, 'Omegadot1',0, ...
    'alphadot2',0, 'Omegadot2',0);

gimbalCommand = zeroCommand;
gimbalCommand.alphadot1 = 1;
[gimbalTorque, ~] = CMG(gyro1, gyro2, gimbalCommand, canonicalState);
assert(gimbalTorque.K < 0, ...
    'Positive gimbal rate at alpha=0 must produce negative vehicle roll torque.');

spinCommand = zeroCommand;
spinCommand.Omegadot1 = 1;
[spinTorque, ~] = CMG(gyro1, gyro2, spinCommand, canonicalState);
assert(spinTorque.M > 0, ...
    'Positive spin acceleration at alpha=0 must produce positive vehicle pitch torque.');

yawState = canonicalState;
yawState(12) = 1;
[yawCouplingTorque, ~] = CMG(gyro1, gyro2, zeroCommand, yawState);
assert(yawCouplingTorque.K < 0, ...
    'Positive body yaw rate at alpha=0 must produce negative vehicle roll torque.');

% Confirm that the single-CMG inverse reproduces a requested vehicle moment.
allocationState = zeros(16,1);
allocationState(13) = 0.37;
allocationState(14) = 40;
gains.Kpp = 1;
gains.Kdp = 0;
desired.phi = 0.08;
desired.theta = 0;
desired.psi = 0;
loop.cycleT = 1;
loop.controlEndTime = inf;
config.mode = 'single';
config.limits.maxGimbalRate = 1e12;
config.limits.maxGimbalAngle = inf;
config.limits.maxFlywheelAccel = 1e12;
config.limits.maxFlywheelSpeed = inf;
config.diagnostics.failOnNonfinite = true;
[requested, command] = CMG_ALLOCATE(0, allocationState, gains, ...
    gyro1, gyro2, desired, loop, config);
[allocatedTorque, ~] = CMG(gyro1, gyro2, command, allocationState);
allocationError = norm([allocatedTorque.K-requested.KD; ...
    allocatedTorque.M-requested.MD]);
assert(allocationError < tolerance, ...
    'Single-CMG inverse does not reproduce the requested K/M moment.');

results.vectorIdentityError = vectorIdentityError;
results.allocationError = allocationError;
results.canonicalSigns = struct( ...
    'positiveGimbalRateProducesNegativeK', gimbalTorque.K < 0, ...
    'positiveSpinAccelerationProducesPositiveM', spinTorque.M > 0, ...
    'positiveYawRateProducesNegativeK', yawCouplingTorque.K < 0);

fprintf('CMG torque-sign verification passed.\n');
fprintf('  Vector identity error: %.3e N m\n', vectorIdentityError);
fprintf('  Single allocation error: %.3e N m\n', allocationError);
end
