function capacity = CMG_CAPACITY_ANALYSIS(gyro, alpha, Omega, limits, ...
        rollInertia, rollAngle, cycleDuration, rollDutyCycle, ...
        rollDragCoefficient)
%CMG_CAPACITY_ANALYSIS Compare initial CMG roll authority with vehicle inertia.
% The calculation is an instantaneous screening test at the initial gimbal
% angle and flywheel speed. It does not claim that the same torque can be
% sustained through a large gimbal excursion; that is checked separately.
%
% A smooth rest-to-rest cubic reference is used:
%   phi(s) = phi_f*(3*s^2 - 2*s^3),  s=t/T.
% Its peak acceleration is 6*phi_f/T^2 and peak speed is 1.5*phi_f/T.
% The inertia and quadratic-drag peaks do not occur at exactly the same time,
% so their sum below is a slightly conservative required-torque estimate.

arguments
    gyro (1,1) struct
    alpha (1,1) double {mustBeFinite}
    Omega (1,1) double {mustBeFinite}
    limits (1,1) struct
    rollInertia (1,1) double {mustBePositive}
    rollAngle (1,1) double {mustBeNonnegative}
    cycleDuration (1,1) double {mustBePositive}
    rollDutyCycle (1,1) double {mustBePositive, mustBeLessThanOrEqual(rollDutyCycle,1)}
    rollDragCoefficient (1,1) double {mustBeFinite}
end

capacity.rotorSpinMomentum = abs(gyro.I*Omega);
capacity.gimbalRollTorque = gyro.I*abs(Omega*cos(alpha)) ...
    * limits.maxGimbalRate;
capacity.flywheelRollTorque = gyro.I*abs(sin(alpha)) ...
    * limits.maxFlywheelAccel;
capacity.instantaneousRollTorque = capacity.gimbalRollTorque ...
    + capacity.flywheelRollTorque;
capacity.maximumRollAcceleration = ...
    capacity.instantaneousRollTorque/rollInertia;
capacity.minimumBangBangTime = 2*sqrt( ...
    rollInertia*rollAngle/max(capacity.instantaneousRollTorque,eps));

capacity.fullCycle = maneuverRequirement(cycleDuration, rollAngle, ...
    rollInertia, rollDragCoefficient, capacity.instantaneousRollTorque);
referenceDuration = rollDutyCycle*cycleDuration;
capacity.referenceRollStage = maneuverRequirement(referenceDuration, ...
    rollAngle, rollInertia, rollDragCoefficient, ...
    capacity.instantaneousRollTorque);
capacity.referenceRollDutyCycle = rollDutyCycle;
end

function requirement = maneuverRequirement(duration, angle, inertia, ...
        dragCoefficient, availableTorque)
    requirement.duration = duration;
    requirement.peakAcceleration = 6*angle/duration^2;
    requirement.peakRate = 1.5*angle/duration;
    requirement.inertialTorque = inertia*requirement.peakAcceleration;
    requirement.dragTorqueBound = abs(dragCoefficient) ...
        * requirement.peakRate^2;
    requirement.requiredTorque = requirement.inertialTorque ...
        + requirement.dragTorqueBound;
    requirement.torqueMargin = availableTorque ...
        / max(requirement.requiredTorque,eps);
    requirement.passesInstantaneousScreen = ...
        availableTorque >= requirement.requiredTorque;
end
