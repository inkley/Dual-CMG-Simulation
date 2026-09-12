function screen = CMG_HARDWARE_SCREENING(state, control, gyro, config)
%CMG_HARDWARE_SCREENING Convert a trajectory into preliminary drive needs.
% This is a requirements screen, not hardware qualification. The example
% motor is included only to establish that the flywheel requirement is in a
% commercially representative range. Packaging, drive voltage/current,
% thermal behavior, pressure housing, bearings, and rotor attachment remain
% outside the current model.

designMargin = 1.20;
limits = config.limits;
alpha = state(:,13);
Omega = state(:,14);
alphadot = control.alphadot(:,1);
Omegadot = control.Omegadot(:,1);

flywheelTorque = gyro.I*Omegadot;
flywheelPower = flywheelTorque.*Omega;
screen.designMargin = designMargin;
screen.flywheel.peakSpeed = max(abs(Omega));
screen.flywheel.peakTorque = max(abs(flywheelTorque));
screen.flywheel.peakMechanicalPower = max(abs(flywheelPower));
screen.flywheel.designSpeed = designMargin*screen.flywheel.peakSpeed;
screen.flywheel.designTorque = designMargin*screen.flywheel.peakTorque;
screen.flywheel.designMechanicalPower = ...
    designMargin*screen.flywheel.peakMechanicalPower;

% Representative comparison only: maxon EC-i 52, 200 W, 24 V, part 606793
% (manufacturer catalog values accessed 2026-09-10).
motor.name = 'maxon EC-i 52 200 W, 24 V, part 606793';
motor.nominalSpeed = 2970*2*pi/60;
motor.maximumSpeed = 5000*2*pi/60;
motor.continuousTorque = 0.646;
motor.nominalPower = 200;
screen.flywheel.representativeMotor = motor;
screen.flywheel.representativeMotorPass = ...
    screen.flywheel.designSpeed <= motor.nominalSpeed ...
    && screen.flywheel.designTorque <= motor.continuousTorque ...
    && screen.flywheel.designMechanicalPower <= motor.nominalPower;

screen.gimbal.peakAngle = max(abs(alpha));
screen.gimbal.peakRate = max(abs(alphadot));
screen.gimbal.peakAccel = max(abs(control.gimbalAccel(:,1)));
screen.gimbal.assemblyInertia = config.gimbal.assemblyInertia(1);
screen.gimbal.torque = screen.gimbal.assemblyInertia*control.gimbalAccel(:,1);
screen.gimbal.power = screen.gimbal.torque.*alphadot;
screen.gimbal.peakTorque = max(abs(screen.gimbal.torque));
screen.gimbal.peakMechanicalPower = max(abs(screen.gimbal.power));
screen.gimbal.designAngle = designMargin*screen.gimbal.peakAngle;
screen.gimbal.designRate = designMargin*screen.gimbal.peakRate;
screen.gimbal.angleWithinConfiguredBound = ...
    screen.gimbal.peakAngle <= limits.maxGimbalAngle;
screen.gimbal.rateWithinConfiguredBound = ...
    screen.gimbal.peakRate <= limits.maxGimbalRate;
screen.gimbal.designAngleWithinConfiguredBound = ...
    screen.gimbal.designAngle <= limits.maxGimbalAngle;
screen.gimbal.designRateWithinConfiguredBound = ...
    screen.gimbal.designRate <= limits.maxGimbalRate;
% Representative direct-drive comparison: maxon EC-i 52, 200 W, 24 V,
% part 606793. The model includes its published rotor inertia (264 g cm^2).
gimbalMotor.name = 'maxon EC-i 52 200 W, 24 V, part 606793';
gimbalMotor.nominalSpeed = 2970*2*pi/60;
gimbalMotor.maximumSpeed = 5000*2*pi/60;
gimbalMotor.continuousTorque = 0.646;
gimbalMotor.nominalPower = 200;
gimbalMotor.rotorInertia = 264e-7;
screen.gimbal.representativeMotor = gimbalMotor;
screen.gimbal.designTorque = designMargin*screen.gimbal.peakTorque;
screen.gimbal.designMechanicalPower = ...
    designMargin*screen.gimbal.peakMechanicalPower;
screen.gimbal.motorTorqueRequirementAvailable = true;
screen.gimbal.representativeMotorValidated = ...
    screen.gimbal.designRate <= gimbalMotor.nominalSpeed ...
    && screen.gimbal.designTorque <= gimbalMotor.continuousTorque ...
    && screen.gimbal.designMechanicalPower <= gimbalMotor.nominalPower;

screen.configuredTrajectoryLimitsPass = ...
    screen.gimbal.designAngleWithinConfiguredBound ...
    && screen.gimbal.designRateWithinConfiguredBound ...
    && designMargin*max(abs(Omega)) <= limits.maxFlywheelSpeed ...
    && designMargin*max(abs(Omegadot)) <= limits.maxFlywheelAccel;
screen.overallHardwareValidated = screen.configuredTrajectoryLimitsPass ...
    && screen.flywheel.representativeMotorPass ...
    && screen.gimbal.representativeMotorValidated;
if screen.overallHardwareValidated
    screen.overallStatus = 'PASS: representative component-level screen';
else
    screen.overallStatus = 'FAIL: representative component-level screen';
end
end
