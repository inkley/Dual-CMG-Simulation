function energy = ENERGY_ANALYSIS(T, Y, tau1, tau2, control, gyro1, gyro2)
%ENERGY_ANALYSIS Compute time-aligned CMG work and ideal actuator metrics.
% CMG numbering identifies modules only; no mounting locations enter this
% energy accounting. Module structural kinetic energy is also outside scope.
%
% System boundary:
% - Useful output is gross roll work, integral(abs(K_total*p)).
% - Flywheel acceleration power is I*Omega*Omegadot.
% - Because gimbal inertia, friction, gearing, and motor efficiency are not
%   modeled, actual electrical input energy cannot yet be calculated.
% - The ideal gimbal-transfer proxy is the mechanical power each CMG
%   delivers to the vehicle, tau_cmg dot omega_body.
% - Initial flywheel spin energy is reported but excluded from maneuver
%   input energy because it is stored in the initial condition.
%
% The combined dual-CMG metric sums both modules. It is never averaged or
% labeled as a per-CMG efficiency.

bodyRates = Y(:,10:12);
tauMatrix1 = [tau1.K, tau1.M, tau1.N];
tauMatrix2 = [tau2.K, tau2.M, tau2.N];

power.bodyAxis1 = tauMatrix1 .* bodyRates;
power.bodyAxis2 = tauMatrix2 .* bodyRates;
power.body1 = sum(power.bodyAxis1, 2);
power.body2 = sum(power.bodyAxis2, 2);
power.roll1 = tau1.K .* Y(:,10);
power.roll2 = tau2.K .* Y(:,10);
power.rollTotal = power.roll1 + power.roll2;

power.flywheel1 = gyro1.I .* Y(:,14) .* control.Omegadot(:,1);
power.flywheel2 = gyro2.I .* Y(:,16) .* control.Omegadot(:,2);
power.idealGimbalGross1 = sum(abs(power.bodyAxis1), 2);
power.idealGimbalGross2 = sum(abs(power.bodyAxis2), 2);

energy.power = power;
energy.output.rollGross = trapz(T, abs(power.rollTotal));
energy.output.rollNet = trapz(T, power.rollTotal);
energy.output.bodyGross = trapz(T, ...
    power.idealGimbalGross1 + power.idealGimbalGross2);

energy.input.flywheel1Gross = trapz(T, abs(power.flywheel1));
energy.input.flywheel2Gross = trapz(T, abs(power.flywheel2));
energy.input.idealGimbal1Gross = trapz(T, power.idealGimbalGross1);
energy.input.idealGimbal2Gross = trapz(T, power.idealGimbalGross2);
energy.input.idealSystemGross = ...
    energy.input.flywheel1Gross + energy.input.flywheel2Gross ...
    + energy.input.idealGimbal1Gross + energy.input.idealGimbal2Gross;

energy.efficiency.idealSystemPercent = safePercent( ...
    energy.output.rollGross, energy.input.idealSystemGross);
energy.efficiency.electricalSystemPercent = nan;
energy.efficiency.electricalEfficiencySupported = false;

energy.perCMG(1) = perCMGMetrics(T, power.roll1, power.idealGimbalGross1, ...
    power.flywheel1);
energy.perCMG(2) = perCMGMetrics(T, power.roll2, power.idealGimbalGross2, ...
    power.flywheel2);

energy.storedInitial.flywheel1 = 0.5 * gyro1.I * Y(1,14)^2;
energy.storedInitial.flywheel2 = 0.5 * gyro2.I * Y(1,16)^2;
energy.storedInitial.total = energy.storedInitial.flywheel1 ...
    + energy.storedInitial.flywheel2;
energy.input.idealSystemGrossIncludingSpinup = ...
    energy.input.idealSystemGross + energy.storedInitial.total;
energy.efficiency.idealIncludingSpinupPercent = safePercent( ...
    energy.output.rollGross, ...
    energy.input.idealSystemGrossIncludingSpinup);
energy.assumptions = [ ...
    "Ideal mechanical transfer only; electrical motor efficiency, " ...
    "gimbal inertia, gearing, friction, windage, and regeneration are " ...
    "not modeled. Initial flywheel spin energy is reported separately " ...
    "and excluded from maneuver input energy."];
end

function metrics = perCMGMetrics(T, rollPower, gimbalGrossPower, flywheelPower)
    metrics.rollGross = trapz(T, abs(rollPower));
    metrics.bodyGross = trapz(T, gimbalGrossPower);
    metrics.flywheelGross = trapz(T, abs(flywheelPower));
    metrics.idealInputGross = metrics.bodyGross + metrics.flywheelGross;
    metrics.idealEfficiencyPercent = safePercent( ...
        metrics.rollGross, metrics.idealInputGross);
end

function percent = safePercent(outputEnergy, inputEnergy)
    if inputEnergy > eps
        percent = 100 * outputEnergy / inputEnergy;
    else
        percent = nan;
    end
end
