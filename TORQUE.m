function [tau_cmg1, tau_cmg2, controlHistory] = TORQUE( ...
        T_OUT, Y_OUT, gains, gyro1, gyro2, auv, params, d, loop, cmgConfig)
%TORQUE Reconstruct exact controller and CMG histories at saved ODE states.
% CMG #1/#2 are module identifiers and do not imply aft/forward placement.
%
% This replays CONTROL at every solver output instead of estimating actuator
% rates with finite differences. All returned histories align with T_OUT.

sampleCount = numel(T_OUT);
tau_cmg1.K = zeros(sampleCount,1);
tau_cmg1.M = zeros(sampleCount,1);
tau_cmg1.N = zeros(sampleCount,1);
tau_cmg2 = tau_cmg1;

controlHistory.requestedMoment = zeros(sampleCount,3);
controlHistory.feedbackRollMoment = zeros(sampleCount,1);
controlHistory.momentumUnloadMoment = zeros(sampleCount,1);
controlHistory.externalDumpMoment = zeros(sampleCount,1);
controlHistory.rollDisturbance = zeros(sampleCount,1);
controlHistory.unconstrainedAchievedMoment = zeros(sampleCount,3);
controlHistory.achievedMoment = zeros(sampleCount,3);
controlHistory.allocationError = zeros(sampleCount,3);
controlHistory.actuatorLimitLoss = zeros(sampleCount,3);
controlHistory.momentError = zeros(sampleCount,3);
controlHistory.alphadot = zeros(sampleCount,2);
controlHistory.commandedAlphadot = zeros(sampleCount,2);
controlHistory.gimbalAccel = zeros(sampleCount,2);
controlHistory.gimbalAccelSaturated = false(sampleCount,1);
controlHistory.Omegadot = zeros(sampleCount,2);
controlHistory.conditionNumber = nan(sampleCount,1);
controlHistory.determinant = nan(sampleCount,1);
controlHistory.sigmaMin = nan(sampleCount,1);
controlHistory.usedDamping = false(sampleCount,1);
controlHistory.nearSingularity = false(sampleCount,1);
controlHistory.unconstrainedAlphadot = nan(sampleCount,2);
controlHistory.unconstrainedOmegadot = nan(sampleCount,2);
controlHistory.solveResidual = nan(sampleCount,1);
controlHistory.columnMagnitudeRatio = nan(sampleCount,1);
controlHistory.cancellationIndex = nan(sampleCount,1);
controlHistory.rollMomentTerms = nan(sampleCount,3);
controlHistory.pitchMomentTerms = nan(sampleCount,3);
controlHistory.gimbalRateSaturated = false(sampleCount,1);
controlHistory.flywheelAccelSaturated = false(sampleCount,1);
controlHistory.gimbalAngleLimited = false(sampleCount,1);
controlHistory.flywheelSpeedLimited = false(sampleCount,1);
controlHistory.thrusterCommand = zeros(sampleCount,2);
controlHistory.thrusterForce = zeros(sampleCount,2);
controlHistory.thrusterForceDot = zeros(sampleCount,2);
controlHistory.thrusterGeneralizedForce = zeros(sampleCount,6);
controlHistory.thrusterForceLimited = false(sampleCount,1);
controlHistory.thrusterRateLimited = false(sampleCount,1);
controlHistory.thrusterRequestedForceMoment = zeros(sampleCount,2);
controlHistory.thrusterAllocatedForceMoment = zeros(sampleCount,2);
controlHistory.thrusterAllocationResidual = zeros(sampleCount,2);
controlHistory.thrusterAllocationScale = ones(sampleCount,1);
controlHistory.thrusterAllocationSaturated = false(sampleCount,1);
controlHistory.hybridActivation = zeros(sampleCount,1);
controlHistory.hybridRollCommand = zeros(sampleCount,1);
controlHistory.hybridRollError = zeros(sampleCount,1);
controlHistory.hybridLateralPosition = zeros(sampleCount,1);
controlHistory.hybridLateralError = zeros(sampleCount,1);
controlHistory.hybridHeadingError = zeros(sampleCount,1);
controlHistory.propulsionCommand = zeros(sampleCount,1);
controlHistory.propulsionLimitedCommand = zeros(sampleCount,1);
controlHistory.propulsionForce = zeros(sampleCount,1);
controlHistory.propulsionForceDot = zeros(sampleCount,1);
controlHistory.propulsionGeneralizedForce = zeros(sampleCount,6);
controlHistory.propulsionForceLimited = false(sampleCount,1);
controlHistory.propulsionRateLimited = false(sampleCount,1);
controlHistory.propulsionVehiclePower = zeros(sampleCount,1);

for index = 1:sampleCount
    [~, data] = CONTROL(T_OUT(index), Y_OUT(index,:).', ...
        gains, gyro1, gyro2, auv, params, d, loop, cmgConfig);

    tau_cmg1.K(index) = data.tau_cmg1.K;
    tau_cmg1.M(index) = data.tau_cmg1.M;
    tau_cmg1.N(index) = data.tau_cmg1.N;
    tau_cmg2.K(index) = data.tau_cmg2.K;
    tau_cmg2.M(index) = data.tau_cmg2.M;
    tau_cmg2.N(index) = data.tau_cmg2.N;

    controlHistory.requestedMoment(index,:) = data.requestedMoment.';
    controlHistory.feedbackRollMoment(index) = data.feedbackRollMoment;
    controlHistory.momentumUnloadMoment(index) = data.momentumUnloadMoment;
    controlHistory.externalDumpMoment(index) = data.externalDumpMoment;
    controlHistory.rollDisturbance(index) = data.rollDisturbance;
    controlHistory.unconstrainedAchievedMoment(index,:) = ...
        data.unconstrainedAchievedMoment.';
    controlHistory.achievedMoment(index,:) = data.achievedMoment.';
    controlHistory.allocationError(index,:) = ...
        (data.requestedMoment-data.unconstrainedAchievedMoment).';
    controlHistory.actuatorLimitLoss(index,:) = ...
        (data.unconstrainedAchievedMoment-data.achievedMoment).';
    controlHistory.momentError(index,:) = ...
        (data.requestedMoment - data.achievedMoment).';
    controlHistory.alphadot(index,:) = [ ...
        data.contpar.alphadot1, data.contpar.alphadot2];
    controlHistory.commandedAlphadot(index,:) = ...
        data.actuator.commandedGimbalRate.';
    controlHistory.gimbalAccel(index,:) = data.actuator.gimbalAccel.';
    controlHistory.gimbalAccelSaturated(index) = ...
        data.actuator.gimbalAccelSaturated;
    controlHistory.Omegadot(index,:) = [ ...
        data.contpar.Omegadot1, data.contpar.Omegadot2];

    allocation = data.allocation;
    controlHistory.conditionNumber(index) = allocation.conditionNumber;
    controlHistory.determinant(index) = allocation.determinant;
    controlHistory.sigmaMin(index) = allocation.sigmaMin;
    controlHistory.usedDamping(index) = allocation.usedDamping;
    controlHistory.nearSingularity(index) = allocation.nearSingularity;
    controlHistory.unconstrainedAlphadot(index,:) = ...
        allocation.unconstrainedGimbalRates.';
    controlHistory.unconstrainedOmegadot(index,:) = ...
        allocation.unconstrainedFlywheelAccel.';
    controlHistory.solveResidual(index) = allocation.solveResidual;
    controlHistory.columnMagnitudeRatio(index) = ...
        allocation.columnMagnitudeRatio;
    controlHistory.cancellationIndex(index) = allocation.cancellationIndex;
    controlHistory.rollMomentTerms(index,:) = allocation.momentTerms(1,:);
    controlHistory.pitchMomentTerms(index,:) = allocation.momentTerms(2,:);
    controlHistory.gimbalRateSaturated(index) = ...
        allocation.gimbalRateSaturated;
    controlHistory.flywheelAccelSaturated(index) = ...
        allocation.flywheelAccelSaturated;
    controlHistory.gimbalAngleLimited(index) = ...
        allocation.gimbalAngleLimited;
    controlHistory.flywheelSpeedLimited(index) = ...
        allocation.flywheelSpeedLimited;
    controlHistory.thrusterCommand(index,:) = ...
        data.thruster.commandedForce.';
    controlHistory.thrusterForce(index,:) = data.thruster.actualForce.';
    controlHistory.thrusterForceDot(index,:) = data.thruster.forceDot.';
    controlHistory.thrusterGeneralizedForce(index,:) = ...
        data.thruster.generalizedForce.';
    controlHistory.thrusterForceLimited(index) = ...
        data.thruster.forceLimited;
    controlHistory.thrusterRateLimited(index) = ...
        data.thruster.forceRateLimited;
    thrusterAllocation = data.thrusterAllocation;
    controlHistory.thrusterRequestedForceMoment(index,:) = ...
        thrusterAllocation.requestedForceMoment.';
    controlHistory.thrusterAllocatedForceMoment(index,:) = ...
        thrusterAllocation.achievedForceMoment.';
    controlHistory.thrusterAllocationResidual(index,:) = ...
        thrusterAllocation.residual.';
    controlHistory.thrusterAllocationScale(index) = ...
        thrusterAllocation.scale;
    controlHistory.thrusterAllocationSaturated(index) = ...
        thrusterAllocation.saturated;
    controlHistory.hybridActivation(index) = data.hybrid.activation;
    controlHistory.hybridRollCommand(index) = data.hybrid.rollCommand;
    controlHistory.hybridRollError(index) = data.hybrid.rollError;
    controlHistory.hybridLateralPosition(index) = ...
        data.hybrid.lateralPosition;
    controlHistory.hybridLateralError(index) = data.hybrid.lateralError;
    controlHistory.hybridHeadingError(index) = data.hybrid.headingError;
    controlHistory.propulsionCommand(index) = data.propulsion.commandedForce;
    controlHistory.propulsionLimitedCommand(index) = data.propulsion.limitedCommand;
    controlHistory.propulsionForce(index) = data.propulsion.actualForce;
    controlHistory.propulsionForceDot(index) = data.propulsion.forceDot;
    controlHistory.propulsionGeneralizedForce(index,:) = data.propulsion.generalizedForce.';
    controlHistory.propulsionForceLimited(index) = data.propulsion.forceLimited;
    controlHistory.propulsionRateLimited(index) = data.propulsion.forceRateLimited;
    controlHistory.propulsionVehiclePower(index) = data.propulsion.vehiclePower;
end
end
