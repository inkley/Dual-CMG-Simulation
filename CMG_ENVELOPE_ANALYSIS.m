function envelope = CMG_ENVELOPE_ANALYSIS( ...
        time, stateHistory, control, gyro, limits)
%CMG_ENVELOPE_ANALYSIS Summarize the single-CMG actuator and momentum envelope.
% For the variable-speed single-CMG K/M mapping used here, det(B) is
% proportional to I^2*Omega. The formal steering singularity is Omega=0;
% alpha does not make the full two-input mapping singular. At cos(alpha)=0,
% however, gimbal motion provides no roll moment and roll control depends on
% flywheel acceleration, creating a practical actuator-limited condition.

alpha = stateHistory(:,13);
Omega = stateHistory(:,14);
unconstrainedGimbalRate = control.unconstrainedAlphadot(:,1);
unconstrainedFlywheelAccel = control.unconstrainedOmegadot(:,1);

envelope.duration = time(end)-time(1);
envelope.minimumAlpha = min(alpha);
envelope.maximumAlpha = max(alpha);
envelope.maximumAbsAlpha = max(abs(alpha));
envelope.finalAlpha = alpha(end);
envelope.gimbalExcursion = max(alpha)-min(alpha);
envelope.minimumOmega = min(Omega);
envelope.maximumOmega = max(Omega);
envelope.maximumAbsOmega = max(abs(Omega));
envelope.minimumAbsOmega = min(abs(Omega));
envelope.finalOmega = Omega(end);
envelope.omegaZeroCrossings = nnz(Omega(1:end-1).*Omega(2:end) <= 0);
envelope.minimumMomentumMagnitude = gyro.I*envelope.minimumAbsOmega;
envelope.maximumMomentumMagnitude = gyro.I*envelope.maximumAbsOmega;
envelope.peakUnconstrainedGimbalRate = ...
    max(abs(unconstrainedGimbalRate));
envelope.peakUnconstrainedFlywheelAccel = ...
    max(abs(unconstrainedFlywheelAccel));
envelope.peakGimbalRateUtilization = ...
    envelope.peakUnconstrainedGimbalRate/limits.maxGimbalRate;
envelope.peakFlywheelAccelUtilization = ...
    envelope.peakUnconstrainedFlywheelAccel/limits.maxFlywheelAccel;
envelope.peakGimbalAngleUtilization = ...
    envelope.maximumAbsAlpha/limits.maxGimbalAngle;
envelope.peakFlywheelSpeedUtilization = ...
    envelope.maximumAbsOmega/limits.maxFlywheelSpeed;
envelope.gimbalRateLimitedSamples = nnz(control.gimbalRateSaturated);
envelope.gimbalAccelLimitedSamples = nnz(control.gimbalAccelSaturated);
envelope.flywheelAccelLimitedSamples = nnz( ...
    control.flywheelAccelSaturated);
envelope.gimbalAngleLimitedSamples = nnz(control.gimbalAngleLimited);
envelope.flywheelSpeedLimitedSamples = nnz( ...
    control.flywheelSpeedLimited);
envelope.nearGimbalRollNullSamples = nnz(abs(cos(alpha)) < 0.05);
envelope.gimbalAngleLimitUndefined = ~isfinite(limits.maxGimbalAngle);
envelope.flywheelSpeedLimitUndefined = ...
    ~isfinite(limits.maxFlywheelSpeed);
envelope.sustainsRequestedMomentWithoutLimiting = ...
    envelope.gimbalRateLimitedSamples == 0 ...
    && envelope.gimbalAccelLimitedSamples == 0 ...
    && envelope.flywheelAccelLimitedSamples == 0 ...
    && envelope.gimbalAngleLimitedSamples == 0 ...
    && envelope.flywheelSpeedLimitedSamples == 0;
end
