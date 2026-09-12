function [tauC, contpar, allocation] = CMG_ALLOCATE( ...
        t, state, gains, gyro1, gyro2, d, loop, cmgConfig)
%CMG_ALLOCATE Calculate the requested moment and CMG actuator commands.
% Allocation depends on rotor momentum and gimbal geometry, not CMG mounting
% position. Module displacement affects the assembled vehicle mass properties
% but does not change an ideal reaction couple's point of application.

phi = state(4);
theta = state(5);
p = state(10);
q = state(11);
r = state(12);

desiredPhi = d.phi;
if isfield(d,'rollToPlane')
    planeCommand = ROLL_TO_PLANE_COMMAND(state(4:6), ...
        d.rollToPlane.desiredLateralDirectionNED,state(4), ...
        d.rollToPlane.bidirectionalThruster);
    desiredPhi = planeCommand.rollAngle;
end
if isfield(d,'rollScheduleTime') && isfield(d,'rollScheduleAngle')
    desiredPhi = interp1(d.rollScheduleTime, d.rollScheduleAngle, t, ...
        'previous', 'extrap');
end
rollError = desiredPhi - phi;
rollRateError = -(p + sin(phi)*tan(theta)*q ...
    + cos(phi)*tan(theta)*r);
% Hold attitude feedback active for the complete simulation. Ending control
% at the nominal maneuver time can hide post-maneuver overshoot or drift.
if t <= loop.controlEndTime
    feedbackRollMoment = gains.Kpp*rollError + gains.Kdp*rollRateError;
else
    feedbackRollMoment = 0;
end

momentumUnloadMoment = 0;
externalDumpMoment = 0;
if isfield(cmgConfig,'momentumManagement') ...
        && cmgConfig.momentumManagement.enabled
    rotorHx = gyro1.I*state(14)*sin(state(13)) ...
        + gyro2.I*state(16)*sin(state(15));
    momentumError = rotorHx-cmgConfig.momentumManagement.referenceHx;
    rawUnloadMoment = cmgConfig.momentumManagement.gain*momentumError;
    momentumUnloadMoment = min(max(rawUnloadMoment, ...
        -cmgConfig.momentumManagement.maxDumpMoment), ...
        cmgConfig.momentumManagement.maxDumpMoment);
    % The external actuator supplies the opposite moment, so unloading does
    % not alter the net vehicle-attitude command.
    externalDumpMoment = -momentumUnloadMoment;
end
desiredRollMoment = feedbackRollMoment+momentumUnloadMoment;

tauC.XD = 0;
tauC.YD = 0;
tauC.ZD = 0;
tauC.KD = desiredRollMoment;
tauC.MD = 0;
tauC.ND = 0;
tauC.KFeedback = feedbackRollMoment;
tauC.KMomentumUnload = momentumUnloadMoment;
tauC.KExternalDump = externalDumpMoment;
tauC.KDisturbance = 0;
tauC.desiredPhi = desiredPhi;
if isfield(cmgConfig,'external') ...
        && isfield(cmgConfig.external,'rollDisturbance')
    tauC.KDisturbance = cmgConfig.external.rollDisturbance;
end

allocation.matrix = nan(2);
allocation.determinant = nan;
allocation.conditionNumber = nan;
allocation.sigmaMin = nan;
allocation.usedDamping = false;
allocation.nearSingularity = false;
allocation.unconstrainedGimbalRates = [nan; nan];
allocation.solveResidual = nan;
allocation.columnMagnitudeRatio = nan;
allocation.cancellationIndex = nan;
allocation.momentTerms = nan(2,3);

switch cmgConfig.mode
    case 'single'
        contpar = allocateSingle(state, gyro1, tauC);
    case 'dual'
        switch cmgConfig.dualController
            case 'legacy'
                contpar = allocateLegacyDual(state, gyro1, gyro2, tauC);
            case 'constant_speed'
                [contpar, allocation] = allocateConstantSpeedDual( ...
                    state, gyro1, gyro2, tauC, cmgConfig.allocator);
            otherwise
                error('Unsupported dual-CMG controller: %s', ...
                    cmgConfig.dualController);
        end
    otherwise
        error('Unsupported CMG mode: %s', cmgConfig.mode);
end

% Preserve the exact inverse/allocation output before actuator constraints.
% This separates mathematical allocation error from lost moment caused by
% gimbal-rate, flywheel-acceleration, or gimbal-angle limits.
allocation.unconstrainedCommand = contpar;
allocation.unconstrainedGimbalRates = [ ...
    contpar.alphadot1; contpar.alphadot2];
allocation.unconstrainedFlywheelAccel = [ ...
    contpar.Omegadot1; contpar.Omegadot2];

[contpar, saturation] = applyActuatorLimits( ...
    contpar, state, cmgConfig.limits);
allocation.gimbalRateSaturated = saturation.gimbalRate;
allocation.flywheelAccelSaturated = saturation.flywheelAccel;
allocation.gimbalAngleLimited = saturation.gimbalAngle;
allocation.flywheelSpeedLimited = saturation.flywheelSpeed;

commandVector = [contpar.alphadot1; contpar.alphadot2; ...
    contpar.Omegadot1; contpar.Omegadot2];
if cmgConfig.diagnostics.failOnNonfinite && any(~isfinite(commandVector))
    error('CMG:NonfiniteCommand', ...
        'Nonfinite CMG command at t = %.9g s. State: %s', ...
        t, mat2str(state.', 6));
end
end

function contpar = allocateSingle(state, gyro, tauC)
% In single mode, only CMG #1 is commanded. CMG #2 is explicitly stationary.
alpha = state(13);
Omega = state(14);
r = state(12);
I = gyro.I;

contpar.Omegadot1 = ...
    (-sin(alpha)*tauC.KD + cos(alpha)*tauC.MD) / I;
contpar.alphadot1 = ...
    -(cos(alpha)*tauC.KD + sin(alpha)*tauC.MD) / (I*Omega) - r;
contpar.Omegadot2 = 0;
contpar.alphadot2 = 0;
end

function contpar = allocateLegacyDual(state, gyro1, gyro2, tauC)
alpha1 = state(13);
Omega1 = state(14);
alpha2 = state(15);
Omega2 = state(16);
r = state(12);
I1 = gyro1.I;
I2 = gyro2.I;

contpar.Omegadot1 = ...
    (-sin(alpha1)*tauC.KD + cos(alpha1)*tauC.MD) / I1;
contpar.alphadot1 = ...
    -(cos(alpha1)*tauC.KD + sin(alpha1)*tauC.MD) / (I1*Omega1) - r;
contpar.Omegadot2 = ...
    (-sin(alpha2)*tauC.KD + cos(alpha2)*tauC.MD) / I2;
contpar.alphadot2 = ...
    -(cos(alpha2)*tauC.KD + sin(alpha2)*tauC.MD) / (I2*Omega2) - r;
end

function [contpar, allocation] = allocateConstantSpeedDual( ...
        state, gyro1, gyro2, tauC, allocator)
alpha1 = state(13);
Omega1 = state(14);
alpha2 = state(15);
Omega2 = state(16);
r = state(12);
h1 = gyro1.I * Omega1;
h2 = gyro2.I * Omega2;

% Vehicle reaction moment:
% [K; M] = B*[alphadot1; alphadot2] + B*[r; r].
% The minus sign converts rotor momentum rate into the equal-and-opposite
% moment exerted by the rotors on the vehicle.
B = -[h1*cos(alpha1), h2*cos(alpha2); ...
      h1*sin(alpha1), h2*sin(alpha2)];
requestedMoment = [tauC.KD; tauC.MD];
bodyRateBias = B*[r; r];
rhs = requestedMoment - bodyRateBias;
singularValues = svd(B);
sigmaMin = singularValues(end);

% The coupled equations are linear in the two gimbal rates, so fsolve is
% unnecessary. A smooth damped least-squares solve avoids both the explicit
% singular inverse and a discontinuous exact/damped switching boundary.
lambda = allocator.damping;
gimbalRates = B' * ((B*B' + lambda^2*eye(2)) \ rhs);
usedDamping = true;
nearSingularity = sigmaMin < allocator.sigmaThreshold;

contpar.alphadot1 = gimbalRates(1);
contpar.alphadot2 = gimbalRates(2);
contpar.Omegadot1 = 0;
contpar.Omegadot2 = 0;

allocation.matrix = B;
allocation.determinant = det(B);
allocation.conditionNumber = cond(B);
allocation.sigmaMin = sigmaMin;
allocation.usedDamping = usedDamping;
allocation.nearSingularity = nearSingularity;
allocation.unconstrainedGimbalRates = gimbalRates;
allocation.solveResidual = norm(B*gimbalRates-rhs) / max(norm(rhs), eps);
columnNorms = vecnorm(B);
allocation.columnMagnitudeRatio = ...
    max(columnNorms) / max(min(columnNorms), eps);
allocation.momentTerms = [ ...
    B(:,1)*gimbalRates(1), B(:,2)*gimbalRates(2), bodyRateBias];
allocation.cancellationIndex = ...
    sum(vecnorm(allocation.momentTerms)) ...
    / max(norm(sum(allocation.momentTerms,2)), eps);
end

function [contpar, saturation] = applyActuatorLimits(contpar, state, limits)
originalRates = [contpar.alphadot1; contpar.alphadot2];
originalAccel = [contpar.Omegadot1; contpar.Omegadot2];

% Apply physical hard bounds. The previous L*tanh(command/L) formulation
% attenuated every nonzero command, even below the specified limit, and thus
% conflated ordinary operation with saturation. The tuned baseline remains
% inside these bounds, so no switching boundary is encountered in that case.
rates = min(max(originalRates, -limits.maxGimbalRate), ...
    limits.maxGimbalRate);
accel = min(max(originalAccel, -limits.maxFlywheelAccel), ...
    limits.maxFlywheelAccel);

angles = [state(13); state(15)];
angleLimited = false(2,1);
for index = 1:2
    pushingOutward = abs(angles(index)) >= limits.maxGimbalAngle ...
        && sign(rates(index)) == sign(angles(index));
    if pushingOutward
        rates(index) = 0;
        angleLimited(index) = true;
    end
end

flywheelSpeeds = [state(14); state(16)];
speedLimited = false(2,1);
for index = 1:2
    pushingSpeedOutward = ...
        abs(flywheelSpeeds(index)) >= limits.maxFlywheelSpeed ...
        && sign(accel(index)) == sign(flywheelSpeeds(index));
    if pushingSpeedOutward
        accel(index) = 0;
        speedLimited(index) = true;
    end
end

contpar.alphadot1 = rates(1);
contpar.alphadot2 = rates(2);
contpar.Omegadot1 = accel(1);
contpar.Omegadot2 = accel(2);

saturation.gimbalRate = any(abs(originalRates) > limits.maxGimbalRate);
saturation.flywheelAccel = any( ...
    abs(originalAccel) > limits.maxFlywheelAccel);
saturation.gimbalAngle = any(angleLimited);
saturation.flywheelSpeed = any(speedLimited);
end
