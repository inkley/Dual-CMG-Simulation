% AUV_SIM.m
% Main driver for single- and dual-CMG AUV roll-control simulations.
%
% Mass-property convention:
% - The body-frame origin is the vehicle reference point and the configured
%   center of gravity is coincident with that origin (xg = yg = zg = 0).
% - Hydrostatic restoring forces/moments are disabled in REMUS.m, which is
%   consistent with neutral buoyancy and no modeled CB-to-CG offset.
% - CMGs are ideal internal pure-moment actuators. Their mounting locations
%   are not state/configuration inputs and do not alter the applied couple.
% - Base vehicle mass properties are augmented by the modeled CMG rotor mass
%   and inertia. ASSEMBLE_VEHICLE_MASS_PROPERTIES applies the parallel-axis
%   theorem and returns properties about the combined center of mass.
% - For a module centered at r = [d,0,0] on the longitudinal body axis, its
%   parallel-axis increment is diag([0,m*d^2,m*d^2]); displacement increases
%   pitch/yaw inertia, not roll inertia. Rotor spin speed Omega is not used in
%   this shift because the module center does not orbit the vehicle origin.
%
% Author:       Tyler J. Inkley
% Last updated: 09/08/2026

clc;
clearvars;
close all;

%% USER CONFIGURATION
cmgConfig.mode = 'dual'; % 'single' or 'dual'
cmgConfig.dualController = 'constant_speed'; % 'constant_speed' or 'legacy'

% Allocation and actuator safeguards. The angle and speed bounds are
% provisional design-screening requirements, not measured hardware ratings.
% They were selected by applying modest margin to the tuned single-CMG run:
% at least +/-100 deg of travel and at least 1800 rpm rotor speed. Replace
% them with vendor/tested continuous and transient ratings before publication.
cmgConfig.allocator.sigmaThreshold = 1e-3;
cmgConfig.allocator.damping = 2e-3;
cmgConfig.limits.maxGimbalRate = 20;       % rad/s
cmgConfig.limits.maxGimbalAngle = deg2rad(100); % rad; provisional +/-100 deg
cmgConfig.limits.maxGimbalAccel = 500;      % rad/s^2; preliminary motor bound
cmgConfig.limits.maxFlywheelAccel = 500;   % rad/s^2
cmgConfig.limits.maxFlywheelSpeed = 1800*2*pi/60; % rad/s; provisional 1800 rpm
cmgConfig.limits.assumptionStatus = 'provisional design-screening bounds';
cmgConfig.gimbal.rateTimeConstant = [0.01, 0.01]; % s; module rate servos
cmgConfig.gimbal.frameInertiaAllowance = 0.25; % fraction of rotor transverse inertia
cmgConfig.gimbal.motorRotorInertia = 264e-7; % kg m^2; representative EC-i 52

% Fixed fore/aft vortex-ring thrusters. These are provisional, cycle-averaged
% actuator assumptions for model development, not measured hardware ratings.
% Both thrust along body y. Common force produces sway; differential force
% produces yaw. Center-plane mounting produces no direct roll moment.
cmgConfig.thruster.enabled = false;             % enabled by later hybrid case
cmgConfig.thruster.commandMode = 'generalized_force';
cmgConfig.thruster.positionBody = ...
    [0.65,-0.65; 0,0; 0,0];                    % [fore,aft], m
cmgConfig.thruster.commandForce = [0;0];        % [fore;aft], N
cmgConfig.thruster.maxForce = 5;                % N/module, provisional
cmgConfig.thruster.timeConstant = 0.15;         % s, provisional force lag
cmgConfig.thruster.maxForceRate = 50;           % N/s/module, provisional
cmgConfig.thruster.modelStatus = ...
    'cycle-averaged provisional actuator model';

% Hybrid maneuver controller. The main baseline leaves it disabled; the
% coordinated validation script enables it after defining a maneuver plane,
% lateral displacement, and desired heading vector.
cmgConfig.hybrid.enabled = false;
cmgConfig.hybrid.rollEnableAngle = deg2rad(1.0);
cmgConfig.hybrid.rollDisableAngle = deg2rad(3.0);
cmgConfig.hybrid.rollEnableRate = deg2rad(1.0);
cmgConfig.hybrid.rollDisableRate = deg2rad(4.0);
cmgConfig.hybrid.KpLateral = 20; % N/m
cmgConfig.hybrid.KdLateral = 30; % N/(m/s)
cmgConfig.hybrid.KpHeading = 2.0; % N m/rad
cmgConfig.hybrid.KdHeading = 6.0; % N m/(rad/s)

% Dual-CMG initial state. Change the second speed to (for example) 9.9*pi
% to study asymmetric/non-uniform counter-rotation.
% A +/-15-deg preload retains two-axis steering rank while providing enough
% stored-momentum range for both positive and negative 90-deg roll maneuvers.
cmgConfig.initial.alpha = deg2rad([-15, 15]);  % [alpha1, alpha2] rad
cmgConfig.initial.Omega = [-40*pi, 40*pi];    % [Omega1, Omega2] rad/s (1200 rpm)

% Installed mass-property model. This publication-level baseline includes
% only the rotor mass/inertia already defined by the CMG model; no unselected
% motor, housing, or frame mass is invented. Dual modules are placed
% symmetrically on the centerline, so their first moments cancel. Mounting x
% affects pitch/yaw inertia through the parallel-axis theorem, not roll inertia.
cmgConfig.installation.includeMassProperties = true;
cmgConfig.installation.dualMountX = [-0.30, 0.30]; % m from vehicle origin
cmgConfig.installation.singleMountX = 0;           % m from vehicle origin
cmgConfig.installation.massScope = 'modeled rotor only';

% Diagnostic thresholds
cmgConfig.diagnostics.failOnNonfinite = true;
cmgConfig.diagnostics.conditionWarning = 1e3;
cmgConfig.diagnostics.magnitudeRatioWarning = 1e3;
cmgConfig.diagnostics.cancellationWarning = 1e3;
% Damped least-squares intentionally trades a small residual for robustness.
cmgConfig.diagnostics.solveResidualTolerance = 1e-3;

% Simulation case:
%   'VFR' - Vehicle From Rest: zero initial linear/angular velocity.
%   'FSV' - Forward Surge Velocity: initial surge velocity u = 3 m/s.
%   'STM' - Standard Turn Maneuver: u = 3 m/s and yaw rate r = 1.5 rad/s.
%   'SPF' - Simple Path Following: initial surge velocity u = 3 m/s.
simConfig.case = 'VFR';
% Command source:
%   'direct_roll'   - use simConfig.directRollAngle.
%   'roll_to_plane' - compute roll from an inertial lateral direction and
%                     the fixed, reversible body-y thruster axis.
simConfig.commandMode = 'roll_to_plane';
simConfig.directRollAngle = pi/2;
simConfig.rollToPlane.desiredLateralDirectionNED = [0;0;1]; % down
simConfig.rollToPlane.bidirectionalThruster = true;
simConfig.duration = 5;          % total validation window (s)
simConfig.targetRollTime = 3;    % desired 90-deg settling horizon (s)
% Krieg (2023) assigns 25% of each control cycle to the roll phase before
% yaw actuation. Capacity is reported for both the full cycle and this
% shorter reference roll stage; it does not change the current controller.
simConfig.referenceRollDutyCycle = 0.25;
simConfig.relTol = 1e-7;
simConfig.absTol = 1e-9;
simConfig.maxStep = 0.01;

plotConfig.allStates = true;
plotConfig.cmgStates = true;
plotConfig.rollConvergence = true;
plotConfig.eulerAngles = true;
plotConfig.individualTorques = true;
plotConfig.combinedTorques = true;
plotConfig.controlDiagnostics = true;
plotConfig.momentumCoupling = true;
plotConfig.energy = true;
plotConfig.exportEPS = true;
plotConfig.exportPNG = true;
% Figure placement (MATLAB R2025a or newer):
%   'container' - tabs in the external Figure Container (recommended)
%   'desktop'   - tabs docked inside the main MATLAB desktop
%   'normal'    - one independent window per figure
plotConfig.windowStyle = 'container';

%% VALIDATION AND OUTPUT DIRECTORY
cmgConfig.mode = validatestring(lower(cmgConfig.mode), {'single','dual'});
if strcmp(cmgConfig.mode, 'dual')
    cmgConfig.dualController = validatestring( ...
        lower(cmgConfig.dualController), {'constant_speed','legacy'});
end
simConfig.case = validatestring(upper(simConfig.case), ...
    {'VFR','FSV','STM','SPF'});

scriptDir = fileparts(mfilename('fullpath'));
if strcmp(cmgConfig.mode, 'dual')
    omegaMagnitudeDifference = abs(abs(cmgConfig.initial.Omega(1)) ...
        - abs(cmgConfig.initial.Omega(2)));
    if omegaMagnitudeDifference <= 10*eps(max(abs(cmgConfig.initial.Omega)))
        spinCase = 'symmetric_spin';
    else
        spinCase = 'asymmetric_spin';
    end
    outputDir = fullfile(scriptDir, 'Working Results', ...
        cmgConfig.mode, spinCase, simConfig.case);
else
    spinCase = 'single_spin';
    outputDir = fullfile(scriptDir, 'Working Results', ...
        cmgConfig.mode, simConfig.case);
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end
fprintf('Running %s-CMG mode, %s maneuver.\n', ...
    cmgConfig.mode, simConfig.case);

%% INITIAL CONDITIONS AND PARAMETERS
state = initializeVehicleState(simConfig.case);
[state, gyro1, gyro2] = initializeCMGs(state, cmgConfig);
cmgConfig.momentumManagement.enabled = false;
cmgConfig.momentumManagement.gain = 0.5;          % 1/s
cmgConfig.momentumManagement.maxDumpMoment = 0.05; % N m external authority
cmgConfig.momentumManagement.referenceHx = rotorRollMomentum( ...
    state,gyro1,gyro2);
cmgConfig.external.rollDisturbance = 0;            % N m
cmgConfig.gimbal.assemblyInertia = repmat( ...
    (1+cmgConfig.gimbal.frameInertiaAllowance)*gyro1.Itransverse ...
    + cmgConfig.gimbal.motorRotorInertia, 1, 2);

stateVec = [state.x; state.y; state.z; state.phi; state.theta; state.psi; ...
    state.u; state.v; state.w; state.p; state.q; state.r; ...
    state.alpha1; state.Omega1; state.alpha2; state.Omega2; ...
    state.alphadot1; state.alphadot2; 0; 0]; % states 19-20: VRT forces

d.phi = simConfig.directRollAngle;
d.theta = 0;
d.psi = 0;
d.thruster.Y = 0; % requested body sway force (N)
d.thruster.N = 0; % requested body yaw moment (N m)
rollToPlaneCommand = struct();
if strcmpi(simConfig.commandMode,'roll_to_plane')
    rollToPlaneCommand = ROLL_TO_PLANE_COMMAND( ...
        [state.phi;state.theta;state.psi], ...
        simConfig.rollToPlane.desiredLateralDirectionNED, ...
        state.phi,simConfig.rollToPlane.bidirectionalThruster);
    d.phi = rollToPlaneCommand.rollAngle;
    d.rollToPlane.desiredLateralDirectionNED = ...
        simConfig.rollToPlane.desiredLateralDirectionNED;
    d.rollToPlane.bidirectionalThruster = ...
        simConfig.rollToPlane.bidirectionalThruster;
elseif ~strcmpi(simConfig.commandMode,'direct_roll')
    error('Unsupported simConfig.commandMode: %s',simConfig.commandMode);
end

auv.W = 2.99e2;
auv.g = 9.81;
auv.m = auv.W / auv.g;
auv.D = 0.14732;
auv.d = auv.D - 0.0127;

gains.Kpu = 4;
gains.Kdu = 1;
% Single-CMG baseline gains selected by a bounded sweep of Kpp and damping
% ratio. The selected pair meets the 3 s, 2% settling requirement without
% violating the configured gimbal-rate or flywheel-acceleration limits.
% Kdp corresponds to zeta=0.8 for Ix=0.177 kg m^2 and Kpp=0.30 N m/rad.
% Kpp=0.28 was the lowest passing grid point but settled at 2.998 s, so
% Kpp=0.30 is retained to provide timing and actuator margin.
gains.Kpp = 0.30;
gains.Kdp = 0.3687;
gains.Kpr = 30;
gains.Kdr = 14;

loop.cycleT = simConfig.duration;
loop.fc = 1 / loop.cycleT;
loop.controlEndTime = inf; % hold roll feedback active after reaching target

% Prescribed unmodified-vehicle properties about the body-frame origin. The
% installed-mass assembly below adds each modeled rotor tensor and its
% parallel-axis contribution. This changes Ma; it is not an extra CMG torque.
baseMassProperties.m = auv.m;
baseMassProperties.cg = [0;0;0];
baseMassProperties.inertia = diag([1.77e-1, 3.45, 3.45]);
[params, massPropertyReport] = assembleInstalledMassProperties( ...
    baseMassProperties, gyro1, gyro2, state, cmgConfig);
auv.m = params.m;
auv.W = auv.m*auv.g;
params.rollDragCoefficient = -0.0013; % Kpp in K_hyd=Kpp*p*abs(p)

capacity = CMG_CAPACITY_ANALYSIS(gyro1, state.alpha1, state.Omega1, ...
    cmgConfig.limits, params.Ix, abs(d.phi-state.phi), ...
    simConfig.targetRollTime, simConfig.referenceRollDutyCycle, ...
    params.rollDragCoefficient);

%% SIMULATION
solverOptions = odeset('RelTol', simConfig.relTol, ...
    'AbsTol', simConfig.absTol, 'MaxStep', simConfig.maxStep);
tic;
[T_OUT, Y_OUT] = ode45(@CONTROL, [0 loop.cycleT], stateVec, solverOptions, ...
    gains, gyro1, gyro2, auv, params, d, loop, cmgConfig);
[tau_cmg1, tau_cmg2, controlHistory] = TORQUE( ...
    T_OUT, Y_OUT, gains, gyro1, gyro2, auv, params, d, loop, cmgConfig);
energy = ENERGY_ANALYSIS( ...
    T_OUT, Y_OUT, tau_cmg1, tau_cmg2, controlHistory, gyro1, gyro2);
envelope = CMG_ENVELOPE_ANALYSIS( ...
    T_OUT, Y_OUT, controlHistory, gyro1, cmgConfig.limits);
momentumCoupling = CMG_MOMENTUM_COUPLING_ANALYSIS( ...
    T_OUT, Y_OUT, tau_cmg1, tau_cmg2, gyro1, gyro2, params);
hardwareScreen = CMG_HARDWARE_SCREENING( ...
    Y_OUT, controlHistory, gyro1, cmgConfig);
if strcmp(cmgConfig.mode, 'dual')
    dualConditioning = DUAL_CMG_CONDITIONING_ANALYSIS( ...
        T_OUT, Y_OUT, controlHistory, gyro1, gyro2, ...
        cmgConfig.limits, cmgConfig.allocator);
else
    dualConditioning = struct();
end
elapsedTime = toc;

%% RUN SUMMARY
simSummary.mode = cmgConfig.mode;
simSummary.case = simConfig.case;
simSummary.spinCase = spinCase;
simSummary.elapsedTime = elapsedTime;
simSummary.finalRoll = Y_OUT(end,4);
simSummary.targetRoll = d.phi;
simSummary.finalRollError = d.phi - Y_OUT(end,4);
simSummary.maxAbsPitch = max(abs(Y_OUT(:,5)));
simSummary.maxAbsYaw = max(abs(Y_OUT(:,6)));
simSummary.allStatesFinite = all(isfinite(Y_OUT(:)));
simSummary.rollOvershootPercent = 100 * max(0, ...
    max(Y_OUT(:,4)) - d.phi) / abs(d.phi);
simSummary.settlingTime2Percent = calculateSettlingTime( ...
    T_OUT, Y_OUT(:,4), d.phi, 0.02);
simSummary.gimbalExcursion = [ ...
    max(Y_OUT(:,13))-min(Y_OUT(:,13)), ...
    max(Y_OUT(:,15))-min(Y_OUT(:,15))];
simSummary.cmg2StateChange = max(max(abs( ...
    Y_OUT(:,15:16) - Y_OUT(1,15:16))));
simSummary.peakGimbalRate = max(abs(controlHistory.alphadot), [], 1);
simSummary.peakGimbalAccel = max(abs(controlHistory.gimbalAccel), [], 1);
simSummary.peakFlywheelAccel = max(abs(controlHistory.Omegadot), [], 1);
simSummary.peakUnconstrainedGimbalRate = max( ...
    abs(controlHistory.unconstrainedAlphadot), [], 1);
simSummary.peakUnconstrainedFlywheelAccel = max( ...
    abs(controlHistory.unconstrainedOmegadot), [], 1);
simSummary.rollMomentRMSE = sqrt(mean(controlHistory.momentError(:,1).^2));
simSummary.pitchMomentRMSE = sqrt(mean(controlHistory.momentError(:,2).^2));
simSummary.allocatorRollRMSE = sqrt(mean( ...
    controlHistory.allocationError(:,1).^2));
simSummary.actuatorLimitRollRMSE = sqrt(mean( ...
    controlHistory.actuatorLimitLoss(:,1).^2));
simSummary.initialRequestedRollMoment = controlHistory.requestedMoment(1,1);
simSummary.initialAvailableRollMoment = gyro1.I * (...
    abs(state.Omega1*cos(state.alpha1))*cmgConfig.limits.maxGimbalRate ...
    + abs(sin(state.alpha1))*cmgConfig.limits.maxFlywheelAccel);
if strcmp(cmgConfig.mode, 'dual')
    simSummary.initialAvailableRollMoment = ...
        simSummary.initialAvailableRollMoment + gyro2.I * (...
        abs(state.Omega2*cos(state.alpha2))*cmgConfig.limits.maxGimbalRate ...
        + abs(sin(state.alpha2))*cmgConfig.limits.maxFlywheelAccel);
end
simSummary.initialRollDemandCapacityRatio = ...
    abs(simSummary.initialRequestedRollMoment) ...
    / max(simSummary.initialAvailableRollMoment, eps);
simSummary.rateSaturationCount = nnz( ...
    controlHistory.gimbalRateSaturated);
simSummary.gimbalAccelSaturationCount = nnz( ...
    controlHistory.gimbalAccelSaturated);
simSummary.flywheelAccelSaturationCount = nnz( ...
    controlHistory.flywheelAccelSaturated);
simSummary.angleLimitCount = nnz(controlHistory.gimbalAngleLimited);
conditionValues = controlHistory.conditionNumber( ...
    isfinite(controlHistory.conditionNumber));
if isempty(conditionValues)
    simSummary.maxConditionNumber = nan;
else
    simSummary.maxConditionNumber = max(conditionValues);
end
simSummary.maxSolveResidual = finiteMaximum(controlHistory.solveResidual);
simSummary.maxColumnMagnitudeRatio = ...
    finiteMaximum(controlHistory.columnMagnitudeRatio);
simSummary.maxCancellationIndex = ...
    finiteMaximum(controlHistory.cancellationIndex);
simSummary.nonfiniteStateCount = nnz(~isfinite(Y_OUT));
simSummary.omegaAsymmetryPercent = 100 * ...
    abs(abs(state.Omega1)-abs(state.Omega2)) ...
    / max(mean(abs([state.Omega1, state.Omega2])), eps);
simSummary.rollEnergyGross = energy.output.rollGross;
simSummary.idealInputEnergyGross = energy.input.idealSystemGross;
simSummary.idealEfficiencyPercent = ...
    energy.efficiency.idealSystemPercent;
simSummary.idealEfficiencyIncludingSpinupPercent = ...
    energy.efficiency.idealIncludingSpinupPercent;
simSummary.capacity = capacity;
simSummary.envelope = envelope;
simSummary.momentumCoupling = momentumCoupling;
simSummary.hardwareScreen = hardwareScreen;
simSummary.dualConditioning = dualConditioning;
simSummary.massProperties = massPropertyReport;

fprintf(['Elapsed: %.3f s | final roll: %.6f rad | roll error: %.6f rad | ' ...
    'max |pitch|: %.6f rad | max |yaw|: %.6f rad | finite: %d\n'], ...
    simSummary.elapsedTime, simSummary.finalRoll, ...
    simSummary.finalRollError, simSummary.maxAbsPitch, ...
    simSummary.maxAbsYaw, simSummary.allStatesFinite);
fprintf(['Installed mass properties (%s): mass %.3f -> %.3f kg | ' ...
    'Ix %.6f -> %.6f | Iy %.6f -> %.6f | Iz %.6f -> %.6f kg m^2\n'], ...
    cmgConfig.installation.massScope, massPropertyReport.baseMass, ...
    massPropertyReport.assembledMass, massPropertyReport.baseInertia(1,1), ...
    massPropertyReport.assembledInertia(1,1), ...
    massPropertyReport.baseInertia(2,2), ...
    massPropertyReport.assembledInertia(2,2), ...
    massPropertyReport.baseInertia(3,3), ...
    massPropertyReport.assembledInertia(3,3));
if strcmp(cmgConfig.mode, 'single')
    fprintf('Inactive CMG #2 maximum state change: %.3e\n', ...
        simSummary.cmg2StateChange);
end
fprintf(['Peak gimbal rates: [%.3f, %.3f] rad/s | roll-moment RMSE: ' ...
    '%.3e N m | rate/accel limited samples: %d/%d\n'], ...
    simSummary.peakGimbalRate(1), simSummary.peakGimbalRate(2), ...
    simSummary.rollMomentRMSE, simSummary.rateSaturationCount, ...
    simSummary.gimbalAccelSaturationCount);
fprintf(['Unconstrained peak gimbal rates: [%.3f, %.3f] rad/s | ' ...
    'allocator roll RMSE: %.3e N m | actuator-limit loss RMSE: %.3e N m\n'], ...
    simSummary.peakUnconstrainedGimbalRate(1), ...
    simSummary.peakUnconstrainedGimbalRate(2), ...
    simSummary.allocatorRollRMSE, simSummary.actuatorLimitRollRMSE);
fprintf(['Initial roll request: %.3f N m | available bounded roll moment: ' ...
    '%.3f N m | demand/capacity: %.2f | flywheel-accel saturations: %d\n'], ...
    simSummary.initialRequestedRollMoment, ...
    simSummary.initialAvailableRollMoment, ...
    simSummary.initialRollDemandCapacityRatio, ...
    simSummary.flywheelAccelSaturationCount);
fprintf(['Per-module instantaneous capacity: %.3f N m | max initial roll ' ...
    'acceleration: %.3f rad/s^2 | ideal minimum 90-deg time: %.3f s\n'], ...
    capacity.instantaneousRollTorque, capacity.maximumRollAcceleration, ...
    capacity.minimumBangBangTime);
fprintf(['Smooth 90-deg rest-to-rest demand: %.3f N m over %.2f s ' ...
    '(margin %.2fx); %.3f N m over %.2f s reference roll stage ' ...
    '(margin %.2fx)\n'], capacity.fullCycle.requiredTorque, ...
    capacity.fullCycle.duration, capacity.fullCycle.torqueMargin, ...
    capacity.referenceRollStage.requiredTorque, ...
    capacity.referenceRollStage.duration, ...
    capacity.referenceRollStage.torqueMargin);
fprintf(['CMG #1 envelope: alpha excursion %.1f deg | Omega %.3f to ' ...
    '%.3f rad/s | minimum |Omega| %.3f rad/s | zero crossings %d\n'], ...
    rad2deg(envelope.gimbalExcursion), envelope.minimumOmega, ...
    envelope.maximumOmega, envelope.minimumAbsOmega, ...
    envelope.omegaZeroCrossings);
fprintf(['Unconstrained utilization: gimbal rate %.2fx | flywheel accel ' ...
    '%.2fx | moment sustained without limiting: %d\n'], ...
    envelope.peakGimbalRateUtilization, ...
    envelope.peakFlywheelAccelUtilization, ...
    envelope.sustainsRequestedMomentWithoutLimiting);
fprintf(['Provisional position/speed utilization: gimbal angle %.1f %% ' ...
    '(%.1f deg peak) | flywheel speed %.1f %% (%.1f rpm peak)\n'], ...
    100*envelope.peakGimbalAngleUtilization, ...
    rad2deg(envelope.maximumAbsAlpha), ...
    100*envelope.peakFlywheelSpeedUtilization, ...
    envelope.maximumAbsOmega*60/(2*pi));
fprintf(['Momentum exchange: peak vehicle Hx %.6f N m s | peak opposite ' ...
    'rotor Delta-Hx %.6f N m s | external-moment-corrected residual ' ...
    '%.3e N m s (%.3f %%)\n'], momentumCoupling.peakVehicleRollMomentum, ...
    momentumCoupling.peakOppositeRotorRollMomentumChange, ...
    momentumCoupling.maxRollMomentumBalanceResidual, ...
    100*momentumCoupling.normalizedRollMomentumBalanceResidual);
fprintf(['Cross-axis coupling: peak CMG M %.3e N m | peak CMG N %.3e N m ' ...
    '| peak rigid-body pitch coupling %.3e N m\n'], ...
    momentumCoupling.peakAbsCmgPitchMoment, ...
    momentumCoupling.peakAbsCmgYawMoment, ...
    momentumCoupling.peakAbsRigidBodyPitchCouplingMoment);
fprintf(['Flywheel drive requirement: %.3f N m peak torque | %.1f W peak ' ...
    'mechanical power | representative motor screen: %s\n'], ...
    hardwareScreen.flywheel.peakTorque, ...
    hardwareScreen.flywheel.peakMechanicalPower, ...
    string(hardwareScreen.flywheel.representativeMotorPass));
fprintf(['Gimbal drive requirement: %.3f N m peak torque | %.1f W peak ' ...
    'mechanical power | %.1f rad/s^2 peak acceleration | screen: %s\n'], ...
    hardwareScreen.gimbal.peakTorque, ...
    hardwareScreen.gimbal.peakMechanicalPower, ...
    hardwareScreen.gimbal.peakAccel, ...
    string(hardwareScreen.gimbal.representativeMotorValidated));
fprintf('Hardware validation status: %s.\n', hardwareScreen.overallStatus);
if envelope.gimbalAngleLimitUndefined
    warning('CMG:UndefinedGimbalAngleLimit', ...
        ['No finite gimbal-angle limit is configured; the %.1f deg ' ...
        'excursion has not been checked against hardware.'], ...
        rad2deg(envelope.gimbalExcursion));
end
if envelope.flywheelSpeedLimitUndefined
    warning('CMG:UndefinedFlywheelSpeedLimit', ...
        ['No finite flywheel-speed limit is configured; the %.1f rpm ' ...
        'peak has not been checked against hardware.'], ...
        envelope.maximumAbsOmega*60/(2*pi));
end
if strcmp(cmgConfig.mode, 'dual')
    fprintf(['Initial Omega: [%.6f, %.6f] rad/s | magnitude asymmetry: ' ...
        '%.3f %%\n'], state.Omega1, state.Omega2, ...
        simSummary.omegaAsymmetryPercent);
    fprintf(['Max condition: %.3e | max solve residual: %.3e | ' ...
        'max column ratio: %.3e | max cancellation index: %.3e | ' ...
        'near-singular samples: %d\n'], simSummary.maxConditionNumber, ...
        simSummary.maxSolveResidual, simSummary.maxColumnMagnitudeRatio, ...
        simSummary.maxCancellationIndex, nnz(controlHistory.nearSingularity));
    fprintf(['Steering path: minimum singularity distance %.2f deg | ' ...
        'minimum sigma %.3e N m s | minimum active roll-capacity margin ' ...
        '%.2fx | rate-infeasible samples %d | full-K/M pass %d | ' ...
        'roll-task pass %d\n'], ...
        rad2deg(dualConditioning.minimumSingularityDistance), ...
        dualConditioning.minimumSigma, ...
        dualConditioning.minimumActiveCapacityMargin, ...
        dualConditioning.rateInfeasibleSamples, ...
        dualConditioning.passesFullKM, dualConditioning.passesRollTask);
end
if simSummary.rateSaturationCount > 0
    warning('CMG:RateSaturation', ...
        'Gimbal-rate saturation occurred at %d saved samples.', ...
        simSummary.rateSaturationCount);
end
if strcmp(cmgConfig.mode, 'dual')
    issueThresholdWarnings(simSummary, cmgConfig.diagnostics, ...
        dualConditioning);
end
fprintf(['Gross roll work: %.6f J | ideal gross actuator-energy proxy: ' ...
    '%.6f J | ideal transfer metric: %.3f %% | including initial spin: ' ...
    '%.3f %%\n'], ...
    simSummary.rollEnergyGross, simSummary.idealInputEnergyGross, ...
    simSummary.idealEfficiencyPercent, ...
    simSummary.idealEfficiencyIncludingSpinupPercent);
if strcmpi(simConfig.commandMode,'roll_to_plane')
    fprintf(['Roll-to-plane command: %.2f deg | thrust polarity %+d | ' ...
        'geometric alignment error %.3e deg\n'], ...
        rad2deg(rollToPlaneCommand.rollAngle), ...
        rollToPlaneCommand.thrusterPolarity, ...
        rad2deg(rollToPlaneCommand.alignmentAngle));
end

runMetadata.timestamp = char(datetime('now', ...
    'Format', 'yyyy-MM-dd HH:mm:ss Z'));
runMetadata.matlabVersion = version;
runMetadata.gitCommit = getGitCommit(scriptDir);
save(fullfile(outputDir, 'simulation_result.mat'), ...
    'T_OUT', 'Y_OUT', 'tau_cmg1', 'tau_cmg2', 'controlHistory', 'energy', ...
    'simSummary', 'cmgConfig', 'simConfig', 'plotConfig', ...
    'gains', 'gyro1', 'gyro2', 'auv', 'params', 'd', 'capacity', ...
    'envelope', 'momentumCoupling', 'hardwareScreen', 'dualConditioning', ...
    'massPropertyReport', 'baseMassProperties', 'rollToPlaneCommand', ...
    'runMetadata');

%% PLOTS
plotSimulationResults(T_OUT, Y_OUT, tau_cmg1, tau_cmg2, ...
    controlHistory, energy, momentumCoupling, dualConditioning, d, ...
    cmgConfig, plotConfig, outputDir);

%% LOCAL FUNCTIONS
function state = initializeVehicleState(simCase)
    state.x = 0; state.y = 0; state.z = 0;
    state.phi = 0; state.theta = 0; state.psi = 0;
    state.u = 0; state.v = 0; state.w = 0;
    state.p = 0; state.q = 0; state.r = 0;

    switch simCase
        case 'VFR'
            % Vehicle from rest: defaults above.
        case 'FSV'
            state.u = 3;
        case 'STM'
            state.u = 3;
            state.r = 1.5;
        case 'SPF'
            state.u = 3;
    end
end

function [state, gyro1, gyro2] = initializeCMGs(state, config)
    % Both modules use identical rotor properties. No mounting-position
    % vectors are defined: "1" and "2" are module identifiers only.
    gyro1 = makeFlywheel(0.0508, 0.0127, 7750);
    gyro2 = makeFlywheel(0.0508, 0.0127, 7750);

    if strcmp(config.mode, 'single')
        state.alpha1 = 0;
        state.Omega1 = 10*pi;
        state.alpha2 = 0;
        state.Omega2 = 0;
    else
        state.alpha1 = config.initial.alpha(1);
        state.Omega1 = config.initial.Omega(1);
        state.alpha2 = config.initial.alpha(2);
        state.Omega2 = config.initial.Omega(2);
    end
    state.alphadot1 = 0;
    state.alphadot2 = 0;
end

function gyro = makeFlywheel(radius, thickness, density)
    % Axial rotor inertia used to calculate spin angular momentum H = I*Omega.
    % This is not the CMG module's full 3-D inertia tensor and is not shifted
    % to the vehicle origin with the parallel-axis theorem.
    gyro.r = radius;
    gyro.t = thickness;
    gyro.rho = density;
    gyro.v = pi * radius^2 * thickness;
    gyro.m = gyro.v * density;
    gyro.I = 0.5 * gyro.m * radius^2;
    gyro.Itransverse = gyro.m*(3*radius^2+thickness^2)/12;
end

function Hx = rotorRollMomentum(state,gyro1,gyro2)
    Hx = gyro1.I*state.Omega1*sin(state.alpha1) ...
        + gyro2.I*state.Omega2*sin(state.alpha2);
end

function [params, report] = assembleInstalledMassProperties( ...
        base, gyro1, gyro2, state, config)
    if ~config.installation.includeMassProperties
        params.m = base.m;
        params.Ix = base.inertia(1,1);
        params.Iy = base.inertia(2,2);
        params.Iz = base.inertia(3,3);
        params.xg = base.cg(1); params.yg = base.cg(2); params.zg = base.cg(3);
        report = makeMassReport(base, params, zeros(0,3), zeros(0,1));
        return;
    end

    if strcmp(config.mode,'single')
        gyros = gyro1;
        alphas = state.alpha1;
        mountX = config.installation.singleMountX;
    else
        gyros = [gyro1,gyro2];
        alphas = [state.alpha1,state.alpha2];
        mountX = config.installation.dualMountX;
    end

    moduleCount = numel(gyros);
    moduleMasses = zeros(moduleCount,1);
    modulePositions = zeros(moduleCount,3);
    moduleInertias = zeros(3,3,moduleCount);
    for module = 1:moduleCount
        moduleMasses(module) = gyros(module).m;
        modulePositions(module,:) = [mountX(module),0,0];
        spinAxis = [sin(alphas(module));-cos(alphas(module));0];
        moduleInertias(:,:,module) = gyros(module).Itransverse*eye(3) ...
            + (gyros(module).I-gyros(module).Itransverse) ...
            *(spinAxis*spinAxis.');
    end

    [mass,cg,inertia] = ASSEMBLE_VEHICLE_MASS_PROPERTIES( ...
        base.m, base.cg, base.inertia, moduleMasses, ...
        modulePositions, moduleInertias);
    offDiagonal = inertia-diag(diag(inertia));
    if max(abs(offDiagonal),[],'all') > 1e-10
        error('CMG:NonDiagonalInstalledInertia', ...
            ['Installed inertia has non-negligible products of inertia. ' ...
             'REMUS must be generalized before using this arrangement.']);
    end
    params.m = mass;
    params.Ix = inertia(1,1);
    params.Iy = inertia(2,2);
    params.Iz = inertia(3,3);
    params.xg = cg(1); params.yg = cg(2); params.zg = cg(3);
    report = makeMassReport(base, params, modulePositions, moduleMasses);
    report.assembledCG = cg;
    report.assembledInertia = inertia;
end

function report = makeMassReport(base, params, positions, masses)
    report.baseMass = base.m;
    report.baseCG = base.cg;
    report.baseInertia = base.inertia;
    report.assembledMass = params.m;
    report.assembledCG = [params.xg;params.yg;params.zg];
    report.assembledInertia = diag([params.Ix,params.Iy,params.Iz]);
    report.modulePositions = positions;
    report.moduleMasses = masses;
end

function plotSimulationResults(T, Y, tau1, tau2, control, energy, momentum, ...
        dualConditioning, d, config, plots, outputDir)
    % MATLAB R2025a+ uses an external, tabbed Figure Container by default.
    % Older startup files commonly force DefaultFigureWindowStyle='normal',
    % which silently restores one independent window per figure. Temporarily
    % remove that session override while this suite creates its figures, then
    % restore the user's original setting when plotting is complete.
    figureDefaultCleanup = configureFigureEnvironment(plots); %#ok<NASGU>

    labels = {'x','y','z','\phi','\theta','\psi','u','v','w','p','q','r', ...
        '\alpha_1','\Omega_1','\alpha_2','\Omega_2', ...
        '\dot{\alpha}_1','\dot{\alpha}_2','F_{VRT,f}','F_{VRT,a}'};
    torqueTime = T;

    if plots.allStates
        fig = createFigure('All simulation states', plots);
        hold on;
        stateIndices = [1:14,17,19,20];
        if strcmp(config.mode, 'dual')
            stateIndices = 1:20;
        end
        for index = stateIndices
            plot(T, Y(:,index), 'LineWidth', 2, 'DisplayName', labels{index});
        end
        hold off;
        formatAxes('Control Cycle Time (s)', 'Magnitude', ...
            sprintf('All Simulation States (%s CMG)', config.mode));
        legend('Location', 'eastoutside');
        exportFigure(fig, outputDir, 'ARC_ALL', plots);
    end

    if plots.cmgStates
        fig = createFigure('CMG states', plots);
        subplot(2,1,1);
        plot(T, Y(:,13), 'LineWidth', 2, 'DisplayName', '\alpha_1');
        hold on;
        if strcmp(config.mode, 'dual')
            plot(T, Y(:,15), 'LineWidth', 2, 'DisplayName', '\alpha_2');
        end
        hold off;
        formatAxes('', '\alpha (rad)', 'CMG States');
        legend('Location', 'best');

        subplot(2,1,2);
        plot(T, Y(:,14), 'LineWidth', 2, 'DisplayName', '\Omega_1');
        hold on;
        if strcmp(config.mode, 'dual')
            plot(T, Y(:,16), 'LineWidth', 2, 'DisplayName', '\Omega_2');
        end
        hold off;
        formatAxes('Control Cycle Time (s)', '\Omega (rad/s)', '');
        legend('Location', 'best');
        exportFigure(fig, outputDir, 'ARC_GYRO2', plots);
    end

    if plots.rollConvergence
        fig = createFigure('Roll convergence', plots);
        plot(T, Y(:,4), 'LineWidth', 2, 'DisplayName', '\phi');
        hold on;
        yline(d.phi, '--k', 'LineWidth', 2, 'DisplayName', '\phi_d');
        hold off;
        formatAxes('Control Cycle Time (s)', 'Angle (rad)', 'Roll Convergence');
        legend('Location', 'best');
        exportFigure(fig, outputDir, 'ARC_ROLL', plots);
    end

    if plots.eulerAngles
        fig = createFigure('Euler angles', plots);
        eulerLabels = {'\phi','\theta','\psi'};
        for index = 1:3
            subplot(3,1,index);
            plot(T, Y(:,index+3), 'LineWidth', 2);
            xLabel = '';
            if index == 3, xLabel = 'Control Cycle Time (s)'; end
            titleText = '';
            if index == 1, titleText = 'Euler Angles'; end
            formatAxes(xLabel, sprintf('%s (rad)', eulerLabels{index}), titleText);
        end
        exportFigure(fig, outputDir, 'ARC_EULER2', plots);
    end

    if plots.individualTorques
        plotTorquePair(torqueTime, tau1, 'CMG #1', outputDir, ...
            'ARC_CMG1', 'ARC_CMG2', plots);
        if strcmp(config.mode, 'dual')
            plotTorquePair(torqueTime, tau2, 'CMG #2', outputDir, ...
                'ARC_CMG3', 'ARC_CMG4', plots);
        end
    end

    if plots.combinedTorques && strcmp(config.mode, 'dual')
        total.K = tau1.K + tau2.K;
        total.M = tau1.M + tau2.M;
        total.N = tau1.N + tau2.N;
        plotTorquePair(torqueTime, total, 'Combined CMG', outputDir, ...
            'ARC_CMG5', 'ARC_CMG6', plots);
    end

    if plots.controlDiagnostics
        fig = createFigure('Moment tracking', plots);
        subplot(2,1,1);
        plot(T, control.requestedMoment(:,1), '--k', 'LineWidth', 2, ...
            'DisplayName', 'Requested K');
        hold on;
        plot(T, control.unconstrainedAchievedMoment(:,1), ':', ...
            'LineWidth', 2, 'DisplayName', 'Unconstrained K');
        plot(T, control.achievedMoment(:,1), 'LineWidth', 2, ...
            'DisplayName', 'Limited K');
        hold off;
        formatAxes('', 'Roll moment (N m)', 'CMG Moment Tracking');
        legend('Location', 'best');

        subplot(2,1,2);
        plot(T, control.requestedMoment(:,2), '--k', 'LineWidth', 2, ...
            'DisplayName', 'Requested M');
        hold on;
        plot(T, control.unconstrainedAchievedMoment(:,2), ':', ...
            'LineWidth', 2, 'DisplayName', 'Unconstrained M');
        plot(T, control.achievedMoment(:,2), 'LineWidth', 2, ...
            'DisplayName', 'Limited M');
        hold off;
        formatAxes('Control Cycle Time (s)', 'Pitch moment (N m)', '');
        legend('Location', 'best');
        exportFigure(fig, outputDir, 'CONTROL_MOMENT_TRACKING', plots);

        if strcmp(config.mode, 'dual')
            fig = createFigure('Allocation conditioning', plots);
            subplot(3,1,1);
            semilogy(T, control.conditionNumber, 'LineWidth', 2);
            formatAxes('', 'Condition number', 'Dual-CMG Allocation Diagnostics');
            subplot(3,1,2);
            semilogy(T, abs(control.unconstrainedAlphadot), 'LineWidth', 2);
            formatAxes('', '|Unconstrained rate| (rad/s)', '');
            legend('\alpha_1 dot', '\alpha_2 dot', 'Location', 'best');
            subplot(3,1,3);
            semilogy(T, control.cancellationIndex, 'LineWidth', 2);
            formatAxes('Control Cycle Time (s)', 'Cancellation index', '');
            exportFigure(fig, outputDir, 'CONTROL_CONDITIONING', plots);

            fig = createFigure('Allocation moment terms', plots);
            termLabels = {'CMG 1 gimbal','CMG 2 gimbal','Body-rate bias'};
            subplot(2,1,1);
            plot(T, control.rollMomentTerms, 'LineWidth', 2);
            formatAxes('', 'Roll moment (N m)', 'Allocation Term Contributions');
            legend(termLabels, 'Location', 'best');
            subplot(2,1,2);
            plot(T, control.pitchMomentTerms, 'LineWidth', 2);
            formatAxes('Control Cycle Time (s)', 'Pitch moment (N m)', '');
            legend(termLabels, 'Location', 'best');
            exportFigure(fig, outputDir, 'CONTROL_MOMENT_TERMS', plots);

            fig = createFigure('Dual-CMG steering margin', plots);
            subplot(2,1,1);
            plot(T, rad2deg(dualConditioning.singularityDistance), ...
                'LineWidth', 2);
            formatAxes('', 'Distance (deg)', ...
                'Distance to Parallel/Antiparallel Steering Geometry');
            subplot(2,1,2);
            plot(T, dualConditioning.pitchNeutralRollCapacity, ...
                'LineWidth', 2, 'DisplayName', 'Available pitch-neutral |K|');
            hold on;
            plot(T, abs(control.requestedMoment(:,1)), '--', ...
                'LineWidth', 2, 'DisplayName', 'Requested |K|');
            hold off;
            formatAxes('Control Cycle Time (s)', 'Roll moment (N m)', ...
                'Directional Roll Capacity');
            legend('Location', 'best');
            exportFigure(fig, outputDir, 'DUAL_CMG_STEERING_MARGIN', plots);
        else
            fig = createFigure('Single-CMG actuator envelope', plots);
            subplot(3,1,1);
            plot(T, control.unconstrainedAlphadot(:,1), 'LineWidth', 2);
            hold on;
            yline(config.limits.maxGimbalRate, '--k');
            yline(-config.limits.maxGimbalRate, '--k');
            hold off;
            formatAxes('', 'alpha dot (rad/s)', ...
                'Single-CMG Unconstrained Commands and States');
            subplot(3,1,2);
            plot(T, control.unconstrainedOmegadot(:,1), 'LineWidth', 2);
            hold on;
            yline(config.limits.maxFlywheelAccel, '--k');
            yline(-config.limits.maxFlywheelAccel, '--k');
            hold off;
            formatAxes('', 'Omega dot (rad/s^2)', '');
            subplot(3,1,3);
            yyaxis left;
            plot(T, Y(:,13), 'LineWidth', 2);
            ylabel('alpha (rad)');
            yyaxis right;
            plot(T, Y(:,14), 'LineWidth', 2);
            ylabel('Omega (rad/s)');
            grid on;
            grid minor;
            xlabel('Control Cycle Time (s)');
            set(gca, 'FontSize', 16, 'LineWidth', 1.0);
            exportFigure(fig, outputDir, 'SINGLE_CMG_ENVELOPE', plots);
        end
    end

    if plots.energy
        grossInputPower = abs(energy.power.flywheel1) ...
            + abs(energy.power.flywheel2) ...
            + energy.power.idealGimbalGross1 ...
            + energy.power.idealGimbalGross2;
        fig = createFigure('Energy and ideal efficiency', plots);
        subplot(2,1,1);
        plot(T, abs(energy.power.rollTotal), 'LineWidth', 2, ...
            'DisplayName', 'Gross roll-output power');
        hold on;
        plot(T, grossInputPower, 'LineWidth', 2, ...
            'DisplayName', 'Ideal gross actuator-power proxy');
        hold off;
        formatAxes('', 'Power (W)', 'CMG Energy Accounting');
        legend('Location', 'best');

        subplot(2,1,2);
        plot(T, cumtrapz(T, abs(energy.power.rollTotal)), ...
            'LineWidth', 2, 'DisplayName', 'Gross roll work');
        hold on;
        plot(T, cumtrapz(T, grossInputPower), ...
            'LineWidth', 2, 'DisplayName', 'Ideal gross input proxy');
        hold off;
        formatAxes('Control Cycle Time (s)', 'Energy (J)', '');
        legend('Location', 'best');
        exportFigure(fig, outputDir, 'ENERGY_ACCOUNTING', plots);
    end

    if plots.momentumCoupling && strcmp(config.mode, 'single')
        fig = createFigure('Momentum exchange and cross-axis coupling', plots);
        subplot(2,1,1);
        plot(T, momentum.vehicleRollMomentumChange, 'LineWidth', 2, ...
            'DisplayName', 'Vehicle Delta H_x');
        hold on;
        plot(T, momentum.oppositeRotorRollMomentumChange, '--', ...
            'LineWidth', 2, 'DisplayName', '-Rotor Delta H_x');
        plot(T, momentum.oppositeRotorPlusExternalImpulse, ':', ...
            'LineWidth', 2, 'DisplayName', '-Rotor Delta H_x + external impulse');
        hold off;
        formatAxes('', 'Angular momentum (N m s)', ...
            'Single-CMG Momentum Exchange');
        legend('Location', 'best');

        subplot(2,1,2);
        plot(T, momentum.cmgPitchMoment, 'LineWidth', 2, ...
            'DisplayName', 'CMG M');
        hold on;
        plot(T, momentum.cmgYawMoment, 'LineWidth', 2, ...
            'DisplayName', 'CMG N');
        plot(T, momentum.rigidBodyPitchCouplingMoment, '--', ...
            'LineWidth', 2, 'DisplayName', '(I_z-I_x)pr');
        hold off;
        formatAxes('Control Cycle Time (s)', 'Moment (N m)', ...
            'Cross-Axis Terms');
        legend('Location', 'best');
        exportFigure(fig, outputDir, 'MOMENTUM_COUPLING', plots);
    end
end

function plotTorquePair(time, torque, titlePrefix, outputDir, ...
        overlayName, subplotName, plots)
    fig = createFigure([titlePrefix ' torques'], plots);
    plot(time, torque.K, 'LineWidth', 2, 'DisplayName', 'K');
    hold on;
    plot(time, torque.M, 'LineWidth', 2, 'DisplayName', 'M');
    plot(time, torque.N, 'LineWidth', 2, 'DisplayName', 'N');
    hold off;
    formatAxes('Control Cycle Time (s)', 'Torque (N m)', [titlePrefix ' Torques']);
    legend('Location', 'best');
    exportFigure(fig, outputDir, overlayName, plots);

    fig = createFigure([titlePrefix ' torque components'], plots);
    components = {'K','M','N'};
    for index = 1:3
        subplot(3,1,index);
        plot(time, torque.(components{index}), 'LineWidth', 2);
        xLabel = '';
        if index == 3, xLabel = 'Control Cycle Time (s)'; end
        titleText = '';
        if index == 1, titleText = [titlePrefix ' Torques']; end
        formatAxes(xLabel, sprintf('%s (N m)', components{index}), titleText);
    end
    exportFigure(fig, outputDir, subplotName, plots);
end

function cleanup = configureFigureEnvironment(plots)
    cleanup = [];
    if ~strcmpi(plots.windowStyle, 'container') || ~usejava('desktop')
        return;
    end

    previousWindowStyle = get(groot, 'DefaultFigureWindowStyle');
    set(groot, 'DefaultFigureWindowStyle', 'remove');
    cleanup = onCleanup(@() set(groot, 'DefaultFigureWindowStyle', ...
        previousWindowStyle));
end

function formatAxes(xLabel, yLabel, titleText)
    grid on;
    grid minor;
    xlabel(xLabel);
    ylabel(yLabel);
    title(titleText);
    set(gca, 'FontSize', 16, 'LineWidth', 1.0);
end

function fig = createFigure(name, plots)
    switch lower(plots.windowStyle)
        case 'container'
            % In R2025a+, WindowStyle='docked' explicitly places the figure
            % in the shared Figure Container. The container itself can remain
            % undocked from (external to) the main MATLAB desktop.
            fig = figure('Name', name, 'NumberTitle', 'off', ...
                'WindowStyle', 'docked');
        case 'desktop'
            fig = figure('Name', name, 'NumberTitle', 'off');
            if usejava('desktop')
                set(fig, 'WindowStyle', 'docked');
            end
        case 'normal'
            fig = figure('Name', name, 'NumberTitle', 'off', ...
                'WindowStyle', 'normal');
        otherwise
            error('Unsupported plotConfig.windowStyle: %s', ...
                plots.windowStyle);
    end
end

function exportFigure(fig, outputDir, baseName, plots)
    set(fig, 'ToolBar', 'none');
    drawnow;
    outputBase = fullfile(outputDir, baseName);
    if plots.exportEPS
        print(fig, '-depsc', [outputBase '.eps']);
    end
    if plots.exportPNG
        print(fig, '-dpng', [outputBase '.png']);
    end
end

function commit = getGitCommit(repositoryDir)
    command = sprintf('git -C "%s" rev-parse --short HEAD', repositoryDir);
    [status, output] = system(command);
    if status == 0
        commit = strtrim(output);
    else
        commit = 'unavailable';
    end
end

function settlingTime = calculateSettlingTime(time, response, target, fraction)
    tolerance = fraction * max(abs(target), eps);
    outsideBand = find(abs(response - target) > tolerance, 1, 'last');
    if isempty(outsideBand)
        settlingTime = time(1);
    elseif outsideBand < numel(time)
        settlingTime = time(outsideBand + 1);
    else
        settlingTime = nan;
    end
end

function maximum = finiteMaximum(values)
    values = values(isfinite(values));
    if isempty(values)
        maximum = nan;
    else
        maximum = max(values);
    end
end

function issueThresholdWarnings(summary, thresholds, conditioning)
    if summary.maxConditionNumber > thresholds.conditionWarning
        if conditioning.requiresSingularityAvoidance
            warning('CMG:RollTaskSingularity', ...
                ['Allocation condition number reached %.3e and the ' ...
                'requested roll moment exceeded pitch-neutral capacity. ' ...
                'A steering avoidance or command-limiting law is required.'], ...
                summary.maxConditionNumber);
        else
            warning('CMG:FormalKMSingularity', ...
                ['Full K/M allocation condition number reached %.3e, but ' ...
                'the active roll task retained feasible authority. No ' ...
                'roll-task singularity avoidance is required.'], ...
                summary.maxConditionNumber);
        end
    end
    if summary.maxColumnMagnitudeRatio > thresholds.magnitudeRatioWarning
        warning('CMG:TermMagnitudeMismatch', ...
            'Allocation-column magnitude ratio reached %.3e.', ...
            summary.maxColumnMagnitudeRatio);
    end
    if summary.maxCancellationIndex > thresholds.cancellationWarning
        warning('CMG:TermCancellation', ...
            'Allocation cancellation index reached %.3e.', ...
            summary.maxCancellationIndex);
    end
    if summary.maxSolveResidual > thresholds.solveResidualTolerance
        warning('CMG:AllocationResidual', ...
            'Normalized simultaneous-solve residual reached %.3e.', ...
            summary.maxSolveResidual);
    end
end
