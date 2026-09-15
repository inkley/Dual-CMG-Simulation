function [Etadot, controlData] = CONTROL(t, state, gains, gyro1, gyro2, ...
        auv, params, d, loop, cmgConfig)
%CONTROL Evaluate the controller, CMG dynamics, and vehicle dynamics.
% CMG outputs are body-axis reaction couples. CONTROL does not translate
% those moments from physical mounting points; installed-module mass effects
% must already be represented by the vehicle properties passed to REMUS.

[tauC, commandedContpar, allocation] = CMG_ALLOCATE( ...
    t, state, gains, gyro1, gyro2, d, loop, cmgConfig);
[contpar, actuator] = applyGimbalActuatorDynamics( ...
    commandedContpar, state, cmgConfig);
[tau_cmg1, tau_cmg2] = CMG(gyro1, gyro2, contpar, state);
[tau_unconstrained1, tau_unconstrained2] = CMG(gyro1, gyro2, ...
    allocation.unconstrainedCommand, state);
requestedThrusterForceMoment = [0;0];
hybrid.activation = 0;
hybrid.rollCommand = tauC.desiredPhi;
hybrid.rollError = tauC.desiredPhi-state(4);
hybrid.rollRate = state(10);
hybrid.lateralPosition = 0;
hybrid.lateralVelocity = 0;
hybrid.lateralError = 0;
hybrid.headingError = 0;
if isfield(cmgConfig,'hybrid') && cmgConfig.hybrid.enabled
    [hybridRequest,hybrid] = HYBRID_MANEUVER_CONTROL(state,d,cmgConfig);
    requestedThrusterForceMoment = [hybridRequest.Y;hybridRequest.N];
end
switch cmgConfig.thruster.commandMode
    case 'generalized_force'
        if ~cmgConfig.hybrid.enabled && isfield(d,'thruster')
            requestedThrusterForceMoment = [d.thruster.Y;d.thruster.N];
        end
        thrusterAllocation = THRUSTER_ALLOCATE( ...
            requestedThrusterForceMoment,cmgConfig.thruster);
    case 'direct_force'
        Bthruster = [1,1;cmgConfig.thruster.positionBody(1,:)];
        requestedThrusterForceMoment = ...
            Bthruster*cmgConfig.thruster.commandForce(:);
        thrusterAllocation = THRUSTER_ALLOCATE( ...
            requestedThrusterForceMoment,cmgConfig.thruster);
    otherwise
        error('Unsupported thruster command mode: %s', ...
            cmgConfig.thruster.commandMode);
end
thrusterConfig = cmgConfig;
thrusterConfig.thruster.commandForce = thrusterAllocation.commandedForce;
thruster = VORTEX_RING_THRUSTERS(t,state,thrusterConfig);
propulsion = AFT_PROPULSION(state,cmgConfig);

Etadot = REMUS(t, auv, contpar, params, state, ...
    tauC, tau_cmg1, tau_cmg2,thruster,propulsion);
if numel(state) >= 20
    Etadot = [Etadot;thruster.forceDot];
end
if numel(state) >= 21
    Etadot = [Etadot;propulsion.forceDot];
end

if cmgConfig.diagnostics.failOnNonfinite && any(~isfinite(Etadot))
    error('CMG:NonfiniteDerivative', ...
        'Nonfinite state derivative at t = %.9g s. State: %s', ...
        t, mat2str(state.', 6));
end

if nargout > 1
    controlData.requestedMoment = [tauC.KD; tauC.MD; tauC.ND];
    controlData.feedbackRollMoment = tauC.KFeedback;
    controlData.momentumUnloadMoment = tauC.KMomentumUnload;
    controlData.externalDumpMoment = tauC.KExternalDump;
    controlData.rollDisturbance = tauC.KDisturbance;
    controlData.achievedMoment = [ ...
        tau_cmg1.K + tau_cmg2.K; ...
        tau_cmg1.M + tau_cmg2.M; ...
        tau_cmg1.N + tau_cmg2.N];
    controlData.unconstrainedAchievedMoment = [ ...
        tau_unconstrained1.K + tau_unconstrained2.K; ...
        tau_unconstrained1.M + tau_unconstrained2.M; ...
        tau_unconstrained1.N + tau_unconstrained2.N];
    controlData.contpar = contpar;
    controlData.commandedContpar = commandedContpar;
    controlData.actuator = actuator;
    controlData.allocation = allocation;
    controlData.tau_cmg1 = tau_cmg1;
    controlData.tau_cmg2 = tau_cmg2;
    controlData.thruster = thruster;
    controlData.propulsion = propulsion;
    controlData.thrusterAllocation = thrusterAllocation;
    controlData.hybrid = hybrid;
end
end

function [actual, actuator] = applyGimbalActuatorDynamics(commanded, state, config)
% Model each gimbal as a rate servo with finite acceleration. Gimbal rate is
% now a state rather than an algebraic command, eliminating instantaneous
% rate changes and enabling preliminary motor torque/power calculations.
actual = commanded;
actualRates = state(17:18);
commandedRates = [commanded.alphadot1; commanded.alphadot2];
angles = [state(13); state(15)];
maxRate = config.limits.maxGimbalRate;
maxAccel = config.limits.maxGimbalAccel;

% A stopping-distance rate envelope prevents a finite-acceleration gimbal
% from coasting appreciably through its mechanical angle bound.
remainingTravel = max(config.limits.maxGimbalAngle-abs(angles), 0);
stoppingRates = sqrt(2*maxAccel*remainingTravel);
commandedRates = min(max(commandedRates, -stoppingRates), stoppingRates);
commandedRates = min(max(commandedRates, -maxRate), maxRate);

timeConstants = config.gimbal.rateTimeConstant(:);
if isscalar(timeConstants)
    timeConstants = repmat(timeConstants,2,1);
end
rawAccel = (commandedRates-actualRates)./timeConstants;
gimbalAccel = min(max(rawAccel, -maxAccel), maxAccel);
atOrBeyondStop = abs(angles) >= config.limits.maxGimbalAngle;
for module = 1:2
    if atOrBeyondStop(module) && sign(actualRates(module)) == sign(angles(module))
        gimbalAccel(module) = min(max( ...
            -actualRates(module)/timeConstants(module), ...
            -maxAccel), maxAccel);
    end
end

if strcmp(config.mode, 'single')
    commandedRates(2) = 0;
    gimbalAccel(2) = min(max( ...
        -actualRates(2)/timeConstants(2), -maxAccel), maxAccel);
end

actual.alphadot1 = actualRates(1);
actual.alphadot2 = actualRates(2);
actual.gimbalAccel1 = gimbalAccel(1);
actual.gimbalAccel2 = gimbalAccel(2);
actuator.commandedGimbalRate = commandedRates;
actuator.actualGimbalRate = actualRates;
actuator.gimbalAccel = gimbalAccel;
actuator.gimbalAccelSaturated = any(abs(rawAccel) > maxAccel);
end
