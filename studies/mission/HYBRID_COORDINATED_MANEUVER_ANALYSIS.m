% HYBRID_COORDINATED_MANEUVER_ANALYSIS.m
% Coordinate CMG plane alignment with closed-loop sway/yaw thrust.

clearvars;
scriptDir = CMG_ROOT();
baseline = load(fullfile(scriptDir,'Working Results','dual', ...
    'symmetric_spin','VFR','simulation_result.mat'));

initialState = baseline.Y_OUT(1,:).';
if numel(initialState) < 20
    initialState(19:20,1) = 0;
else
    initialState(19:20) = 0;
end
initialState(1:12) = 0;

% Demonstration: roll into an oblique east/down plane, translate 0.25 m
% along its lateral direction, and turn the nose 30 deg into that plane.
lateralDirection = [0;1;1]/sqrt(2);
turnAngle = deg2rad(30);
initialLongitudinal = [1;0;0];
desiredHeading = cos(turnAngle)*initialLongitudinal ...
    +sin(turnAngle)*lateralDirection;
planeCommand = ROLL_TO_PLANE_COMMAND(initialState(4:6), ...
    lateralDirection,initialState(4),true);

desired = baseline.d;
desired.phi = planeCommand.rollAngle;
desired.rollToPlane.desiredLateralDirectionNED = lateralDirection;
desired.rollToPlane.bidirectionalThruster = true;
desired.hybrid.initialPositionNED = initialState(1:3);
desired.hybrid.lateralDirectionNED = lateralDirection;
desired.hybrid.lateralDisplacement = 0.25;
desired.hybrid.desiredHeadingNED = desiredHeading;

config = baseline.cmgConfig;
config.hybrid.enabled = true;
config.thruster.enabled = true;
config.thruster.commandMode = 'generalized_force';
config.momentumManagement.enabled = false;
duration = 15;
loop.cycleT = duration;
loop.fc = 1/duration;
loop.controlEndTime = inf;
solverOptions = odeset('RelTol',baseline.simConfig.relTol, ...
    'AbsTol',baseline.simConfig.absTol,'MaxStep',baseline.simConfig.maxStep);

[time,state] = ode45(@CONTROL,[0,duration],initialState,solverOptions, ...
    baseline.gains,baseline.gyro1,baseline.gyro2,baseline.auv, ...
    baseline.params,desired,loop,config);
[~,~,control] = TORQUE(time,state,baseline.gains,baseline.gyro1, ...
    baseline.gyro2,baseline.auv,baseline.params,desired,loop,config);
conditioning = DUAL_CMG_CONDITIONING_ANALYSIS(time,state,control, ...
    baseline.gyro1,baseline.gyro2,config.limits,config.allocator);

activeIndex = find(control.hybridActivation >= 0.95,1,'first');
activationTime = nan;
if ~isempty(activeIndex)
    activationTime = time(activeIndex);
end
finalPlaneCommand = ROLL_TO_PLANE_COMMAND(state(end,4:6).', ...
    lateralDirection,state(end,4),true);
alignment = ROLL_TO_PLANE_ALIGNMENT(state(end,4:6).', ...
    lateralDirection,finalPlaneCommand.thrusterPolarity);
planeAlignmentErrorDeg = zeros(size(time));
for sample = 1:numel(time)
    instantaneousCommand = ROLL_TO_PLANE_COMMAND(state(sample,4:6).', ...
        lateralDirection,state(sample,4),true);
    instantaneousAlignment = ROLL_TO_PLANE_ALIGNMENT( ...
        state(sample,4:6).',lateralDirection, ...
        instantaneousCommand.thrusterPolarity);
    planeAlignmentErrorDeg(sample) = ...
        rad2deg(instantaneousAlignment.alignmentAngle);
end
activeSamples = control.hybridActivation >= 0.95;

result.activationTime = activationTime;
result.finalPlaneAlignmentErrorDeg = rad2deg(alignment.alignmentAngle);
result.maximumActivePlaneAlignmentErrorDeg = ...
    max(planeAlignmentErrorDeg(activeSamples));
result.finalHeadingErrorDeg = rad2deg(control.hybridHeadingError(end));
result.finalLateralPosition = control.hybridLateralPosition(end);
result.finalLateralError = control.hybridLateralError(end);
result.maximumThrusterForce = max(abs(control.thrusterForce),[],'all');
result.thrusterAllocationSaturationSamples = ...
    nnz(control.thrusterAllocationSaturated);
result.thrusterForceRateLimitSamples = nnz(control.thrusterRateLimited);
result.cmgRateLimitSamples = nnz(control.gimbalRateSaturated);
result.cmgAngleLimitSamples = nnz(control.gimbalAngleLimited);
result.rollInfeasibleSamples = conditioning.rateInfeasibleSamples;
result.minimumRollCapacityMargin = conditioning.minimumActiveCapacityMargin;
result.passes = isfinite(activationTime) ...
    && result.finalPlaneAlignmentErrorDeg <= 1 ...
    && result.maximumActivePlaneAlignmentErrorDeg <= 1 ...
    && abs(result.finalHeadingErrorDeg) <= 1 ...
    && abs(result.finalLateralError) <= 0.01 ...
    && result.thrusterAllocationSaturationSamples == 0 ...
    && result.cmgRateLimitSamples == 0 ...
    && result.cmgAngleLimitSamples == 0 ...
    && result.rollInfeasibleSamples == 0;

fprintf(['Hybrid maneuver: thrust active at %.3f s | final/max-active ' ...
    'plane error %.3f/%.3f deg | heading error %+.3f deg | lateral %.3f m ' ...
    '(error %+.4f m)\n'],result.activationTime, ...
    result.finalPlaneAlignmentErrorDeg, ...
    result.maximumActivePlaneAlignmentErrorDeg,result.finalHeadingErrorDeg, ...
    result.finalLateralPosition,result.finalLateralError);
fprintf(['  peak module force %.3f N | allocation/rate saturation samples ' ...
    '%d/%d | CMG rate/angle/roll-infeasible samples %d/%d/%d | pass %d\n'], ...
    result.maximumThrusterForce,result.thrusterAllocationSaturationSamples, ...
    result.thrusterForceRateLimitSamples,result.cmgRateLimitSamples, ...
    result.cmgAngleLimitSamples,result.rollInfeasibleSamples,result.passes);
assert(result.passes,'Coordinated hybrid maneuver did not meet requirements.');

fig = figure('Name','Coordinated CMG-thruster maneuver');
tiledlayout(3,2,'TileSpacing','compact');
nexttile;
plot(time,rad2deg(state(:,4)),'LineWidth',2,'DisplayName','Achieved roll');
hold on; plot(time,rad2deg(control.hybridRollCommand),'--','LineWidth',2, ...
    'DisplayName','Plane command'); hold off; grid on;
xlabel('Time (s)'); ylabel('Roll (deg)'); title('Plane Alignment');
legend('Location','best');
nexttile;
plot(time,control.hybridActivation,'LineWidth',2); grid on;
xlabel('Time (s)'); ylabel('Activation'); title('Thruster Coordination Gate');
ylim([-0.05,1.05]);
nexttile;
plot(time,control.hybridLateralPosition,'LineWidth',2); hold on;
yline(desired.hybrid.lateralDisplacement,'--k','LineWidth',1.5);
hold off; grid on; xlabel('Time (s)'); ylabel('Displacement (m)');
title('Lateral-Plane Translation');
nexttile;
plot(time,rad2deg(control.hybridHeadingError),'LineWidth',2); grid on;
xlabel('Time (s)'); ylabel('Heading error (deg)');
title('Heading-in-Plane Error');
nexttile;
plot(time,control.thrusterRequestedForceMoment,'--','LineWidth',1.5);
hold on; plot(time,control.thrusterGeneralizedForce(:,[2,6]), ...
    'LineWidth',2); hold off; grid on;
xlabel('Time (s)'); ylabel('Force / moment');
title('Requested and Achieved Y/N');
legend('Y request','N request','Y achieved','N achieved','Location','best');
nexttile;
plot(time,control.thrusterForce,'LineWidth',2); grid on;
xlabel('Time (s)'); ylabel('Module force (N)');
title('Fore/Aft Thruster Forces'); legend('Fore','Aft','Location','best');

outputDir = fullfile(scriptDir,'Working Results');
exportgraphics(fig,fullfile(outputDir,'HYBRID_COORDINATED_MANEUVER.png'), ...
    'Resolution',300);
save(fullfile(outputDir,'hybrid_coordinated_maneuver.mat'), ...
    'result','time','state','control','desired','config', ...
    'planeAlignmentErrorDeg');
