% HYBRID_MANEUVER_SWEEP.m
% Evaluate combined roll, sway, and yaw control across maneuver directions.

clearvars;
scriptDir = CMG_ROOT();
baseline = load(fullfile(scriptDir,'Working Results','dual', ...
    'symmetric_spin','VFR','simulation_result.mat'));

% [plane angle deg, in-plane heading change deg, lateral displacement m]
caseMatrix = [ ...
    -90,-30, 0.25; ...
    -60, 30, 0.20; ...
    -30,-20,-0.15; ...
      0, 45, 0.25; ...
     30,-30, 0.20; ...
     60, 30,-0.20; ...
     90, 30, 0.25; ...
     45, 45, 0.00; ...
    -45,-45, 0.00; ...
     45,  0, 0.30; ...
    -45,  0,-0.30];
caseCount = size(caseMatrix,1);
duration = 18;
solverOptions = odeset('RelTol',baseline.simConfig.relTol, ...
    'AbsTol',baseline.simConfig.absTol,'MaxStep',baseline.simConfig.maxStep);
rows = repmat(struct(),caseCount,1);

for caseIndex = 1:caseCount
    planeAngle = deg2rad(caseMatrix(caseIndex,1));
    turnAngle = deg2rad(caseMatrix(caseIndex,2));
    lateralDisplacement = caseMatrix(caseIndex,3);
    initialLongitudinal = [1;0;0];
    lateralDirection = [0;cos(planeAngle);sin(planeAngle)];
    planeNormal = cross(initialLongitudinal,lateralDirection);
    planeNormal = planeNormal/norm(planeNormal);
    desiredHeading = cos(turnAngle)*initialLongitudinal ...
        +sin(turnAngle)*lateralDirection;

    initialState = baseline.Y_OUT(1,:).';
    if numel(initialState) < 20
        initialState(19:20,1) = 0;
    else
        initialState(19:20) = 0;
    end
    initialState(1:12) = 0;
    planeCommand = ROLL_TO_PLANE_COMMAND(initialState(4:6), ...
        lateralDirection,initialState(4),true);

    desired = baseline.d;
    desired.phi = planeCommand.rollAngle;
    desired.rollToPlane.desiredLateralDirectionNED = lateralDirection;
    desired.rollToPlane.bidirectionalThruster = true;
    desired.hybrid.initialPositionNED = initialState(1:3);
    desired.hybrid.lateralDirectionNED = lateralDirection;
    desired.hybrid.lateralDisplacement = lateralDisplacement;
    desired.hybrid.desiredHeadingNED = desiredHeading;

    config = baseline.cmgConfig;
    config.hybrid.enabled = true;
    % Use a tighter capture gate for the multi-axis sweep so the most highly
    % rolled cases begin thrust only after a firm plane lock.
    config.hybrid.rollEnableAngle = deg2rad(0.5);
    config.hybrid.rollDisableAngle = deg2rad(1.5);
    config.hybrid.rollEnableRate = deg2rad(0.5);
    config.hybrid.rollDisableRate = deg2rad(3.0);
    config.hybrid.KpLateral = 20;
    config.hybrid.KdLateral = 30;
    config.hybrid.KpHeading = 2.0;
    config.hybrid.KdHeading = 6.0;
    config.thruster.enabled = true;
    config.thruster.commandMode = 'generalized_force';
    config.momentumManagement.enabled = false;
    loop.cycleT = duration;
    loop.fc = 1/duration;
    loop.controlEndTime = inf;

    [time,state] = ode45(@CONTROL,[0,duration],initialState,solverOptions, ...
        baseline.gains,baseline.gyro1,baseline.gyro2,baseline.auv, ...
        baseline.params,desired,loop,config);
    [~,~,control] = TORQUE(time,state,baseline.gains,baseline.gyro1, ...
        baseline.gyro2,baseline.auv,baseline.params,desired,loop,config);
    conditioning = DUAL_CMG_CONDITIONING_ANALYSIS(time,state,control, ...
        baseline.gyro1,baseline.gyro2,config.limits,config.allocator);

    active = control.hybridActivation >= 0.95;
    activationSample = find(active,1,'first');
    activationTime = nan;
    if ~isempty(activationSample)
        activationTime = time(activationSample);
    end
    planeErrorDeg = zeros(size(time));
    for sample = 1:numel(time)
        command = ROLL_TO_PLANE_COMMAND(state(sample,4:6).', ...
            lateralDirection,state(sample,4),true);
        alignment = ROLL_TO_PLANE_ALIGNMENT(state(sample,4:6).', ...
            lateralDirection,command.thrusterPolarity);
        planeErrorDeg(sample) = rad2deg(alignment.alignmentAngle);
    end
    displacement = state(end,1:3).'-initialState(1:3);
    outOfPlaneDisplacement = dot(displacement,planeNormal);

    rows(caseIndex).caseNumber = caseIndex;
    rows(caseIndex).planeAngleDeg = caseMatrix(caseIndex,1);
    rows(caseIndex).headingCommandDeg = caseMatrix(caseIndex,2);
    rows(caseIndex).lateralCommand = lateralDisplacement;
    rows(caseIndex).activationTime = activationTime;
    rows(caseIndex).finalActivation = control.hybridActivation(end);
    rows(caseIndex).activeFraction = mean(active);
    rows(caseIndex).maximumActivePlaneErrorDeg = max(planeErrorDeg(active));
    rows(caseIndex).finalPlaneErrorDeg = planeErrorDeg(end);
    rows(caseIndex).finalHeadingErrorDeg = ...
        rad2deg(control.hybridHeadingError(end));
    rows(caseIndex).finalLateralError = control.hybridLateralError(end);
    rows(caseIndex).outOfPlaneDisplacement = outOfPlaneDisplacement;
    rows(caseIndex).peakModuleForce = max(abs(control.thrusterForce),[],'all');
    rows(caseIndex).thrusterAllocationSaturationSamples = ...
        nnz(control.thrusterAllocationSaturated);
    rows(caseIndex).thrusterRateLimitSamples = ...
        nnz(control.thrusterRateLimited);
    rows(caseIndex).cmgRateLimitSamples = nnz(control.gimbalRateSaturated);
    rows(caseIndex).cmgAngleLimitSamples = nnz(control.gimbalAngleLimited);
    rows(caseIndex).maximumGimbalAngleDeg = ...
        rad2deg(max(abs(state(:,[13,15])),[],'all'));
    rows(caseIndex).rollInfeasibleSamples = conditioning.rateInfeasibleSamples;
    rows(caseIndex).minimumRollCapacityMargin = ...
        conditioning.minimumActiveCapacityMargin;
    rows(caseIndex).passes = isfinite(activationTime) ...
        && rows(caseIndex).maximumActivePlaneErrorDeg <= 1 ...
        && abs(rows(caseIndex).finalHeadingErrorDeg) <= 1 ...
        && abs(rows(caseIndex).finalLateralError) <= 0.015 ...
        && abs(rows(caseIndex).outOfPlaneDisplacement) <= 0.015 ...
        && rows(caseIndex).thrusterAllocationSaturationSamples == 0 ...
        && rows(caseIndex).cmgRateLimitSamples == 0 ...
        && rows(caseIndex).cmgAngleLimitSamples == 0 ...
        && rows(caseIndex).rollInfeasibleSamples == 0;
end

results = struct2table(rows);
disp(results(:,{'planeAngleDeg','headingCommandDeg','lateralCommand', ...
    'maximumActivePlaneErrorDeg','finalHeadingErrorDeg', ...
    'finalLateralError','finalActivation','maximumGimbalAngleDeg', ...
    'rollInfeasibleSamples','cmgAngleLimitSamples','peakModuleForce','passes'}));
fprintf(['Hybrid sweep passed %d/%d | max active plane error %.3f deg | ' ...
    'max final heading error %.3f deg | max lateral error %.4f m | ' ...
    'max out-of-plane drift %.4e m | peak module force %.3f N\n'], ...
    sum(results.passes),height(results), ...
    max(results.maximumActivePlaneErrorDeg), ...
    max(abs(results.finalHeadingErrorDeg)), ...
    max(abs(results.finalLateralError)), ...
    max(abs(results.outOfPlaneDisplacement)),max(results.peakModuleForce));

fig = figure('Name','Combined roll-sway-yaw sweep');
tiledlayout(2,2,'TileSpacing','compact');
nexttile;
scatter(results.planeAngleDeg,results.finalHeadingErrorDeg,70, ...
    results.headingCommandDeg,'filled'); grid on; colorbar;
xlabel('Maneuver-plane angle (deg)'); ylabel('Heading error (deg)');
title('Final Heading Accuracy');
nexttile;
scatter(results.planeAngleDeg,100*results.finalLateralError,70, ...
    results.lateralCommand,'filled'); grid on; colorbar;
xlabel('Maneuver-plane angle (deg)'); ylabel('Lateral error (cm)');
title('Final Translation Accuracy');
nexttile;
stem(results.planeAngleDeg,results.maximumActivePlaneErrorDeg,'filled', ...
    'LineWidth',1.5); hold on; yline(1,'--r','1 deg requirement');
hold off; grid on;
xlabel('Maneuver-plane angle (deg)'); ylabel('Maximum error (deg)');
title('Active Plane Alignment');
nexttile;
scatter(results.planeAngleDeg,results.peakModuleForce,70, ...
    abs(results.headingCommandDeg),'filled'); grid on; colorbar;
xlabel('Maneuver-plane angle (deg)'); ylabel('Peak force (N)');
title('Thruster Demand');

outputDir = fullfile(scriptDir,'Working Results');
exportgraphics(fig,fullfile(outputDir,'HYBRID_MANEUVER_SWEEP.png'), ...
    'Resolution',300);
writetable(results,fullfile(outputDir,'hybrid_maneuver_sweep.csv'));
save(fullfile(outputDir,'hybrid_maneuver_sweep.mat'), ...
    'results','rows','caseMatrix','duration');
if ~all(results.passes)
    warning('CMG:HybridEnvelopeFailures', ...
        '%d of %d combined maneuver cases exceeded a requirement.', ...
        nnz(~results.passes),height(results));
end
