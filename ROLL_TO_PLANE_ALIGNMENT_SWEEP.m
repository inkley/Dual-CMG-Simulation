% ROLL_TO_PLANE_ALIGNMENT_SWEEP.m
% Validate generated and dynamically achieved maneuver-plane alignment.

clearvars;
scriptDir = fileparts(mfilename('fullpath'));
baseline = load(fullfile(scriptDir,'Working Results','dual', ...
    'symmetric_spin','VFR','simulation_result.mat'));

% Plane angles are defined from body +y toward body +z before roll. The last
% two cases exercise the same geometry at nonzero pitch and yaw.
planeDeg = [-90,-60,-30,0,30,60,90,60,-45];
pitchDeg = [0,0,0,0,0,0,0,15,-10];
yawDeg = [0,0,0,0,0,0,0,45,-60];
caseCount = numel(planeDeg);
finalToleranceDeg = 1.0;

solverOptions = odeset('RelTol',baseline.simConfig.relTol, ...
    'AbsTol',baseline.simConfig.absTol,'MaxStep',baseline.simConfig.maxStep);
rows = repmat(struct(),caseCount,1);

for index = 1:caseCount
    theta0 = deg2rad(pitchDeg(index));
    psi0 = deg2rad(yawDeg(index));
    beta = deg2rad(planeDeg(index));
    Rz = [cos(psi0),-sin(psi0),0; sin(psi0),cos(psi0),0; 0,0,1];
    Ry = [cos(theta0),0,sin(theta0); 0,1,0; -sin(theta0),0,cos(theta0)];
    desiredDirection = Rz*Ry*[0;cos(beta);sin(beta)];

    initialState = baseline.Y_OUT(1,:).';
    initialState(4:6) = [0;theta0;psi0];
    command = ROLL_TO_PLANE_COMMAND(initialState(4:6), ...
        desiredDirection,initialState(4),true);
    desired = baseline.d;
    if isfield(desired,'rollToPlane')
        desired = rmfield(desired,'rollToPlane');
    end
    desired.phi = command.rollAngle;
    if isfield(desired,'rollScheduleTime')
        desired = rmfield(desired,{'rollScheduleTime','rollScheduleAngle'});
    end
    config = baseline.cmgConfig;
    config.momentumManagement.enabled = false;
    config.external.rollDisturbance = 0;
    loop.cycleT = baseline.simConfig.duration;
    loop.fc = 1/loop.cycleT;
    loop.controlEndTime = inf;

    [time,state] = ode45(@CONTROL,[0,baseline.simConfig.duration], ...
        initialState,solverOptions,baseline.gains,baseline.gyro1, ...
        baseline.gyro2,baseline.auv,baseline.params,desired,loop,config);
    [~,~,control] = TORQUE(time,state,baseline.gains,baseline.gyro1, ...
        baseline.gyro2,baseline.auv,baseline.params,desired,loop,config);
    conditioning = DUAL_CMG_CONDITIONING_ANALYSIS(time,state,control, ...
        baseline.gyro1,baseline.gyro2,config.limits,config.allocator);
    achieved = ROLL_TO_PLANE_ALIGNMENT(state(end,4:6).', ...
        desiredDirection,command.thrusterPolarity);

    rows(index).caseNumber = index;
    rows(index).planeCommandDeg = planeDeg(index);
    rows(index).initialPitchDeg = pitchDeg(index);
    rows(index).initialYawDeg = yawDeg(index);
    rows(index).generatedRollDeg = rad2deg(command.rollAngle);
    rows(index).thrustPolarity = command.thrusterPolarity;
    rows(index).generatorErrorDeg = rad2deg(command.alignmentAngle);
    rows(index).finalRollDeg = rad2deg(state(end,4));
    rows(index).finalAlignmentErrorDeg = rad2deg(achieved.alignmentAngle);
    rows(index).peakGimbalRate = max(abs(control.alphadot),[],'all');
    rows(index).peakGimbalAccel = max(abs(control.gimbalAccel),[],'all');
    rows(index).rateLimitedSamples = nnz(control.gimbalRateSaturated);
    rows(index).angleLimitedSamples = nnz(control.gimbalAngleLimited);
    rows(index).rollInfeasibleSamples = conditioning.rateInfeasibleSamples;
    rows(index).minimumRollCapacityMargin = ...
        conditioning.minimumActiveCapacityMargin;
    rows(index).passes = rows(index).generatorErrorDeg < 1e-9 ...
        && rows(index).finalAlignmentErrorDeg <= finalToleranceDeg ...
        && rows(index).rateLimitedSamples == 0 ...
        && rows(index).angleLimitedSamples == 0 ...
        && rows(index).rollInfeasibleSamples == 0;
end

results = struct2table(rows);
disp(results(:,{'planeCommandDeg','initialPitchDeg','initialYawDeg', ...
    'generatedRollDeg','thrustPolarity','finalAlignmentErrorDeg', ...
    'peakGimbalRate','peakGimbalAccel','rollInfeasibleSamples','passes'}));
assert(all(results.passes),'At least one roll-to-plane case failed.');

fprintf(['Roll-to-plane sweep passed %d/%d cases | maximum final alignment ' ...
    'error %.3f deg | peak gimbal rate %.3f rad/s | peak gimbal ' ...
    'acceleration %.1f rad/s^2\n'],sum(results.passes),height(results), ...
    max(results.finalAlignmentErrorDeg),max(results.peakGimbalRate), ...
    max(results.peakGimbalAccel));

fig = figure('Name','Roll-to-plane alignment sweep');
tiledlayout(2,1,'TileSpacing','compact');
nexttile;
plot(results.planeCommandDeg,results.generatedRollDeg,'ok', ...
    'LineWidth',1.5,'DisplayName','Generated command');
hold on;
plot(results.planeCommandDeg,results.finalRollDeg,'x','LineWidth',2, ...
    'MarkerSize',9,'DisplayName','Achieved roll');
plot([-90,90],[-90,90],'--','LineWidth',1.5,'DisplayName','Ideal');
hold off; grid on;
xlabel('Requested plane angle (deg)'); ylabel('Roll angle (deg)');
legend('Location','best'); title('Roll-to-Plane Commands');
nexttile;
stem(results.planeCommandDeg,results.finalAlignmentErrorDeg,'filled', ...
    'LineWidth',1.5,'DisplayName','Final alignment error');
hold on;
yline(finalToleranceDeg,'--r','1 deg requirement','LineWidth',1.5);
hold off; grid on;
xlabel('Requested plane angle (deg)'); ylabel('Absolute error (deg)');
title('Achieved Thruster-Plane Alignment');

outputDir = fullfile(scriptDir,'Working Results');
exportgraphics(fig,fullfile(outputDir,'ROLL_TO_PLANE_ALIGNMENT.png'), ...
    'Resolution',300);
writetable(results,fullfile(outputDir,'roll_to_plane_alignment.csv'));
save(fullfile(outputDir,'roll_to_plane_alignment.mat'), ...
    'results','rows','finalToleranceDeg');
