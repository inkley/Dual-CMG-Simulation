% VORTEX_RING_THRUSTER_INTEGRATION.m
% Demonstrate common-mode sway and differential-mode yaw actuation.

clearvars;
scriptDir = CMG_ROOT();
baseline = load(fullfile(scriptDir,'Working Results','dual', ...
    'symmetric_spin','VFR','simulation_result.mat'));

tests(1).name = 'common_sway';
tests(1).request = [2;0];
tests(2).name = 'differential_yaw';
tests(2).request = [0;1.3];
duration = 4;
solverOptions = odeset('RelTol',baseline.simConfig.relTol, ...
    'AbsTol',baseline.simConfig.absTol,'MaxStep',baseline.simConfig.maxStep);

for index = 1:numel(tests)
    initialState = baseline.Y_OUT(1,:).';
    if numel(initialState) < 20
        initialState(19:20,1) = 0;
    else
        initialState(19:20) = 0;
    end
    desired = baseline.d;
    if isfield(desired,'rollToPlane')
        desired = rmfield(desired,'rollToPlane');
    end
    desired.phi = initialState(4);
    config = baseline.cmgConfig;
    config.thruster.enabled = true;
    config.thruster.commandMode = 'generalized_force';
    desired.thruster.Y = tests(index).request(1);
    desired.thruster.N = tests(index).request(2);
    config.momentumManagement.enabled = false;
    loop.cycleT = duration;
    loop.fc = 1/duration;
    loop.controlEndTime = inf;

    [time,state] = ode45(@CONTROL,[0,duration],initialState,solverOptions, ...
        baseline.gains,baseline.gyro1,baseline.gyro2,baseline.auv, ...
        baseline.params,desired,loop,config);
    [~,~,control] = TORQUE(time,state,baseline.gains,baseline.gyro1, ...
        baseline.gyro2,baseline.auv,baseline.params,desired,loop,config);

    result.name = tests(index).name;
    result.time = time;
    result.state = state;
    result.control = control;
    result.finalSwayPosition = state(end,2);
    result.finalSwayVelocity = state(end,8);
    result.finalYawDeg = rad2deg(state(end,6));
    result.finalYawRateDegPerSec = rad2deg(state(end,12));
    result.peakRollDeg = rad2deg(max(abs(state(:,4))));
    result.peakPitchDeg = rad2deg(max(abs(state(:,5))));
    result.peakSwayForce = max(abs( ...
        control.thrusterGeneralizedForce(:,2)));
    result.peakYawMoment = max(abs( ...
        control.thrusterGeneralizedForce(:,6)));
    result.forceLimitSamples = nnz(control.thrusterForceLimited);
    result.rateLimitSamples = nnz(control.thrusterRateLimited);
    results.(tests(index).name) = result;
end

common = results.common_sway;
differential = results.differential_yaw;
assert(common.finalSwayPosition > 0 && common.finalSwayVelocity > 0, ...
    'Common-mode thrust did not produce positive sway.');
assert(abs(common.finalYawDeg) < 1e-8, ...
    'Common-mode thrust should not generate yaw for symmetric mounts.');
assert(differential.finalYawDeg > 0 ...
    && differential.finalYawRateDegPerSec > 0, ...
    'Differential thrust did not produce positive yaw.');
assert(abs(differential.finalSwayPosition) < 1e-8, ...
    'Differential thrust should not generate sway at rest.');

fprintf(['VRT integration passed. Common [1,1] N: final y %.3f m, ' ...
    'v %.3f m/s, yaw %.3e deg.\n'],common.finalSwayPosition, ...
    common.finalSwayVelocity,common.finalYawDeg);
fprintf(['Differential [1,-1] N: final yaw %.3f deg, r %.3f deg/s, ' ...
    'sway %.3e m.\n'],differential.finalYawDeg, ...
    differential.finalYawRateDegPerSec,differential.finalSwayPosition);

fig = figure('Name','Fixed vortex-ring thruster integration');
tiledlayout(2,2,'TileSpacing','compact');
nexttile;
plot(common.time,common.control.thrusterForce,'LineWidth',2); grid on;
xlabel('Time (s)'); ylabel('Module force (N)');
title('Common-Mode Force'); legend('Fore','Aft','Location','best');
nexttile;
plot(common.time,common.state(:,2),'LineWidth',2); grid on;
xlabel('Time (s)'); ylabel('Sway position y (m)');
title('Sway Response');
nexttile;
plot(differential.time,differential.control.thrusterForce,'LineWidth',2); grid on;
xlabel('Time (s)'); ylabel('Module force (N)');
title('Differential Force'); legend('Fore','Aft','Location','best');
nexttile;
plot(differential.time,rad2deg(differential.state(:,6)),'LineWidth',2); grid on;
xlabel('Time (s)'); ylabel('Yaw \psi (deg)');
title('Yaw Response');

outputDir = fullfile(scriptDir,'Working Results');
exportgraphics(fig,fullfile(outputDir,'VORTEX_RING_THRUSTER_INTEGRATION.png'), ...
    'Resolution',300);
save(fullfile(outputDir,'vortex_ring_thruster_integration.mat'), ...
    'results','tests','duration');
