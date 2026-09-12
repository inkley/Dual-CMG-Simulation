% MOMENTUM_UNLOADING_ANALYSIS.m
% Demonstrate why CMGs require an external reaction for momentum dumping and
% compare unmanaged/managed sustained-disturbance and unidirectional cases.

clearvars;
scriptDir = fileparts(mfilename('fullpath'));
baseline = load(fullfile(scriptDir,'Working Results','dual', ...
    'symmetric_spin','VFR','simulation_result.mat'));

tests(1) = makeTest('sustained_off',false,15,0,0.02);
tests(2) = makeTest('sustained_on', true, 15,0,0.02);
rollTimes = 0:6:24;
rollAngles = deg2rad(90:90:450);
tests(3) = makeTest('unidirectional_off',false,30,rollAngles,0);
tests(3).scheduleTime = rollTimes;
tests(4) = makeTest('unidirectional_on', true, 30,rollAngles,0);
tests(4).scheduleTime = rollTimes;

solverOptions = odeset('RelTol',baseline.simConfig.relTol, ...
    'AbsTol',baseline.simConfig.absTol,'MaxStep',baseline.simConfig.maxStep);
for testIndex = 1:numel(tests)
    test = tests(testIndex);
    config = baseline.cmgConfig;
    config.momentumManagement.enabled = test.unloadingEnabled;
    config.external.rollDisturbance = test.disturbance;
    desired = baseline.d;
    if isfield(desired,'rollToPlane')
        desired = rmfield(desired,'rollToPlane');
    end
    if numel(test.command) > 1
        desired.rollScheduleTime = test.scheduleTime;
        desired.rollScheduleAngle = test.command;
    else
        desired.phi = test.command;
    end
    loop.cycleT = test.duration;
    loop.fc = 1/test.duration;
    loop.controlEndTime = inf;
    initialState = baseline.Y_OUT(1,:).';

    [time,state] = ode45(@CONTROL,[0,test.duration],initialState, ...
        solverOptions,baseline.gains,baseline.gyro1,baseline.gyro2, ...
        baseline.auv,baseline.params,desired,loop,config);
    [~,~,control] = TORQUE(time,state,baseline.gains,baseline.gyro1, ...
        baseline.gyro2,baseline.auv,baseline.params,desired,loop,config);
    conditioning = DUAL_CMG_CONDITIONING_ANALYSIS(time,state,control, ...
        baseline.gyro1,baseline.gyro2,config.limits,config.allocator);
    commandedRoll = desired.phi*ones(size(time));
    if isfield(desired,'rollScheduleTime')
        commandedRoll = interp1(desired.rollScheduleTime, ...
            desired.rollScheduleAngle,time,'previous','extrap');
    end
    rotorHx = baseline.gyro1.I*state(:,14).*sin(state(:,13)) ...
        + baseline.gyro2.I*state(:,16).*sin(state(:,15));

    result.name = test.name;
    result.time = time;
    result.state = state;
    result.commandedRoll = commandedRoll;
    result.control = control;
    result.rotorHxError = rotorHx-config.momentumManagement.referenceHx;
    result.finalMomentumError = result.rotorHxError(end);
    result.maximumMomentumError = max(abs(result.rotorHxError));
    result.finalRollErrorDeg = rad2deg(commandedRoll(end)-state(end,4));
    result.maximumRollErrorDeg = rad2deg(max(abs(commandedRoll-state(:,4))));
    result.maximumGimbalAngleDeg = rad2deg(max(abs( ...
        state(:,[13,15])),[],'all'));
    result.peakExternalDumpMoment = max(abs(control.externalDumpMoment));
    result.externalDumpImpulse = trapz(time,control.externalDumpMoment);
    result.unloadingPairCancellationError = max(abs( ...
        control.momentumUnloadMoment+control.externalDumpMoment));
    assert(result.unloadingPairCancellationError < 1e-12, ...
        'CMG unloading and external counter-moment do not cancel.');
    result.rateLimitedSamples = nnz(control.gimbalRateSaturated);
    result.angleLimitedSamples = nnz(control.gimbalAngleLimited);
    result.nearSingularSamples = conditioning.nearSingularSamples;
    result.rollRateInfeasibleSamples = conditioning.rateInfeasibleSamples;
    result.requiresSingularityAvoidance = ...
        conditioning.requiresSingularityAvoidance;
    results.(test.name) = result;
end

fprintf('\nMomentum-unloading analysis\n');
names = fieldnames(results);
for index = 1:numel(names)
    value = results.(names{index});
    fprintf(['%s: final Hx error %+.4e N m s | max |Hx error| %.4e | ' ...
        'final roll error %+.3f deg | max angle %.1f deg\n'], ...
        value.name,value.finalMomentumError,value.maximumMomentumError, ...
        value.finalRollErrorDeg,value.maximumGimbalAngleDeg);
    fprintf(['  peak dump %.4f N m | dump impulse %+.4e N m s | ' ...
        'rate/angle/near-singular/roll-infeasible samples %d/%d/%d/%d\n'], ...
        value.peakExternalDumpMoment,value.externalDumpImpulse, ...
        value.rateLimitedSamples,value.angleLimitedSamples, ...
        value.nearSingularSamples,value.rollRateInfeasibleSamples);
end

fig = figure('Name','CMG momentum unloading');
tiledlayout(2,2,'TileSpacing','compact');
plotPair(results.sustained_off,results.sustained_on,'Sustained disturbance');
plotPair(results.unidirectional_off,results.unidirectional_on, ...
    'Repeated positive rolls');
exportgraphics(fig,fullfile(scriptDir,'Working Results', ...
    'MOMENTUM_UNLOADING.png'),'Resolution',300);
save(fullfile(scriptDir,'Working Results','momentum_unloading.mat'), ...
    'results','tests');

function test = makeTest(name,enabled,duration,command,disturbance)
    test.name = name;
    test.unloadingEnabled = enabled;
    test.duration = duration;
    test.command = command;
    test.disturbance = disturbance;
    test.scheduleTime = 0;
end

function plotPair(off,on,titleText)
    nexttile;
    plot(off.time,off.rotorHxError,'LineWidth',2);
    hold on;
    plot(on.time,on.rotorHxError,'--','LineWidth',2);
    hold off; grid on;
    xlabel('Time (s)'); ylabel('Rotor H_x error (N m s)');
    title([titleText,' momentum']);
    legend('Unmanaged','Managed','Location','best');
    nexttile;
    plot(off.time,rad2deg(off.state(:,4)),'LineWidth',2);
    hold on;
    plot(on.time,rad2deg(on.state(:,4)),'--','LineWidth',2);
    plot(on.time,rad2deg(on.commandedRoll),':k','LineWidth',1.5);
    hold off; grid on;
    xlabel('Time (s)'); ylabel('Roll (deg)');
    title([titleText,' response']);
    legend('Unmanaged','Managed','Command','Location','best');
end
