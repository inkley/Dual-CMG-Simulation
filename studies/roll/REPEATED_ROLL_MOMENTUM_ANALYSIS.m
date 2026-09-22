% REPEATED_ROLL_MOMENTUM_ANALYSIS.m
% Compare reciprocal and cumulative dual-CMG roll sequences to determine
% whether gimbal geometry and stored angular momentum are recovered.

clearvars;
scriptDir = CMG_ROOT();
baseline = load(fullfile(scriptDir, 'Working Results', 'dual', ...
    'symmetric_spin', 'VFR', 'simulation_result.mat'));

scenarios(1).name = 'reciprocal';
scenarios(1).scheduleTime = [0,6,12,18];
scenarios(1).scheduleAngle = deg2rad([90,0,90,0]);
scenarios(2).name = 'cumulative';
scenarios(2).scheduleTime = [0,6,12,18];
scenarios(2).scheduleAngle = deg2rad([90,180,270,360]);
duration = 24;

solverOptions = odeset('RelTol', baseline.simConfig.relTol, ...
    'AbsTol', baseline.simConfig.absTol, ...
    'MaxStep', baseline.simConfig.maxStep);
loop.cycleT = duration;
loop.fc = 1/duration;
loop.controlEndTime = inf;

for scenarioIndex = 1:numel(scenarios)
    scenario = scenarios(scenarioIndex);
    desired = baseline.d;
    desired.rollScheduleTime = scenario.scheduleTime;
    desired.rollScheduleAngle = scenario.scheduleAngle;
    initialState = baseline.Y_OUT(1,:).';

    [time,state] = ode45(@CONTROL,[0,duration],initialState,solverOptions, ...
        baseline.gains,baseline.gyro1,baseline.gyro2,baseline.auv, ...
        baseline.params,desired,loop,baseline.cmgConfig);
    [tau1,tau2,control] = TORQUE(time,state,baseline.gains, ...
        baseline.gyro1,baseline.gyro2,baseline.auv,baseline.params, ...
        desired,loop,baseline.cmgConfig);
    conditioning = DUAL_CMG_CONDITIONING_ANALYSIS(time,state,control, ...
        baseline.gyro1,baseline.gyro2,baseline.cmgConfig.limits, ...
        baseline.cmgConfig.allocator);

    commandedRoll = interp1(scenario.scheduleTime,scenario.scheduleAngle, ...
        time,'previous','extrap');
    rotorHx = baseline.gyro1.I*state(:,14).*sin(state(:,13)) ...
        + baseline.gyro2.I*state(:,16).*sin(state(:,15));
    segmentEndTimes = [scenario.scheduleTime(2:end) ...
        - baseline.simConfig.maxStep, duration].';
    segmentEndIndices = arrayfun(@(value) find(time<=value,1,'last'), ...
        segmentEndTimes);

    result.name = scenario.name;
    result.time = time;
    result.state = state;
    result.commandedRoll = commandedRoll;
    result.tau1 = tau1;
    result.tau2 = tau2;
    result.control = control;
    result.conditioning = conditioning;
    result.rotorRollMomentum = rotorHx;
    result.segmentEndRollErrorDeg = rad2deg( ...
        commandedRoll(segmentEndIndices)-state(segmentEndIndices,4));
    result.finalRollErrorDeg = rad2deg(commandedRoll(end)-state(end,4));
    result.rotorMomentumDrift = rotorHx(end)-rotorHx(1);
    result.gimbalStateDriftDeg = rad2deg( ...
        [state(end,13)-state(1,13),state(end,15)-state(1,15)]);
    result.maximumConditionNumber = conditioning.maximumConditionNumber;
    result.minimumSingularityDistanceDeg = ...
        rad2deg(conditioning.minimumSingularityDistance);
    result.minimumRollCapacityMargin = conditioning.minimumActiveCapacityMargin;
    result.nearSingularSamples = conditioning.nearSingularSamples;
    result.rateLimitedSamples = nnz(control.gimbalRateSaturated);
    result.accelLimitedSamples = nnz(control.gimbalAccelSaturated);
    result.angleLimitedSamples = nnz(control.gimbalAngleLimited);
    result.maximumAbsRollErrorDeg = rad2deg(max(abs(commandedRoll-state(:,4))));
    results.(scenario.name) = result;
end

fprintf('\nRepeated-roll momentum analysis\n');
names = fieldnames(results);
for index = 1:numel(names)
    value = results.(names{index});
    fprintf(['%s: final error %.3f deg | rotor-Hx drift %.3e N m s | ' ...
        'gimbal drift [%+.2f,%+.2f] deg\n'], value.name, ...
        value.finalRollErrorDeg,value.rotorMomentumDrift, ...
        value.gimbalStateDriftDeg);
    fprintf(['  max condition %.2f | min singularity distance %.2f deg | ' ...
        'min capacity margin %.2fx | near-singular/rate/angle samples %d/%d/%d\n'], ...
        value.maximumConditionNumber,value.minimumSingularityDistanceDeg, ...
        value.minimumRollCapacityMargin,value.nearSingularSamples, ...
        value.rateLimitedSamples,value.angleLimitedSamples);
end

fig = figure('Name','Repeated roll and CMG momentum');
tiledlayout(2,2,'TileSpacing','compact');
for index = 1:numel(names)
    value = results.(names{index});
    nexttile(index);
    plot(value.time,rad2deg(value.state(:,4)),'LineWidth',2);
    hold on;
    plot(value.time,rad2deg(value.commandedRoll),'--k','LineWidth',1.5);
    hold off; grid on;
    xlabel('Time (s)'); ylabel('Roll (deg)');
    title([upper(value.name(1)),value.name(2:end),' roll response']);
    legend('Achieved','Commanded','Location','best');
    nexttile(index+2);
    yyaxis left;
    plot(value.time,value.rotorRollMomentum,'LineWidth',2);
    ylabel('Rotor H_x (N m s)');
    yyaxis right;
    plot(value.time,rad2deg(value.conditioning.singularityDistance), ...
        '--','LineWidth',2);
    ylabel('Singularity distance (deg)');
    xlabel('Time (s)'); grid on;
    title([upper(value.name(1)),value.name(2:end),' momentum state']);
end
exportgraphics(fig,fullfile(scriptDir,'Working Results', ...
    'REPEATED_ROLL_MOMENTUM.png'),'Resolution',300);
save(fullfile(scriptDir,'Working Results', ...
    'repeated_roll_momentum.mat'),'results','scenarios');
