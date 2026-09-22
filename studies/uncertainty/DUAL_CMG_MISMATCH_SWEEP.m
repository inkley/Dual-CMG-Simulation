% DUAL_CMG_MISMATCH_SWEEP.m
% Robustness sweep for counter-rotating flywheel-speed mismatch and unequal
% gimbal rate-servo response. The allocator receives the actual rotor speeds.

clearvars;
scriptDir = CMG_ROOT();
baseline = load(fullfile(scriptDir, 'Working Results', 'dual', ...
    'symmetric_spin', 'VFR', 'simulation_result.mat'));

speedMismatchPercent = [-20,-10,-5,0,5,10,20];
servoMismatchPercent = [-20,-10,-5,0,5,10,20];
gridSize = [numel(servoMismatchPercent), numel(speedMismatchPercent)];
fields = {'settlingTime','overshootPercent','finalErrorDeg', ...
    'peakPitchDeg','peakYawDeg','rollMomentRMSE','maximumConditionNumber', ...
    'minimumSingularityDistanceDeg','minimumRollCapacityMargin', ...
    'rateLimitedSamples','accelLimitedSamples'};
for fieldIndex = 1:numel(fields)
    results.(fields{fieldIndex}) = nan(gridSize);
end

baseSpeed = mean(abs([baseline.Y_OUT(1,14),baseline.Y_OUT(1,16)]));
baseTimeConstant = baseline.cmgConfig.gimbal.rateTimeConstant(1);
loop.cycleT = baseline.simConfig.duration;
loop.fc = 1/loop.cycleT;
loop.controlEndTime = inf;
solverOptions = odeset('RelTol', baseline.simConfig.relTol, ...
    'AbsTol', baseline.simConfig.absTol, ...
    'MaxStep', baseline.simConfig.maxStep);

for servoIndex = 1:numel(servoMismatchPercent)
    servoFraction = servoMismatchPercent(servoIndex)/100;
    for speedIndex = 1:numel(speedMismatchPercent)
        speedFraction = speedMismatchPercent(speedIndex)/100;
        config = baseline.cmgConfig;
        config.mode = 'dual';
        config.gimbal.rateTimeConstant = [baseTimeConstant, ...
            baseTimeConstant*(1+servoFraction)];

        initialState = baseline.Y_OUT(1,:).';
        % Preserve mean speed while changing the difference in magnitudes.
        initialState(14) = -baseSpeed*(1+speedFraction/2);
        initialState(16) =  baseSpeed*(1-speedFraction/2);
        initialState(17:18) = 0;

        [time,state] = ode45(@CONTROL, [0 loop.cycleT], ...
            initialState, solverOptions, baseline.gains, baseline.gyro1, ...
            baseline.gyro2, baseline.auv, baseline.params, baseline.d, ...
            loop, config);
        [~,~,control] = TORQUE(time, state, baseline.gains, ...
            baseline.gyro1, baseline.gyro2, baseline.auv, baseline.params, ...
            baseline.d, loop, config);
        conditioning = DUAL_CMG_CONDITIONING_ANALYSIS(time, state, control, ...
            baseline.gyro1, baseline.gyro2, config.limits, config.allocator);

        roll = state(:,4);
        target = baseline.d.phi;
        results.settlingTime(servoIndex,speedIndex) = ...
            localSettlingTime(time, roll, target, 0.02);
        results.overshootPercent(servoIndex,speedIndex) = ...
            100*max(0,max(roll)-target)/abs(target);
        results.finalErrorDeg(servoIndex,speedIndex) = ...
            rad2deg(target-roll(end));
        results.peakPitchDeg(servoIndex,speedIndex) = ...
            rad2deg(max(abs(state(:,5))));
        results.peakYawDeg(servoIndex,speedIndex) = ...
            rad2deg(max(abs(state(:,6))));
        results.rollMomentRMSE(servoIndex,speedIndex) = ...
            sqrt(mean(control.momentError(:,1).^2));
        results.maximumConditionNumber(servoIndex,speedIndex) = ...
            conditioning.maximumConditionNumber;
        results.minimumSingularityDistanceDeg(servoIndex,speedIndex) = ...
            rad2deg(conditioning.minimumSingularityDistance);
        results.minimumRollCapacityMargin(servoIndex,speedIndex) = ...
            conditioning.minimumActiveCapacityMargin;
        results.rateLimitedSamples(servoIndex,speedIndex) = ...
            nnz(control.gimbalRateSaturated);
        results.accelLimitedSamples(servoIndex,speedIndex) = ...
            nnz(control.gimbalAccelSaturated);
    end
end

summary.maximumSettlingTime = max(results.settlingTime,[],'all');
summary.maximumAbsFinalErrorDeg = max(abs(results.finalErrorDeg),[],'all');
summary.maximumPeakPitchDeg = max(results.peakPitchDeg,[],'all');
summary.maximumPeakYawDeg = max(results.peakYawDeg,[],'all');
summary.maximumConditionNumber = max(results.maximumConditionNumber,[],'all');
summary.minimumSingularityDistanceDeg = ...
    min(results.minimumSingularityDistanceDeg,[],'all');
summary.minimumRollCapacityMargin = ...
    min(results.minimumRollCapacityMargin,[],'all');
summary.totalRateLimitedSamples = sum(results.rateLimitedSamples,'all');
summary.totalAccelLimitedSamples = sum(results.accelLimitedSamples,'all');

fprintf('\nDual-CMG mismatch sweep (%d cases)\n', prod(gridSize));
fprintf('Speed mismatch: %+g%% to %+g%% | servo mismatch: %+g%% to %+g%%\n', ...
    speedMismatchPercent(1), speedMismatchPercent(end), ...
    servoMismatchPercent(1), servoMismatchPercent(end));
fprintf('Worst settling time: %.4f s | worst |final error|: %.4f deg\n', ...
    summary.maximumSettlingTime, summary.maximumAbsFinalErrorDeg);
fprintf('Worst peak pitch/yaw: %.4f / %.4f deg\n', ...
    summary.maximumPeakPitchDeg, summary.maximumPeakYawDeg);
fprintf('Max condition: %.3f | min singularity distance: %.3f deg\n', ...
    summary.maximumConditionNumber, summary.minimumSingularityDistanceDeg);
fprintf('Min active roll-capacity margin: %.3fx | rate/accel limited samples: %d/%d\n', ...
    summary.minimumRollCapacityMargin, summary.totalRateLimitedSamples, ...
    summary.totalAccelLimitedSamples);

fig = figure('Name','Dual-CMG mismatch sensitivity');
tiledlayout(2,2,'TileSpacing','compact');
plotMap(results.settlingTime, speedMismatchPercent, servoMismatchPercent, ...
    'Settling time (s)');
plotMap(results.peakPitchDeg, speedMismatchPercent, servoMismatchPercent, ...
    'Peak |pitch| (deg)');
plotMap(results.peakYawDeg, speedMismatchPercent, servoMismatchPercent, ...
    'Peak |yaw| (deg)');
plotMap(results.maximumConditionNumber, speedMismatchPercent, ...
    servoMismatchPercent, 'Maximum condition number');
exportgraphics(fig, fullfile(scriptDir, 'Working Results', ...
    'DUAL_CMG_MISMATCH_SWEEP.png'), 'Resolution', 300);
save(fullfile(scriptDir, 'Working Results', ...
    'dual_cmg_mismatch_sweep.mat'), 'speedMismatchPercent', ...
    'servoMismatchPercent', 'results', 'summary');

function value = localSettlingTime(time, response, target, fraction)
    outside = abs(response-target) > fraction*abs(target);
    lastOutside = find(outside,1,'last');
    if isempty(lastOutside)
        value = time(1);
    elseif lastOutside == numel(time)
        value = inf;
    else
        value = time(lastOutside+1);
    end
end

function plotMap(values, speed, servo, titleText)
    nexttile;
    imagesc(speed,servo,values);
    axis xy;
    colorbar;
    xlabel('Flywheel-speed mismatch (%)');
    ylabel('CMG #2 servo-time mismatch (%)');
    title(titleText);
end
