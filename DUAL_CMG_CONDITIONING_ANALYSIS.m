function result = DUAL_CMG_CONDITIONING_ANALYSIS( ...
        time, state, control, gyro1, gyro2, limits, allocator)
%DUAL_CMG_CONDITIONING_ANALYSIS Evaluate dual-CMG steering robustness.
% For B = -[h1*s1,h2*s2], det(B)=h1*h2*sin(alpha2-alpha1).
% A formal singularity occurs when either rotor momentum is zero or the two
% momentum-direction columns are parallel/antiparallel. Directional roll
% capacity additionally accounts for the pitch-neutral allocation and the
% configured gimbal-rate bound.

sampleCount = numel(time);
conditionNumber = nan(sampleCount,1);
determinant = nan(sampleCount,1);
sigmaMin = nan(sampleCount,1);
singularityDistance = nan(sampleCount,1);
pitchNeutralRollCapacity = zeros(sampleCount,1);

for index = 1:sampleCount
    alpha1 = state(index,13);
    alpha2 = state(index,15);
    h1 = gyro1.I*state(index,14);
    h2 = gyro2.I*state(index,16);
    B = -[h1*cos(alpha1), h2*cos(alpha2); ...
          h1*sin(alpha1), h2*sin(alpha2)];
    singularValues = svd(B);
    conditionNumber(index) = cond(B);
    determinant(index) = det(B);
    sigmaMin(index) = singularValues(end);
    singularityDistance(index) = asin(abs(sin(alpha2-alpha1)));

    if singularValues(end) > eps(singularValues(1))
        ratesPerUnitRoll = B\[1;0];
        pitchNeutralRollCapacity(index) = limits.maxGimbalRate ...
            / max(abs(ratesPerUnitRoll));
    end
end

requestedRoll = abs(control.requestedMoment(:,1));
capacityMargin = pitchNeutralRollCapacity./max(requestedRoll, eps);
active = requestedRoll > max(requestedRoll)*1e-3;

result.conditionNumber = conditionNumber;
result.determinant = determinant;
result.sigmaMin = sigmaMin;
result.singularityDistance = singularityDistance;
result.pitchNeutralRollCapacity = pitchNeutralRollCapacity;
result.capacityMargin = capacityMargin;
result.maximumConditionNumber = max(conditionNumber);
result.minimumAbsDeterminant = min(abs(determinant));
result.minimumSigma = min(sigmaMin);
result.minimumSingularityDistance = min(singularityDistance);
if any(active)
    result.minimumActiveCapacityMargin = min(capacityMargin(active));
    result.minimumActiveRollCapacity = min(pitchNeutralRollCapacity(active));
else
    % A zero-moment hold has no active demand against which to form a
    % capacity ratio. Treat it as feasible rather than returning empties.
    result.minimumActiveCapacityMargin = inf;
    result.minimumActiveRollCapacity = inf;
end
result.nearSingularSamples = nnz(sigmaMin < allocator.sigmaThreshold);
result.rateInfeasibleSamples = nnz(active ...
    & requestedRoll > pitchNeutralRollCapacity);
result.formallySingular = any(~isfinite(conditionNumber)) ...
    || result.minimumSigma <= eps(max(sigmaMin));
result.passesConditionThreshold = ...
    result.maximumConditionNumber < 1e3;
result.passesDirectionalCapacity = result.rateInfeasibleSamples == 0;
result.passes = ~result.formallySingular ...
    && result.nearSingularSamples == 0 ...
    && result.passesConditionThreshold ...
    && result.passesDirectionalCapacity;
result.passesFullKM = result.passes;
result.passesRollTask = result.passesDirectionalCapacity;
% Singularity avoidance is only required for the present controller if the
% commanded roll direction becomes infeasible. A rank loss in the unused
% pitch direction is retained as a reported condition, but is not by itself
% a failure of the roll-only task.
result.requiresSingularityAvoidance = ~result.passesRollTask;
end
