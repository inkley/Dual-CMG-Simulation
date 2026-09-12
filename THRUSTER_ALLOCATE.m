function allocation = THRUSTER_ALLOCATE(requestedForceMoment,thrusterConfig)
%THRUSTER_ALLOCATE Map requested body sway force/yaw moment to two modules.
% The fixed modules produce body-y forces at their configured body positions.
% A common scale factor enforces module force limits while retaining the
% requested Y:N direction whenever the unconstrained request is infeasible.

arguments
    requestedForceMoment (2,1) double {mustBeFinite}
    thrusterConfig struct
end

positions = thrusterConfig.positionBody;
if ~isequal(size(positions),[3,2])
    error('CMG:InvalidThrusterGeometry', ...
        'thruster.positionBody must be a 3-by-2 [fore,aft] matrix.');
end

% For force F_i*[0;1;0], Y_i=F_i and N_i=x_i*F_i.
B = [1,1;positions(1,1),positions(1,2)];
if rcond(B) < 1e-12
    error('CMG:SingularThrusterAllocation', ...
        'Fore and aft thrusters require distinct longitudinal positions.');
end
unconstrainedForce = B\requestedForceMoment;
peakUtilization = max(abs(unconstrainedForce))/thrusterConfig.maxForce;
allocationScale = min(1,1/max(peakUtilization,1));
commandedForce = allocationScale*unconstrainedForce;
achievedForceMoment = B*commandedForce;

allocation.matrix = B;
allocation.conditionNumber = cond(B);
allocation.requestedForceMoment = requestedForceMoment;
allocation.unconstrainedForce = unconstrainedForce;
allocation.commandedForce = commandedForce;
allocation.achievedForceMoment = achievedForceMoment;
allocation.residual = requestedForceMoment-achievedForceMoment;
allocation.scale = allocationScale;
allocation.saturated = allocationScale < 1-10*eps;
allocation.peakUnconstrainedUtilization = peakUtilization;
end
