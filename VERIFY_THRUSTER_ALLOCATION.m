% VERIFY_THRUSTER_ALLOCATION.m
% Verify common, differential, combined, and saturated allocations.

clearvars;
config.positionBody = [0.65,-0.65;0,0;0,0];
config.maxForce = 5;
tol = 1e-12;

common = THRUSTER_ALLOCATE([4;0],config);
assert(norm(common.commandedForce-[2;2]) < tol);
assert(norm(common.residual) < tol && ~common.saturated);

differential = THRUSTER_ALLOCATE([0;2.6],config);
assert(norm(differential.commandedForce-[2;-2]) < tol);
assert(norm(differential.residual) < tol && ~differential.saturated);

combined = THRUSTER_ALLOCATE([3;0.65],config);
assert(norm(combined.commandedForce-[2;1]) < tol);
assert(norm(combined.residual) < tol);

saturated = THRUSTER_ALLOCATE([20;0],config);
assert(norm(saturated.commandedForce-[5;5]) < tol);
assert(saturated.saturated && abs(saturated.scale-0.5) < tol);
assert(norm(saturated.achievedForceMoment-[10;0]) < tol);

fprintf(['Thruster allocation checks passed: common, differential, ' ...
    'combined, and direction-preserving saturation.\n']);
