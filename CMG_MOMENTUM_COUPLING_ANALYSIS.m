function result = CMG_MOMENTUM_COUPLING_ANALYSIS( ...
        time, state, tau1, tau2, gyro1, gyro2, params)
%CMG_MOMENTUM_COUPLING_ANALYSIS Diagnose roll exchange and cross-axis terms.
% The roll balance compares vehicle Ix*p with rotor Hx = I*Omega*sin(alpha).
% The integral of the modeled hydrodynamic roll moment is included because
% hydrodynamic drag exchanges angular momentum with the surrounding fluid.
% This scalar check is intentionally separate from a closed-system vector
% conservation claim: REMUS includes added inertia and external fluid loads.

p = state(:,10);
q = state(:,11);
r = state(:,12);

rotorRollMomentum = gyro1.I*state(:,14).*sin(state(:,13)) ...
    + gyro2.I*state(:,16).*sin(state(:,15));
vehicleRollMomentum = params.Ix*p;
externalRollMoment = params.rollDragCoefficient*p.*abs(p);
externalRollImpulse = cumtrapz(time, externalRollMoment);

result.vehicleRollMomentumChange = ...
    vehicleRollMomentum-vehicleRollMomentum(1);
result.oppositeRotorRollMomentumChange = ...
    -(rotorRollMomentum-rotorRollMomentum(1));
result.oppositeRotorPlusExternalImpulse = ...
    result.oppositeRotorRollMomentumChange+externalRollImpulse;
result.rollMomentumBalanceResidual = result.vehicleRollMomentumChange ...
    - result.oppositeRotorPlusExternalImpulse;
result.peakVehicleRollMomentum = max(abs(result.vehicleRollMomentumChange));
result.peakOppositeRotorRollMomentumChange = ...
    max(abs(result.oppositeRotorRollMomentumChange));
result.maxRollMomentumBalanceResidual = ...
    max(abs(result.rollMomentumBalanceResidual));
result.normalizedRollMomentumBalanceResidual = ...
    result.maxRollMomentumBalanceResidual ...
    / max(result.peakVehicleRollMomentum, eps);

result.cmgPitchMoment = tau1.M+tau2.M;
result.cmgYawMoment = tau1.N+tau2.N;
% For diagonal dry-body inertia, the pitch equation contains the gyroscopic
% forcing (Iz-Ix)*p*r after moving CRB*nu to the right-hand side.
result.rigidBodyPitchCouplingMoment = (params.Iz-params.Ix)*p.*r;
result.rigidBodyYawCouplingMoment = (params.Ix-params.Iy)*p.*q;
result.peakAbsCmgPitchMoment = max(abs(result.cmgPitchMoment));
result.peakAbsCmgYawMoment = max(abs(result.cmgYawMoment));
result.peakAbsRigidBodyPitchCouplingMoment = ...
    max(abs(result.rigidBodyPitchCouplingMoment));
result.peakAbsRigidBodyYawCouplingMoment = ...
    max(abs(result.rigidBodyYawCouplingMoment));
result.peakAbsPitch = max(abs(state(:,5)));
result.peakAbsYaw = max(abs(state(:,6)));
end
