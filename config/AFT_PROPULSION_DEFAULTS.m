function p = AFT_PROPULSION_DEFAULTS()
% Provisional forward-only net-thrust model; not IVER hardware ratings.
p.enabled = false;
p.commandForce = 0;        % N; direct thrust command, no speed loop yet
p.maxForce = 10;           % N; screening assumption
p.timeConstant = 0.5;      % s; commanded-thrust lag
p.maxForceRate = 10;       % N/s
p.positionBody = [-0.8;0;0]; % m; centerline aft, thrust along positive body x
p.modelStatus = 'provisional net-thrust model; shaft torque and inflow omitted';
end
