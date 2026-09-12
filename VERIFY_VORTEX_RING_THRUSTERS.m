% VERIFY_VORTEX_RING_THRUSTERS.m
% Verify fixed-thruster force/moment signs, actuator lag, and hard limits.

clearvars;
config.thruster.enabled = true;
config.thruster.positionBody = [0.65,-0.65;0,0;0,0];
config.thruster.maxForce = 5;
config.thruster.maxForceRate = 50;
config.thruster.timeConstant = 0.15;
state = zeros(20,1);

% Common mode: positive sway and zero yaw.
config.thruster.commandForce = [2;2];
state(19:20) = config.thruster.commandForce;
common = VORTEX_RING_THRUSTERS(0,state,config);
assert(abs(common.Y-4) < 1e-12 && abs(common.N) < 1e-12);
assert(abs(common.K) < 1e-12 && abs(common.M) < 1e-12);

% Differential mode: zero sway and positive yaw for fore-positive/aft-negative.
config.thruster.commandForce = [2;-2];
state(19:20) = config.thruster.commandForce;
differential = VORTEX_RING_THRUSTERS(0,state,config);
assert(abs(differential.Y) < 1e-12);
assert(abs(differential.N-2.6) < 1e-12);
assert(abs(differential.K) < 1e-12 && abs(differential.M) < 1e-12);

% At zero actual force, finite actuator dynamics must not jump instantly.
state(19:20) = 0;
transient = VORTEX_RING_THRUSTERS(0,state,config);
assert(all(transient.actualForce == 0));
assert(all(abs(transient.forceDot) <= config.thruster.maxForceRate));

% Oversized commands are clipped at the module force limit.
config.thruster.commandForce = [10;-10];
limited = VORTEX_RING_THRUSTERS(0,state,config);
assert(all(limited.limitedCommand == [5;-5]));
assert(limited.forceLimited);

fprintf(['Vortex-ring thruster checks passed: common Y %.2f N, ' ...
    'differential N %.2f N m.\n'],common.Y,differential.N);
