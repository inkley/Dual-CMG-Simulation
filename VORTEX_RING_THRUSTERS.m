function thruster = VORTEX_RING_THRUSTERS(~,state,config)
%VORTEX_RING_THRUSTERS Low-order model of two fixed lateral VRT modules.
% Each module produces a cycle-averaged body-y force at its configured
% mounting point. The force state follows a bounded first-order response.
% This captures actuator lag and force/slew limits, but not individual vortex
% formation, circulation, pulse timing, or vehicle-vortex interaction.

thrusterConfig = config.thruster;
moduleCount = 2;
actualForce = zeros(moduleCount,1);
if numel(state) >= 20
    actualForce = state(19:20);
end

commandedForce = zeros(moduleCount,1);
if thrusterConfig.enabled
    commandedForce = thrusterConfig.commandForce(:);
end
limitedCommand = min(max(commandedForce, ...
    -thrusterConfig.maxForce),thrusterConfig.maxForce);
rawForceDot = (limitedCommand-actualForce)/thrusterConfig.timeConstant;
forceDot = min(max(rawForceDot,-thrusterConfig.maxForceRate), ...
    thrusterConfig.maxForceRate);

generalized = zeros(6,1);
for module = 1:moduleCount
    forceBody = [0;actualForce(module);0];
    momentBody = cross(thrusterConfig.positionBody(:,module),forceBody);
    generalized = generalized+[forceBody;momentBody];
end

thruster.commandedForce = commandedForce;
thruster.limitedCommand = limitedCommand;
thruster.actualForce = actualForce;
thruster.forceDot = forceDot;
thruster.generalizedForce = generalized;
thruster.X = generalized(1);
thruster.Y = generalized(2);
thruster.Z = generalized(3);
thruster.K = generalized(4);
thruster.M = generalized(5);
thruster.N = generalized(6);
thruster.forceLimited = any(commandedForce ~= limitedCommand);
thruster.forceRateLimited = any(abs(rawForceDot) > ...
    thrusterConfig.maxForceRate);
end
