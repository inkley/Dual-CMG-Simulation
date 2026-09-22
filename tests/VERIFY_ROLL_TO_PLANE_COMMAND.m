% VERIFY_ROLL_TO_PLANE_COMMAND.m
% Unit checks for the roll-to-plane geometry and solution continuity.

clearvars;
tol = 1e-12;

east = ROLL_TO_PLANE_COMMAND([0;0;0],[0;1;0],0,true);
assert(abs(east.rollAngle) < tol);
assert(east.thrusterPolarity == 1);

down = ROLL_TO_PLANE_COMMAND([0;0;0],[0;0;1],0,true);
assert(abs(down.rollAngle-pi/2) < tol);
assert(down.alignmentAngle < tol);

% At 90 deg yaw, the body-y axis points north-negative at zero roll.
yawed = ROLL_TO_PLANE_COMMAND([0;0;pi/2],[-1;0;0],0,true);
assert(abs(yawed.rollAngle) < tol);
assert(yawed.alignmentAngle < tol);

% Reversible thrust makes antipodal directions share the same plane.
reverse = ROLL_TO_PLANE_COMMAND([0;0;0],[0;-1;0],0,true);
assert(abs(reverse.rollAngle) < tol);
assert(reverse.thrusterPolarity == -1);
assert(reverse.alignmentAngle < tol);

% The solution nearest the reference prevents an avoidable 180/360-deg roll.
continuous = ROLL_TO_PLANE_COMMAND([0;0;0],[0;0;1], ...
    deg2rad(100),true);
assert(abs(continuous.rollAngle-pi/2) < tol);

fprintf('Roll-to-plane command checks passed.\n');
fprintf('Level down-plane command: %.1f deg, thrust polarity %+d.\n', ...
    rad2deg(down.rollAngle),down.thrusterPolarity);
