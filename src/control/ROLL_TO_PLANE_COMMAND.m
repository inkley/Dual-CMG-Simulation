function command = ROLL_TO_PLANE_COMMAND( ...
        eulerAngles, desiredLateralDirectionNED, referenceRoll, bidirectional)
%ROLL_TO_PLANE_COMMAND Align the fixed body-y thruster axis with a plane.
% The desired maneuver plane contains the vehicle longitudinal body-x axis
% and a requested inertial lateral direction. Rolling the vehicle rotates
% the fixed body-y thruster axis within the transverse body y-z plane.
%
% Inputs use the REMUS 3-2-1 Euler and NED conventions:
%   eulerAngles = [phi; theta; psi] (rad)
%   desiredLateralDirectionNED = [north; east; down]
%   referenceRoll selects the continuous/nearest equivalent solution.
%   bidirectional allows the same physical thruster axis to be used with
%   reversed thrust, making roll solutions separated by pi equivalent.

arguments
    eulerAngles (3,1) double {mustBeFinite}
    desiredLateralDirectionNED (3,1) double {mustBeFinite}
    referenceRoll (1,1) double {mustBeFinite} = eulerAngles(1)
    bidirectional (1,1) logical = true
end

theta = eulerAngles(2);
psi = eulerAngles(3);
desiredNorm = norm(desiredLateralDirectionNED);
if desiredNorm <= 1e-12
    error('CMG:ZeroManeuverDirection', ...
        'The desired lateral maneuver direction must be nonzero.');
end
desiredUnit = desiredLateralDirectionNED/desiredNorm;

% Remove yaw and pitch while deliberately excluding roll. In this pre-roll
% frame, the positive body-y thruster axis is [0;cos(phi);sin(phi)].
Rz = [cos(psi),-sin(psi),0; sin(psi),cos(psi),0; 0,0,1];
Ry = [cos(theta),0,sin(theta); 0,1,0; -sin(theta),0,cos(theta)];
preRollDirection = (Rz*Ry).'*desiredUnit;
transverseDirection = [0;preRollDirection(2);preRollDirection(3)];
transverseNorm = norm(transverseDirection);
if transverseNorm <= 1e-9
    error('CMG:UndefinedManeuverPlane', ...
        ['The requested direction is parallel to the vehicle longitudinal ' ...
        'axis; roll cannot define a lateral maneuver plane.']);
end
transverseUnit = transverseDirection/transverseNorm;
rawRoll = atan2(transverseUnit(3),transverseUnit(2));

period = 2*pi;
if bidirectional
    period = pi;
end
% Search equivalent solutions explicitly. Listing the raw solution first
% makes exact distance ties deterministic and prefers positive thrust over
% an equally short reversed-thrust maneuver.
equivalentTurns = [0,-1,1,-2,2];
candidates = rawRoll+period*equivalentTurns;
[~,nearest] = min(abs(candidates-referenceRoll));
rollAngle = candidates(nearest);

Rx = [1,0,0; 0,cos(rollAngle),-sin(rollAngle); ...
      0,sin(rollAngle), cos(rollAngle)];
positiveThrusterAxisNED = Rz*Ry*Rx*[0;1;0];
axisDot = dot(positiveThrusterAxisNED,desiredUnit);
thrusterPolarity = 1;
if bidirectional && axisDot < 0
    thrusterPolarity = -1;
end
alignedThrustDirection = thrusterPolarity*positiveThrusterAxisNED;
projectedDirectionNED = Rz*Ry*transverseUnit;
alignmentAngle = atan2(norm(cross(alignedThrustDirection, ...
    projectedDirectionNED)),dot(alignedThrustDirection, ...
    projectedDirectionNED));
requestedDirectionResidualAngle = atan2(norm(cross( ...
    alignedThrustDirection,desiredUnit)),dot(alignedThrustDirection, ...
    desiredUnit));

command.rollAngle = rollAngle;
command.rawRollAngle = rawRoll;
command.thrusterPolarity = thrusterPolarity;
command.desiredDirectionNED = desiredUnit;
command.projectedDirectionNED = projectedDirectionNED;
command.positiveThrusterAxisNED = positiveThrusterAxisNED;
command.alignedThrustDirectionNED = alignedThrustDirection;
command.alignmentAngle = alignmentAngle;
command.requestedDirectionResidualAngle = requestedDirectionResidualAngle;
command.outOfPlaneComponent = preRollDirection(1);
command.bidirectional = bidirectional;
end
