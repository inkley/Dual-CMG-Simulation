function result = ROLL_TO_PLANE_ALIGNMENT( ...
        eulerAngles, desiredLateralDirectionNED, thrusterPolarity)
%ROLL_TO_PLANE_ALIGNMENT Measure achieved fixed-thruster plane alignment.
% Uses the REMUS 3-2-1 body-to-NED rotation. The desired direction is
% projected normal to the current vehicle longitudinal axis because surge
% content cannot be produced by the fixed body-y lateral thruster axis.

arguments
    eulerAngles (3,1) double {mustBeFinite}
    desiredLateralDirectionNED (3,1) double {mustBeFinite}
    thrusterPolarity (1,1) double {mustBeMember(thrusterPolarity,[-1,1])}
end

phi = eulerAngles(1);
theta = eulerAngles(2);
psi = eulerAngles(3);
Rz = [cos(psi),-sin(psi),0; sin(psi),cos(psi),0; 0,0,1];
Ry = [cos(theta),0,sin(theta); 0,1,0; -sin(theta),0,cos(theta)];
Rx = [1,0,0; 0,cos(phi),-sin(phi); 0,sin(phi),cos(phi)];
RbodyToNED = Rz*Ry*Rx;

desiredNorm = norm(desiredLateralDirectionNED);
if desiredNorm <= 1e-12
    error('CMG:ZeroManeuverDirection', ...
        'The desired lateral maneuver direction must be nonzero.');
end
desiredUnit = desiredLateralDirectionNED/desiredNorm;
longitudinalAxis = RbodyToNED*[1;0;0];
transverse = desiredUnit-longitudinalAxis*dot( ...
    longitudinalAxis,desiredUnit);
if norm(transverse) <= 1e-9
    error('CMG:UndefinedManeuverPlane', ...
        ['The requested direction is parallel to the vehicle longitudinal ' ...
        'axis; roll cannot define a lateral maneuver plane.']);
end
projectedDirection = transverse/norm(transverse);
actualThrustDirection = thrusterPolarity*RbodyToNED*[0;1;0];
alignmentAngle = atan2(norm(cross(actualThrustDirection, ...
    projectedDirection)),dot(actualThrustDirection,projectedDirection));

result.alignmentAngle = alignmentAngle;
result.actualThrustDirectionNED = actualThrustDirection;
result.projectedDesiredDirectionNED = projectedDirection;
result.longitudinalComponent = dot(longitudinalAxis,desiredUnit);
end
