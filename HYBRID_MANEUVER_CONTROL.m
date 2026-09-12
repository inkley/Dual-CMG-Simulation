function [request,diagnostics] = HYBRID_MANEUVER_CONTROL(state,d,config)
%HYBRID_MANEUVER_CONTROL Coordinate plane alignment with sway/yaw thrust.
% Thrusters are smoothly enabled only after roll alignment and roll-rate
% conditions are satisfied. Y regulates inertial displacement along the
% requested lateral direction; N aligns the vehicle longitudinal axis with
% the desired heading vector inside that maneuver plane.

phi = state(4); theta = state(5); psi = state(6);
uvw = state(7:9);
p = state(10); q = state(11); r = state(12);
rollCommand = ROLL_TO_PLANE_COMMAND([phi;theta;psi], ...
    d.rollToPlane.desiredLateralDirectionNED,phi, ...
    d.rollToPlane.bidirectionalThruster);
rollError = atan2(sin(rollCommand.rollAngle-phi), ...
    cos(rollCommand.rollAngle-phi));
rollRate = p+sin(phi)*tan(theta)*q+cos(phi)*tan(theta)*r;

angleGate = smoothGate(abs(rollError), ...
    config.hybrid.rollEnableAngle,config.hybrid.rollDisableAngle);
rateGate = smoothGate(abs(rollRate), ...
    config.hybrid.rollEnableRate,config.hybrid.rollDisableRate);
activation = angleGate*rateGate;

RbodyToNED = bodyToNED(phi,theta,psi);
position = state(1:3);
velocityNED = RbodyToNED*uvw;
lateral = d.hybrid.lateralDirectionNED(:);
lateral = lateral/norm(lateral);
lateralPosition = dot(position-d.hybrid.initialPositionNED(:),lateral);
lateralVelocity = dot(velocityNED,lateral);
lateralError = d.hybrid.lateralDisplacement-lateralPosition;
requestedAlongDirection = config.hybrid.KpLateral*lateralError ...
    -config.hybrid.KdLateral*lateralVelocity;

currentLongitudinal = RbodyToNED*[1;0;0];
currentNormal = RbodyToNED*[0;0;1];
desiredHeading = d.hybrid.desiredHeadingNED(:);
desiredHeading = desiredHeading/norm(desiredHeading);
headingError = atan2(dot(cross(currentLongitudinal,desiredHeading), ...
    currentNormal),dot(currentLongitudinal,desiredHeading));

request.Y = activation*rollCommand.thrusterPolarity ...
    *requestedAlongDirection;
request.N = activation*(config.hybrid.KpHeading*headingError ...
    -config.hybrid.KdHeading*r);

diagnostics.activation = activation;
diagnostics.rollCommand = rollCommand.rollAngle;
diagnostics.rollError = rollError;
diagnostics.rollRate = rollRate;
diagnostics.lateralPosition = lateralPosition;
diagnostics.lateralVelocity = lateralVelocity;
diagnostics.lateralError = lateralError;
diagnostics.headingError = headingError;
diagnostics.desiredHeadingNED = desiredHeading;
diagnostics.currentHeadingNED = currentLongitudinal;
end

function value = smoothGate(errorMagnitude,enableThreshold,disableThreshold)
    if disableThreshold <= enableThreshold
        error('CMG:InvalidHybridGate', ...
            'The roll disable threshold must exceed the enable threshold.');
    end
    normalized = (disableThreshold-errorMagnitude) ...
        /(disableThreshold-enableThreshold);
    normalized = min(max(normalized,0),1);
    value = normalized^2*(3-2*normalized);
end

function R = bodyToNED(phi,theta,psi)
    R = [cos(psi)*cos(theta), ...
        -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi), ...
        sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi); ...
        sin(psi)*cos(theta), ...
        cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi), ...
        -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi); ...
        -sin(theta),cos(theta)*sin(phi),cos(theta)*cos(phi)];
end
