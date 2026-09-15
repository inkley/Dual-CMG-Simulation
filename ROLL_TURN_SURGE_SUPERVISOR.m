function [memory,command,metrics] = ROLL_TURN_SURGE_SUPERVISOR(t,x,memory,m)
% Pure sampled supervisor. No persistent state or ODE-side switching memory.
% A dwell is measured from the first qualifying sample, not accumulated over
% interrupted captures. Qualification is sampled, not a continuous-time proof.
if isempty(memory)
    memory.phase="ROLL"; memory.phaseStart=t; memory.qualifiedSince=NaN;
    memory.surgeOrigin=zeros(3,1); memory.reason=""; memory.hasSurged=false;
end
lateral=[0;cosd(m.planeDeg);sind(m.planeDeg)];
normal=cross([1;0;0],lateral);
target=[cosd(m.headingDeg);sind(m.headingDeg)*lateral(2:3)];
phi=x(4); theta=x(5); psi=x(6);
R=[cos(psi)*cos(theta),cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi),cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi); ...
sin(psi)*cos(theta),sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi),sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi); ...
-sin(theta),cos(theta)*sin(phi),cos(theta)*cos(phi)];
metrics.headingError=atan2(norm(cross(R(:,1),target)),dot(R(:,1),target));
metrics.planeError=acos(min(1,abs(dot(R(:,3),normal))));
metrics.angularRate=norm(x(10:12));
metrics.rollError=abs(atan2(sin(deg2rad(m.planeDeg)-phi),cos(deg2rad(m.planeDeg)-phi)));
elapsed=t-memory.phaseStart;
previousPhase=memory.phase;
if memory.phase=="ROLL"
    good=metrics.rollError<=m.rollCaptureAngle && metrics.angularRate<=m.rollCaptureRate;
    memory=qualify(memory,t,good);
    if good && t-memory.qualifiedSince>=m.rollDwell-1e-9
        memory=transition(memory,"TURN",t);
    elseif elapsed>=m.rollTimeout
        memory=transition(memory,"ABORT",t); memory.reason="roll capture timeout";
    end
elseif memory.phase=="TURN"
    good=elapsed>=m.turnDuration && metrics.headingError<=m.headingCaptureAngle ...
        && metrics.planeError<=m.planeCaptureAngle && metrics.angularRate<=m.turnCaptureRate;
    memory=qualify(memory,t,good);
    if good && t-memory.qualifiedSince>=m.turnDwell-1e-9
        memory=transition(memory,"SURGE",t); memory.surgeOrigin=x(1:3); memory.hasSurged=true;
    elseif elapsed>=m.turnTimeout
        memory=transition(memory,"ABORT",t); memory.reason="turn capture timeout";
    end
elseif memory.phase=="SURGE"
    if metrics.headingError>m.headingAbortAngle || metrics.planeError>m.planeAbortAngle ...
            || metrics.angularRate>m.angularRateAbort
        memory=transition(memory,"ABORT",t); memory.reason="surge alignment inhibit";
    elseif elapsed>=m.surgeDuration-1e-9
        memory=transition(memory,"COMPLETE",t);
    end
end
if memory.phase=="ABORT" && previousPhase~="ABORT"
    memory.abortHeading=R(:,1); % hold current nose direction, no new turn
end
command.phase=memory.phase;
command.lateral=lateral; command.targetHeading=target;
command.heading=target;
if memory.phase=="ROLL"
    command.heading=[1;0;0];
elseif memory.phase=="TURN"
    u=min(max((t-memory.phaseStart)/m.turnDuration,0),1);
    q=10*u^3-15*u^4+6*u^5;
    command.heading=[cosd(m.headingDeg*q);sind(m.headingDeg*q)*lateral(2:3)];
elseif memory.phase=="ABORT"
    command.heading=memory.abortHeading;
end
% COMPLETE marks the end of the recorded cruise segment, not a stop request.
command.propulsionEnabled=memory.phase=="SURGE" || memory.phase=="COMPLETE";
command.speedReference=0; command.speedReferenceRate=0;
if memory.phase=="COMPLETE"
    command.speedReference=m.surgeSpeed;
elseif memory.phase=="SURGE"
    u=min(max((t-memory.phaseStart)/m.speedRampTime,0),1);
    command.speedReference=m.surgeSpeed*(10*u^3-15*u^4+6*u^5);
    command.speedReferenceRate=m.surgeSpeed*(30*u^2-60*u^3+30*u^4)/m.speedRampTime;
end
command.crossTrackDirection=cosd(m.headingDeg)*lateral-sind(m.headingDeg)*[1;0;0];
end
function memory=qualify(memory,t,good)
if ~good, memory.qualifiedSince=NaN;
elseif isnan(memory.qualifiedSince), memory.qualifiedSince=t;
end
end
function memory=transition(memory,phase,t)
memory.phase=phase; memory.phaseStart=t; memory.qualifiedSince=NaN;
end
