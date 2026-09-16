function result = RUN_ROLL_TURN_SURGE(overrides)
%RUN_ROLL_TURN_SURGE Sampled roll-turn-cruise mission using current actuators.
% Starts from rest and stops recording at completion of a cruise segment;
% COMPLETE does not mean zero speed or arrival at a waypoint.
if nargin<1, overrides=struct(); end
root=fileparts(mfilename('fullpath'));
b=load(fullfile(root,'Working Results','dual','symmetric_spin','VFR','simulation_result.mat'));
m=ROLL_TURN_SURGE_DEFAULTS();
for name=string(fieldnames(overrides)).'
    assert(isfield(m,name),'Unknown mission option %s',name); m.(name)=overrides.(name);
end
assert(m.turnDuration>0 && m.speedRampTime>0 && m.sampleTime>0 && m.surgeDuration>=m.speedRampTime);
assert(abs(m.planeDeg)<=90 && abs(m.headingDeg)<90, ...
    'This demonstrator requires |plane|<=90 and |heading|<90 degrees.');
assert(strcmp(b.cmgConfig.mode,'dual') && strcmp(b.cmgConfig.dualController,'constant_speed'));
m.speed.effectiveMass=b.auv.m+.93;
[b.auv,b.params]=APPLY_INSTALLED_MASS_UNCERTAINTY( ...
    b.auv,b.params,m.installedMassScales);
cfg=b.cmgConfig; cfg.propulsion=AFT_PROPULSION_DEFAULTS();
if m.actuatorCase~=0
    cases=ACTUATOR_UNCERTAINTY_CASES();
    assert(isscalar(m.actuatorCase) && m.actuatorCase==fix(m.actuatorCase) ...
        && m.actuatorCase>=1 && m.actuatorCase<=numel(cases));
    cfg.actuatorUncertainty=cases(m.actuatorCase).parameters;
end
cfg.thruster.enabled=true; cfg.thruster.commandMode='generalized_force'; cfg.hybrid.enabled=true;
cfg.momentumManagement.enabled=false; cfg.external.rollDisturbance=0;
cfg.hybrid.rollEnableAngle=deg2rad(.5); cfg.hybrid.rollDisableAngle=deg2rad(1.5);
cfg.hybrid.rollEnableRate=deg2rad(.5); cfg.hybrid.rollDisableRate=deg2rad(3);
cfg.hybrid.KpLateral=20; cfg.hybrid.KdLateral=30; cfg.hybrid.KpHeading=2; cfg.hybrid.KdHeading=6;
d=b.d; d.rollToPlane.bidirectionalThruster=true;
d.hybrid.lateralDisplacement=0; d.hybrid.initialPositionNED=zeros(3,1);
x=b.Y_OUT(1,:).'; x(1:12)=0; x(17:21)=0;
loop.cycleT=100; loop.fc=.01; loop.controlEndTime=inf;
options=odeset('RelTol',b.simConfig.relTol,'AbsTol',b.simConfig.absTol, ...
    'MaxStep',m.maxStep,'Events',@(t,x) stops(t,x,cfg.limits.maxGimbalAngle));
t=0; memory=[]; integral=0; k=0; histories=struct([]); events=struct([]);
previous=""; terminal="";
maxTime=m.rollTimeout+m.turnTimeout+m.surgeDuration+m.abortCoastTime+1;
while t<=maxTime
    [memory,cmd,metrics]=ROLL_TURN_SURGE_SUPERVISOR(t,x,memory,m);
    if memory.phase~=previous
        ev.time=t; ev.phase=memory.phase; ev.reason=memory.reason;
        if isempty(events), events=ev; else, events(end+1)=ev; end %#ok<AGROW>
        fprintf('t=%.2f %s %s\n',t,memory.phase,memory.reason); previous=memory.phase;
    end
    current=cfg; current.propulsion.enabled=cmd.propulsionEnabled;
    if cmd.propulsionEnabled
        [current.propulsion.commandForce,integral,speed]=SURGE_SPEED_CONTROL( ...
            x(7),cmd.speedReference,cmd.speedReferenceRate,integral,m.sampleTime,m.speed,cfg.propulsion.maxForce);
    else
        current.propulsion.commandForce=0; integral=0;
        speed.saturated=false; speed.rawForce=0;
    end
    reference=d; reference.rollToPlane.desiredLateralDirectionNED=cmd.lateral;
    reference.hybrid.desiredHeadingNED=cmd.heading;
    reference.hybrid.lateralDirectionNED=cmd.lateral;
    if memory.phase=="SURGE" || memory.phase=="COMPLETE" || ...
            (memory.phase=="ABORT" && memory.hasSurged)
        reference.hybrid.lateralDirectionNED=cmd.crossTrackDirection;
        reference.hybrid.initialPositionNED=memory.surgeOrigin;
    end
    [~,data]=CONTROL(t,x,b.gains,b.gyro1,b.gyro2,b.auv,b.params,reference,loop,current);
    row.time=t; row.state=x.'; row.phase=memory.phase;
    row.headingError=metrics.headingError; row.planeError=metrics.planeError;
    row.angularRate=metrics.angularRate; row.speedReference=cmd.speedReference;
    row.headingReference=cmd.heading.'; row.speedIntegral=integral;
    row.speedRawForce=speed.rawForce;
    row.propulsionCommand=current.propulsion.commandForce; row.propulsionForce=x(21);
    row.crossTrack=dot(x(1:3)-memory.surgeOrigin,cmd.crossTrackDirection);
    row.outOfPlane=dot(x(1:3)-memory.surgeOrigin,cross([1;0;0],cmd.lateral));
    row.pathProgress=dot(x(1:3)-memory.surgeOrigin,cmd.targetHeading);
    row.gate=data.hybrid.activation; row.speedSaturated=speed.saturated;
    row.thrusterForce=data.thruster.actualForce.';
    row.gimbalRate=data.actuator.actualGimbalRate.'; row.gimbalAccel=data.actuator.gimbalAccel.';
    h=[b.gyro1.I*x(14),b.gyro2.I*x(16)];
    B=-[h.*cos(x([13,15]).');h.*sin(x([13,15]).')];
    required=data.requestedMoment(1:2)-B*[x(12);x(12)];
    feasibleRates=pinv(B)*required;
    row.rollFeasible=norm(B*feasibleRates-required)<1e-7 ...
        && max(abs(feasibleRates))<=cfg.limits.maxGimbalRate+1e-8;
    row.limited=[data.allocation.gimbalRateSaturated,data.actuator.gimbalAccelSaturated, ...
        data.thrusterAllocation.saturated,data.propulsion.forceLimited,data.propulsion.forceRateLimited, ...
        data.thruster.forceLimited,data.thruster.forceRateLimited];
    k=k+1; if k==1, histories=row; else, histories(k)=row; end
    if memory.phase=="COMPLETE", terminal="COMPLETE"; break; end
    if memory.phase=="ABORT" && t-memory.phaseStart>=m.abortCoastTime-1e-9
        terminal="ABORT"; break;
    end
    [tt,xx,te]=ode45(@(tt,xx) CONTROL(tt,xx,b.gains,b.gyro1,b.gyro2, ...
        b.auv,b.params,reference,loop,current),[t,t+m.sampleTime],x,options);
    t=tt(end); x=xx(end,:).';
    if ~isempty(te), terminal="ANGLE_STOP"; break; end
end
if terminal=="", terminal="TIMEOUT"; end
history=struct2table(histories); result.terminal=terminal; result.events=struct2table(events);
result.finalHeadingErrorDeg=rad2deg(history.headingError(end));
result.finalSpeed=history.state(end,7); result.finalProgress=history.pathProgress(end);
surge=history.phase=="SURGE";
result.maxSurgeHeadingErrorDeg=NaN; result.maxSurgePlaneErrorDeg=NaN; result.maxCrossTrack=NaN;
result.finalSpeedError=NaN;
result.maxOutOfPlane=NaN; result.peakSpeed=max(history.state(:,7));
if any(surge)
    result.maxSurgeHeadingErrorDeg=rad2deg(max(history.headingError(surge)));
    result.maxSurgePlaneErrorDeg=rad2deg(max(history.planeError(surge)));
    result.maxCrossTrack=max(abs(history.crossTrack(surge)));
    result.maxOutOfPlane=max(abs(history.outOfPlane(surge)));
    result.finalSpeedError=abs(result.finalSpeed-m.surgeSpeed);
end
result.peakAftForce=max(history.propulsionForce);
result.maxGimbalDeg=rad2deg(max(abs([history.state(:,[13,15]);x([13,15]).']),[],'all'));
result.limitedSamples=sum(history.limited,1);
result.preSurgeForce=max(history.propulsionForce(history.phase=="ROLL" | history.phase=="TURN"));
result.rollInfeasibleSamples=nnz(~history.rollFeasible);
result.passes=terminal=="COMPLETE" && result.maxSurgeHeadingErrorDeg<=1 ...
    && result.maxSurgePlaneErrorDeg<=1 && result.maxCrossTrack<=.05 ...
    && result.finalSpeedError<=.025 && result.finalProgress>=5 ...
    && result.preSurgeForce<1e-9 && ~any(result.limitedSamples) ...
    && result.maxOutOfPlane<=.05 && result.rollInfeasibleSamples==0;
out=fullfile(root,'Working Results','roll_turn_surge', ...
    sprintf('plane_%g_heading_%g_dt_%g_timeout_%g',m.planeDeg,m.headingDeg,m.sampleTime,m.rollTimeout));
if ~isfolder(out), mkdir(out); end
if any(m.installedMassScales~=1)
    out=fullfile(out,sprintf('mass_%g_roll_%g_transverse_%g',m.installedMassScales));
    if ~isfolder(out), mkdir(out); end
end
if m.actuatorCase~=0
    out=fullfile(out,['actuator_',char(cases(m.actuatorCase).name)]);
    if ~isfolder(out), mkdir(out); end
end
save(fullfile(out,'mission.mat'),'result','history','m','b','cfg','x','t');
writetable(result.events,fullfile(out,'events.csv')); disp(result);
fig=figure('Visible','off','Position',[100 100 1200 850]); tiledlayout(2,2);
nexttile; plot3(history.state(:,1),history.state(:,2),-history.state(:,3),'LineWidth',1.5); hold on;
plot3(history.state(1,1),history.state(1,2),-history.state(1,3),'go','MarkerFaceColor','g');
plot3(history.state(end,1),history.state(end,2),-history.state(end,3),'ro','MarkerFaceColor','r');
grid on; axis equal; xlabel('North (m)'); ylabel('East (m)'); zlabel('Up (m)'); title('Travel after turn');
nexttile; plot(history.time,rad2deg([history.headingError,history.planeError]),'LineWidth',1.5);
grid on; xlabel('Time (s)'); ylabel('Error (deg)'); legend('Full heading','Fixed plane');
nexttile; plot(history.time,[history.speedReference,history.state(:,7)],'LineWidth',1.5);
grid on; xlabel('Time (s)'); ylabel('Surge speed (m/s)'); legend('Reference','Actual','Location','northwest');
nexttile; plot(history.time,[history.propulsionCommand,history.propulsionForce],'LineWidth',1.5);
grid on; xlabel('Time (s)'); ylabel('Aft thrust (N)'); legend('Command','Actual');
for tile=2:4
    ax=nexttile(tile);
    for e=2:height(result.events)
        xline(ax,result.events.time(e),':',result.events.phase(e),'HandleVisibility','off');
    end
end
exportgraphics(fig,fullfile(out,'ROLL_TURN_SURGE.png'),'Resolution',200); close(fig);
end
function [value,isterminal,direction]=stops(~,x,limit)
value=limit-max(abs(x([13,15]))); isterminal=1; direction=-1;
end
