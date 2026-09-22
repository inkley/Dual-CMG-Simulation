function results = OBLIQUE_PRELOAD_SHAPING_SWEEP(selectedCases,dt)
% Compare preload and heading-reference shaping without changing controllers.
% Fixed +/-45-degree plane/heading targets, zero lateral displacement.
% Stop at the configured gimbal bound: post-stop dynamics are unvalidated.
% All cases have a common 40-second budget. Shaping uses a quintic angle
% reference in the fixed inertial maneuver plane, not Euler-yaw interpolation.
if nargin<1, selectedCases=1:20; end
if nargin<2, dt=.01; end
validateattributes(selectedCases,{'numeric'},{'vector','integer','>=',1,'<=',20});
validateattributes(dt,{'numeric'},{'scalar','positive','finite'});
root=CMG_ROOT();
b=load(fullfile(root,'Working Results','dual','symmetric_spin','VFR','simulation_result.mat'));
assert(strcmp(b.cmgConfig.mode,'dual') && ...
    strcmp(b.cmgConfig.dualController,'constant_speed'), ...
    'Requires the constant-speed dual baseline.');
out=fullfile(root,'Working Results','oblique_preload_shaping');
if ~isequal(selectedCases,1:20) || dt~=.01
    out=fullfile(out,'refinement');
end
if ~isfolder(out), mkdir(out); end
% [signed preload degrees (alpha1=-preload), start, shaping duration]
variants=[15,0,0;5,0,0;30,0,0;45,0,0;-15,0,0; ...
    15,4,0;15,4,4;15,4,8;15,4,16;15,4,24];
names=["baseline step";"5 deg preload";"30 deg preload"; ...
    "45 deg preload";"reversed preload";"delayed step"; ...
    "4 s shaping";"8 s shaping";"16 s shaping";"24 s shaping"];
assert(profile(-1,0,4)==0 && profile(0,0,4)==0 && profile(4,0,4)==1);
assert(abs(profile(2,0,4)-.5)<eps);
duration=40; histories=cell(numel(selectedCases),1); rows=histories;
for caseIndex=1:numel(selectedCases)
    j=selectedCases(caseIndex);
    v=mod(j-1,10)+1; direction=1-2*(j>10);
    x0=b.Y_OUT(1,:).'; x0(1:12)=0; x0(17:20)=0;
    x0([13,15])=deg2rad([-variants(v,1),variants(v,1)]);
    d=b.d; cfg=b.cmgConfig;
    cfg.initial.alpha=x0([13,15]).'; cfg.initial.Omega=x0([14,16]).';
    lateral=[0;cosd(direction*45);sind(direction*45)];
    targetAngle=deg2rad(direction*45);
    target=[cos(targetAngle);sin(targetAngle)*lateral(2:3)];
    d.rollToPlane.desiredLateralDirectionNED=lateral;
    d.rollToPlane.bidirectionalThruster=true;
    d.hybrid.initialPositionNED=zeros(3,1);
    d.hybrid.lateralDirectionNED=lateral; d.hybrid.lateralDisplacement=0;
    cfg.hybrid.enabled=true; cfg.thruster.enabled=true;
    cfg.thruster.commandMode='generalized_force';
    cfg.momentumManagement.enabled=false;
    cfg.hybrid.rollEnableAngle=deg2rad(.5); cfg.hybrid.rollDisableAngle=deg2rad(1.5);
    cfg.hybrid.rollEnableRate=deg2rad(.5); cfg.hybrid.rollDisableRate=deg2rad(3);
    cfg.hybrid.KpLateral=20; cfg.hybrid.KdLateral=30;
    cfg.hybrid.KpHeading=2; cfg.hybrid.KdHeading=6;
    loop.cycleT=duration; loop.fc=1/duration; loop.controlEndTime=inf;
    start=variants(v,2); ramp=variants(v,3);
    options=odeset('RelTol',b.simConfig.relTol,'AbsTol',b.simConfig.absTol, ...
        'MaxStep',dt,'Events',@(t,x) stopAtBound(t,x,cfg.limits.maxGimbalAngle));
    [t,x,te]=ode45(@rhs,0:dt:duration,x0,options);
    n=numel(t); gate=zeros(n,1); fullError=gate; planeError=gate;
    rollError=gate; rawYaw=gate; yaw=gate; shapedAngle=gate;
    rate=zeros(n,2); accel=rate; force=rate; moments=rate;
    flags=false(n,3);
    for k=1:n
        [~,c]=rhs(t(k),x(k,:).');
        R=rotation(x(k,4:6)); nose=R(:,1);
        fullError(k)=rad2deg(atan2(norm(cross(nose,target)),dot(nose,target)));
        cmd=ROLL_TO_PLANE_COMMAND(x(k,4:6).',lateral,x(k,4),true);
        align=ROLL_TO_PLANE_ALIGNMENT(x(k,4:6).',lateral,cmd.thrusterPolarity);
        planeError(k)=rad2deg(align.alignmentAngle);
        gate(k)=c.hybrid.activation; rollError(k)=rad2deg(c.hybrid.rollError);
        rate(k,:)=c.actuator.actualGimbalRate.';
        accel(k,:)=c.actuator.gimbalAccel.';
        force(k,:)=c.thruster.actualForce.';
        moments(k,:)=[c.requestedMoment(1),c.achievedMoment(1)];
        rawYaw(k)=2*c.hybrid.headingError-6*x(k,12);
        yaw(k)=c.thruster.N;
        shapedAngle(k)=rad2deg(targetAngle*profile(t(k),start,ramp));
        flags(k,:)=[c.allocation.gimbalRateSaturated,c.actuator.gimbalAccelSaturated, ...
            c.thrusterAllocation.saturated];
    end
    active=gate>=.95; activeMax=inf;
    if any(active), activeMax=max(planeError(active)); end
    normal=cross([1;0;0],lateral);
    lateralError=abs(x(end,1:3)*lateral); drift=abs(x(end,1:3)*normal);
    finalHold=t>=duration-5;
    held=isempty(te) && any(finalHold) && all(fullError(finalHold)<=1);
    s.variant=names(v); s.planeDeg=direction*45; s.preloadDeg=variants(v,1);
    s.shapeSeconds=ramp; s.startSeconds=start; s.endTime=t(end);
    s.hitAngleStop=~isempty(te); s.finalHeadingErrorDeg=fullError(end);
    s.finalGate=gate(end); s.maxActivePlaneErrorDeg=activeMax;
    s.finalLateralError=lateralError; s.finalOutOfPlaneDrift=drift;
    s.maxGimbalDeg=rad2deg(max(abs(x(:,[13,15])),[],'all'));
    s.peakGimbalRate=max(abs(rate),[],'all'); s.peakGimbalAccel=max(abs(accel),[],'all');
    s.peakForce=max(abs(force),[],'all'); s.rateLimited=any(flags(:,1));
    s.accelLimited=any(flags(:,2)); s.thrusterSaturated=any(flags(:,3));
    s.headingHeldLast5Seconds=held;
    s.passes=held && gate(end)>=.95 && activeMax<=1 && lateralError<=.015 ...
        && drift<=.015 && ~any(flags,'all') && ~s.hitAngleStop;
    rows{caseIndex}=s;
    histories{caseIndex}=struct('time',t,'state',x,'gate',gate,'fullHeadingError',fullError, ...
        'planeError',planeError,'rollError',rollError,'rawYaw',rawYaw, ...
        'actualYaw',yaw,'shapedAngle',shapedAngle,'moments',moments, ...
        'config',cfg,'desired',d,'initialState',x0);
    fprintf('%+g plane %s: end %.2f heading %.3f gate %.3f stop %d pass %d\n', ...
        s.planeDeg,s.variant,s.endTime,s.finalHeadingErrorDeg,s.finalGate,s.hitAngleStop,s.passes);
    results=struct2table(vertcat(rows{1:caseIndex}));
    save(fullfile(out,'sweep.mat'),'results','histories','variants','b','duration','dt','selectedCases');
    writetable(results,fullfile(out,'summary.csv'));
end
disp(results);

    function [dx,c]=rhs(t,x)
        reference=d;
        angle=targetAngle*profile(t,start,ramp);
        reference.hybrid.desiredHeadingNED=[cos(angle);sin(angle)*lateral(2:3)];
        [dx,c]=CONTROL(t,x,b.gains,b.gyro1,b.gyro2,b.auv,b.params,reference,loop,cfg);
    end
end

function y=profile(t,start,duration)
if duration==0, y=double(t>=start); return; end
u=min(max((t-start)/duration,0),1); y=10*u^3-15*u^4+6*u^5;
end

function [value,terminal,direction]=stopAtBound(~,x,limit)
value=limit-max(abs(x([13,15]))); terminal=1; direction=-1;
end

function R=rotation(e)
p=e(1); t=e(2); s=e(3);
R=[cos(s)*cos(t),cos(s)*sin(t)*sin(p)-sin(s)*cos(p),cos(s)*sin(t)*cos(p)+sin(s)*sin(p); ...
sin(s)*cos(t),sin(s)*sin(t)*sin(p)+cos(s)*cos(p),sin(s)*sin(t)*cos(p)-cos(s)*sin(p); ...
-sin(t),cos(t)*sin(p),cos(t)*cos(p)];
end
