function results=INSTALLED_MASS_UNCERTAINTY_SWEEP
% Nine paired roll-only/complete mission cases; deterministic sensitivity screen.
% Total mass +/-10%, assembled rigid-body inertias +/-20%. No retuning.
root=fileparts(mfilename('fullpath'));
b=load(fullfile(root,'Working Results','dual','symmetric_spin','VFR','simulation_result.mat'));
scales=[1,1,1;.9,1,1;1.1,1,1;1,.8,1;1,1.2,1;1,1,.8;1,1,1.2;.9,.8,.8;1.1,1.2,1.2];
rows=struct([]); rolls=cell(size(scales,1),1); missions=rolls;
out=fullfile(root,'Working Results','installed_mass_uncertainty');
if ~isfolder(out), mkdir(out); end
for k=1:size(scales,1)
    [auv,params]=APPLY_INSTALLED_MASS_UNCERTAINTY(b.auv,b.params,scales(k,:));
    c=b.cmgConfig; c.hybrid.enabled=false; c.thruster.enabled=false;
    c.thruster.commandMode='direct_force'; c.thruster.commandForce=[0;0];
    c.propulsion=AFT_PROPULSION_DEFAULTS(); c.propulsion.enabled=false;
    loop.cycleT=5; loop.fc=.2; loop.controlEndTime=inf;
    options=odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',.005, ...
        'Events',@(t,x) angleStop(t,x,c.limits.maxGimbalAngle));
    [t,x]=ode45(@(t,x) CONTROL(t,x,b.gains,b.gyro1,b.gyro2,auv,params,b.d,loop,c), ...
        0:.01:5,b.Y_OUT(1,:).',options);
    [~,~,control]=TORQUE(t,x,b.gains,b.gyro1,b.gyro2,auv,params,b.d,loop,c);
    err=rad2deg(b.d.phi-x(:,4)); last=find(abs(err)>1.8,1,'last');
    settling=inf;
    if isempty(last), settling=0; elseif last<numel(t), settling=t(last+1); end
    rolls{k}=struct('t',t,'x',x,'control',control,'auv',auv,'params',params);
    mission=RUN_ROLL_TURN_SURGE(struct('installedMassScales',scales(k,:)));
    missions{k}=mission;
    s.caseIndex=k; s.massScale=scales(k,1); s.rollInertiaScale=scales(k,2);
    s.transverseInertiaScale=scales(k,3); s.massKg=auv.m;
    s.Ix=params.Ix; s.Iy=params.Iy; s.Iz=params.Iz;
    s.rollSettlingTime=settling; s.rollFinalErrorDeg=err(end);
    s.rollPass=t(end)>=5 && settling<=4 && abs(err(end))<=1.8;
    s.missionTerminal=mission.terminal; s.missionPass=mission.passes;
    s.finalHeadingErrorDeg=mission.finalHeadingErrorDeg;
    s.maxPlaneErrorDeg=mission.maxSurgePlaneErrorDeg;
    s.finalSpeed=mission.finalSpeed; s.progress=mission.finalProgress;
    s.crossTrack=mission.maxCrossTrack; s.outOfPlane=mission.maxOutOfPlane;
    s.peakAftForce=mission.peakAftForce; s.maxGimbalDeg=mission.maxGimbalDeg;
    s.limitSamples=sum(mission.limitedSamples);
    if isempty(rows), rows=s; else, rows(end+1)=s; end %#ok<AGROW>
    results=struct2table(rows);
    save(fullfile(out,'sweep.mat'),'results','rolls','missions','scales','b');
    writetable(results,fullfile(out,'summary.csv'));
end
disp(results);
end
function [value,terminal,direction]=angleStop(~,x,bound)
value=bound-max(abs(x([13,15]))); terminal=1; direction=-1;
end
