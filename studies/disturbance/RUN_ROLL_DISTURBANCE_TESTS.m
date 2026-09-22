function results=RUN_ROLL_DISTURBANCE_TESTS(runLoadedCases)
% Default validates only zero-load reference; true opts into planned loads.
if nargin<1, runLoadedCases=false; end
assert(islogical(runLoadedCases) && isscalar(runLoadedCases));
root=CMG_ROOT(); plan=ROLL_DISTURBANCE_TEST_PLAN;
b=load(fullfile(root,'Working Results','dual','symmetric_spin','VFR','simulation_result.mat'));
ids=1; if runLoadedCases, ids=1:numel(plan.cases); end
out=fullfile(root,'Working Results','roll_disturbance_tests');
if ~isfolder(out), mkdir(out); end
rows=struct([]);
for k=ids
    test=plan.cases(k); c=b.cmgConfig;
    c.external.rollDisturbance=0; c.external.sinusoidalRoll=plan.load;
    c.external.sinusoidalRoll.amplitude=test.amplitude; c.external.sinusoidalRoll.period=test.period;
    c.momentumManagement.enabled=false; c.hybrid.enabled=false;
    c.hybrid.planePitchCorrection=test.feedback;
    c.hybrid.planePitchKp=.5; c.hybrid.planePitchKd=2; c.hybrid.planePitchMaxMoment=.02;
    c.thruster.enabled=false; c.thruster.commandMode='direct_force'; c.thruster.commandForce=[0;0];
    c.propulsion=AFT_PROPULSION_DEFAULTS(); c.propulsion.enabled=false;
    d=struct('phi',plan.holdAngle,'theta',0,'psi',0);
    d.hybrid.missionPhase="TURN";
    d.hybrid.planeNormalNED=[0;-sin(plan.holdAngle);cos(plan.holdAngle)];
    x0=b.Y_OUT(1,:).'; x0(1:12)=0; x0(4)=plan.holdAngle; x0(17:20)=0;
    loop.cycleT=plan.endTime; loop.fc=1/plan.endTime; loop.controlEndTime=inf;
    if ~test.feedback, loop.controlEndTime=-inf; end
    opts=odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',plan.maxStep, ...
        'Events',@(t,x) bounds(t,x,c.limits));
    [t,x,te]=ode45(@(t,x) CONTROL(t,x,b.gains,b.gyro1,b.gyro2,b.auv,b.params,d,loop,c), ...
        0:plan.sampleTime:plan.endTime,x0,opts);
    [~,~,h]=TORQUE(t,x,b.gains,b.gyro1,b.gyro2,b.auv,b.params,d,loop,c);
    errorDeg=rad2deg(x(:,4)-plan.holdAngle);
    active=t>=plan.load.startTime & t<=plan.load.startTime+plan.load.duration;
    recovery=t>=plan.endTime-1;
    s.name=test.name; s.feedback=test.feedback; s.completed=isempty(te) && t(end)>=plan.endTime;
    s.endTime=t(end); s.peakRollErrorDeg=max(abs(errorDeg)); s.rmsLoadedErrorDeg=NaN;
    if nnz(active)>1, s.rmsLoadedErrorDeg=sqrt(trapz(t(active),errorDeg(active).^2)/(t(find(active,1,'last'))-t(find(active,1)))); end
    s.recoveryErrorDeg=NaN; if any(recovery), s.recoveryErrorDeg=max(abs(errorDeg(recovery))); end
    s.peakLoadNm=max(abs(h.rollDisturbance));
    H=b.gyro1.I*x(:,14).*sin(x(:,13))+b.gyro2.I*x(:,16).*sin(x(:,15));
    s.maxMomentumChange=max(abs(H-H(1)));
    s.limitedSamples=nnz(h.gimbalRateSaturated | h.gimbalAccelSaturated ...
        | h.flywheelAccelSaturated | h.gimbalAngleLimited | h.flywheelSpeedLimited);
    s.peakPitchDeg=max(abs(rad2deg(x(:,5)))); s.peakYawDeg=max(abs(rad2deg(x(:,6))));
    s.meetsTrackingScreen=s.completed && s.peakRollErrorDeg<=plan.peakRollErrorDeg ...
        && s.rmsLoadedErrorDeg<=plan.rmsRollErrorDeg ...
        && s.recoveryErrorDeg<=plan.finalRecoveryErrorDeg && s.limitedSamples==0;
    % Passive cases are references, not claimed controller passes/failures.
    if isempty(rows), rows=s; else, rows(end+1)=s; end %#ok<AGROW>
    save(fullfile(out,[char(test.name),'.mat']),'t','x','h','s','c','d','loop','plan','b');
end
results=struct2table(rows);
tag='zero_reference_summary'; if runLoadedCases, tag='loaded_summary'; end
writetable(results,fullfile(out,[tag,'.csv'])); disp(results);
end
function [v,terminal,direction]=bounds(~,x,l)
v=[l.maxGimbalAngle-max(abs(x([13,15])));l.maxFlywheelSpeed-max(abs(x([14,16]))); ...
    deg2rad(80)-abs(x(5))];
terminal=[1;1;1]; direction=[-1;-1;-1];
end
