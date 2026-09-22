function results=EVALUATE_ROLL_DISTURBANCE_REJECTION(experimentIds)
% Diagnose tracking and finite momentum capacity without retuning.
% Pass 3 to reuse completed refinement/zero-mean runs and repeat bias only.
if nargin<1, experimentIds=1:3; end
root=CMG_ROOT();
out=fullfile(root,'Working Results','roll_disturbance_tests');
plan=ROLL_DISTURBANCE_TEST_PLAN;
rows=struct([]);
for k=find([plan.cases.feedback])
    a=load(fullfile(out,char(plan.cases(k).name)+".mat"));
    rows=append(rows,measure(a,plan.cases(k).name));
end
base=load(fullfile(out,'A0.03_T15_feedback1.mat'));
for experiment=1:3
    a=rmfield(base,'s'); % Do not carry the old 80-second summary into new runs.
    if experiment==1
        name="fine_step"; a.plan.sampleTime=.005; a.plan.maxStep=.005;
    else
        a.plan.load.duration=180; a.plan.endTime=200;
        a.c.external.sinusoidalRoll.duration=180;
        name="extended_zero_mean";
        if experiment==3
            name="extended_bias_0p003Nm";
            % Deliberate ideal constant bias from t=0, separate stress test.
            a.c.external.rollDisturbance=.003;
        end
    end
    a.loop.cycleT=a.plan.endTime; a.loop.fc=1/a.plan.endTime;
    if ~ismember(experiment,experimentIds)
        cached=load(fullfile(out,char(name)+".mat"));
        if isfield(cached,'s')
            cached=rmfield(cached,'s');
            save(fullfile(out,char(name)+".mat"),'-struct','cached');
        end
        rows=append(rows,measure(cached,name));
        continue
    end
    guardLimits=a.c.limits;
    if experiment==3
        % Diagnostic stop before the symmetric roll steering singularity.
        % This does not change the physical 100-degree gimbal limit.
        guardLimits.maxGimbalAngle=deg2rad(85);
    end
    a.diagnosticGuardLimits=guardLimits;
    opts=odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',a.plan.maxStep, ...
        'Events',@(t,x) guard(t,x,guardLimits));
    [a.t,a.x,a.eventTime,~,a.eventIndex]=ode45(@(t,x) CONTROL(t,x,a.b.gains, ...
        a.b.gyro1,a.b.gyro2,a.b.auv,a.b.params,a.d,a.loop,a.c), ...
        0:a.plan.sampleTime:a.plan.endTime,a.x(1,:).',opts);
    [~,~,a.h]=TORQUE(a.t,a.x,a.b.gains,a.b.gyro1,a.b.gyro2,a.b.auv,a.b.params,a.d,a.loop,a.c);
    a.analysis=measure(a,name); rows=append(rows,a.analysis);
    save(fullfile(out,char(name)+".mat"),'-struct','a');
    fprintf('Completed diagnostic %s at t=%.3f s\n',name,a.t(end));
end
results=struct2table(rows); disp(results);
writetable(results,fullfile(out,'rejection_momentum_diagnostics.csv'));
end

function s=measure(a,name)
t=a.t; x=a.x; h=a.h; cfg=a.c.external.sinusoidalRoll;
e=x(:,4)-a.plan.holdAngle;
H=a.b.gyro1.I*x(:,14).*sin(x(:,13))+a.b.gyro2.I*x(:,16).*sin(x(:,15));
% Valid scalar balance for the symmetric, pure-roll experiment only.
assert(max(abs(x(:,[5,6,11,12])),[],'all')<1e-8);
external=h.rollDisturbance+a.b.params.rollDragCoefficient*x(:,10).*abs(x(:,10));
balance=a.b.params.Ix*(x(:,10)-x(1,10))+H-H(1)-cumtrapz(t,external);
s.name=name; s.endTime=t(end); s.completed=t(end)>=a.plan.endTime;
s.peakErrorDeg=max(abs(rad2deg(e)));
active=t>=cfg.startTime & t<=cfg.startTime+cfg.duration;
s.rmsErrorDeg=sqrt(trapz(t(active),rad2deg(e(active)).^2)/(t(find(active,1,'last'))-t(find(active,1))));
s.peakMomentumExcursion=max(abs(H-H(1))); s.finalMomentumChange=H(end)-H(1);
s.peakBalanceResidual=max(abs(balance));
s.peakGimbalDeg=rad2deg(max(abs(x(:,[13,15])),[],'all'));
s.peakRate=max(abs(h.alphadot),[],'all');
s.peakAccel=max(abs(h.gimbalAccel),[],'all');
s.peakMomentResidual=max(abs(h.achievedMoment(:,1)-h.requestedMoment(:,1)));
s.limitedSamples=nnz(h.gimbalRateSaturated|h.gimbalAccelSaturated|h.gimbalAngleLimited|h.flywheelSpeedLimited|h.flywheelAccelSaturated);
% Fit late full cycles, omitting onset/removal ramps. Not a proof of linearity.
fitMask=t>=cfg.startTime+2*cfg.period & t<=min(t(end),cfg.startTime+cfg.duration-cfg.period);
s.fittedAmplitudeDeg=NaN; s.linearAmplitudeDeg=NaN; s.cycleMomentumDrift=NaN;
if nnz(fitMask)>10
    w=2*pi/cfg.period;
    fit=[sin(w*(t(fitMask)-cfg.startTime)),cos(w*(t(fitMask)-cfg.startTime)),ones(nnz(fitMask),1)]\e(fitMask);
    s.fittedAmplitudeDeg=rad2deg(hypot(fit(1),fit(2)));
    servoTau=mean(a.c.gimbal.rateTimeConstant);
    z=1i*w;
    s.linearAmplitudeDeg=rad2deg(cfg.amplitude/abs(a.b.params.Ix*z^2+(a.b.gains.Kpp+a.b.gains.Kdp*z)/(1+servoTau*z)));
    cycleTimes=(cfg.startTime+2*cfg.period:cfg.period:min(t(end),cfg.startTime+cfg.duration-cfg.period)).';
    if numel(cycleTimes)>1
        cyc=interp1(t,H,cycleTimes); drift=polyfit(cycleTimes,cyc,1);
        s.cycleMomentumDrift=drift(1);
    end
end
end

function rows=append(rows,s)
if isempty(rows), rows=s; else, rows(end+1)=s; end
end

function [v,terminal,direction]=guard(~,x,l)
v=[l.maxGimbalAngle-max(abs(x([13,15])));l.maxFlywheelSpeed-max(abs(x([14,16])));deg2rad(80)-abs(x(5))];
terminal=ones(3,1); direction=-ones(3,1);
end
