function results=VERIFY_ROLL_DISTURBANCE_ENVELOPE(variant)
% Fixed-gain nonlinear checks of a deliberately conservative operating box.
root=CMG_ROOT();
out=fullfile(root,'Working Results','roll_disturbance_envelope');
if ~isfolder(out), mkdir(out); end
base=load(fullfile(root,'Working Results','roll_disturbance_tests','zero_reference.mat'));
env=ROLL_DISTURBANCE_ENVELOPE;
if nargin>0
    out=fullfile(out,char(variant.name));
    if ~isfolder(out), mkdir(out); end
    base.c.estimation.rotorInertia=[base.b.gyro1.I,base.b.gyro2.I];
    base.c.estimation.speedScaleBias=variant.speedBias;
    base.b.gyro1.I=base.b.gyro1.I*variant.inertiaScale(1);
    base.b.gyro2.I=base.b.gyro2.I*variant.inertiaScale(2);
    uncertainties=ACTUATOR_UNCERTAINTY_CASES;
    base.c.actuatorUncertainty=uncertainties(1).parameters;
    base.c.actuatorUncertainty.gimbalLag=variant.gimbalLag;
    base.c.actuatorUncertainty.gimbalAccel=variant.gimbalAccel;
    env.testPeriods=[15,15]; env.testBias=[-env.maxAbsBiasNm,env.maxAbsBiasNm];
end
% Local linear frequency sweep, not a nonlinear continuum guarantee.
frequency=linspace(env.frequencyHz(1),env.frequencyHz(2),1001).';
z=2i*pi*frequency;
gain=1./abs(base.b.params.Ix*z.^2+(base.b.gains.Kpp+base.b.gains.Kdp*z)./(1+mean(base.c.gimbal.rateTimeConstant)*z));
dcError=env.maxAbsBiasNm/base.b.gains.Kpp;
predictedPeakDeg=rad2deg(env.maxAmplitudeNm*gain+dcError);
predictedRmsDeg=rad2deg(sqrt((env.maxAmplitudeNm*gain).^2/2+dcError^2));
assert(max(predictedPeakDeg)<env.criteria.peakRollErrorDeg);
assert(max(predictedRmsDeg)<env.criteria.rmsRollErrorDeg);
writetable(table(frequency,predictedPeakDeg,predictedRmsDeg),fullfile(out,'linear_frequency_screen.csv'));
x0=base.x(1,:).';
H0=base.b.gyro1.I*x0(14)*sin(x0(13))+base.b.gyro2.I*x0(16)*sin(x0(15));
Hguard=(abs(base.b.gyro1.I*x0(14))+abs(base.b.gyro2.I*x0(16)))*sin(env.diagnosticGimbalAngle);
headroom=Hguard-abs(H0); % Worst sign, fixed-speed symmetric branch.
% Conservative planning estimate: biased impulse + sinusoidal impulse +
% predicted oscillatory body momentum. Nonlinear histories are checked below.
plannedExcursion=env.maxAbsBiasNm*env.maxTotalDuration ...
    +2*env.maxAmplitudeNm/(2*pi*env.frequencyHz(1)) ...
    +base.b.params.Ix*max(2*pi*frequency.*env.maxAmplitudeNm.*gain);
budget=(1-env.momentumReserveFraction)*headroom;
assert(plannedExcursion<budget);
fprintf('Linear predicted peak/RMS: %.4f / %.4f deg; planning momentum %.6f / %.6f N m s\n', ...
    max(predictedPeakDeg),max(predictedRmsDeg),plannedExcursion,budget);
rows=struct([]);
for k=1:numel(env.testPeriods)
    a=rmfield(base,'s'); c=a.c;
    c.external.rollDisturbance=env.testBias(k);
    c.external.sinusoidalRoll=struct('amplitude',env.maxAmplitudeNm,'period',env.testPeriods(k), ...
        'startTime',env.loadStart,'duration',env.loadDuration,'rampTime',env.rampTime);
    loop=a.loop; loop.cycleT=env.maxTotalDuration; loop.fc=1/loop.cycleT;
    opts=odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',.01, ...
        'Events',@(t,x) guard(t,x,c.limits,env.diagnosticGimbalAngle));
    [t,x,te,~,ie]=ode45(@(t,x) CONTROL(t,x,a.b.gains,a.b.gyro1,a.b.gyro2,a.b.auv,a.b.params,a.d,loop,c), ...
        0:.01:env.maxTotalDuration,x0,opts);
    [~,~,h]=TORQUE(t,x,a.b.gains,a.b.gyro1,a.b.gyro2,a.b.auv,a.b.params,a.d,loop,c);
    errorDeg=rad2deg(x(:,4)-env.criteria.holdAngle);
    active=t>=env.loadStart & t<=env.loadStart+env.loadDuration;
    recovery=t>=env.maxTotalDuration-1;
    H=a.b.gyro1.I*x(:,14).*sin(x(:,13))+a.b.gyro2.I*x(:,16).*sin(x(:,15));
    s.period=env.testPeriods(k); s.bias=env.testBias(k); s.completed=isempty(te)&&t(end)>=env.maxTotalDuration;
    s.peakErrorDeg=max(abs(errorDeg));
    s.rmsErrorDeg=sqrt(trapz(t(active),errorDeg(active).^2)/(t(find(active,1,'last'))-t(find(active,1))));
    s.recoveryErrorDeg=NaN; if any(recovery), s.recoveryErrorDeg=max(abs(errorDeg(recovery))); end
    s.peakMomentumExcursion=max(abs(H-H0)); s.finalMomentumChange=H(end)-H0;
    s.peakGimbalDeg=rad2deg(max(abs(x(:,[13,15])),[],'all'));
    s.peakPitchDeg=rad2deg(max(abs(x(:,5))));
    s.peakYawDeg=rad2deg(max(abs(x(:,6))));
    s.peakTrueSpeedRPM=max(abs(x(:,[14,16])),[],'all')*60/(2*pi);
    s.limitedSamples=nnz(h.gimbalRateSaturated|h.gimbalAccelSaturated|h.gimbalAngleLimited|h.flywheelSpeedLimited|h.flywheelAccelSaturated);
    s.pass=s.completed && s.peakErrorDeg<=env.criteria.peakRollErrorDeg ...
        && s.rmsErrorDeg<=env.criteria.rmsRollErrorDeg && s.recoveryErrorDeg<=env.criteria.finalRecoveryErrorDeg ...
        && s.limitedSamples==0 && s.peakMomentumExcursion<=budget;
    assert(max(abs(h.rollDisturbance-(c.external.rollDisturbance+SINUSOIDAL_ROLL_LOAD(t,c.external.sinusoidalRoll))))<1e-12);
    assert(all(h.externalDumpMoment==0));
    if isempty(rows), rows=s; else, rows(end+1)=s; end %#ok<AGROW>
    save(fullfile(out,sprintf('case_%d.mat',k)),'a','env','c','loop','t','x','h','s','te','ie','budget','plannedExcursion');
    fprintf('Envelope case %d/%d: pass=%d, peak/RMS=%.4f/%.4f deg\n',k,numel(env.testPeriods),s.pass,s.peakErrorDeg,s.rmsErrorDeg);
end
results=struct2table(rows); disp(results);
writetable(results,fullfile(out,'envelope_summary.csv'));
if nargin==0
    assert(all(results.pass),'One or more proposed-envelope cases failed; inspect saved summaries.');
end % Uncertainty screening retains failures and continues the matrix.
end

function [v,terminal,direction]=guard(~,x,l,angle)
v=[angle-max(abs(x([13,15])));l.maxFlywheelSpeed-max(abs(x([14,16])));deg2rad(80)-abs(x(5))];
terminal=ones(3,1); direction=-ones(3,1);
end
