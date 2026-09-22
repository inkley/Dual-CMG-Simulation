function results=SWEEP_DISTURBANCE_INERTIA_ESTIMATES(errorPairs,bias,maxStep)
% Fixed unequal-inertia plant; errors are fractions of EACH TRUE inertia.
% Default: 3x3 grid at +/-5%, for the previously failing negative-bias load.
if nargin<1, [e1,e2]=ndgrid([-.05,0,.05]); errorPairs=[e1(:),e2(:)]; end
if nargin<2, bias=-.0005; end
if nargin<3, maxStep=.01; end
assert(size(errorPairs,2)==2 && all(isfinite(errorPairs),'all') && all(errorPairs>-1,'all'));
assert(isscalar(bias)&&abs(bias)<=.0005 && maxStep>0 && maxStep<=.01);
root=CMG_ROOT();
source=fullfile(root,'Working Results','roll_disturbance_envelope','unequal_inertia','case_1.mat');
a=load(source);
out=fullfile(root,'Working Results','inertia_estimation_bounds');
if ~isfolder(out), mkdir(out); end
trueInertia=[a.a.b.gyro1.I,a.a.b.gyro2.I];
rows=struct([]);
batchTag=sprintf('batch_n%d_first_%g_%g_last_%g_%g_bias_%+.6f_step_%g', ...
    size(errorPairs,1),errorPairs(1,:),errorPairs(end,:),bias,maxStep);
for k=1:size(errorPairs,1)
    c=a.c; c.estimation.rotorInertia=trueInertia.*(1+errorPairs(k,:));
    c.external.rollDisturbance=bias;
    opts=odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',maxStep, ...
        'Events',@(t,x) guard(t,x,c.limits,a.env.diagnosticGimbalAngle));
    [t,x,te]=ode45(@(t,x) CONTROL(t,x,a.a.b.gains,a.a.b.gyro1,a.a.b.gyro2,a.a.b.auv,a.a.b.params,a.a.d,a.loop,c), ...
        0:.01:200,a.x(1,:).',opts);
    [~,~,h]=TORQUE(t,x,a.a.b.gains,a.a.b.gyro1,a.a.b.gyro2,a.a.b.auv,a.a.b.params,a.a.d,a.loop,c);
    e=rad2deg(x(:,4)-a.env.criteria.holdAngle); active=t>=10 & t<=190;
    s.error1Percent=100*errorPairs(k,1); s.error2Percent=100*errorPairs(k,2); s.bias=bias; s.maxStep=maxStep;
    s.completed=isempty(te)&&t(end)>=200; s.endTime=t(end);
    s.peakErrorDeg=max(abs(e)); s.rmsErrorDeg=NaN;
    if nnz(active)>1, s.rmsErrorDeg=sqrt(trapz(t(active),e(active).^2)/(t(find(active,1,'last'))-t(find(active,1)))); end
    s.recoveryErrorDeg=NaN; if any(t>=199), s.recoveryErrorDeg=max(abs(e(t>=199))); end
    H=trueInertia(1)*x(:,14).*sin(x(:,13))+trueInertia(2)*x(:,16).*sin(x(:,15));
    s.peakMomentumExcursion=max(abs(H-H(1)));
    s.peakGimbalDeg=rad2deg(max(abs(x(:,[13,15])),[],'all'));
    s.peakPitchDeg=rad2deg(max(abs(x(:,5)))); s.peakYawDeg=rad2deg(max(abs(x(:,6))));
    s.limitedSamples=nnz(h.gimbalRateSaturated|h.gimbalAccelSaturated|h.gimbalAngleLimited|h.flywheelSpeedLimited|h.flywheelAccelSaturated);
    s.pass=s.completed && s.peakErrorDeg<=a.env.criteria.peakRollErrorDeg ...
        && s.rmsErrorDeg<=a.env.criteria.rmsRollErrorDeg && s.recoveryErrorDeg<=a.env.criteria.finalRecoveryErrorDeg ...
        && s.limitedSamples==0 && s.peakMomentumExcursion<=a.budget;
    tag=sprintf('e1_%+.6f_e2_%+.6f_bias_%+.6f_step_%g',errorPairs(k,:),bias,maxStep);
    save(fullfile(out,[tag,'.mat']),'t','x','h','c','s','a','trueInertia','te');
    if isempty(rows), rows=s; else, rows(end+1)=s; end %#ok<AGROW>
    results=struct2table(rows);
    writetable(results,fullfile(out,[batchTag,'.csv']));
    fprintf('Error [%+.2f,%+.2f]%% bias %+.4g: peak %.6f RMS %.6f, pass=%d\n',s.error1Percent,s.error2Percent,bias,s.peakErrorDeg,s.rmsErrorDeg,s.pass);
end
end
function [v,terminal,direction]=guard(~,x,l,angle)
v=[angle-max(abs(x([13,15])));l.maxFlywheelSpeed-max(abs(x([14,16])));deg2rad(80)-abs(x(5))];
terminal=ones(3,1); direction=-ones(3,1);
end
