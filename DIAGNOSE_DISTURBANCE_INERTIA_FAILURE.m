function DIAGNOSE_DISTURBANCE_INERTIA_FAILURE
% Read saved failure and isolate allocation, estimator, and servo effects.
root=fileparts(mfilename('fullpath'));
out=fullfile(root,'Working Results','roll_disturbance_envelope','unequal_inertia');
a=load(fullfile(out,'case_1.mat'));
n=numel(a.t); sigma=zeros(n,1); angle=zeros(n,1);
predicted=zeros(n,2); estimated=zeros(n,2);
for k=1:n
    x=a.x(k,:); alpha=x([13,15]);
    h=[a.a.b.gyro1.I*x(14),a.a.b.gyro2.I*x(16)];
    he=a.c.estimation.rotorInertia.*x([14,16]);
    B=-[h.*cos(alpha);h.*sin(alpha)];
    Be=-[he.*cos(alpha);he.*sin(alpha)];
    sv=svd(B); sigma(k)=sv(end);
    angle(k)=rad2deg(asin(abs(sin(alpha(2)-alpha(1)))));
    rates=a.h.commandedAlphadot(k,:).'+x(12);
    predicted(k,:)=(B*rates).'; estimated(k,:)=(Be*rates).';
end
requested=a.h.requestedMoment(:,1:2); actual=a.h.achievedMoment(:,1:2);
allocationResidual=estimated-requested;
estimationResidual=predicted-estimated;
servoResidual=actual-predicted;
assert(max(abs(allocationResidual+estimationResidual+servoResidual-(actual-requested)),[],'all')<1e-12);
[~,peak]=max(abs(a.x(:,4)-a.env.criteria.holdAngle));
[~,sing]=min(sigma);
indices=unique([peak;sing]);
summary=table(a.t(indices),rad2deg(a.x(indices,4)-a.env.criteria.holdAngle), ...
    angle(indices),sigma(indices),requested(indices,1),actual(indices,1), ...
    allocationResidual(indices,1),estimationResidual(indices,1),servoResidual(indices,1), ...
    'VariableNames',{'time','rollErrorDeg','singularityDistanceDeg','trueSigmaMin','requestedK','actualK','allocationK','estimationK','servoK'});
disp(summary); writetable(summary,fullfile(out,'diagnostic_key_times.csv'));
fprintf('Peak absolute K/M residuals [allocation; estimation; servo]:\n');
disp([max(abs(allocationResidual));max(abs(estimationResidual));max(abs(servoResidual))]);
violating=abs(rad2deg(a.x(:,4)-a.env.criteria.holdAngle))>a.env.criteria.peakRollErrorDeg;
fprintf('Recorded >2deg span %.3f to %.3f s; sampled duration %.3f s\n',a.t(find(violating,1)),a.t(find(violating,1,'last')),trapz(a.t,double(violating)));
save(fullfile(out,'allocation_diagnosis.mat'),'summary','sigma','angle','predicted','estimated','allocationResidual','estimationResidual','servoResidual');
% Counterfactual: same true unequal-inertia plant, exact allocator inertia.
% This is a diagnostic experiment, not a production-controller change.
c=a.c; c.estimation.rotorInertia=[a.a.b.gyro1.I,a.a.b.gyro2.I];
opts=odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',.01, ...
    'Events',@(t,x) stop(t,x,c.limits,a.env.diagnosticGimbalAngle));
[t,x,te]=ode45(@(t,x) CONTROL(t,x,a.a.b.gains,a.a.b.gyro1,a.a.b.gyro2,a.a.b.auv,a.a.b.params,a.a.d,a.loop,c),a.t,a.x(1,:).',opts);
[~,~,h]=TORQUE(t,x,a.a.b.gains,a.a.b.gyro1,a.a.b.gyro2,a.a.b.auv,a.a.b.params,a.a.d,a.loop,c);
e=rad2deg(x(:,4)-a.env.criteria.holdAngle); active=t>=10 & t<=190;
exact.peakErrorDeg=max(abs(e)); exact.rmsErrorDeg=sqrt(trapz(t(active),e(active).^2)/(t(find(active,1,'last'))-t(find(active,1))));
exact.recoveryErrorDeg=NaN; if any(t>=199), exact.recoveryErrorDeg=max(abs(e(t>=199))); end
exact.completed=isempty(te)&&t(end)>=200;
exact.limitedSamples=nnz(h.gimbalRateSaturated|h.gimbalAccelSaturated|h.gimbalAngleLimited|h.flywheelSpeedLimited|h.flywheelAccelSaturated);
H=a.a.b.gyro1.I*x(:,14).*sin(x(:,13))+a.a.b.gyro2.I*x(:,16).*sin(x(:,15));
exact.peakMomentumExcursion=max(abs(H-H(1)));
exact.pass=exact.completed && exact.peakErrorDeg<=2 && exact.rmsErrorDeg<=1 && exact.recoveryErrorDeg<=.5 && exact.limitedSamples==0 && exact.peakMomentumExcursion<=a.budget;
disp(exact);
save(fullfile(out,'exact_knowledge_diagnostic.mat'),'t','x','h','c','exact');
end
function [v,terminal,direction]=stop(~,x,l,angle)
v=[angle-max(abs(x([13,15])));l.maxFlywheelSpeed-max(abs(x([14,16])));deg2rad(80)-abs(x(5))];
terminal=ones(3,1); direction=-ones(3,1);
end
