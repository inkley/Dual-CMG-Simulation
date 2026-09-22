function REFINE_DISTURBANCE_INERTIA_FAILURE
% Independent half-step check of the unequal-inertia/negative-bias failure.
root=CMG_ROOT();
out=fullfile(root,'Working Results','roll_disturbance_envelope','unequal_inertia');
a=load(fullfile(out,'case_1.mat'));
opts=odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',.005);
[t,x]=ode45(@(t,x) CONTROL(t,x,a.a.b.gains,a.a.b.gyro1,a.a.b.gyro2, ...
    a.a.b.auv,a.a.b.params,a.a.d,a.loop,a.c),a.t,a.x(1,:).',opts);
assert(max(abs(x(:,[13,15])),[],'all')<a.env.diagnosticGimbalAngle);
assert(max(abs(x(:,[14,16])),[],'all')<a.c.limits.maxFlywheelSpeed);
assert(max(abs(x(:,5)))<deg2rad(80));
[peakErrorDeg,index]=max(abs(rad2deg(x(:,4)-a.env.criteria.holdAngle)));
peakTime=t(index);
maxRollDifferenceDeg=max(abs(rad2deg(x(:,4)-a.x(:,4))));
fprintf('Refined failure: peak %.8f deg at %.3f s; common-grid max change %.8g deg\n',peakErrorDeg,peakTime,maxRollDifferenceDeg);
assert(peakErrorDeg>a.env.criteria.peakRollErrorDeg,'Failure not reproduced.');
assert(maxRollDifferenceDeg<.001,'Step-sensitive result; further refinement needed.');
save(fullfile(out,'refinement.mat'),'t','x','peakErrorDeg','peakTime','maxRollDifferenceDeg');
end
