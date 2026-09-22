function VERIFY_CMG_ESTIMATION
root=CMG_ROOT();
b=load(fullfile(root,'Working Results','dual','symmetric_spin','VFR','simulation_result.mat'));
x=b.Y_OUT(1,:).'; c=b.cmgConfig;
[y,g1,g2]=CMG_CONTROLLER_ESTIMATE(x,b.gyro1,b.gyro2,c);
assert(isequal(x,y) && isequal(g1,b.gyro1) && isequal(g2,b.gyro2));
c.estimation.speedScaleBias=[.05,-.05];
c.estimation.speedOffsetRPM=[10,20];
c.estimation.rotorInertia=[b.gyro1.I*.9,b.gyro2.I*1.1];
[y,g1,g2]=CMG_CONTROLLER_ESTIMATE(x,b.gyro1,b.gyro2,c);
assert(norm(y([14,16])-(x([14,16]).*[1.05;.95]+[10;20]*2*pi/60))<1e-12);
assert(g1.I==b.gyro1.I*.9 && g2.I==b.gyro2.I*1.1);
other=setdiff(1:numel(x),[14,16]); assert(isequal(x(other),y(other)));
loop.cycleT=5; loop.fc=.2; loop.controlEndTime=inf;
plain=rmfield(c,'estimation'); zero=plain;
zero.estimation.speedScaleBias=[0,0];
zero.estimation.rotorInertia=[b.gyro1.I,b.gyro2.I];
a0=CONTROL(0,x,b.gains,b.gyro1,b.gyro2,b.auv,b.params,b.d,loop,plain);
a1=CONTROL(0,x,b.gains,b.gyro1,b.gyro2,b.auv,b.params,b.d,loop,zero);
assert(isequal(a0,a1),'Zero-error configuration must preserve derivatives exactly.');
[~,h]=CONTROL(0,x,b.gains,b.gyro1,b.gyro2,b.auv,b.params,b.d,loop,c);
[a,z]=CMG(b.gyro1,b.gyro2,h.contpar,x);
assert(norm(h.achievedMoment-[a.K+z.K;a.M+z.M;a.N+z.N])<1e-14);
assert(norm(h.estimatedRotorSpeed-y([14,16]))<1e-14);
disp('CMG estimator identity, bias convention, and plant-truth separation passed.');
end
