function VERIFY_PLANE_PITCH_CORRECTION
root=CMG_ROOT();
b=load(fullfile(root,'Working Results','dual','symmetric_spin','VFR','simulation_result.mat'));
c=b.cmgConfig; c.hybrid.planePitchCorrection=true;
c.hybrid.planePitchKp=.5; c.hybrid.planePitchKd=2; c.hybrid.planePitchMaxMoment=.02;
d=b.d; d.hybrid.missionPhase="TURN"; d.hybrid.planeNormalNED=[0;0;1];
x=b.Y_OUT(1,:).'; x(4:6)=[0;-.01;0]; x(10:12)=0;
loop.cycleT=5; loop.fc=.2; loop.controlEndTime=inf;
[t,~,~]=CMG_ALLOCATE(0,x,b.gains,b.gyro1,b.gyro2,d,loop,c);
assert(abs(t.MD-.005)<1e-12,'Positive q must correct nose below plane.');
x(11)=.1; [t,~,~]=CMG_ALLOCATE(0,x,b.gains,b.gyro1,b.gyro2,d,loop,c);
assert(t.MD==-.02,'Pitch damping and request bound must apply.');
for phase=["ROLL","ABORT"]
    d.hybrid.missionPhase=phase;
    [t,~,~]=CMG_ALLOCATE(0,x,b.gains,b.gyro1,b.gyro2,d,loop,c);
    assert(t.MD==0);
end
c.hybrid.planePitchCorrection=false; d.hybrid.missionPhase="TURN";
[t,~,~]=CMG_ALLOCATE(0,x,b.gains,b.gyro1,b.gyro2,d,loop,c); assert(t.MD==0);
disp('Plane-pitch correction sign, damping, bound, disable and phase checks passed.');
end
