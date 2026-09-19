function VERIFY_ROLL_DISTURBANCE_SETUP
p=ROLL_DISTURBANCE_TEST_PLAN; assert(numel(p.cases)==9);
s=p.load; s.amplitude=.03; t=(-1:.001:81).'; K=SINUSOIDAL_ROLL_LOAD(t,s);
assert(all(K(t<10 | t>70)==0) && max(abs(K))<=.03+eps);
assert(abs(SINUSOIDAL_ROLL_LOAD(11.25,s)-.03*.5*(1-cos(pi*1.25/2)))<1e-12);
s.amplitude=0; assert(all(SINUSOIDAL_ROLL_LOAD(t,s)==0));
root=fileparts(mfilename('fullpath'));
a=load(fullfile(root,'Working Results','roll_disturbance_tests','zero_reference.mat'));
assert(a.s.completed && a.s.meetsTrackingScreen);
assert(max(abs(a.h.rollDisturbance))==0 && max(abs(a.h.externalDumpMoment))==0);
assert(max(abs(a.x(:,4)-p.holdAngle))<1e-10);
disp('Bounded waveform, phase/sign, zero-load hold and no-unloading checks passed.');
end
