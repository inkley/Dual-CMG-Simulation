function VERIFY_ACTUATOR_UNCERTAINTY
root=fileparts(mfilename('fullpath'));
b=load(fullfile(root,'Working Results','dual','symmetric_spin','VFR','simulation_result.mat'));
c=b.cmgConfig; c.propulsion=AFT_PROPULSION_DEFAULTS(); cases=ACTUATOR_UNCERTAINTY_CASES();
assert(isequal(c,ACTUATOR_PLANT_CONFIG(c)));
loop.cycleT=5; loop.fc=.2; loop.controlEndTime=inf; x=b.Y_OUT(1,:).';
[v,a]=CONTROL(0,x,b.gains,b.gyro1,b.gyro2,b.auv,b.params,b.d,loop,c);
c.actuatorUncertainty=cases(1).parameters;
[w,z]=CONTROL(0,x,b.gains,b.gyro1,b.gyro2,b.auv,b.params,b.d,loop,c);
assert(isequal(v,w));
c.actuatorUncertainty=cases(9).parameters;
[~,z]=CONTROL(0,x,b.gains,b.gyro1,b.gyro2,b.auv,b.params,b.d,loop,c);
assert(isequal(a.commandedContpar,z.commandedContpar),'Uncertainty must not alter allocator knowledge.');
p=ACTUATOR_PLANT_CONFIG(c);
assert(norm(p.gimbal.rateTimeConstant-c.gimbal.rateTimeConstant(:).*[.8;1.2])<1e-12);
p.thruster.enabled=true; p.thruster.commandForce=[1;1]; x(19:20)=0;
t=VORTEX_RING_THRUSTERS(0,x,p);
assert(norm(t.forceDot-1./p.thruster.timeConstant)<1e-12);
c.hybrid.enabled=false; c.thruster.enabled=true;
c.thruster.commandMode='direct_force'; c.thruster.commandForce=[1;1];
[~,z]=CONTROL(0,x,b.gains,b.gyro1,b.gyro2,b.auv,b.params,b.d,loop,c);
assert(norm(z.thruster.forceDot-[.9;1.1]./p.thruster.timeConstant)<1e-12);
assert(norm(z.thrusterAllocation.commandedForce-[1;1])<1e-12);
disp('Actuator identity, nominal allocation and unequal lag checks passed.');
end
