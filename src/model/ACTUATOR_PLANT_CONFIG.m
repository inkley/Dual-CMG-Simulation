function p=ACTUATOR_PLANT_CONFIG(c)
% Plant-only uncertain response; allocator/controller retain nominal config.
p=c;
if ~isfield(c,'actuatorUncertainty'), return; end
u=c.actuatorUncertainty;
names={'gimbalLag','gimbalAccel','vrtLag','vrtSlew','vrtGain','aftLag','aftSlew'};
for k=1:numel(names)
    v=u.(names{k}); assert(all(isfinite(v(:))) && all(v(:)>0));
end
assert(numel(u.gimbalLag)==2 && numel(u.vrtLag)==2 && numel(u.vrtGain)==2);
assert(isscalar(u.gimbalAccel) && isscalar(u.vrtSlew) && isscalar(u.aftLag) && isscalar(u.aftSlew));
p.gimbal.rateTimeConstant=c.gimbal.rateTimeConstant(:).*u.gimbalLag(:);
p.limits.maxGimbalAccel=c.limits.maxGimbalAccel*u.gimbalAccel;
p.thruster.timeConstant=c.thruster.timeConstant.*u.vrtLag(:);
p.thruster.maxForceRate=c.thruster.maxForceRate*u.vrtSlew;
p.thruster.forceGain=u.vrtGain(:);
if isfield(c,'propulsion')
    p.propulsion.timeConstant=c.propulsion.timeConstant*u.aftLag;
    p.propulsion.maxForceRate=c.propulsion.maxForceRate*u.aftSlew;
end
end
