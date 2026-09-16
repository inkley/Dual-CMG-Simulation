function [measured,g1,g2] = CMG_CONTROLLER_ESTIMATE(state,gyro1,gyro2,config)
% Optional estimator-only uncertainty. Physical CMG dynamics retain truth.
% Omega_measured = Omega_true*(1+speedScaleBias) + speedOffsetRPM*2*pi/60.
% rotorInertia specifies absolute controller estimates, NOT plant multipliers.
% Defaults preserve existing simulations exactly. Only speed is measured with
% error here; angle/body-state sensing remains ideal. Speed-limit decisions
% in CMG_ALLOCATE consequently use measured speed; audits must check truth.
measured=state; g1=gyro1; g2=gyro2;
if ~isfield(config,'estimation'), return; end
e=config.estimation;
scale=zeros(2,1); offset=zeros(2,1);
if isfield(e,'speedScaleBias'), scale=e.speedScaleBias(:); end
if isfield(e,'speedOffsetRPM'), offset=e.speedOffsetRPM(:); end
assert(numel(scale)==2 && numel(offset)==2 && all(isfinite([scale;offset])));
measured([14,16])=state([14,16]).*(1+scale)+offset*2*pi/60;
if isfield(e,'rotorInertia')
    inertia=e.rotorInertia(:);
    assert(numel(inertia)==2 && all(isfinite(inertia)) && all(inertia>0));
    g1.I=inertia(1); g2.I=inertia(2);
end
end
