function [force,nextIntegral,diagnostics] = SURGE_SPEED_CONTROL( ...
        speed,reference,referenceRate,integral,dt,cfg,maxForce)
% Sampled PI with model-based feedforward and conditional anti-windup.
% Forward-only propulsion cannot actively brake. Negative demand clips to 0.
validateattributes([speed,reference,referenceRate,integral],{'numeric'},{'real','finite'});
validateattributes(dt,{'numeric'},{'scalar','positive','finite'});
validateattributes(maxForce,{'numeric'},{'scalar','positive','finite'});
validateattributes([cfg.Kp,cfg.Ki,cfg.dragCoefficient],{'numeric'},{'nonnegative','finite'});
validateattributes(cfg.effectiveMass,{'numeric'},{'scalar','positive','finite'});
error=reference-speed;
feedforward=cfg.effectiveMass*referenceRate+cfg.dragCoefficient*reference*abs(reference);
raw=feedforward+cfg.Kp*error+cfg.Ki*integral;
force=min(max(raw,0),maxForce);
integrate=(raw>=0 && raw<=maxForce) || (raw>maxForce && error<0) || (raw<0 && error>0);
nextIntegral=integral;
if cfg.Ki>0 && integrate
    nextIntegral=min(max(integral+dt*error,-maxForce/cfg.Ki),maxForce/cfg.Ki);
end
diagnostics.error=error; diagnostics.rawForce=raw;
diagnostics.feedforward=feedforward; diagnostics.saturated=(raw~=force);
diagnostics.integrating=integrate;
end
