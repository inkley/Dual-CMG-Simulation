function K=SINUSOIDAL_ROLL_LOAD(t,s)
% Synthetic body-roll moment (Nm), not a sea-state/hydrodynamic wave model.
% A raised-cosine envelope provides bounded, smooth onset and removal.
assert(all(isfinite([s.amplitude,s.period,s.startTime,s.duration,s.rampTime])) ...
    && s.amplitude>=0 && s.period>0 && s.startTime>=0 && s.duration>0 ...
    && s.rampTime>0 && 2*s.rampTime<=s.duration);
elapsed=t-s.startTime; envelope=zeros(size(t));
active=elapsed>=0 & elapsed<=s.duration;
u=elapsed(active);
envelope(active)=min(1,.5*(1-cos(pi*min(u,s.rampTime)/s.rampTime))) ...
    .*min(1,.5*(1-cos(pi*min(s.duration-u,s.rampTime)/s.rampTime)));
K=s.amplitude*envelope.*sin(2*pi*elapsed/s.period);
end
