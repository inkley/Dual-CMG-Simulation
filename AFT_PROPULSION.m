function propulsion = AFT_PROPULSION(state,config)
%AFT_PROPULSION Forward-only axial thrust with finite lag and slew limit.
% Optional state 21 is actual force in N. Missing configuration means off.
% Disabling requests zero thrust; a nonzero force state decays continuously.
% Net force/moment are [F; r cross F], not motor/shaft reaction torque.
if isfield(config,'propulsion')
    p=config.propulsion;
else
    p=AFT_PROPULSION_DEFAULTS();
end
validateattributes(p.enabled,{'logical','numeric'},{'scalar','binary'});
validateattributes(p.commandForce,{'numeric'},{'scalar','real','finite'});
validateattributes(p.maxForce,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(p.timeConstant,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(p.maxForceRate,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(p.positionBody,{'numeric'},{'numel',3,'real','finite'});
if p.enabled && numel(state)<21
    error('CMG:MissingPropulsionState','Enabled propulsion requires force state 21.');
end
actual=0;
if numel(state)>=21, actual=state(21); end
validateattributes(actual,{'numeric'},{'scalar','real','finite'});
% Initial force must be inside the envelope; tiny numerical excursions are
% tolerated but never silently clipped in the applied force calculation.
if actual < -1e-6 || actual > p.maxForce+1e-6
    error('CMG:PropulsionForceOutsideEnvelope','Aft force state is outside [0,maxForce].');
end
command=0; if p.enabled, command=p.commandForce; end
limited=min(max(command,0),p.maxForce);
rawDot=(limited-actual)/p.timeConstant;
forceDot=min(max(rawDot,-p.maxForceRate),p.maxForceRate);
forceBody=[actual;0;0];
propulsion.commandedForce=command;
propulsion.limitedCommand=limited;
propulsion.actualForce=actual;
propulsion.forceDot=forceDot;
propulsion.forceLimited=(limited~=command);
propulsion.forceRateLimited=abs(rawDot)>p.maxForceRate;
propulsion.generalizedForce=[forceBody;cross(p.positionBody(:),forceBody)];
propulsion.vehiclePower=actual*state(7); % F*u only; NOT shaft/electrical input
end
