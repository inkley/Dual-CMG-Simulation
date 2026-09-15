function VERIFY_AFT_PROPULSION()
% Analytic and boundary tests of the forward-only commanded-thrust model.
c.propulsion=AFT_PROPULSION_DEFAULTS(); c.propulsion.enabled=true;
c.propulsion.commandForce=5; c.propulsion.maxForceRate=100;
t=(0:.01:5).'; options=odeset('RelTol',1e-10,'AbsTol',1e-12,'MaxStep',.01);
[~,F]=ode45(@(t,F) derivative(F,c),t,0,options);
assert(max(abs(F-5*(1-exp(-t/.5))))<1e-7);
c.propulsion.commandForce=20; c.propulsion.maxForceRate=10;
[~,F]=ode45(@(t,F) derivative(F,c),t,0,options);
expected=10*t; late=t>.5; expected(late)=10-5*exp(-(t(late)-.5)/.5);
assert(max(abs(F-expected))<1e-6);
assert(min(F)>=-1e-9 && max(F)<=10+1e-9);
x=zeros(21,1); a=AFT_PROPULSION(x,c);
assert(a.forceLimited && a.forceRateLimited && a.actualForce==0 && a.forceDot==10);
c.propulsion.commandForce=-3; a=AFT_PROPULSION(x,c);
assert(a.limitedCommand==0 && a.forceDot==0 && a.forceLimited);
c.propulsion.enabled=false;
[~,F]=ode45(@(t,F) derivative(F,c),t,4,options);
assert(max(abs(F-4*exp(-t/.5)))<1e-7);
x(21)=4; x(7)=2; a=AFT_PROPULSION(x,c);
assert(norm(a.generalizedForce-[4;0;0;0;0;0])<1e-12 && a.vehiclePower==8);
c.propulsion.positionBody=[-.8;.1;.2]; a=AFT_PROPULSION(x,c);
assert(norm(a.generalizedForce-[4;0;0;0;.8;-.4])<1e-12);
a=AFT_PROPULSION(zeros(20,1),struct());
assert(all(a.generalizedForce==0) && a.forceDot==0);
c.propulsion.enabled=true;
assertError(@() AFT_PROPULSION(zeros(20,1),c),'CMG:MissingPropulsionState');
x(21)=11; assertError(@() AFT_PROPULSION(x,c),'CMG:PropulsionForceOutsideEnvelope');
fprintf('Aft propulsion: lag, slew, clipping, shutdown, mounting, power and compatibility checks passed.\n');
end
function value=derivative(F,c)
x=zeros(21,1); x(21)=F; a=AFT_PROPULSION(x,c); value=a.forceDot;
end
function assertError(f,id)
try
    f();
catch exception
    assert(strcmp(exception.identifier,id)); return;
end
error('Expected error %s was not raised.',id);
end
