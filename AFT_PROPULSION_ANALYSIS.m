function result = AFT_PROPULSION_ANALYSIS()
% Isolated surge verification; does NOT implement mission sequencing.
root=fileparts(mfilename('fullpath'));
b=load(fullfile(root,'Working Results','dual','symmetric_spin','VFR','simulation_result.mat'));
out=fullfile(root,'Working Results','aft_propulsion_validation');
if ~isfolder(out), mkdir(out); end
cfg=b.cmgConfig; cfg.propulsion=AFT_PROPULSION_DEFAULTS();
cfg.propulsion.enabled=true; cfg.propulsion.commandForce=5;
cfg.thruster.enabled=false; cfg.hybrid.enabled=false; cfg.momentumManagement.enabled=false;
cfg.external.rollDisturbance=0;
d=b.d; if isfield(d,'rollToPlane'), d=rmfield(d,'rollToPlane'); end
if isfield(d,'rollScheduleTime'), d=rmfield(d,{'rollScheduleTime','rollScheduleAngle'}); end
d.phi=0;
loop.cycleT=60; loop.fc=1/60; loop.controlEndTime=inf;
x0=b.Y_OUT(1,:).'; x0(1:12)=0; x0(17:21)=0;
tspan=(0:.01:60).';
options=odeset('RelTol',1e-9,'AbsTol',1e-11,'MaxStep',.01);
[t,x]=ode45(@rhs,tspan,x0,options);
% Independent reduced surge ODE uses the same documented hydrodynamic
% coefficients as REMUS, but not its mass matrix or CONTROL implementation.
[~,reference]=ode45(@reduced,tspan,[0;0],options);
result.maxSurgeReferenceError=max(abs(x(:,7)-reference(:,1)));
result.maxForceReferenceError=max(abs(x(:,21)-reference(:,2)));
assert(result.maxSurgeReferenceError<1e-6 && result.maxForceReferenceError<1e-6);
result.equilibriumSpeed=sqrt(5/1.62);
[~,at40]=min(abs(t-40)); result.speedAtShutdown=x(at40,7);
result.finalCoastingSpeed=x(end,7);
result.distance=x(end,1); result.maxCrossAxisState=max(abs(x(:,[2:6,8:12])),[],'all');
assert(result.maxCrossAxisState<1e-10);
% Orientation check: identical body surge after a 90-degree inertial yaw.
xYaw=x0; xYaw(6)=pi/2;
[~,yawState]=ode45(@rhs,tspan,xYaw,options);
result.yawRotatedSurgeError=max(abs(yawState(:,7)-x(:,7)));
result.yawRotatedPositionError=max(abs(yawState(:,2)-x(:,1)));
assert(result.yawRotatedSurgeError<1e-6 && result.yawRotatedPositionError<1e-6);
% Backward compatibility: absent/off propulsion has exactly the old RHS,
% and appending a zero force state does not alter the first 20 derivatives.
old=b.cmgConfig; if isfield(old,'propulsion'), old=rmfield(old,'propulsion'); end
off=old; off.propulsion=AFT_PROPULSION_DEFAULTS();
maxDifference=0;
for k=round(linspace(1,size(b.Y_OUT,1),20))
    state=b.Y_OUT(k,1:20).';
    dxOld=CONTROL(b.T_OUT(k),state,b.gains,b.gyro1,b.gyro2,b.auv,b.params,b.d,loop,old);
    dxOff=CONTROL(b.T_OUT(k),[state;0],b.gains,b.gyro1,b.gyro2,b.auv,b.params,b.d,loop,off);
    maxDifference=max(maxDifference,max(abs(dxOld-dxOff(1:20))));
    assert(dxOff(21)==0);
end
result.disabledDerivativeDifference=maxDifference; assert(maxDifference==0);
baselineOptions=odeset('RelTol',b.simConfig.relTol,'AbsTol',b.simConfig.absTol, ...
    'MaxStep',b.simConfig.maxStep);
[~,baselineReplay]=ode45(@CONTROL,b.T_OUT,b.Y_OUT(1,1:20).',baselineOptions, ...
    b.gains,b.gyro1,b.gyro2,b.auv,b.params,b.d,loop,old);
result.disabledBaselineReplayError=max(abs(baselineReplay-b.Y_OUT(:,1:20)),[],'all');
assert(result.disabledBaselineReplayError<1e-4,'Disabled propulsion changed the saved baseline.');
% Exercise the public history reconstruction with the extra state present.
current=cfg; current.propulsion.commandForce=5;
[~,~,replayed]=TORQUE(t(1:10),x(1:10,:),b.gains,b.gyro1,b.gyro2, ...
    b.auv,b.params,d,loop,current);
assert(max(abs(replayed.propulsionForce-x(1:10,21)))<1e-12);
command=5*double(t<40);
result.vehicleWork=trapz(t,x(:,21).*x(:,7));
result.note='F*u is vehicle mechanical transfer, not propeller input or electrical energy.';
save(fullfile(out,'validation.mat'),'result','t','x','reference','yawState','cfg','b','command');
disp(result);
fig=figure('Visible','off','Position',[100 100 1100 800]); tiledlayout(2,2);
nexttile; plot(t,[command,x(:,21)],'LineWidth',1.5); grid on;
xlabel('Time (s)'); ylabel('Thrust (N)'); legend('Requested','Actual');
nexttile; plot(t,[x(:,7),reference(:,1)],'LineWidth',1.5); hold on;
yline(result.equilibriumSpeed,'k:'); grid on; xlabel('Time (s)'); ylabel('Surge speed (m/s)');
legend('Full vehicle','Reduced reference','5 N equilibrium','Location','best');
nexttile; plot(t,x(:,1),'LineWidth',1.5); grid on; xlabel('Time (s)'); ylabel('Forward distance (m)');
nexttile; plot(t,x(:,21).*x(:,7),'LineWidth',1.5); grid on;
xlabel('Time (s)'); ylabel('Vehicle mechanical power F u (W)');
exportgraphics(fig,fullfile(out,'AFT_PROPULSION_RESPONSE.png'),'Resolution',200); close(fig);
    function [dx,c]=rhs(t,x)
        current=cfg; current.propulsion.commandForce=5*double(t<40);
        [dx,c]=CONTROL(t,x,b.gains,b.gyro1,b.gyro2,b.auv,b.params,d,loop,current);
    end
    function dy=reduced(t,y)
        Ftarget=5*double(t<40);
        Fdot=min(max((Ftarget-y(2))/.5,-10),10);
        dy=[(y(2)-1.62*y(1)*abs(y(1)))/(b.auv.m+.93);Fdot];
    end
end
