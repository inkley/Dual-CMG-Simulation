function results=MATCHED_ENERGY_COMPARISON(maxStep)
% Two common-speed, equal-total-rotor-mass/inertia/initial-energy pairs.
% Single uses variable spin; dual uses constant spin. No gain retuning.
if nargin<1, maxStep=.01; end
root=CMG_ROOT();
b=load(fullfile(root,'Working Results','dual','symmetric_spin','VFR','simulation_result.mat'));
rows=struct([]); runs=cell(4,1);
for k=1:4
    pair=ceil(k/2); n=1+mod(k+1,2);
    thickness=[.5,1]; thickness=thickness(pair)/n;
    g=FLYWHEEL_GEOMETRY(.0508,thickness*.0254,7750,5.8*.0254);
    assert(g.rotorOnlyEnvelopeFits);
    x0=zeros(20,1); x0(14)=1200*2*pi/60;
    c=b.cmgConfig;
    if n==1
        c.mode='single'; mounts=0;
    else
        c.mode='dual'; c.dualController='constant_speed';
        x0(13)=deg2rad(-15); x0(15)=deg2rad(15);
        x0(14)=-x0(14); x0(16)=-x0(14);
        mounts=c.installation.dualMountX(:);
    end
    tensors=zeros(3,3,n);
    for j=1:n
        alpha=x0(11+2*j); axis=[sin(alpha);-cos(alpha);0];
        tensors(:,:,j)=g.Itransverse*eye(3)+(g.I-g.Itransverse)*(axis*axis.');
    end
    base=b.baseMassProperties;
    [mass,cg,J]=ASSEMBLE_VEHICLE_MASS_PROPERTIES(base.m,base.cg,base.inertia, ...
        repmat(g.m,n,1),[mounts,zeros(n,2)],tensors);
    assert(norm(J-diag(diag(J)),'fro')<1e-10 && norm(cg)<1e-10);
    auv=b.auv; auv.m=mass; auv.W=mass*auv.g;
    p=b.params; p.m=mass; p.Ix=J(1,1); p.Iy=J(2,2); p.Iz=J(3,3);
    p.xg=0; p.yg=0; p.zg=0;
    c.gimbal.assemblyInertia=repmat((1+c.gimbal.frameInertiaAllowance)*g.Itransverse ...
        +c.gimbal.motorRotorInertia,1,2);
    c.hybrid.enabled=false; c.thruster.enabled=false;
    c.thruster.commandMode='direct_force'; c.thruster.commandForce=[0;0];
    c.propulsion=AFT_PROPULSION_DEFAULTS(); c.propulsion.enabled=false;
    c.momentumManagement.enabled=false; c.external.rollDisturbance=0;
    loop.cycleT=5; loop.fc=.2; loop.controlEndTime=inf;
    d=b.d; d.phi=pi/2;
    opts=odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',maxStep, ...
        'Events',@(t,x) stops(t,x,c.limits));
    [t,x]=ode45(@(t,x) CONTROL(t,x,b.gains,g,g,auv,p,d,loop,c),0:.001:5,x0,opts);
    [q1,q2,h]=TORQUE(t,x,b.gains,g,g,auv,p,d,loop,c);
    e=CMG_MECHANICAL_ACCOUNTING(t,x,q1,q2,h,g,g,c);
    err=rad2deg(d.phi-x(:,4)); last=find(abs(err)>1.8,1,'last'); settling=inf;
    if isempty(last), settling=0; elseif last<numel(t), settling=t(last+1); end
    s.pair=pair; s.mode=string(c.mode); s.rotorThicknessIn=thickness;
    s.totalRotorMass=n*g.m; s.totalSpinInertia=n*g.I; s.initialSpinJ=sum(e.spinInitial);
    s.installedMass=mass; s.Ix=p.Ix; s.Iy=p.Iy; s.Iz=p.Iz;
    s.duration=t(end); s.settlingTime=settling; s.finalErrorDeg=err(end);
    s.pitchDeg=max(abs(rad2deg(x(:,5)))); s.yawDeg=max(abs(rad2deg(x(:,6))));
    s.gimbalDeg=max(abs(rad2deg(x(:,[13,15]))),[],'all');
    s.gimbalRate=max(abs(h.alphadot),[],'all');
    s.spinRPM=max(abs(x(:,[14,16])),[],'all')*60/(2*pi);
    s.rollGrossJ=e.roll.gross; s.finalSpinJ=sum(e.spinFinal);
    s.spinPositiveJ=sum(e.spinAxialWork.positive); s.spinNegativeJ=sum(e.spinAxialWork.negativeMagnitude);
    s.gimbalInertialPositiveJ=sum(e.gimbalInertialWork.positive);
    s.limitSamples=nnz(h.gimbalRateSaturated | h.gimbalAccelSaturated | ...
        h.flywheelAccelSaturated | h.gimbalAngleLimited | h.flywheelSpeedLimited);
    s.passes=t(end)>=5 && settling<=4 && abs(err(end))<=1.8 && s.limitSamples==0;
    if isempty(rows), rows=s; else, rows(end+1)=s; end %#ok<AGROW>
    runs{k}=struct('t',t,'x',x,'energy',e,'control',h,'config',c,'gyro',g,'auv',auv,'params',p);
end
results=struct2table(rows); out=fullfile(root,'Working Results','matched_energy');
if ~isfolder(out), mkdir(out); end
save(fullfile(out,['comparison_',num2str(maxStep),'.mat']),'results','runs','b');
writetable(results,fullfile(out,['comparison_',num2str(maxStep),'.csv'])); disp(results);
end
function [value,terminal,direction]=stops(~,x,limits)
value=[limits.maxGimbalAngle-max(abs(x([13,15])));limits.maxFlywheelSpeed-max(abs(x([14,16])))];
terminal=[1;1]; direction=[-1;-1];
end
