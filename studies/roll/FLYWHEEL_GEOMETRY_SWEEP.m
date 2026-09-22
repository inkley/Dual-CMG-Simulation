function results=FLYWHEEL_GEOMETRY_SWEEP(maxStep)
% Physically consistent solid-disk geometry study, dual +90deg roll only.
if nargin<1, maxStep=.01; end
root=CMG_ROOT();
b=load(fullfile(root,'Working Results','dual','symmetric_spin','VFR','simulation_result.mat'));
tubeDiameter=5.8*.0254; % User-supplied envelope; usable bore is NOT confirmed.
dimensions=[3.2,.5;4,.5;4.8,.5;4,.25;4,.75]; % inches: diameter, thickness
rows=struct([]); runs=cell(size(dimensions,1),1);
for k=1:size(dimensions,1)
    g=FLYWHEEL_GEOMETRY(dimensions(k,1)*.0254/2,dimensions(k,2)*.0254,7750,tubeDiameter);
    assert(g.rotorOnlyEnvelopeFits);
    x0=b.Y_OUT(1,:).';
    tensors=zeros(3,3,2);
    for j=1:2
        alpha=x0(11+2*j); axis=[sin(alpha);-cos(alpha);0];
        tensors(:,:,j)=g.Itransverse*eye(3)+(g.I-g.Itransverse)*(axis*axis.');
    end
    positions=[b.cmgConfig.installation.dualMountX(:),zeros(2,2)];
    base=b.baseMassProperties;
    [mass,cg,J]=ASSEMBLE_VEHICLE_MASS_PROPERTIES(base.m,base.cg,base.inertia, ...
        [g.m;g.m],positions,tensors);
    assert(norm(J-diag(diag(J)),'fro')<1e-10 && norm(cg)<1e-10);
    auv=b.auv; auv.m=mass; auv.W=mass*auv.g;
    p=b.params; p.m=mass; p.Ix=J(1,1); p.Iy=J(2,2); p.Iz=J(3,3);
    p.xg=cg(1); p.yg=cg(2); p.zg=cg(3);
    c=b.cmgConfig;
    c.gimbal.assemblyInertia=repmat((1+c.gimbal.frameInertiaAllowance)*g.Itransverse ...
        +c.gimbal.motorRotorInertia,1,2);
    c.hybrid.enabled=false; c.thruster.enabled=false;
    c.thruster.commandMode='direct_force'; c.thruster.commandForce=[0;0];
    c.propulsion=AFT_PROPULSION_DEFAULTS(); c.propulsion.enabled=false;
    loop.cycleT=5; loop.fc=.2; loop.controlEndTime=inf;
    opts=odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',maxStep, ...
        'Events',@(t,x) stop(t,x,c.limits.maxGimbalAngle));
    [t,x]=ode45(@(t,x) CONTROL(t,x,b.gains,g,g,auv,p,b.d,loop,c),0:.002:5,x0,opts);
    [q1,q2,h]=TORQUE(t,x,b.gains,g,g,auv,p,b.d,loop,c);
    energy=CMG_MECHANICAL_ACCOUNTING(t,x,q1,q2,h,g,g,c);
    err=rad2deg(b.d.phi-x(:,4)); last=find(abs(err)>1.8,1,'last'); settling=inf;
    if isempty(last), settling=0; elseif last<numel(t), settling=t(last+1); end
    s.caseIndex=k; s.diameterIn=dimensions(k,1); s.thicknessIn=dimensions(k,2);
    s.rotorMassKg=g.m; s.spinInertia=g.I; s.transverseInertia=g.Itransverse;
    s.sweptDiameterMM=g.sweptDiameter*1000; s.radialRemainderMM=g.radialEnvelopeRemainder*1000;
    s.installedMassKg=mass; s.installedIx=p.Ix; s.installedIy=p.Iy; s.installedIz=p.Iz;
    s.initialSpinEnergyJ=sum(energy.spinInitial);
    s.momentumPerRotor=g.I*abs(x0(14));
    s.settlingTime=settling; s.finalErrorDeg=err(end);
    s.peakGimbalDeg=max(abs(rad2deg(x(:,[13,15]))),[],'all');
    s.peakRate=max(abs(h.alphadot),[],'all');
    s.peakAccel=max(abs(h.gimbalAccel),[],'all');
    torque=h.gimbalAccel.*c.gimbal.assemblyInertia;
    s.peakGimbalInertialTorque=max(abs(torque),[],'all');
    s.peakGimbalInertialPower=max(abs(torque.*h.alphadot),[],'all');
    s.flywheelTorqueAtAccelLimit=g.I*c.limits.maxFlywheelAccel;
    s.limitSamples=nnz(h.gimbalRateSaturated | h.gimbalAccelSaturated ...
        | h.gimbalAngleLimited | h.flywheelSpeedLimited | h.flywheelAccelSaturated);
    s.rollPass=t(end)>=5 && settling<=4 && abs(err(end))<=1.8;
    s.screenPass=s.rollPass && s.limitSamples==0;
    if isempty(rows), rows=s; else, rows(end+1)=s; end %#ok<AGROW>
    runs{k}=struct('t',t,'x',x,'gyro',g,'config',c,'auv',auv,'params',p,'control',h,'energy',energy);
end
results=struct2table(rows); out=fullfile(root,'Working Results','flywheel_geometry');
if ~isfolder(out), mkdir(out); end
save(fullfile(out,['sweep_',num2str(maxStep),'.mat']),'results','runs','b','tubeDiameter');
writetable(results,fullfile(out,['summary_',num2str(maxStep),'.csv'])); disp(results);
end
function [value,terminal,direction]=stop(~,x,bound)
value=bound-max(abs(x([13,15]))); terminal=1; direction=-1;
end
