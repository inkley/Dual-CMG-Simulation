function results = CMG_ESTIMATION_UNCERTAINTY_SWEEP(maxStep)
% Deterministic design screening, not sensor tolerances or probability bounds.
% True spin-axis inertia changes at fixed installed vehicle inertia/mass:
% isolates rotor momentum-model uncertainty, NOT a resized hardware design.
if nargin<1, maxStep=.01; end
root=fileparts(mfilename('fullpath'));
out=fullfile(root,'Working Results','estimation_uncertainty');
if ~isfolder(out), mkdir(out); end
rows=struct([]); histories={};
current=load(fullfile(root,'Working Results','dual','symmetric_spin','VFR','simulation_result.mat'));
for mode=["single","dual"]
    if mode=="single", folder=fullfile('single','VFR');
    else, folder=fullfile('dual','symmetric_spin','VFR'); end
    b=load(fullfile(root,'Working Results',folder,'simulation_result.mat'));
    patterns=[1,1];
    if mode=="dual", patterns=[1,1;1,-1;0,1]; end
    for pattern=1:size(patterns,1)
        for bias=[-.05,0,.05]
            for delta=[-.10,0,.10]
                c=b.cmgConfig;
                % Older single baseline predates optional VRT/hybrid fields.
                c.thruster=current.cmgConfig.thruster; c.thruster.enabled=false;
                c.thruster.commandMode='direct_force'; c.thruster.commandForce=[0;0];
                c.hybrid=current.cmgConfig.hybrid; c.hybrid.enabled=false;
                c.propulsion=AFT_PROPULSION_DEFAULTS(); c.propulsion.enabled=false;
                c.estimation.speedScaleBias=bias*patterns(pattern,:);
                c.estimation.rotorInertia=[b.gyro1.I,b.gyro2.I];
                g1=b.gyro1; g2=b.gyro2;
                g1.I=g1.I*(1+delta*patterns(pattern,1));
                g2.I=g2.I*(1+delta*patterns(pattern,2));
                loop.cycleT=5; loop.fc=.2; loop.controlEndTime=inf;
                options=odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',maxStep, ...
                    'Events',@(t,x) stopEvent(t,x,c.limits));
                [t,x]=ode45(@(t,x) CONTROL(t,x,b.gains,g1,g2,b.auv, ...
                    b.params,b.d,loop,c),0:.01:5,b.Y_OUT(1,:).',options);
                [~,~,h]=TORQUE(t,x,b.gains,g1,g2,b.auv,b.params,b.d,loop,c);
                e=rad2deg(b.d.phi-x(:,4));
                outside=find(abs(e)>.02*abs(rad2deg(b.d.phi)),1,'last');
                settling=inf;
                if isempty(outside), settling=0;
                elseif outside<numel(t), settling=t(outside+1); end
                s.mode=mode; s.pattern=pattern; s.speedBiasPercent=100*bias;
                s.trueInertiaErrorPercent=100*delta;
                s.finalTime=t(end); s.settlingTime=settling;
                s.finalRollErrorDeg=e(end);
                s.peakPitchDeg=max(abs(rad2deg(x(:,5))));
                s.peakYawDeg=max(abs(rad2deg(x(:,6))));
                s.rollRMSE=sqrt(trapz(t,h.momentError(:,1).^2)/(t(end)-t(1)));
                s.peakGimbalDeg=max(abs(rad2deg(x(:,[13,15]))),[],'all');
                s.peakSpeedRPM=max(abs(x(:,[14,16])),[],'all')*60/(2*pi);
                s.limitSamples=nnz(h.gimbalRateSaturated | h.gimbalAccelSaturated ...
                    | h.flywheelAccelSaturated | h.gimbalAngleLimited | h.flywheelSpeedLimited);
                s.rollPass=t(end)>=5 && abs(e(end))<=1.8 && settling<=4;
                % Coupling is reported separately; rollPass is NOT a mission pass.
                if isempty(rows), rows=s; else, rows(end+1)=s; end
                histories{end+1}=struct('t',t,'x',x,'config',c, ...
                    'gyro1',g1,'gyro2',g2,'control',h); %#ok<AGROW>
                fprintf('%s pattern%d bias%+g%% inertia%+g%%: error %.3f deg, pitch/yaw %.3f/%.3f, limits%d\n', ...
                    mode,pattern,100*bias,100*delta,e(end),s.peakPitchDeg,s.peakYawDeg,s.limitSamples);
            end
        end
    end
end
results=struct2table(rows);
tag=num2str(maxStep);
writetable(results,fullfile(out,['summary_dt_',tag,'.csv']));
save(fullfile(out,['sweep_dt_',tag,'.mat']),'results','histories','maxStep');
end

function [value,terminal,direction]=stopEvent(~,x,limits)
value=[limits.maxGimbalAngle-max(abs(x([13,15]))); ...
    limits.maxFlywheelSpeed-max(abs(x([14,16])))];
terminal=[1;1]; direction=[-1;-1];
end
