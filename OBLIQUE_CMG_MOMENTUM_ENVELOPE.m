function results = OBLIQUE_CMG_MOMENTUM_ENVELOPE(reuseSaved)
%OBLIQUE_CMG_MOMENTUM_ENVELOPE Characterize, without retuning the main model.
% Reruns zero-translation oblique turns from the saved symmetric dual VFR
% baseline. Diagnostic variants are explicitly labeled, not new baselines.
% H is rotor spin momentum, not total vehicle-plus-fluid angular momentum.
% The instantaneous K interval imposes M=0, includes body-yaw-rate bias and
% angle/rate bounds, but NOT acceleration/servo dynamics. Those are reported
% separately from replayed actual actuator histories.

if nargin<1, reuseSaved=false; end
verifyInterval();
scriptDir = fileparts(mfilename('fullpath'));
baselinePath = fullfile(scriptDir,'Working Results','dual', ...
    'symmetric_spin','VFR','simulation_result.mat');
b = load(baselinePath);
assert(strcmp(b.cmgConfig.mode,'dual') && ...
    strcmp(b.cmgConfig.dualController,'constant_speed'), ...
    'Requires a constant-speed dual baseline.');
out = fullfile(scriptDir,'Working Results','oblique_momentum_envelope');
if ~isfolder(out), mkdir(out); end
% plane, heading, duration, diagnostic gate variant, max integration step
cases = [45,30,18,0,.01;45,35,18,0,.01;45,40,18,0,.01; ...
    45,45,18,0,.01;45,60,18,0,.01;-45,-45,18,0,.01; ...
    45,45,60,0,.01;45,45,18,1,.01;45,45,18,0,.005];
labels = ["30 deg";"35 deg";"40 deg";"45 deg";"60 deg"; ...
    "negative mirror";"60 second hold";"rate gate relaxed";"half step"];
if reuseSaved
    saved=load(fullfile(out,'characterization.mat'));
    histories=saved.histories; results=saved.results; metadata=saved.metadata;
    b=saved.b;
else
histories = cell(size(cases,1),1);
rows = cell(size(cases,1),1);
for j=1:size(cases,1)
    x0=b.Y_OUT(1,:).'; x0(1:12)=0; x0(19:20)=0;
    d=b.d; cfg=b.cmgConfig;
    lateral=[0;cosd(cases(j,1));sind(cases(j,1))];
    desiredHeading=[cosd(cases(j,2));sind(cases(j,2))*lateral(2:3)];
    d.rollToPlane.desiredLateralDirectionNED=lateral;
    d.rollToPlane.bidirectionalThruster=true;
    d.hybrid.initialPositionNED=zeros(3,1);
    d.hybrid.lateralDirectionNED=lateral;
    d.hybrid.lateralDisplacement=0;
    d.hybrid.desiredHeadingNED=desiredHeading;
    cfg.hybrid.enabled=true; cfg.thruster.enabled=true;
    cfg.thruster.commandMode='generalized_force';
    cfg.momentumManagement.enabled=false;
    cfg.hybrid.rollEnableAngle=deg2rad(.5);
    cfg.hybrid.rollDisableAngle=deg2rad(1.5);
    cfg.hybrid.rollEnableRate=deg2rad(.5);
    cfg.hybrid.rollDisableRate=deg2rad(3);
    cfg.hybrid.KpLateral=20; cfg.hybrid.KdLateral=30;
    cfg.hybrid.KpHeading=2; cfg.hybrid.KdHeading=6;
    if cases(j,4)==1
        % Diagnostic only: remove absolute Euler roll-rate gating over the
        % encountered rates, but retain the normal roll-error gate.
        cfg.hybrid.rollEnableRate=1e3; cfg.hybrid.rollDisableRate=2e3;
    end
    loop.cycleT=cases(j,3); loop.fc=1/loop.cycleT; loop.controlEndTime=inf;
    options=odeset('RelTol',b.simConfig.relTol,'AbsTol',b.simConfig.absTol, ...
        'MaxStep',cases(j,5));
    [t,x]=ode45(@CONTROL,0:cases(j,5):cases(j,3),x0,options, ...
        b.gains,b.gyro1,b.gyro2,b.auv,b.params,d,loop,cfg);
    [~,~,c]=TORQUE(t,x,b.gains,b.gyro1,b.gyro2,b.auv,b.params,d,loop,cfg);
    h=[b.gyro1.I*x(:,14),b.gyro2.I*x(:,16)]; a=x(:,[13,15]);
    H=[sum(h.*sin(a),2),-sum(h.*cos(a),2),zeros(size(t))];
    bound=sum(abs(h),2);
    Hdot=[sum(h.*cos(a).*c.alphadot,2), ...
        sum(h.*sin(a).*c.alphadot,2),zeros(size(t))];
    transport=cross(x(:,10:12),H,2);
    identity=c.achievedMoment+Hdot+transport;
    % Body-frame integral must include transport; integral(K) alone is not
    % -Delta Hx during a coupled turn.
    reconstructed=H(1,:)+cumtrapz(t,-c.achievedMoment-transport);
    Kbounds=nan(numel(t),2); fullHeading=zeros(size(t));
    fixedPlaneError=zeros(size(t)); projectionNorm=zeros(size(t));
    normal=cross([1;0;0],lateral);
    for k=1:numel(t)
        B=-[h(k,:).*cos(a(k,:));h(k,:).*sin(a(k,:))];
        Kbounds(k,:)=rollInterval(B,x(k,12),a(k,:),cfg.limits);
        R=rotation(x(k,4:6)); nose=R(:,1);
        fullHeading(k)=atan2(norm(cross(nose,desiredHeading)), ...
            dot(nose,desiredHeading));
        fixedPlaneError(k)=acos(min(1,abs(dot(R(:,3),normal))));
        projectionNorm(k)=norm(cross(nose,lateral));
    end
    outside=c.requestedMoment(:,1)<Kbounds(:,1)-1e-6 | ...
        c.requestedMoment(:,1)>Kbounds(:,2)+1e-6 | isnan(Kbounds(:,1));
    firstActive=find(c.hybridActivation>=.95,1);
    captureLoss=nan;
    if ~isempty(firstActive)
        ix=find((1:numel(t)).'>firstActive & c.hybridActivation<.5,1);
        if ~isempty(ix), captureLoss=t(ix); end
    end
    row.label=labels(j); row.planeDeg=cases(j,1); row.headingDeg=cases(j,2);
    row.duration=cases(j,3);
    row.finalHeadingMetricDeg=rad2deg(c.hybridHeadingError(end));
    row.finalFullHeadingDeg=rad2deg(fullHeading(end));
    row.finalFixedPlaneErrorDeg=rad2deg(fixedPlaneError(end));
    row.finalGate=c.hybridActivation(end); row.captureLossTime=captureLoss;
    row.maxMomentumNormFraction=max(vecnorm(H,2,2)./bound);
    row.maxAbsHxFraction=max(abs(H(:,1))./bound);
    row.minHxHeadroom=min(bound-abs(H(:,1)));
    row.maxGimbalDeg=rad2deg(max(abs(a),[],'all'));
    row.minSingularDistanceDeg=rad2deg(min(asin(abs(sin(a(:,2)-a(:,1))))));
    row.peakGimbalRate=max(abs(c.alphadot),[],'all');
    row.peakGimbalAccel=max(abs(c.gimbalAccel),[],'all');
    row.rateInfeasibleSeconds=trapz(t,double(outside));
    row.peakRollResidual=max(abs(c.momentError(:,1)));
    row.rollResidualRMSE=sqrt(trapz(t,c.momentError(:,1).^2)/t(end));
    row.angleLimitSamples=nnz(c.gimbalAngleLimited);
    row.rateLimitSamples=nnz(c.gimbalRateSaturated);
    row.accelLimitSamples=nnz(c.gimbalAccelSaturated);
    row.minEulerCosPitch=min(abs(cos(x(:,5))));
    row.minPlaneProjection= min(projectionNorm);
    row.reactionIdentityError=max(abs(identity),[],'all');
    row.bodyIntegralError=max(abs(H-reconstructed),[],'all');
    rows{j}=row;
    histories{j}=struct('time',t,'state',x,'control',c,'H',H, ...
        'Hbound',bound,'Kbounds',Kbounds,'fullHeading',fullHeading, ...
        'fixedPlaneError',fixedPlaneError,'transport',transport, ...
        'desired',d,'config',cfg);
    fprintf('%s: heading %.3f deg, |H|/bound %.4f, |Hx|/bound %.4f, gate %.3f\n', ...
        labels(j),row.finalFullHeadingDeg,row.maxMomentumNormFraction, ...
        row.maxAbsHxFraction,row.finalGate);
end
results=struct2table(vertcat(rows{:})); disp(results);
assert(all(results.reactionIdentityError<1e-10),'Reaction identity failed.');
assert(all(results.maxMomentumNormFraction<=1+1e-10),'Momentum outer bound failed.');
metadata.baselinePath=baselinePath; metadata.baselineConfig=b.cmgConfig;
metadata.baselineInitialState=b.Y_OUT(1,:); metadata.MATLAB=version;
metadata.created=char(datetime('now')); metadata.cases=cases;
[~,metadata.gitCommit]=system('git rev-parse HEAD');
metadata.note='Working-tree analysis additions may postdate this commit. MAT stores configurations and histories.';
end
for j=1:height(results)
    z=histories{j}; t=z.time; c=z.control; x=z.state;
    afterRoll=t>=3.5;
    results.postRollMomentRMSE(j)=sqrt(trapz(t(afterRoll), ...
        c.momentError(afterRoll,1).^2)/(t(end)-t(find(afterRoll,1))));
    results.postRollMinSingularDistanceDeg(j)=rad2deg(min(asin(abs( ...
        sin(x(afterRoll,15)-x(afterRoll,13))))));
    results.finalUngatedYawRequest(j)=z.config.hybrid.KpHeading*c.hybridHeadingError(end) ...
        -z.config.hybrid.KdHeading*x(end,12);
    results.finalBodyYawRate(j)=x(end,12);
    results.firstAngleBoundExceeded(j)=firstTime(t, ...
        any(abs(x(:,[13,15]))>z.config.limits.maxGimbalAngle+1e-6,2));
    results.firstRateInfeasible(j)=firstTime(t, ...
        c.requestedMoment(:,1)<z.Kbounds(:,1)-1e-6 | ...
        c.requestedMoment(:,1)>z.Kbounds(:,2)+1e-6 | isnan(z.Kbounds(:,1)));
end
assert(abs(results.finalFullHeadingDeg(4)-results.finalFullHeadingDeg(9))<1e-3, ...
    'Heading outcome is sensitive to the half-step check.');
assert(abs(results.maxGimbalDeg(4)-results.maxGimbalDeg(9))<1e-3, ...
    'Gimbal excursion is sensitive to the half-step check.');
save(fullfile(out,'characterization.mat'),'results','histories','metadata','b');
writetable(results,fullfile(out,'summary.csv'));

fig=figure('Visible','off','Position',[100 100 1300 900]);
tiledlayout(2,2,'TileSpacing','compact');
nexttile; hold on;
% Sample actual angle-restricted reachable set, not its convex hull.
angles=linspace(-b.cmgConfig.limits.maxGimbalAngle, ...
    b.cmgConfig.limits.maxGimbalAngle,301); [a1,a2]=ndgrid(angles);
hs=[b.gyro1.I*b.Y_OUT(1,14),b.gyro2.I*b.Y_OUT(1,16)];
scatter(hs(1)*sin(a1(:))+hs(2)*sin(a2(:)), ...
    -hs(1)*cos(a1(:))-hs(2)*cos(a2(:)),1,[.9 .9 .9],'.', ...
    'DisplayName','Angle-bounded reachable samples');
colors=lines(3); selected=[1,4,6];
for q=1:3
    j=selected(q);
    z=histories{j}; plot(z.H(:,1),z.H(:,2),'LineWidth',1.6, ...
        'Color',colors(q,:),'DisplayName',labels(j));
end
axis equal; grid on; xlabel('Rotor H_x (N m s)'); ylabel('Rotor H_y (N m s)');
title('Rotor momentum paths'); legend('Location','best');
nexttile; hold on;
for q=1:3
    j=selected(q);
    z=histories{j}; plot(z.time,vecnorm(z.H,2,2)./z.Hbound, ...
        'LineWidth',1.6,'Color',colors(q,:),'DisplayName',labels(j));
end
yline(1,'--','DisplayName','Outer magnitude bound'); ylim([0 1.05]); grid on;
xlabel('Time (s)'); ylabel('|H| / sum |I Omega|'); title('Momentum magnitude usage');
legend('Location','best');
nexttile; z=histories{4};
plot(z.time,rad2deg(z.state(:,[13,15])),'LineWidth',1.5); hold on;
yline(100,'--'); yline(-100,'--'); grid on; xlabel('Time (s)'); ylabel('Gimbal angle (deg)');
title('45 degree heading case'); legend('CMG 1','CMG 2','Location','best');
nexttile; yyaxis left;
plot(z.time,rad2deg(z.fullHeading),'LineWidth',1.5); ylabel('Full heading error (deg)');
yyaxis right; plot(z.time,z.control.hybridActivation,'LineWidth',1.5);
ylabel('Thrust-enable multiplier'); xlabel('Time (s)'); grid on; title('Heading and thrust gate');
exportgraphics(fig,fullfile(out,'MOMENTUM_ENVELOPE.png'),'Resolution',200); close(fig);
fig=figure('Visible','off','Position',[100 100 1100 800]); tiledlayout(3,1);
nexttile; plot(z.time,[z.control.requestedMoment(:,1),z.control.achievedMoment(:,1)],'LineWidth',1.3);
grid on; ylabel('Roll moment (N m)'); legend('Requested','Achieved');
nexttile; plot(z.time,z.Kbounds,'LineWidth',1.3); hold on;
plot(z.time,z.control.requestedMoment(:,1),'k--'); grid on; ylabel('Roll moment (N m)');
legend('Static lower bound','Static upper bound','Requested');
nexttile; plot(z.time,[z.H(:,1)-z.H(1,1), ...
    -cumtrapz(z.time,z.control.achievedMoment(:,1)), ...
    -cumtrapz(z.time,z.control.achievedMoment(:,1)+z.transport(:,1))],'LineWidth',1.3);
grid on; ylabel('Momentum change (N m s)'); xlabel('Time (s)');
legend('Actual Delta Hx','Torque integral only','Including rotating-frame transport');
exportgraphics(fig,fullfile(out,'ROLL_AUTHORITY.png'),'Resolution',200); close(fig);
end

function time=firstTime(t,mask)
ix=find(mask,1); time=nan; if ~isempty(ix), time=t(ix); end
end

function verifyInterval()
limits.maxGimbalRate=2; limits.maxGimbalAngle=1;
assert(norm(rollInterval(eye(2),.3,[0 0],limits)-[-1.7 2.3])<1e-12);
assert(norm(rollInterval([1 1;0 0],0,[0 0],limits)-[-4 4])<1e-12);
assert(all(isnan(rollInterval(eye(2),3,[0 0],limits))));
assert(norm(rollInterval(eye(2),0,[1 0],limits)-[-2 0])<1e-12);
fprintf('Four analytic rate-envelope regression checks passed.\n');
end

function interval=rollInterval(B,r,angles,limits)
% Intersect the rate box with M=0; extrema of K occur on its edges.
lo=-limits.maxGimbalRate*ones(2,1); hi=-lo;
for j=1:2
    if angles(j)>=limits.maxGimbalAngle, hi(j)=0; end
    if angles(j)<=-limits.maxGimbalAngle, lo(j)=0; end
end
bias=B*[r;r]; target=-bias(2); vertices=[];
for j=1:2
    other=3-j;
    for edge=[lo(j),hi(j)]
        if abs(B(2,other))>1e-14
            v=zeros(2,1); v(j)=edge;
            v(other)=(target-B(2,j)*edge)/B(2,other);
            if all(v>=lo-1e-9 & v<=hi+1e-9), vertices(:,end+1)=v; end %#ok<AGROW>
        else
            for otherEdge=[lo(other),hi(other)]
                v=zeros(2,1); v(j)=edge; v(other)=otherEdge;
                if abs(B(2,:)*v-target)<1e-12, vertices(:,end+1)=v; end %#ok<AGROW>
            end
        end
    end
end
if isempty(vertices), interval=[nan,nan]; else
    K=B(1,:)*vertices+bias(1); interval=[min(K),max(K)];
end
end

function R=rotation(e)
p=e(1); t=e(2); s=e(3);
R=[cos(s)*cos(t),cos(s)*sin(t)*sin(p)-sin(s)*cos(p),cos(s)*sin(t)*cos(p)+sin(s)*sin(p); ...
sin(s)*cos(t),sin(s)*sin(t)*sin(p)+cos(s)*cos(p),sin(s)*sin(t)*cos(p)-cos(s)*sin(p); ...
-sin(t),cos(t)*sin(p),cos(t)*cos(p)];
end
