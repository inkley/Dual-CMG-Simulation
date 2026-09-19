function results=INVESTIGATE_ROLL_FEASIBILITY
root=fileparts(mfilename('fullpath')); cases=MISSION_ESTIMATION_CASES();
folder=['estimation_',char(cases(10).name)];
rows=struct([]); traces=cell(3,1);
for grid=1:3
    dt=[.05,.025,.025];
    a=load(fullfile(root,'Working Results','roll_turn_surge', ...
        sprintf('plane_45_heading_45_dt_%g_timeout_15',dt(grid)),folder,'mission.mat'));
    [~,cmd]=ROLL_TURN_SURGE_SUPERVISOR(0,a.history.state(1,:).',[],a.m);
    reference=a.b.d; reference.rollToPlane.desiredLateralDirectionNED=cmd.lateral;
    reference.hybrid.desiredHeadingNED=cmd.heading;
    reference.hybrid.lateralDirectionNED=cmd.lateral;
    reference.hybrid.lateralDisplacement=0;
    reference.hybrid.initialPositionNED=zeros(3,1);
    loop.cycleT=100; loop.fc=.01; loop.controlEndTime=inf;
    % Dense replay of saved state interpolation isolates output sampling;
    % this is not an independently integrated high-resolution trajectory.
    t=(1.2:.0001:1.45).'; x=interp1(a.history.time,a.history.state,t,'pchip');
    if grid==3
        reference.hybrid.desiredHeadingNED=cmd.heading;
        reference.hybrid.lateralDirectionNED=cmd.lateral;
        reference.hybrid.lateralDisplacement=0;
        reference.hybrid.initialPositionNED=zeros(3,1);
        opts=odeset('RelTol',1e-10,'AbsTol',1e-12,'MaxStep',.0001);
        [t,x]=ode45(@(tt,xx) CONTROL(tt,xx,a.b.gains,a.b.gyro1,a.b.gyro2, ...
            a.b.auv,a.b.params,reference,loop,a.cfg),t,x(1,:).',opts);
    end
    maxRate=zeros(size(t)); residual=maxRate; conds=maxRate; flags=false(size(t));
    rollError=maxRate; pitchError=maxRate;
    rollMargin=maxRate; priorityPitch=maxRate; actualError=zeros(numel(t),3);
    actualRate=maxRate; actualAccel=maxRate;
    for i=1:numel(t)
        state=x(i,:).';
        [measured,g1,g2]=CMG_CONTROLLER_ESTIMATE(state,a.b.gyro1,a.b.gyro2,a.cfg);
        [tau,~,~]=CMG_ALLOCATE(t(i),measured,a.b.gains,g1,g2,reference,loop,a.cfg);
        h=[a.b.gyro1.I*state(14),a.b.gyro2.I*state(16)];
        B=-[h.*cos(state([13,15]).');h.*sin(state([13,15]).')];
        rhs=[tau.KD;tau.MD]-B*[state(12);state(12)];
        q=ROLL_FEASIBILITY_DIAGNOSTIC(B,rhs,a.cfg.limits.maxGimbalRate);
        maxRate(i)=max(abs(q.exactRates)); residual(i)=q.boundedResidual;
        conds(i)=q.condition; flags(i)=~q.legacyFeasible;
        rollError(i)=q.momentError(1); pitchError(i)=q.momentError(2);
        rollMargin(i)=q.rollMargin; priorityPitch(i)=q.rollPriorityPitchResidual;
        [~,data]=CONTROL(t(i),state,a.b.gains,a.b.gyro1,a.b.gyro2, ...
            a.b.auv,a.b.params,reference,loop,a.cfg);
        actualError(i,:)=(data.achievedMoment-data.requestedMoment).';
        actualRate(i)=max(abs(data.actuator.actualGimbalRate));
        actualAccel(i)=max(abs(data.actuator.gimbalAccel));
    end
    s.localIntegration=grid==3;
    s.sampleTime=dt(grid); s.originalFlagCount=a.result.rollInfeasibleSamples;
    s.denseFlagCount=nnz(flags); s.firstFlag=NaN; s.lastFlag=NaN;
    if any(flags), s.firstFlag=t(find(flags,1)); s.lastFlag=t(find(flags,1,'last')); end
    s.peakExactRate=max(maxRate); s.peakCondition=max(conds);
    s.maxBoundedResidual=max(residual); s.maxRollResidual=max(abs(rollError));
    s.maxPitchResidual=max(abs(pitchError));
    s.minRollMarginNm=min(rollMargin);
    s.rollOnlyInfeasibleSamples=nnz(rollMargin < -1e-10);
    s.pitchNeutralDurationS=trapz(t,double(residual>1e-7));
    s.peakRollPriorityPitchNm=max(abs(priorityPitch));
    s.rollPriorityAbsPitchImpulseNms=trapz(t,abs(priorityPitch));
    s.peakActualRollErrorNm=max(abs(actualError(:,1)));
    s.peakActualPitchErrorNm=max(abs(actualError(:,2)));
    s.actualAbsRollErrorImpulseNms=trapz(t,abs(actualError(:,1)));
    s.actualAbsPitchErrorImpulseNms=trapz(t,abs(actualError(:,2)));
    s.peakActualRate=max(actualRate); s.peakActualAccel=max(actualAccel);
    s.pitchStartDeg=rad2deg(x(1,5)); s.pitchEndDeg=rad2deg(x(end,5));
    s.maxAbsPitchDeg=max(abs(rad2deg(x(:,5))));
    if isempty(rows), rows=s; else, rows(end+1)=s; end %#ok<AGROW>
    traces{grid}=table(t,maxRate,residual,conds,flags,rollError,pitchError, ...
        rollMargin,priorityPitch,actualError,actualRate,actualAccel);
end
results=struct2table(rows); disp(results);
out=fullfile(root,'Working Results','roll_feasibility_diagnostic');
if ~isfolder(out), mkdir(out); end
save(fullfile(out,'diagnostic.mat'),'results','traces');
writetable(results,fullfile(out,'summary.csv'));
end
