function results=EXACT_INERTIA_KNOWLEDGE_COMPARISON(runCases)
% Same unequal-inertia plant with nominal vs exact allocator inertia.
% Cases 12/13 have unbiased speed sensing; no gains or criteria are changed.
if nargin<1, runCases=true; end
root=CMG_ROOT(); cases=MISSION_ESTIMATION_CASES();
if runCases
    for k=[12,13]
        RUN_ROLL_TURN_SURGE(struct('estimationCase',k,'exactRotorInertiaKnowledge',true));
    end
end
rows=struct([]);
for k=[12,13]
    folder=fullfile(root,'Working Results','roll_turn_surge', ...
        'plane_45_heading_45_dt_0.05_timeout_15',['estimation_',char(cases(k).name)]);
    nominal=load(fullfile(folder,'mission.mat'));
    exact=load(fullfile(folder,'exact_inertia','mission.mat'));
    assert(isequaln(nominal.b,exact.b),'Physical baseline must be identical.');
    cfg=exact.cfg; cfg.estimation.rotorInertia=nominal.cfg.estimation.rotorInertia;
    assert(isequaln(cfg,nominal.cfg),'Only allocator inertia knowledge may change.');
    assert(isequal(exact.cfg.estimation.rotorInertia,[exact.b.gyro1.I,exact.b.gyro2.I]));
    assert(all(exact.cfg.estimation.speedScaleBias==0));
    for knowledge=1:2
        a=nominal; if knowledge==2, a=exact; end
        s.caseIndex=k; s.exactKnowledge=knowledge==2;
        s.terminal=a.result.terminal; s.passes=a.result.passes;
        j=find(a.history.phase=="TURN",1,'last');
        s.lastTurnHeadingDeg=rad2deg(a.history.headingError(j));
        s.lastTurnPlaneDeg=rad2deg(a.history.planeError(j));
        s.lastTurnRateDeg=rad2deg(a.history.angularRate(j));
        state=a.history.state(j,:).';
        cp.alphadot1=a.history.gimbalRate(j,1); cp.alphadot2=a.history.gimbalRate(j,2);
        cp.Omegadot1=0; cp.Omegadot2=0;
        [q1,q2]=CMG(a.b.gyro1,a.b.gyro2,cp,state);
        s.lastTurnCmgPitchNm=q1.M+q2.M; s.lastTurnCmgYawNm=q1.N+q2.N;
        s.surgeStart=NaN; index=find(a.result.events.phase=="SURGE",1);
        if ~isempty(index), s.surgeStart=a.result.events.time(index); end
        s.finalHeadingDeg=a.result.finalHeadingErrorDeg;
        s.surgePlaneDeg=a.result.maxSurgePlaneErrorDeg;
        s.outOfPlane=a.result.maxOutOfPlane;
        s.limitSamples=sum(a.result.limitedSamples);
        s.rollInfeasible=a.result.rollInfeasibleSamples;
        if isempty(rows), rows=s; else, rows(end+1)=s; end %#ok<AGROW>
    end
end
results=struct2table(rows);
out=fullfile(root,'Working Results','exact_inertia_knowledge');
if ~isfolder(out), mkdir(out); end
save(fullfile(out,'comparison.mat'),'results');
writetable(results,fullfile(out,'comparison.csv')); disp(results);
end
