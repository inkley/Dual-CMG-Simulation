function results=MISSION_ESTIMATION_SWEEP
root=fileparts(mfilename('fullpath')); cases=MISSION_ESTIMATION_CASES();
out=fullfile(root,'Working Results','mission_estimation');
if ~isfolder(out), mkdir(out); end
rows=struct([]); missions=cell(numel(cases),1);
for k=1:9 % original combined-error study; ablation cases have a separate runner
    r=RUN_ROLL_TURN_SURGE(struct('estimationCase',k)); missions{k}=r;
    s.caseIndex=k; s.name=cases(k).name; s.terminal=r.terminal; s.passes=r.passes;
    s.surgeStart=NaN; index=find(r.events.phase=="SURGE",1);
    if ~isempty(index), s.surgeStart=r.events.time(index); end
    s.headingDeg=r.finalHeadingErrorDeg; s.planeDeg=r.maxSurgePlaneErrorDeg;
    s.crossTrack=r.maxCrossTrack; s.outOfPlane=r.maxOutOfPlane;
    s.finalSpeed=r.finalSpeed; s.progress=r.finalProgress;
    s.gimbalDeg=r.maxGimbalDeg; s.limitSamples=sum(r.limitedSamples);
    s.rollInfeasible=r.rollInfeasibleSamples;
    if isempty(rows), rows=s; else, rows(end+1)=s; end %#ok<AGROW>
    results=struct2table(rows);
    save(fullfile(out,'sweep.mat'),'results','missions','cases');
    writetable(results,fullfile(out,'summary.csv'));
end
disp(results);
end
