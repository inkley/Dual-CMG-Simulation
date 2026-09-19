function results=PLANE_PITCH_CORRECTION_SWEEP
root=fileparts(mfilename('fullpath')); cases=MISSION_ESTIMATION_CASES();
out=fullfile(root,'Working Results','plane_pitch_correction');
if ~isfolder(out), mkdir(out); end
rows=struct([]); missions=cell(13,1);
for k=1:13
    r=RUN_ROLL_TURN_SURGE(struct('estimationCase',k,'planePitchCorrection',true));
    missions{k}=r;
    s.caseIndex=k; s.name=cases(k).name; s.terminal=r.terminal; s.passes=r.passes;
    s.surgeStart=NaN; i=find(r.events.phase=="SURGE",1);
    if ~isempty(i), s.surgeStart=r.events.time(i); end
    s.headingDeg=r.finalHeadingErrorDeg; s.planeDeg=r.maxSurgePlaneErrorDeg;
    s.outOfPlane=r.maxOutOfPlane; s.crossTrack=r.maxCrossTrack;
    s.finalSpeed=r.finalSpeed; s.gimbalDeg=r.maxGimbalDeg;
    s.limits=sum(r.limitedSamples); s.rollInfeasible=r.rollInfeasibleSamples;
    if isempty(rows), rows=s; else, rows(end+1)=s; end %#ok<AGROW>
    results=struct2table(rows);
    save(fullfile(out,'sweep.mat'),'results','missions','cases');
    writetable(results,fullfile(out,'summary.csv'));
end
disp(results);
end
