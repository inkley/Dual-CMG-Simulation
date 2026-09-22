function results=ACTUATOR_UNCERTAINTY_SWEEP
root=CMG_ROOT(); cases=ACTUATOR_UNCERTAINTY_CASES();
out=fullfile(root,'Working Results','actuator_uncertainty');
if ~isfolder(out), mkdir(out); end
rows=struct([]); missions=cell(numel(cases),1);
for k=1:numel(cases)
    r=RUN_ROLL_TURN_SURGE(struct('actuatorCase',k)); missions{k}=r;
    s.name=cases(k).name; s.terminal=r.terminal; s.passes=r.passes;
    s.headingDeg=r.finalHeadingErrorDeg; s.planeDeg=r.maxSurgePlaneErrorDeg;
    s.crossTrack=r.maxCrossTrack; s.outOfPlane=r.maxOutOfPlane;
    s.finalSpeed=r.finalSpeed; s.peakForce=r.peakAftForce;
    s.gimbalDeg=r.maxGimbalDeg; s.limitSamples=sum(r.limitedSamples);
    s.rollInfeasible=r.rollInfeasibleSamples;
    if isempty(rows), rows=s; else, rows(end+1)=s; end %#ok<AGROW>
    results=struct2table(rows);
    save(fullfile(out,'sweep.mat'),'results','missions','cases');
    writetable(results,fullfile(out,'summary.csv'));
end
disp(results);
end
